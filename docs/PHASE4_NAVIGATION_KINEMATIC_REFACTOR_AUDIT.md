# PHASE 4 — Navigation Kinematic Refactor AUDIT（只读）

日期：2026-08-17  
基线 commit：`71c466bedb15df4049a904976cc567eae4abb6c8`（V0.11 / 3F-CORRECTIVE）  
状态：**AUDIT COMPLETE — 未改代码**  
约束：**不要进入 3G**；本阶段只输出根因 / 架构 / 计划 / 文件 / 测试。

---

## 0. 一句话结论

当前系统已经有 **Policy / Probe / Commitment / Recovery 执行闭环**，但用户感知到的「蓝带过短、贴障才绕、原地转、缺搓车」主要来自 **职责混淆 + 局部 horizon 过短 + 无真正运动学/扫掠校验**，而不是单纯「某个 steps 数字太小」。

| 用户症状 | 主根因（架构层） | 不是 |
|----------|------------------|------|
| 蓝带≈0.3m | UI 画的是 **Local Physical Trajectory**（1.5s rollout），被误当成 Global Preview | 缺 Global 路径本身 |
| 贴障才绕 | `OBSTACLE_APPROACH` / side compare 仍绑定 `front_near` 阈值 | 完全没有 Policy 状态机 |
| 原地转圈 | ALIGN/TURN 仍可抢权；`rotation_safe` 非全车扫掠 | 3F reverse 又坏了（3F 仍 PASS） |
| 不倒车搓车 | Reverse 仅 Recovery 直线 tracker；无 MANEUVER_REVERSAL / REVERSE_ARC | Safety 一律禁倒车 |
| Global 不可执行 | A*+圆足迹+转角罚分 ≠ κ / ω / swept footprint | Global 完全没有路径 |

**3F-CORRECTIVE 必须保持 PASS**：`LOCAL_REVERSE→REVERSE_ESCAPE→vx<0→pose progress`。本重构不得回退该链路。

---

## 1. CURRENT ARCHITECTURE（谁负责什么）

### 1.1 完整调用链（Web 仿真 ~20Hz）

```text
Mission / Goal (/api/nav/plan)
    ↓
GlobalPlannerModel.plan / replan          # nav_models.py
    → SimWorld.plan_path_quality          # path_quality + A*
    → (x,y) polyline · robot_r=planner_radius
    ↓
state._global_path / _planned_path
    ↓
_physics_loop (sim_api_ext.py)
    ProgressTracker
    [optional] Global replan on stuck
    ↓
LocalMppiModel.step                       # nav_models.py
    ├─ build_obstacle_snapshot + ProbeEngine   # evidence
    ├─ NavigationPolicy.decide                 # WHY / corridor half_width / profiles
    ├─ authorize_side_switch_gate              # 3E
    ├─ evaluate_recovery                       # 3F ladder
    ├─ _select_active_corridor                 # UI active PhysicalTrajectory
    ├─ ManeuverFSM.decide (+ LocalManeuverSelector.compare)
    └─ DiffDriveMppi.step(force_*, vx bounds)
    ↓
apply_safety → safe_vx / safe_w             # HARD gate
    ↓
accel limit + integrate → state.vx/w/pose
    ↓
breadcrumb.record / progress / recovery_exec
```

### 1.2 十个所有权问题（当前事实）

| # | 问题 | 当前所有者 | 缺口 |
|---|------|------------|------|
| 1 | 长期路径 | `GlobalPlannerModel` + `path_quality` | 仅 (x,y)；无 yaw/κ |
| 2 | 局部避障 | `LocalManeuverSelector` + FSM LOCAL_* | horizon **1.5s**；过依赖 capture |
| 3 | 车辆运动学 | `rollout_candidate` / MPPI integrate | Global **未**验证；走廊 ribbon ≠ DD swept |
| 4 | 全车 footprint | `_footprint_points`（9 点）+ 三套 radius | **不一致** |
| 5 | 最终 vx | Safety（导航）/ 手动 3055 旁路 | Planner 不能 bypass Safety ✓ |
| 6 | 最终 w | 同 Safety；ALIGN/TURN 由 force_w 主导 | 无限 spin guard **仅 Recovery 侧完善** |
| 7 | 允许 reverse | Policy `allow_recovery` + FSM `REVERSE_ESCAPE` + Safety rear | **无**正常 MANEUVER_REVERSAL |
| 8 | 允许 pure rotation | FSM ALIGN/TURN + `rotation_safe_at` | 非 full-body rotational sweep |
| 9 | Replan | Policy REPLAN + GlobalPlannerModel.replan | 与 kinematic reject **未衔接** |
| 10 | 「路在但车走不了」 | **缺失** `KinematicPathValidator` | A* PASS ≠ vehicle feasible |

### 1.3 三层轨迹（当前实际 vs 应然）

| 层 | 应然 | 当前实现 | UI 现状 |
|----|------|----------|---------|
| **A. Global Reference Corridor** | 4–8m preview，物理边界 | Global path 有全长；`run_web_sim._local_path(..., 5.0)` 可切 5m，但主图 **不再优先用它** | 常被 Local 蓝带盖住/替代 |
| **B. Local Physical Trajectory** | 2–4s DD poses + swept | Probe/Local/MPPI ≈ **1.5–1.6s**；`PhysicalTrajectoryCorridor` = centerline ± `0.5*W+margin` | **主图蓝带默认画这一层** → 看起来「只有几十厘米」 |
| **C. Safety Envelope** | stop / 碰撞守卫 | `front_stop_m` 等固定阈值 + circle collide | 无独立可视化层 |

**核心认识错误：** `PhysicalTrajectoryCorridor` 被当成「长距离规划蓝带」。它只是 **已有 poses 的可视化/证据带**，不是 Global Preview Planner。

### 1.4 Horizon / Preview 实测量级

| 模块 | 参数 | 时间 | @ vx=0.20 | @ vx=0.15 |
|------|------|------|-----------|-----------|
| Local candidate | `ROLLOUT_STEPS=15`, `dt=0.1` | **1.5s** | 0.30m | 0.225m |
| MPPI | `time_steps=16`, `model_dt=0.1` | **1.6s** | 0.32m | 0.24m |
| Probe short/med/long | 0.50 / 1.05 / 1.80m | 空间窗 | ≤2.2m clip | — |
| Corridor `half_width` | `0.5*0.55 + inflate ≈ 0.455m` | — | ribbon | — |
| Global `_local_path` | `horizon_m=5.0` | — | **存在但未作主图 SoT** | — |

### 1.5 Geometry 打架（当前）

```text
planner_radius = 0.25   # A* 圆
local_radius   = 0.24   # MPPI 圆代价
safety_radius  = 0.28   # 物理守卫圆
_footprint_points       # 9 离散点（Local/Probe）
corridor half_width     # 恒定侧向带（无前后 overhang）
rotation_safe_at        # 转弯半径环 8 点
```

同一辆 AMB-150（L=1.05, W=0.55）在不同层用不同碰撞模型 → 每层可各自 PASS，系统整体 FAIL。

### 1.6 Obstacle Approach（当前）

`nav_policy.py` 已有状态 `OBSTACLE_APPROACH`，但触发主因是：

```text
scene == APPROACH
OR velocity-but-path-stalled
OR front_near < front_clear + …
```

**没有：**

```text
distance_to_required_turn
curvature_preview_window
future footprint sweep on global path
```

因此行为仍接近：「`front_near` 变小 → CAUTION/AVOID → 可能 STOP → TURN」。

### 1.7 Recovery / Reverse（3F 后，必须保留）

```text
evaluate_recovery → LOCAL_REVERSE
  → ManeuverFSM 强制 REVERSE_ESCAPE
  → force_vx=-0.12, force_w≈0  (TEMPORARY_REVERSE_TRACKER)
  → signed_progress_m / stall / timeout
  → Safety → state.vx < 0 → pose back
```

缺口：仅直线；无 `REVERSE_ARC`；无「正常避障搓车」`MANEUVER_REVERSAL`。

### 1.8 职责重叠（先指出，勿堆 if）

| 重叠 | 表现 |
|------|------|
| Policy corridor half_width vs PhysicalTrajectoryCorridor | 都叫 corridor，语义不同（lateral budget vs pose ribbon） |
| Local compare vs MPPI | LOCAL_* 时 MPPI 直通 force_*，采样形同虚设 |
| Probe vs Local candidate | 都做 DD rollout，horizon/评分不一致 |
| front_stop vs Policy APPROACH vs FSM DEAD_END | 多阈值触发同一「贴障」现象 |
| Global path follow (MPPI 5·gdev) vs require_capture | Global 近似硬轨，与「可绕行」冲突 |

---

## 2. ROOT CAUSES（按优先级）

### RC-1 — 蓝带语义错位（P0 感知）

主图 `applySnap` 优先 `nav.physical_trajectory`（Probe/Local 短 poses）。  
用户期望的是 **Global Reference 4–8m**。  
→ 「蓝带过短」首先是 **产品/架构定义错误**，其次才是 horizon 数值。

### RC-2 — Local / MPPI horizon 过短（P0 行为）

1.5–1.6s × 巡航速度 ≈ 0.25–0.35m。  
无法支撑「提前 2–3m 选边成弧」。  
**禁止**仅把 steps 拉到 50 当全局规划。

### RC-3 — 无 KinematicPathValidator（P0）

Global 只有栅格圆 + 转角罚分 + LOS/smooth。  
无 path yaw、无 κ→ω、无全车 swept 沿 path 校验。  
→ 「路很漂亮但车转不过」无法在规划层拒绝。

### RC-4 — Corridor = 定宽 ribbon（P0）

`build_corridor_from_poses`：`± half_width` 垂线。  
曲线时 **不** 覆盖 front/rear overhang、外圈扩张。  
→ 内圈/外圈撞障可漏检。

### RC-5 — Obstacle Approach 非预测（P0）

无 `curvature_preview` / `distance_to_turn`。  
→ 贴 `front_stop` 前后才 LOCAL_AVOID / TURN。

### RC-6 — Rotation 检查过粗 + 非 Recovery 路径 spin 风险（P0）

`rotation_safe_at` ≠ full rotational sweep。  
3F 已堵 Recovery 空转；普通 ALIGN/TURN 仍可能长时间 `vx≈0,w≠0`。

### RC-7 — Reverse 能力面过窄（P1）

仅 Recovery 直线。  
缺：正常 MANEUVER_REVERSAL、REVERSE_ARC、`w≠0` 且 footprint 校验。

### RC-8 — Candidate 评分短视（P1）

过重 `path_capture`；缺 `future_escape_quality` / recoverability。  
→ 眼前 clearance 大但死胡同仍可能高分。

### RC-9 — 速度 / 停车距固定魔法（P1）

`max_vx=0.40`、常用 0.14–0.22；`front_stop_m=0.70` 孤立。  
无 `d_stop(v,a,latency)` 进入 Approach/Safety/Local。

### RC-10 — UI 单蓝带（P2）

未分层 GLOBAL / LOCAL / SAFETY。  
vehicle-centric local candidate 已 PASS，但不足以纠正主图误解。

---

## 3. PLANNED ARCHITECTURE（目标栈，最小侵入）

```text
Mission
  → Global Planner (topology, long-range)
  → densify + yaw_i + κ_i
  → KinematicPathValidator (footprint swept + |ω|≤max_w)
       INVALID → reject / replan  （不把坏路交给 MPPI）
  → Global Reference Corridor (adaptive 2–8m preview)
  → NavigationPolicy
       WHY + OBSTACLE_APPROACH(anticipation) + authorization
  → Local Maneuver Selector (2–4s candidates, recoverability)
  → ManeuverFSM (FORWARD / ARC / TURN / REVERSE_ESCAPE / MANEUVER_REVERSAL)
  → Controller (MPPI within action space; reverse arc when authorized)
  → Safety (final gate; dynamic_stop_distance)
  → Execution evidence (vx/w/pose/progress)
  → Recovery ladder (3F 保持) / Replan
```

### 3.1 三层轨迹（强制分离）

| ID | 名称 | Preview | 来源 | UI |
|----|------|---------|------|-----|
| A | Global Reference Corridor | adaptive **2–8m**（默认~5） | validated global poses + swept | 半透明蓝 |
| B | Local Physical Trajectory | **2–4s**（场景自适应） | DD rollout / MPPI best | 高亮 |
| C | Safety Envelope | 当前停止距 / footprint | Safety + VehicleGeometry | 独立层 |

`PhysicalTrajectoryCorridor` **升级为** A/B 的 **几何引擎**（swept union 近似），**不是**第四个规划器。

### 3.2 Adaptive Global Preview（概念）

```text
preview_m = clamp(
  f(speed, obstacle_density, curvature, free_space, goal_distance, stop_distance),
  MIN_PREVIEW_M=2.0,
  MAX_PREVIEW_M=8.0
)
# NORMAL ≈ 5.0 — 非硬编码永远 5
```

### 3.3 控制哲学（与用户 §42 对齐）

- Global：长期绕哪里  
- Kinematic Validator：物理能不能走  
- Reference Corridor：未来几米参考  
- Obstacle Approach：**不可恢复前**介入  
- Local：选哪条可行弧  
- FSM：执行态  
- Controller：本周期 vx/w  
- Safety：现在允不允许  
- Recovery：失败退出（3F 保留）  
- Execution：**必须有 pose/vx 证据**

---

## 4. FILES TO CHANGE（分阶段，勿一次推倒）

### P0

| 文件 | 改动意图 |
|------|----------|
| `nav_geometry.py` | 统一 preview/stop/horizon/sweep 步长常量；注释 physical/planner/local/safety |
| **新建** `nav_kinematic.py`（名可调） | `KinematicPathValidator`；path yaw/κ；required_w；swept sample API |
| `local_maneuver.py` | 升级 footprint/swept；延长 candidate horizon（自适应）；评分加 recoverability |
| `nav_trajectory.py` | ribbon → pose×footprint⊕margin 密采样 swept |
| `nav_policy.py` | Approach anticipation：`distance_to_turn` / curvature window；勿仅 front_near |
| `maneuver.py` | `ROTATION_EXECUTION_GUARD`；保留 3F REVERSE_ESCAPE；可选 MANEUVER_REVERSAL 入口 |
| `sim_api_ext.py` / `run_web_sim.py` | 分离 global/local/safety 进 snapshot |
| `sim3d.js` / `sim_main.html` | 三层可视化 |

### P1

| 文件 | 改动意图 |
|------|----------|
| `mppi_controller.py` | local horizon 2–4s（场景档）；勿无脑 8s×大批 batch |
| `nav_recovery.py` / FSM | 正常搓车 vs Recovery 语义拆分；REVERSE_ARC |
| `path_quality.py` / `sim_world.py` | densify yaw；validator 挂钩 reject |
| `nav_models.py` | 接线 validator / preview / telemetry |
| `nav_probe.py` | 与新 horizon/swept 对齐（保持 3D evidence 语义） |
| `nav_debug.py` / `nav_phase4_telemetry.py` | §27 字段 |

### P2

| 文件 | 改动意图 |
|------|----------|
| UI / docs / audits | 分层显示与报告 |

### 明确不改 / 保护

- 3C Commitment、3E Side-switch authorization 闸门  
- 3F `POLICY_RECOVERY|LOCAL_REVERSE` → `REVERSE_ESCAPE` + progress/timeout  
- Safety 最终权威（禁止 planner bypass）

---

## 5. TEST PLAN

### 5.1 必须保留（回归）

```text
scripts/_audit_phase4_commitment.py
scripts/_audit_phase4_probe.py
scripts/_audit_phase4_side_switch.py
scripts/_audit_phase4_recovery_execution.py
scripts/_trace_phase4_recovery_corrective_live.py  # S1/S2/S6
```

验收：**3F-CORRECTIVE 仍 PASS**（含 `state.vx<0` + pose back）。

### 5.2 新建（本任务）

| Script | 覆盖 |
|--------|------|
| `_audit_global_preview.py` | open≥4m；受限自适应缩短 |
| `_audit_kinematic_path.py` | yaw/κ/ω；invalid hard corner |
| `_audit_swept_footprint.py` | 内/外圈：centerline clear 但 footprint hit |
| `_audit_obstacle_approach.py` | Approach **早于** front_stop |
| `_audit_rotation_guard.py` | full-body rotation；bounded spin → REPLAN/REVERSE/STOP |
| `_audit_reverse_arc.py` | `vx<0` 且 `w≠0` 安全弧 |
| `_trace_navigation_escape_live.py` | Scene A–F LIVE 物理证据 |

### 5.3 LIVE Scenes（用户 §29）

| Scene | 期望 | 证据字段 |
|-------|------|----------|
| A 空旷 | Global preview ≥4m | `global_preview_m` |
| B 前方转弯 | Approach 早于贴障；减速+弧 | `OBSTACLE_APPROACH`, `dist_to_turn`, pose |
| C 左绕 | LOCAL_LEFT arc，非 STOP→TURN | mode, vx≠0 |
| D 右绕 | 提前 RIGHT | 同上 |
| E 三面堵 | 3F S1 | requested/safe/state vx, signed_progress |
| F 四面堵 | SAFE_STOP/REPLAN, spin=0 | spin_ticks |

### 5.4 验收禁令（再次钉死）

禁止：`action=LOCAL_REVERSE` 单独 PASS；`global_path!=[]` 单独 PASS；`corridor exists` 单独 PASS；只改 UI 拉长蓝带；只改 steps；删 TURN；一律 reverse/stop/replan；放宽 collision 骗测；改 audit 标准。

---

## 6. 分阶段实施顺序（下一步才允许改码）

```text
AUDIT (本文件) ✅
    ↓
P0-A  VehicleGeometry + swept footprint API + telemetry schema
    ↓  tests: _audit_swept_footprint
P0-B  Global preview corridor (adaptive) + UI layer A
    ↓  tests: _audit_global_preview
P0-C  KinematicPathValidator on densified path
    ↓  tests: _audit_kinematic_path
P0-D  Obstacle Approach anticipation
    ↓  tests: _audit_obstacle_approach
P0-E  Rotation guard (full-body + bound)
    ↓  tests: _audit_rotation_guard
    ↓  3F regression
P1    Local horizon 2–4s + reverse arc + recoverability + dynamic speed
P2    UI polish / docs REPORT
```

**任一阶段破坏 3F S1/S2/S6 → 停止前进。**

---

## 7. 对用户八项症状的预判（审计级，非最终 PASS）

| # | 问题 | 审计判定 | 证据方向 |
|---|------|----------|----------|
| 1 | 蓝带过短 | **CONFIRMED** — Local SoT + 1.5s horizon | 主图绑 `physical_trajectory.length` |
| 2 | Global 无车辆运动学 | **CONFIRMED** — 无 validator | path 仅 xy |
| 3 | 贴障才绕 | **CONFIRMED** — Approach≈front_near | policy 触发条件 |
| 4 | 原地转圈 | **PARTIAL** — Recovery 已抑；普通 TURN 仍弱 | rotation_safe_at |
| 5 | 缺倒车搓车 | **CONFIRMED** — 仅直线 Recovery | force_w≈0 |
| 6 | 内外圈 swept | **CONFIRMED** — ribbon + 9 点 | nav_trajectory / footprint |
| 7 | 长距离 preview 缺失 | **CONFIRMED** — 5m helper 未作主 SoT | run_web_sim._local_path |
| 8 | Geometry 不一致 | **CONFIRMED** — 三 radius + 点 + 带 | nav_geometry |

最终是否「真正解决」→ 写入后续  
`docs/PHASE4_NAVIGATION_KINEMATIC_REFACTOR_REPORT.md`  
并标 **PASS / PARTIAL / FAIL / NOT TESTED** + LIVE 证据。

---

## 8. 本阶段交付物

| 项 | 状态 |
|----|------|
| 完整调用链 | ✅ 本文 §1 |
| 十问所有权 | ✅ §1.2 |
| 职责重叠 | ✅ §1.8 |
| ROOT CAUSES | ✅ §2 |
| PLANNED ARCHITECTURE | ✅ §3 |
| FILES TO CHANGE | ✅ §4 |
| TEST PLAN | ✅ §5 |
| 代码修改 | ❌ **未开始**（按用户要求） |
| 进入 3G | ❌ **禁止** |

---

## 9. 下一步（等待确认）

用户确认本审计后，从 **P0-A（统一 footprint / swept API）** 开始改码，并先落：

```text
docs/PHASE4_NAVIGATION_KINEMATIC_REFACTOR_AUDIT.md   ← 本文件
（随后）scripts/_audit_swept_footprint.py
```

在未确认前：**不修改行为代码。**
