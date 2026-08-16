# PHASE 4 STEP 3A — Architecture Review (Read-Only)

日期：2026-08-16  
状态：**DESIGN GATE — 未改决策逻辑**  
范围：在 STEP1/STEP2 证据与现码之上，锁定 STEP3 实现边界、复用点、风险与分阶段计划。

依据：

- `docs/CURRENT_CONTROL_OWNERSHIP_MAP.md`（部分过时，见 §1）
- `docs/NAVIGATION_POLICY_ARCHITECTURE_REPORT.md`
- `docs/PHASE4_STEP1_SIDE_SWITCH_AUDIT.md`
- `docs/PHASE4_STEP2_REPRODUCTION.md` + `docs/_phase4_trace/mid_inject_*.jsonl`
- 代码：`nav_policy.py` / `maneuver.py` / `local_maneuver.py` / `nav_models.py` / `nav_geometry.py` / `sim_api_ext.py` / `nav_debug.py` / `nav_ui.js`

---

## 0. Executive Verdict

| 问题 | 结论 |
|------|------|
| 是否推翻现有栈？ | **否**。保留 Policy → FSM → Selector → MPPI → Safety |
| 根因是否已钉死？ | **是**（STEP2 LIVE）：无 Avoidance Commitment；`allow_side_compare=True` 时 `_select` 可 `RIGHT_ONLY`/`LOWER_TOTAL_COST` 换边 |
| STEP3 目标？ | Commitment + Probe(evidence) + Side-switch **authorization** + Safety feedback + Recovery/Replan 闭环 |
| 本阶段改码？ | **否**（3A 只读） |

Git 工作区：**脏**（大量既有 Phase 改动 + untracked Phase4 docs/scripts）。**禁止** `reset --hard`；后续只追加/局部编辑，不覆盖用户未提交内容。

---

## 1. Ownership 现状（代码事实，相对 MAP 的修正）

旧 MAP 写「缺 NavigationPolicy」——**已过时**。当前 `LocalMppiModel` 已接线：

```text
nav_models.LocalMppiModel.step
  → NavigationPolicy.step(...)          # WHY
  → policy_ctx = {allow_side_compare, allow_replan, allow_recovery, ...}
  → ManeuverFSM.decide(policy_ctx=...)  # HOW
       → LocalManeuverSelector.compare  # candidate eval（仍有换边权）
  → DiffDriveMppi.step(force_*)
  → apply_safety(...)                   # HARD
```

| 层 | 文件 | 实际权威 | STEP3 目标权威 |
|----|------|----------|----------------|
| Policy | `nav_policy.py` | behavior/state/corridor/profile；**仍把 `allow_side_compare=True` 在 LOCAL_*** | + Commitment + **side_switch_authorized** + failure class |
| Selector | `local_maneuver.py` | **现行换边写入者**（`_select`） | 降为候选评估；换边需 Policy 授权 |
| FSM | `maneuver.py` | mode 提交 + force | 只执行 **已授权** side |
| Probe | **不存在** | — | `nav_probe.py` evidence only |
| MPPI | `mppi_controller.py` | LOCAL_* 短路执行 force | 不变 |
| Safety | `sim_api_ext.apply_safety` | 最终 vx/w | 不变；增加 **persistent reject → Policy** 反馈 |
| Geometry | `nav_geometry.py` + `local_maneuver._footprint_points` | L≈1.05 W≈0.55；圆足迹局部碰撞 | Probe **必须复用**，禁第三套 |

---

## 2. STEP2 钉死的因果（实现约束）

LIVE `mid_inject_1786888402.jsonl`：

```text
空场导航 → Stage1 → LOCAL_LEFT (allow=True)
  → Stage2 左硬堵 → _select RIGHT_ONLY → LOCAL_RIGHT
  → mppi_w 符号翻转 → FRONT_OBSTACLE / safe_vx=0 → REPOSITION 失败环
```

同时确认：

1. **微差翻边**（双方可行 + RIGHT slightly better）在 soft LIVE **未出现**（hysteresis 挡住），但代码路径 `LOWER_TOTAL_COST` **仍存在** → Commitment 必须关掉「无授权重投票」。
2. **硬失效切边**是合法需求 → 禁止「永远锁 LEFT」；必须走 REVALIDATE + authorize。
3. **Policy 标签滞后**：FSM=`LOCAL_RIGHT` 时 `policy_behavior` 仍可短暂 `AVOID_LEFT` → telemetry 必须拆 `intent / committed / fsm / selector / controller`。
4. **Safety 允许贴障纯转**：`front_near < front_stop` 且 `turning` 且 `|vx|<0.04` → `return 0, w`（`sim_api_ext.apply_safety`）——解释 STEP2「停住仍 yaw 转」。STEP3 需 **旋转 footprint Probe**，**不可**凭感觉直接禁 w。

---

## 3. 现有可复用资产（禁止重写）

| 资产 | 位置 | Probe/Commit 用法 |
|------|------|-------------------|
| `rollout_candidate` / `_collide_body` / `_footprint_points` | `local_maneuver.py` | L/R/F 预测碰撞与 clearance |
| `sector_free` / `rotation_safe_at` | `maneuver.py` | 侧向自由空间；旋转安全初筛 |
| `DEFAULT_GEOM` (length/width/stop/clear) | `nav_geometry.py` | ProbeConfig 唯一几何源 |
| `MIN_AVOID_HOLD_S` / `avoid_side` | `nav_policy.py` | **不够**；升级为 Commitment，不删旧字段（兼容 telemetry） |
| `oscillation_loop` / `replan_loop` / `recovery_loop` | `nav_policy.py` | 扩展 **side_switch** 窗口计数 |
| `planned_rejected_by_safety` / `safety_zero` | `nav_models` → Policy | 升级为 persistent safety reject |
| `obstacle_passed` | selector + `policy.mark_obstacle_passed` | Commitment RELEASE 条件 |
| Debug hub / `nav_ui.js` 组件系统 | 已有 Policy/Local cards | 新增 PROBE / COMMIT / SIDE SWITCH 组件 |

**缺口：**

- 无 Backward **轨迹** Probe（仅有 `rear_near` 阈值 + `LAST_RESORT_REVERSE`）
- 无统一 `ObstacleSnapshot`（F/L/R 可能跨 tick）
- 无 `side_switch_authorized`
- Selector `_select` 不读 failure type / probe evidence

---

## 4. 目标职责边界（锁定）

```text
NavigationPolicy
  WHY + Commitment lifecycle
  side_switch_authorized (唯一换边授权)
  failure classification (HARD vs SOFT)
  allow_side_compare := (no active commit) OR (revalidation window)
  allow_recovery / allow_replan (门控不变语义，加 Probe 证据)

ProbeEngine (new nav_probe.py)
  Evidence only: F/B/L/R (+ optional TURN_IN_PLACE sweep)
  复用 footprint/rollout；不写 vx/w；不提交 mode

LocalManeuverSelector
  Candidate scores；在 commit 下：
    - 默认 KEEP committed side
    - 仅当 policy 授权 REVALIDATE 才比较对侧
  禁止仅因 LOWER_TOTAL_COST 在 commit 中换边

ManeuverFSM
  执行已授权 mode；不自行发明换边

MPPI / Safety
  执行与硬门；Safety 永不被 Commitment 覆盖
```

### 禁止的错误实现（验收红线）

| ID | 错误 | 替代 |
|----|------|------|
| A | `if LOCAL_LEFT: return LEFT` | Commit + HARD failure → revalidate |
| B | `MIN_HOLD=5s` | Commit + Probe persistence |
| C | 每帧 Probe 后重选边 | Probe=evidence；选边需 authorize |
| D | Probe 写 vx/w | 禁止 |
| E | 任意失败 → reverse | Backward Probe 先证伪 |

---

## 5. AvoidanceCommitment 设计（3C 将实现）

### 5.1 状态（语义，非第三套 FSM 打架）

Policy 内嵌 commitment 子状态即可（**不是**独立 ProbeFSM）：

```text
NONE
  → SELECTING          # 首次两侧 Probe + selector
  → COMMITTED_LEFT | COMMITTED_RIGHT
  → EVALUATING         # soft degradation / probe soft risk（保持边）
  → PASSING            # obstacle_passed 临近
  → RELEASED           # → PATH_RECAPTURE / FOLLOW

COMMITTED_*
  → FAILED_HARD | FAILED_PERSISTENT
  → REVALIDATING       # 短窗口：同 snapshot 验 F/L/R/B
  → SWITCH_AUTHORIZED  # 一次性授权对侧
  → COMMITTED_(other)
  → NO_ESCAPE          # → WAIT / REPLAN / SAFE_STOP / RECOVERY(若 B valid)
```

### 5.2 字段（建议）

```text
active, side, phase,
started_at, start_pose,
obstacle_signature (粗：front_near bucket + left/right free + obstacle count/hash),
failure_reason (SideFailureReason),
soft_fail_streak / soft_fail_since,
hard_fail (bool),
switch_count_window, last_switch_at,
release_reason
```

### 5.3 核心规则

| 条件 | 行为 |
|------|------|
| `COMMITTED` + 无 HARD/PERSISTENT fail | `allow_side_compare=False`（或 True 但 selector 强制 KEEP） |
| RIGHT score better, LEFT still feasible | **KEEP**（SOFT） |
| LEFT hard collision / infeasible persistent | `REVALIDATING` |
| REVALIDATE: other side valid + authorize | 一次 `SIDE_SWITCH_AUTHORIZED` → 新 commit |
| `OBSTACLE_PASSED` / recapture | RELEASE |
| Safety emergency / collision | Safety 优先；可强制破 commit |

### 5.4 SideFailureReason（最小集）

```text
NONE
COLLISION_PREDICTED | HARD_INFEASIBLE
PERSISTENT_SAFETY_REJECT
CORRIDOR_BLOCKED | DEAD_END | NO_PROGRESS
OBSTACLE_TOPOLOGY_CHANGED | LOCAL_GEOMETRY_INVALID
PROBE_FAILED
TEMPORARY_COST_WORSE   # 明确：不得触发换边
```

---

## 6. ProbeEngine 设计（3D）

### 6.1 文件

新建 `agv_bridge/nav_probe.py`：

- `ProbeConfig`（集中参数，单位/注释）
- `ObstacleSnapshot`（pose + collide + clearance_at + front/rear + stamp）
- `ProbeResult` / `ProbeBundle`（F/B/L/R 同 snapshot）
- `ProbeEngine.evaluate(snapshot) -> ProbeBundle`

### 6.2 复用策略

| 方向 | 实现要点 |
|------|----------|
| FORWARD | 短 rollout（vx≥0,w≈0）+ stopping_margin vs front_near |
| LEFT/RIGHT | **调用/包装** `rollout_candidate(DEC_LEFT/RIGHT)`；valid≈feasible∧¬hard_collision；附 capture/progress |
| BACKWARD | 负 vx footprint rollout + rear_stop + escape_distance；**禁止**仅 `rear_near>thr` |
| TURN_IN_PLACE | yaw 扫描 `_collide_body`（补 STEP2 贴障纯转风险） |

### 6.3 Horizon（初值，实现时用量测校准，禁止拍脑袋×10）

基于 `DEFAULT_GEOM` 与巡航 ~0.15–0.22 m/s：

| 档 | 建议距离 | 用途 |
|----|----------|------|
| SHORT | 0.4–0.6 m | 硬碰撞 / Safety 衔接 |
| MEDIUM | 0.9–1.2 m | 侧向绕行 |
| LONG | 1.6–2.2 m | 死胡同 / escape |

`horizon = clip(base + k*|vx|, min, max)`。

### 6.4 valid 语义

区分：`VALID / INVALID / UNKNOWN / STALE`  
`UNKNOWN≠FREE`；长时间 STALE → WAIT/SAFE_STOP（诊断）。

### 6.5 Cache

`input_signature` + TTL（动态障碍短 TTL）；异常 → UNKNOWN，不崩环。

---

## 7. Side Switch Authorization（3E）

唯一合法路径：

```text
COMMITTED_LEFT
  → failure HARD|PERSISTENT
  → REVALIDATING (same ObstacleSnapshot)
  → candidate RIGHT Probe VALID
  → side_switch_authorized=True
       reason=CURRENT_SIDE_HARD_FAILED
       evidence=...
  → FSM LOCAL_RIGHT
  → COMMITTED_RIGHT
```

**非法（STEP2 现状）：**

```text
LOCAL_LEFT → selector RIGHT_ONLY → LOCAL_RIGHT
```

Selector 在 commit 下看到对侧更优但未授权 → `HYSTERESIS_KEEP` / `COMMIT_KEEP`（新 reason）。

Oscillation：`MAX_SIDE_SWITCHES` / `SIDE_SWITCH_WINDOW_S` → `OSCILLATION_LOOP`（已有 w 翻转 guard，需 **side** 专用）。

---

## 8. Safety Feedback / Recovery / Replan（3F–3G）

### 8.1 Safety（已证实行为）

```text
front blocked + turning + |vx|<0.04 → (0, w)   # 允许贴障转
front blocked + vx>0 → (0, w*0.2) + FRONT_OBSTACLE
```

反馈升级：

```text
commanded_vx significant && safe_vx≈0 for SAFETY_REJECT_CONFIRM_S
  → flags.persistent_safety_reject
  → Policy: EVALUATING / REVALIDATING / WAIT / RECOVERY（不直接换边）
```

### 8.2 Recovery 顺序（原则，非机械写死）

```text
ProbeBundle:
  当前侧 / 对侧 / Forward / Backward
→ 能安全继续 committed > 授权换侧 > forward recapture
→ backward valid 才 REPOSITION/REVERSE_ESCAPE
→ 全 INVALID → WAIT / REPLAN / SAFE_STOP
```

衔接现有：`allow_recovery`、`MAX_RECOVERY_ATTEMPTS`、`RECOVERY_LOOP→SAFE_STOP`。

### 8.3 Replan

`REPLAN_LOOP` 已有；Commitment 下 **不因** 普通 global refresh 取消绕行；Goal/map 失效才强制解除。

---

## 9. Telemetry / UI（3B 先做可观测，3H 完善）

### 9.1 Debug 必答 10 问（字段映射）

| # | 问题 | 字段 |
|---|------|------|
| 1 | 为何绕 | `policy_state` / `scene` / `reason` |
| 2 | 绕哪边 | `committed_side` / `fsm_side` |
| 3 | 是否 commit | `commitment_active` / `commitment_phase` |
| 4 | 为何不切边 | `side_switch_authorized=false` + `side_switch_reason` |
| 5 | 当前边失效？ | `current_side_failure` |
| 6 | Probe 哪边可走 | `probe.{forward,left,right,backward}` |
| 7–8 | 前/后安全 | forward/backward Probe + front/rear_near |
| 9 | Safety 拒？ | `persistent_safety_reject` / `stop_reason` |
| 10 | 下一步 | `policy.next_hint` 或 `commitment_phase` 出口 |

### 9.2 UI 组件（`+组件`，半透明可拖）

`NAV PROBE` / `AVOIDANCE COMMIT` / `SIDE SWITCH`（跟现有 Policy/Local 同系统）。

---

## 10. 目标状态机（验收用）

### 正常

```text
FOLLOW_GLOBAL
  → OBSTACLE_APPROACH / CAUTION
  → SIDE_SELECT (Probe F/L/R)
  → COMMITTED_LEFT|RIGHT
  → EVALUATING/PASSING (Probe soft ok)
  → OBSTACLE_PASSED
  → PATH_RECAPTURE
  → FOLLOW_GLOBAL (RELEASE)
```

### 异常换边

```text
COMMITTED_LEFT
  → FAILED_HARD|PERSISTENT
  → REVALIDATING
  → SWITCH_AUTHORIZED(RIGHT)
  → COMMITTED_RIGHT
```

### 无路

```text
REVALIDATING
  → F/L/R/B all INVALID
  → WAIT | REPLAN | SAFE_STOP
  （仅 B VALID → RECOVERY）
```

---

## 11. 测试与 LIVE 计划（分阶段，禁假 PASS）

| 阶段 | 必测 | LIVE |
|------|------|------|
| 3B Telemetry | 字段存在且不改决策 | 19999 抽样可见 |
| 3C Commit | C1–C7（微差不切；硬失效可进 REVALIDATE） | mid_inject soft 类：KEEP_LEFT |
| 3D Probe | P1–P11 unit | — |
| 3E Authorize | mid_inject 硬：AUTHORIZED 后切 RIGHT | **必须**有 `side_switch_authorized` 事件，禁裸 `RIGHT_ONLY` |
| 3F Recovery | R1–R7 | 后空/后堵各一 |
| 3G Safety FB | S1–S5 | persistent reject 可见 |
| 3I | 原 A 矩阵 + mid_inject Cases A–J | 真实 pose 运动 + JSONL |

方法：**先导航再中途投放**（STEP2 教训）。

---

## 12. 参数集中位置（3C+ 引入，暂不定死数值）

建议单文件常量块（`nav_policy.py` 或 `nav_probe.py` 顶部 `ProbeConfig` + `CommitConfig`）：

```text
PROBE_*_HORIZON_M, PROBE_MIN_CLEARANCE_M, PROBE_SAFETY_MARGIN_M
COMMIT_MIN_HOLD_S, COMMIT_SOFT_FAIL_CONFIRM_S, COMMIT_HARD_IMMEDIATE
SIDE_SWITCH_COOLDOWN_S, MAX_SIDE_SWITCHES, SIDE_SWITCH_WINDOW_S
SAFETY_REJECT_CONFIRM_S
RECOVERY_MAX_ATTEMPTS / REPLAN_MAX_ATTEMPTS（已有则复用）
```

改参必须写「依据 / 场景影响」；禁止为 PASS 放大阈值。

---

## 13. 文件变更预告（后续阶段，本阶段未改）

| 文件 | 计划动作 | 风险 |
|------|----------|------|
| `nav_policy.py` | Commitment + authorize + failure | 高：行为权威 |
| **new** `nav_probe.py` | ProbeEngine | 中：CPU |
| `local_maneuver.py` | 尊重 commit/authorize；新 reason | 高 |
| `maneuver.py` | 换边仅当授权；policy 同步 | 中 |
| `nav_models.py` | 同 snapshot 调 Probe；接线 | 中 |
| `nav_debug.py` / `sim_api_ext.py` | telemetry | 低 |
| `nav_ui.js` / `component_system.js` | 三组件 | 低 |
| `scripts/_audit_phase4_*.py` | 测试 | — |
| `docs/PHASE4_STEP3_IMPLEMENTATION_REPORT.md` | 终报 | — |

**不改：** Safety 硬阈值拍脑袋、全局 A* 算法、STEP1/2 历史报告内容。

---

## 14. 性能预算

20Hz physics。Probe 四向应：

- 复用 collide/clearance 闭包
- 侧向尽量一次 rollout 结果共享 Selector
- 记录 `probe_ms` / cycle P50/P95 进 debug.performance

目标：不明显拖垮 UI；超限则降 LONG horizon 或升 TTL（有说明）。

---

## 15. 3A 出口条件（完成定义）

- [x] 读完 STEP1/2 与 ownership
- [x] 确认现码权威与缺口
- [x] 锁定 Commitment / Probe / Authorize 边界
- [x] 锁定复用几何、禁止项、分阶段顺序
- [x] 本文档落盘
- [ ] **未开始改决策代码**（本阶段遵守）

---

## 16. 下一阶段

```text
STEP 3B — Telemetry First
```

仅增加只读/旁路字段与事件钩子（默认值 `NONE` / `false`），**先不改变** `allow_side_compare` 与 `_select` 结果；确认 `/api/nav/debug` 可见后再进入 **3C Commitment**。

---

## 17. 待实现时必须回答的预填（证据待 LIVE）

| # | 问题 | 3A 设计答案（待代码+LIVE 钉死） |
|---|------|--------------------------------|
| 1 | 为何无故 LEFT→RIGHT 不再发生 | Commit 下无授权禁止 score/RIGHT_ONLY 直切 |
| 2 | 真失效为何仍可切 | HARD fail → REVALIDATE → authorize |
| 3 | 换边权威 | **NavigationPolicy.side_switch_authorized** |
| 4–7 | Probe 预测什么 | 同 snapshot 未来 horizon footprint 可行性 |
| 8–9 | 噪声/动态 | SOFT persistence；dynamic_short→WAIT 不切边 |
| 10 | Safety 反馈 | persistent_reject flag → Policy |
| 11–12 | reverse | 仅 Backward Probe VALID |
| 13–15 | 无限 recovery/replan/osc | 既有 loop + side_switch guard |
| 16–17 | Global path / recapture | REFERENCE；passed+corridor+forward probe |
| 18 | 旋转碰撞 | TURN Probe / footprint sweep |
| 19–20 | LIVE / STEP2 场景 | 3E/3I 用 mid_inject 重跑 |

**3A STATUS: COMPLETE (DESIGN ONLY)**
