# PHASE 4 STEP 3F — Physical Trajectory Corridor + Recovery + Replan + Breadcrumb

日期：2026-08-16  
状态：**PASS**  
`BEHAVIOR CHANGED: **YES** (Probe-gated recovery ladder; reverse only with BACKWARD VALID)`  
`3G PREREQUISITES: **PASS** (corridor + breadcrumb + recovery evidence online; no auto next phase)`

---

## 0. Verdict / Acceptance

| Check | Result |
|-------|--------|
| 蓝色物理轨迹带（footprint⊕margin） | **PASS** |
| 差速运动学 poses（来自 rollout） | **PASS** |
| 与 Probe **同一数据源**（无第二套 rollout） | **PASS** |
| Forward / Backward / Left / Right / Turn poses+corridor | **PASS** |
| Breadcrumb（距离采样 / trusted） | **PASS** |
| Historical Retreat 逐段重验证 + 部分回撤 | **PASS** |
| Local reverse 仅在 F/L/R 失效且 B VALID | **PASS** |
| 侧向仍 VALID 时不盲倒车 | **PASS** (LIVE S6) |
| Dead End / Deadlock 分类 | **PASS** (ladder) |
| Recovery 不绕过 Safety / 不继承旧 Commitment | **PASS** (release on recovery) |
| Replan loop 复用既有 guard | **PASS** |
| 20Hz 可接受（probe ~2ms offline；无新增完整 rollout） | **PASS** |
| LIVE 19999 S1/S2/S6 | **PASS** |
| 3C/3D/3E regression | **PASS** |

---

## 1. Read-only audit（先于改码）

| Exist → REUSE | Gap → NEW |
|---------------|-----------|
| `rollout_candidate` / `_footprint_points` / `_collide_body` / `DEFAULT_GEOM` | `PhysicalTrajectoryCorridor` |
| `ProbeEngine`（原只留标量） | Probe 附带 **poses + corridor** |
| `REVERSE_ESCAPE` / `allow_recovery` / `MAX_RECOVERY_ATTEMPTS` / `recovery_loop` / `replan_loop` | `evaluate_recovery` 优先级梯子 |
| `pose_trace`（仅观测） | `TrajectoryBreadcrumb` + Historical Retreat |
| `setGuideBand`（旧：中心线假宽度） | 用 footprint half-width + status 着色 |

**禁止**：为蓝带再积分一套轨迹；`/obstacles/clear` 作为中途测试手段。

---

## 2. Architecture

```text
rollout_candidate / reverse integrate
        ↓ poses (with yaw)
ProbeResult.poses + .corridor
        ↓
PhysicalTrajectoryCorridor (swept edges)
        ↓ UI Blue Band
Breadcrumb.record(executed pose)
        ↓
HistoricalRetreat (segment revalidate)
        ↓
evaluate_recovery → Policy allow_recovery / release commitment
        ↓
Maneuver / MPPI force_reverse (gated) → Safety → Execution
```

---

## 3. Files

### ADDED

| File | Role |
|------|------|
| `agv_bridge/nav_trajectory.py` | `TrajectorySample` / `PhysicalTrajectoryCorridor` |
| `agv_bridge/nav_breadcrumb.py` | 距离采样历史 |
| `agv_bridge/nav_recovery.py` | DeadEnd/Deadlock + ladder + retreat probe |
| `scripts/_audit_phase4_recovery.py` | Offline C/B/R matrix |
| `scripts/_trace_phase4_recovery_live.py` | LIVE S1/S2/S6 |
| `docs/PHASE4_STEP3F_RECOVERY_REPLAN_TRAJECTORY.md` | 本报告 |

### MODIFIED

| File | Why |
|------|-----|
| `local_maneuver.py` | path 点带 `yaw` |
| `nav_probe.py` | `_attach_poses_corridor`；reverse 返回 path |
| `nav_models.py` | breadcrumb + recovery + reverse gate + active corridor |
| `sim_api_ext.py` | 透传 recovery / corridor / breadcrumb |
| `nav_debug.py` | phase4 aliases；旧 recovery → `execution_recovery` |
| `nav_phase4_telemetry.py` | schema `phase4_step3f_v1` |
| `sim3d.js` / `sim_main.html` | 蓝带 = footprint⊕margin + status 色 |
| `nav_ui.js` | **PHYSICAL TRAJECTORY** 组件 |

---

## 4. Physical Trajectory Corridor

定义：

> 基于 AGV base pose、差速 `vx/w/dt` 积分位姿、`DEFAULT_GEOM` footprint，再 ⊕ (safety+localization+control) margin 的 **swept 物理走廊**。

- `half_width_m ≈ 0.5·W + 0.18 ≈ 0.455 m`（**不是**裸车宽，也不是 UI 随便 offset）
- `left_edge` / `right_edge` / `swept_polygon` / `centerline` / `poses`
- 状态色：VALID→绿 / soft→黄 / INVALID→红 / UNKNOWN|STALE→灰 / 默认蓝候选

---

## 5. ONE SOURCE OF TRUTH

Probe 方向评估结束后，**同一** `cand.path` / reverse `path` → poses → corridor。  
UI **不**自算碰撞；只 render backend 字段。

---

## 6. Breadcrumb

| 参数 | 值 |
|------|-----|
| sample_dist | 0.08 m |
| max_distance | 25 m |
| max_age | 180 s |
| max_points | 400 |
| trusted | 非 emergency / 非 collision |

历史路径 **不能**直接当安全路径 → 必须 `evaluate_historical_retreat` 逐段 footprint 复检。

---

## 7. Recovery ladder（Policy 证据）

```text
1 FORWARD VALID → CONTINUE
2 side_switch_authorized → AUTHORIZED_SIDE_SWITCH
3 LEFT/RIGHT VALID → CONTINUE_LOCAL（禁止立刻 reverse）
4 dynamic_short → WAIT
5 F/L/R INVALID + BACKWARD VALID → LOCAL_REVERSE（release commitment）
6 else Historical Retreat VALID → HISTORICAL_RETREAT
7 else REPLAN（复用 note_replan / replan_loop）
8 else SAFE_STOP
```

Deadlock：几何仍可能 VALID，但 `safety_zero` + stuck → 分类 `DEADLOCK`。  
Dead End：F/L/R INVALID（± progress_low）。

---

## 8. Safety

- Recovery **不** bypass Safety
- `force_reverse` 仅当 BACKWARD Probe VALID 且 ladder 允许
- 进入 recovery → `release_commitment`（禁止继承旧 side）

---

## 9. Q1–Q20

| # | Answer |
|---|--------|
| 1 | 蓝带 = 车体真实扫掠走廊（非 Global Path / 装饰） |
| 2 | footprint corners + inflate margins along poses |
| 3 | 是 — `rollout_candidate` / reverse 差速积分 |
| 4 | 是 — Probe poses → corridor → UI |
| 5 | Selector 选边；走廊是证据可视化，不替代授权 |
| 6 | Forward rollout path → corridor |
| 7 | Reverse integrate path → corridor |
| 8 | 定点 yaw sweep samples → corridor |
| 9 | 必须 ⊕ safety/loc/control margin |
| 10 | Breadcrumb 按距离采样 executed pose |
| 11 | 环境可变 → 必须重 Probe |
| 12 | 逐段 `_collide_body`；支持 partial |
| 13 | 先左右 / 授权换边，再 reverse |
| 14 | F/L/R INVALID + BACKWARD VALID + attempts |
| 15 | 无局部 escape + breadcrumb 段验证通过 |
| 16 | pose 变了，旧 evidence 失效 |
| 17 | 旧 side 会再次陷入死胡同 |
| 18 | 仅 path/topology 失效或无局部 escape；有 `replan_loop` |
| 19 | status 来自 Probe；INVALID 不画成可走绿/蓝 |
| 20 | 同一 geom/kinematics 可迁实车；UI 仅显示 |

---

## 10. LIVE（19999）

| Scene | Trace | Expect | Result |
|-------|-------|--------|--------|
| S1 三面堵、后开 | `step3f_scene1_1786893678.jsonl` | `LOCAL_REVERSE` | **PASS** |
| S2 四面笼 | `step3f_scene2_1786893697.jsonl` | REPLAN/SAFE_STOP，无 reverse | **PASS** |
| S6 前+左堵 | `step3f_scene6_1786893713.jsonl` | 侧 VALID 时不非法 reverse | **PASS** |

注：S1 中 `mode_reverse` 可能仍为 REPOSITION/TURN（Safety/FSM 执行层）；验收以 **recovery.action=LOCAL_REVERSE** 为准（梯子正确），强制倒车仍受 Safety 门控。

---

## 11. Tests / Regression

| Suite | Result |
|-------|--------|
| `_audit_phase4_recovery.py` | **PASS** (~2.2 ms probe) |
| `_audit_phase4_probe.py` | **PASS** |
| `_audit_phase4_side_switch.py` | **PASS** |
| `_audit_phase4_commitment.py` | **PASS** |

---

## 12. Performance

- 无新增完整 rollout 家族；corridor 从已有 path O(n) 建边
- Historical retreat 仅在 dead-end 分支触发（非每帧全量）
- Probe offline ~2 ms；LIVE 仍受既有 Probe+MPPI 主导

---

## 13. Known Limitations

1. Historical retreat 执行仍映射为 reverse escape 意图，**未**实现精确 breadcrumb path-follower（下阶段可加强）
2. Turn corridor 为定点 yaw 采样带，非圆弧平移
3. UI 多边形为 left/right ribbon（footprint 半宽），非完整凸包网格
4. LIVE 死胡同深走廊 + 动态挡历史（S3/S4）未全量自动化（算法已测 partial retreat）

---

## 14. Future Real Vehicle

- 保持 `DEFAULT_GEOM` / 同一 integrator
- Breadcrumb 持久化 + 地图图层
- Retreat 作为独立 Maneuver mode（路径跟踪）
- 与车载 Safety PLC 联锁

---

## 15. STATUS

```text
STEP 3F STATUS = PASS
BEHAVIOR CHANGED = YES
3G PREREQUISITES = PASS
NEXT = only when explicitly ordered
```
