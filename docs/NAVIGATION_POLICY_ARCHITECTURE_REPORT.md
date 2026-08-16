# AGV Navigation Policy / Planning / Local Planning / Control / Safety 最终重构报告

日期：2026-08-16  
范围：`V0.1仿真版` Web 仿真（`agv_bridge` + `delivery_web`）  
原则：架构级职责分离；扩展现有 ManeuverFSM / Local Maneuver / MPPI / Safety；不迁移 Nav2。

---

## 1. 当前旧架构的问题

审计见 `docs/CURRENT_CONTROL_OWNERSHIP_MAP.md`。核心问题：

- Global Path 经 MPPI `5·gdev` + local `require_capture` 近似**硬轨道**
- 无统一 NavigationPolicy；Behavior / Maneuver / MPPI candidate 语义重叠
- LEFT/RIGHT 有时与 ALIGN 抢入口；WAIT（动态）几乎不触发
- Reverse 虽已非 stuck 默认，仍可能与 Recovery 计数交叉
- 缺 CAUTION / OBSTACLE_APPROACH / PATH_RECAPTURE / 走廊偏离时限 / loop guards
- `planned_vx` 字段缺失；用 `mppi_vx` / before-safety 代替

---

## 2. 新架构

```
Mission / Goal
  → NavigationPolicy          # WHY / scene / corridor / profiles / loop guards
  → Global Path (REFERENCE)   # 长期意图，非硬轨
  → ManeuverFSM               # HOW TO MANEUVER（保留并扩展）
  → Local Trajectory (MPPI)   # 在 behavior 动作空间内
  → Controller tracking
  → Safety / SIL              # HARD OVERRIDE
  → Execution
```

新建：`agv_bridge/nav_policy.py`  
接入：`nav_models.LocalMppiModel` → Policy.step → Maneuver.decide(policy_ctx) → MPPI(path_follow_weight, vx_scale)

---

## 3. Planning 职责

- Global：A* + path_quality（不变）；输出 `global_path` 作参考
- 不写 vx/w；不直接 Recovery
- Replan 由 Policy `allow_replan` + FSM `REPLAN` + stuck splice/alt 共同门控

---

## 4. Behavior 职责

`NavigationPolicy` 输出：

| 字段 | 含义 |
|------|------|
| state | FOLLOW_GLOBAL / CAUTION / OBSTACLE_APPROACH / LOCAL_AVOID / PATH_RECAPTURE / WAIT / ALIGN / TURN / REPOSITION / REPLAN / RECOVERY / SAFE_STOP / … |
| behavior | FOLLOW_GLOBAL / AVOID_LEFT / AVOID_RIGHT / WAIT / … |
| corridor | half_width / lateral / exceeded / time_away |
| profile | NORMAL / CAUTION / AVOID / RECAPTURE / RECOVERY |
| allow_side_compare / allow_replan / allow_recovery | 门控 |
| path_follow_weight | MPPI 全局偏差权重（AVOID 时软化） |

不积分 pose，不绕过 Safety，不每帧直接写 vx/w。

---

## 5. Maneuver 职责

保留 `ManeuverFSM`：ALIGN / TURN / LOCAL_LEFT/RIGHT / REPOSITION / REVERSE_ESCAPE / …  
新增：`policy_ctx` 门控侧向比较、Recovery、Replan；`dynamic_short` 传入 selector。  
LOCAL_* 的 `legacy_phase` → `local_avoid`（Safety 允许小 vx 时偏航）。

---

## 6. Local Planning 职责

- `LocalManeuverSelector`：LEFT/RIGHT rollout + capture + hysteresis（保留）
- AVOID 时 `require_capture_hard=False`，偏差代价 × `path_follow_scale`
- MPPI：`path_follow_weight`、`vx_scale`；LOCAL_* 仍为有向 force（Trajectory 执行，非业务择路）

---

## 7. Control 职责

Controller（MPPI/PP）只执行选定轨迹 / force 命令 → `mppi_vx/w`。  
不择 LEFT/RIGHT，不 REPLAN，不 Recovery。

---

## 8. Safety/SIL 职责

`apply_safety` 仍为最终门；turning 含 `local_avoid`。  
Emergency / safe_stop / collision / front/rear stop 可强制归零。  
Behavior / Local / Controller **不能** override Safety。

---

## 9. Recovery 职责

仅当 Policy `allow_recovery` 且真正困境（碰撞陷阱 / 死胡同 / FSM REVERSE）时进入。  
`RECOVERY_LOOP` / 次数耗尽 → SAFE_STOP。  
目标在后方：**不**默认 reverse（ALIGN/TURN/REPOSITION）。

---

## 10. Global Path 权重策略

| Profile | path_follow 相对 | 典型 path_follow_weight |
|---------|------------------|-------------------------|
| NORMAL | 1.0 | ≈5.0 |
| CAUTION | 0.85 | ≈4.25 |
| AVOID | 0.25 | ≈1.25 |
| RECAPTURE | 0.7 | ≈3.5 |
| RECOVERY | 0.15 | ≈0.75 |

---

## 11. Local Avoid 策略

Scene：`BLOCK_LEFT_WIDE` / `BLOCK_RIGHT_WIDE` / … → `LOCAL_AVOID`  
走廊加宽（≈0.85–1.21 m）；`MIN_AVOID_HOLD` 防左右画龙；obstacle_passed → PATH_RECAPTURE。

---

## 12. LEFT / RIGHT 决策

真实 footprint rollout 成本比较（非仅 free-space）。  
综合 clearance / capture / heading / progress / deviation。  
Hysteresis + hold + soft free-space 仅作两侧硬失败回退。

---

## 13. Path Corridor

`CORRIDOR_NORMAL/CAUTION/AVOID/SEVERE/RECAPTURE`  
`MAX_LOCAL_DEVIATION_TIME_S=8` → `LOCAL_DEVIATION_EXCEEDED` → RECAPTURE/REPLAN。

---

## 14. Path Capture

保留 `find_best_path_capture`；侧向过远仍可 `NO_PATH_CAPTURE`。  
AVOID 时硬门放宽，避免“有宽空间却被 capture 卡死”。

---

## 15. Replan

Policy `allow_replan` + stuck/path invalid/deadlock/dynamic_long。  
`REPLAN_LOOP`（窗口内次数超限）→ SAFE_STOP。

---

## 16. Recovery

见 §9；与 stuck 物理环：不再直接 `begin_reverse_escape` 为主路径。

---

## 17. Reverse

结果驱动仍由 ManeuverFSM reverse snapshot（IMPROVED/WORSE/abort）负责；Policy 授权进入。

---

## 18. Dynamic Obstacle

Actors 前锥距离 → `dynamic_short` / `dynamic_long` → WAIT / REPLAN。

---

## 19. Oscillation prevention

`note_cmd_w` 符号翻转计数 → `OSCILLATION_LOOP` → WAIT。  
Avoid side hold + selector hysteresis。

---

## 20. Deadlock prevention

`planned_rejected_by_safety` + `safety_zero` + stuck → `NAVIGATION_DEADLOCK` → REPLAN。

---

## 21. 测试结果（Offline）

| 套件 | 结果 |
|------|------|
| `_audit_nav_policy_matrix.py`（A 矩阵） | **PASS 27/27** |
| `_audit_local_maneuver.py` T1–T25 | **PASS 27/27** |
| `_audit_maneuver_cases.py` M1–M18 | **PASS 33/33** |

---

## 22. Live Test

单进程 `run_web_sim`，端口 19999 Listen PID 已确认。  
场景：`outdoor_campus`。

| Case | 结果 | 观察 |
|------|------|------|
| LIVE-A 前障+右堵 | **PASS** | policy `LOCAL_AVOID`，man `LOCAL_LEFT`，max_rev=0 |
| LIVE-B 前障+左堵 | **PASS** | `LOCAL_AVOID` / `LOCAL_RIGHT` |
| LIVE-C 清障后 | **PASS** | `PATH_RECAPTURE` |
| LIVE-E 笼障 | **PASS** | 无无限 reverse（max_rev=0） |
| LIVE-F 目标在后 | **PASS** | `TURN_IN_PLACE` / `REPOSITION`，非 reverse 默认 |
| LIVE-TEL | **PASS** | `nav_policy.state` 有值 |
| LIVE-D 动态 / LIVE-G divergence | **NOT FULLY SCRIPTED** | 逻辑有；专用脚本未单独断言 |

旧 `_audit_local_maneuver_live.py`：与本次同进程复测（见终端）。

---

## 23. 未解决问题 / 诚实清单

| 项 | 状态 |
|----|------|
| 地图 Local Corridor 几何可视化（Three.js 走廊带） | **NOT IMPLEMENTED**（telemetry/corridor 字段有；地图多边形未画） |
| LIVE-D / LIVE-G 专用脚本断言 | **PARTIAL** |
| Idle 手动 3055 旁路 Safety | **仍存在**（设计兼容；未改） |
| ROS `agv_safety` / Nav2 统一 | **未做**（Web 自研链优先） |
| `planned_vx` 正式字段命名 | **PARTIAL**（仍用 mppi/before-safety） |
| MPPI 在 AVOID 下完全禁止右偏采样 | **PARTIAL**（LOCAL_* force 锁定；FOLLOW 下采样仍可小幅两侧） |

---

## Debug UI

`+组件` → **POLICY** 组：

- NAVIGATION POLICY
- POLICY TIMELINE
- POLICY WEIGHTS  

缓存：`nav_ui.js?v=20260816p1`

---

## 关键文件

| 文件 | 作用 |
|------|------|
| `agv_bridge/nav_policy.py` | **新建** Policy / Corridor / Profiles / Loops |
| `agv_bridge/nav_models.py` | Policy → Maneuver → MPPI 管道 |
| `agv_bridge/maneuver.py` | policy_ctx / dynamic_short / local_avoid phase |
| `agv_bridge/local_maneuver.py` | soft capture / path_follow_scale |
| `agv_bridge/mppi_controller.py` | path_follow_weight / vx_scale |
| `agv_bridge/sim_api_ext.py` | dynamic actors cue / safety flags / policy telem |
| `agv_bridge/nav_debug.py` | `nav_policy` blob |
| `delivery_web/www/nav_ui.js` | Policy 三卡 |
| `scripts/_audit_nav_policy_matrix.py` | A 矩阵 |
| `scripts/_audit_nav_policy_live.py` | Live A/B/C/E/F/TEL |
| `docs/CURRENT_CONTROL_OWNERSHIP_MAP.md` | STEP1–3 审计 |

---

## MIGRATION_CANDIDATE（未来，非本轮）

若需工业级 Behavior Tree / 标准 Collision Monitor：

- Nav2 Smac / MPPI Controller
- Behavior Server
- Collision Monitor + Velocity Smoother  

本轮 Web 架构边界已可解释控制命令来源：Policy → Maneuver → Local → Safety。

---

**验收对照（用户目标时间线）**：LIVE-A/B 已出现  
`LOCAL_AVOID` → `LOCAL_LEFT/RIGHT` →（清障）`PATH_RECAPTURE`，且非  
贴障 → 画龙 → STOP → 无限 REVERSE。
