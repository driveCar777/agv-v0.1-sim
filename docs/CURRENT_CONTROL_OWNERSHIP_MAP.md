# CURRENT CONTROL OWNERSHIP MAP

日期：2026-08-16  
范围：`V0.1仿真版` Web 仿真（`agv_bridge` + `delivery_web`）  
阶段：STEP 1–3 审计结果（实现前基线）

---

## 1. 权威栈（谁赢）

```
Behavior (ManeuverFSM + LocalManeuverSelector)   ← 缺统一 NavigationPolicy
    → force_vx / force_w / vx_min/max / mode / phase
Controller (DiffDriveMppi | PP-only)
    → mppi_vx / mppi_w  （建议）
Safety Supervisor (apply_safety)
    → safe_vx / safe_w  （导航最终门）
Accel limit + integrate
    → state.vx / state.w / pose
```

| 旁路 | 何时 |
|------|------|
| 旁路 Behavior+Controller | 手动 3055 且非 tracking/avoid（Safety 结果被覆盖） |
| 旁路 Controller | ALIGN / TURN / LOCAL_LEFT/RIGHT / SAFE_STOP / WAIT：MPPI 直通 force_* |
| 旁路 Safety（导航） | 无（除 idle 手动） |
| Physics 覆盖 Behavior | stuck 耗尽 → SAFE_STOP；`force_rev` 仅当 phase 已是 reverse_escape |

---

## 2. 物理环调用链

```
run_web_sim.py → patch_mock_state → _physics_loop (~20Hz)
  ProgressTracker.update (accumulate_stuck = not pause_stuck)
  [optional] GlobalPlannerModel.replan
  LocalMppiModel.step
      ManeuverFSM.decide (± LocalManeuverSelector.compare)
      DiffDriveMppi.step(force_*, vx bounds)
  apply_safety → safe_*
  state.vx/w accel → integrate
```

---

## 3. 信号归属

| 信号 | 主写者 | 最终权威 |
|------|--------|----------|
| force_vx/w | ManeuverFSM.decide | Behavior → Controller 服从 |
| force_rev | physics 若已 reverse_escape；FSM 设 mode | Behavior 管进入 |
| phase / mode | ManeuverFSM（+ physics SAFE_STOP） | Behavior |
| LEFT/RIGHT/REVERSE 选择 | LocalManeuverSelector → FSM | Behavior |
| mppi_vx（planned） | DiffDriveMppi | Controller 建议 |
| safe_vx/w | apply_safety（+ idle 手动） | Safety |
| state.vx/w | _physics_loop | Plant |
| global replan | GlobalPlannerModel / SimWorld | Path 层（不写 vx） |

**注意：** 无名为 `planned_vx` 的字段；用 `mppi_vx` / `_cmd_vx_before_safety`。

---

## 4. 关键旁路 / 冲突

1. Idle 手动 3055 可旁路 Safety 结果  
2. LOCAL_LEFT/RIGHT 下 MPPI 不采样，selector 即控制器  
3. Safety 在 reverse_escape+碰撞时可注入 vx=-0.10  
4. MPPI `5·gdev` + local `require_capture` → Global Path 近似硬轨道  
5. 走廊 compare 可抢 ALIGN 入口  
6. `dynamic_short=False` 常驻 → WAIT 几乎死  
7. 无 CAUTION / OBSTACLE_APPROACH / PATH_RECAPTURE / ARRIVED / FAILED 统一 Policy 状态  
8. `begin_reverse_escape` 仍在代码中，physics stuck 主路径已不再调用  

---

## 5. 关键文件（扩展而非重写）

| 优先级 | 文件 |
|--------|------|
| P0 | `maneuver.py`, `local_maneuver.py`, `sim_api_ext.py`, `nav_models.py` |
| P1 | `mppi_controller.py`, `path_progress.py`, `nav_geometry.py`, `nav_debug.py`, `nav_ui.js` |
| 新建 | `nav_policy.py`（NavigationPolicy / Corridor / Profiles / Loop guards） |

---

## 6. 本轮目标权威栈（实现目标）

```
Mission / Goal
  → NavigationPolicy (WHY / behavior / corridor / profiles / loop guards)
  → Global Path (REFERENCE, not hard rail)
  → ManeuverFSM (HOW TO MANEUVER)
  → Local Trajectory (MPPI within behavior action space)
  → Controller tracking
  → Safety / SIL (HARD OVERRIDE)
  → Execution
```

Recovery 仅由 Policy 在失败兜底时授权，不是 Local Planner 普通输出。
