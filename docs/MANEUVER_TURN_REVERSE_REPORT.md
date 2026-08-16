# AGV Maneuver / Turn / Reverse 专项修复报告

日期：2026-08-16  
范围：Web 仿真导航（`agv_bridge` + `delivery_web`）  
原则：Maneuver Semantics，非参数调优；不引入 Hybrid-A* / Nav2。

---

## 1. 当前 Reverse 真实调用链（修复后）

```
sim_api_ext._physics_loop
  → progress.update(accumulate_stuck = not maneuver.pause_stuck)
  → stuck≥8s 且 phase=forward：仅日志 STUCK→MANEUVER_DECIDE
     （已删除直接 begin_reverse_escape 主路径）
  → LocalMppiModel.step(...)
       → ManeuverFSM.decide(...)
            仅在 DEAD_END / 碰撞陷阱 / STUCK且转向与reposition皆不可时
            → mode=REVERSE_ESCAPE
       → phase = decision.legacy_phase  # "reverse_escape"
       → DiffDriveMppi.step(force_reverse=True, vx∈[rev_min,0], ...)
  → apply_safety(..., phase)  # 后雷达/碰撞门；不发明普通 reverse
  → accel limiter → state.vx / state.w
```

旧链（问题根因）：`stuck_s ≥ 8` → `begin_reverse_escape()` → `force_rev` → MPPI 负 vx → 长时间 reverse。

---

## 2. 当前 Forward Feasibility 真实逻辑

`maneuver.assess_forward_feasibility()`：

| 条件 | reason | maneuver_required |
|------|--------|-------------------|
| collision | COLLISION | RECOVERY_REQUIRED |
| front_near < front_stop | CLEARANCE_TOO_LOW | RECOVERY_REQUIRED |
| \|herr\| > horizon·wz_max | HEADING_UNACHIEVABLE | ALIGN_REQUIRED |
| \|herr\| > H_ALIGN | CURRENT_HORIZON_CANNOT_CAPTURE_PATH | ALIGN_REQUIRED |
| no capture | PATH_CAPTURE_UNREACHABLE | REPLAN |
| 候选全撞 | COLLISION | RECOVERY_REQUIRED |
| 否则 | OK / HEADING_UNACHIEVABLE | DIRECT_FORWARD / FORWARD_TURN / ALIGN_REQUIRED |

`max_heading_change_in_horizon ≈ 0.42×1.6 = 0.672 rad`。

---

## 3. 新 Maneuver FSM

模块：`agv_bridge/maneuver.py` → `ManeuverFSM`  
接入：`LocalMppiModel.step` → 约束 `DiffDriveMppi` 动作空间。

状态：IDLE / FORWARD_TRACK / FORWARD_TURN / ALIGN / TURN_IN_PLACE / REPOSITION / REVERSE_ESCAPE / POST_TURN / WAIT_FOR_CLEARANCE / REPLAN / SAFE_STOP

---

## 4. 各状态进入/退出（摘要）

| 状态 | 进入 | 退出 |
|------|------|------|
| FORWARD_TRACK | \|herr\|≤H_FORWARD 且 forward OK | herr 增大 / 不可行 |
| FORWARD_TURN | H_FORWARD<\|herr\|≤H_ALIGN | 收敛或失败 |
| ALIGN / TURN_IN_PLACE | ALIGN_REQUIRED 或 goal-behind override；需 rotation_safe | \|herr\|≤ALIGN_TOL + capture 近 → POST_TURN → FORWARD |
| REPOSITION | 需对准但旋转不安全 | timeout → REPLAN；或 clearance 改善后 ALIGN |
| REVERSE_ESCAPE | 真死胡同 / 碰撞陷阱（非“目标在后”） | IMPROVED→ALIGN；WORSE→立即 abort；timeout→REPLAN |
| SAFE_STOP | SIL 无效 / emergency / recovery 耗尽 | — |

转向期间 `pause_stuck=True`，不触发 stuck→recovery 抢权。

---

## 5. ALIGN / TURN_IN_PLACE

- `force_vx=0`，`force_w = clamp(1.15·herr, ±wz_max)`
- MPPI 在 ALIGN/TURN 模式直接执行该命令（仍经 Safety）
- 退出：航向容差 + path capture 距离
- 超时：TURN_MAX_S / ALIGN_MAX_S → REPOSITION 或 REPLAN

---

## 6. REPOSITION

短距 `force_vx≈±0.06` + 向更宽侧 `force_w`；不可原地转时使用。

---

## 7. REVERSE_ESCAPE

- 默认关闭 NORMAL_REVERSE
- 进入时 `REVERSE_DECISION_SNAPSHOT`
- ≥REVERSE_EVAL_S 评估 clearance/progress/heading → IMPROVED / NO_CHANGE / WORSE
- WORSE 同 tick 禁止再进 reverse（`reverse_aborting`）

---

## 8. Path Capture

`find_best_path_capture()`：距离 + 航向差 + clearance 代价，非单纯最近点。  
Goal-behind override：当 goal 航向差大且 progress 停滞，即使用 path tangent 小误差，也改用 goal heading 决策。

---

## 9. Forward / Reverse selection

Maneuver mode 限制动作空间：

- FORWARD*：vx≥0  
- ALIGN/TURN：\|vx\|≤ε，w 自由  
- RECOVERY_REVERSE：vx<0  

MPPI 不再在 FORWARD 模式下偷选负 vx。

---

## 10. Safety / SIL

- 所有机动经 `apply_safety`
- ALIGN/TURN 时碰撞不发明 reverse；允许 vx≈0 时保留 w
- NaN pose / emergency → SAFE_STOP / SIL_INPUT_INVALID
- Debug UI 只读，不写控制

---

## 11. Reverse before/after

Telemetry：`reverse_before` / `reverse_after`（clearance_delta, progress_delta, heading_delta, result）。

---

## 12. 测试矩阵结果

| Case | 结果 |
|------|------|
| M1–M18 offline (`_audit_maneuver_cases.py`) | **PASS 32/32** |
| Component system | **PASS**（含 dbg_maneuver*） |
| Live rear-goal (`_audit_rear_goal_live.py`) | **PASS** |

Live 实录（目标在车后约 3m）：

```
TURN_IN_PLACE (herr 2.72→0.23)
→ FORWARD_TRACK
reverse_streak = 0
```

---

## 13. 当前真实案例回放（对应原失败模式）

原模式：FORWARD → DIVERGENCE → PATH_PROGRESS_STALLED → reverse_escape → FAILED  

修复后同场景语义：

1. Global path 可仍为 GOOD  
2. Forward NOT_AVAILABLE / HEADING_UNACHIEVABLE → **ALIGN_REQUIRED**  
3. Maneuver → **TURN_IN_PLACE / ALIGN**（非 reverse）  
4. 对准后 **POST_TURN → FORWARD_TRACK**  
5. 仅真死胡同才 REVERSE_ESCAPE，且可 WORSE 早退  

说明：实况验证时曾误连到旧进程（端口 19999 被 18:21 旧实例占用）；杀干净后新代码行为符合预期。

---

## 14. 未解决问题

1. **Progress 指标**：path_progress 仍可能在弯道/重捕获时偏慢，依赖 stuck gating；未彻底重写 progress 算法。  
2. **左右绕行 rollout 成本**：left/right free 已作 feasibility 输入，尚未做完整 short-horizon left/right cost 比较器。  
3. **TURN 期间 reason 显示**：偶发显示 `HEADING_UNACHIEVABLE`（来自 forward 子结构）而非 `HEADING_ALIGN`——行为正确，文案可再统一。  
4. **动态障碍 WAIT**：有 WAIT_FOR_CLEARANCE，长期动态堵塞的 replan 策略仍偏简。  
5. **多实例端口**：本地易残留旧 `run_web_sim`；建议启动前检查 19999。

---

## 15. 下一阶段建议

- Local planner 与 Maneuver 进一步拆分（候选生成 / 代价 / 执行）  
- 评估 Hybrid-A*（仅当 path capture 系统失败率仍高）  
- Path smoother / sticky candidate  
- 独立航向控制器（在 Maneuver 之外）  
- 左右绕行 short rollout 成本比较落地  

---

## 关键文件

| 文件 | 作用 |
|------|------|
| `agv_bridge/maneuver.py` | Maneuver FSM / capture / feasibility / reverse guard |
| `agv_bridge/nav_models.py` | LocalMppi 接入 FSM |
| `agv_bridge/mppi_controller.py` | 按 mode 约束动作空间 |
| `agv_bridge/sim_api_ext.py` | stuck 不再直接 reverse；Safety；telemetry |
| `agv_bridge/nav_debug.py` | Maneuver / FWD-REV / Capture 字段 |
| `delivery_web/www/nav_ui.js` | 三张卡 + Maneuver Timeline |
| `scripts/_audit_maneuver_cases.py` | M1–M18 |
| `scripts/_audit_rear_goal_live.py` | 后方目标实况 |

---

**验收对照**：A/B/E/F/G/H/I/J/K/L/M/N 在 offline + 后方目标实况上已覆盖；C/D 由 M3/M4 覆盖。
