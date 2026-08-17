# PHASE4 P0-D — Obstacle Approach Anticipation Report

日期：2026-08-17  
基线：`605e23e` / P1-2-LIVE PARTIAL  
范围：**P0-D Forward Future Preview + predictive OBSTACLE_APPROACH trigger + early local activation**  
禁止：P0-E / P1-2 impl / 3G / REVERSE_ARC / 修改 `front_stop_m` / 粉色点拉远到 5m

```text
P0-D STATUS = PASS (offline A–L)
P0-D LIVE    = PARTIAL (trace script ready; manual obstacle placement)
3C/3D/3E     = PASS (regression)
3F           = PASS (local planner / probe / side-switch; recovery C2_edges pre-existing)
REVERSE_ARC  = NOT IMPLEMENTED
Pink point   = DISPLAY-ONLY (unchanged 1.4m GLOBAL)
NEXT         = P0-E / P1-2 / BLOCKED until user request
3G           = FORBIDDEN
```

---

## 1. 为什么以前这么晚才开始绕？

| 根因 | 证据 |
|------|------|
| Policy `OBSTACLE_APPROACH` 绑定 `front_near` / scene `APPROACH` | `front_cost_m+0.55` ≈ 1.45m 才 compare side |
| Local Planner `fwd_ok` 需 `front_near ≥ front_stop_m+0.15` (0.85m) | OPEN 场景 FORWARD 长期优先 |
| 粉色点 1.4m **DISPLAY-ONLY**，不参与规划 | P1-2-LIVE CONFIRMED |
| 无 forward swept-footprint 预测 | 只有 reactive `front_near` |

P0-D 增加：**沿 Global Reference 的 swept footprint 未来碰撞预测**，在 `first_collision_distance < required_avoidance_distance` 时提前进入 `OBSTACLE_APPROACH` 并激活 Rolling Local Planner 侧向候选。

---

## 2. Future preview 检测距离

| 参数 | 值 |
|------|-----|
| `MIN_FUTURE_PREVIEW_M` | 2.0m |
| `NORMAL_FUTURE_PREVIEW_M` | 5.0m |
| `MAX_FUTURE_PREVIEW_M` | 8.0m |
| Adaptive | speed × 5, global preview, goal distance, tight curvature |

Offline audit B：5m 障碍 → `future_collision=true`, `first_collision_distance_m` ≈ 4.0–4.5m。

---

## 3. Maneuver start distance

```text
required_avoidance_distance =
  d_latency (v × 0.25s)
+ d_setup (0.45×L + 0.25)
+ d_turn (1.15×W + 0.65×v + 0.15×|w|)
+ d_safety (front_stop_m + margin + 0.20)
+ d_brake (v² / 2a_dec)
```

| v | required (approx) |
|---|-------------------|
| 0.10 m/s | 2.43m |
| 0.30 m/s | 2.68m |

Audit I：障碍 @3.4m → 高速 `approach_active=true`，低速 `false`。

---

## 4. Hard stop

```text
D_HARD_STOP = front_stop_m = 0.70m  (UNCHANGED)
```

---

## 5. High speed 是否提前？

**YES** — `required_avoidance_distance` 随速度增大；同障碍位置下高速更早触发 `approach_active`（Audit I PASS）。

---

## 6. Left/right 如何比较？

- 左右 corridor：`kappa=±0.55` 弧 rollout + `trajectory_collision` swept 验证
- 输出：`left_valid`, `right_valid`, `left_clearance`, `right_clearance`
- Global reconnect hint：`left_reconnect_m` / `right_reconnect_m` → `preferred_side`
- **非 blind LEFT** — 两侧均做几何验证（Audit D/E PASS）

---

## 7. Global Path 如何参与？

- Forward preview 沿 **Global Reference** densified polyline（非 centerline-only）
- Local candidate scoring 保留 `reconnect_m` / `global_deviation_m`
- **Global Reconnect Gate**：`obstacle_passed=false` 且 `future_collision` 时 `reconnect_cost × 3.6`，FORWARD hard-invalid `FUTURE_GLOBAL_BLOCKED`（Audit L PASS）

---

## 8. Kinematic validation 是否参与？

- Future preview：**发现**未来路径问题（P0-D）
- Local candidate rollout：**P0-C 同级** `trajectory_collision` + clearance（P1-1 已有）
- Global kinematic validator 不变

---

## 9. Swept footprint 是否参与？

**YES** — `nav_footprint.trajectory_collision` on interpolated segments; segment-wise `first_collision_distance_m`（Audit H PASS）。

---

## 10. 如何防止过早 reconnect？

`RollingLocalPlanner._rollout(reconnect_gate=True)` when `not obstacle_passed && future_collision`:

- FORWARD → `FUTURE_GLOBAL_BLOCKED` (invalid)
- `reconnect_cost × 3.6`

`obstacle_pass_state`: APPROACHING / BESIDE / PASSING / PASSED / UNKNOWN

---

## 11. Ultrasonic 接入风险是否降低？

**PARTIAL** — 规划层现在在 ~2.4–2.7m（速度相关）开始 maneuver，早于 0.70m hard stop。真实 ultrasonic latency 可通过 `NAV_APPROACH_LATENCY_S` 配置。LIVE 未完整验证 sensor fusion。

---

## 12. 是否仍可能在 obstacle front 停住？

**YES** — Safety `front_stop_m=0.70m` 仍为最终门。若 LEFT/RIGHT 均 invalid → 现有 replan/recovery 路径。P0-D 不消除所有 dead-end。

---

## 13. 3F 是否保持？

| Check | Status |
|-------|--------|
| Local planner open | PASS |
| Probe | PASS |
| Side-switch | PASS |
| Recovery audit C2_edges | FAIL (pre-existing footprint edge count; unrelated to P0-D policy) |

3F corrective LIVE：**NOT RE-RUN** this session.

---

## 14. Reverse Arc 是否实现？

**NOT IMPLEMENTED** — Recovery 仍为 LOCAL_REVERSE / HISTORICAL_RETREAT。

---

## 15. Pink point 是否仍 display-only?

**YES** — `sim_api_ext._lookahead_point(gpath, 1.4m)` 未改。新增 **orange Future Preview** 层（`sim3d.js` `futurePreview`）。

---

## Files changed

| File | Role |
|------|------|
| `nav_obstacle_preview.py` | **NEW** — FuturePreviewResult, adaptive preview, corridors, events |
| `nav_policy.py` | `FUTURE_COLLISION→OBSTACLE_APPROACH` trigger (OR legacy) |
| `nav_local_planner.py` | Early side select, reconnect gate, future_global_blocked |
| `nav_models.py` | Preview each tick → policy + local planner |
| `mppi_controller.py` | `compute_dynamic_lookahead_m` for PP reference |
| `sim_api_ext.py` | debug blob + `GET /api/nav/obstacle-preview` |
| `run_web_sim.py` | API route |
| `sim3d.js` / `nav_ui.js` / `sim_main.html` | Orange future preview layer |
| `scripts/_audit_phase4_obstacle_approach.py` | **NEW** A–L |
| `scripts/_trace_phase4_obstacle_approach_live.py` | **NEW** L1–L7 |

---

## P1-2-LIVE 引用

P1-2-LIVE **未捕获 bypass/turnback** — P0-D **不得声称已修复 turnback**。

| Item | Status |
|------|--------|
| Bypass early activation | PARTIAL (offline PASS; LIVE NOT TESTED) |
| Turnback | NOT TESTED |
| Pink display-only | PASS (unchanged) |
| LOCAL_REVERSE | PASS (unchanged) |

---

## Observability events

`FUTURE_OBSTACLE_DETECTED`, `FUTURE_CLEARANCE_WARNING`, `OBSTACLE_APPROACH_ENTER`, `OBSTACLE_APPROACH_EXIT`, `LATE_AVOIDANCE_SUSPECTED`, `PLANNER_DELAYED_AVOIDANCE`, `LOCAL_PLAN_EXECUTION_MISMATCH`

---

## STOP

P0-D complete. **Do NOT enter P0-E / P1-2 / 3G** without explicit user request.
