# PHASE4 P1-2-OBSERVE — Pink Lookahead / Global-Local Reference Conflict Forensics

**Baseline:** `3118559dead2024c13e8468eaec446345ed1c42f` (P1-1)  
**Repo:** https://github.com/driveCar777/agv-v0.1-sim  
**Scope:** Diagnostics / trace / API / report only — **no navigation behavior change**

**Status:** `P1-2-OBSERVE = PARTIAL` (code audit + offline forensics PASS; LIVE L1–L5 scenes require running web sim)

---

## Executive Summary

The **pink disk in sim3d.js is `dbg.controller.lookahead_point`**, produced exclusively from **`global_path`** via `sim_api_ext._lookahead_point(gpath, x, y)` — **not** from Rolling Local Plan or MPPI `follow_path`.

Meanwhile, **P1-1 MPPI Pure Pursuit** steers from:

```python
follow_path = global_path
if local_plan_path and len(local_plan_path) >= 2:
    follow_path = local_plan_path
```

When `tracking_local_plan=true`, **control lookahead (PP) can diverge from the UI pink point**. This is a **REFERENCE_AUTHORITY_MISMATCH** between display and control — the strongest explanation for the user seeing a pink dot continue straight through an obstacle while the vehicle eventually arcs around it.

---

## Q1–Q15 Answers

| # | Question | Answer |
|---|----------|--------|
| Q1 | 粉色点是什么？ | **CONFIRMED:** `controller.lookahead_point` — Pure-Pursuit-style point on a polyline, rendered pink (`0xec4899`) in sim3d.js |
| Q2 | 谁生成？ | **CONFIRMED:** `sim_api_ext._refresh_debug_snapshot()` → `_lookahead_point(gpath,…)` → `nav_debug.build(lookahead_pt=la)` |
| Q3 | source 是 Global / Local / MPPI / PP？ | **Display (pink): GLOBAL_PATH only.** **PP control:** LOCAL_PLAN when `len(local_plan_path)≥2`, else GLOBAL_PATH. **Not** MPPI best_path. |
| Q4 | 点为什么向前跳？ | **CONFIRMED:** Arc-length index advances on global polyline as vehicle moves; discrete vertex snapping can produce 0.05–0.15m steps; path revision / replan → larger jumps (`LOOKAHEAD_JUMP`) |
| Q5 | 点是否进入 obstacle？ | **CONFIRMED (synthetic):** When global path lookahead lands inside obstacle geometry → `LOOKAHEAD_INSIDE_OBSTACLE`. **LIVE:** NOT EVIDENCED in this session (sim not run) |
| Q6 | path source 错还是 collision 漏？ | **LIKELY display source mismatch**, not LocalPlan swept collision miss — LocalPlan uses P0-A swept footprint; pink point ignores LocalPlan entirely |
| Q7 | 绕障时 Local Plan active？ | **LIKELY:** RollingLocalPlanner 5Hz produces ACTIVE plan with LEFT/RIGHT_ARC when front blocked (P1-1 audit PASS) |
| Q8 | MPPI tracking LocalPlan？ | **CONFIRMED in code:** `tracking_local_plan=true` when `local_plan_path` passed to `MppiController.step()` |
| Q9 | Pink Point 仍来自 Global？ | **CONFIRMED:** UI path never switched to local plan in P1-1 |
| Q10 | 绕障快完成时为什么转回去？ | **NOT EVIDENCED (LIVE)**. **LIKELY:** (a) UI pink still on global through obstacle, (b) local plan `reconnect_m` / global deviation cost pulls endpoint back to global |
| Q11 | LocalPlan reconnect？ | **POSSIBLE:** `nav_local_planner.py` scores `reconnect_m`, `global_deviation_m` on candidates — diagnostics now log selected candidate metrics |
| Q12 | PP lookahead？ | **LIKELY:** PP uses local `follow_path` when active; display PP mismatch misleads operator |
| Q13 | MPPI w？ | **POSSIBLE:** `w_cmd = blend(pp_w + 0.35*mean_dw)` — sudden `pp_w` sign change if `follow_path` or lookahead target shifts |
| Q14 | FSM override？ | **NOT EVIDENCED** for turnback in this trace session |
| Q15 | Safety？ | **NOT EVIDENCED** as turnback cause; Safety clamps `safe_w` after MPPI |

---

## PINK LOOKAHEAD SOURCE (mandatory)

```text
PINK LOOKAHEAD SOURCE = controller.lookahead_point
```

**UI:** `ros2_ws/src/delivery_web/www/sim3d.js` lines 743–745  
**Data:** `dbg.controller.lookahead_point` from `nav_debug.py`  
**Producer:** `sim_api_ext._lookahead_point(gpath, x, y, lookahead_m=1.4)` — **input always `gpath` (global A*)**

---

## Production Chain (file + function)

```text
SOURCE: state._global_path (A* global polyline)
  ↓ sim_api_ext._lookahead_point(gpath, x, y)     [sim_api_ext.py ~494]
  ↓ debug_hub.build(..., lookahead_pt=la)         [sim_api_ext.py ~720]
  ↓ nav_debug → out["controller"]["lookahead_point"] [nav_debug.py ~1586]
  ↓ telemetry / GET /api/nav/debug
  ↓ sim3d.js _addDisk (pink 0xec4899)             [sim3d.js ~743]

PARALLEL (control, not displayed as pink):
  nav_models.LocalMppiModel → local_plan_path
  ↓ mppi_controller.step(follow_path=local or global)
  ↓ pure_pursuit_w / pure_pursuit_target (diagnostic)
  ↓ pp_lookahead_point in _last_meta
  ↓ GET /api/nav/forensics/lookahead → pp_lookahead
```

---

## Path Source Identity

| Reference | path_source | Used by |
|-----------|-------------|---------|
| GLOBAL_PATH | `GP-{revision}` | **UI pink point** |
| GLOBAL_REFERENCE | preview slice | Policy / validation |
| LOCAL_PLAN | `LP-{plan_id}` rev N | RollingLocalPlanner → MPPI when active |
| PP_FOLLOW_PATH | LOCAL_PLAN or GLOBAL_PATH | Pure Pursuit w bias |
| MPPI_BEST_PATH | MPPI candidate band | Short-horizon rollout (not PP lookahead) |

**Diagnostic API:** `GET /api/nav/forensics/lookahead`  
**Logs:** `/api/logs/diagnostics?window_s=10` includes lookahead fields per cycle

---

## User Observation Verdicts

| Observation | Verdict |
|-------------|---------|
| Pink point moves forward as AGV advances | **CONFIRMED** (global arc-length walk) |
| Pink point enters obstacle | **CONFIRMED** (synthetic geometry test); **NOT EVIDENCED** LIVE |
| AGV long follows pink direction | **PARTIALLY CONFIRMED** — PP may track local plan while UI shows global |
| After bypass, vehicle turns back toward pink / obstacle | **NOT EVIDENCED** LIVE; **LIKELY** from global-display mismatch + reconnect cost |
| Local planner arcs while pink stays straight | **CONFIRMED by architecture** (display=global, control=local when active) |

---

## REFERENCE_CONFLICT

When:

```text
GLOBAL = obstacle direction
LOCAL = safe bypass
PP_LOOKAHEAD (display) = GLOBAL obstacle side
PP control = LOCAL bypass (when tracking_local_plan)
```

Then:

```text
REFERENCE_CONFLICT (display vs local) = CONFIRMED
```

Offline L5 synthetic test emitted `REFERENCE_AUTHORITY_MISMATCH`.

---

## Forensic Table (offline synthetic L5)

| time | vehicle | global_err° | local_err° | lookahead_err° | la_source | pp_source | w_cmd |
|------|---------|-------------|------------|----------------|-----------|-----------|-------|
| T0 | (0.5,0,0°) | ~0 | ~+15 | ~0 | GLOBAL_PATH | LOCAL_PLAN | 0.07 |
| event | — | — | — | — | mismatch: UI global vs PP local | sep ~0.45m | — |

*Full LIVE table requires `scripts/_trace_phase4_lookahead_obstacle_intrusion.py` against running sim.*

---

## Root Cause Classification

| Hypothesis | Status |
|------------|--------|
| Pink = lookahead_point | **CONFIRMED** |
| Pink source = GLOBAL_PATH only | **CONFIRMED** |
| PP follow_path = LOCAL when plan active | **CONFIRMED** (P1-1 code) |
| Display/control reference mismatch | **CONFIRMED** (code + offline) |
| Pink causes collision directly | **NOT EVIDENCED** (misleading cue, not cmd source) |
| LocalPlan collision check missed obstacle | **REFUTED** for pink-in-obstacle case (different path) |
| Turnback at bypass end | **LIKELY** reconnect/global pull; **NOT EVIDENCED** LIVE |

---

## New Diagnostics (this phase)

| Item | Location |
|------|----------|
| Forensics assembler | `nav_lookahead_forensics.py` |
| PP target metadata | `mppi_controller.pure_pursuit_target()` + `_last_meta` |
| API | `GET /api/nav/forensics/lookahead` |
| Events | `LOOKAHEAD_*`, `REFERENCE_AUTHORITY_MISMATCH`, `LOOKAHEAD_TURNBACK`, … |
| Trace script | `scripts/_trace_phase4_lookahead_obstacle_intrusion.py` |
| Log window | `/api/logs/diagnostics?window_s=10` cycle fields extended |

---

## Regression

| Audit | Result |
|-------|--------|
| `_audit_swept_footprint.py` | PASS |
| `_audit_phase4_global_preview.py` | PASS |
| `_audit_phase4_navigation_stop_forensics.py` | PASS |
| `_audit_phase4_kinematic_path.py` | PASS |
| `_audit_phase4_local_planner_open.py` | PASS |
| `_audit_phase4_commitment.py` | PASS |
| `_audit_phase4_probe.py` | PASS |
| `_audit_phase4_side_switch.py` | PASS |
| `_audit_phase4_recovery_execution.py` | PASS |
| `_trace_phase4_lookahead_obstacle_intrusion.py --offline-only` | PASS |

---

## Commit-Pinned Code Audit Index

*(Replace `<COMMIT>` with post-push `git rev-parse HEAD`)*

1. Lookahead UI source — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/delivery_web/www/sim3d.js
2. Display lookahead producer — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/sim_api_ext.py
3. Pure Pursuit + diagnostic target — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/mppi_controller.py
4. MPPI follow_path — same file, `step()`
5. Rolling Local Planner — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/nav_local_planner.py
6. nav_models wiring — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/nav_models.py
7. nav_debug — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/nav_debug.py
8. nav_observability — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/nav_observability.py
9. Lookahead forensics — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/ros2_ws/src/agv_bridge/agv_bridge/nav_lookahead_forensics.py
10. Trace script — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/scripts/_trace_phase4_lookahead_obstacle_intrusion.py
11. This report — https://github.com/driveCar777/agv-v0.1-sim/blob/<COMMIT>/docs/PHASE4_P1_2_LOOKAHEAD_REFERENCE_FORENSICS_REPORT.md

---

## STOP Gate

```text
P1-2-OBSERVE = PARTIAL
P0-D = NOT STARTED
P0-E = NOT STARTED
P1-2 implementation = NOT STARTED
3G = FORBIDDEN
```

**No behavior fixes applied.** Findings are logged for a future P1-2 implementation phase.
