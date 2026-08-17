# PHASE4 P1-2-LIVE — Lookahead Distance + Turn-Back + Breadcrumb / Reverse Forensics

**Repo:** https://github.com/driveCar777/agv-v0.1-sim  
**Baseline:** P1-2-OBSERVE `3aaa317` / forensics `bbef47a`  
**LIVE trace commit:** `25653215fbc6fb64d0a33555ff2a2078d3539bd5`  
**Status:** `P1-2-LIVE = PARTIAL` (core lookahead + recovery LIVE; bypass/turnback not reproduced in window)

---

## Executive Summary

| Topic | LIVE Verdict |
|-------|----------------|
| Pink point ≈ fixed 1.4m display parameter | **CONFIRMED** (measured 1.43–1.93m) |
| Display vs control lookahead split | **CONFIRMED** (code + LIVE display; PP meta not in debug API) |
| Obstacle late bypass / turnback | **NOT EVIDENCED** in 12–15s LIVE windows |
| Recovery LOCAL_REVERSE execution | **CONFIRMED** (R1/R2 + 3F corrective LIVE) |
| Historical Retreat evaluation | **CONFIRMED** (R2 action seen; execution via REVERSE_ESCAPE) |
| Breadcrumb = executed history | **CONFIRMED** (code); UI yellow = pose_trace not breadcrumb |
| Breadcrumb reset on replan | **CONFIRMED** (`plan_nav_xy` → `_clear_progress_and_recovery`) |
| REVERSE_ARC in local planner | **REFUTED** (not implemented) |

---

## 1. Lookahead Distance Constants

```text
DISPLAY_LOOKAHEAD_M     = 1.4   (hard default, sim_api_ext._lookahead_point(..., lookahead_m=1.4))
CONTROL_PP_LOOKAHEAD_M  = dynamic in mppi_controller.step():
                          base max(0.55, min(2.2, max(0.7, |vx|*1.25)))
                          capped min(la_m, local_plan_horizon_m * 0.70) when local plan active
LOCAL_PLAN_LOOKAHEAD_CAP = local_plan_horizon_m * 0.70  (e.g. 2.01m → ~1.41m)
```

**Display only:** `_lookahead_point(gpath, x, y)` — always global A* polyline.  
**Control:** `pure_pursuit_w(follow_path, …)` where `follow_path = local_plan_path if len≥2 else global_path`.

`nav_debug.controller.pp_lookahead_m` is **stale 1.4** in telemetry block; actual control uses MPPI `_last_meta.pp_lookahead_m`.

---

## 2. LIVE L0 — Fixed Display Lookahead

**Scene:** outdoor open, goal 12m, 12s, 5Hz sample.

| Metric | Value |
|--------|-------|
| display_lookahead mean | **1.653m** |
| range | 1.427 – 1.931m |
| std | 0.153m |
| vehicle_speed range | 0.076 – 0.294 m/s |
| correlation speed↔distance | **none** (distance tracks polyline vertex, not speed) |

**Verdict:** `FIXED_DISPLAY_LOOKAHEAD` — parameter 1.4m + polyline vertex snapping → ~1.4–1.9m measured. **Not** `DYNAMIC_LOOKAHEAD` for UI pink point.

---

## 3. Lookahead Suitability (offline sweep)

Vehicle length L = **1.05m**. Decel assumption 0.35 m/s².

| v (m/s) | stopping_dist | 1.4m / stopping | 1.4m / L |
|---------|---------------|-----------------|----------|
| 0.10 | 0.014m | 98× | 1.33× |
| 0.20 | 0.057m | 24× | 1.33× |
| 0.30 | 0.129m | 11× | 1.33× |

At OPEN cruise 0.30 m/s, display lookahead ≈ **4.3× stopping distance** and **1.33× vehicle length**.  
For tight obstacle approach, **1.4m display can appear “close” visually** while still being >> braking distance in sim — but **<< local plan horizon (2m)** when planner active.

---

## 4. Pink Point Product Semantics

**Answer: A — display-only debug marker**

- Rendered: `sim3d.js` `dbg.controller.lookahead_point` (pink `0xec4899`)
- Source: **GLOBAL_PATH only**
- **Not** rolling local target, **not** MPPI best path
- PP control target is separate (`_last_meta.pp_lookahead_point`) — **not shown as pink**

**UI risk:** operator interprets pink as “where AGV must go” while control may follow local plan.

---

## 5. LIVE Obstacle Scenes L1–L5

Obstacle placed ~2.5–4m ahead on global straight path.

| Scene | front_near min | local_selected_kind | turnback | display_inside_obstacle |
|-------|----------------|---------------------|----------|-------------------------|
| L1 | 1.29m | None | 0 events | false |
| L2 | 1.27m | None | 0 | false |
| L4 | 1.19m | None | 0 | false |
| L5 | 1.54m | None | 0 | false |

**Findings:**
- Vehicle **never reached** `front_stop_m=0.70m` in these windows (min front ~1.19m)
- `local_plan` / `selected_kind` **not present** in LIVE `/api/nav/debug` during runs (offline P1-1 audit PASS — wiring gap or exception in LIVE path **POSSIBLE**)
- No `LEFT_ARC`/`RIGHT_ARC` → **no bypass phase** → **turnback NOT EVIDENCED**
- Pink point stayed on global straight; **inside_obstacle NOT EVIDENCED** LIVE

**Avoidance timing (L1):**
```text
d_first_obstacle_aware (front<2.5m): ~early in scene
d_side_select: NOT EVIDENCED
d_turn_start (|state_w|>0.06): NOT EVIDENCED
d_front_stop: NOT EVIDENCED
```

**Conclusion:** “贴障才绕” **NOT EVIDENCED** in LIVE — vehicle did not get close enough for side-arc selection in 12–15s. **LIKELY** would occur closer to `front_stop_m` based on code (`nav_local_planner` forward_ok requires `front_near >= front_stop_m + 0.15`).

---

## 6. Turn-Back / Global Pull (L2/L4)

**NOT EVIDENCED** in LIVE trace — no bypass phase captured.

Code audit (P1-2-OBSERVE): when local plan active, display pink can remain on global while PP uses local → `REFERENCE_AUTHORITY_MISMATCH` **LIKELY** during bypass but not LIVE-proven here.

---

## 7. Breadcrumb / UI Colors (code evidence)

| UI element | Color | Source (sim3d.js) |
|------------|-------|-------------------|
| **Yellow line** | `0xf59e0b` | `layers.actualTrace` → `paths.actual_trace` / `pose_trace` (NavDebugHub ring) |
| **Red disk/sector** | `0xef4444` | Safety footprint, front stop sector, collision markers |
| **Pink disk** | `0xec4899` | `controller.lookahead_point` |
| **Breadcrumb** | *not rendered in 3D* | Telemetry only (`nav_ui.js` widget) |

**Yellow ≠ breadcrumb.** Yellow = short executed-pose trace for debug overlay.  
**Red ≠ historical breadcrumb.** Red = safety/collision geometry.

Backend: `TrajectoryBreadcrumb` (`nav_breadcrumb.py`) — trusted executed-pose history for retreat revalidation.

Config: sample 0.08m, max 400 pts, max 25m, max age 180s.

---

## 8. Breadcrumb Reset Chain

`breadcrumb.reset()` called from `LocalMppiModel.reset()` → triggered by:

| Trigger | Function | Evidence |
|---------|----------|----------|
| New plan | `plan_nav_xy` | `_clear_progress_and_recovery()` line 326 |
| Cancel / soft stop path clear | `soft_stop` | line 191 |
| Session reset | `_reset_nav_session` | exposed on state |
| Replan during nav | R3 LIVE | replan at t=5.2s; count drops after `plan_nav_xy` |

**STOP后轨迹消失 (yellow pose_trace):** **SESSION_RESET / REPLAN** — not age trim (180s) in short stops.

---

## 9. Historical Retreat Execution Chain

```text
TrajectoryBreadcrumb.record()          [nav_models.py ~754]
  ↓
evaluate_recovery()                    [nav_recovery.py]
  ↓ evaluate_historical_retreat()
  ↓ ACT_LOCAL_REVERSE (preferred) or ACT_HISTORICAL_RETREAT
ManeuverFSM → REVERSE_ESCAPE           [maneuver.py ~632+]
  ↓ force_vx = -0.12
MPPI step (allow_rev)                  [mppi_controller.py]
Safety supervisor                      [sim_api_ext safety_gate]
```

**HISTORICAL_RETREAT_EXECUTION = CONNECTED** (same REVERSE_ESCAPE path as LOCAL_REVERSE; retreat corridor may be selected for UI `physical_trajectory`).

---

## 10. Three Reverse Capabilities

| Capability | Status |
|------------|--------|
| **A. LOCAL_REVERSE** | **IMPLEMENTED** — Probe backward VALID → `ACT_LOCAL_REVERSE` |
| **B. HISTORICAL_RETREAT** | **IMPLEMENTED** evaluation; execution via REVERSE_ESCAPE when LOCAL_REVERSE unavailable |
| **C. REVERSE_ARC** | **NOT IMPLEMENTED** — Local planner: FORWARD / LEFT_ARC / RIGHT_ARC only |

P1-1 RollingLocalPlanner explicitly excludes reverse arcs.

---

## 11. LIVE Recovery Scenes

### R1 — Three-side blocked, rear open
- Inject t≈1.7s → `front_near` min **0.363m**
- Actions: `LOCAL_REVERSE`, stop `FRONT_OBSTACLE` / `REVERSE_ESCAPE`
- `state_vx` reached **-0.12** — **CONFIRMED reverse motion**

### R2 — Move then seal
- Breadcrumb max **6** points before seal
- Actions: `LOCAL_REVERSE`, **`HISTORICAL_RETREAT`**, `REVERSE_ESCAPE`
- `state_vx` **-0.12** — **CONFIRMED**

### R3 — Breadcrumb reset
- Replan via `/api/nav/plan` → `_clear_progress_and_recovery()` → breadcrumb cleared
- **CONFIRMED:** reset cause = **REPLAN / SESSION_RESET**

### 3F Corrective LIVE (regression)
- Scene 1: signed back **0.13m**, `requested_vx=-0.12`, `safe_vx=-0.12` — **PASS**

---

## 12. Safety / front_stop_m

```text
front_stop_m = 0.70m   (nav_geometry.DEFAULT_GEOM)
front_cost_m = 0.90m   (MPPI soft cost, not hard stop)
```

`safety_gate`: when `front_near < front_stop_m` and `vx≥0` → **vx=0** (`STOP_FRONT`) unless turning phase with low vx.

**LIVE:** L1–L5 never crossed 0.70m front_near — Safety hard stop **not triggered** in obstacle scenes.

**Ultrasonic risk (analysis only):** if real sensor noise/latency pushes effective stop inward, bypass must begin **before** `front_stop_m` or vehicle stalls before side arc completes.

---

## 13. User Observation Table

| Observation | Verdict |
|-------------|---------|
| 粉色点很近 | **CONFIRMED** (~1.4–1.9m display; ~1.3× vehicle length) |
| 粉色点不断前跳 | **CONFIRMED** (global polyline index advance) |
| 粉色点进入障碍 | **NOT EVIDENCED** LIVE (P1-2-OBSERVE synthetic CONFIRMED) |
| 快碰才开始绕 | **NOT EVIDENCED** LIVE (did not reach front_stop) |
| 绕过去又扭回来 | **NOT EVIDENCED** LIVE |
| 后方黄/红是历史轨迹 | **PARTIALLY CONFIRMED** — yellow=pose_trace; red=safety; **not breadcrumb** |
| 历史轨迹消失 | **CONFIRMED** reset on replan/plan (`SESSION_RESET`) |
| 没有倒车 planning | **CONFIRMED** no REVERSE_ARC; recovery reverse only |
| Historical Retreat 执行 | **CONFIRMED** R2 + 3F LIVE |
| 3F 直线 reverse | **CONFIRMED** force_vx=-0.12 straight reverse |

---

## 14. Problem Tree (current)

```text
用户问题
├── A 粉色点过近
│   ├── display = 1.4m fixed CONFIRMED LIVE
│   ├── control = dynamic PP CONFIRMED code
│   └── source conflict display=GLOBAL CONFIRMED
├── B 贴障才绕
│   ├── LIVE NOT EVIDENCED (stayed >1.1m from obstacle)
│   └── LIKELY near front_stop_m=0.7 per code
├── C 绕障后回头
│   └── NOT EVIDENCED LIVE
└── D 不倒车
    ├── LOCAL_REVERSE CONFIRMED LIVE
    ├── HISTORICAL_RETREAT CONFIRMED evaluation
    ├── breadcrumb reset on replan CONFIRMED
    └── REVERSE_ARC NOT IMPLEMENTED
```

---

## 15. Future Design (NOT IMPLEMENTED)

- **C:** Active LocalPlan-relative display lookahead (align pink with PP follow_path)
- Dynamic lookahead 0.8–2.5m tied to speed
- Obstacle-pass gate before global reconnect
- REVERSE_ARC candidate in local planner
- UI label: “Global reference preview” vs “Control target”

---

## 16. Regression

All required audits **PASS** including `_trace_phase4_recovery_corrective_live.py`.

---

**LIVE trace:** `logs/live_lookahead_reverse/live_forensics_20260817_154345.jsonl`

## Code Audit Index (commit `25653215fbc6fb64d0a33555ff2a2078d3539bd5`)

1. Display lookahead — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/sim_api_ext.py
2. Pure Pursuit / PP meta — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/mppi_controller.py
3. Rolling Local Planner — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/nav_local_planner.py
4. Breadcrumb — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/nav_breadcrumb.py
5. Recovery / Historical Retreat — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/nav_recovery.py
6. UI colors — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/delivery_web/www/sim3d.js
7. Lookahead forensics — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/ros2_ws/src/agv_bridge/agv_bridge/nav_lookahead_forensics.py
8. LIVE trace — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/scripts/_trace_phase4_live_lookahead_reverse.py
9. Lookahead sweep — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/scripts/_audit_phase4_lookahead_distance_sweep.py
10. This report — https://github.com/driveCar777/agv-v0.1-sim/blob/25653215fbc6fb64d0a33555ff2a2078d3539bd5/docs/PHASE4_P1_2_LIVE_LOOKAHEAD_REVERSE_FORENSICS_REPORT.md

| Artifact | Path |
|----------|------|
| LIVE trace runner | `scripts/_trace_phase4_live_lookahead_reverse.py` |
| Lookahead sweep | `scripts/_audit_phase4_lookahead_distance_sweep.py` |
| LIVE JSONL | `logs/live_lookahead_reverse/live_forensics_20260817_154345.jsonl` |
| LIVE summary | `logs/live_lookahead_reverse/live_forensics_20260817_154345_summary.json` |

---

## STOP Gate

```text
P1-2-LIVE = PARTIAL
P0-D = NOT STARTED
P0-E = NOT STARTED
P1-2 implementation = NOT STARTED
3G = FORBIDDEN
```

**No navigation behavior was modified in this phase.**
