# PHASE4 P1-1 — Rolling Local Planner + Open-Space Speed Policy

**Status:** `P1-1 = PASS`  
**Baseline:** `6e33261fb41025dcd6f1a0c361925ee62501d765`  
**Scope:** Implementation of a **new** `RollingLocalPlanner` + `SpeedPolicy`.  
`LocalManeuverSelector` is **not** expanded into a continuous planner.

```text
P0-A / P0-B0 / P0-B / P0-C / P0-C.1 / P1-0 = PASS (regressed)
3C / 3D / 3E / 3F (offline reverse execution) = PASS
P0-D / P0-E / P1-2 / 3G = NOT STARTED / FORBIDDEN
```

---

## One-sentence result

OPEN now has a **continuous rolling local plan (~2.0 m / ~3–7 s spatial-driven)** sitting between Global (~5 m) and MPPI (~1.6 s), with an **explicit OPEN cruise `target_vx=0.30`** that MPPI samples around; Local Selector remains obstacle-triggered avoidance; Safety remains the last gate.

---

## 1. Local Planner — does it actually run continuously?

**YES** in OPEN / FOLLOW_GLOBAL / FORWARD_TRACK.

```text
LocalMppiModel.step
  → SpeedPolicy.compute
  → RollingLocalPlanner.update   (period 0.20s, physics local_period also 0.20s)
  → MPPI.step(local_plan_path, target_vx)   if authority == ROLLING_LOCAL
  → apply_safety
  → physics
```

OPEN no longer means “no local planning”.  
`LocalManeuverSelector.compare()` can still be `false` (`NONE_OPEN_FORWARD`). That is **avoidance compare**, not the rolling planner.

Smoke (`LocalMppiModel.step` open, no obstacles):

```text
authority = ROLLING_LOCAL
plan_id   = LP-000001
horizon_m = 2.01
selected  = FORWARD_12
target_vx = 0.30
selector.current = FORWARD   (unchanged role)
```

---

## 2. Open space — 2–4 s rolling plan?

**YES, spatial-first.**

OPEN floor is `max(2.0 m, v * horizon_s)` capped by Global preview.  
At `v≈0.30`, 2.0 m needs ~6.7 s of integration — **spatial floor wins over the 4 s advisory cap** (hard cap 8 s). Reported:

```text
horizon_m = 2.01
horizon_s ≈ steps * 0.10  (extended to reach 2 m)
coverage vs Global 5 m = 0.40
```

This is **not** cheating by painting a longer MPPI band. Poses come from `RollingLocalPlanner._rollout`.

---

## 3. Local plan ≥ 2 m?

**YES** (Test A + smoke): `2.01 m`.

| Kind | OPEN evidence |
|------|----------------|
| PLANNED | 2.01 m |
| RENDERED | up to 64 poses in UI/API |
| EXECUTED | MPPI tracks plan; plant follows Safety cmd |

---

## 4. Does MPPI only execute?

**Mostly YES in OPEN.**

- Topology / which arc: **RollingLocalPlanner**
- Short-horizon vx/w optimization: **MPPI** (still owns command)
- PP: still a **w hint**, lookahead can follow local plan
- Avoidance override: **LocalManeuverSelector + FSM** (`LOCAL_LEFT/RIGHT`) — MPPI force path
- Recovery: **3F FSM** — MPPI force reverse, planner does not write mode
- Hard veto: **Safety**

MPPI horizon **unchanged** (`16 × 0.1 = 1.6 s`).

---

## 5. Open cruise explicit?

**YES.**

```text
NAV_OPEN_CRUISE_VX default = 0.30
SpeedPolicy.target_vx
MPPI _score uses target_vx (no longer hardcoded 0.22)
SIDE_VX = 0.22 remains legacy selector-only
```

`NO EXPLICIT OPEN-SPACE CRUISE` from P1-0 is **resolved**.  
0.30 is **configurable**, not a claimed final vehicle calibration.

---

## 6. Decoupled from 0.16 prior?

**YES.**

Before each sample (OPEN):

```text
_mean_vx = 0.55 * _mean_vx + 0.45 * target_vx
```

Sampler std still explores around that center. Init `_mean_vx=0.16` is only a start value.

Offline EMA @ 0.20 s: cmd ≈ 0.29 by 1.5 s (was ~0.24 after 5 s at 0.35 s with 0.82/0.18).

Smoke first tick `vx≈0.11` is remaining cmd EMA from 0 — **not** a frozen 0.16 prior.

---

## 7. Acceleration smoothing kept?

**YES.** Physics `acc_v=0.9` unchanged. Command EMA kept, just faster:

```text
mean: 0.85/0.15  (was 0.9/0.1)
cmd:  0.50/0.50  (was 0.82/0.18)
```

No single-frame 0.16 → 0.40 jump.

---

## 8. Safety still final authority?

**YES.** `LocalPlan` never writes `cmd_vel`. Path remains MPPI → `apply_safety` → physics.

---

## 9. 3F preserved?

**YES.** Recovery authority `AUTH_RECOVERY` skips local-plan tracking; `REVERSE_ESCAPE` still uses `force_vx<0`.

```text
_audit_phase4_recovery_execution.py
  LOCAL_REVERSE → REVERSE_ESCAPE + force_vx=-0.12 + progress
```

Planner does **not** implement REVERSE_ARC / rotation guard (P0-E not started).

---

## 10. Old LocalManeuverSelector still avoidance?

**YES.** Untouched compare gate. OPEN compare may still be false. Dual-local rule:

```text
Normal:     RollingLocalPlanner
Avoidance:  LocalManeuverSelector
Recovery:   Recovery FSM
```

One `last_maneuver_authority` at a time. Planner does not set FSM `mode`.

---

## Ownership map (P1-1)

```text
MISSION
  ↓
GLOBAL REFERENCE (~5 m)
  ↓
NAVIGATION POLICY + SpeedPolicy.target_vx
  ↓
ROLLING LOCAL PLANNER (2 m+ OPEN)     ← NEW, always in OPEN
  ↓
LocalManeuverSelector                 ← still obstacle-triggered only
  ↓
MPPI (1.6 s, tracks local plan in OPEN)
  ↓
FSM force path when avoidance/recovery
  ↓
SAFETY
  ↓
PHYSICS (20 Hz)
```

---

## Horizon matrix (updated)

| Layer | Horizon | Distance (OPEN) | Rate |
|-------|---------|-----------------|------|
| Global preview | — | ~5 m | on path |
| Rolling Local | 2–4 s (spatial floor 2 m OPEN) | **≥2.0 m** | **5 Hz** (`local_period=0.20`) |
| Local Selector | 1.5 s | 0.27–0.33 m | when compare |
| MPPI | **1.6 s unchanged** | ~0.3–0.5 m at cruise | with local step |
| Safety / Physics | now | stop envelope | 20 Hz |

---

## Speed chain (real)

```text
geom.max_vx = 0.40
  → SpeedPolicy.target_vx (OPEN 0.30, then curvature/clearance/goal/front)
  → Local candidate v samples in [0.10, v_max_allowed]
  → MPPI sample center = blended mean/target
  → vx_raw → mean EMA → cmd EMA
  → vx_scale (OPEN 1.0)
  → Safety
  → state.vx via acc_v
```

---

## Tests A–E (`scripts/_audit_phase4_local_planner_open.py`)

| Test | Result |
|------|--------|
| A open ≥2 m, target 0.30, FORWARD, coverage≥0.4, <80 ms | PASS (16.6 ms, 2.01 m) |
| B mean/cmd leave 0.16 toward 0.30 | PASS |
| C gentle path has LEFT/RIGHT arc families | PASS |
| D tight/curvature reduces target | PASS |
| E left obstacle: RIGHT_ARC valid **before** contact | PASS (selected still FORWARD if it clears — generation capability, not P0-D execute-now) |
| Rolling replace plan_id | PASS |

---

## Baseline vs P1-1 (same open straight idea)

| Metric | `6e33261` P1-0 | P1-1 |
|--------|----------------|------|
| Local OPEN role | OBSERVATION_ONLY | **EXECUTION reference** |
| local horizon_m | ~0.3 | **2.01** |
| target_vx | none (soft 0.22) | **0.30** |
| mean prior | stuck ~0.16 | tracks 0.30 |
| MPPI horizon | 1.6 s | 1.6 s |
| Safety | pass-through OPEN | unchanged |

---

## UI / API

- `GET /api/nav/local-plan`
- `GET /api/nav/preview` includes `local_plan` + `mppi_summary`
- Map: Global slate, Rolling candidates cyan/blue, selected local plan navy, Probe/MPPI band still short
- Source label: `ROLLING_LOCAL_PLANNER` (MPPI no longer impersonates Local in OPEN)

---

## Risks

| Risk | Mitigation |
|------|------------|
| 2 m at 0.30 m/s needs >4 s integration | Spatial floor documented; hard cap 8 s |
| `local_period` 0.35→0.20 increases MPPI CPU | Planner itself ~17 ms OPEN; monitor |
| LEFT obstacle still selecting FORWARD if clear | P0-D will decide *when* to commit to the arc |
| Faster EMA | Still clipped by acc_v; 3F uses force path |

---

## STOP

```text
P0-D = NOT STARTED
P0-E = NOT STARTED
P1-2 = NOT STARTED
3G   = FORBIDDEN
```

---

## Code Audit Index

**FULL COMMIT:** `3118559dead2024c13e8468eaec446345ed1c42f`  
**SHORT:** `3118559`  
**URL:** https://github.com/driveCar777/agv-v0.1-sim/commit/3118559dead2024c13e8468eaec446345ed1c42f

| # | Topic | Blob |
|---|-------|------|
| 1 | Rolling Local Planner | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/nav_local_planner.py |
| 2 | Speed Policy | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/nav_speed_policy.py |
| 3 | LocalPlan interface / wiring | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/nav_models.py |
| 4 | MPPI integration | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/mppi_controller.py |
| 5 | Local Selector (unchanged) | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/local_maneuver.py |
| 6 | Telemetry / events | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/nav_observability.py |
| 7 | API / preview | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/agv_bridge/agv_bridge/sim_api_ext.py |
| 8 | Open audit A–E | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/scripts/_audit_phase4_local_planner_open.py |
| 9 | Report | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/docs/PHASE4_P1_1_ROLLING_LOCAL_PLANNER_REPORT.md |
| 10 | UI layers | https://github.com/driveCar777/agv-v0.1-sim/blob/3118559dead2024c13e8468eaec446345ed1c42f/ros2_ws/src/delivery_web/www/sim3d.js |

Also: `nav_global_preview.collect_local_candidates` (rolling priority), `run_web_sim.py` routes, P1-0 audit `docs/PHASE4_P1_0_LOCAL_PLANNING_ARCHITECTURE_AUDIT.md`.
