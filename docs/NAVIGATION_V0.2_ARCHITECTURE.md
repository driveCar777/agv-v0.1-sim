# AGV Navigation V0.2 — Architecture Design

**Status:** DESIGN (based on `V0.1仿真版` codebase audit, 2026-08-17)  
**Baseline:** P0-D.1 predictive avoidance stack (`nav_avoidance_phase`, `nav_side_probe`, `nav_dynamic_resume`)  
**Scope:** Physics-first redesign; migration from phase-stacked V0.1

---

## 1. Why V0.2

V0.1 passes offline audits (P0-D A–L, P0-D.1 A–F) but exhibits **field failure**:

```text
heading diverges from obstacle
→ footprint still approaches obstacle
→ planning UI red
→ mppi_vx=0, safe_vx=0, FAILED
→ vehicle permanently stopped
```

Root cause is **architectural**, not a single threshold:

| Layer | V0.1 problem |
|-------|----------------|
| Probe / Commit | `controls_vehicle: false` — diagnostics only |
| Execution | MPPI samples full (vx, w) space; no ExecutionCorridor from committed_side |
| Geometry | MPPI collision = center-point `collide(x,y)`, not footprint |
| Semantics | `front_near` = LiDAR ray distance, not footprint clearance |
| Failure | `SAFE_STOP` → `STOP_FAILED` with no MPPI-infeasible recovery ladder |
| Diagnostics | No `safe_vx_reason`; `front=` in log misleads when side-approaching |

V0.2 replaces “phase patches on MPPI” with **Vehicle → World → Behavior → Corridor → Planner → Validator → Safety**.

---

## 2. Target Control Chain

```text
Sensors (LiDAR A/B) + Localization
        ↓
WorldModel (static map + local obstacles + dynamic tracks + UNKNOWN policy)
        ↓
GlobalPlanner → GlobalPath + corridor context
        ↓
BehaviorManager (FSM — NOT MPPI)
        ↓
ExecutionCorridor (allowed region, preferred side, speed bounds, heading target)
        ↓
KinematicFeasibility (time-horizon footprint clearance)
        ↓
TrajectoryPlanner (MPPI — optimization only, constrained by corridor)
        ↓
TrajectoryValidator (independent footprint + braking check)
        ↓
SafetySupervisor → approved_command
        ↓
Controller / Robokit
        ↓
ProgressMonitor + RecoveryPlanner (feedback)
```

**Iron rules (from field failure):**

1. Vehicle is not a point.
2. Heading change ≠ lateral escape.
3. Probe ≠ execution.
4. MPPI infeasible ≠ navigation failed.
5. safe_vx=0 ≠ HARD_FAIL.

---

## 3. Module Map (V0.1 → V0.2)

| V0.1 module | V0.2 role | Action |
|-------------|-----------|--------|
| `nav_geometry.py` / `nav_footprint.py` | `vehicle/VehicleModel` | **Extend** — single truth; add braking, latency |
| `nav_kinematic.py` | `vehicle/KinematicModel` + `planning/TrajectoryValidator` | **Split** — validator becomes execution gate |
| `nav_policy.py` | `behavior/BehaviorManager` | **Refactor** — owns FSM; emits ExecutionCorridor |
| `nav_avoidance_phase.py` | `behavior/AvoidancePhase` (sub-state) | **Migrate** — feeds Behavior, not UI-only |
| `nav_side_probe.py` | `behavior/SideFeasibilityEvaluator` | **Refactor** — output → corridor, not hint |
| `nav_commitment.py` | `behavior/CommitmentTracker` | **Keep** — bind to corridor side |
| `nav_local_planner.py` | `planning/RollingLocalPlanner` | **Keep** — must consume corridor constraints |
| `mppi_controller.py` | `planning/MppiOptimizer` | **Constrain** — corridor-bounded sampling |
| `local_maneuver.py` | `behavior/LocalManeuverPolicy` | **Merge** into BehaviorManager decisions |
| `maneuver.py` | `behavior/ManeuverExecutor` | **Slim** — HOW not WHY |
| `nav_recovery.py` | `recovery/RecoveryPlanner` | **Extend** — MPPI_INFEASIBLE ladder |
| `sim_api_ext.apply_safety` | `safety/SafetySupervisor` | **Extract** — structured reason codes |
| `nav_observability.py` | `diagnostics/NavigationDiagnostics` | **Extend** — safe_vx_reason, footprint clearance |

**Forbidden in V0.2:** REVERSE_ARC auto-detour, 3G, pink-point behavior control.

---

## 4. Core Data Types

### 4.1 VehicleModel (`vehicle/model.py`)

```python
@dataclass(frozen=True)
class VehicleModelConfig:
    length_m: float          # SPEC: ~1.058
    width_m: float           # SPEC: ~0.550
    front_overhang_m: float  # MEASURED/CALIBRATED
    rear_overhang_m: float
    center_offset_x_m: float
    max_vx_mps: float        # CALIBRATED (not assumed 0.30)
    max_ax_mps2: float
    max_decel_mps2: float
    max_omega_rad_s: float
    max_alpha_rad_s2: float
    command_latency_s: float # CALIBRATION_REQUIRED
    perception_latency_s: float
    safety_margin_m: float
    track_width_m: Optional[float]  # UNKNOWN
    wheelbase_m: Optional[float]    # UNKNOWN
```

Sources: `nav_geometry.VehicleGeometry` today; extend, do not duplicate.

### 4.2 ExecutionCorridor

```python
@dataclass
class ExecutionCorridor:
    mode: Literal["FORWARD","LEFT","RIGHT","WAIT","NARROW_CENTER"]
    committed_side: Optional[Literal["LEFT","RIGHT"]]
    allowed_polygon: List[Point]      # world frame, inflated
    preferred_heading_rad: Optional[float]
    vx_min_mps: float
    vx_max_mps: float
    omega_min_rad_s: float
    omega_max_rad_s: float
    min_predicted_clearance_m: float  # hard constraint target
    global_path_relation: str         # FOLLOW / DETOUR / RECONNECT
    valid_until: float
    source: str                       # BEHAVIOR | RECOVERY
```

**Probe → Commit → Corridor** must be one pipeline. MPPI receives `ExecutionCorridor` and **rejects samples outside**.

### 4.3 SafetyDecision

```python
@dataclass
class SafetyDecision:
    approved_vx: float
    approved_omega: float
    veto: bool
    reason_code: str  # NORMAL | FRONT_CLEARANCE_VETO | MPPI_NO_FEASIBLE | ...
    inputs: dict      # front_clearance_footprint_m, braking_distance_m, ...
```

---

## 5. Behavior FSM (V0.2)

States inherit from audit + field failure lessons:

```text
INIT → READY → CRUISE
         ↓
    FUTURE_PREVIEW (slow, no commit)
         ↓
    STATIC_PROBE / DYNAMIC_CLASSIFY
         ↓
    LEFT_COMMIT | RIGHT_COMMIT
         ↓
    LOCAL_AVOID (corridor active)
         ↓
    OBSTACLE_PASS → GLOBAL_RECONNECT → CRUISE

Branches:
  LOCAL_PLAN_INFEASIBLE → REPROBE → REPLAN_CORRIDOR → ALTERNATE_SIDE → WAIT → SAFE_STOP
  NARROW_CORRIDOR → CENTERLINE | WALL_FOLLOW
  SENSOR_DEGRADED → speed cap
  NAVIGATION_FAILED (only after recovery budget exhausted)
```

Each state defines: **entry, exit, allowed command envelope, recovery transition**.

---

## 6. MPPI Role (Reduced)

MPPI in V0.2:

- **Does:** optimize trajectory inside `ExecutionCorridor` over 1–3 s horizon
- **Does not:** choose LEFT/RIGHT, handle dynamic wait, declare navigation failed

Constraints propagated into MPPI:

- `omega` sign locked when `corridor.mode == LEFT|RIGHT`
- `vx` capped by `corridor.vx_max` and kinematic `v_max(kappa)`
- Collision check uses **`footprint_collide`** on rollout (reuse `nav_footprint`)
- Candidate metadata: `candidate_count`, `valid_count`, `collision_rejected`, `constraint_rejected`

---

## 7. Trajectory Validator (Independent)

Post-MPPI gate (new module):

```text
selected trajectory
  → swept footprint over horizon (nav_footprint.trajectory_collision)
  → minimum predicted clearance
  → braking feasibility: stop_distance < available_clearance
  → corridor containment
→ VALID | INVALID + reason
```

INVALID → RecoveryPlanner, not immediate FAILED.

---

## 8. Safety Supervisor

Extract from `sim_api_ext.apply_safety`:

| Check | Input | Threshold source |
|-------|-------|------------------|
| Emergency | `emergency` flag | immediate |
| Footprint front clearance | predicted, not `front_near` ray | `BrakingModel` |
| Rear gate | `rear_clearance_footprint_m` | `rear_stop_m` + braking |
| Stale perception | sensor age | `sensors.yaml` |
| Planner invalid | validator result | recovery, not fail |

**Rename diagnostics:**

- `front_near` → `lidar_front_ray_min_m` (keep for sensor view)
- Add `footprint_front_clearance_m` for safety decisions

---

## 9. Recovery Ladder

```text
MPPI_INFEASIBLE | VALIDATOR_INVALID
    ↓ slow / hold
    ↓ recheck sensor freshness
    ↓ recompute footprint clearance
    ↓ REPROBE (keep commitment unless alternate provably better)
    ↓ REBUILD_CORRIDOR
    ↓ rerun MPPI
    ↓ success → RESUME
    ↓ fail → ALTERNATE_SIDE (if authorized)
    ↓ fail → DYNAMIC_WAIT (if dynamic) | GLOBAL_REPLAN (if static blocked)
    ↓ budget exhausted → SAFE_STOP
    ↓ persistent → NAVIGATION_FAILED
```

Static vs dynamic policy split preserved from `nav_dynamic_resume.py`.

---

## 10. Directory Layout (incremental)

```text
ros2_ws/src/agv_bridge/agv_bridge/
  navigation/
    vehicle/          # VehicleModel, BrakingModel, footprint
    world/            # WorldModel, obstacle layers
    behavior/         # BehaviorManager, AvoidancePhase, SideFeasibility
    planning/         # RollingLocalPlanner, MppiOptimizer
    validation/       # TrajectoryValidator, KinematicFeasibility
    safety/           # SafetySupervisor
    recovery/         # RecoveryPlanner
    diagnostics/      # structured logs, reason codes
  # compatibility shims re-export old imports during migration
```

Migrate without breaking `sim_api_ext` API surface initially.

---

## 11. Configuration

Centralize (new `config/` or yaml):

```text
vehicle.yaml      — dimensions, limits (tagged MEASURED/CALIBRATED/UNKNOWN)
navigation.yaml   — horizons, recovery budgets
safety.yaml       — margins, braking, timeouts
sensors.yaml      — lidar extrinsics, freshness
```

---

## 12. Testing (V0.2)

Mandatory scenarios:

- **SCENE-021 FIELD_P0D1_MPPIVX_ZERO** — heading vs footprint clearance divergence
- SCENE-003/004 Left/Right detour with corridor enforcement
- SCENE-014 MPPI infeasible → recovery before FAILED
- SCENE-008/009 Corner + narrow corridor + swept footprint
- P0-D A–L regression (must not break)

---

## 13. Migration Phases

| Phase | Deliverable |
|-------|-------------|
| M1 | VehicleModel + BrakingModel + footprint-only safety clearance API |
| M2 | ExecutionCorridor + BehaviorManager emits corridor |
| M3 | MPPI corridor constraints + footprint collision in MPPI |
| M4 | TrajectoryValidator + RecoveryPlanner for MPPI_INFEASIBLE |
| M5 | SafetySupervisor reason codes + diagnostics |
| M6 | Scenario regression + LIVE trace |
| M7 | Deprecate V0.1 parallel decision paths |

---

## 14. Known UNKNOWNs (require field data)

| Item | Status |
|------|--------|
| Real LiDAR A/B extrinsics | UNKNOWN (sim: ±0.35 m, π yaw rear) |
| wheelbase / track | UNKNOWN |
| command + perception latency | CALIBRATION_REQUIRED |
| max decel / stopping distance | DERIVED from `acc_v=0.9`, not measured |
| mass 320 kg dynamics | NOT IN CODE |
| Field failure JSONL | NOT IN REPO |

---

## 15. References (V0.1 evidence)

- Control ownership: `docs/CURRENT_CONTROL_OWNERSHIP_MAP.md`
- P0-D.1 report: `docs/PHASE4_P0D1_PREDICTIVE_AVOIDANCE_REPORT.md`
- P1-0 audit: `docs/PHASE4_P1_0_LOCAL_PLANNING_ARCHITECTURE_AUDIT.md`
- Physics loop: `sim_api_ext.py` → `LocalMppiModel.step` → `apply_safety`
- `front_near` definition: `sim_world.dual_lidar()` — 2nd-nearest ray in ±22° cone, meters
