# PHASE 4 — Navigation Log / Diagnostic API (P0-B-0 + P0-B)

基线：`71c466b`  
状态：**FROZEN for future Debug UI**（P0-B 仅追加字段 / 事件，不改 envelope）  
Internal nav events ≠ HTTP access logs.

---

## Base

Web sim: `http://127.0.0.1:19999`

Aliases: `/api/logs/...` and `/api/nav/logs/...` are equivalent.

Default `limit <= 200`. Use `since` / `cursor` for incremental fetch.

Preview (read-only): `GET /api/nav/preview` → `global_reference` + `local_candidates` + `selected_local` + `global_vs_local` + `kinematic_validation` + `open_space_forensics`.

Forensics (read-only): `GET /api/nav/forensics/open-space` → full P0-C.1 / P1-0 open-space local planning forensics blob (`controls_vehicle=false`).

P1-0 diagnostic extras (assemble-only): `mppi.speed_target` (soft 0.22), `diagnostics.local_selector_role_in_open`, `diagnostics.open_space_cruise_speed`, `diagnostics.planned_distance_m` / `rendered_distance_m` / `executed_distance_m`. See `docs/PHASE4_P1_0_LOCAL_PLANNING_ARCHITECTURE_AUDIT.md`.

Env: `NAV_GLOBAL_PREVIEW=0` disables Global Preview build (telemetry empty / DISABLED); control unchanged.

Env: `NAV_KINEMATIC_VALIDATOR=0` disables P0-C validator (`kinematic_valid=null`, `status=NOT_VALIDATED`); control unchanged.

---

## GET `/api/logs`

Filter query params:

| Param | Notes |
|-------|--------|
| `level` | exact, or `WARN+` for WARN and above |
| `category` | `SAFETY`, `CANDIDATE`, `DIAGNOSTIC`, `GLOBAL_PLANNING`, `KINEMATIC`, … |
| `event` | e.g. `SPIN_LOOP_SUSPECTED`, `GLOBAL_PREVIEW_UPDATED` |
| `source` / `component` | |
| `trace_id` / `cycle_id` | |
| `from_ts` / `to_ts` | unix seconds |
| `since` | `event_id` (exclusive) **or** timestamp |
| `focus` | `STOP` / `SPIN` / `RECOVERY` / `PLANNING` / `SAFETY` |
| `limit` | default 200, max 200 |
| `cursor` | last `event_id` (exclusive) |
| `include_api` | `true` to mix HTTP access logs (default **false**) |

Response:

```json
{
  "success": true,
  "events": [],
  "count": 0,
  "has_more": false,
  "next_cursor": "EVT-000012",
  "limit": 200
}
```

Event schema (minimum):

```text
ts, event_id, cycle_id, trace_id, level, category, event, source, component
reason?, message?, data?, data_ref?
```

---

## GET `/api/logs/summary`

Dashboard payload: mode, policy, candidate counts, requested/safe/state vx-w, last warn/error, `stop_reason`, `likely_owner`, **`immediate_owner`**, **`likely_root_owner`** (null if unknown), spin/deadlock/starvation/mismatch flags, `logging_overhead_ms_ema`.

**P0-B fields:**

```text
global_preview_m
global_remaining_m
global_preview_reason   # NORMAL_5M | GOAL_LIMITED | PATH_LIMITED
global_path_revision
local_max_distance_m
expected_nominal_distance_m   # dynamic: speed × local horizon (≈1.5s)
legacy_nominal_baseline_m     # historical 0.25m (not starvation truth)
global_vs_local { global_preview_m, local_max_distance_m, selected_candidate, expected_local_distance_m }
```

**P0-C fields:**

```text
kinematic_status            # VALID | INVALID | DEGRADED | NOT_VALIDATED
kinematic_valid             # true | false | null
max_curvature
min_turn_radius_m
first_invalid_distance_m
speed_limited
validation_revision
cache_hit
compute_ms
```

Owner semantics:

```text
likely_owner      = immediate blocking owner (backward compatible)
immediate_owner   = same as likely_owner
likely_root_owner = immediate_owner, or null if UNKNOWN/NONE
```

---

## GET `/api/logs/trace/<trace_id>`

```json
{ "success": true, "trace_id": "...", "metadata": {}, "summary": {}, "outcome": {}, "events": [] }
```

Lifecycle events: `TRACE_START` / `TRACE_END` / `TRACE_UPDATE` / `TRACE_ABORT`.

---

## GET `/api/logs/cycle/<cycle_id>`

Returns the `NavDecisionTrace` for that cycle (`observe…outcome`).

`decision.global` includes preview fields; `decision.diagnostics` includes global/local comparison + dynamic expected distance.

`cycle_semantics` = `diagnostic_control_refresh_cycle` (~20Hz debug refresh; not a planner decision gate).

---

## GET `/api/logs/diagnostics?window_s=10`

Last N seconds (default **10**, clamp 1–60) of events + compact cycle rows.

---

## GET `/api/logs/api`

**Separated** HTTP access log. Polling `/api/state` is sampled (~1 Hz), never full body.

4xx / 5xx / slow (≥100 ms) always retained.

Sensitive keys redacted as `***REDACTED***`.

---

## POST `/api/logs/config`

```json
{ "level": "DEBUG", "candidate_detail": true, "enabled": true, "sample_hz": 4 }
```

Env defaults: `NAV_LOG_LEVEL`, `NAV_LOG_RING_SIZE`, `NAV_LOG_FILE`, `NAV_LOG_CANDIDATE_DETAIL`, `NAV_LOG_API_BODY`, `NAV_LOG_SAMPLE_HZ`.

---

## IDs

```text
event_id  EVT-000001
cycle_id  NAV-000123     # one physics debug refresh
trace_id  TRACE-AVOID-000017 / TRACE-RECOVERY-000004 / TRACE-REPLAN-000012
```

---

## Critical events (always retained)

`GOAL_REACHED`, `NO_CANDIDATE`, `ALL_CANDIDATES_INVALID`, `CANDIDATE_SELECTED`, `SAFETY_BLOCK`, `SAFETY_CLAMP`, `REPLAN`, `REPLAN_FAILED`, `RECOVERY_ENTER`, `RECOVERY_EXECUTION`, `RECOVERY_EXIT`, `NAVIGATION_STOP_DIAGNOSTIC`, `NAVIGATION_DEADLOCK_SUSPECTED`, `SPIN_LOOP_SUSPECTED`, `PLANNING_STARVATION_SUSPECTED`, `PLAN_EXECUTION_MISMATCH`, `NAV_CYCLE_OVERRUN`,

**P0-B:** `GLOBAL_PREVIEW_UPDATED`, `GLOBAL_PREVIEW_LIMITED`, `GLOBAL_LOCAL_HORIZON_MISMATCH`, `TRACE_START`, `TRACE_END`, `TRACE_UPDATE`, `TRACE_ABORT`.

**P0-C:** `KINEMATIC_VALIDATION_STARTED`, `KINEMATIC_VALIDATION_RESULT`, `KINEMATIC_PATH_REJECTED`, `KINEMATIC_CLEARANCE_WARNING`, `KINEMATIC_SPEED_LIMITED`.

### Global Preview events

| Event | When |
|-------|------|
| `GLOBAL_PREVIEW_UPDATED` | Meaningful preview change (≥0.4 m / reason / path_revision) — not every 20 Hz tick |
| `GLOBAL_PREVIEW_LIMITED` | Entering `GOAL_LIMITED` or `PATH_LIMITED` |
| `GLOBAL_LOCAL_HORIZON_MISMATCH` | `global_preview_m / local_max ≥ ~8` — **fact only**, not a bug verdict (P1 may revisit local horizon) |

### Kinematic validation events (P0-C)

Emitted on `validation_id` / status change only (not 20 Hz). `focus=PLANNING` includes these. `category=KINEMATIC`.

| Event | When |
|-------|------|
| `KINEMATIC_VALIDATION_STARTED` | New validation_id |
| `KINEMATIC_VALIDATION_RESULT` | Result attached (`VALID` / `INVALID` / `DEGRADED`) |
| `KINEMATIC_PATH_REJECTED` | `INVALID` and reason ≠ `NO_PATH` |
| `KINEMATIC_CLEARANCE_WARNING` | Clearance below `safety_margin_m` (`DEGRADED`) |
| `KINEMATIC_SPEED_LIMITED` | Feasible `v = ω_max/|κ|` below `v_max` (still may be `VALID`) |

Validator does **not** emit cmd_vel and does **not** SAFE_STOP on INVALID.

### Planning starvation (dynamic)

```text
expected_nominal_distance_m = effective_speed × 1.5s
starve if max_candidate_distance < max(0.5 × expected, 0.08m) sustained ≥2s
```

Legacy `0.25m` remains as `legacy_nominal_baseline_m` only.
