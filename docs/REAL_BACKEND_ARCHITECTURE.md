# REAL BACKEND ARCHITECTURE
# AGV Navigation V0.2 — M3.9.3
# Last updated: 2026-08-18

## Overview

The AGV control system uses a layered backend architecture with strict mode isolation:

```
Navigation / MPPI
    │ vx, omega (continuous)
    ▼
set_velocity(Twist2D)
    │
    ▼
RealBackend.set_velocity()
    │ VelocityCommand {vx, vy, w, duration}
    ▼
TcpApiClient.set_velocity_cmd()
    │ API 2010 on TCP port 19205
    ▼
Robokit Vehicle
```

## Backend Modes

| Mode | Class | Default | Real Vehicle |
|------|-------|---------|-------------|
| `mock` | MockBackend | ✓ | No |
| `sim` | MockBackend | — | No |
| `tcp`/`api` | RealBackend | — | No (Mock server) |
| `real` | RealBackend | — | Yes (requires gates) |

## Mode Activation Gates

### real mode — ALL must be satisfied:
1. `AGV_BACKEND=real` or explicit `create_backend("real")`
2. `REAL_ROBOT_ENABLED=true` — explicit opt-in
3. `AGV_HOST=<vehicle-ip>` — no default IP allowed
4. API profile verified (2010 velocity unverified as of M3.9.3)

### tcp mode — for Mock server integration testing:
1. `AGV_HOST=127.0.0.1` (or target) — must be explicit
2. No `REAL_ROBOT_ENABLED` required
3. NOT for production vehicle use

## Navigation Control Mode (M3.9.3)

```
NAVIGATION_CONTROL_MODE = VELOCITY_2010

Navigation generates: vx, omega (continuous streaming at 20Hz)
  → set_velocity(Twist2D) → API 2010 (continuous velocity)

NOT using 3055/3056 (motion tasks):
  3055 = "move forward X meters at V m/s" → requires MotionPrimitive layer
  3056 = "rotate Y degrees at W rad/s" → requires MotionPrimitive layer
  These are incompatible with continuous velocity control.

MOTION_PRIMITIVE_CONTROLLER (3055/3056 based):
  Status: NOT implemented in V0.2
  Planned: V0.2.1 if field requires task-based API
```

## API Contract (M3.9.3)

### set_velocity_cmd (API 2010) — Continuous Velocity
```
Port: 19205 (CONTROL)
Payload: {"vx": float, "vy": float, "w": float, "duration": int}
duration=0: hold velocity until next command (UNVERIFIED on real vehicle)
Status: UNVERIFIED — code correct, real vehicle behavior not confirmed
```

### translate_distance (API 3055) — Distance Task
```
Port: 19205 (CONTROL)
Payload: {"dist": float, "vx": float, "mode": int}
Semantic: move dist meters at vx m/s
NOT for velocity streaming.
```

### rotate_angle (API 3056) — Angle Task
```
Port: 19205 (CONTROL)
Payload: {"angle": float, "vw": float, "mode": int}
Semantic: rotate angle radians at vw rad/s
NOT for angular velocity streaming.
```

### stop (API 2000) — Soft Stop
```
Port: 19205 (CONTROL)
Payload: {} (empty)
On failure: STOP_FAILED — caller must enter safe state
NO fallback to 3055 zero-velocity — forbidden by M3.9.3
Status: UNVERIFIED on real vehicle
```

### Emergency state (API 1012) — Read Only
```
Port: 19204 (STATUS)
Payload: {} (empty)
Response: {"emergency": bool, "soft_emc": bool}
Note: 1012, NOT 1011 (1011 was a pre-M3.9.3 bug)
Status: CONFIRMED via robokit_client.py
```

## STOP_FAILED Handling

If `stop()` returns `STOP_FAILED`:
1. `RealBackend._stop_failed` is set to `True`
2. All subsequent `set_velocity()` calls are blocked
3. Safety layer must escalate (E-stop, operator intervention)
4. `clear_stop_failed()` requires operator confirmation

This prevents continued motion after a failed stop — no unsafe "pretend stopped" state.

## Telemetry

All motion API calls emit telemetry:
```python
{
    "api_id": int,          # API type ID
    "tcp_port": int,        # TCP port used
    "request": dict,        # payload sent
    "request_ts": float,    # Unix timestamp
    "response_ts": float,   # Unix timestamp
    "latency_ms": float,    # round-trip time
    "result": str,          # "ok" | "error" | "exception"
    "error": str | None,    # error message if failed
    "command_source": str,  # "nav" | "teleop" | ...
    "command_age_ms": float,# age of command at send time
    "sequence": int,        # monotonic sequence number
}
```

## Release Gates (M3.9.3 Output)

```
REAL_BACKEND_CONTRACT  = PASS (code contract corrected)
PHYSICAL_TEST_ALLOWED  = NO  (2010 unverified on real vehicle)
RC1_READY              = NO  (emergency API 1012 unverified on real hw)
REAL_BACKEND_READY     = NO  (pending 2010 real-vehicle field test)
```

## Pre-conditions for PHYSICAL_TEST_ALLOWED = YES

1. API 2010 confirmed operational on real vehicle with correct payload
2. API 1012 emergency query confirmed on real vehicle
3. API 2000 stop behavior confirmed (or secondary stop path established)
4. Lock/unlock (4005/4006) confirmed operational
5. Navigation M3.9.2 planner realtime PASS (< 300ms max compute)
6. Full regression on Mock server passes
7. Safety watchdog active during field test
