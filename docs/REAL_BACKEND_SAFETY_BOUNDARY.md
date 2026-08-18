# REAL BACKEND SAFETY BOUNDARY
# AGV Navigation V0.2 — M3.9.3
# Last updated: 2026-08-18

## Safety State Machine

```
DISCONNECTED
    │ connect() → ping AGV_HOST:19204
    ▼
CONNECTED
    │ request_control() → lock (4005)
    ▼
CONTROL_GRANTED
    │ set_velocity() / stop()
    ▼
MOVING / STOPPED
    │ stop() fails → STOP_FAILED
    ▼
STOP_FAILED (motion locked out)
    │ clear_stop_failed() — operator only
    ▼
STOPPED (confirmed)
```

## Hard Rules (M3.9.3)

| Rule | Description | Enforcement |
|------|-------------|-------------|
| R1 | No default AGV IP in real mode | `ValueError` on init if missing host |
| R2 | REAL_ROBOT_ENABLED gate | `RuntimeError` from factory |
| R3 | AGV_HOST required for real/tcp mode | `RuntimeError` from factory |
| R4 | STOP_FAILED blocks all motion | `set_velocity()` returns immediately |
| R5 | stop() STOP_FAILED = no 3055 fallback | No fallback code path exists |
| R6 | Emergency flag blocks motion | `set_velocity()` checks `_emergency` |
| R7 | has_control required for motion | `set_velocity()` checks `_has_control` |
| R8 | Emergency API = 1012 only | Static constant, unit tested |
| R9 | Velocity API = 2010 only | No 3055 in velocity path |
| R10 | Mock backend is default | No env variable = mock |

## What M3.9.3 Does NOT Guarantee

- API 2010 behavior on real vehicle (UNVERIFIED)
- Stop reliability (2000 UNVERIFIED, no secondary stop path confirmed)
- Emergency API 1012 behavior on real hardware (code confirmed, hw unverified)
- Navigation safety (separate from backend contract)
- Collision avoidance (separate navigation concern)

## PHYSICAL_TEST_ALLOWED Checklist

Before setting PHYSICAL_TEST_ALLOWED = YES, all must be confirmed:

- [ ] API 2010: real vehicle accepts payload {vx, vy, w, duration=0}
- [ ] API 2010: vehicle executes velocity until new command received
- [ ] API 2000: stop works reliably (or secondary stop path confirmed)
- [ ] API 1012: emergency state correctly read from vehicle
- [ ] API 4005/4006: lock/unlock confirmed operational
- [ ] Navigation M3.9.2: planner compute p95 < 150ms on target hardware
- [ ] Watchdog: controller watchdog active and tested
- [ ] Emergency button: hardware E-stop tested independently
- [ ] Observer: qualified person present during all physical tests
- [ ] Environment: cleared test area, no bystanders

## Operator Guide for STOP_FAILED

If the backend enters STOP_FAILED state:

1. Verify vehicle has physically stopped (visual confirmation)
2. If vehicle still moving: use hardware E-stop button
3. Do NOT call `clear_stop_failed()` until vehicle is confirmed stopped
4. Investigate root cause of 2000 failure before resuming
5. If 2000 continues to fail: vehicle API contract must be re-investigated

## API Contract Version History

| Version | Change |
|---------|--------|
| pre-M3.9.3 | Emergency = 1011 (WRONG), translate(vx,vy,w) → 3055 (WRONG), stop fallback to 3055 (WRONG), default IP 192.168.192.5 (WRONG) |
| M3.9.3 | Emergency = 1012 (CORRECT), set_velocity_cmd(vx,vy,w) → 2010 (CORRECT), stop STOP_FAILED no fallback (CORRECT), no default IP (CORRECT) |
