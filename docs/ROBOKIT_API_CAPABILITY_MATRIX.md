# ROBOKIT API CAPABILITY MATRIX
# AGV Navigation V0.2 — M3.9.3
# Last updated: 2026-08-18

## Protocol Architecture

```
TCP Port  |  Purpose           |  APIs (API ID)
----------|--------------------|--------------------------------------------------
19204     |  STATUS (read)     |  1004 loc, 1005 speed, 1006 block, 1007 battery,
          |                    |  1009 laser, 1012 emergency, 1020 task_status,
          |                    |  1021 reloc_status, 1060 control_owner, 1100 batch,
          |                    |  1300 map, 1301 station, 1303 path_query
19205     |  CONTROL (write)   |  2002 reloc, 2003 confirm_loc, 2004 cancel_reloc,
          |                    |  2010 velocity (continuous), 2022 switch_map,
          |                    |  2025 upload_switch, 4010 upload_map
19206     |  NAVIGATION        |  3001 pause_nav, 3002 resume_nav, 3003 cancel_nav,
          |                    |  3051 goto_station
19207     |  CONFIG            |  4005 lock, 4006 unlock, 4011 download_map
19210     |  OTHER             |  —
19301     |  PUSH              |  9300 push_config
```

**CRITICAL**: API ID ≠ TCP Port. API ID is a field in the 16-byte binary header.
Transport port is the TCP connection port. These are completely distinct concepts.

---

## API Entries

### API 1004 — get_pose (loc)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204                           |
| Semantic        | Read robot current pose         |
| Request Payload | `{}` (empty)                    |
| Response Fields | `x, y, angle, confidence`       |
| Current Code    | `robokit_client.get_pose()`     |
| Status          | **CONFIRMED** (real-vehicle tested 2026-07-28) |
| Risk            | LOW                             |

### API 1005 — get_speed
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204                           |
| Semantic        | Read actual + commanded velocity |
| Request Payload | `{}` (empty)                    |
| Response Fields | `vx, vy, w, r_vx, r_vy, r_w, is_stop` |
| Current Code    | `robokit_client.get_speed()`    |
| Status          | **CONFIRMED** (real-vehicle tested) |
| Risk            | LOW                             |
| Note            | `r_vx=0` with `vx>0` → chassis not executing command (P0 diagnostic) |

### API 1006 — get_block_status
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204                           |
| Semantic        | Query AGV block/path-blocked state |
| Request Payload | `{}` (empty)                    |
| Current Code    | `robokit_client.get_block_status()` |
| Status          | **UNVERIFIED** — code present, no confirmed real-vehicle test |
| Risk            | MEDIUM                          |

### API 1007 — get_battery
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204                           |
| Semantic        | Read battery level and charging state |
| Response Fields | `battery_level (0–100 or 0–1), charging` |
| Current Code    | `robokit_client.get_battery()`  |
| Status          | **CONFIRMED**                   |
| Risk            | LOW                             |

### API 1011 — DEPRECATED / WRONG EMERGENCY API
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204 (assumed)                 |
| Semantic        | **WRONG** — used in tcp_client.py (agv_control) as emergency |
| Current Code    | `tcp_client.py: API_EMERGENCY = 1011` |
| Status          | **CRITICAL ERROR** — must be 1012, not 1011 |
| Risk            | **P0 CRITICAL** — real vehicle will not respond correctly |
| Action          | Replace with 1012 in tcp_client.py immediately |

### API 1012 — get_emergency (CORRECT)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19204                           |
| Semantic        | Read emergency stop status      |
| Request Payload | `{}` (empty)                    |
| Response Fields | `emergency` (bool), `soft_emc` (bool) |
| Current Code    | `robokit_client.py: API_EMERGENCY = 1012` ✓ |
| Status          | **CONFIRMED** (real-vehicle tested) |
| Risk            | P0 if wrong API used            |
| Note            | tcp_client.py incorrectly uses 1011 — must fix |

### API 2000 — SOFT_STOP (status: UNVERIFIED)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19205 (assumed)                 |
| Semantic        | Soft stop command               |
| Current Code    | `tcp_client.py: API_SOFT_STOP = 2000` |
| Status          | **UNVERIFIED** — real vehicle behaviour unknown |
| Risk            | **HIGH** — if 2000 fails, current code falls back to `translate(0,0,0)` via 3055. This is FORBIDDEN |
| Action          | Confirmed fallback must be STOP_FAILED state, NOT zero-velocity 3055 |

### API 2010 — set_velocity / Continuous Velocity Control
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19205                           |
| Semantic        | Continuous velocity streaming (vx, vy, w) until new cmd or stop |
| Request Payload | `{vx: float, vy: float, w: float, duration: int}` |
| Current Code    | **MISSING** in tcp_client.py (agv_control) |
|                 | **NOT in robokit_client.py** (needs to be added) |
| Status          | **UNVERIFIED** — API ID confirmed from project spec, real behavior not tested |
| Risk            | **HIGH** — Navigation continuous control depends on this |
| Note            | `duration=0` per spec means: vehicle executes until new command. Must be verified |
| Action          | Add `set_velocity_cmd()` to both `tcp_client.py` and future `robokit_client.py` |

### API 2002/2003/2004 — Relocation
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19205                           |
| Semantic        | Trigger/confirm/cancel relocation |
| Current Code    | `robokit_client.py: relocate(), confirm_loc(), cancel_reloc()` |
| Status          | **CONFIRMED** (tested in production) |
| Risk            | LOW (not in critical motion path) |

### API 3051 — goto_station
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19206                           |
| Semantic        | Navigate to target station ID   |
| Request Payload | `{"id": target_id}` — NOTE: source_id must NOT be sent (real vehicle rejects) |
| Current Code    | `robokit_client.goto_station()`  |
| Status          | **CONFIRMED** — real vehicle tested 2026-07-28; source_id must be omitted |
| Risk            | LOW                             |

### API 3055 — Translate Motion Task (NOT velocity streaming)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19205                           |
| Semantic        | **Motion task: translate by distance at velocity** |
| Request Payload | `{"dist": float, "vx": float, "mode": int}` |
| **FORBIDDEN**   | `{"vx": float, "vy": float, "w": float}` — velocity command — WRONG semantics |
| Current Code (agv_control) | `tcp_client.translate(vx, vy, w)` → sends `{vx, vy, w}` → **P0 WRONG** |
| Current Code (agv_bridge) | NOT USED (correct — agv_bridge uses robokit_client without 3055) |
| Status          | **CRITICAL CONTRACT VIOLATION** in agv_control/tcp_client.py |
| Risk            | **P0** — will not work correctly on real vehicle, may cause unexpected motion |
| Action          | Rename to `translate_distance(dist, vx, mode)`, fix payload |

### API 3056 — Rotate Motion Task
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19205                           |
| Semantic        | **Motion task: rotate by angle at angular velocity** |
| Request Payload | `{"angle": float, "vw": float, "mode": int}` |
| **FORBIDDEN**   | `{"w": float}` only — missing required fields |
| Current Code    | `tcp_client.rotate(w)` → sends `{w: float}` → **P0 WRONG** |
| Status          | **CRITICAL CONTRACT VIOLATION** |
| Risk            | **P0** — payload missing required fields |
| Action          | Rename to `rotate_angle(angle, vw, mode)`, fix payload |

### API 4005 — Lock (request control)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19207                           |
| Semantic        | Request exclusive control authority |
| Request Payload | `{"nick_name": str}`            |
| Response        | `ret_code == 0` means granted   |
| Current Code    | `tcp_client.lock()`, `robokit_client.lock()` |
| Status          | **CONFIRMED**                   |
| Risk            | MEDIUM — must verify lock before any motion |

### API 4006 — Unlock (release control)
| Field           | Value                           |
|-----------------|---------------------------------|
| Transport Port  | 19207                           |
| Semantic        | Release control authority       |
| Current Code    | `tcp_client.unlock()`, `robokit_client.unlock()` |
| Status          | **CONFIRMED**                   |
| Risk            | LOW                             |

---

## Summary: Critical Issues Found (M3.9.3 Audit)

| # | Severity | Issue | File | Action |
|---|----------|-------|------|--------|
| 1 | **P0** | Emergency API uses 1011 (wrong), must be 1012 | `tcp_client.py` | Fix immediately |
| 2 | **P0** | `translate(vx,vy,w)` sends velocity as 3055 — wrong semantics | `tcp_client.py` | Rename + fix payload |
| 3 | **P0** | `rotate(w)` sends only `{w}` for 3056 — missing dist/angle/mode | `tcp_client.py` | Rename + fix payload |
| 4 | **P0** | `stop()` falls back to `translate(0,0,0)` via 3055 — forbidden | `tcp_client.py` | Remove fallback, use STOP_FAILED state |
| 5 | **P0** | 2010 velocity API missing — Navigation has no correct continuous velocity path | `tcp_client.py` | Add `set_velocity_cmd()` |
| 6 | **P0** | `RealBackend.__init__` defaults to `192.168.192.5` even when not in real mode | `real.py` | Require explicit host in real mode |
| 7 | **P0** | `set_velocity()` in RealBackend calls `translate(vx, vy, w)` → wrong 3055 payload | `real.py` | Switch to `set_velocity_cmd()` → 2010 |
| 8 | **HIGH** | factory.py `tcp` mode silently defaults to 127.0.0.1 — no block for real mode | `factory.py` | Block without explicit AGV_HOST |
| 9 | **HIGH** | 2000 soft-stop unverified but code uses it with wrong fallback | `tcp_client.py` | Mark UNVERIFIED, remove bad fallback |
| 10 | **MED** | No telemetry on API call latency / errors | `tcp_client.py` | Add timing instrumentation |

---

## NAVIGATION_CONTROL_MODE (to be decided before field deployment)

```
VELOCITY_2010 (continuous):
  Navigation vx/omega → VelocityCommand → Safety → 2010
  Status: API contract UNVERIFIED on real vehicle
  Decision required before PHYSICAL_TEST_ALLOWED = YES

MOTION_PRIMITIVE (task-based):
  Navigation vx/omega → MotionPrimitive converter → 3055/3056
  Status: Requires separate V0.2.1 MOTION_PRIMITIVE_CONTROLLER
  NOT available in M3.9.3
```

---

## PHYSICAL_TEST_ALLOWED Assessment (M3.9.3)

```
REAL_BACKEND_CONTRACT  = FAIL (pre-M3.9.3 implementation)
                       = PARTIAL (after M3.9.3: code fixed, 2010 unverified on real vehicle)
PHYSICAL_TEST_ALLOWED  = NO
RC1_READY              = NO (emergency API unverified on real hardware)
REAL_BACKEND_READY     = NO (2010 velocity path unverified)
```
