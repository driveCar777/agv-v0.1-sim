# PHASE4 P0-D.1 — Predictive Avoidance / Side Probe / Dynamic Resume Report

日期：2026-08-17  
基线：`3d828d1` / P0-D OFFLINE PASS  
范围：**P0-D.1 phased avoidance + side probe + soft slowdown + dynamic resume forensics**

```text
P0-D.1 OFFLINE = PASS (A–F)
P0-D regression  = PASS (A–L)
P1-1 regression  = PASS
P0-D.1 LIVE      = PARTIAL (trace scripts ready; sim manual obstacles)
REVERSE_ARC      = NOT IMPLEMENTED
Pink point       = DISPLAY-ONLY (1.4m unchanged)
3G               = FORBIDDEN
NEXT             = P0-E / P1-2 (STOP)
```

---

## 1. Future Preview 是否真的 5m 有效？

**YES for DETECTION, NO for immediate maneuver.**

| Tier | Typical @ v=0.20 | Meaning |
|------|------------------|---------|
| `d_detection_m` | ~5.0m | Know obstacle exists |
| `d_probe_start_m` | ~3.3m | Start virtual side probe + soft slowdown |
| `d_commit_m` | ~2.5m | Maneuver-ready / commit threshold |
| `d_hard_stop_m` | **0.70m** | Safety (unchanged) |

Audit B: obstacle @5m → `future_collision=true`, phase=`FUTURE_PREVIEW`, **not** `SIDE_COMMIT`.

---

## 2. 为什么以前探到了却不提前动作？

P0-D 把 **detection** 和 **action** 绑在同一阈值 (`first_collision < required_avoidance` ≈ 2.5m)。

P0-D.1 分层：

```text
5m  → DETECTED / FUTURE_PREVIEW (soft slowdown only)
~3m → SIDE_PROBE (virtual L/R evaluation, vehicle may still FORWARD)
~2.5m → OBSTACLE_APPROACH / commit-ready
commit → SIDE_COMMIT → LOCAL_AVOID (3C/3E)
```

---

## 3. Probe/Approach 分层

```text
OPEN → FUTURE_PREVIEW → OBSTACLE_APPROACH → SIDE_PROBE → SIDE_COMMIT
     → OBSTACLE_PASS → GLOBAL_RECONNECT → FOLLOW_GLOBAL
```

Signals: `DETECTED | PREDICTED | WARNING | MANEUVER_READY | COMMITTED`

---

## 4. 提前减速多少？

| Phase | target_vx (from 0.30 cruise) | speed_reason |
|-------|------------------------------|--------------|
| OPEN | 0.30 | NORMAL_CRUISE |
| FUTURE_PREVIEW | ~0.28 | FUTURE_OBSTACLE_PREVIEW |
| SIDE_PROBE | ~0.25 | SIDE_PROBE |
| SIDE_COMMIT | ~0.23 | SIDE_COMMIT |
| Dynamic resume | 0.15→0.20→0.25→0.30 ramp | DYNAMIC_RESUME |

**Not** 0.30→0.10 cliff. MPPI/Safety/Physics still gate actual vx.

---

## 5. LEFT/RIGHT 如何评分？

`nav_side_probe.py`: κ ∈ {0.25, 0.45, 0.65, 0.85} × LEFT/RIGHT, full preview horizon swept.

Confidence = f(clearance, reconnect, progress, future_blocked).

Cost priority:

```text
COLLISION > CLEARANCE > KINEMATIC > PASSABILITY > COMMITMENT > PROGRESS > GLOBAL_RECONNECT
```

---

## 6. 为什么不会贴墙？

Nonlinear `clearance_cost` + `WALL_HUGGING_SUSPECTED` diagnostic. Hard invalid when clearance < `safety_margin_m`.

---

## 7. 为什么不会频繁左右摆？

- Side probe is **virtual** (no physical L/R thrash before commit)
- `COMMIT_HOLD_S` hysteresis
- `SIDE_PROBE_OSCILLATION` event if side history flips ≥3 in window
- 3C/3E commitment reused (no second lock system)

---

## 8. 为什么不会过早 reconnect?

`global_reconnect_blocked=true` until `obstacle_passed`. Local planner FORWARD hard-invalid + reconnect_cost ×3.6 when gate active. `EARLY_GLOBAL_RECONNECT` event if violated.

---

## 9. Corner 怎么处理？

Multi-κ arcs evaluate corner cut feasibility over full horizon; future_blocked if arc clears first 1m but blocks later. Corner-specific metadata = **PARTIAL** (no dedicated corner detector yet).

---

## 10. 移动障碍离开后为什么恢复？

`nav_dynamic_resume.py`:

- Tracks `dynamic_short/long` transitions
- Emits `ENTER_DYNAMIC_WAIT`, `DYNAMIC_CLEAR`, `DYNAMIC_RESUME`
- Policy: `DYNAMIC_CLEAR→RESUME` when clearance restored
- Maneuver FSM: `WAIT_FOR_CLEARANCE` → `FORWARD_TRACK` on `dynamic_resume_clear`
- Gradual speed ramp 0.15→0.30

---

## 11. 为什么以前停住后不恢复？

Diagnosed causes A–J in `diagnose_resume_block()`:

| Code | Cause |
|------|-------|
| A | WAIT_FOR_CLEARANCE stuck after dynamic clear |
| B–C | REPLAN/RECOVERY loop |
| D | Safety zero |
| E | forward_feasible false |
| F | local plan stale |
| H | oscillation loop |
| J | FSM TURN/REPOSITION |

---

## 12. Breadcrumb Historical Retreat?

**YES — unchanged.** Breadcrumb reset on replan still recorded; not used as forward planner input.

---

## 13. REVERSE_ARC?

**NOT IMPLEMENTED**

---

## 14. 3F?

P1-1 / 3D / 3E probe regressions **PASS**. 3F corrective LIVE **NOT RE-RUN** this session.

---

## Files

| File | Role |
|------|------|
| `nav_side_probe.py` | **NEW** multi-κ virtual side probe |
| `nav_avoidance_phase.py` | **NEW** phased state machine + tier distances |
| `nav_dynamic_resume.py` | **NEW** dynamic wait/resume + diagnostics |
| `nav_speed_policy.py` | Soft slowdown + speed_reason |
| `nav_policy.py` | FUTURE_PREVIEW, SIDE_PROBE, dynamic resume |
| `nav_local_planner.py` | Commit-gated side select, clearance cost |
| `nav_models.py` | Wire trackers |
| `maneuver.py` | DYNAMIC_RESUME FSM exit |
| `sim_api_ext.py` / `nav_observability.py` | API + summary fields |
| `sim3d.js` | D/P/S tier markers |
| `scripts/_audit_phase4_predictive_avoidance.py` | **NEW** |
| `scripts/_trace_phase4_dynamic_obstacle_live.py` | **NEW** |

---

## STOP

P0-D.1 complete. Do **NOT** enter P0-E / P1-2 / 3G without explicit instruction.
