# PHASE 4 STEP 3F-CORRECTIVE-A — Recovery Execution Audit (Read-Only)

日期：2026-08-16  
状态：**AUDIT COMPLETE — execution FAIL diagnosed**  
范围：只读；尚未改码前的因果钉死。

---

## 0. Verdict

| Question | Answer |
|----------|--------|
| Decision ladder `LOCAL_REVERSE`? | **Works** (`nav_recovery.evaluate_recovery`) |
| Does that become `mode=REVERSE_ESCAPE`? | **Usually NO** — FSM prefers TURN/REPOSITION |
| Who should emit negative vx? | `DiffDriveMppi.step(force_reverse=True)` or `force_vx<0` under `REVERSE_ESCAPE` |
| Why `vx≈0, w≠0`? | Mode=`ALIGN`/`TURN_IN_PLACE`/`REPOSITION` → `force_vx=0` (or ±0.06) + `force_w≠0` |
| Safety blocking reverse? | **Often never asked** — reverse command never issued |

```text
3F DECISION = PASS (ladder)
3F EXECUTION = FAIL (FSM/controller ownership gap)
```

---

## 1. Intended chain (who converts LOCAL_REVERSE → −vx)

```text
evaluate_recovery()                 # nav_recovery.py
  action=LOCAL_REVERSE, allow_recovery=True
        ↓
LocalMppiModel.step                 # nav_models.py
  policy_ctx.allow_recovery=True
  policy_ctx.recovery_action=LOCAL_REVERSE
        ↓
ManeuverFSM.decide(...)             # maneuver.py  ★ BREAK HERE
  SHOULD set mode=REVERSE_ESCAPE
  ACTUALLY often TURN_IN_PLACE / REPOSITION / ALIGN
        ↓
DiffDriveMppi.step(
  force_reverse=want_rev,           # only True if mode=REVERSE_ESCAPE
                                    #   OR recovery wants reverse AND mode∉{ALIGN,TURN,...}
  force_vx, force_w, maneuver_mode
)
        ↓
apply_safety(...)                   # sim_api_ext.py
        ↓
state.vx / pose integrate
```

---

## 2. Exact break points

### Break A — FSM never forced to REVERSE_ESCAPE

`maneuver.py` reverse only when:

```text
front_near < front_stop AND not rot_safe AND allow_recovery
→ DEAD_END_REVERSE
```

If `rot_safe=True` (common with rear open):

```text
abs_h > H_ALIGN → TURN_IN_PLACE / ALIGN   (HEADING_ALIGN)
stuck → REPOSITION                         (STUCK→REPOSITION)
```

**`recovery_action=LOCAL_REVERSE` is NOT consumed by ManeuverFSM.**  
Only `allow_recovery` boolean is passed — no forced mode.

### Break B — ALIGN/TURN force pure spin

```text
ALIGN / TURN_IN_PLACE:
  force_vx = 0.0
  force_w = 1.15 * herr   # nonzero
```

MPPI short-circuits these modes with directed `force_vx/force_w` → **`vx=0, w≠0`**.

### Break C — nav_models blocks want_rev on TURN

```text
if decision.mode in (ALIGN, TURN_IN_PLACE, POST_TURN, SAFE_STOP):
    # want_rev from recovery is NOT applied
```

Even when ladder says LOCAL_REVERSE, TURN mode prevents `force_reverse=True`.

### Break D — REPOSITION is not reverse

```text
REPOSITION:
  force_vx = +0.06 if front_free>0.9 else -0.06
  force_w = 0.25 * reposition_dir
```

Tiny vx + spin — **cannot** satisfy recovery progress.

### Break E — REVERSE_ESCAPE has no force_vx

Even when mode eventually is REVERSE_ESCAPE, `force_vx/force_w` stay `None`; relies on `force_reverse` + MPPI sampling. Works only if `phase=reverse_escape` and `force_rev` from sim loop.

---

## 3. Safety role (must not guess)

Two distinct cases:

| Case | requested_vx | safe_vx | Meaning |
|------|--------------|---------|---------|
| A | −0.12 | 0 | **Safety blocked** reverse (`STOP_REAR` / collision) |
| B | 0 | 0, w≠0 | **Never commanded** reverse — FSM/TURN path |

LIVE S1 previously showed REPOSITION/TURN + no reverse mode → **Case B**.

---

## 4. Main Map blue band gap

| Layer | Status |
|-------|--------|
| Backend `physical_trajectory.active` | Exists in `/api/nav/debug` |
| `PHYSICAL TRAJECTORY` widget | Reads debug |
| Main map `applySnap` | Reads `debug.physical_trajectory.active` → `setGuideBand` |
| `nav.local_path` fallback | Still old kinematic_band / planned path |

If debug payload missing `active` edges/poses during cycle, map falls back to thin/old band.  
Also: band not pushed into `state` snapshot `nav.*` — only debug path. Corrective must put corridor into **main snapshot nav** and prefer swept edges.

---

## 5. Vehicle-centric candidate gap

`rollout_candidate` is already differential-drive (forward+w arcs). Misread risk is **UI world-frame drawing** looking like lateral offset. Corrective: local candidate panel must render in body frame (front=+X body).

---

## 6. Corrective plan (next)

1. **COR-B**: `recovery_action∈{LOCAL_REVERSE,HISTORICAL_RETREAT}` → force `mode=REVERSE_ESCAPE`, `force_vx<0`, `force_w≈0` (from Probe.backward path); progress/timeout; no endless TURN
2. **COR-C**: Main map always renders active corridor from backend swept edges; recovery switches to rearward band
3. **COR-D**: Vehicle-centric local candidate audit + UI
4. **LIVE S1**: require `state.vx<0` + backward displacement + escape

---

## 7. Answers required by user

### Q1 — Who converts LOCAL_REVERSE → negative vx?

**Intended:** `ManeuverFSM` → `REVERSE_ESCAPE` → `DiffDriveMppi.step(force_reverse=True)` → `apply_safety` → physics.  
**Actual today:** conversion **does not happen** because FSM does not enter `REVERSE_ESCAPE`.

### Q2 — Why mode=REPOSITION not REVERSE_ESCAPE?

FSM priority: heading align / stuck reposition **outranks** dead-end reverse when `rot_safe=True`. Recovery action is ignored.

### Q3 — Why vx≈0, w≠0?

`ALIGN`/`TURN_IN_PLACE` set `force_vx=0`, `force_w≠0`; MPPI executes that directed command.

### Q4 — Safety?

Cannot claim Safety blocked until Case A evidence exists; current failure is Case B (no reverse request).
