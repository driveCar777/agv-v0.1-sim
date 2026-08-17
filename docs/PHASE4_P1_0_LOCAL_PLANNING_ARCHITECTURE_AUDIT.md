# PHASE4 P1-0 — Local Planning / MPPI / Speed Architecture Audit

**Status:** `P1-0 = PASS`  
**Baseline commit (start):** `5302909ec87ad8d91400fd1018e7fcc7b3023c33`  
**Scope:** READ / TRACE / AUDIT / OBSERVABILITY / DESIGN only — **no planning or control behavior change**.

```text
P0-A  = PASS (unchanged)
P0-B0 = PASS
P0-B  = PASS
P0-C  = PASS
P0-C.1 = PASS
3C–3F = PASS (offline re-run this stage)
P0-D / P0-E / P1 impl / 3G = NOT STARTED / FORBIDDEN
```

---

## One-sentence verdict

In open space the stack is **Global reference (~5 m) + short-horizon MPPI control (~1.6 s) with Pure Pursuit w-bias**, not a multi-timescale Global / continuous Local / Control pyramid: **Local Selector is an obstacle-triggered side comparator (OPEN → compare off → OBSERVATION_ONLY)**, and **MPPI already owns OPEN trajectory + vx/w** without an explicit open-space cruise speed (soft preference `0.22` + `_mean_vx` init `0.16` + sample std `0.08` + EMA), so neither Local nor MPPI ever grows a mid-range (1–3 m) rolling plan.

---

## 1. Current ownership (code call chain)

Confirmed from `sim_api_ext` physics tick → `LocalMppiModel.step` (`nav_models.py`) → ManeuverFSM → `DiffDriveMppi.step` → `apply_safety` → state integration:

```text
MISSION / goal
  ↓
GLOBAL A* path + Global Preview (~5 m)          nav_global_preview / global planner
  ↓
NavigationPolicy (scene/state/profile/vx_scale) nav_policy.py
  ↓
ProbeEngine (F/L/R/B corridors)                 nav_probe.py
  ↓
ManeuverFSM  ──optional──► LocalManeuverSelector.compare   local_maneuver.py
  │                         (only if need_side_compare)
  ↓
DiffDriveMppi.step  (default AGV_LOCAL_CONTROL=mppi)
  │   • samples around _mean_vx / _mean_dw
  │   • pure_pursuit_w() seeds w samples + w_des   ← ACTIVE, not dead
  │   • softmax → vx_raw / dw_raw
  │   • EMA → _mean_vx / _cmd_vx / _cmd_w
  ↓
apply_safety (front_stop / rear / collision)      sim_api_ext.apply_safety
  ↓
COMMAND (safe_vx, safe_w) → PHYSICS (acc_v, ~20 Hz)
```

### Q1–Q10

| Q | Answer | Evidence |
|---|--------|----------|
| Q1 Who generates Local Candidate? | `LocalManeuverSelector.compare` → `rollout_candidate` | `local_maneuver.py` |
| Q2 Who generates MPPI trajectories? | `DiffDriveMppi.step` sample loop + `_rollout_body` | `mppi_controller.py` |
| Q3 Who finally selects trajectory? | Softmax over MPPI samples; best band for UI; executed = cmd integral | same |
| Q4 Who finally selects vx? | MPPI `vx_cmd` then Safety (may zero) then physics accel | `step` + `apply_safety` |
| Q5 Who finally selects w? | `w_des = pp_w + 0.35*_mean_dw` then blend to `_cmd_w`, then Safety | `mppi_controller.py` ~571–579 |
| Q6 Pure Pursuit in default path? | **YES** — seeds `pp_w` every MPPI step; `pp_only` only if env | `pure_pursuit_w(... lookahead_m=1.4)` |
| Q7 MPPI outputs final command? | **YES** (default); Safety can override | `LocalMppiModel` → `apply_safety` |
| Q8 FSM override MPPI? | **Sometimes** — `force_vx`/`force_w`/`vx_min`/`vx_max`/`force_reverse` | maneuver decision → `mppi.step` |
| Q9 Safety often change cmd? | OPEN clear: **no** (P0-C.1 LIVE requested≈safe) | forensics + LIVE |
| Q10 OPEN Local Selector in control? | **No** — `compare_called=0`, reason `NONE_OPEN_FORWARD` | P0-C.1 + `maneuver.py` gate |

**LOCAL_SELECTOR_ROLE_IN_OPEN = OBSERVATION_ONLY** (when compare not called; UI may still show MPPI fallback candidates).

---

## 2. Horizon / time-scale matrix

| Layer | Horizon | dt / period | Speed prior | Approx distance | Refresh | Purpose |
|-------|---------|-------------|-------------|-----------------|---------|---------|
| Global Path | full A* | event | n/a | goal-dependent | on replan | route |
| Global Reference | **~5.0 m** preview | event | n/a | 5 m | with path | UI + follow ref |
| Probe F/L/R/B | **0.50 / 1.05 / 1.80 m** | 0.10 s sample | state vx | those meters | each local step | feasibility / corridor |
| Local Selector | **1.5 s** (15×0.1) | 0.1 s | NOMINAL 0.18 / SIDE 0.22 | **0.27 / 0.33 m** | only when compare | side pick |
| MPPI | **1.6 s** (16×0.1) | 0.1 s model; **local_period 0.35 s** | mean init 0.16; soft track 0.22 | **~0.26–0.35 m** at 0.16–0.22 | ~2.9 Hz | control prediction + cmd |
| Pure Pursuit | lookahead **1.35–1.4 m** | per MPPI step | uses cmd/vx | geometric L | with MPPI | w bias / pp_only cruise 0.22 |
| FSM | mode dwell | event | vx_min/max | n/a | each local | mode / limits |
| Safety | stop envelope **front_stop 0.70 m** | every physics tick | n/a | stop | 20 Hz | hard veto |
| Physics | integration | **dt≈0.05 (20 Hz)** | acc_v 0.9 | — | continuous | plant |

---

## 3. Trajectory sources (UI / data)

| Visual / Data | Source (function / file) |
|---------------|--------------------------|
| Global Reference | `build_global_reference` / `nav_global_preview.py` → `sim_main` `setGlobalReference` |
| Local Candidates | Prefer `maneuver.local_compare` → `collect_local_candidates` (`LOCAL_SELECTOR`); else MPPI `path_candidates` (`source=MPPI`) |
| Selected Local | `local_compare` selected / `decision_label` |
| MPPI Best Path | `DiffDriveMppi.step` → `best_path` / kinematic_band → `state._planned_path` |
| Physical Trajectory | `LocalMppiModel._select_active_corridor` from **Probe** F/L/R/B |
| Probe F/L/R/B | `ProbeEngine.evaluate` → `nav_probe.py`; UI `sim_main` probe cards |
| Guide Band | `kinematic_band` / MPPI candidate guides → `setGuideBand` |

OPEN LIVE (P0-C.1): Local Candidates UI source often **MPPI** because selector compare is off; physical corridor source **PROBE**.

---

## 4. Speed generation chain (real structure)

Not a clean `MIN(hardware, policy, …)` ladder. Actual OPEN path:

```text
geom.max_vx = 0.40
    → MPPI a_vx_max ≈ 0.38 (0.95 * max_vx)
    → samples: vx ← N(_mean_vx, vx_std≈0.08) clipped to [a_vx_min, a_vx_max]
    → softmax → vx_raw
    → _mean_vx := 0.9*_mean_vx + 0.1*vx_raw
    → _cmd_vx  := 0.82*_cmd_vx + 0.18*_mean_vx
    → if vx_cmd>0: vx_cmd *= policy.vx_scale   (OPEN NORMAL = 1.0)
    → soft front slowdown if front_near < front_cost_m (0.90)
    → apply_safety (OPEN clear: pass-through)
    → state.vx += clamp(acc_v*dt, safe_vx - state.vx)
```

**OPEN nominal target speed = NO EXPLICIT OPEN-SPACE CRUISE SPEED**  
Soft preference only: `speed_track_cost = 0.25 * abs(mean_vx - 0.22)`.

### Why `0.22`?

| Use | Value | Role |
|-----|-------|------|
| `SIDE_VX` (local_maneuver) | 0.22 | side-candidate rollout speed |
| `_score` speed_track | 0.22 | soft cost preference |
| `step_pp_only` cruise | 0.22 | hardcoded PP-only forward speed |
| softmax `temperature` | 0.22 | **unrelated** (kelvin-like) |

**CURRENT CODE TARGET (soft) = 0.22** — not a documented open cruise 0.30–0.40.

### Policy `vx_scale` (no hidden re-multiply beyond one apply in MPPI)

| Scene / profile | vx_scale |
|-----------------|----------|
| OPEN / FOLLOW_GLOBAL / NORMAL | **1.0** |
| CAUTION | 0.75 |
| AVOID | 0.65 |
| RECAPTURE | 0.70 |
| RECOVERY | 0.50 |

---

## 5. OPEN behavior

```text
OPEN + forward_feasible + front clear + FOLLOW_GLOBAL
  → need_side_compare = false
  → Local Selector compare_called = false (NONE_OPEN_FORWARD)
  → Command = Global path tracking via MPPI (+ PP w)
  → Local Selector does NOT participate in execution
```

Architecture label:

```text
NORMAL:   Global + MPPI (+ PP for w)     ← no continuous Local Planner
AVOID:    Local Selector + Maneuver + MPPI
RECOVERY: Recovery FSM + reverse/side ladder
```

---

## 6. Why Local is short

**Not only** `ROLLOUT_STEPS=15`.

Dependency:

```text
ROLLOUT_STEPS=15, ROLLOUT_DT=0.1
  → horizon_s = 1.5
  → with NOMINAL_VX=0.18 → planned ≈ 0.27 m
  → with SIDE_VX=0.22 → planned ≈ 0.33 m
  → comment in code: "1.5s short-horizon compare"
  → purpose: LEFT/RIGHT vs FORWARD side switch under corridor stress — NOT mid-range planning
```

Distance taxonomy (OPEN):

| Kind | Typical OPEN | Verdict |
|------|--------------|---------|
| PLANNED_DISTANCE | 0.27–0.33 m if compare; else n/a | **SHORT_BY_DESIGN** |
| RENDERED_DISTANCE | MPPI fallback ~0.3 m | packaging OK |
| EXECUTED_DISTANCE | vehicle follows MPPI cmd, not Local path | Local not executed |

For user goal “plan obstacle bypass 2–3 m ahead”: current Local **NOT SUFFICIENT** — but that is a **role/horizon design gap**, not a UI bug.

---

## 7. Why speed is conservative (CONFIRMED factors)

1. **No hard open cruise** — only soft 0.22 track (weight 0.25).  
2. **`_mean_vx` init 0.16** + samples std **0.08** → exploration stays near prior.  
3. **EMA** `0.9/0.1` mean and `0.82/0.18` cmd + **local_period 0.35 s** → slow climb even if raw prefers higher.  
4. LIVE OPEN: mean_vx ≈ 0.16–0.21, state ≈ same band.  
5. Safety / capture / P0-C: **NOT EVIDENCED** as OPEN speed bottleneck.

Cost sweep nuance: on empty straight path, **goal progress term prefers higher vx** (lowest total at 0.40 in offline constant-vx sweep). Conservatism is therefore **not** “path cost hates speed” on open straight; it is **sampling prior + EMA + soft track**, i.e. the optimizer rarely *proposes* high vx.

---

## 8. MPPI cost landscape (offline)

Script: `scripts/_audit_phase4_open_space_speed_cost.py`

Straight open path, w=0, no obstacles, path_follow_weight=5:

| vx | speed_track | gpath | goal | total |
|----|-------------|-------|------|-------|
| 0.10 | 0.030 | 0.375 | 41.118 | 41.523 |
| 0.22 | 0.000 | 0.668 | 40.062 | 40.730 |
| 0.30 | 0.020 | 0.575 | 39.358 | 39.953 |
| **0.40** | 0.045 | 0.517 | 38.478 | **39.040** ← lowest |

**Landscape “likes” higher progress speed when vx is *forced*; live sampler does not force it.**

Lateral offset y=0.15: still lowest at 0.40 — path cost does not reverse the ranking on open space.

---

## 9. MPPI convergence dynamics

Formulas (production):

```text
_mean_vx ← 0.9 * _mean_vx + 0.1 * vx_raw
_cmd_vx  ← 0.82 * _cmd_vx + 0.18 * _mean_vx
```

Script: `scripts/_audit_phase4_open_space_speed_response.py`

| Assumption | Result |
|------------|--------|
| Sustained `vx_raw=0.30`, update every **0.35 s** (local_period) | After ~5 s: mean≈0.27, cmd≈0.24 — **not yet 0.30** |
| Same, every **0.05 s** (if MPPI ran at physics rate) | cmd≈0.30 by ~2–3 s |
| Sustained `vx_raw=0.22` | asymptote mean/cmd → **0.22** |

Live OPEN mean≈0.21 matches soft preference + slow EMA, **not** max_vx.

---

## 10. Pure Pursuit usage

```text
UNUSED?  NO — not dead code on default path.

PP_w = pure_pursuit_w(..., lookahead_m=1.4)
  → sample w = pp_w + dw
  → w_des = pp_w + 0.35*_mean_dw
  → w_cmd blend / rate limit
  → Safety → final_w

pp_only mode: AGV_LOCAL_CONTROL=pp|pp_only|pure_pursuit → step_pp_only (vx=0.22 hard)
```

**Authority:** MPPI owns blended `(vx_cmd, w_cmd)`; PP owns the **heading/curvature prior** inside that blend.

---

## 11–12. Policy / Safety interaction

- OPEN: `NORMAL` profile, `vx_scale=1.0`, `path_follow_weight≈5.0`.  
- Safety OPEN clear: `requested == safe` → **NOT SAFETY**.  
- front_stop_m=0.70 only bites when near obstacle.

---

## 13. Architecture mismatch

| Desired | Current |
|---------|---------|
| GLOBAL 2–8 m “where to go” | ≈5 m preview — **OK** |
| LOCAL 1–3 m / 2–4 s rolling executable | Selector 0.27–0.33 m **and disabled in OPEN** — **GAP** |
| CONTROL 0.1–1 s command | MPPI 1.6 s prediction used as **both** short local + control — **merged roles** |

Current system:

```text
Global tracking + special-mode local avoidance + recovery
```

Not:

```text
Global + continuous Local Planner + short Control
```

---

## 14. Desired architecture (design only)

```text
Global:   4–8 m reference (keep ~5 m class)
Local:    1–3 m physical candidates / 2–4 s adaptive rolling horizon
          continuous in OPEN (forward continuation), longer arcs on approach
MPPI:     1–2 s control prediction (stay short; do NOT become 5 s planner)
Recovery: separate shorter / dedicated horizon
```

**Do not** “fix short Local” by only lengthening MPPI to 5 s.

---

## 15. P1 implementation proposal (not this stage)

1. Redefine Local Selector as **continuous rolling local planner** (or add a sibling module) with adaptive 2–4 s horizon.  
2. OPEN: always refresh forward / mild arc candidates (not only obstacle-triggered compare).  
3. Keep MPPI as short control layer; feed selected local path as stronger prior.  
4. Introduce **explicit open-space cruise** (or raise soft track + sample prior) **separately** from horizon work.  
5. Keep Recovery / 3F reverse contract untouched.  
6. Candidate space: FORWARD / LEFT_ARC / RIGHT_ARC / REVERSE / ROTATION with clear ownership vs FSM.

---

## 16. Risks

| Risk | Note |
|------|------|
| Longer Local without speed raise | Still ~0.5–0.7 m at 0.22×3 s — may still feel “short” |
| Raising speed without mid Local | Faster into obstacles; Probe/Safety pressure rises |
| Merging Local into MPPI 5 s | Cost explosion, UI confusion, fights Global |
| Breaking OPEN compare=false assumption | Changes behavior; needs staged P1 + regression |

---

## 17. Tests / audits this stage

| Script | Result |
|--------|--------|
| `_audit_phase4_open_space_speed_cost.py` | PASS |
| `_audit_phase4_open_space_speed_response.py` | PASS |
| `_audit_phase4_local_horizon_coverage.py` | PASS |
| `_audit_phase4_kinematic_path.py` (P0-C) | PASS |
| `_audit_phase4_global_preview.py` (P0-B) | PASS |
| `_audit_phase4_commitment.py` (3C) | PASS |
| `_audit_phase4_probe.py` (3D) | PASS |
| `_audit_phase4_side_switch.py` (3E) | PASS |
| `_audit_phase4_recovery_execution.py` (3F reverse) | PASS (`force_vx=-0.12`, `REVERSE_ESCAPE`) |
| `_audit_phase4_recovery.py` C2_edges | **FAIL on baseline `5302909` also** (pre-existing corridor edge len check; not introduced by P1-0) |

Coverage matrix (excerpt):

| vx \ horizon | 1.5 s | 1.6 s | 3.0 s | 4.0 s |
|--------------|-------|-------|-------|-------|
| 0.16 | 0.24 m | 0.26 m | 0.48 m | 0.64 m |
| 0.18 | 0.27 m | 0.29 m | 0.54 m | 0.72 m |
| 0.22 | 0.33 m | 0.35 m | 0.66 m | 0.88 m |
| 0.30 | 0.45 m | 0.48 m | 0.90 m | 1.20 m |
| 0.40 | 0.60 m | 0.64 m | 1.20 m | 1.60 m |

Local short = **short time × low speed** (two independent factors).

---

## Root-cause tags

| Tag | Verdict |
|-----|---------|
| FIXED_LOCAL_HORIZON_1_5S | **CONFIRMED** |
| FIXED_MPPI_HORIZON_1_6S | **CONFIRMED** |
| OPEN_LOCAL_COMPARE_DISABLED | **CONFIRMED** |
| LOCAL_SELECTOR_OBSTACLE_TRIGGER_NOT_CONTINUOUS | **CONFIRMED** |
| NO_EXPLICIT_OPEN_CRUISE_SPEED | **CONFIRMED** |
| MPPI_SPEED_SOFT_PRIOR_0_22 | **CONFIRMED** |
| MPPI_MEAN_INIT_0_16_PLUS_SAMPLE_STD | **CONFIRMED** |
| COMMAND_EMA_SMOOTHING | **CONFIRMED** (contributor; stronger at 0.35 s cadence) |
| PP_ACTIVE_AS_W_PRIOR | **CONFIRMED** |
| PATH_FOLLOW_COST_BLOCKS_HIGH_SPEED_OPEN | **NOT EVIDENCED** (constant-vx sweep prefers higher vx) |
| SAFETY_LIMIT_OPEN | **NOT EVIDENCED** |
| CAPTURE_LIMIT_OPEN | **NOT EVIDENCED** |
| P0C_PATH_INVALID_OPEN | **NOT EVIDENCED** |
| UI_PACKAGING_SHORTENS_LOCAL | **NOT EVIDENCED** (SHORT_BY_DESIGN) |

---

## Observability (diagnostics-only extensions)

Reuses P0-B0 / P0-C.1 forensics (`GET /api/nav/forensics/open-space`):

- `mppi.speed_target` / `speed_target_kind=SOFT_SPEED_TRACK_COST`
- `diagnostics.local_selector_role_in_open`
- `diagnostics.open_space_cruise_speed=NO_EXPLICIT_...`
- `diagnostics.planned/rendered/executed_distance_m`
- architecture mode labels for OPEN vs AVOID

---

## STOP

No Local/MPPI horizon, NOMINAL/SIDE/vx, Safety, Policy, FSM, Recovery, scoring, capture, or reverse changes.  
**P1 implementation / P0-D / P0-E / 3G not started.**
