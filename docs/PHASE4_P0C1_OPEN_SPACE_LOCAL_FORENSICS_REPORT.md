# PHASE4 P0-C.1 — Open-Space Local Planning Forensics

日期：2026-08-17  
基线：`7c3449c68def519bab5ed32292a8825b821bc852` / P0-C COMPLETE  
范围：**只读诊断**（telemetry / API / audit / report）  
禁止：改 Local horizon、MPPI prior、Policy vx_scale、Safety、FSM、Recovery、P0-C validator、进入 P1

```text
P0-C.1 STATUS = PASS
P0-A  = PASS
P0-B0 = PASS
P0-B  = PASS
P0-C  = PASS
3C    = PASS
3D    = PASS
3E    = PASS
3F    = PASS (LIVE S1 signed_back=0.125m)
```

LIVE 证据：`docs/_phase4_trace/open_space_forensics_1786940337_*`

---

## 一句话结论

在前方 8–10 m 空旷、`Global≈5 m`、`forward_feasible=true`、`scene=OPEN` 时：

1. **Local Selector 不持续 compare**（`compare_called=false`，`NONE_OPEN_FORWARD` / FSM `NEED_SIDE_COMPARE_FALSE`）。
2. UI 上看到的短线 **主要来自 MPPI candidates / Probe corridor**（`render.local_candidates_source=MPPI`，`active_physical_source=PROBE`），不是 Local Selector 刚算出来的 1.5 s FORWARD 弧。
3. 短距离是 **固定短 horizon × 当前速度** 的数学结果，不是碰撞 / capture / Safety / P0-C 把路径掐断。
4. 「胆小」主要来自 **MPPI `_mean_vx` 初值 0.16 + cmd 混合滞后**，以及 OPEN 正常目标并非 `0.22` / `0.40`。

---

## LIVE 数字（outdoor open，goal=10 m，4.5 s）

| 量 | 实测 |
|----|------|
| policy | `FOLLOW_GLOBAL` / scene `OPEN` / `vx_scale=1.0` / profile `NORMAL` |
| global_preview_m | mean **4.95** |
| local_max_distance_m | mean **0.364**（UI layer；source=**MPPI**） |
| coverage_ratio | mean **0.074** |
| compare_called | **0/13** → `NONE_OPEN_FORWARD` |
| short_horizon_reason | **FIXED_1_5S** (13/13) |
| mean_vx (MPPI) | mean **0.212**（从 ~0.18 爬升） |
| vx_raw | ~0.22–0.30 |
| requested / safe / state | **三者几乎相等**（mean **0.164**） |
| safety_clamp | **0** |
| kinematic_valid | **true** |
| path_valid | **true**（`len(global_path)>=2`；**不读** P0-C） |
| speed_ratio = state/max_vx | mean **0.41** |
| expected `0.18×1.5` | **0.27 m** |

数学核对：

```text
FIXED horizon:
  Local Selector design: 15 × 0.1s = 1.5s × NOMINAL_VX 0.18 → 0.27 m
  MPPI design:           16 × 0.1s = 1.6s × state≈0.20   → ≈0.32 m
LIVE local_max ≈ 0.33–0.39 m ≈ MPPI best_path_m (0.337 @ T+1.08)
→ NOT early collision / capture cut
```

---

## Q1 — Local 短是不是固定 horizon？

```text
YES — CONFIRMED FIXED_1_5S / FIXED short-horizon design
Local Selector: ROLLOUT_STEPS=15, ROLLOUT_DT=0.1 → 1.5s
MPPI: time_steps=16, model_dt=0.1 → 1.6s
Probe forward rollout reuses same 1.5s local rollout API
```

## Q2 — Local 短是否因为低速度？

```text
PARTLY — CONFIRMED secondary
mean_state_vx ≈ 0.16 → 1.5s×0.16=0.24m ; 1.6s×0.16=0.26m
LIVE lines ~0.33–0.39m track rising state/mppi speed, not 0.15m floor
```

## Q3 — Candidate 是否提前 invalid？

```text
NOT EVIDENCED (open-space LIVE)
planned ≈ survived; no early collision cut in OPEN samples
```

## Q4 — Path capture 是否限制 OPEN Local？

```text
NOT EVIDENCED in OPEN LIVE
require_capture_hard=true under FOLLOW_GLOBAL, but compare never ran
```

## Q5 — OPEN 是否持续 compare？

```text
NO — CONFIRMED
compare_called=false for 13/13 samples
FSM need_side_compare false when FORWARD + front clear + not LOCAL_*
needs_compare also returns false → NONE_OPEN_FORWARD
```

## Q6 — 速度被谁压低？

```text
PRIMARY: MPPI prior + cmd lag  (CONFIRMED)
  _mean_vx init 0.16
  vx_cmd = 0.82*_cmd_vx + 0.18*_mean_vx
  OPEN FORWARD_TRACK vx_max = max_vx*0.95 = 0.38, but prior starts low

Policy vx_scale: 1.0 NORMAL — NOT the clamp
Safety: requested≈safe — NOT EVIDENCED
Execution: state≈safe — not the bottleneck
```

## Q7 — P0-C 是否让 Policy 变保守？

```text
NOT EVIDENCED
path_valid = bool(global_path) and len>=2   # nav_models.py
kinematic_valid is telemetry-only; not wired into Policy.step
LIVE: kin=true, scene=OPEN, state=FOLLOW_GLOBAL, vx_scale=1.0
```

## Q8 — MPPI mean_vx 是否长期停在 0.16？

```text
CONFIRMED start near 0.16–0.18; LIVE mean over window ≈0.21 (climbing)
Not stuck forever at 0.16, but OPEN cruise target is still far below 0.40
```

## Q9 — 空旷 FOLLOW_GLOBAL 目标速度到底是多少？

```text
Code chain (not guessed):
  Selector FORWARD vx = 0.18 (NOMINAL_VX) — only when compare runs
  Selector SIDE vx = 0.22 — only when LEFT/RIGHT compare runs
  Policy NORMAL vx_scale = 1.0
  Maneuver FORWARD_TRACK allows [0, max_vx*0.95]=[0,0.38]
  MPPI samples around _mean_vx (init 0.16), then lags into cmd

Effective OPEN target ≈ MPPI mean dynamics (≈0.16→0.22+), NOT 0.22/0.40 hard setpoint
```

## Q10 — 「胆小」归属？

```text
ROOT: short fixed planning horizon (1.5/1.6s) + MPPI speed prior/lag
SECONDARY: OPEN does not refresh Local Selector compare (UI falls back to MPPI)
NOT: Safety clamp / P0-C PATH_INVALID / capture failure (open LIVE)
```

---

## Root-cause tags

| Tag | Verdict |
|-----|---------|
| LOCAL_HORIZON_FIXED_1_5S | **CONFIRMED** |
| LOW_SPEED_VS_SIDE_NOM_0_22 | **CONFIRMED** |
| MPPI_MEAN_VX_NEAR_0_16 | **CONFIRMED** |
| OPEN_NO_SUSTAINED_COMPARE | **CONFIRMED** |
| SAFETY_CLAMP | **NOT EVIDENCED** |
| PATH_CAPTURE_FAILURE | **NOT EVIDENCED** |
| EARLY_INVALIDATION | **NOT EVIDENCED** |
| P0C_FORCES_PATH_INVALID | **NOT EVIDENCED** |

---

## 代码事实链（审计）

```text
1. Local candidates generated only when ManeuverFSM invokes local_selector.compare
2. Re-compare gated by needs_compare + need_side_compare
3. OPEN + FORWARD + front_near large → compare NOT called
4. Command authority: Policy → Maneuver limits → MPPI.step → Safety.apply_safety → state.vx
5. MPPI best_path is what executes (plus maneuver force_* when in LOCAL_*/REVERSE)
6. UI short band: Probe physical corridor; Local Candidates layer falls back to MPPI when no compare
7. OPEN: profile NORMAL, vx_scale=1.0
8. MPPI _mean_vx init/reset = 0.16
9–12. LIVE: vx_raw > requested≈safe≈state (lag), no safety cut
13. kinematic_valid does NOT set path_valid
14. path_valid=false would scene=PATH_INVALID — did not happen in OPEN LIVE
15. Safety does not decelerate open-space (requested==safe)
```

---

## Files

| Path | Role |
|------|------|
| `nav_open_space_forensics.py` | **NEW** assemble-only forensics |
| `local_maneuver.py` | diagnostics: horizon/nominal/compare_reason/distance_m |
| `maneuver.py` | diagnostics: compare invoked / skip reason |
| `mppi_controller.py` | telemetry: mean_vx_before/after, vx_raw/cmd, scales |
| `nav_models.py` | expose `last_path_valid` (no behavior change) |
| `nav_global_preview.py` | candidate `source` MPPI vs LOCAL_SELECTOR |
| `sim_api_ext.py` | attach forensics + GET `/api/nav/forensics/open-space` |
| `scripts/run_web_sim.py` | route + `/api/state` compact summary |
| `scripts/_trace_phase4_open_space_local_forensics.py` | LIVE timeline JSONL/CSV |

---

## STOP

未修改 Local horizon / vx / Safety / P0-C。  
未进入 P0-D / P0-E / P1 / 3G。
