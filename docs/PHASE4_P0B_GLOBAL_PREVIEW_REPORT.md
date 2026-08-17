# PHASE4 P0-B — Global Reference Preview Report

日期：2026-08-17  
基线：`71c466b` / V0.11  
范围：**仅 P0-B Global Reference Preview**（REFERENCE ONLY）  
禁止：P0-C / P0-D / P0-E / P1 / 3G / Local horizon 加长 / cmd_vel 接线

```text
P0-B STATUS = PASS
3C = PASS
3D = PASS
3E = PASS
3F = PASS

GLOBAL_PREVIEW_NORMAL = 5.0m
GLOBAL_PREVIEW_MIN = 2.0m
GLOBAL_PREVIEW_MAX = 8.0m

LOCAL_MAX_DISTANCE = ~0.15–0.35m (LIVE; short horizon expected)

NEXT ALLOWED STEP = P0-C
```

---

## Q1 — 为什么以前蓝带只有 ~0.3m？

主图画的是 **Selected Local Physical Trajectory**（≈1.5s rollout → 约 0.2–0.4 m），不是 Global Preview。P0-B 增加了独立的长灰线 **Global Reference**（adaptive 2–8 m，normal ≈5 m）。

## Q2 — Global Preview 是否控制车辆？

```text
NO
controls_vehicle = false
kinematic_valid = null
```

构建发生在 control / Safety **之后**；异常只降级 telemetry。

## Q3 — Local / MPPI horizon 是否被修改？

```text
NO
```

## Q4 — 3F 是否回退？

```text
NO
```

LIVE S1/S2/S6：**PASS**（`signed_back ≥ 0.12m`，S2 无 spin，S6 无非法 reverse）。

## Q5 — Global / Local 是否同时可见？

```text
YES (LIVE)
GLOBAL ≈ 4.9m + LOCAL_MAX ≈ 0.25–0.34m
```

## Q6 — `kinematic_valid` 是否伪造成 false？

```text
NO — always null until P0-C
```

---

## Files

| Path | Change |
|------|--------|
| `ros2_ws/.../nav_global_preview.py` | **NEW** densify / adaptive slice / first-turn / local packaging |
| `ros2_ws/.../nav_observability.py` | dynamic starvation; GLOBAL_* events; owner semantics; global fields |
| `ros2_ws/.../sim_api_ext.py` | after-control preview attach + `get_nav_preview` |
| `scripts/run_web_sim.py` | `/api/state` nav layers + `/api/nav/preview` |
| `www/sim3d.js` | Global Reference + all Local Candidates layers |
| `www/sim_main.html` | applySnap layers + legend + status widget |
| `scripts/_audit_phase4_global_preview.py` | Tests A–J |
| `scripts/_trace_phase4_global_local_preview.py` | GLOBAL vs LOCAL timeline |
| `docs/PHASE4_NAVIGATION_LOG_API.md` | P0-B fields / events |
| `docs/PHASE4_NAVIGATION_KINEMATIC_CONTRACT.md` | Global Reference = P0-B implemented |

---

## Audits

| Test | Result |
|------|--------|
| A Open ~5m | PASS |
| B Goal limited | PASS |
| C Path limited | PASS |
| D Start near pose | PASS |
| E Progress stable | PASS |
| F First turn | PASS |
| G Global/Local separation | PASS |
| H Env toggle / isolation | PASS (`NAV_GLOBAL_PREVIEW=0` → `DISABLED`; control continues) |
| I Path revision | PASS |
| J Goal reached | PASS |
| P0-A swept | PASS |
| 3C / 3D / 3E / 3F offline | PASS |
| 3F LIVE S1/S2/S6 | PASS |

---

## Known limitations (do NOT “fix” here)

1. Local candidates remain short (~0.3 m) — expected until P1.  
2. Global Reference is **unvalidated** — turns may be kinematically infeasible (P0-C).  
3. First-turn is metadata only — does not change Approach (P0-D).  
4. Display swept on Global is lightweight, not collision proof.  
5. `GLOBAL_LOCAL_HORIZON_MISMATCH` is a **fact** event, not a bug verdict.

---

## STOP

未进入 P0-C / P0-D / P0-E / P1 / 3G。
