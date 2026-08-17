#!/usr/bin/env python3
"""P1-1 offline: Rolling Local Planner + Speed Policy (open space).

Tests A–E. Does not start the web sim. Does not change 3F Recovery.
"""

from __future__ import annotations

import math
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_local_planner import (  # noqa: E402
    AUTH_ROLLING,
    KIND_LEFT_ARC,
    KIND_RIGHT_ARC,
    LocalPlanRequest,
    RollingLocalPlanner,
)
from agv_bridge.nav_speed_policy import OPEN_CRUISE_VX, SpeedPolicy  # noqa: E402


def _ok(name: str, cond: bool, detail: str = "") -> bool:
    mark = "PASS" if cond else "FAIL"
    print(f"  {mark} {name} {detail}")
    return bool(cond)


def straight_path(length: float = 12.0, step: float = 0.25):
    n = max(2, int(length / step) + 1)
    return [(i * step, 0.0) for i in range(n)]


def arc_path(radius: float = 4.0, sweep_deg: float = 50.0, step: float = 0.25):
    pts = [(0.0, 0.0)]
    yaw = 0.0
    x = y = 0.0
    kappa = 1.0 / radius
    dist = 0.0
    target = radius * math.radians(sweep_deg)
    while dist < target:
        x += step * math.cos(yaw)
        y += step * math.sin(yaw)
        yaw += kappa * step
        dist += step
        pts.append((x, y))
    return pts


def simulate_ema(target: float = 0.30, dt: float = 0.20, seconds: float = 2.0):
    mean = 0.16
    cmd = 0.0
    rows = [(0.0, mean, cmd)]
    t = 0.0
    while t + 1e-9 < seconds:
        mean = 0.55 * mean + 0.45 * target
        mean = 0.85 * mean + 0.15 * mean  # raw≈mean in open
        cmd = 0.50 * cmd + 0.50 * mean
        t += dt
        rows.append((round(t, 3), mean, cmd))
    return rows


def main() -> int:
    fails = 0
    print("=== P1-1 Rolling Local Planner / Speed Policy ===")
    print(f"OPEN_CRUISE_VX={OPEN_CRUISE_VX}")

    pol = SpeedPolicy()
    planner = RollingLocalPlanner()

    # --- A: open straight, plan ≥ 2m, target ≈ 0.30 ---
    g = straight_path()
    spd = pol.compute(scene="OPEN", policy_state="FOLLOW_GLOBAL", front_near=30.0, left_free=8.0, right_free=8.0, goal_distance_m=10.0)
    req = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.16, global_path=g, global_preview_m=5.0, goal=g[-1],
        collide=lambda _x, _y: False, clearance_at=lambda _x, _y: 5.0,
        front_near=30.0, left_free=8.0, right_free=8.0, scene="OPEN",
        policy_state="FOLLOW_GLOBAL", speed=spd, now=time.time(), authority=AUTH_ROLLING,
    )
    t0 = time.perf_counter()
    res = planner.update(req)
    ms = (time.perf_counter() - t0) * 1000.0
    plan = res.plan
    print(f"  compute_ms={ms:.1f} horizon_m={None if plan is None else plan.horizon_m} kind={None if plan is None else plan.selected_candidate_id}")
    fails += not _ok("A_plan_exists", plan is not None)
    fails += not _ok("A_horizon_ge_2m", plan is not None and float(plan.horizon_m) >= 2.0, str(None if plan is None else plan.horizon_m))
    fails += not _ok("A_target_0_30", abs(spd.target_vx - 0.30) < 0.021, str(spd.target_vx))
    fails += not _ok("A_open_prefers_forward", plan is not None and "FORWARD" in str(plan.selected_candidate_id))
    fails += not _ok("A_kinematic_valid", plan is not None and plan.kinematic_valid)
    fails += not _ok("A_compute_budget", ms < 80.0, f"{ms:.1f}ms")
    cov = None if plan is None else plan.horizon_m / 5.0
    fails += not _ok("A_coverage_ge_0_4", cov is not None and cov >= 0.40, str(cov))

    # --- B: EMA climbs off 0.16 toward 0.30 within ~1.5s at 0.20s period ---
    rows = simulate_ema()
    cmd_15 = next((c for t, m, c in rows if t >= 1.4), rows[-1][2])
    fails += not _ok("B_not_stuck_at_0_16", rows[-1][1] > 0.22, f"mean={rows[-1][1]:.3f}")
    fails += not _ok("B_cmd_near_target_1_5s", cmd_15 >= 0.24, f"cmd@1.5s={cmd_15:.3f}")
    mppi = DiffDriveMppi()
    mppi._mean_vx = 0.16
    mppi._cmd_vx = 0.0
    collide = lambda _x, _y: False
    path = straight_path()
    for i in range(8):
        r = mppi.step(0.0, 0.0, 0.0, path, path[-1], collide, front_near=30.0, target_vx=0.30, local_plan_path=plan.as_xy() if plan else None)
    meta = mppi._last_meta
    fails += not _ok("B_mppi_target_field", meta.get("target_vx") == 0.30, str(meta.get("target_vx")))
    fails += not _ok("B_mean_left_0_16", float(meta.get("mean_vx_after") or 0) > 0.20, str(meta.get("mean_vx_after")))
    fails += not _ok("B_cmd_not_frozen", abs(float(r.vx)) > 0.18, str(r.vx))

    # --- C: gentle turn generates an arc family (not only a point) ---
    planner.reset()
    ap = arc_path()
    req_c = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=ap, global_preview_m=5.0, goal=ap[-1],
        collide=lambda _x, _y: False, clearance_at=lambda _x, _y: 4.0,
        front_near=30.0, left_free=4.0, right_free=4.0, scene="OPEN", speed=spd, now=time.time() + 1.0,
    )
    rc = planner.update(req_c)
    kinds = {c.kind for c in (rc.plan.candidates if rc.plan else []) if c.valid}
    fails += not _ok("C_has_left_or_right_arc", KIND_LEFT_ARC in kinds or KIND_RIGHT_ARC in kinds, str(kinds))
    fails += not _ok("C_plan_length", rc.plan is not None and rc.plan.horizon_m >= 1.2, str(None if rc.plan is None else rc.plan.horizon_m))

    # --- D: tight / high curvature reduces speed ---
    spd_d = pol.compute(scene="TIGHT", policy_state="FOLLOW_GLOBAL", front_near=30.0, left_free=0.40, right_free=0.40, abs_kappa=0.9)
    fails += not _ok("D_tight_slower", spd_d.target_vx < 0.24, str(spd_d.target_vx))
    spd_k = pol.compute(scene="OPEN", abs_kappa=1.2, front_near=30.0, left_free=5.0, right_free=5.0)
    fails += not _ok("D_curvature_limit", "CURVATURE" in spd_k.limits or spd_k.target_vx < OPEN_CRUISE_VX, str(spd_k.to_dict()))

    # --- E: obstacle on left → RIGHT_ARC can win (generation, not P0-D timing) ---
    planner.reset()

    def collide_left_wall(px, py):
        # wall along y=+0.9 from x=1.2 to 4.0 — left of a forward track
        return 1.2 <= px <= 4.0 and 0.55 <= py <= 1.6

    spd_e = pol.compute(scene="OPEN", front_near=3.0, left_free=0.6, right_free=4.0, goal_distance_m=10.0)
    req_e = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.22, global_path=g, global_preview_m=5.0, goal=g[-1],
        collide=collide_left_wall, clearance_at=lambda x, y: 0.2 if collide_left_wall(x, y) else 3.0,
        front_near=3.0, left_free=0.6, right_free=4.0, scene="OPEN", speed=spd_e, now=time.time() + 2.0,
    )
    re = planner.update(req_e)
    sel_kind = None
    right_valid = False
    if re.plan:
        sel_kind = next((c.kind for c in re.plan.candidates if c.selected), None)
        right_valid = any(c.valid and c.kind == KIND_RIGHT_ARC for c in re.plan.candidates)
    fails += not _ok("E_right_arc_valid", right_valid, f"selected={sel_kind}")
    # Selected may still be FORWARD if it clears; require RIGHT exists early (3m obstacle)
    fails += not _ok("E_plan_before_contact", re.plan is not None and re.plan.horizon_m >= 1.0)

    # Rolling refresh (not one-shot)
    planner2 = RollingLocalPlanner()
    tbase = time.time()
    r1 = planner2.update(LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.16, global_path=g, global_preview_m=5.0, goal=g[-1],
        collide=lambda _x, _y: False, scene="OPEN", speed=spd, now=tbase,
    ))
    r2 = planner2.update(LocalPlanRequest(
        x=0.15, y=0.0, yaw=0.0, vx=0.20, global_path=g, global_preview_m=5.0, goal=g[-1],
        collide=lambda _x, _y: False, scene="OPEN", speed=spd, now=tbase + 0.25,
    ))
    fails += not _ok("ROLL_replaces", r1.plan is not None and r2.plan is not None and r2.plan.plan_id != r1.plan.plan_id, f"{getattr(r1.plan,'plan_id',None)}→{getattr(r2.plan,'plan_id',None)}")

    # Speed policy does not bypass hardware
    fails += not _ok("SPD_le_max", spd.target_vx <= 0.40)

    print("RESULT", "FAIL" if fails else "PASS")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
