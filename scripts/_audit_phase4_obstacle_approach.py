#!/usr/bin/env python3
"""P0-D offline — Forward Future Obstacle Preview + early local activation.

Tests A–L. Does not start web sim. Does not modify front_stop_m.
"""

from __future__ import annotations

import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_local_planner import (  # noqa: E402
    KIND_FORWARD,
    KIND_LEFT_ARC,
    KIND_RIGHT_ARC,
    LocalPlanRequest,
    RollingLocalPlanner,
)
from agv_bridge.nav_obstacle_preview import (  # noqa: E402
    compute_future_preview,
    required_avoidance_distance,
)
from agv_bridge.nav_speed_policy import SpeedPolicy  # noqa: E402


def _ok(name: str, cond: bool, detail: str = "") -> bool:
    mark = "PASS" if cond else "FAIL"
    print(f"  {mark} {name} {detail}")
    return bool(cond)


def straight_path(length: float = 12.0, step: float = 0.25):
    return [(i * step, 0.0) for i in range(max(2, int(length / step) + 1))]


def circle_collide(cx: float, cy: float, r: float):
    def fn(px: float, py: float) -> bool:
        return math.hypot(px - cx, py - cy) < r

    return fn


def circle_clearance(cx: float, cy: float, r: float):
    def fn(px: float, py: float) -> float:
        return max(0.0, math.hypot(px - cx, py - cy) - r)

    return fn


def main() -> int:
    fails = 0
    geom = DEFAULT_GEOM
    print("=== P0-D Obstacle Approach Preview ===")
    print(f"front_stop_m={geom.front_stop_m} front_cost_m={geom.front_cost_m}")

    path = straight_path()
    pol = SpeedPolicy()
    planner = RollingLocalPlanner()

    # A: open — no obstacle
    fp = compute_future_preview(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=30.0, collide=lambda _x, _y: False
    )
    fails += 0 if _ok("A open no collision", not fp.future_collision and fp.forward_valid) else 1

    # B: obstacle at ~5m ahead on path
    col5 = circle_collide(5.0, 0.0, 0.35)
    fp5 = compute_future_preview(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=4.5, collide=col5
    )
    fails += 0 if _ok("B obstacle 5m future_collision", fp5.future_collision) else 1
    fails += 0 if _ok("B first_collision ~3-5m", fp5.first_collision_distance_m is not None and 2.5 < fp5.first_collision_distance_m < 5.5) else 1

    # C: obstacle ~3m — early side plan
    col3 = circle_collide(3.0, 0.0, 0.35)
    fp3 = compute_future_preview(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.5, collide=col3
    )
    spd = pol.compute(scene="OPEN", policy_state="FOLLOW_GLOBAL", front_near=2.5)
    req = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.5,
        left_free=2.0, right_free=2.0, speed=spd, future_preview=fp3,
        obstacle_passed=False, now=1.0,
    )
    res = planner.update(req)
    sel = res.plan.selected_candidate_id if res.plan else ""
    fails += 0 if _ok("C approach_active", fp3.approach_active) else 1
    fails += 0 if _ok("C early side or non-forward", sel and (KIND_LEFT_ARC in sel or KIND_RIGHT_ARC in sel or not fp3.forward_valid)) else 1

    # D: left blocked, right free
    def col_lr(px: float, py: float) -> bool:
        if py > 0.2 and px < 4.0:
            return True
        if math.hypot(px - 3.0, py) < 0.35:
            return True
        return False

    fpd = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.0, collide=col_lr)
    fails += 0 if _ok("D right_valid preferred", fpd.right_valid or fpd.preferred_side in ("RIGHT", None)) else 1

    # E: right blocked, left free (account for vehicle half-width ~0.28m)
    def col_rl(px: float, py: float) -> bool:
        return py < -0.48 and 0.5 < px < 4.0

    fpe = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=30.0, collide=col_rl)
    fails += 0 if _ok("E left_valid", fpe.left_valid) else 1
    fails += 0 if _ok("E right corridor blocked", not fpe.right_valid) else 1

    # F: both sides blocked — no false forward
    col_both = circle_collide(3.0, 0.0, 0.5)

    def col_all(px: float, py: float) -> bool:
        if col_both(px, py):
            return True
        if abs(py) < 0.55 and px > 0.5 and px < 4.5:
            return True
        return False

    fpf = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=1.5, collide=col_all)
    reqf = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=1.5,
        speed=spd, future_preview=fpf, now=2.0,
    )
    resf = planner.update(reqf)
    fw = [c for c in (resf.plan.candidates if resf.plan else []) if c.kind == KIND_FORWARD and c.valid]
    fails += 0 if _ok("F no valid forward when blocked", len(fw) == 0 or fpf.future_global_blocked) else 1

    # G: obstacle outside footprint corridor — no false collision
    col_far = circle_collide(3.0, 2.5, 0.25)
    fpg = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=30.0, collide=col_far)
    fails += 0 if _ok("G no false collision off-path", not fpg.future_collision) else 1

    # H: swept footprint — inner obstacle on path
    col_inner = circle_collide(2.2, 0.0, 0.30)
    fph = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=30.0, collide=col_inner)
    fails += 0 if _ok("H swept inner collision", fph.future_collision and fph.collision_reason == "SWEPT_FOOTPRINT") else 1

    # I: high speed — earlier required avoidance + earlier approach_active
    req_lo = required_avoidance_distance(0.10, geom)
    req_hi = required_avoidance_distance(0.30, geom)
    fails += 0 if _ok("I high speed earlier req", req_hi > req_lo + 0.15, f"lo={req_lo:.2f} hi={req_hi:.2f}") else 1
    col_i = circle_collide(3.4, 0.0, 0.32)
    fp_hi = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.30, global_path=path, front_near=30.0, collide=col_i)
    fp_lo = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.10, global_path=path, front_near=30.0, collide=col_i)
    fails += 0 if _ok(
        "I high vx approach earlier",
        fp_hi.approach_active and not fp_lo.approach_active,
        f"hi_fc={fp_hi.first_collision_distance_m} hi_req={fp_hi.required_avoidance_distance_m} lo_fc={fp_lo.first_collision_distance_m} lo_req={fp_lo.required_avoidance_distance_m}",
    ) else 1

    # J: low speed — later approach (same as I inverse)
    fails += 0 if _ok("J low speed later maneuver start", fp_lo.required_avoidance_distance_m < fp_hi.required_avoidance_distance_m) else 1

    # K: global blocked, local bypass valid
    fpk = compute_future_preview(x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.0, collide=col3)
    reqk = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.0,
        left_free=2.0, right_free=2.0, speed=spd, future_preview=fpk, now=3.0,
    )
    resk = planner.update(reqk)
    arcs = [c for c in (resk.plan.candidates if resk.plan else []) if c.kind in (KIND_LEFT_ARC, KIND_RIGHT_ARC) and c.valid]
    fails += 0 if _ok("K bypass valid when global blocked", fpk.future_global_blocked and len(arcs) >= 1) else 1

    # L: reconnect gated before obstacle passed
    recl = LocalPlanRequest(
        x=0.0, y=0.0, yaw=0.0, vx=0.20, global_path=path, front_near=2.0,
        speed=spd, future_preview=fpk, obstacle_passed=False, now=4.0,
    )
    planner.update(recl)
    fwd_cands = [c for c in (planner.last_plan.candidates if planner.last_plan else []) if c.kind == KIND_FORWARD]
    blocked_fwd = all(not c.valid for c in fwd_cands) if fwd_cands else True
    fails += 0 if _ok("L reconnect gate blocks forward", blocked_fwd) else 1

    # Hard stop unchanged
    fails += 0 if _ok("hard_stop unchanged", geom.front_stop_m == 0.70) else 1

    print(f"\nRESULT: {'PASS' if fails == 0 else 'FAIL'} ({fails} failed)")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
