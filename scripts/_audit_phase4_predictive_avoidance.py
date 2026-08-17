#!/usr/bin/env python3
"""P0-D.1 offline — phased avoidance, side probe, dynamic resume."""

from __future__ import annotations

import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.nav_avoidance_phase import (  # noqa: E402
    AvoidancePhaseTracker,
    PHASE_FUTURE_PREVIEW,
    PHASE_OPEN,
    PHASE_SIDE_COMMIT,
    PHASE_SIDE_PROBE,
    SIGNAL_MANEUVER_READY,
    SIGNAL_PREDICTED,
    compute_tier_distances,
)
from agv_bridge.nav_dynamic_resume import DynamicResumeTracker, diagnose_resume_block  # noqa: E402
from agv_bridge.nav_obstacle_preview import compute_future_preview  # noqa: E402
from agv_bridge.nav_side_probe import run_side_probe  # noqa: E402
from agv_bridge.nav_speed_policy import REASON_FUTURE_PREVIEW, REASON_SIDE_PROBE, SpeedPolicy  # noqa: E402


def _ok(name: str, cond: bool, detail: str = "") -> bool:
    mark = "PASS" if cond else "FAIL"
    print(f"  {mark} {name} {detail}")
    return bool(cond)


def straight_path(n: int = 40):
    return [(i * 0.25, 0.0) for i in range(n)]


def circle_collide(cx: float, cy: float, r: float):
    return lambda px, py: math.hypot(px - cx, py - cy) < r


def main() -> int:
    fails = 0
    path = straight_path()
    print("=== P0-D.1 Predictive Avoidance / Side Probe ===")

    # A: tier ordering
    tiers = compute_tier_distances(0.20, 5.0)
    fails += 0 if _ok(
        "A tier order",
        tiers["d_detection_m"] > tiers["d_probe_start_m"] > tiers["d_commit_m"] > tiers["d_hard_stop_m"],
        str(tiers),
    ) else 1

    # B: 5m detect != immediate turn
    col5 = circle_collide(5.0, 0.0, 0.35)
    fp5 = compute_future_preview(x=0, y=0, yaw=0, vx=0.20, global_path=path, collide=col5)
    sp5 = run_side_probe(x=0, y=0, yaw=0, vx=0.20, horizon_m=fp5.preview_distance_m, collide=col5, probe_active=True)
    tr = AvoidancePhaseTracker()
    av5 = tr.update(
        now=1.0, vx=0.20, preview_m=fp5.preview_distance_m, future_collision=fp5.future_collision,
        first_collision_m=fp5.first_collision_distance_m, front_near=30.0, left_free=2, right_free=2,
        lateral_error=0, side_probe_active=sp5.probe_active, probe_confidence_left=sp5.left_confidence,
        probe_confidence_right=sp5.right_confidence, preferred_side=sp5.preferred_side,
        commit_ready=sp5.commit_ready, committed_side_external=None, obstacle_passed_external=False,
        commitment_active=False,
    )
    fails += 0 if _ok("B 5m detected", fp5.future_collision and fp5.first_collision_distance_m and fp5.first_collision_distance_m > 3.5) else 1
    fails += 0 if _ok("B not commit at 5m", av5.phase in (PHASE_FUTURE_PREVIEW, PHASE_OPEN, PHASE_SIDE_PROBE) and not sp5.commit_ready) else 1

    # C: soft slowdown
    pol = SpeedPolicy()
    s_open = pol.compute(avoidance_phase="OPEN")
    s_prev = pol.compute(avoidance_phase="FUTURE_PREVIEW", probe_active=True)
    s_probe = pol.compute(avoidance_phase="SIDE_PROBE", probe_active=True)
    fails += 0 if _ok("C soft slowdown", s_open.target_vx > s_prev.target_vx > s_probe.target_vx, f"{s_open.target_vx}>{s_prev.target_vx}>{s_probe.target_vx}") else 1
    fails += 0 if _ok("C speed reasons", s_prev.reason == REASON_FUTURE_PREVIEW and s_probe.reason == REASON_SIDE_PROBE) else 1

    # D: multi-kappa probe
    col3 = circle_collide(3.0, 0.0, 0.35)
    sp3 = run_side_probe(x=0, y=0, yaw=0, vx=0.20, horizon_m=5.0, collide=col3, probe_active=True)
    fails += 0 if _ok("D multi kappa", len(sp3.left_arcs) >= 4 and len(sp3.right_arcs) >= 4) else 1

    # E: dynamic resume
    dr = DynamicResumeTracker()
    dr.update(now=1.0, dynamic_short=True, dynamic_long=False, policy_state="WAIT_FOR_CLEARANCE",
              policy_reason="DYNAMIC_SHORT→WAIT", maneuver_mode="WAIT_FOR_CLEARANCE",
              front_near=1.0, forward_feasible=False, safety_zero=False, local_plan_valid=False)
    st2 = dr.update(now=2.0, dynamic_short=False, dynamic_long=False, policy_state="WAIT_FOR_CLEARANCE",
                    policy_reason="DYNAMIC_SHORT→WAIT", maneuver_mode="WAIT_FOR_CLEARANCE",
                    front_near=2.0, forward_feasible=True, safety_zero=False, local_plan_valid=True)
    diag = diagnose_resume_block(st2)
    fails += 0 if _ok("E dynamic resume", st2.resume_allowed or st2.dynamic_state in ("RESUMING", "CLEAR")) else 1
    fails += 0 if _ok("E resume diag", isinstance(diag.get("checks"), dict)) else 1

    # F: commit threshold
    col_close = circle_collide(2.5, 0.0, 0.35)
    fp_c = compute_future_preview(x=0, y=0, yaw=0, vx=0.20, global_path=path, collide=col_close)
    sp_c = run_side_probe(x=0, y=0, yaw=0, vx=0.20, horizon_m=5.0, collide=col_close, probe_active=True)
    av_c = tr.update(
        now=3.0, vx=0.20, preview_m=5.0, future_collision=True,
        first_collision_m=fp_c.first_collision_distance_m, front_near=2.0, left_free=2, right_free=2,
        lateral_error=0, side_probe_active=True, probe_confidence_left=sp_c.left_confidence,
        probe_confidence_right=sp_c.right_confidence, preferred_side=sp_c.preferred_side,
        commit_ready=sp_c.commit_ready, committed_side_external=None, obstacle_passed_external=False,
        commitment_active=False,
    )
    fails += 0 if _ok("F closer → maneuver ready signal", av_c.signal in (SIGNAL_MANEUVER_READY, SIGNAL_PREDICTED) or av_c.phase in (PHASE_SIDE_PROBE, PHASE_SIDE_COMMIT)) else 1

    print(f"\nRESULT: {'PASS' if fails == 0 else 'FAIL'} ({fails} failed)")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
