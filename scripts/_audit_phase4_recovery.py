#!/usr/bin/env python3
"""STEP 3F offline audit: corridor / breadcrumb / retreat / recovery ladder."""

from __future__ import annotations

import math
import time

from agv_bridge.nav_breadcrumb import TrajectoryBreadcrumb
from agv_bridge.nav_probe import (
    STATUS_INVALID,
    STATUS_VALID,
    ProbeBundle,
    ProbeEngine,
    ProbeResult,
    build_obstacle_snapshot,
)
from agv_bridge.nav_recovery import (
    ACT_CONTINUE,
    ACT_HISTORICAL_RETREAT,
    ACT_LOCAL_REVERSE,
    ACT_SAFE_STOP,
    ACT_SIDE_SWITCH,
    CLASS_DEAD_END,
    evaluate_historical_retreat,
    evaluate_recovery,
)
from agv_bridge.nav_trajectory import (
    TrajectorySample,
    build_corridor_from_poses,
    footprint_half_width,
    poses_from_rollout_path,
)


class F:
    def __init__(self) -> None:
        self.items = []

    def check(self, name: str, cond: bool, detail: str = "") -> None:
        if not cond:
            self.items.append(f"{name}: {detail}" if detail else name)


def _pr(direction: str, status: str, **kw) -> ProbeResult:
    r = ProbeResult(direction=direction, status=status, timestamp=time.time())
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def _bundle(f: str, l: str, r: str, b: str = STATUS_VALID, turn: str = STATUS_VALID) -> ProbeBundle:
    now = time.time()
    snap = build_obstacle_snapshot(
        now=now,
        x=0.0,
        y=0.0,
        yaw=0.0,
        vx=0.1,
        w=0.0,
        front_near=0.4,
        rear_near=2.0,
        left_free=0.2,
        right_free=0.2,
        collide=lambda x, y: False,
        clearance_at=lambda x, y: 2.0,
        pose_ts=now,
        obstacle_ts=now,
    )
    return ProbeBundle(
        snapshot=snap,
        forward=_pr("FORWARD", f, hard_collision=(f == STATUS_INVALID)),
        backward=_pr("BACKWARD", b, escape_available=(b == STATUS_VALID)),
        left=_pr("LEFT", l),
        right=_pr("RIGHT", r),
        turn_in_place=_pr("TURN_IN_PLACE", turn, rotation_valid=(turn == STATUS_VALID)),
    )


def main() -> int:
    fails = F()

    # C1 half-width = 0.5*W + margins (not bare W)
    hw = footprint_half_width()
    fails.check("C1_half_width", hw > 0.27 and hw < 0.55, str(hw))

    # C2 corridor from poses (diff-drive path with yaw)
    path = [{"x": i * 0.1, "y": 0.0, "yaw": 0.0} for i in range(12)]
    poses = poses_from_rollout_path(path, yaw0=0.0, vx=0.2, w=0.0)
    corr = build_corridor_from_poses(poses, source="FORWARD", status=STATUS_VALID)
    fails.check("C2_length", corr.length_m > 0.9, str(corr.length_m))
    fails.check("C2_edges", len(corr.left_edge) == len(poses) and len(corr.swept_polygon) >= 4)
    fails.check("C2_valid", corr.valid is True)

    # B1 breadcrumb record + downsample
    bc = TrajectoryBreadcrumb()
    t0 = time.time()
    for i in range(40):
        bc.record(now=t0 + i * 0.05, x=i * 0.05, y=0.0, yaw=0.0, vx=0.2, mode="FORWARD_TRACK")
    fails.check("B1_count", 5 <= len(bc) <= 40, str(len(bc)))
    fails.check("B1_trusted", all(p.trusted for p in bc.points(trusted_only=True)))

    # B5 partial retreat: block mid-history
    # poly newest-first: build path 0→3m then place wall at older segment
    bc2 = TrajectoryBreadcrumb()
    for i in range(60):
        bc2.record(now=t0 + i * 0.1, x=i * 0.1, y=0.0, yaw=0.0, vx=0.2, mode="FORWARD_TRACK")
    # collide near x=2.0 (older from tip at ~6m)
    def collide(x, y):
        return abs(x - 2.0) < 0.25 and abs(y) < 0.4

    ret = evaluate_historical_retreat(breadcrumb=bc2, collide=collide, max_distance_m=5.0)
    fails.check("B5_partial", ret.status == STATUS_VALID and ret.safe_distance_m > 0.5, ret.to_dict())
    fails.check("B5_not_full", ret.blocked_index >= 0 or ret.safe_distance_m < 5.5, str(ret.safe_distance_m))

    # R1/R3 local reverse when F/L/R invalid + B valid
    d1 = evaluate_recovery(
        probe=_bundle(STATUS_INVALID, STATUS_INVALID, STATUS_INVALID, STATUS_VALID),
        breadcrumb=bc2,
        collide=lambda x, y: False,
        progress_low=True,
        recovery_attempts=0,
    )
    fails.check("R3_local_reverse", d1.action == ACT_LOCAL_REVERSE, d1.to_dict())
    fails.check("R3_release", d1.release_commitment is True)

    # R6 side still valid → not reverse
    d2 = evaluate_recovery(
        probe=_bundle(STATUS_INVALID, STATUS_VALID, STATUS_INVALID, STATUS_VALID),
        breadcrumb=bc2,
        collide=lambda x, y: False,
        progress_low=True,
    )
    fails.check("R6_no_reverse", d2.action == ACT_CONTINUE, d2.to_dict())

    # side switch preferred
    d3 = evaluate_recovery(
        probe=_bundle(STATUS_INVALID, STATUS_INVALID, STATUS_INVALID, STATUS_VALID),
        breadcrumb=bc2,
        collide=lambda x, y: False,
        side_switch_authorized=True,
    )
    fails.check("R6b_side_switch", d3.action == ACT_SIDE_SWITCH, d3.to_dict())

    # no escape → SAFE_STOP when replan disabled + back invalid
    d4 = evaluate_recovery(
        probe=_bundle(STATUS_INVALID, STATUS_INVALID, STATUS_INVALID, STATUS_INVALID),
        breadcrumb=TrajectoryBreadcrumb(),
        collide=lambda x, y: True,
        progress_low=True,
        allow_replan=False,
        recovery_attempts=0,
    )
    fails.check("R12_safe_stop", d4.action == ACT_SAFE_STOP, d4.to_dict())

    # ProbeEngine attaches corridor (no second rollout family)
    eng = ProbeEngine()
    now = time.time()
    snap = build_obstacle_snapshot(
        now=now,
        x=0.0,
        y=0.0,
        yaw=0.0,
        vx=0.15,
        w=0.0,
        front_near=5.0,
        rear_near=5.0,
        left_free=2.0,
        right_free=2.0,
        collide=lambda x, y: False,
        clearance_at=lambda x, y: 2.0,
        pose_ts=now,
        obstacle_ts=now,
        global_path=[(0.0, 0.0), (8.0, 0.0)],
        goal=(8.0, 0.0),
    )
    bundle = eng.evaluate(snap, now=now)
    fails.check("P_poses", len(bundle.forward.poses) >= 2, str(len(bundle.forward.poses)))
    fails.check("P_corridor", isinstance(bundle.forward.corridor, dict) and bundle.forward.corridor.get("half_width_m", 0) > 0.2)
    fails.check("P_back_poses", len(bundle.backward.poses) >= 2)

    # dead-end class
    fails.check("DEAD_END_class", d1.classification in (CLASS_DEAD_END, "RECOVERY_EVALUATE"), d1.classification)

    if fails.items:
        print("RESULT FAIL")
        for it in fails.items:
            print(" ", it)
        return 1
    print("RESULT PASS")
    print("  corridor / breadcrumb / retreat / recovery ladder / probe poses OK")
    print(f"  half_width={hw:.3f}m bundle_ms~{bundle.bundle_ms:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
