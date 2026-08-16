#!/usr/bin/env python3
"""STEP 3C offline audit: Commitment KEEP / REVALIDATION (no web required)."""

from __future__ import annotations

import math
import sys
import time

from agv_bridge.local_maneuver import LocalManeuverSelector, DEC_LEFT, DEC_RIGHT, MIN_HOLD_S
from agv_bridge.nav_policy import NavigationPolicy, LOCAL_AVOID


def _path():
    return [(0.0, 0.0), (8.0, 0.0)]


def main() -> int:
    fails = []
    now = time.time()
    sel = LocalManeuverSelector()
    sel.current = DEC_LEFT
    sel.hold_until = now - 1.0  # past hold

    # Soft: LEFT feasible, RIGHT cheaper → COMMIT_KEEP
    def collide_ok(x, y):
        return False

    def clr_ok(x, y):
        return 2.0

    r = sel.compare(
        now=now,
        x=0.2,
        y=0.35,
        yaw=0.4,
        path=_path(),
        goal=(8.0, 0.0),
        front_near=0.55,
        rear_near=3.0,
        collide=collide_ok,
        clearance_at=clr_ok,
        forward_feasible=False,
        rotation_safe=True,
        left_free=1.2,
        right_free=2.5,
        force=True,
        require_capture=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=False,
    )
    if r.selected != "LEFT" or r.reason != "COMMIT_KEEP":
        fails.append(f"soft KEEP expected LEFT/COMMIT_KEEP got {r.selected}/{r.reason}")

    # Hard: LEFT infeasible, RIGHT feasible → keep LEFT + REVALIDATION (no RIGHT)
    pillar = (0.6, 0.45)

    def collide_hard(x, y):
        return math.hypot(x - pillar[0], y - pillar[1]) < 0.5

    def clr_hard(x, y):
        return max(0.02, math.hypot(x - pillar[0], y - pillar[1]) - 0.4)

    sel2 = LocalManeuverSelector()
    sel2.current = DEC_LEFT
    sel2.hold_until = now - 1.0
    r2 = sel2.compare(
        now=now + 1.0,
        x=0.15,
        y=0.35,
        yaw=0.5,
        path=_path(),
        goal=(8.0, 0.0),
        front_near=0.45,
        rear_near=3.0,
        collide=collide_hard,
        clearance_at=clr_hard,
        forward_feasible=False,
        rotation_safe=True,
        left_free=0.3,
        right_free=2.8,
        force=True,
        require_capture=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=False,
    )
    if r2.selected != "LEFT" or r2.reason != "COMMIT_REVALIDATION_REQUIRED":
        fails.append(
            f"hard REVALIDATION expected LEFT/COMMIT_REVALIDATION_REQUIRED got {r2.selected}/{r2.reason}"
        )
    if r2.selected == "RIGHT":
        fails.append("HARD FAIL must not select RIGHT in 3C")

    # Policy lifecycle create + hard fail flag
    pol = NavigationPolicy()
    pol.create_commitment(now=now, side="LEFT", reason="TEST", pose=(0.0, 0.0, 0.0), signature="t")
    if not pol.commitment.active or pol.commitment.side != "LEFT":
        fails.append("create_commitment failed")
    pol.update_commitment_after_selection(
        now=now + 0.2,
        selected="LEFT",
        left_feasible=False,
        right_feasible=True,
        left_cost=200.0,
        right_cost=40.0,
        policy_state=LOCAL_AVOID,
        maneuver_mode="LOCAL_LEFT",
        front_near=0.4,
        left_free=0.2,
        right_free=2.5,
        scene="TEST",
        pose=(0.2, 0.3, 0.4),
    )
    if not pol.commitment.hard_fail:
        fails.append("expected hard_fail after left infeasible")
    if pol.commitment.authorization_status != "REVALIDATION_REQUIRED":
        fails.append(f"auth status {pol.commitment.authorization_status}")
    # Soft cost worse while feasible
    pol2 = NavigationPolicy()
    pol2.create_commitment(now=now, side="LEFT", reason="TEST2")
    pol2.update_commitment_after_selection(
        now=now + 0.2,
        selected="LEFT",
        left_feasible=True,
        right_feasible=True,
        left_cost=80.0,
        right_cost=40.0,
        policy_state=LOCAL_AVOID,
        maneuver_mode="LOCAL_LEFT",
    )
    if pol2.commitment.failure_reason != "TEMPORARY_COST_WORSE":
        fails.append(f"soft fail reason {pol2.commitment.failure_reason}")
    if pol2.commitment.authorization_status != "LOCKED_BY_COMMITMENT":
        fails.append("soft should stay LOCKED")

    # Without commitment, RIGHT_ONLY still allowed (baseline)
    sel3 = LocalManeuverSelector()
    sel3.current = DEC_LEFT
    sel3.hold_until = now - 1.0
    r3 = sel3.compare(
        now=now + 2.0,
        x=0.15,
        y=0.35,
        yaw=0.5,
        path=_path(),
        goal=(8.0, 0.0),
        front_near=0.45,
        rear_near=3.0,
        collide=collide_hard,
        clearance_at=clr_hard,
        forward_feasible=False,
        rotation_safe=True,
        left_free=0.3,
        right_free=2.8,
        force=True,
        require_capture=False,
        commitment_active=False,
    )
    if r3.selected == "LEFT" and r3.reason == "COMMIT_REVALIDATION_REQUIRED":
        fails.append("without commitment should not emit COMMIT_REVALIDATION")

    if fails:
        print("RESULT FAIL")
        for f in fails:
            print(" ", f)
        return 1
    print("RESULT PASS")
    print("  soft COMMIT_KEEP ok")
    print("  hard COMMIT_REVALIDATION_REQUIRED ok (no RIGHT)")
    print("  policy hard_fail + soft LOCKED ok")
    print(f"  baseline without commit selected={r3.selected} reason={r3.reason}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
