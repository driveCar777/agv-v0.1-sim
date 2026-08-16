#!/usr/bin/env python3
"""STEP 3F-CORRECTIVE — Recovery execution audit (action → cmd → safe → state → pose)."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.maneuver import (  # noqa: E402
    ManeuverFSM,
    POLICY_REVERSE_VX,
    REVERSE_ESCAPE,
    TURN_IN_PLACE,
    REPOSITION,
)


def _decide_policy_reverse(**kw):
    fsm = ManeuverFSM()
    now = time.time()
    path = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
    d = fsm.decide(
        now=now,
        x=kw.get("x", 0.0),
        y=kw.get("y", 0.0),
        yaw=kw.get("yaw", 0.0),
        path=path,
        goal=(8.0, 0.0),
        front_near=kw.get("front_near", 0.35),
        rear_near=kw.get("rear_near", 2.5),
        collision=False,
        emergency=False,
        path_progress=0.1,
        lateral_error=0.0,
        stuck_s=kw.get("stuck_s", 4.0),
        recovery_attempts=0,
        collide=lambda *_: False,
        clearance_at=lambda *_: 1.5,
        actual_clearance=1.2,
        nav_active=True,
        policy_ctx={
            "allow_recovery": True,
            "allow_replan": True,
            "allow_side_compare": True,
            "recovery_action": "LOCAL_REVERSE",
            "recovery_target_distance_m": 1.0,
            "recovery_force_vx": POLICY_REVERSE_VX,
            "recovery_force_w": 0.0,
            "state": "LOCAL_AVOID",
            "behavior": "CAUTION",
        },
    )
    return fsm, d


def main() -> int:
    fails = []
    # Large heading error would previously force TURN_IN_PLACE
    fsm, d = _decide_policy_reverse(yaw=1.6, stuck_s=5.0, front_near=0.3)
    if d.mode != REVERSE_ESCAPE:
        fails.append(f"mode={d.mode} expected REVERSE_ESCAPE (not TURN/REPOSITION)")
    if d.force_vx is None or d.force_vx >= 0:
        fails.append(f"force_vx={d.force_vx} expected < 0")
    if d.force_w is not None and abs(d.force_w) > 0.13:
        fails.append(f"force_w={d.force_w} arbitrary yaw not allowed")
    if d.recovery_exec and d.recovery_exec.get("status") not in (
        "PLANNED",
        "EXECUTING",
        "PROGRESSING",
    ):
        fails.append(f"recovery_exec.status={d.recovery_exec}")

    # Progress: move backward along -heading of recovery start (yaw kept)
    now = time.time()
    start_yaw = 1.6
    bx = -math.cos(start_yaw) * 0.30
    by = -math.sin(start_yaw) * 0.30
    d2 = fsm.decide(
        now=now + 0.5,
        x=bx,
        y=by,
        yaw=start_yaw,
        path=[(0.0, 0.0), (3.0, 0.0)],
        goal=(8.0, 0.0),
        front_near=0.4,
        rear_near=2.5,
        collision=False,
        emergency=False,
        path_progress=0.1,
        lateral_error=0.0,
        stuck_s=5.0,
        recovery_attempts=1,
        collide=lambda *_: False,
        clearance_at=lambda *_: 1.5,
        actual_clearance=1.3,
        nav_active=True,
        policy_ctx={
            "allow_recovery": True,
            "allow_replan": True,
            "recovery_action": "LOCAL_REVERSE",
            "recovery_target_distance_m": 1.0,
            "recovery_force_vx": -0.12,
            "recovery_force_w": 0.0,
        },
    )
    prog = (d2.recovery_exec or {}).get("signed_progress_m", 0)
    if prog < 0.15:
        fails.append(f"signed_progress_m={prog} expected ~0.30 after reverse move")

    # SAFE_STOP when policy says so — no endless TURN
    fsm3 = ManeuverFSM()
    d3 = fsm3.decide(
        now=time.time(),
        x=0.0,
        y=0.0,
        yaw=1.5,
        path=[(0.0, 0.0), (2.0, 0.0)],
        goal=(4.0, 0.0),
        front_near=0.2,
        rear_near=0.2,
        collision=False,
        emergency=False,
        path_progress=0.0,
        lateral_error=0.0,
        stuck_s=6.0,
        recovery_attempts=3,
        collide=lambda *_: False,
        clearance_at=lambda *_: 0.3,
        actual_clearance=0.3,
        nav_active=True,
        policy_ctx={
            "allow_recovery": False,
            "allow_replan": True,
            "recovery_action": "SAFE_STOP",
        },
    )
    if d3.mode not in ("SAFE_STOP", "REPLAN"):
        fails.append(f"SAFE_STOP policy → mode={d3.mode} (must not be {TURN_IN_PLACE}/{REPOSITION})")
    if d3.force_vx and abs(d3.force_vx) > 1e-6 and d3.mode == "SAFE_STOP":
        fails.append("SAFE_STOP must command vx=0")

    print("=== Recovery Execution Audit ===")
    if fails:
        for f in fails:
            print("FAIL:", f)
        return 1
    print("PASS: LOCAL_REVERSE → REVERSE_ESCAPE + force_vx<0 + progress tracked")
    print(f"  force_vx={d.force_vx} force_w={d.force_w} mode={d.mode}")
    print(f"  progress after move={prog}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
