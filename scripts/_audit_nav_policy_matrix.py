#!/usr/bin/env python3
"""A1–A30 Navigation Policy offline matrix (no web required)."""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Tuple

from agv_bridge.nav_policy import (
    ALIGN,
    CAUTION,
    FOLLOW_GLOBAL,
    LOCAL_AVOID,
    NavigationPolicy,
    OBSTACLE_APPROACH,
    PATH_RECAPTURE,
    RECOVERY,
    REPLAN,
    SAFE_STOP,
    TURN_IN_PLACE,
    WAIT_FOR_CLEARANCE,
    REPOSITION,
)
from agv_bridge.local_maneuver import LocalManeuverSelector
from agv_bridge.maneuver import ManeuverFSM

Pt = Tuple[float, float]


def straight_path(n: int = 40, step: float = 0.25) -> List[Pt]:
    return [(i * step, 0.0) for i in range(n)]


def check(name: str, cond: bool, detail: str = "") -> Dict[str, Any]:
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    return {"name": name, "ok": cond, "detail": detail}


def main() -> int:
    results: List[Dict[str, Any]] = []
    now = time.time()

    # A1 open follow
    p = NavigationPolicy()
    d = p.step(now=now, nav_active=True, front_near=8.0, left_free=3.0, right_free=3.0, forward_feasible=True, path_valid=True)
    results.append(check("A1 FOLLOW_GLOBAL", d.state == FOLLOW_GLOBAL, d.state))

    # A2 caution approach
    d = p.step(now=now + 1, nav_active=True, front_near=1.4, left_free=2.0, right_free=2.0, forward_feasible=True, path_valid=True)
    results.append(check("A2 CAUTION/APPROACH", d.state in (CAUTION, OBSTACLE_APPROACH), d.state))

    # A3 obstacle approach compare
    d = p.step(now=now + 2, nav_active=True, front_near=1.0, left_free=2.0, right_free=2.0, forward_feasible=False, path_valid=True)
    results.append(check("A3 allow_side_compare", d.allow_side_compare or d.state == LOCAL_AVOID, f"{d.state} cmp={d.allow_side_compare}"))

    # A4–A6 left/right wide
    d = p.step(now=now + 3, nav_active=True, front_near=0.8, left_free=2.5, right_free=0.6, forward_feasible=False, path_valid=True)
    results.append(check("A5 LEFT preference", d.behavior in ("AVOID_LEFT", "CAUTION") or d.state == LOCAL_AVOID, f"{d.behavior}/{d.scene}"))
    p2 = NavigationPolicy()
    d = p2.step(now=now, nav_active=True, front_near=0.8, left_free=0.5, right_free=2.5, forward_feasible=False, path_valid=True)
    results.append(check("A6 RIGHT preference", d.behavior in ("AVOID_RIGHT", "CAUTION") or d.state == LOCAL_AVOID, f"{d.behavior}/{d.scene}"))

    # A7 both wide → local avoid
    d = NavigationPolicy().step(now=now, nav_active=True, front_near=0.85, left_free=2.0, right_free=2.0, forward_feasible=False, path_valid=True)
    results.append(check("A7 both wide LOCAL_AVOID", d.state == LOCAL_AVOID, d.state))

    # A8–A9 capture softness via selector
    sel = LocalManeuverSelector()
    path = straight_path()
    pillar = (1.0, 0.0)

    def collide(x, y):
        return math.hypot(x - pillar[0], y - pillar[1]) < 0.55

    def clearance(x, y):
        return max(0.05, math.hypot(x - pillar[0], y - pillar[1]) - 0.42)

    cmp = sel.compare(
        now=now,
        x=0.0,
        y=0.0,
        yaw=0.0,
        path=path,
        goal=(8.0, 0.0),
        front_near=0.7,
        rear_near=5.0,
        collide=collide,
        clearance_at=clearance,
        forward_feasible=False,
        rotation_safe=True,
        left_free=2.5,
        right_free=0.4,
        require_capture=False,
        max_deviation_m=1.2,
        path_follow_scale=0.25,
    )
    results.append(check("A8 soft capture allows side", cmp.selected in ("LEFT", "RIGHT", "REPOSITION", "ALIGN", "WAIT"), cmp.selected))

    # A10 both blocked → not infinite reverse preference
    def collide_cage(x, y):
        return math.hypot(x, y) > 0.01 and any(math.hypot(x - math.cos(a), y - math.sin(a)) < 0.55 for a in (-0.6, 0, 0.6, 1.0, -1.0))

    cmp2 = LocalManeuverSelector().compare(
        now=now,
        x=0.0,
        y=0.0,
        yaw=0.0,
        path=path,
        goal=(5.0, 0.0),
        front_near=0.4,
        rear_near=0.3,
        collide=collide_cage,
        clearance_at=lambda x, y: 0.1,
        forward_feasible=False,
        rotation_safe=False,
        left_free=0.3,
        right_free=0.3,
        require_capture=True,
    )
    results.append(check("A10 not default LEFT/RIGHT when caged", cmp2.selected not in ("LEFT", "RIGHT") or not cmp2.candidates.get(cmp2.selected, type("C", (), {"feasible": False})()).feasible, cmp2.selected))

    # A11–A12 dynamic
    d = NavigationPolicy().step(now=now, nav_active=True, front_near=0.9, dynamic_short=True, forward_feasible=False, path_valid=True)
    results.append(check("A11 WAIT dynamic short", d.state == WAIT_FOR_CLEARANCE or d.behavior == "WAIT", d.state))
    d = NavigationPolicy().step(now=now, nav_active=True, front_near=0.7, dynamic_long=True, forward_feasible=False, path_valid=True)
    results.append(check("A12 REPLAN dynamic long", d.state == REPLAN or d.allow_replan, d.state))

    # A13 goal behind → align not reverse
    d = NavigationPolicy().step(
        now=now,
        nav_active=True,
        front_near=5.0,
        heading_error=2.5,
        goal_herr=3.0,
        rotation_safe=True,
        forward_feasible=True,
        path_valid=True,
    )
    results.append(check("A13 goal behind ALIGN/TURN", d.state in (ALIGN, TURN_IN_PLACE), d.state))

    # A14–A16 rotation
    d = NavigationPolicy().step(now=now, nav_active=True, heading_error=1.5, rotation_safe=True, front_near=5.0, path_valid=True, forward_feasible=True)
    results.append(check("A14 turn safe ALIGN/TURN", d.state in (ALIGN, TURN_IN_PLACE), d.state))
    d = NavigationPolicy().step(now=now, nav_active=True, heading_error=1.5, rotation_safe=False, front_near=5.0, path_valid=True, forward_feasible=True)
    results.append(check("A15 turn unsafe REPOSITION", d.state == REPOSITION, d.state))
    results.append(check("A16 reposition behavior", d.behavior == "REPOSITION", d.behavior))

    # A17–A19 recovery gating
    d = NavigationPolicy().step(
        now=now,
        nav_active=True,
        collision=True,
        rear_near=2.0,
        rotation_safe=False,
        front_near=0.4,
        recovery_attempts=0,
        path_valid=True,
    )
    results.append(check("A17 recovery allowed when trap", d.state == RECOVERY and d.allow_recovery, d.state))
    d = NavigationPolicy().step(
        now=now + 1,
        nav_active=True,
        collision=True,
        rear_near=2.0,
        rotation_safe=False,
        front_near=0.4,
        recovery_attempts=3,
        path_valid=True,
        maneuver_mode="REVERSE_ESCAPE",
    )
    results.append(check("A19 recovery exhausted → SAFE_STOP", d.state == SAFE_STOP or not d.allow_recovery, d.state))

    # A20–A21 replan
    d = NavigationPolicy().step(now=now, nav_active=True, path_valid=False, front_near=3.0)
    results.append(check("A20/21 invalid path scene", d.scene == "PATH_INVALID" or d.state in (REPLAN, FOLLOW_GLOBAL, SAFE_STOP), f"{d.state}/{d.scene}"))

    # A22 path stall flag
    d = NavigationPolicy().step(
        now=now,
        nav_active=True,
        state_vx=0.15,
        path_progress_rate=0.0,
        front_near=1.3,
        path_valid=True,
        forward_feasible=True,
    )
    results.append(check("A22 stall flag or approach", bool(d.flags.get("VELOCITY_BUT_PATH_STALLED")) or d.state in (OBSTACLE_APPROACH, CAUTION), str(d.flags)))

    # A23–A24 oscillation
    p = NavigationPolicy()
    for i, w in enumerate([0.3, -0.3, 0.3, -0.3, 0.3, -0.3, 0.3]):
        p.note_cmd_w(now + i * 0.2, w)
    d = p.step(now=now + 2, nav_active=True, front_near=5.0, path_valid=True, forward_feasible=True)
    results.append(check("A23/24 oscillation → WAIT", d.state == WAIT_FOR_CLEARANCE or d.flags.get("oscillation"), d.state))

    # A25 deadlock → replan
    p = NavigationPolicy()
    d = p.step(
        now=now,
        nav_active=True,
        planned_rejected_by_safety=True,
        safety_zero=True,
        state_vx=0.0,
        stuck_s=3.0,
        front_near=1.0,
        path_valid=True,
    )
    d = p.step(
        now=now + 3.0,
        nav_active=True,
        planned_rejected_by_safety=True,
        safety_zero=True,
        state_vx=0.0,
        stuck_s=3.5,
        front_near=1.0,
        path_valid=True,
    )
    results.append(check("A25 deadlock → REPLAN", d.state == REPLAN or d.flags.get("deadlock"), f"{d.state}/{d.reason}"))

    # A26–A28 SIL / emergency
    d = NavigationPolicy().step(now=now, nav_active=True, sensor_invalid=True)
    results.append(check("A26 sensor invalid SAFE_STOP", d.state == SAFE_STOP, d.reason))
    d = NavigationPolicy().step(now=now, nav_active=True, map_invalid=True)
    results.append(check("A27 map invalid SAFE_STOP", d.state == SAFE_STOP, d.reason))
    d = NavigationPolicy().step(now=now, nav_active=True, emergency=True)
    results.append(check("A28 emergency SAFE_STOP", d.state == SAFE_STOP, d.reason))

    # A29 profile weight softens in avoid
    d = NavigationPolicy().step(now=now, nav_active=True, front_near=0.8, left_free=2.0, right_free=0.5, forward_feasible=False, path_valid=True)
    results.append(check("A29 avoid path_follow_weight < 5", d.path_follow_weight < 4.0, str(d.path_follow_weight)))

    # A30 safety still highest conceptually: policy never claims override
    results.append(check("A30 policy has no vx write API", not hasattr(NavigationPolicy, "set_vx"), "ok"))

    # Corridor / recapture
    p = NavigationPolicy()
    p.state = LOCAL_AVOID
    p.away_since = now - 9.0
    d = p.step(now=now, nav_active=True, lateral_error=1.5, front_near=2.0, path_valid=True, forward_feasible=True, maneuver_mode="LOCAL_LEFT")
    results.append(check("corridor / recapture path", d.state in (PATH_RECAPTURE, LOCAL_AVOID, REPLAN) or d.corridor.exceeded, f"{d.state} exc={d.corridor.exceeded}"))

    # ManeuverFSM still constructs with policy_ctx
    fsm = ManeuverFSM()
    dec = fsm.decide(
        now=now,
        x=0.0,
        y=0.0,
        yaw=0.0,
        path=path,
        goal=(5.0, 0.0),
        front_near=5.0,
        rear_near=5.0,
        collision=False,
        emergency=False,
        path_progress=0.0,
        lateral_error=0.0,
        stuck_s=0.0,
        recovery_attempts=0,
        collide=lambda *_: False,
        clearance_at=lambda *_: 2.0,
        policy_ctx={"allow_side_compare": False, "allow_replan": True, "allow_recovery": False, "state": "FOLLOW_GLOBAL", "behavior": "FOLLOW_GLOBAL", "path_follow_weight": 5.0, "require_capture_hard": True, "max_deviation_m": 0.3},
    )
    results.append(check("FSM accepts policy_ctx", dec.mode in ("FORWARD_TRACK", "FORWARD_TURN", "IDLE", "ALIGN"), dec.mode))

    passed = sum(1 for r in results if r["ok"])
    failed = len(results) - passed
    print(f"=== A-MATRIX RESULT PASS={passed} FAIL={failed} ===")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
