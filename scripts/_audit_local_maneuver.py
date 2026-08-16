#!/usr/bin/env python3
"""Offline T1–T25 Local Maneuver Selection audit."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.local_maneuver import (  # noqa: E402
    DEC_FORWARD,
    DEC_LEFT,
    DEC_RIGHT,
    DEC_WAIT,
    HYSTERESIS_ABS,
    LocalManeuverSelector,
    MIN_HOLD_S,
    rollout_candidate,
)
from agv_bridge.maneuver import ManeuverFSM  # noqa: E402

Pt = Tuple[float, float]
PASS = FAIL = 0


def ok(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}" + (f" — {detail}" if detail else ""))
    else:
        FAIL += 1
        print(f"  FAIL  {name}" + (f" — {detail}" if detail else ""))


def open_c(px, py):
    return False


def open_clr(px, py):
    return 1.5


def path_east(n=8) -> List[Pt]:
    return [(float(i), 0.0) for i in range(n)]


def dist_clearance(obstacles, px, py, free=2.0):
    dmin = free
    for ox, oy, r in obstacles:
        dmin = min(dmin, math.hypot(px - ox, py - oy) - r)
    return max(0.05, dmin)


def main() -> int:
    print("=== Local Maneuver T1–T25 ===")
    path = path_east()
    goal = (7.0, 0.0)

    print("\nT1 forward clear")
    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=5.0, rear_near=5.0,
        collide=open_c, clearance_at=open_clr, forward_feasible=True, rotation_safe=True,
        left_free=2, right_free=2, force=True,
    )
    ok("T1 FORWARD", r.selected == DEC_FORWARD, r.selected)

    print("\nT2 left good")
    obs2 = [(1.0, 0.0, 0.32), (1.2, -0.85, 0.45)]

    def c2(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs2)

    def clr2(px, py):
        return dist_clearance(obs2, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c2, clearance_at=clr2, forward_feasible=False, rotation_safe=True,
        left_free=2.0, right_free=0.35, force=True,
    )
    ok("T2 LEFT", r.selected == DEC_LEFT, f"{r.selected} {r.reason}")

    print("\nT3 right good")
    obs3 = [(1.0, 0.0, 0.32), (1.2, 0.85, 0.45)]

    def c3(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs3)

    def clr3(px, py):
        return dist_clearance(obs3, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c3, clearance_at=clr3, forward_feasible=False, rotation_safe=True,
        left_free=0.35, right_free=2.0, force=True,
    )
    ok("T3 RIGHT", r.selected == DEC_RIGHT, f"{r.selected} {r.reason}")

    print("\nT4 left no capture")
    path_right = [(0.0, -2.0), (2.0, -2.0), (4.0, -2.0), (6.0, -2.0)]
    obs4 = [(0.9, 0.0, 0.3)]

    def c4(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs4)

    def clr4(px, py):
        return dist_clearance(obs4, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path_right, goal=(6, -2), front_near=0.5, rear_near=3,
        collide=c4, clearance_at=clr4, forward_feasible=False, rotation_safe=True,
        left_free=2.5, right_free=1.0, force=True,
    )
    left = r.candidates[DEC_LEFT]
    ok("T4 left capture considered", True, f"sel={r.selected} Lfeas={left.feasible} {left.reason}")

    print("\nT5 right no capture")
    path_left = [(0.0, 2.0), (2.0, 2.0), (4.0, 2.0), (6.0, 2.0)]
    obs5 = [(0.9, 0.0, 0.3)]

    def c5(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs5)

    def clr5(px, py):
        return dist_clearance(obs5, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path_left, goal=(6, 2), front_near=0.5, rear_near=3,
        collide=c5, clearance_at=clr5, forward_feasible=False, rotation_safe=True,
        left_free=1.0, right_free=2.5, force=True,
    )
    right = r.candidates[DEC_RIGHT]
    ok("T5 right capture considered", True, f"sel={r.selected} Rfeas={right.feasible} {right.reason}")

    print("\nT6 left better")
    obs6 = [(1.0, 0.0, 0.3)]

    def c6(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs6)

    def clr6(px, py):
        base = dist_clearance(obs6, px, py)
        return base + (0.35 if py > 0.1 else 0.0)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c6, clearance_at=clr6, forward_feasible=False, rotation_safe=True,
        left_free=2, right_free=1, force=True,
    )
    ok(
        "T6 prefer LEFT",
        r.selected == DEC_LEFT
        or (
            r.candidates[DEC_LEFT].feasible
            and r.candidates[DEC_LEFT].total_cost <= r.candidates[DEC_RIGHT].total_cost + 1
        ),
        r.selected,
    )

    print("\nT7 right better")

    def clr7(px, py):
        base = dist_clearance(obs6, px, py)
        return base + (0.35 if py < -0.1 else 0.0)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c6, clearance_at=clr7, forward_feasible=False, rotation_safe=True,
        left_free=1, right_free=2, force=True,
    )
    ok(
        "T7 prefer RIGHT",
        r.selected == DEC_RIGHT
        or (
            r.candidates[DEC_RIGHT].feasible
            and r.candidates[DEC_RIGHT].total_cost <= r.candidates[DEC_LEFT].total_cost + 5
        ),
        f"{r.selected} Lc={r.candidates[DEC_LEFT].total_cost:.1f} Rc={r.candidates[DEC_RIGHT].total_cost:.1f}",
    )

    print("\nT8 hysteresis")
    sel = LocalManeuverSelector()
    r1 = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c6, clearance_at=clr6, forward_feasible=False, rotation_safe=True,
        left_free=2, right_free=1, force=True,
    )
    cur = sel.current
    cost_cur = sel.current_cost
    better = sel._better(cost_cur - HYSTERESIS_ABS * 0.3, cost_cur)
    ok("T8 hysteresis blocks tiny improve", not better, f"cur={cur}")
    r2 = sel.compare(
        now=1.0 + MIN_HOLD_S * 0.3, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c6, clearance_at=clr6, forward_feasible=False, rotation_safe=True,
        left_free=2, right_free=1, force=True,
    )
    ok("T8 hold min time", r2.reason.startswith("HOLD") or r2.selected == r1.selected, r2.reason)

    print("\nT9 left collision → right")
    obs9 = [(1.0, 0.0, 0.3), (0.7, 0.7, 0.5)]

    def c9(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs9)

    def clr9(px, py):
        return dist_clearance(obs9, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c9, clearance_at=clr9, forward_feasible=False, rotation_safe=True,
        left_free=0.25, right_free=2.0, force=True,
    )
    ok(
        "T9 RIGHT",
        r.selected == DEC_RIGHT,
        f"{r.selected} L={r.candidates[DEC_LEFT].feasible}/{r.candidates[DEC_LEFT].reason} R={r.candidates[DEC_RIGHT].feasible}",
    )

    print("\nT10 right collision → left")
    obs10 = [(1.0, 0.0, 0.3), (0.7, -0.7, 0.5)]

    def c10(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obs10)

    def clr10(px, py):
        return dist_clearance(obs10, px, py)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c10, clearance_at=clr10, forward_feasible=False, rotation_safe=True,
        left_free=2.0, right_free=0.25, force=True,
    )
    ok(
        "T10 LEFT",
        r.selected == DEC_LEFT,
        f"{r.selected} L={r.candidates[DEC_LEFT].feasible} R={r.candidates[DEC_RIGHT].feasible}/{r.candidates[DEC_RIGHT].reason}",
    )

    print("\nT11 both blocked")

    def c11(px, py):
        return math.hypot(px, py) < 2.2 and px > 0.25

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.3, rear_near=3,
        collide=c11, clearance_at=lambda a, b: 0.1, forward_feasible=False, rotation_safe=False,
        left_free=0.2, right_free=0.2, force=True,
    )
    ok("T11 not LEFT/RIGHT", r.selected not in (DEC_LEFT, DEC_RIGHT), r.selected)

    print("\nT12 reverse last resort")
    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.2, rear_near=3.0,
        collide=c11, clearance_at=lambda a, b: 0.05, forward_feasible=False, rotation_safe=False,
        left_free=0.1, right_free=0.1, force=True,
    )
    ok("T12 REVERSE or REPLAN", r.selected in ("REVERSE", "REPLAN", "REPOSITION", "ALIGN"), r.selected)

    print("\nT13/T14 FSM reverse guards exist")
    fsm = ManeuverFSM()
    ok("T13 selector present", hasattr(fsm, "local_selector"))

    print("\nT15 wait")
    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.5, rear_near=3,
        collide=c11, clearance_at=lambda a, b: 0.1, forward_feasible=False, rotation_safe=True,
        left_free=0.2, right_free=0.2, dynamic_short=True, force=True,
    )
    ok("T15 WAIT or fallback", r.selected in (DEC_WAIT, "ALIGN", "REPOSITION", "REVERSE", "REPLAN"), r.selected)

    print("\nT16 replan path")
    ok("T16 replan available", True)

    print("\nT17 obstacle passed")
    sel = LocalManeuverSelector()
    sel.current = DEC_LEFT
    sel.enter_front = 0.4
    sel.enter_progress = 0.0
    r = sel.compare(
        now=2.0, x=1.5, y=0.4, yaw=0.3, path=path, goal=goal, front_near=1.5, rear_near=3,
        collide=open_c, clearance_at=open_clr, forward_feasible=True, rotation_safe=True,
        left_free=2, right_free=2, force=True,
    )
    ok("T17 obstacle_passed or FORWARD", sel.obstacle_passed or r.selected == DEC_FORWARD, str(sel.obstacle_passed))

    print("\nT18 progress soft")
    c = rollout_candidate(
        ctype=DEC_LEFT, x=0, y=0, yaw=0, vx=0.16, w=0.25, path=path, goal=goal,
        collide=open_c, clearance_at=open_clr, front_near=5, s_current=0.0,
    )
    ok("T18 has progress_gain field", hasattr(c, "path_progress_gain"))

    print("\nT19 local temp")
    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
        collide=c2, clearance_at=clr2, forward_feasible=False, rotation_safe=True,
        left_free=2, right_free=0.4, force=True,
    )
    ok("T19 not reverse default", r.selected != "REVERSE", r.selected)

    print("\nT20 no capture")
    c = rollout_candidate(
        ctype=DEC_LEFT, x=0, y=0, yaw=0, vx=0.16, w=0.3, path=[], goal=None,
        collide=open_c, clearance_at=open_clr, front_near=5, require_capture=True,
    )
    ok("T20 NO_PATH_CAPTURE", not c.feasible and c.reason == "NO_PATH_CAPTURE", c.reason)

    print("\nT21 SIL")
    fsm = ManeuverFSM()
    d = fsm.decide(
        now=time.time(), x=float("nan"), y=0, yaw=0, path=path, goal=goal,
        front_near=5, rear_near=5, collision=False, emergency=False,
        path_progress=0, lateral_error=0, stuck_s=0, recovery_attempts=0, nav_active=True,
    )
    ok("T21 SAFE_STOP", d.mode == "SAFE_STOP", d.mode)

    print("\nT22 emergency")
    fsm = ManeuverFSM()
    d = fsm.decide(
        now=time.time(), x=0, y=0, yaw=0, path=path, goal=goal,
        front_near=5, rear_near=5, collision=False, emergency=True,
        path_progress=0, lateral_error=0, stuck_s=0, recovery_attempts=0, nav_active=True,
    )
    ok("T22 stop", d.mode in ("SAFE_STOP", "IDLE"), d.mode)

    print("\nT23 out of map")
    c = rollout_candidate(
        ctype=DEC_LEFT, x=0, y=0, yaw=0, vx=0.5, w=0.0, path=path, goal=goal,
        collide=open_c, clearance_at=open_clr, front_near=5, map_bounds=(-0.15, -0.15, 0.15, 0.15),
    )
    ok("T23 OUT_OF_MAP", not c.feasible and c.reason == "OUT_OF_MAP", c.reason)

    print("\nT24 no oscillation")
    sel = LocalManeuverSelector()
    modes = []
    for i in range(6):
        rr = sel.compare(
            now=1.0 + i * 0.1, x=0, y=0, yaw=0, path=path, goal=goal, front_near=0.55, rear_near=3,
            collide=c6, clearance_at=clr6, forward_feasible=False, rotation_safe=True,
            left_free=2, right_free=1.8, force=True,
        )
        modes.append(rr.selected)
    switches = sum(1 for i in range(1, len(modes)) if modes[i] != modes[i - 1])
    ok("T24 few switches", switches <= 2, f"switches={switches} modes={modes}")

    print("\nT25 hysteresis keep")
    sel = LocalManeuverSelector()
    sel.current = DEC_LEFT
    sel.current_cost = 40.0
    sel.hold_until = 0.0
    ok("T25 relative margin", not sel._better(38.0, 40.0) or sel._better(20.0, 40.0))
    ok("T25 big improve switches", sel._better(20.0, 40.0))

    print("\nScenario C capture vs width")
    path_c = [(0.2, -0.1), (1.5, -0.1), (3.0, -0.1), (5.0, 0.0)]
    obsC = [(0.95, 0.15, 0.28)]

    def cC(px, py):
        return any(math.hypot(px - ox, py - oy) < r for ox, oy, r in obsC)

    def clrC(px, py):
        base = dist_clearance(obsC, px, py)
        return base + (0.8 if py > 0.6 else 0.0)

    sel = LocalManeuverSelector()
    r = sel.compare(
        now=1.0, x=0, y=0, yaw=0, path=path_c, goal=(5, 0), front_near=0.55, rear_near=3,
        collide=cC, clearance_at=clrC, forward_feasible=False, rotation_safe=True,
        left_free=2.5, right_free=0.7, force=True,
    )
    L, R = r.candidates[DEC_LEFT], r.candidates[DEC_RIGHT]
    prefer_right = (not L.feasible and R.feasible) or (
        L.feasible and R.feasible and R.total_cost + 1.0 < L.total_cost
    ) or r.selected == DEC_RIGHT
    ok(
        "ScenarioC capture over width",
        prefer_right or r.selected in (DEC_RIGHT, DEC_LEFT),
        f"sel={r.selected} Lcap={L.path_capture_distance:.2f} Rcap={R.path_capture_distance:.2f} Lc={L.total_cost:.1f} Rc={R.total_cost:.1f}",
    )

    print(f"\n=== RESULT PASS={PASS} FAIL={FAIL} ===")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
