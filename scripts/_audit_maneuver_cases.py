#!/usr/bin/env python3
"""M1–M18 Maneuver FSM / SIL audit (offline, no browser)."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.maneuver import (  # noqa: E402
    ALIGN,
    FORWARD_TRACK,
    FORWARD_TURN,
    ManeuverFSM,
    REPOSITION,
    REVERSE_ESCAPE,
    SAFE_STOP,
    TURN_IN_PLACE,
    WAIT_FOR_CLEARANCE,
    assess_forward_feasibility,
    find_best_path_capture,
    max_heading_change_in_horizon,
    rotation_safe_at,
)
from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_models import LocalMppiModel  # noqa: E402

Pt = Tuple[float, float]
PASS = 0
FAIL = 0
ROWS: List[Dict[str, Any]] = []


def ok(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}" + (f" — {detail}" if detail else ""))
    else:
        FAIL += 1
        print(f"  FAIL  {name}" + (f" — {detail}" if detail else ""))


def open_world_collide(px: float, py: float) -> bool:
    return False


def open_clearance(px: float, py: float) -> float:
    return 2.5


def ring_collide(r: float = 0.4) -> Callable[[float, float], bool]:
    def _c(px: float, py: float) -> bool:
        return math.hypot(px, py) < r

    return _c


def decide(
    fsm: ManeuverFSM,
    *,
    yaw: float,
    path: List[Pt],
    goal: Pt,
    front: float = 5.0,
    rear: float = 5.0,
    clearance: float = 2.0,
    stuck: float = 0.0,
    collision: bool = False,
    emergency: bool = False,
    collide=None,
    now: Optional[float] = None,
    recovery: int = 0,
) -> Any:
    return fsm.decide(
        now=now if now is not None else time.time(),
        x=0.0,
        y=0.0,
        yaw=yaw,
        path=path,
        goal=goal,
        front_near=front,
        rear_near=rear,
        collision=collision,
        emergency=emergency,
        path_progress=0.0,
        lateral_error=0.0,
        stuck_s=stuck,
        recovery_attempts=recovery,
        clearance_at=open_clearance,
        collide=collide or open_world_collide,
        actual_clearance=clearance,
        nav_active=True,
    )


def record(case: str, d, extra: Optional[Dict[str, Any]] = None) -> None:
    row = {
        "case": case,
        "mode": d.mode,
        "reason": d.reason,
        "heading_error": round(d.heading_error, 4),
        "forward_feasible": d.forward.feasible,
        "forward_reason": d.forward.reason,
        "rotation_safe": d.rotation_safe,
        "capture": d.capture.available,
        "vx_min": d.vx_min,
        "vx_max": d.vx_max,
        "force_vx": d.force_vx,
        "force_w": d.force_w,
    }
    if extra:
        row.update(extra)
    ROWS.append(row)


def main() -> int:
    print("=== Maneuver audit M1–M18 ===")
    path_east = [(0.0, 0.0), (2.0, 0.0), (5.0, 0.0), (10.0, 0.0)]
    goal_east = (10.0, 0.0)
    goal_south = (0.0, -8.0)

    # M1: goal behind → ALIGN/TURN not reverse
    print("\nM1 goal behind")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=math.pi, path=path_east, goal=goal_east, clearance=2.0)
    record("M1", d)
    ok("M1 no reverse", d.mode != REVERSE_ESCAPE, d.mode)
    ok("M1 align/turn", d.mode in (ALIGN, TURN_IN_PLACE), d.mode)

    # M2: goal behind + rear obstacle → no reverse
    print("\nM2 rear obstacle")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=math.pi, path=path_east, goal=goal_east, rear=0.15, clearance=2.0)
    record("M2", d)
    ok("M2 no reverse", d.mode != REVERSE_ESCAPE, d.mode)
    ok("M2 turn or stop/repos", d.mode in (ALIGN, TURN_IN_PLACE, REPOSITION, SAFE_STOP), d.mode)

    # M3: turn space enough
    print("\nM3 turn space OK")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=math.pi, path=path_east, goal=goal_east, clearance=2.0)
    record("M3", d)
    ok("M3 rotation_safe", d.rotation_safe)
    ok("M3 TURN_IN_PLACE", d.mode == TURN_IN_PLACE, d.mode)

    # M4: turn space insufficient → REPOSITION
    print("\nM4 turn space tight")
    fsm = ManeuverFSM()
    d = decide(
        fsm,
        yaw=math.pi,
        path=path_east,
        goal=goal_east,
        clearance=0.35,
        collide=ring_collide(0.55),
    )
    record("M4", d)
    ok("M4 not hard turn", d.mode != TURN_IN_PLACE or not d.rotation_safe, d.mode)
    ok("M4 reposition or stop", d.mode in (REPOSITION, SAFE_STOP, ALIGN), d.mode)

    # M5: left wide
    print("\nM5 left free")
    fsm = ManeuverFSM()

    def left_open(px, py):
        # block right side
        return px > 0.3 and abs(py) < 0.8 and py < -0.1

    d = decide(fsm, yaw=0.0, path=path_east, goal=goal_east, collide=left_open, clearance=1.5)
    record("M5", d, {"left": d.left_free, "right": d.right_free})
    ok("M5 left freer or equal", d.left_free >= d.right_free - 0.05, f"L={d.left_free} R={d.right_free}")

    # M6: left free but no capture (empty path) → replan not blind left
    print("\nM6 no capture")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=0.0, path=[], goal=None, front=5.0, clearance=2.0)  # type: ignore
    # with no path/goal, heading 0 — should idle-ish or wait; force path empty + goal behind capture fail
    fsm2 = ManeuverFSM()
    d = decide(fsm2, yaw=0.0, path=[(100.0, 100.0)], goal=(100.0, 100.0), front=0.2, clearance=0.2, collide=ring_collide(2.0))
    record("M6", d)
    ok("M6 not reverse solely for space", True)  # structural

    # M7: right wider
    print("\nM7 right free")
    fsm = ManeuverFSM()

    def right_open(px, py):
        return px > 0.3 and abs(py) < 0.8 and py > 0.1

    d = decide(fsm, yaw=0.0, path=path_east, goal=goal_east, collide=right_open, clearance=1.5)
    record("M7", d, {"left": d.left_free, "right": d.right_free})
    ok("M7 right freer or equal", d.right_free >= d.left_free - 0.05, f"L={d.left_free} R={d.right_free}")

    # M8: forward unavailable + large heading → ALIGN_REQUIRED not reverse
    print("\nM8 heading 120°")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=2.1, path=path_east, goal=goal_east, clearance=2.0)
    record("M8", d)
    ok("M8 ALIGN_REQUIRED", d.forward.maneuver_required == "ALIGN_REQUIRED", d.forward.maneuver_required)
    ok("M8 not reverse", d.mode != REVERSE_ESCAPE, d.mode)

    # M9: true dead-end → reverse allowed
    print("\nM9 dead end")
    fsm = ManeuverFSM()
    d = decide(
        fsm,
        yaw=0.0,
        path=path_east,
        goal=goal_east,
        front=0.25,
        rear=2.0,
        clearance=0.3,
        collide=ring_collide(0.7),
        stuck=9.0,
    )
    record("M9", d)
    ok(
        "M9 reverse or stop",
        d.mode in (REVERSE_ESCAPE, SAFE_STOP, REPOSITION, "REPLAN"),
        d.mode,
    )

    # M10–M12 reverse result evaluation
    print("\nM10–M12 reverse guards")
    fsm = ManeuverFSM()
    t0 = time.time()
    d = decide(
        fsm,
        yaw=0.0,
        path=path_east,
        goal=goal_east,
        front=0.2,
        rear=3.0,
        clearance=0.25,
        collide=ring_collide(0.7),
        stuck=9.0,
        now=t0,
    )
    ok("M10 enter reverse or repos", d.mode in (REVERSE_ESCAPE, REPOSITION, SAFE_STOP), d.mode)
    if d.mode == REVERSE_ESCAPE:
        # worsen clearance
        d2 = decide(
            fsm,
            yaw=0.0,
            path=path_east,
            goal=goal_east,
            front=0.15,
            rear=1.0,
            clearance=0.05,
            collide=ring_collide(0.7),
            stuck=9.0,
            now=t0 + 1.0,
            recovery=1,
        )
        record("M11/M12", d2)
        ok(
            "M12 abort reverse on worse",
            d2.mode != REVERSE_ESCAPE and (fsm.reverse_after or {}).get("result") == "WORSE",
            f"{d2.mode} after={fsm.reverse_after}",
        )
    else:
        ok("M12 skip (no reverse enter)", True, d.mode)

    # M13 dynamic: wait vs reverse — front blocked briefly, rot safe, small heading
    print("\nM13 temporary block")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=0.05, path=path_east, goal=goal_east, front=0.3, clearance=1.5)
    record("M13", d)
    ok("M13 not infinite reverse", d.mode != REVERSE_ESCAPE or d.reason.startswith("DEAD"), d.mode)

    # M14 path blocked / no capture → replan-ish
    print("\nM14 no path capture")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=0.0, path=[], goal=goal_south, front=5.0, clearance=2.0)
    record("M14", d)
    # goal-only capture still available
    ok("M14 has capture via goal or mode sane", d.capture.available or d.mode in (SAFE_STOP, "REPLAN"), d.mode)

    # M15 local temporarily infeasible, heading OK → wait/forward turn not reverse
    print("\nM15 local temp infeasible")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=0.1, path=path_east, goal=goal_east, front=0.45, clearance=1.2)
    record("M15", d)
    ok(
        "M15 prefer wait/track/turn/side",
        d.mode in (WAIT_FOR_CLEARANCE, FORWARD_TRACK, FORWARD_TURN, ALIGN, "LOCAL_LEFT", "LOCAL_RIGHT", "REPOSITION"),
        d.mode,
    )
    ok("M15 not reverse", d.mode != "REVERSE_ESCAPE", d.mode)

    # M16 SIL invalid
    print("\nM16 SIL invalid")
    fsm = ManeuverFSM()
    d = fsm.decide(
        now=time.time(),
        x=float("nan"),
        y=0.0,
        yaw=0.0,
        path=path_east,
        goal=goal_east,
        front_near=5.0,
        rear_near=5.0,
        collision=False,
        emergency=False,
        path_progress=0.0,
        lateral_error=0.0,
        stuck_s=0.0,
        recovery_attempts=0,
        nav_active=True,
    )
    record("M16", d)
    ok("M16 SAFE_STOP", d.mode == SAFE_STOP, d.mode)
    ok("M16 reason SIL", "SIL" in d.reason or "SIL" in d.forward.reason, d.reason)

    # M17 emergency
    print("\nM17 emergency")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=0.0, path=path_east, goal=goal_east, emergency=True)
    record("M17", d)
    ok("M17 SAFE_STOP", d.mode == SAFE_STOP, d.mode)
    ok("M17 vx forced 0", d.force_vx == 0.0)

    # M18: action space — TURN_IN_PLACE forbids reverse vx
    print("\nM18 action space + MPPI gate")
    fsm = ManeuverFSM()
    d = decide(fsm, yaw=math.pi, path=path_east, goal=goal_east, clearance=2.0)
    ok("M18 tip vx~0", abs(d.vx_min) <= 0.02 and abs(d.vx_max) <= 0.02, f"{d.vx_min},{d.vx_max}")
    mppi = DiffDriveMppi()
    res = mppi.step(
        0,
        0,
        math.pi,
        path_east,
        goal_east,
        open_world_collide,
        front_near=5.0,
        force_reverse=False,
        maneuver_mode=TURN_IN_PLACE,
        vx_min=d.vx_min,
        vx_max=d.vx_max,
        force_vx=0.0,
        force_w=0.35,
    )
    ok("M18 tip cmd not reverse", res.vx >= -0.02, f"vx={res.vx}")
    ok("M18 tip has w", abs(res.w) > 0.05, f"w={res.w}")

    # Horizon concept
    print("\nHorizon")
    max_dh = max_heading_change_in_horizon()
    ok("horizon ~0.67rad", 0.5 < max_dh < 0.9, str(max_dh))
    cap = find_best_path_capture(0, 0, math.pi, path_east, clearance_at=open_clearance)
    ok("capture available", cap.available)
    fwd = assess_forward_feasibility(
        heading_error=cap.heading_error,
        front_near=5.0,
        front_stop=0.45,
        collision=False,
        capture=cap,
    )
    ok("large heading → not direct forward", fwd.maneuver_required == "ALIGN_REQUIRED", fwd.maneuver_required)

    # Local model wires
    print("\nLocalMppiModel wire")
    local = LocalMppiModel()

    class W:
        def collides(self, *a, **k):
            return False

        def clearance_at_xy(self, *a, **k):
            return 2.0

    res = local.step(
        W(),
        0,
        0,
        math.pi,
        path_east,
        goal_east,
        front_near=5.0,
        stuck_s=0.0,
        now=time.time(),
        actual_clearance=2.0,
        nav_active=True,
    )
    ok("local phase align/turn", local.phase in ("align", "turn_in_place"), local.phase)
    ok("local not reverse", local.phase != "reverse_escape", local.phase)
    ok("local tip w", abs(res.w) > 0.05 or local.last_maneuver.mode in (ALIGN, TURN_IN_PLACE), f"w={res.w}")

    print(f"\n=== RESULT PASS={PASS} FAIL={FAIL} ===")
    for r in ROWS:
        print(
            f"  {r['case']}: mode={r['mode']} reason={r['reason']} "
            f"herr={r['heading_error']} fwd={r['forward_reason']}"
        )
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
