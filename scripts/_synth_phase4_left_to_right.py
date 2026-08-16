#!/usr/bin/env python3
"""DETERMINISTIC SYNTHETIC REPRODUCTION — LocalManeuverSelector LEFT→RIGHT.

Uses real LocalManeuverSelector.compare() without modifying selector code.
Marks output as SYNTHETIC (not LIVE 19999).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.local_maneuver import (  # noqa: E402
    DEC_LEFT,
    DEC_RIGHT,
    LocalManeuverSelector,
)

Obs = Tuple[float, float, float]  # x,y,r


def _make_fns(obstacles: List[Obs]):
    def collide(x: float, y: float) -> bool:
        for ox, oy, r in obstacles:
            if (x - ox) ** 2 + (y - oy) ** 2 <= (r + 0.22) ** 2:
                return True
        return False

    def clearance_at(x: float, y: float) -> float:
        best = 99.0
        for ox, oy, r in obstacles:
            d = math.hypot(x - ox, y - oy) - r
            if d < best:
                best = d
        return max(0.0, best)

    return collide, clearance_at


def _parts(c):
    if c is None:
        return None
    return {
        "feasible": bool(c.feasible),
        "total_cost": round(float(c.total_cost), 3),
        "collision": bool(c.collision),
        "min_clearance": round(float(c.min_clearance), 3),
        "path_capture_distance": round(float(getattr(c, "path_capture_distance", 0.0) or 0.0), 3),
        "path_progress_gain": round(float(getattr(c, "path_progress_gain", 0.0) or 0.0), 3),
        "heading_error": round(float(getattr(c, "heading_error", 0.0) or 0.0), 3),
        "lateral_error": round(float(getattr(c, "lateral_error", 0.0) or 0.0), 3),
        "route_quality": c.route_quality,
        "reason": c.reason,
        # cost components if present on candidate
        "clearance_cost": round(float(getattr(c, "clearance_cost", 0.0) or 0.0), 3),
        "capture_cost": round(float(getattr(c, "capture_cost", 0.0) or 0.0), 3),
        "heading_cost": round(float(getattr(c, "heading_cost", 0.0) or 0.0), 3),
        "progress_cost": round(float(getattr(c, "progress_cost", 0.0) or 0.0), 3),
        "deviation_cost": round(float(getattr(c, "deviation_cost", 0.0) or 0.0), 3),
        "collision_cost": round(float(getattr(c, "collision_cost", 0.0) or 0.0), 3),
    }


def main() -> int:
    sel = LocalManeuverSelector()
    path = [(0.0, 0.0), (6.0, 0.0)]
    goal = (6.0, 0.0)
    rows = []
    switched = False
    switch_row = None

    # Phase A: center pillar — LEFT preferred
    obs_a: List[Obs] = [(1.55, 0.05, 0.35)]
    # Phase B: LEFT corridor filled with soft clutter (still rollable without hard
    # collision if careful), RIGHT opened — forces LOWER_TOTAL_COST after hold.
    obs_b: List[Obs] = [
        (1.55, 0.05, 0.38),
        (0.85, 0.55, 0.32),
        (1.15, 0.75, 0.30),
        (1.45, 0.90, 0.28),
        (0.55, 0.35, 0.25),
        (2.2, -0.85, 0.22),  # far right clutter only
    ]

    for i in range(0, 40):
        t = i * 0.1
        phase = "A" if t < 0.90 else "B"
        obstacles = obs_a if phase == "A" else obs_b
        collide, clearance_at = _make_fns(obstacles)
        # Keep FORWARD blocked so we stay in side-compare branch
        front_near = 0.55 if phase == "A" else 0.50
        # Slight leftward pose in B so LEFT rollouts skim clutter
        x, y, yaw = (0.15, 0.05, 0.05) if phase == "A" else (0.40, 0.28, 0.25)

        res = sel.compare(
            now=t,
            x=x,
            y=y,
            yaw=yaw,
            path=path,
            goal=goal,
            front_near=front_near,
            rear_near=2.0,
            collide=collide,
            clearance_at=clearance_at,
            forward_feasible=False,
            rotation_safe=True,
            left_free=1.2 if phase == "A" else 0.35,
            right_free=0.8 if phase == "A" else 1.4,
            force=True,
            require_capture=False,
            max_deviation_m=1.5,
            path_follow_scale=0.6,
        )
        L = res.candidates.get(DEC_LEFT)
        R = res.candidates.get(DEC_RIGHT)
        Lp, Rp = _parts(L), _parts(R)
        row = {
            "t": round(t, 3),
            "phase": phase,
            "selected": res.selected,
            "reason": res.reason,
            "current": sel.current,
            "hold_until": round(sel.hold_until, 3),
            "Lf": Lp["feasible"] if Lp else None,
            "Rf": Rp["feasible"] if Rp else None,
            "Lc": Lp["total_cost"] if Lp else None,
            "Rc": Rp["total_cost"] if Rp else None,
            "left": Lp,
            "right": Rp,
            "n_obstacles": len(obstacles),
        }
        rows.append(row)
        print(
            f"t={t:.1f} sel={res.selected} reason={res.reason} "
            f"Lc={row['Lc']} Rc={row['Rc']} Lf={row['Lf']} Rf={row['Rf']}"
        )
        if (
            not switched
            and len(rows) >= 2
            and rows[-2]["selected"] == "LEFT"
            and res.selected == "RIGHT"
        ):
            switched = True
            switch_row = row
            print("SYNTHETIC_SWITCH LEFT->RIGHT")
            print("BOTH_FEASIBLE", row["Lf"], row["Rf"])
            if Lp and Rp:
                print("DELTA", round(Lp["total_cost"] - Rp["total_cost"], 3))
                print("LEFT_PARTS", {k: Lp[k] for k in (
                    "collision_cost", "clearance_cost", "capture_cost",
                    "heading_cost", "progress_cost", "deviation_cost",
                    "min_clearance", "path_capture_distance", "path_progress_gain",
                )})
                print("RIGHT_PARTS", {k: Rp[k] for k in (
                    "collision_cost", "clearance_cost", "capture_cost",
                    "heading_cost", "progress_cost", "deviation_cost",
                    "min_clearance", "path_capture_distance", "path_progress_gain",
                )})

    out = ROOT / "docs" / "_phase4_trace" / "synthetic_left_to_right.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    pattern = "UNKNOWN"
    if switched and switch_row:
        if switch_row.get("Lf") is False and switch_row.get("Rf") is True:
            pattern = "HARD_LEFT_INFEASIBLE_RIGHT_ONLY"
        elif switch_row.get("Lf") and switch_row.get("Rf") and switch_row.get("reason") == "LOWER_TOTAL_COST":
            pattern = "BOTH_FEASIBLE_LOWER_TOTAL_COST"
        else:
            pattern = f"SWITCH_REASON_{switch_row.get('reason')}"
    payload = {
        "reproduction_type": "DETERMINISTIC_SYNTHETIC_REPRODUCTION",
        "pattern": pattern,
        "switched": switched,
        "switch_row": switch_row,
        "rows": rows,
        "note": "Offline LocalManeuverSelector.compare only; not LIVE 19999.",
    }
    out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("WROTE", out, "SWITCH", switched, "PATTERN", pattern)
    return 0 if switched else 2


if __name__ == "__main__":
    raise SystemExit(main())
