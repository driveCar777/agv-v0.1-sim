#!/usr/bin/env python3
"""Search obstacle/free configs for both-feasible LOWER_TOTAL_COST LEFT→RIGHT."""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.local_maneuver import DEC_LEFT, DEC_RIGHT, LocalManeuverSelector  # noqa: E402


def make_fns(obstacles):
    def collide(x, y):
        for ox, oy, r in obstacles:
            if (x - ox) ** 2 + (y - oy) ** 2 <= (r + 0.22) ** 2:
                return True
        return False

    def clearance_at(x, y):
        best = 99.0
        for ox, oy, r in obstacles:
            d = math.hypot(x - ox, y - oy) - r
            if d < best:
                best = d
        return max(0.0, best)

    return collide, clearance_at


def run_once(lf, rf, obs_b, pose_b=(0.25, 0.15, 0.12)):
    sel = LocalManeuverSelector()
    path = [(0.0, 0.0), (6.0, 0.0)]
    goal = (6.0, 0.0)
    obs_a = [(1.55, 0.05, 0.35)]
    prev = "FORWARD"
    switch = None
    timeline = []
    for i in range(30):
        t = i * 0.1
        phase = "A" if t < 0.9 else "B"
        obstacles = obs_a if phase == "A" else obs_b
        collide, clearance_at = make_fns(obstacles)
        x, y, yaw = (0.1, 0.0, 0.0) if phase == "A" else pose_b
        left_free, right_free = (1.2, 0.7) if phase == "A" else (lf, rf)
        res = sel.compare(
            now=t,
            x=x,
            y=y,
            yaw=yaw,
            path=path,
            goal=goal,
            front_near=0.52,
            rear_near=2.0,
            collide=collide,
            clearance_at=clearance_at,
            forward_feasible=False,
            rotation_safe=True,
            left_free=left_free,
            right_free=right_free,
            force=True,
            require_capture=False,
            path_follow_scale=0.5,
        )
        L = res.candidates[DEC_LEFT]
        R = res.candidates[DEC_RIGHT]
        timeline.append(
            (
                t,
                res.selected,
                res.reason,
                L.feasible,
                R.feasible,
                round(L.total_cost, 2),
                round(R.total_cost, 2),
                round(L.clearance_cost, 2),
                round(R.clearance_cost, 2),
                round(L.capture_cost, 2),
                round(R.capture_cost, 2),
            )
        )
        if prev == "LEFT" and res.selected == "RIGHT":
            switch = {
                "t": t,
                "reason": res.reason,
                "Lf": L.feasible,
                "Rf": R.feasible,
                "Lc": L.total_cost,
                "Rc": R.total_cost,
                "L_clr_c": L.clearance_cost,
                "R_clr_c": R.clearance_cost,
                "L_cap_c": L.capture_cost,
                "R_cap_c": R.capture_cost,
                "L_prog_c": getattr(L, "progress_cost", None),
                "R_prog_c": getattr(R, "progress_cost", None),
                "L_head_c": getattr(L, "heading_cost", None),
                "R_head_c": getattr(R, "heading_cost", None),
                "L_dev_c": getattr(L, "deviation_cost", None),
                "R_dev_c": getattr(R, "deviation_cost", None),
                "L_min_clr": L.min_clearance,
                "R_min_clr": R.min_clearance,
            }
            break
        prev = res.selected
    return switch, timeline


def main():
    configs = []
    for lf, rf in [(0.2, 2.0), (0.3, 1.8), (0.1, 2.5), (0.4, 2.0)]:
        for obs in [
            [(1.55, 0.05, 0.35), (0.9, 0.95, 0.40), (1.2, 1.1, 0.35)],
            [(1.55, 0.05, 0.35), (1.0, 1.2, 0.45)],
            [(1.6, 0.0, 0.32), (0.7, 0.85, 0.38), (1.1, 1.05, 0.35), (1.4, 1.15, 0.30)],
            # soft only: clearance blobs far from body collide radius along left arc
            [(1.55, 0.05, 0.35), (0.6, 1.35, 0.55), (1.0, 1.45, 0.50), (1.4, 1.50, 0.45)],
            [(1.55, 0.05, 0.35), (0.4, 1.5, 0.6), (0.9, 1.6, 0.55)],
        ]:
            for pose_b in [(0.25, 0.15, 0.12), (0.35, 0.22, 0.2), (0.2, 0.05, 0.05), (0.5, 0.35, 0.3)]:
                configs.append((lf, rf, obs, pose_b))

    found = None
    near = []
    for lf, rf, obs_b, pose_b in configs:
        switch, timeline = run_once(lf, rf, obs_b, pose_b)
        if not switch:
            continue
        near.append((switch, lf, rf, pose_b, obs_b, timeline[:12]))
        if switch["Lf"] and switch["Rf"] and switch["reason"] == "LOWER_TOTAL_COST":
            found = (lf, rf, obs_b, pose_b, switch, timeline)
            break

    if found:
        lf, rf, obs_b, pose_b, switch, timeline = found
        print("FOUND_BOTH_FEASIBLE_LOWER_TOTAL_COST")
        print("lf", lf, "rf", rf, "pose_b", pose_b)
        print("obs_b", obs_b)
        print("switch", switch)
        for row in timeline[:20]:
            print(row)
        return 0

    print("NO_BOTH_FEASIBLE_FLIP; near misses:")
    for switch, lf, rf, pose_b, obs_b, tl in near[:8]:
        print(switch, "lf", lf, "rf", rf, "pose", pose_b)
        for row in tl[-3:]:
            print(" ", row)
    if not near:
        # show one baseline timeline
        sw, tl = run_once(0.2, 2.0, [(1.55, 0.05, 0.35), (0.6, 1.35, 0.55)])
        for row in tl:
            print(row)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
