#!/usr/bin/env python3
"""P1-0 offline: OPEN-space MPPI cost landscape vs constant vx (no behavior change).

Sweeps constant-speed forward rollouts on a straight global path with no obstacles
and prints cost_breakdown from DiffDriveMppi._score.
"""

from __future__ import annotations

import math
import os
import sys
from typing import List, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402

Pt = Tuple[float, float]


def straight_path(length_m: float = 12.0, step: float = 0.25) -> List[Pt]:
    n = max(2, int(length_m / step) + 1)
    return [(i * step, 0.0) for i in range(n)]


def rollout(vx: float, w: float, steps: int, dt: float, x0=0.0, y0=0.0, yaw0=0.0) -> Tuple[List[Pt], List[float], List[float]]:
    path: List[Pt] = [(x0, y0)]
    vx_seq: List[float] = []
    w_seq: List[float] = []
    x, y, yaw = x0, y0, yaw0
    for _ in range(steps):
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += w * dt
        path.append((x, y))
        vx_seq.append(vx)
        w_seq.append(w)
    return path, vx_seq, w_seq


def main() -> int:
    mppi = DiffDriveMppi()
    gpath = straight_path()
    goal = gpath[-1]
    collide = lambda _x, _y: False
    front_near = 30.0
    print("=== P1-0 OPEN MPPI Speed Cost Sweep ===")
    print(f"time_steps={mppi.time_steps} dt={mppi.model_dt} horizon_s={mppi.time_steps * mppi.model_dt}")
    print(f"path_follow_weight default in _score arg=5.0  speed_track preference=0.22")
    print(
        f"{'vx':>6} {'speed':>8} {'gpath':>8} {'goal':>8} {'w':>8} {'jerk':>8} {'obst':>8} {'total':>8}"
    )
    rows = []
    for vx in (0.10, 0.15, 0.18, 0.20, 0.22, 0.25, 0.30, 0.35, 0.40):
        path, vx_seq, w_seq = rollout(vx, 0.0, mppi.time_steps, mppi.model_dt)
        cost, mode, meta = mppi._score(
            path,
            vx_seq,
            w_seq,
            gpath,
            goal,
            collide,
            front_near,
            mppi.geom.front_cost_m,
            path_follow_weight=5.0,
        )
        bd = meta.get("cost_breakdown") or {}
        row = {
            "vx": vx,
            "mode": mode,
            "speed_track_cost": bd.get("speed_track_cost"),
            "global_path_cost": bd.get("global_path_cost"),
            "goal_cost": bd.get("goal_cost"),
            "w_cost": bd.get("w_cost"),
            "jerk_cost": bd.get("jerk_cost"),
            "front_obstacle_cost": bd.get("front_obstacle_cost"),
            "total": bd.get("total") if bd.get("total") is not None else cost,
        }
        rows.append(row)
        print(
            f"{vx:6.2f} {row['speed_track_cost']:8.3f} {row['global_path_cost']:8.3f} "
            f"{row['goal_cost']:8.3f} {row['w_cost']:8.3f} {row['jerk_cost']:8.3f} "
            f"{row['front_obstacle_cost']:8.3f} {row['total']:8.3f}"
        )
    best = min(rows, key=lambda r: float(r["total"]))
    print(f"LOWEST_TOTAL_COST vx={best['vx']} total={best['total']}")
    print("NOTE: with w=0 and collinear path, global_path_cost≈0; goal_cost falls with higher vx (more progress).")
    print("      speed_track_cost alone is minimized at vx=0.22 (soft preference, weight 0.25).")
    # Also show interaction: if path has lateral offset, higher speed may not dominate
    print("--- offset path (y=0.15 lateral start) ---")
    rows2 = []
    for vx in (0.10, 0.18, 0.22, 0.30, 0.40):
        path, vx_seq, w_seq = rollout(vx, 0.0, mppi.time_steps, mppi.model_dt, y0=0.15)
        cost, mode, meta = mppi._score(
            path, vx_seq, w_seq, gpath, goal, collide, front_near, mppi.geom.front_cost_m, path_follow_weight=5.0
        )
        bd = meta.get("cost_breakdown") or {}
        rows2.append({"vx": vx, "total": bd.get("total"), "gpath": bd.get("global_path_cost"), "speed": bd.get("speed_track_cost"), "goal": bd.get("goal_cost")})
        print(f"vx={vx:.2f} total={bd.get('total')} gpath={bd.get('global_path_cost')} speed={bd.get('speed_track_cost')} goal={bd.get('goal_cost')}")
    best2 = min(rows2, key=lambda r: float(r["total"]))
    print(f"LOWEST_TOTAL_OFFSET vx={best2['vx']} total={best2['total']}")
    print("P1-0 SPEED COST SWEEP = PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
