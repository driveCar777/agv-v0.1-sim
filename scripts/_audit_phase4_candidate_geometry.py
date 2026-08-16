#!/usr/bin/env python3
"""STEP 3F-CORRECTIVE — Vehicle-centric candidate geometry (no lateral teleport)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.local_maneuver import (  # noqa: E402
    DEC_FORWARD,
    DEC_LEFT,
    DEC_RIGHT,
    DEC_REVERSE,
    rollout_candidate,
)


def _body(dx, dy, yaw0):
    c, s = math.cos(yaw0), math.sin(yaw0)
    return dx * c + dy * s, -dx * s + dy * c


def _check_dd(path, vx, w, dt=0.1, tol=0.08):
    """Each step must match differential-drive integration (no lateral teleport)."""
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        yaw = float(a.get("yaw", 0.0))
        ex = a["x"] + vx * math.cos(yaw) * dt
        ey = a["y"] + vx * math.sin(yaw) * dt
        if math.hypot(b["x"] - ex, b["y"] - ey) > tol:
            return False, i, (ex, ey), (b["x"], b["y"])
    return True, -1, None, None


def main() -> int:
    fails = []
    x, y, yaw = 1.0, 2.0, 0.7
    path_global = [(1.0, 2.0), (3.0, 3.5), (5.0, 4.0)]
    collide = lambda *_: False

    specs = [
        (DEC_FORWARD, 0.14, 0.0),
        (DEC_LEFT, 0.14, 0.28),
        (DEC_RIGHT, 0.14, -0.28),
        (DEC_REVERSE, -0.12, 0.0),
    ]
    for ctype, vx, w in specs:
        c = rollout_candidate(
            ctype=ctype,
            x=x,
            y=y,
            yaw=yaw,
            vx=vx,
            w=w,
            path=path_global,
            goal=(6.0, 5.0),
            collide=collide,
            clearance_at=lambda *_: 2.0,
            front_near=2.0,
            require_capture=False,
        )
        ok, idx, exp, got = _check_dd(c.path, c.vx, c.w)
        if not ok:
            fails.append(f"{ctype}: DD break at step {idx} exp={exp} got={got}")
        # Body-frame endpoints
        bx, by = _body(c.path[-1]["x"] - x, c.path[-1]["y"] - y, yaw)
        if ctype == DEC_FORWARD and bx < 0.05:
            fails.append(f"FORWARD body_x={bx} expected forward")
        if ctype == DEC_REVERSE and bx > -0.05:
            fails.append(f"REVERSE body_x={bx} expected rearward")
        if ctype == DEC_LEFT and by < 0.02:
            fails.append(f"LEFT body_y={by} expected +left (curvature), not lateral teleport alone")
        if ctype == DEC_RIGHT and by > -0.02:
            fails.append(f"RIGHT body_y={by} expected -right")
        # No single-step lateral teleport: |Δy_body| per step bounded by |v|dt + small
        for i in range(len(c.path) - 1):
            a, b = c.path[i], c.path[i + 1]
            ddx, ddy = b["x"] - a["x"], b["y"] - a["y"]
            step_bx, step_by = _body(ddx, ddy, float(a.get("yaw", yaw)))
            # lateral-only jump without longitudinal when |v|>0.05 is invalid for DD
            if abs(c.vx) > 0.05 and abs(step_bx) < 1e-4 and abs(step_by) > 0.05:
                fails.append(f"{ctype}: lateral teleport at step {i}")

    print("=== Vehicle-Centric Candidate Geometry Audit ===")
    if fails:
        for f in fails:
            print("FAIL:", f)
        return 1
    print("PASS: F/L/R/B differential-drive arcs, no lateral teleport")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
