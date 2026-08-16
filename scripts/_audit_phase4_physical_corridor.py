#!/usr/bin/env python3
"""STEP 3F-CORRECTIVE — Physical corridor SoT consistency (Probe ↔ corridor ↔ payload)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_trajectory import (  # noqa: E402
    SRC_BACKWARD,
    SRC_FORWARD,
    TrajectorySample,
    build_corridor_from_poses,
)


def _integrate(vx: float, w: float, n: int = 12, dt: float = 0.1):
    x = y = yaw = 0.0
    poses = [TrajectorySample(t=0.0, x=x, y=y, yaw=yaw, vx=vx, w=w)]
    for i in range(n):
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += w * dt
        poses.append(TrajectorySample(t=(i + 1) * dt, x=x, y=y, yaw=yaw, vx=vx, w=w))
    return poses


def main() -> int:
    fails = []
    fwd = build_corridor_from_poses(_integrate(0.15, 0.0), source=SRC_FORWARD, status="VALID")
    back = build_corridor_from_poses(_integrate(-0.12, 0.0), source=SRC_BACKWARD, status="VALID")
    left = build_corridor_from_poses(_integrate(0.14, 0.28), source="LEFT", status="VALID")

    for name, c in (("FWD", fwd), ("BACK", back), ("LEFT", left)):
        d = c.to_dict()
        if not d.get("left_edge") or not d.get("right_edge"):
            fails.append(f"{name}: missing edges")
        if not d.get("poses"):
            fails.append(f"{name}: missing poses")
        if float(d.get("half_width_m") or 0) < 0.2:
            fails.append(f"{name}: half_width too small {d.get('half_width_m')}")
        # corridor must start at vehicle base
        p0 = d["poses"][0]
        if abs(p0["x"]) > 1e-6 or abs(p0["y"]) > 1e-6:
            fails.append(f"{name}: corridor does not start at base pose")

    # Backward displacement along -x when yaw=0
    if back.poses[-1].x >= -0.05:
        fails.append(f"BACK: endpoint x={back.poses[-1].x} expected negative")
    # Left arc should gain +y (body left when yaw=0)
    if left.poses[-1].y <= 0.02:
        fails.append(f"LEFT: endpoint y={left.poses[-1].y} expected positive (left arc)")

    print("=== Physical Corridor Audit ===")
    if fails:
        for f in fails:
            print("FAIL:", f)
        return 1
    print("PASS: edges/poses/base-origin + DD geometry")
    print(f"  fwd.hw={fwd.half_width_m:.3f} back.len={back.length_m:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
