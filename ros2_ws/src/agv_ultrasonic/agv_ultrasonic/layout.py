"""超声布局与射线距离 — 无 ROS 依赖。"""

from __future__ import annotations

import math
from typing import List, Tuple

SENSOR_LAYOUT: List[Tuple[float, float, float]] = [
    (0.35, 0.30, 0.0),
    (0.35, 0.20, 0.0),
    (0.35, 0.10, 0.0),
    (0.35, -0.10, 0.0),
    (0.35, -0.20, 0.0),
    (0.35, -0.30, 0.0),
    (-0.35, 0.30, math.pi),
    (-0.35, 0.20, math.pi),
    (-0.35, 0.10, math.pi),
    (-0.35, -0.10, math.pi),
    (-0.35, -0.20, math.pi),
    (-0.35, -0.30, math.pi),
]


def closest_distance(
    sx: float,
    sy: float,
    yaw: float,
    scene,
    range_max: float,
) -> float:
    dx, dy = math.cos(yaw), math.sin(yaw)
    best = range_max
    for ox, oy, radius in scene:
        wx, wy = ox - sx, oy - sy
        proj = wx * dx + wy * dy
        if proj < 0:
            continue
        perp_x = wx - proj * dx
        perp_y = wy - proj * dy
        perp = math.hypot(perp_x, perp_y)
        if perp >= radius:
            hit = proj
        else:
            hit = proj - math.sqrt(max(0.0, radius * radius - perp * perp))
        if 0 < hit < best:
            best = hit
    return best
