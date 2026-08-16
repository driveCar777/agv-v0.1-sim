"""激光扫描纯几何仿真 — 无 ROS 依赖。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple


@dataclass
class ScanResult:
    frame_id: str
    ranges: List[float]
    range_min: float = 0.1
    range_max: float = 30.0
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = math.radians(1.0)
    intensities: List[float] = field(default_factory=list)

    class _Header:
        def __init__(self, frame_id: str):
            self.frame_id = frame_id
            self.stamp = None

    def __post_init__(self):
        self.header = self._Header(self.frame_id)


def make_scan(
    stamp,
    frame_id: str,
    obstacle_world_xy: Sequence,
    lidar_world_xy: tuple,
    lidar_yaw: float,
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    angle_increment: float = math.radians(1.0),
    range_min: float = 0.1,
    range_max: float = 30.0,
) -> ScanResult:
    n = int(round((angle_max - angle_min) / angle_increment)) + 1
    ranges = [float(range_max)] * n
    lx, ly = lidar_world_xy
    cos_yaw = math.cos(-lidar_yaw)
    sin_yaw = math.sin(-lidar_yaw)
    for ox, oy, radius in obstacle_world_xy:
        wx, wy = ox - lx, oy - ly
        lx_local = wx * cos_yaw - wy * sin_yaw
        ly_local = wx * sin_yaw + wy * cos_yaw
        if radius <= 0:
            d_world = math.hypot(wx, wy)
            if d_world > range_max:
                continue
            ang = math.atan2(ly_local, lx_local)
            if not (angle_min <= ang <= angle_max):
                continue
            idx = int(round((ang - angle_min) / angle_increment))
            if 0 <= idx < n and d_world < ranges[idx]:
                ranges[idx] = d_world
            continue
        d_center = math.hypot(wx, wy)
        if d_center == 0:
            continue
        d_hit = max(0.0, d_center - radius)
        ang = math.atan2(ly_local, lx_local)
        if not (angle_min <= ang <= angle_max):
            continue
        idx = int(round((ang - angle_min) / angle_increment))
        if 0 <= idx < n and d_hit < ranges[idx]:
            ranges[idx] = d_hit
    return ScanResult(
        frame_id=frame_id,
        ranges=ranges,
        range_min=range_min,
        range_max=range_max,
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment,
    )


DEFAULT_SCENE: List[Tuple[float, float, float]] = [
    *[(x, y, 0.05) for x in (-2.0, 2.0) for y in (-2.0, 2.0)],
    (1.0, 0.5, 0.20),
    (-1.2, 0.8, 0.15),
    (0.8, -1.0, 0.30),
    (-0.5, -1.5, 0.10),
]
