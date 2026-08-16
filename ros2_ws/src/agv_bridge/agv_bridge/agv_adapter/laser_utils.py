"""Transform Robokit 1009 laser JSON into map-frame LaserScan DTO."""

from __future__ import annotations

import math
from typing import Any, Dict, List

from agv_bridge.agv_adapter.models import LaserPoint, LaserScan, RobotPose


def laser_scan_from_robokit(
    pose: RobotPose,
    raw: Dict[str, Any],
    max_points: int = 800,
) -> LaserScan:
    ax = pose.x
    ay = pose.y
    ayaw = pose.angle
    pts: List[LaserPoint] = []
    lasers = raw.get("lasers") or raw.get("laser") or []
    if isinstance(lasers, dict):
        lasers = [lasers]
    for laser in lasers:
        if not isinstance(laser, dict):
            continue
        inst = laser.get("install_info") or {}
        ix = float(inst.get("x", 0.0) or 0.0)
        iy = float(inst.get("y", 0.0) or 0.0)
        iyaw = float(inst.get("yaw", 0.0) or 0.0)
        beams = laser.get("beams") or []
        step = max(1, len(beams) // max_points) if beams else 1
        for i, b in enumerate(beams):
            if i % step != 0:
                continue
            if not isinstance(b, dict):
                continue
            if b.get("valid") is False:
                continue
            dist = float(b.get("dist", 0.0) or 0.0)
            if dist <= 0.05 or dist > 80.0:
                continue
            beam_deg = float(b.get("angle", 0.0) or 0.0)
            ang = math.radians(beam_deg) + iyaw
            lx = ix + dist * math.cos(ang)
            ly = iy + dist * math.sin(ang)
            mx = ax + lx * math.cos(ayaw) - ly * math.sin(ayaw)
            my = ay + lx * math.sin(ayaw) + ly * math.cos(ayaw)
            pts.append(LaserPoint(x=mx, y=my))
            if len(pts) >= max_points:
                return LaserScan(
                    ok=True,
                    points=pts,
                    beam_count=len(pts),
                    source="robokit_1009",
                    message="ok",
                )
    return LaserScan(
        ok=True,
        points=pts,
        beam_count=len(pts),
        source="robokit_1009",
        message="ok",
    )
