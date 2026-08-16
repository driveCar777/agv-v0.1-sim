"""Load Roboshop/Robokit .smap assets for offline sim (point cloud / routes / stations).

Assets live under note/agv_downloaded and are mounted as /data/agv_downloaded.
"""

from __future__ import annotations

import json
import math
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


DEFAULT_MAPS_DIR = Path("/data/agv_downloaded/maps")
DEFAULT_SMAP = "20260723112931750.smap"


def list_smaps(maps_dir: Path | str = DEFAULT_MAPS_DIR) -> List[Dict[str, Any]]:
    root = Path(maps_dir)
    out: List[Dict[str, Any]] = []
    if not root.is_dir():
        return out
    for p in sorted(root.glob("*.smap")):
        out.append({"file": p.name, "bytes": p.stat().st_size, "path": str(p)})
    return out


def _bezier(p0, p1, p2, p3, n: int = 12) -> List[Dict[str, float]]:
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1.0 - t
        x = u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0]
        y = u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1]
        pts.append({"x": x, "y": y})
    return pts


def _sample_curve(c: Dict[str, Any]) -> List[Dict[str, float]]:
    sp = (c.get("startPos") or {}).get("pos") or {}
    ep = (c.get("endPos") or {}).get("pos") or {}
    p0 = (float(sp.get("x", 0.0)), float(sp.get("y", 0.0)))
    p3 = (float(ep.get("x", 0.0)), float(ep.get("y", 0.0)))
    cls = str(c.get("className") or "")
    if "Bezier" in cls or c.get("controlPos1") or c.get("controlPos2"):
        c1 = c.get("controlPos1") or {}
        c2 = c.get("controlPos2") or {}
        p1 = (float(c1.get("x", (p0[0] + p3[0]) / 2)), float(c1.get("y", (p0[1] + p3[1]) / 2)))
        p2 = (float(c2.get("x", (p0[0] + p3[0]) / 2)), float(c2.get("y", (p0[1] + p3[1]) / 2)))
        return _bezier(p0, p1, p2, p3, 16 if "Degenerate" in cls else 20)
    return [{"x": p0[0], "y": p0[1]}, {"x": p3[0], "y": p3[1]}]


def load_smap_layers(
    smap_path: Path | str,
    *,
    max_cloud: int = 6000,
) -> Dict[str, Any]:
    path = Path(smap_path)
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    header = data.get("header") or {}
    stations: Dict[str, Dict[str, float]] = {}
    for pt in data.get("advancedPointList") or []:
        name = str(pt.get("instanceName") or "").strip()
        if not name:
            continue
        pos = pt.get("pos") or {}
        yaw = float(pt.get("dir", 0.0) or 0.0)
        stations[name] = {
            "x": float(pos.get("x", 0.0)),
            "y": float(pos.get("y", 0.0)),
            "yaw": yaw,
            "angle": yaw,
            "type": str(pt.get("className") or "LocationMark"),
        }

    cloud_raw = data.get("normalPosList") or []
    step = max(1, len(cloud_raw) // max_cloud) if cloud_raw else 1
    cloud = [
        {"x": float(p["x"]), "y": float(p["y"])}
        for i, p in enumerate(cloud_raw)
        if i % step == 0 and "x" in p and "y" in p
    ]

    curves = []
    for c in data.get("advancedCurveList") or []:
        curves.append(
            {
                "name": str(c.get("instanceName") or ""),
                "class": str(c.get("className") or ""),
                "start": str((c.get("startPos") or {}).get("instanceName") or ""),
                "end": str((c.get("endPos") or {}).get("instanceName") or ""),
                "points": _sample_curve(c),
            }
        )

    return {
        "map_file": path.name,
        "map_name": str(header.get("mapName") or path.stem),
        "vehicle_model": "AMB-150",
        "header": {
            "minPos": header.get("minPos"),
            "maxPos": header.get("maxPos"),
            "resolution": header.get("resolution"),
            "mapType": header.get("mapType"),
        },
        "stations": stations,
        "cloud": cloud,
        "cloud_total": len(cloud_raw),
        "cloud_shown": len(cloud),
        "curves": curves,
        "curve_count": len(curves),
    }


def stations_json_from_layers(layers: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "map_file": layers.get("map_file"),
        "map_name": layers.get("map_name"),
        "vehicle_model": layers.get("vehicle_model") or "AMB-150",
        "stations": layers.get("stations") or {},
    }


def ray_hits_box(
    ox: float, oy: float, dx: float, dy: float, box: Dict[str, float], max_d: float
) -> Optional[float]:
    """AABB obstacle: center x,y width w height h. Return hit distance or None."""
    hx = float(box["w"]) * 0.5
    hy = float(box["h"]) * 0.5
    minx, maxx = float(box["x"]) - hx, float(box["x"]) + hx
    miny, maxy = float(box["y"]) - hy, float(box["y"]) + hy
    # slab method
    tmin, tmax = 0.0, max_d
    for o, d, lo, hi in ((ox, dx, minx, maxx), (oy, dy, miny, maxy)):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
            continue
        inv = 1.0 / d
        t1 = (lo - o) * inv
        t2 = (hi - o) * inv
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    if tmin <= 0:
        return tmax if 0 < tmax <= max_d else None
    return tmin if tmin <= max_d else None


def nearest_cloud_hit(
    ox: float,
    oy: float,
    ang: float,
    cloud: List[Dict[str, float]],
    max_d: float = 8.0,
    beam_half_width: float = 0.08,
) -> Optional[float]:
    """Approximate lidar hit: nearest cloud point along a beam cone."""
    best = None
    ca, sa = math.cos(ang), math.sin(ang)
    for p in cloud:
        vx = float(p["x"]) - ox
        vy = float(p["y"]) - oy
        along = vx * ca + vy * sa
        if along <= 0.05 or along > max_d:
            continue
        lat = abs(-vx * sa + vy * ca)
        if lat > beam_half_width + along * 0.02:
            continue
        if best is None or along < best:
            best = along
    return best


def sim_laser_from_map(
    ax: float,
    ay: float,
    ayaw: float,
    cloud: List[Dict[str, float]],
    obstacles: List[Dict[str, Any]],
    *,
    beams: int = 120,
    max_d: float = 8.0,
) -> List[Dict[str, float]]:
    pts: List[Dict[str, float]] = []
    # Use a local subset of cloud for speed
    local = [
        p
        for p in cloud
        if abs(float(p["x"]) - ax) < max_d + 1.0 and abs(float(p["y"]) - ay) < max_d + 1.0
    ]
    for i in range(beams):
        ang = ayaw + (i / beams) * 2 * math.pi - math.pi
        d = nearest_cloud_hit(ax, ay, ang, local, max_d=max_d)
        for obs in obstacles:
            hit = ray_hits_box(ax, ay, math.cos(ang), math.sin(ang), obs, max_d)
            if hit is not None and (d is None or hit < d):
                d = hit
        if d is None:
            d = max_d
        pts.append({"x": ax + d * math.cos(ang), "y": ay + d * math.sin(ang)})
    return pts


def segment_hits_obstacle(
    x0: float, y0: float, x1: float, y1: float, obstacles: List[Dict[str, Any]]
) -> Optional[str]:
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy) or 1e-6
    steps = max(8, int(length / 0.15))
    for i in range(steps + 1):
        t = i / steps
        x = x0 + dx * t
        y = y0 + dy * t
        for obs in obstacles:
            hx = float(obs["w"]) * 0.5
            hy = float(obs["h"]) * 0.5
            if abs(x - float(obs["x"])) <= hx and abs(y - float(obs["y"])) <= hy:
                return str(obs.get("id") or "obstacle")
    return None


def new_obstacle(x: float, y: float, w: float = 0.6, h: float = 0.6) -> Dict[str, Any]:
    return {
        "id": str(uuid.uuid4())[:8],
        "x": float(x),
        "y": float(y),
        "w": float(w),
        "h": float(h),
    }
