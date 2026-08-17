"""加载 Robokit/Seer .smap 为仿真占用栅格 + 点云 + 站点。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class MapPOI:
    id: str
    x: float
    y: float
    kind: str = "LocationMark"
    yaw: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {"id": self.id, "x": self.x, "y": self.y, "kind": self.kind, "yaw": self.yaw, "r": 0.0}


@dataclass
class LoadedSmap:
    name: str
    path: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    resolution: float
    cloud: List[Tuple[float, float]]  # 下采样可视化点
    occupied: set  # (ix, iy) @ plan_res — 规划用（已膨胀）
    plan_res: float
    occupied_raw: set = field(default_factory=set)  # 雷达用（未膨胀，贴合可视点云）
    pois: List[MapPOI] = field(default_factory=list)
    scene_kind: str = "indoor"  # indoor | outdoor
    path_color: str = "#E8B84A"  # indoor yellow
    requested_inflate_m: float = 0.0
    actual_inflate_m: float = 0.0

    @property
    def width(self) -> float:
        return max(1.0, self.max_x - self.min_x)

    @property
    def height(self) -> float:
        return max(1.0, self.max_y - self.min_y)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor((x - self.min_x) / self.plan_res)),
            int(math.floor((y - self.min_y) / self.plan_res)),
        )

    def cell_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        return (
            self.min_x + (ix + 0.5) * self.plan_res,
            self.min_y + (iy + 0.5) * self.plan_res,
        )

    def in_bounds(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


def _downsample(points: List[Tuple[float, float]], max_n: int = 12000) -> List[Tuple[float, float]]:
    if len(points) <= max_n:
        return points
    step = max(1, len(points) // max_n)
    return points[::step]


def _inflate(occupied: set, radius_cells: int) -> set:
    """旧接口：按 cell 半径膨胀（棋盘欧氏格距）。保留兼容。"""
    if radius_cells <= 0:
        return set(occupied)
    out = set(occupied)
    for ix, iy in occupied:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    out.add((ix + dx, iy + dy))
    return out


def _inflate_meters(occupied: set, plan_res: float, inflate_m: float) -> Tuple[set, float]:
    """按真实米制膨胀：邻格中心距 <= inflate_m 才纳入。

    避免 ceil(inflate_m/plan_res) 把 0.28m 量化成 0.50m。
    返回 (occupied_inflated, actual_inflate_m)；actual ≈ requested。
    """
    if inflate_m <= 0:
        return set(occupied), 0.0
    rad_cells = max(1, int(math.ceil(inflate_m / plan_res)))
    out = set(occupied)
    for ix, iy in occupied:
        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if math.hypot(dx * plan_res, dy * plan_res) <= inflate_m + 1e-9:
                    out.add((ix + dx, iy + dy))
    return out, float(inflate_m)


def load_smap(
    path: Path,
    plan_res: float = 0.25,
    inflate_m: float = 0.35,
    cloud_max: int = 14000,
    scene_kind: str = "indoor",
) -> LoadedSmap:
    raw = json.loads(path.read_text(encoding="utf-8"))
    header = raw.get("header") or {}
    min_pos = header.get("minPos") or {"x": 0, "y": 0}
    max_pos = header.get("maxPos") or {"x": 10, "y": 10}
    min_x = float(min_pos.get("x", 0.0))
    min_y = float(min_pos.get("y", 0.0))
    max_x = float(max_pos.get("x", 10.0))
    max_y = float(max_pos.get("y", 10.0))
    resolution = float(header.get("resolution") or 0.02)
    name = str(header.get("mapName") or path.stem)

    pts: List[Tuple[float, float]] = []
    for p in raw.get("normalPosList") or []:
        if "x" not in p or "y" not in p:
            continue
        pts.append((float(p["x"]), float(p["y"])))

    # feature lines → dense samples as occupied
    for line in raw.get("advancedLineList") or []:
        ln = line.get("line") or {}
        s, e = ln.get("startPos") or {}, ln.get("endPos") or {}
        if not s or not e:
            continue
        x0, y0 = float(s["x"]), float(s["y"])
        x1, y1 = float(e["x"]), float(e["y"])
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        for i in range(n + 1):
            t = i / n
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))

    occupied: set = set()
    for x, y in pts:
        ix = int(math.floor((x - min_x) / plan_res))
        iy = int(math.floor((y - min_y) / plan_res))
        occupied.add((ix, iy))

    occupied_raw = set(occupied)
    occupied, actual_inflate = _inflate_meters(occupied, plan_res, inflate_m)

    pois: List[MapPOI] = []
    for ap in raw.get("advancedPointList") or []:
        pos = ap.get("pos") or {}
        pid = str(ap.get("instanceName") or f"P{len(pois)}")
        pois.append(
            MapPOI(
                id=pid,
                x=float(pos.get("x", 0.0)),
                y=float(pos.get("y", 0.0)),
                kind=str(ap.get("className") or "LocationMark"),
            )
        )

    path_color = "#E8B84A" if scene_kind == "indoor" else "#3B82F6"
    return LoadedSmap(
        name=name,
        path=str(path),
        min_x=min_x,
        min_y=min_y,
        max_x=max_x,
        max_y=max_y,
        resolution=resolution,
        cloud=_downsample(pts, cloud_max),
        occupied=occupied,
        occupied_raw=occupied_raw,
        plan_res=plan_res,
        pois=pois,
        scene_kind=scene_kind,
        path_color=path_color,
        requested_inflate_m=float(inflate_m),
        actual_inflate_m=float(actual_inflate),
    )


def build_outdoor_campus(plan_res: float = 0.4) -> LoadedSmap:
    """公开可用的室外园区/停车场示意场景（离线可复现）。"""
    min_x, min_y, max_x, max_y = -40.0, -40.0, 40.0, 40.0
    pts: List[Tuple[float, float]] = []
    occupied: set = set()

    def mark_rect(x0: float, y0: float, x1: float, y1: float, step: float = 0.4) -> None:
        x = x0
        while x <= x1:
            y = y0
            while y <= y1:
                pts.append((x, y))
                occupied.add(
                    (int(math.floor((x - min_x) / plan_res)), int(math.floor((y - min_y) / plan_res)))
                )
                y += step
            x += step

    # 边界围墙
    for x in range(-40, 41):
        mark_rect(float(x), -40.0, float(x), -39.2)
        mark_rect(float(x), 39.2, float(x), 40.0)
    for y in range(-40, 41):
        mark_rect(-40.0, float(y), -39.2, float(y))
        mark_rect(39.2, float(y), 40.0, float(y))

    # 建筑块
    buildings = [
        (-25, -20, -10, -5),
        (10, -25, 28, -8),
        (-30, 10, -12, 28),
        (12, 8, 30, 25),
        (-5, -5, 5, 5),  # 中心岛
    ]
    for x0, y0, x1, y1 in buildings:
        mark_rect(x0, y0, x1, y1, step=0.5)

    # 停车位短线
    for i in range(-6, 7):
        mark_rect(-8.0 + i * 2.5, 15.0, -7.2 + i * 2.5, 18.0, step=0.3)

    occupied_raw = set(occupied)
    req_inf = 0.4
    occupied, actual_inflate = _inflate_meters(occupied, plan_res, req_inf)
    pois = [
        MapPOI("GATE", -35.0, 0.0, "Gate"),
        MapPOI("PARK_A", -20.0, 16.0, "Parking"),
        MapPOI("PARK_B", 20.0, 16.0, "Parking"),
        MapPOI("DOCK", 0.0, -30.0, "Dock"),
        MapPOI("YARD", 25.0, -20.0, "Yard"),
        MapPOI("CENTER", 0.0, -12.0, "Plaza"),
    ]
    return LoadedSmap(
        name="outdoor_campus_public",
        path="builtin:outdoor_campus_public",
        min_x=min_x,
        min_y=min_y,
        max_x=max_x,
        max_y=max_y,
        resolution=0.05,
        cloud=_downsample(pts, 10000),
        occupied=occupied,
        occupied_raw=occupied_raw,
        plan_res=plan_res,
        pois=pois,
        scene_kind="outdoor",
        path_color="#3B82F6",
        requested_inflate_m=float(req_inf),
        actual_inflate_m=float(actual_inflate),
    )


def build_m32_open_straight(plan_res: float = 0.4, inflate_m: float = 0.28) -> LoadedSmap:
    """M3.2 open straight baseline — wide empty corridor, boundary walls only.

    Start M32_A (-25, 0) → Goal M32_B (25, 0): 50 m straight segment through
    obstacle-free center. Enters WorldModel/planner like any smap scene.
    """
    min_x, min_y, max_x, max_y = -50.0, -50.0, 50.0, 50.0
    pts: List[Tuple[float, float]] = []
    occupied: set = set()

    def mark_rect(x0: float, y0: float, x1: float, y1: float, step: float = 0.4) -> None:
        x = min(x0, x1)
        while x <= max(x0, x1):
            y = min(y0, y1)
            while y <= max(y0, y1):
                pts.append((x, y))
                occupied.add(
                    (int(math.floor((x - min_x) / plan_res)), int(math.floor((y - min_y) / plan_res)))
                )
                y += step
            x += step

    # Perimeter only — center runway (-25..25, ±15) stays free
    wall_th = 1.6
    for x in range(-50, 51):
        mark_rect(float(x), -50.0, float(x), -50.0 + wall_th)
        mark_rect(float(x), 50.0 - wall_th, float(x), 50.0)
    for y in range(-50, 51):
        mark_rect(-50.0, float(y), -50.0 + wall_th, float(y))
        mark_rect(50.0 - wall_th, float(y), 50.0, float(y))

    occupied_raw = set(occupied)
    occupied, actual_inflate = _inflate_meters(occupied, plan_res, inflate_m)
    pois = [
        MapPOI("M32_A", -25.0, 0.0, "M32Start", yaw=0.0),
        MapPOI("M32_B", 25.0, 0.0, "M32Goal", yaw=0.0),
    ]
    return LoadedSmap(
        name="m32_open_straight",
        path="builtin:m32_open_straight",
        min_x=min_x,
        min_y=min_y,
        max_x=max_x,
        max_y=max_y,
        resolution=0.05,
        cloud=_downsample(pts, 6000),
        occupied=occupied,
        occupied_raw=occupied_raw,
        plan_res=plan_res,
        pois=pois,
        scene_kind="open",
        path_color="#22C55E",
        requested_inflate_m=float(inflate_m),
        actual_inflate_m=float(actual_inflate),
    )


def default_smap_candidates() -> List[Path]:
    # .../V0.1仿真版/ros2_ws/src/agv_bridge/agv_bridge/smap_loader.py
    here = Path(__file__).resolve()
    v01 = here.parents[4]  # V0.1仿真版
    project = here.parents[5]  # AGV项目
    cands = [
        project / "agv_downloaded" / "maps" / "20260723112931750.smap",
        project / "agv_downloaded" / "maps" / "20260621164335249.smap",
        project / "agv_downloaded" / "maps" / "default.smap",
        v01 / "maps" / "office.smap",
    ]
    return [p for p in cands if p.is_file()]
