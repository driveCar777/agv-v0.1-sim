"""Physical Trajectory Corridor — shared geometry for Local Physical Trajectory.

P0-A: Local corridor edges/swept_polygon come from VehicleGeometry footprint
swept sampling (nav_footprint), NOT centerline ± half_width as final truth.

Does NOT invent a second integrator. Consumes poses from rollout_candidate /
Probe reverse samples. Global Reference Corridor is P0-B (not here).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import (
    SWEPT_ANGULAR_STEP_RAD,
    SWEPT_SPATIAL_STEP_M,
    footprint_points,
    geometry_telemetry,
    swept_boundary_edges,
)
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry

Pt = Tuple[float, float]

STATUS_VALID = "VALID"
STATUS_INVALID = "INVALID"
STATUS_UNKNOWN = "UNKNOWN"
STATUS_STALE = "STALE"

SRC_FORWARD = "FORWARD"
SRC_LEFT = "LEFT"
SRC_RIGHT = "RIGHT"
SRC_BACKWARD = "BACKWARD"
SRC_TURN = "TURN_IN_PLACE"
SRC_RETREAT = "HISTORICAL_RETREAT"
SRC_EXECUTED = "EXECUTED"
SRC_MPPI = "MPPI"


@dataclass
class TrajectorySample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    w: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t": round(self.t, 3),
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 4),
            "vx": round(self.vx, 3),
            "w": round(self.w, 3),
        }


def corridor_inflate_m(
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    safety: float = 0.08,
    loc: float = 0.05,
    ctrl: float = 0.05,
) -> float:
    """Margin beyond geometric body (NOT a free UI fudge). Prefer geom.safety_margin_m."""
    base = float(getattr(geom, "safety_margin_m", None) or safety)
    return float(base + loc + ctrl)


def footprint_half_width(geom: VehicleGeometry = DEFAULT_GEOM, *, inflate: Optional[float] = None) -> float:
    """Diagnostic lateral half-width (legacy UI). Not swept-volume truth."""
    inf = corridor_inflate_m(geom) if inflate is None else float(inflate)
    return 0.5 * float(geom.width) + inf


def poses_from_rollout_path(
    path: Sequence[Any],
    *,
    yaw0: float,
    vx: float = 0.0,
    w: float = 0.0,
    dt: float = 0.1,
) -> List[TrajectorySample]:
    out: List[TrajectorySample] = []
    yaw = float(yaw0)
    for i, p in enumerate(path or []):
        if isinstance(p, dict):
            x = float(p.get("x") or 0.0)
            y = float(p.get("y") or 0.0)
            if p.get("yaw") is not None or p.get("theta") is not None:
                yaw = float(p.get("yaw", p.get("theta")))
            elif i > 0:
                yaw = _wrap(yaw + float(w) * dt)
        else:
            x, y = float(p[0]), float(p[1])
            if i > 0:
                yaw = _wrap(yaw + float(w) * dt)
        out.append(TrajectorySample(t=i * dt, x=x, y=y, yaw=yaw, vx=vx, w=w))
    return out


def poses_from_xy_yaw(
    xs: Sequence[float],
    ys: Sequence[float],
    yaws: Sequence[float],
    *,
    dt: float = 0.1,
    vx: float = 0.0,
    w: float = 0.0,
) -> List[TrajectorySample]:
    return [
        TrajectorySample(t=i * dt, x=float(xs[i]), y=float(ys[i]), yaw=float(yaws[i]), vx=vx, w=w)
        for i in range(min(len(xs), len(ys), len(yaws)))
    ]


@dataclass
class PhysicalTrajectoryCorridor:
    """Local Physical Trajectory swept corridor (footprint⊕margin). Not Global Ref (P0-B)."""

    source: str
    poses: List[TrajectorySample] = field(default_factory=list)
    status: str = STATUS_UNKNOWN
    collision: bool = False
    soft_risk: bool = False
    min_clearance: Optional[float] = None
    length_m: float = 0.0
    duration_s: float = 0.0
    half_width_m: float = 0.0
    left_edge: List[Pt] = field(default_factory=list)
    right_edge: List[Pt] = field(default_factory=list)
    swept_polygon: List[Pt] = field(default_factory=list)
    centerline: List[Pt] = field(default_factory=list)
    failure_reason: str = "NONE"
    risk: str = "NONE"
    progress: Optional[float] = None
    valid: bool = False
    geometry_model: str = "swept_footprint_polygon"

    def to_dict(self, *, max_poses: int = 24, max_poly: int = 48) -> Dict[str, Any]:
        poses = self.poses
        if len(poses) > max_poses:
            step = max(1, len(poses) // max_poses)
            poses = poses[::step]
            if poses[-1] is not self.poses[-1]:
                poses = list(poses) + [self.poses[-1]]
        poly = self.swept_polygon
        if len(poly) > max_poly:
            step = max(1, len(poly) // max_poly)
            poly = poly[::step]
        return {
            "source": self.source,
            "status": self.status,
            "valid": self.valid,
            "collision": self.collision,
            "soft_risk": self.soft_risk,
            "min_clearance": None if self.min_clearance is None else round(float(self.min_clearance), 3),
            "length_m": round(self.length_m, 3),
            "duration_s": round(self.duration_s, 3),
            "half_width_m": round(self.half_width_m, 3),
            "failure_reason": self.failure_reason,
            "risk": self.risk,
            "progress": self.progress,
            "geometry_model": self.geometry_model,
            "swept_sampling_spatial_m": SWEPT_SPATIAL_STEP_M,
            "swept_sampling_angular_deg": round(math.degrees(SWEPT_ANGULAR_STEP_RAD), 2),
            "poses": [p.to_dict() for p in poses],
            "centerline": [{"x": round(a, 3), "y": round(b, 3)} for a, b in self.centerline[:max_poses]],
            "swept_polygon": [{"x": round(a, 3), "y": round(b, 3)} for a, b in poly],
            "left_edge": [{"x": round(a, 3), "y": round(b, 3)} for a, b in self.left_edge[:max_poses]],
            "right_edge": [{"x": round(a, 3), "y": round(b, 3)} for a, b in self.right_edge[:max_poses]],
            "note": "Local Physical Trajectory: footprint swept⊕margin (not centerline ribbon truth)",
        }


def build_corridor_from_poses(
    poses: Sequence[TrajectorySample],
    *,
    source: str,
    status: str = STATUS_UNKNOWN,
    collision: bool = False,
    soft_risk: bool = False,
    min_clearance: Optional[float] = None,
    failure_reason: str = "NONE",
    progress: Optional[float] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
    inflate: Optional[float] = None,
) -> PhysicalTrajectoryCorridor:
    """Build Local Physical Trajectory corridor via footprint swept edges."""
    margin = corridor_inflate_m(geom) if inflate is None else float(inflate)
    hw = footprint_half_width(geom, inflate=margin)
    center: List[Pt] = []
    length = 0.0
    prev: Optional[TrajectorySample] = None
    for p in poses:
        center.append((p.x, p.y))
        if prev is not None:
            length += math.hypot(p.x - prev.x, p.y - prev.y)
        prev = p
    if len(poses) >= 1:
        left, right, poly = swept_boundary_edges(poses, geom, margin_m=margin)
    else:
        left, right, poly = [], [], []
    if len(left) < 2 and len(poses) >= 2:
        for p in poses:
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            lx, ly = -s, c
            left.append((p.x + lx * hw, p.y + ly * hw))
            right.append((p.x - lx * hw, p.y - ly * hw))
        poly = list(left) + list(reversed(right))
    duration = float(poses[-1].t - poses[0].t) if len(poses) >= 2 else 0.0
    valid = status == STATUS_VALID and not collision
    risk = "HARD" if collision or status == STATUS_INVALID else ("SOFT" if soft_risk else "NONE")
    return PhysicalTrajectoryCorridor(
        source=source,
        poses=list(poses),
        status=status,
        collision=collision,
        soft_risk=soft_risk,
        min_clearance=min_clearance,
        length_m=length,
        duration_s=duration,
        half_width_m=hw,
        left_edge=left,
        right_edge=right,
        swept_polygon=poly,
        centerline=center,
        failure_reason=failure_reason,
        risk=risk,
        progress=progress,
        valid=valid,
        geometry_model="swept_footprint_polygon",
    )


def corridor_from_probe_result(
    pr: Any, *, yaw0: float, vx: float = 0.0, w: float = 0.0
) -> Optional[PhysicalTrajectoryCorridor]:
    poses = getattr(pr, "poses", None)
    if not poses:
        path = getattr(pr, "path", None)
        if path:
            poses = poses_from_rollout_path(path, yaw0=yaw0, vx=vx, w=w)
        else:
            return None
    if poses and isinstance(poses[0], dict):
        poses = [
            TrajectorySample(
                t=float(p.get("t") or i * 0.1),
                x=float(p["x"]),
                y=float(p["y"]),
                yaw=float(p.get("yaw") or p.get("theta") or yaw0),
                vx=float(p.get("vx") or vx),
                w=float(p.get("w") or w),
            )
            for i, p in enumerate(poses)
        ]
    return build_corridor_from_poses(
        poses,
        source=str(getattr(pr, "direction", SRC_FORWARD)),
        status=str(getattr(pr, "status", STATUS_UNKNOWN)),
        collision=bool(getattr(pr, "collision", False)),
        soft_risk=bool(getattr(pr, "soft_risk", False)),
        min_clearance=getattr(pr, "min_clearance", None),
        failure_reason=str(getattr(pr, "failure_reason", "NONE")),
        progress=getattr(pr, "predicted_progress", None),
    )


def visual_status_color(status: str, *, soft_risk: bool = False) -> str:
    s = str(status or "").upper()
    if s == STATUS_VALID and soft_risk:
        return "YELLOW"
    if s == STATUS_VALID:
        return "GREEN"
    if s == STATUS_INVALID:
        return "RED"
    if s in (STATUS_UNKNOWN, STATUS_STALE):
        return "GRAY"
    return "BLUE"


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def footprint_sample_points(x: float, y: float, yaw: float, geom: VehicleGeometry = DEFAULT_GEOM) -> List[Pt]:
    return footprint_points(x, y, yaw, geom)


def local_corridor_geometry_meta(geom: VehicleGeometry = DEFAULT_GEOM) -> Dict[str, Any]:
    return geometry_telemetry(geom)
