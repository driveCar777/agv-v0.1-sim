"""Unified vehicle footprint + swept-volume geometry — P0-A.

VehicleGeometry (nav_geometry.DEFAULT_GEOM) is the ONLY size truth source.
Radius fields remain BROAD-PHASE approximations; footprint polygon is NARROW-PHASE truth.

Does NOT change NavigationPolicy / ManeuverFSM / MPPI horizon / Recovery semantics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
PoseLike = Any  # object with .x .y .yaw OR (x,y,yaw) OR dict

# Discrete swept sampling (documented in P0-A report)
SWEPT_SPATIAL_STEP_M = 0.08
SWEPT_ANGULAR_STEP_RAD = math.radians(5.0)

# Collision model labels (telemetry)
BROAD_PHASE_MODEL = "bounding_radius"
NARROW_PHASE_MODEL = "footprint_polygon_samples"


def get_vehicle_geometry() -> VehicleGeometry:
    """Single access point — never invent a second size table."""
    return DEFAULT_GEOM


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _as_pose(p: PoseLike) -> Tuple[float, float, float]:
    if isinstance(p, dict):
        return float(p["x"]), float(p["y"]), float(p.get("yaw", p.get("theta", 0.0)))
    if isinstance(p, (tuple, list)) and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return float(p.x), float(p.y), float(p.yaw)


def footprint_polygon_body(
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
) -> List[Pt]:
    """Axis-aligned rectangle in body frame: FL, FR, RR, RL.

    +x front, +y left. Uses length/width (and bumper_l as front/rear overhang).
    """
    # Prefer bumper_l as overhang when consistent; else half-length
    half_l = 0.5 * float(geom.length)
    half_w = 0.5 * float(geom.width) + float(margin_m)
    xf = float(getattr(geom, "front_overhang_m", None) or geom.bumper_l or half_l) + float(margin_m)
    xr = -(float(getattr(geom, "rear_overhang_m", None) or geom.bumper_l or half_l) + float(margin_m))
    # If bumper_l shorter than half length, keep rectangle within length
    xf = min(xf, half_l + float(margin_m))
    xr = max(xr, -half_l - float(margin_m))
    ox = float(getattr(geom, "center_offset_x_m", 0.0) or 0.0)
    return [
        (ox + xf, half_w),   # front-left
        (ox + xf, -half_w),  # front-right
        (ox + xr, -half_w),  # rear-right
        (ox + xr, half_w),   # rear-left
    ]


def transform_footprint(
    x: float,
    y: float,
    yaw: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
) -> List[Pt]:
    """World-frame footprint polygon (FL→FR→RR→RL)."""
    c, s = math.cos(yaw), math.sin(yaw)
    out: List[Pt] = []
    for lx, ly in footprint_polygon_body(geom, margin_m=margin_m):
        out.append((x + c * lx - s * ly, y + s * lx + c * ly))
    return out


def footprint_points(
    x: float,
    y: float,
    yaw: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
) -> List[Pt]:
    """Narrow-phase samples: 4 corners + edge midpoints + center (covers overhang)."""
    poly = transform_footprint(x, y, yaw, geom, margin_m=margin_m)
    if len(poly) < 4:
        return poly
    fl, fr, rr, rl = poly[0], poly[1], poly[2], poly[3]

    def mid(a: Pt, b: Pt) -> Pt:
        return (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))

    return [
        fl,
        fr,
        rr,
        rl,
        mid(fl, fr),  # front-center
        mid(rl, rr),  # rear-center
        mid(fl, rl),  # left-center
        mid(fr, rr),  # right-center
        (x, y),       # center
    ]


def footprint_collide(
    x: float,
    y: float,
    yaw: float,
    collide: CollideFn,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
) -> bool:
    """Narrow-phase: any footprint sample hits collide()."""
    return any(collide(px, py) for px, py in footprint_points(x, y, yaw, geom, margin_m=margin_m))


def broad_phase_radius(geom: VehicleGeometry = DEFAULT_GEOM, *, kind: str = "local") -> float:
    """Bounding circle for early rejection only — NOT final truth."""
    k = (kind or "local").lower()
    if k == "planner":
        return float(geom.planner_radius)
    if k == "safety":
        return float(geom.safety_radius)
    return float(geom.local_radius)


def interpolate_poses(
    poses: Sequence[PoseLike],
    *,
    spatial_step_m: float = SWEPT_SPATIAL_STEP_M,
    angular_step_rad: float = SWEPT_ANGULAR_STEP_RAD,
) -> List[Tuple[float, float, float]]:
    """Insert intermediate poses so |Δs| and |Δyaw| stay within steps."""
    if not poses:
        return []
    out: List[Tuple[float, float, float]] = []
    prev = _as_pose(poses[0])
    out.append(prev)
    spat = max(1e-3, float(spatial_step_m))
    ang = max(1e-4, float(angular_step_rad))
    for raw in poses[1:]:
        cur = _as_pose(raw)
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        dist = math.hypot(dx, dy)
        dyaw = _wrap(cur[2] - prev[2])
        n = max(1, int(math.ceil(dist / spat)), int(math.ceil(abs(dyaw) / ang)))
        for k in range(1, n + 1):
            t = k / n
            x = prev[0] + dx * t
            y = prev[1] + dy * t
            yaw = _wrap(prev[2] + dyaw * t)
            out.append((x, y, yaw))
        prev = cur
    return out


@dataclass
class FootprintPose:
    x: float
    y: float
    yaw: float
    polygon: List[Pt] = field(default_factory=list)
    samples: List[Pt] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 4),
            "polygon": [{"x": round(a, 3), "y": round(b, 3)} for a, b in self.polygon],
        }


def sample_swept_footprint(
    poses: Sequence[PoseLike],
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
    spatial_step_m: float = SWEPT_SPATIAL_STEP_M,
    angular_step_rad: float = SWEPT_ANGULAR_STEP_RAD,
) -> List[FootprintPose]:
    """Dense footprint poses along trajectory (outer expansion captured via corners)."""
    dense = interpolate_poses(poses, spatial_step_m=spatial_step_m, angular_step_rad=angular_step_rad)
    out: List[FootprintPose] = []
    for x, y, yaw in dense:
        poly = transform_footprint(x, y, yaw, geom, margin_m=margin_m)
        out.append(
            FootprintPose(
                x=x,
                y=y,
                yaw=yaw,
                polygon=poly,
                samples=footprint_points(x, y, yaw, geom, margin_m=margin_m),
            )
        )
    return out


@dataclass
class CollisionResult:
    collision: bool = False
    first_index: int = -1
    first_pose: Optional[Dict[str, float]] = None
    hit_point: Optional[Pt] = None
    samples_checked: int = 0
    model: str = NARROW_PHASE_MODEL

    def to_dict(self) -> Dict[str, Any]:
        return {
            "collision": self.collision,
            "first_index": self.first_index,
            "first_pose": self.first_pose,
            "hit_point": None
            if self.hit_point is None
            else {"x": round(self.hit_point[0], 3), "y": round(self.hit_point[1], 3)},
            "samples_checked": self.samples_checked,
            "model": self.model,
        }


def trajectory_collision(
    poses: Sequence[PoseLike],
    collide: CollideFn,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
    spatial_step_m: float = SWEPT_SPATIAL_STEP_M,
    angular_step_rad: float = SWEPT_ANGULAR_STEP_RAD,
) -> CollisionResult:
    """Narrow-phase swept collision along interpolated trajectory."""
    res = CollisionResult()
    swept = sample_swept_footprint(
        poses,
        geom,
        margin_m=margin_m,
        spatial_step_m=spatial_step_m,
        angular_step_rad=angular_step_rad,
    )
    for i, fp in enumerate(swept):
        for px, py in fp.samples:
            res.samples_checked += 1
            if collide(px, py):
                res.collision = True
                res.first_index = i
                res.first_pose = {"x": fp.x, "y": fp.y, "yaw": fp.yaw}
                res.hit_point = (px, py)
                return res
    return res


def sample_rotation_sweep(
    x: float,
    y: float,
    yaw_start: float,
    yaw_end: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
    angular_step_rad: float = SWEPT_ANGULAR_STEP_RAD,
) -> List[FootprintPose]:
    """In-place yaw sweep footprints (geometry only — no FSM policy)."""
    dyaw = _wrap(float(yaw_end) - float(yaw_start))
    n = max(1, int(math.ceil(abs(dyaw) / max(1e-4, angular_step_rad))))
    out: List[FootprintPose] = []
    for k in range(n + 1):
        t = k / n
        yaw = _wrap(float(yaw_start) + dyaw * t)
        poly = transform_footprint(x, y, yaw, geom, margin_m=margin_m)
        out.append(
            FootprintPose(
                x=x,
                y=y,
                yaw=yaw,
                polygon=poly,
                samples=footprint_points(x, y, yaw, geom, margin_m=margin_m),
            )
        )
    return out


def swept_boundary_edges(
    poses: Sequence[PoseLike],
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    margin_m: float = 0.0,
    spatial_step_m: float = SWEPT_SPATIAL_STEP_M,
    angular_step_rad: float = SWEPT_ANGULAR_STEP_RAD,
) -> Tuple[List[Pt], List[Pt], List[Pt]]:
    """Left/right outer edges + approximate swept polygon from footprint corners.

    At each dense pose, pick max body-+y (left) and min body-+y (right) among
    footprint polygon vertices — captures turn outer expansion / overhang.
    """
    swept = sample_swept_footprint(
        poses,
        geom,
        margin_m=margin_m,
        spatial_step_m=spatial_step_m,
        angular_step_rad=angular_step_rad,
    )
    left: List[Pt] = []
    right: List[Pt] = []
    for fp in swept:
        c, s = math.cos(fp.yaw), math.sin(fp.yaw)
        best_l: Optional[Pt] = None
        best_r: Optional[Pt] = None
        best_ly = -1e18
        best_ry = 1e18
        for px, py in fp.polygon:
            # body y: left positive
            by = -s * (px - fp.x) + c * (py - fp.y)
            if by > best_ly:
                best_ly = by
                best_l = (px, py)
            if by < best_ry:
                best_ry = by
                best_r = (px, py)
        if best_l:
            left.append(best_l)
        if best_r:
            right.append(best_r)
    poly: List[Pt] = list(left) + list(reversed(right))
    return left, right, poly


def geometry_telemetry(geom: VehicleGeometry = DEFAULT_GEOM) -> Dict[str, Any]:
    """P0-A debug fields."""
    return {
        "length_m": round(float(geom.length), 3),
        "width_m": round(float(geom.width), 3),
        "front_overhang_m": round(max(p[0] for p in footprint_polygon_body(geom)), 3),
        "rear_overhang_m": round(abs(min(p[0] for p in footprint_polygon_body(geom))), 3),
        "bumper_sample_m": round(float(geom.bumper_l), 3),
        "polygon_half_length_m": round(0.5 * float(geom.length), 3),
        "center_offset_x_m": round(float(getattr(geom, "center_offset_x_m", 0.0) or 0.0), 3),
        "safety_margin_m": round(float(getattr(geom, "safety_margin_m", 0.08) or 0.08), 3),
        "track_width_m": getattr(geom, "track_width_m", None),
        "wheelbase_m": getattr(geom, "wheelbase_m", None),
        "track_width_unavailable": getattr(geom, "track_width_m", None) is None,
        "wheelbase_unavailable": getattr(geom, "wheelbase_m", None) is None,
        "footprint_model": "polygon",
        "collision_model_broad_phase": BROAD_PHASE_MODEL,
        "collision_model_narrow_phase": NARROW_PHASE_MODEL,
        "swept_sampling_spatial_m": SWEPT_SPATIAL_STEP_M,
        "swept_sampling_angular_deg": round(math.degrees(SWEPT_ANGULAR_STEP_RAD), 2),
        "planner_radius_approx": round(float(geom.planner_radius), 3),
        "local_radius_approx": round(float(geom.local_radius), 3),
        "safety_radius_approx": round(float(geom.safety_radius), 3),
        "note": "radius=broad-phase approx; footprint polygon=narrow-phase truth",
    }
