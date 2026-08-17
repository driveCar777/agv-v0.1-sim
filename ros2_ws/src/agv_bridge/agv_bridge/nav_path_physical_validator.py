"""Global path physical feasibility validator (V0.2 M3.3).

Distinguishes:
  GLOBAL_PATH_REACHABLE       — A* returned a polyline
  GLOBAL_PATH_PHYSically_FEASIBLE — footprint + swept volume + clearance OK

Does not emit commands or alter planner behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import (
    SWEPT_SPATIAL_STEP_M,
    trajectory_collision,
    trajectory_min_clearance,
)
from agv_bridge.nav_geometry import DEFAULT_GEOM, BrakingModel, VehicleGeometry, get_vehicle_model
from agv_bridge.nav_trajectory import TrajectorySample

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

REASON_NONE = "NONE"
REASON_EMPTY = "EMPTY_PATH"
REASON_FOOTPRINT_COLLISION = "FOOTPRINT_COLLISION"
REASON_CLEARANCE = "CLEARANCE_TOO_LOW"
REASON_TURN_RADIUS = "TURN_RADIUS_INFEASIBLE"
REASON_CORRIDOR_WIDTH = "CORRIDOR_TOO_NARROW"
REASON_BRAKING = "BRAKING_INFEASIBLE"
REASON_BRAKING_NA = "BRAKING_MODEL_UNAVAILABLE"

PATH_CLASS_A = "A"  # not reachable
PATH_CLASS_B = "B"  # reachable, not physically feasible
PATH_CLASS_C = "C"  # physically feasible (local may still fail)


@dataclass
class SegmentAudit:
    index: int
    from_pt: Pt
    to_pt: Pt
    length_m: float
    yaw_rad: float
    turn_deg: Optional[float]
    physically_feasible: bool
    min_clearance_m: Optional[float]
    invalid_reason: str = REASON_NONE
    first_collision_pose: Optional[Dict[str, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "from": {"x": self.from_pt[0], "y": self.from_pt[1]},
            "to": {"x": self.to_pt[0], "y": self.to_pt[1]},
            "length_m": round(self.length_m, 3),
            "yaw_deg": round(math.degrees(self.yaw_rad), 2),
            "turn_deg": None if self.turn_deg is None else round(self.turn_deg, 2),
            "physically_feasible": self.physically_feasible,
            "min_clearance_m": None if self.min_clearance_m is None else round(self.min_clearance_m, 4),
            "invalid_reason": self.invalid_reason,
            "first_collision_pose": self.first_collision_pose,
        }


@dataclass
class PathPhysicalValidationResult:
    global_path_reachable: bool = False
    physically_feasible: bool = False
    path_class: str = PATH_CLASS_A
    path_min_clearance_m: Optional[float] = None
    minimum_corridor_width_m: Optional[float] = None
    first_invalid_index: int = -1
    first_invalid_pose: Optional[Dict[str, float]] = None
    first_collision_reason: str = REASON_NONE
    first_collision_index: int = -1
    kinematic_feasible: bool = True
    turning_radius_feasible: bool = True
    braking_feasible: Optional[bool] = None
    required_corridor_width_m: float = 0.0
    segment_audits: List[SegmentAudit] = field(default_factory=list)
    samples_checked: int = 0
    inflation_contract: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "global_path_reachable": self.global_path_reachable,
            "physically_feasible": self.physically_feasible,
            "path_class": self.path_class,
            "path_min_clearance_m": self.path_min_clearance_m,
            "minimum_corridor_width_m": self.minimum_corridor_width_m,
            "first_invalid_index": self.first_invalid_index,
            "first_invalid_pose": self.first_invalid_pose,
            "first_collision_reason": self.first_collision_reason,
            "first_collision_index": self.first_collision_index,
            "kinematic_feasible": self.kinematic_feasible,
            "turning_radius_feasible": self.turning_radius_feasible,
            "braking_feasible": self.braking_feasible,
            "required_corridor_width_m": round(self.required_corridor_width_m, 3),
            "segment_audits": [s.to_dict() for s in self.segment_audits],
            "samples_checked": self.samples_checked,
            "inflation_contract": self.inflation_contract,
        }


def _segment_yaw(a: Pt, b: Pt) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _turn_angle_deg(prev: Pt, mid: Pt, nxt: Pt) -> Optional[float]:
    v1 = (mid[0] - prev[0], mid[1] - prev[1])
    v2 = (nxt[0] - mid[0], nxt[1] - mid[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return None
    cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cos))


def _interpolate_segment(a: Pt, b: Pt, step_m: float) -> List[TrajectorySample]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    yaw = _segment_yaw(a, b)
    if length < 1e-6:
        return [TrajectorySample(0.0, a[0], a[1], yaw, 0.15, 0.0)]
    n = max(1, int(math.ceil(length / max(0.05, step_m))))
    poses: List[TrajectorySample] = []
    for k in range(n + 1):
        t = k / n
        poses.append(
            TrajectorySample(
                t * length / 0.15,
                a[0] + dx * t,
                a[1] + dy * t,
                yaw,
                0.15,
                0.0,
            )
        )
    return poses


def _min_turn_radius(geom: VehicleGeometry) -> float:
    w_max = float(geom.max_w or 0.45)
    v_ref = 0.15
    if w_max < 1e-6:
        return float("inf")
    return v_ref / w_max


def inflation_ownership_contract(map_inflate_m: float, validator_margin_m: float) -> Dict[str, Any]:
    """Document who owns which safety layer (audit-only, no behavior change)."""
    return {
        "raw_obstacle": "smap_loader occupied_raw (static map points)",
        "map_inflation_m": map_inflate_m,
        "map_inflation_owner": "GLOBAL_PLANNER / SimWorld.plan_path (A* on m.occupied inflated grid)",
        "global_planner_robot_r": "VehicleGeometry.planner_radius (broad-phase disk for A* free cells)",
        "footprint_owner": "nav_footprint (narrow-phase rectangle from length/width)",
        "validator_margin_m": validator_margin_m,
        "validator_margin_owner": "PathPhysicalValidator + TrajectoryValidator runtime safety margin on footprint samples",
        "note": "map inflation and validator margin are different layers; not double-counting the same responsibility",
    }


def validate_global_path(
    path: Sequence[Pt],
    collide: CollideFn,
    clearance_at: ClearanceFn,
    *,
    geom: VehicleGeometry = DEFAULT_GEOM,
    margin_m: Optional[float] = None,
    spatial_step_m: float = SWEPT_SPATIAL_STEP_M,
    map_inflate_m: float = 0.28,
    braking: Optional[BrakingModel] = None,
    cruise_vx_mps: float = 0.20,
) -> PathPhysicalValidationResult:
    margin_m = float(geom.safety_margin_m if margin_m is None else margin_m)
    braking = braking or get_vehicle_model().braking
    required_width = float(geom.width) + 2.0 * margin_m
    out = PathPhysicalValidationResult(
        required_corridor_width_m=required_width,
        inflation_contract=inflation_ownership_contract(map_inflate_m, margin_m),
    )

    if not path or len(path) < 2:
        out.path_class = PATH_CLASS_A
        out.first_collision_reason = REASON_EMPTY
        return out

    out.global_path_reachable = True
    min_clr_global: Optional[float] = None
    min_corridor: Optional[float] = None
    r_min = _min_turn_radius(geom)

    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        yaw = _segment_yaw(a, b)
        turn = None
        if 0 < i < len(path) - 1:
            turn = _turn_angle_deg(path[i - 1], path[i], path[i + 1])
        elif i == 0 and len(path) >= 3:
            turn = _turn_angle_deg(path[0], path[1], path[2])

        poses = _interpolate_segment(a, b, spatial_step_m)
        coll = trajectory_collision(poses, collide, geom, margin_m=margin_m, spatial_step_m=spatial_step_m)
        clr = trajectory_min_clearance(poses, clearance_at, geom, margin_m=margin_m, spatial_step_m=spatial_step_m)
        out.samples_checked += coll.samples_checked

        seg_ok = not coll.collision
        seg_reason = REASON_NONE
        first_pose = None
        if coll.collision:
            seg_ok = False
            seg_reason = REASON_FOOTPRINT_COLLISION
            first_pose = coll.first_pose
        elif clr.minimum_clearance_m is not None and clr.minimum_clearance_m < margin_m:
            seg_ok = False
            seg_reason = REASON_CLEARANCE
            first_pose = {"x": poses[0].x, "y": poses[0].y, "yaw": poses[0].yaw}

        turn_ok = True
        if turn is not None and turn > 5.0 and length >= 0.30:
            # Rough corner radius from segment lengths and turn angle
            half_angle = math.radians(turn) * 0.5
            if half_angle > 1e-3:
                arc_r = min(length, 2.0) / (2.0 * math.sin(half_angle))
                if arc_r < r_min * 0.85:
                    turn_ok = False
                    if seg_ok:
                        seg_ok = False
                        seg_reason = REASON_TURN_RADIUS

        if clr.minimum_clearance_m is not None:
            eff_width = float(geom.width) + 2.0 * clr.minimum_clearance_m
            min_corridor = eff_width if min_corridor is None else min(min_corridor, eff_width)
            min_clr_global = clr.minimum_clearance_m if min_clr_global is None else min(min_clr_global, clr.minimum_clearance_m)

        seg = SegmentAudit(
            index=i,
            from_pt=a,
            to_pt=b,
            length_m=length,
            yaw_rad=yaw,
            turn_deg=turn,
            physically_feasible=seg_ok and turn_ok,
            min_clearance_m=clr.minimum_clearance_m,
            invalid_reason=seg_reason if not seg_ok else REASON_NONE,
            first_collision_pose=first_pose,
        )
        out.segment_audits.append(seg)

        if not seg_ok and out.first_invalid_index < 0:
            out.first_invalid_index = i
            out.first_collision_index = i
            out.first_invalid_pose = first_pose or {"x": a[0], "y": a[1], "yaw": yaw}
            out.first_collision_reason = seg_reason
            out.turning_radius_feasible = turn_ok
            out.kinematic_feasible = turn_ok

    out.path_min_clearance_m = min_clr_global
    out.minimum_corridor_width_m = min_corridor
    out.physically_feasible = out.first_invalid_index < 0
    out.path_class = PATH_CLASS_B if not out.physically_feasible else PATH_CLASS_C

    decel = braking.max_decel_mps2
    if decel is None or decel <= 0:
        out.braking_feasible = None
    else:
        stop_dist = cruise_vx_mps * cruise_vx_mps / (2.0 * decel) + float(braking.safety_margin_m)
        out.braking_feasible = (out.path_min_clearance_m or 99.0) > stop_dist * 0.3

    return out


def classify_path_case(reachable: bool, validation: PathPhysicalValidationResult) -> str:
    if not reachable:
        return PATH_CLASS_A
    if not validation.physically_feasible:
        return PATH_CLASS_B
    return PATH_CLASS_C
