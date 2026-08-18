"""V0.2 M1 trajectory validator.

Independent post-rollout checks shared by local planning, MPPI and Safety.
This module does not own behavior decisions and does not emit commands.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence

from agv_bridge.nav_footprint import (
    CollisionResult,
    ClearanceResult,
    current_footprint_clearance,
    trajectory_collision,
    trajectory_min_clearance,
)
from agv_bridge.nav_execution_corridor import ExecutionCorridor, corridor_allows_omega_sign
from agv_bridge.nav_geometry import DEFAULT_GEOM, BrakingModel, VehicleGeometry, get_vehicle_model

PoseLike = Any
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

REASON_NONE = "NONE"
REASON_EMPTY = "EMPTY_TRAJECTORY"
REASON_COLLISION = "FOOTPRINT_COLLISION"
REASON_CLEARANCE = "CLEARANCE_TOO_LOW"
REASON_VX_LIMIT = "VX_LIMIT"
REASON_W_LIMIT = "OMEGA_LIMIT"
REASON_AX_LIMIT = "ACCEL_LIMIT"
REASON_CORRIDOR = "CORRIDOR_SIDE_BLOCKED"
REASON_BRAKING = "BRAKING_INFEASIBLE"
REASON_BRAKING_UNAVAILABLE = "BRAKING_MODEL_UNAVAILABLE"
REASON_AY_LIMIT = "LATERAL_ACCEL_LIMIT"
REASON_ALPHA_LIMIT = "ALPHA_LIMIT"
REASON_JERK_LIMIT = "JERK_LIMIT"


@dataclass
class TrajectoryValidationResult:
    valid: bool = True
    reason: str = REASON_NONE
    reasons: List[str] = field(default_factory=list)
    collision: CollisionResult = field(default_factory=CollisionResult)
    minimum_clearance: ClearanceResult = field(default_factory=ClearanceResult)
    current_clearance: ClearanceResult = field(default_factory=ClearanceResult)
    max_abs_vx: float = 0.0
    max_abs_w: float = 0.0
    max_abs_ax: float = 0.0
    pose_count: int = 0
    source: str = "TRAJECTORY_VALIDATOR"

    def reject(self, reason: str) -> None:
        if reason not in self.reasons:
            self.reasons.append(reason)
        self.valid = False
        if self.reason == REASON_NONE:
            self.reason = reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "valid": self.valid,
            "reason": self.reason,
            "reasons": list(self.reasons),
            "collision": self.collision.to_dict(),
            "minimum_clearance": self.minimum_clearance.to_dict(),
            "current_clearance": self.current_clearance.to_dict(),
            "max_abs_vx": round(self.max_abs_vx, 4),
            "max_abs_w": round(self.max_abs_w, 4),
            "max_abs_ax": round(self.max_abs_ax, 4),
            "pose_count": self.pose_count,
            "source": self.source,
        }


def _pose_xy_yaw(p: PoseLike) -> tuple[float, float, float]:
    if isinstance(p, dict):
        return float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("yaw", p.get("theta", 0.0)))
    if isinstance(p, (tuple, list)) and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return float(p.x), float(p.y), float(p.yaw)


def _segment_dt(prev: PoseLike, cur: PoseLike, default_dt: float) -> float:
    if isinstance(cur, dict):
        t1 = cur.get("t")
        if t1 is None:
            t1 = cur.get("time_s")
        if isinstance(prev, dict):
            t0 = prev.get("t")
            if t0 is None:
                t0 = prev.get("time_s")
            if t0 is not None and t1 is not None:
                return max(1e-3, float(t1) - float(t0))
    return max(1e-3, float(default_dt))


def _infer_dynamics(poses: Sequence[PoseLike], dt: float) -> tuple[float, float, float]:
    max_abs_vx = 0.0
    max_abs_w = 0.0
    max_abs_ax = 0.0
    prev_vx: Optional[float] = None
    prev_yaw: Optional[float] = None
    prev_xy: Optional[tuple[float, float]] = None
    prev_raw: Optional[PoseLike] = None
    for raw in poses:
        x, y, yaw = _pose_xy_yaw(raw)
        seg_dt = _segment_dt(prev_raw, raw, dt) if prev_raw is not None else max(1e-3, float(dt))
        if prev_xy is not None:
            ds = math.hypot(x - prev_xy[0], y - prev_xy[1])
            vx = ds / seg_dt
            max_abs_vx = max(max_abs_vx, abs(vx))
            if prev_vx is not None:
                max_abs_ax = max(max_abs_ax, abs(vx - prev_vx) / seg_dt)
            prev_vx = vx
        prev_xy = (x, y)
        if prev_yaw is not None:
            dyaw = yaw - prev_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            max_abs_w = max(max_abs_w, abs(dyaw / seg_dt))
        prev_yaw = yaw
        prev_raw = raw
    return max_abs_vx, max_abs_w, max_abs_ax


def _corridor_rejects_poses(
    poses: Sequence[PoseLike],
    execution_corridor: Optional[ExecutionCorridor],
    dt: float,
) -> bool:
    if execution_corridor is None or not execution_corridor.active or not execution_corridor.constrains_side():
        return False
    prev_yaw: Optional[float] = None
    prev_raw: Optional[PoseLike] = None
    for raw in poses:
        _, _, yaw = _pose_xy_yaw(raw)
        seg_dt = _segment_dt(prev_raw, raw, dt) if prev_raw is not None else max(1e-3, float(dt))
        if prev_yaw is not None:
            dyaw = yaw - prev_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            omega = dyaw / seg_dt
            if not corridor_allows_omega_sign(execution_corridor, omega):
                return True
        prev_yaw = yaw
        prev_raw = raw
    return False


def braking_feasible(
    vx_mps: float,
    clearance_m: float,
    braking: BrakingModel,
    *,
    hard_stop_m: float,
) -> tuple[bool, str]:
    """Return (feasible, reason). Unknown decel → unavailable reason, not silent pass."""
    v = max(0.0, float(vx_mps))
    if v < 0.04:
        return True, REASON_NONE
    stop_dist = braking.stopping_distance_m(v)
    if stop_dist is None:
        # Conservative: require at least front_stop without inventing decel
        if clearance_m < hard_stop_m:
            return False, REASON_BRAKING_UNAVAILABLE
        return True, REASON_BRAKING_UNAVAILABLE
    required = stop_dist + hard_stop_m * 0.25
    if clearance_m < required:
        return False, REASON_BRAKING
    return True, REASON_NONE


def validate_trajectory(
    poses: Sequence[PoseLike],
    *,
    collide: Optional[CollideFn] = None,
    clearance_at: Optional[ClearanceFn] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
    margin_m: Optional[float] = None,
    dt: float = 0.1,
    enforce_limits: bool = True,
    execution_corridor: Optional[ExecutionCorridor] = None,
    braking: Optional[BrakingModel] = None,
) -> TrajectoryValidationResult:
    res = TrajectoryValidationResult(pose_count=len(poses))
    if not poses:
        res.reject(REASON_EMPTY)
        return res

    model = get_vehicle_model()
    limits = model.limits
    margin = float(geom.safety_margin_m if margin_m is None else margin_m)

    if collide is not None:
        res.collision = trajectory_collision(poses, collide, geom, margin_m=0.0)
        if res.collision.collision:
            res.reject(REASON_COLLISION)

    if clearance_at is not None:
        res.current_clearance = current_footprint_clearance(poses[0], clearance_at, geom, margin_m=0.0)
        res.minimum_clearance = trajectory_min_clearance(poses, clearance_at, geom, margin_m=0.0)
        min_cl = res.minimum_clearance.minimum_clearance_m
        if min_cl is not None and min_cl < margin:
            res.reject(REASON_CLEARANCE)

    max_abs_vx, max_abs_w, max_abs_ax = _infer_dynamics(poses, dt)
    res.max_abs_vx = max_abs_vx
    res.max_abs_w = max_abs_w
    res.max_abs_ax = max_abs_ax

    if execution_corridor is not None and _corridor_rejects_poses(poses, execution_corridor, dt):
        res.reject(REASON_CORRIDOR)

    if enforce_limits:
        if max_abs_vx > float(limits.max_vx_mps) + 1e-6:
            res.reject(REASON_VX_LIMIT)
        if max_abs_w > float(limits.max_omega_rad_s) + 1e-6:
            res.reject(REASON_W_LIMIT)
        if max_abs_ax > float(limits.max_accel_mps2) + 1e-6:
            res.reject(REASON_AX_LIMIT)

    if braking is not None and max_abs_vx > 0.04:
        min_cl = res.minimum_clearance.minimum_clearance_m
        clr = min_cl if min_cl is not None else (res.current_clearance.minimum_clearance_m or 99.0)
        ok, br_reason = braking_feasible(max_abs_vx, float(clr), braking, hard_stop_m=float(geom.front_stop_m))
        if not ok:
            res.reject(br_reason)

    if not res.valid and res.reason == REASON_NONE:
        res.reason = res.reasons[0] if res.reasons else REASON_UNKNOWN
    return res


def validate_dynamics(
    *,
    vx: float,
    omega: float,
    ax: Optional[float] = None,
    alpha: Optional[float] = None,
    longitudinal_jerk: Optional[float] = None,
    angular_jerk: Optional[float] = None,
    enforce: bool = True,
) -> TrajectoryValidationResult:
    """Command-level dynamics check. Collision-free is NOT sufficient."""
    from agv_bridge.nav_motion_dynamics import get_motion_limits, lateral_accel_vw, curvature

    res = TrajectoryValidationResult(source="DYNAMICS_VALIDATOR")
    lim = get_motion_limits()
    res.max_abs_vx = abs(float(vx))
    res.max_abs_w = abs(float(omega))
    ay = abs(lateral_accel_vw(vx, omega))
    kap = curvature(vx, omega)
    if not enforce:
        return res
    if abs(vx) > lim.max_vx_mps + 1e-6:
        res.reject(REASON_VX_LIMIT)
    if abs(omega) > lim.max_omega_rad_s + 1e-6:
        res.reject(REASON_W_LIMIT)
    if ax is not None and abs(ax) > lim.max_accel_mps2 + 1e-6:
        res.reject(REASON_AX_LIMIT)
    if alpha is not None and abs(alpha) > lim.max_alpha_rad_s2 + 1e-6:
        res.reject(REASON_ALPHA_LIMIT)
    if ay > lim.max_lateral_accel_mps2 + 1e-6:
        res.reject(REASON_AY_LIMIT)
    if longitudinal_jerk is not None and lim.max_longitudinal_jerk_mps3 is not None:
        if abs(longitudinal_jerk) > lim.max_longitudinal_jerk_mps3 + 1e-6:
            res.reject(REASON_JERK_LIMIT)
    if angular_jerk is not None and lim.max_angular_jerk_rps3 is not None:
        if abs(angular_jerk) > lim.max_angular_jerk_rps3 + 1e-6:
            res.reject(REASON_JERK_LIMIT)
    res.max_abs_ax = abs(ax or 0.0)
    if kap is not None:
        res.reasons  # keep
    if not res.valid and res.reason == REASON_NONE:
        res.reason = res.reasons[0] if res.reasons else REASON_UNKNOWN
    return res
