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
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_model

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
REASON_UNKNOWN = "UNKNOWN"


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


def validate_trajectory(
    poses: Sequence[PoseLike],
    *,
    collide: Optional[CollideFn] = None,
    clearance_at: Optional[ClearanceFn] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
    margin_m: Optional[float] = None,
    dt: float = 0.1,
    enforce_limits: bool = True,
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

    if enforce_limits:
        if max_abs_vx > float(limits.max_vx_mps) + 1e-6:
            res.reject(REASON_VX_LIMIT)
        if max_abs_w > float(limits.max_omega_rad_s) + 1e-6:
            res.reject(REASON_W_LIMIT)
        if max_abs_ax > float(limits.max_accel_mps2) + 1e-6:
            res.reject(REASON_AX_LIMIT)

    if not res.valid and res.reason == REASON_NONE:
        res.reason = res.reasons[0] if res.reasons else REASON_UNKNOWN
    return res
