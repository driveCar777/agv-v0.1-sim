"""V0.2 M1 execution corridor.

Transforms probe / commitment / recovery output into planner-executable
constraints. The corridor is a contract, not just a UI hint.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry

MODE_INACTIVE = "INACTIVE"
MODE_FORWARD = "FORWARD"
MODE_LEFT = "LEFT"
MODE_RIGHT = "RIGHT"
MODE_WAIT = "WAIT"


@dataclass
class CorridorBounds:
    left_bound_m: Optional[float] = None
    right_bound_m: Optional[float] = None
    center_offset_m: float = 0.0
    heading_target_rad: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "left_bound_m": self.left_bound_m,
            "right_bound_m": self.right_bound_m,
            "center_offset_m": round(self.center_offset_m, 3),
            "heading_target_rad": self.heading_target_rad,
        }


@dataclass
class ExecutionCorridor:
    active: bool = False
    mode: str = MODE_INACTIVE
    committed_side: Optional[str] = None
    preferred_region: str = "CENTER"
    minimum_clearance_m: float = 0.0
    max_vx_mps: float = 0.0
    max_omega_rad_s: float = 0.0
    source: str = "NONE"
    reason: str = "NONE"
    timestamp_s: float = 0.0
    bounds: CorridorBounds = field(default_factory=CorridorBounds)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def constrains_side(self) -> bool:
        return self.mode in (MODE_LEFT, MODE_RIGHT) and self.committed_side in ("LEFT", "RIGHT")

    def allows_forward(self) -> bool:
        return self.active and self.mode in (MODE_FORWARD, MODE_LEFT, MODE_RIGHT)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "active": self.active,
            "mode": self.mode,
            "committed_side": self.committed_side,
            "preferred_region": self.preferred_region,
            "minimum_clearance_m": round(self.minimum_clearance_m, 3),
            "max_vx_mps": round(self.max_vx_mps, 3),
            "max_omega_rad_s": round(self.max_omega_rad_s, 3),
            "source": self.source,
            "reason": self.reason,
            "timestamp_s": round(self.timestamp_s, 3),
            "bounds": self.bounds.to_dict(),
            "metadata": dict(self.metadata),
        }


def build_execution_corridor(
    *,
    now: float,
    avoidance_phase: str,
    committed_side: Optional[str],
    preferred_side: Optional[str],
    commit_ready: bool,
    front_near: float,
    left_free: float,
    right_free: float,
    probe_confidence_left: float = 0.0,
    probe_confidence_right: float = 0.0,
    geom: VehicleGeometry = DEFAULT_GEOM,
    source: str = "BEHAVIOR",
    reason: str = "NONE",
) -> ExecutionCorridor:
    corridor = ExecutionCorridor(
        active=False,
        mode=MODE_INACTIVE,
        committed_side=committed_side,
        minimum_clearance_m=float(getattr(geom, "safety_margin_m", 0.08) or 0.08),
        max_vx_mps=float(geom.max_vx),
        max_omega_rad_s=float(geom.max_w),
        source=source,
        reason=reason,
        timestamp_s=float(now),
    )
    phase = str(avoidance_phase or "").upper()
    side = committed_side or preferred_side
    side_commit_active = phase == "SIDE_COMMIT" or (
        phase == "LOCAL_AVOID" and side in ("LEFT", "RIGHT")
    )
    if side_commit_active and side in ("LEFT", "RIGHT") and (commit_ready or phase == "SIDE_COMMIT"):
        corridor.active = True
        corridor.mode = MODE_LEFT if side == "LEFT" else MODE_RIGHT
        corridor.committed_side = side
        corridor.preferred_region = f"{side}_CORRIDOR"
        corridor.bounds.center_offset_m = 0.18 if side == "LEFT" else -0.18
        corridor.bounds.left_bound_m = max(0.18, float(left_free) - 0.15)
        corridor.bounds.right_bound_m = max(0.18, float(right_free) - 0.15)
        corridor.max_vx_mps = min(float(geom.max_vx) * 0.75, 0.25 if front_near < 2.5 else 0.30)
        corridor.max_omega_rad_s = min(float(geom.max_w), 0.42)
        corridor.metadata = {
            "probe_confidence_left": round(float(probe_confidence_left), 3),
            "probe_confidence_right": round(float(probe_confidence_right), 3),
            "avoidance_phase": phase,
        }
        return corridor
    if phase in ("SIDE_PROBE", "OBSTACLE_APPROACH", "FUTURE_PREVIEW"):
        corridor.active = True
        corridor.mode = MODE_FORWARD
        corridor.preferred_region = "CENTER"
        corridor.max_vx_mps = min(float(geom.max_vx), 0.28)
        corridor.max_omega_rad_s = min(float(geom.max_w), 0.35)
        corridor.metadata = {"avoidance_phase": phase, "preferred_side_hint": preferred_side}
        return corridor
    if phase == "SAFE_STOP":
        corridor.active = True
        corridor.mode = MODE_WAIT
        corridor.max_vx_mps = 0.0
        corridor.max_omega_rad_s = 0.0
        corridor.metadata = {"avoidance_phase": phase}
        return corridor
    corridor.active = True
    corridor.mode = MODE_FORWARD
    corridor.preferred_region = "CENTER"
    corridor.max_vx_mps = float(geom.max_vx)
    corridor.max_omega_rad_s = float(geom.max_w)
    corridor.metadata = {"avoidance_phase": phase}
    return corridor


def corridor_allows_omega_sign(corridor: Optional[ExecutionCorridor], omega: float) -> bool:
    if corridor is None or not corridor.constrains_side():
        return True
    if corridor.mode == MODE_LEFT:
        return omega >= -1e-6
    if corridor.mode == MODE_RIGHT:
        return omega <= 1e-6
    return True
