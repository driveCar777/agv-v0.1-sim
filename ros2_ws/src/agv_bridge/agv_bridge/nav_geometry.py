"""统一车辆几何 / 碰撞 / 安全阈值（Web 仿真）。

P0-A: VehicleGeometry is the ONLY size truth source.
  - footprint polygon (nav_footprint) = narrow-phase TRUTH
  - planner_radius / local_radius / safety_radius = BROAD-PHASE approximation only

差异必须来自此配置，禁止再散落魔法数字。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class VehicleGeometry:
    """AMB-150 近似（复用既有尺寸，不另起第二套常量）。

    Body frame: +x front, +y left (see PHASE4_NAVIGATION_KINEMATIC_CONTRACT.md).

    Broad-phase (approximation — early reject only):
      planner_radius, local_radius, safety_radius

    Narrow-phase truth:
      length / width / bumper_l → footprint polygon via nav_footprint
    """

    length: float = 1.05
    width: float = 0.55
    bumper_l: float = 0.55  # MPPI bumper SAMPLE offset (center→check-point). NOT polygon half-length.
    # Narrow-phase rectangle uses 0.5*length = 0.525 m (P0-A clamps bumper_l to half-length).
    # bumper_l remains 0.55 because MPPI bumper_pose samples 2.5 cm proud of the body — do not retune.

    # Broad-phase bounding circles (NOT final collision truth)
    planner_radius: float = 0.25
    local_radius: float = 0.24
    safety_radius: float = 0.28
    # Optional physical params — do NOT invent if unknown
    center_offset_x_m: float = 0.0
    safety_margin_m: float = 0.08
    track_width_m: Optional[float] = None  # UNAVAILABLE
    wheelbase_m: Optional[float] = None  # UNAVAILABLE
    # Safety Supervisor（最终裁决阈值 — 行为层；P0-A 不改数值）
    front_stop_m: float = 0.70
    front_clear_m: float = 1.20
    rear_stop_m: float = 0.55
    # MPPI 仅作代价软提示，不最终停车
    front_cost_m: float = 0.90
    # 速度限制
    max_vx: float = 0.40
    max_w: float = 0.45
    acc_v: float = 0.9
    acc_w: float = 0.55

    # --- aliases for contract / telemetry (no second size table) ---
    @property
    def length_m(self) -> float:
        return float(self.length)

    @property
    def width_m(self) -> float:
        return float(self.width)

    @property
    def front_overhang_m(self) -> float:
        return float(self.bumper_l)

    @property
    def rear_overhang_m(self) -> float:
        return float(self.bumper_l)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["length_m"] = self.length_m
        d["width_m"] = self.width_m
        d["front_overhang_m"] = self.front_overhang_m
        d["rear_overhang_m"] = self.rear_overhang_m
        d["footprint_model"] = "polygon"
        d["collision_model_broad_phase"] = "bounding_radius"
        d["collision_model_narrow_phase"] = "footprint_polygon_samples"
        d["track_width_unavailable"] = self.track_width_m is None
        d["wheelbase_unavailable"] = self.wheelbase_m is None
        d["radius_role"] = "approximation_broad_phase_only"
        return d


DEFAULT_GEOM = VehicleGeometry()


def get_vehicle_geometry() -> VehicleGeometry:
    """Canonical accessor — same instance as DEFAULT_GEOM."""
    return DEFAULT_GEOM


# Stuck / Recovery（dt=0.05、局部 0.35s、巡航~0.15m/s）
# 8s 内沿路径至少前进 0.30m；否则视为无进展
STUCK_NO_PROGRESS_S = 8.0
STUCK_PROGRESS_MIN_M = 0.30
STUCK_REVERSE_TRIGGER_S = 8.0
REVERSE_MAX_S = 4.0
RECOVERY_COOLDOWN_S = 8.0
MAX_RECOVERY_ATTEMPTS = 3
GLOBAL_SPLICE_S = 25.0
GLOBAL_ALT_S = 45.0
