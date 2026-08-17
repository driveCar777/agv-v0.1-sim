"""Unified vehicle model / geometry / safety thresholds.

V0.2 M1 keeps the existing ``VehicleGeometry`` import surface stable while
promoting a richer ``VehicleModel`` as the canonical physics source.

Rules:
  - footprint polygon (``nav_footprint``) = narrow-phase truth
  - planner/local/safety radius = broad-phase approximation only
  - limits / offsets / braking assumptions come from one model instance
  - unknown real-vehicle values stay explicit as ``None`` / calibration flags
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional

CALIBRATION_REQUIRED = "CALIBRATION_REQUIRED"
SPEC = "SPEC"
DERIVED = "DERIVED"
ENGINEERING_ASSUMPTION = "ENGINEERING_ASSUMPTION"


@dataclass(frozen=True)
class VehicleLimits:
    max_vx_mps: float = 0.40
    min_vx_mps: float = -0.18
    max_accel_mps2: float = 0.9
    max_decel_mps2: Optional[float] = None
    max_omega_rad_s: float = 0.45
    max_alpha_rad_s2: float = 0.55
    max_decel_status: str = CALIBRATION_REQUIRED

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BrakingModel:
    reaction_latency_s: Optional[float] = None
    controller_latency_s: Optional[float] = None
    command_latency_s: Optional[float] = None
    max_decel_mps2: Optional[float] = None
    safety_margin_m: float = 0.08
    calibration_status: str = CALIBRATION_REQUIRED

    def stopping_distance_m(self, vx_mps: float) -> Optional[float]:
        v = max(0.0, abs(float(vx_mps)))
        decel = self.max_decel_mps2
        if decel is None or decel <= 1e-6:
            return None
        reaction = sum(
            float(v or 0.0)
            for v in (self.reaction_latency_s, self.controller_latency_s, self.command_latency_s)
        )
        return v * reaction + (v * v) / (2.0 * decel) + float(self.safety_margin_m)


@dataclass(frozen=True)
class VehicleGeometry:
    """Geometry-only compatibility view backed by the unified vehicle model.

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
    center_offset_y_m: float = 0.0
    safety_margin_m: float = 0.08
    track_width_m: Optional[float] = None  # UNAVAILABLE
    wheelbase_m: Optional[float] = None  # UNAVAILABLE
    base_link_offset_x_m: float = 0.0
    base_link_offset_y_m: float = 0.0
    # Safety Supervisor（最终裁决阈值 — 行为层；P0-A 不改数值）
    front_stop_m: float = 0.70
    front_clear_m: float = 1.20
    rear_stop_m: float = 0.55
    # MPPI 仅作代价软提示，不最终停车
    front_cost_m: float = 0.90
    # 速度限制
    max_vx: float = 0.40
    min_vx: float = -0.18
    max_w: float = 0.45
    acc_v: float = 0.9
    acc_w: float = 0.55
    max_decel_mps2: Optional[float] = None
    command_latency_s: Optional[float] = None
    controller_latency_s: Optional[float] = None
    perception_latency_s: Optional[float] = None
    calibration_status: str = CALIBRATION_REQUIRED
    parameter_sources: Dict[str, str] = field(
        default_factory=lambda: {
            "length_m": SPEC,
            "width_m": SPEC,
            "front_overhang_m": DERIVED,
            "rear_overhang_m": DERIVED,
            "max_vx_mps": ENGINEERING_ASSUMPTION,
            "max_accel_mps2": ENGINEERING_ASSUMPTION,
            "max_decel_mps2": CALIBRATION_REQUIRED,
            "max_omega_rad_s": ENGINEERING_ASSUMPTION,
            "max_alpha_rad_s2": ENGINEERING_ASSUMPTION,
            "track_width_m": CALIBRATION_REQUIRED,
            "wheelbase_m": CALIBRATION_REQUIRED,
            "command_latency_s": CALIBRATION_REQUIRED,
            "controller_latency_s": CALIBRATION_REQUIRED,
            "perception_latency_s": CALIBRATION_REQUIRED,
        }
    )

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

    @property
    def footprint_polygon_body(self) -> list[tuple[float, float]]:
        half_l = 0.5 * float(self.length)
        half_w = 0.5 * float(self.width)
        xf = min(float(self.front_overhang_m), half_l)
        xr = max(-float(self.rear_overhang_m), -half_l)
        ox = float(self.base_link_offset_x_m)
        oy = float(self.base_link_offset_y_m)
        return [
            (ox + xf, oy + half_w),
            (ox + xf, oy - half_w),
            (ox + xr, oy - half_w),
            (ox + xr, oy + half_w),
        ]

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
        d["min_vx"] = self.min_vx
        d["max_decel_mps2"] = self.max_decel_mps2
        d["command_latency_s"] = self.command_latency_s
        d["controller_latency_s"] = self.controller_latency_s
        d["perception_latency_s"] = self.perception_latency_s
        d["base_link_offset_x_m"] = self.base_link_offset_x_m
        d["base_link_offset_y_m"] = self.base_link_offset_y_m
        d["footprint_polygon_body"] = [
            {"x": round(p[0], 3), "y": round(p[1], 3)} for p in self.footprint_polygon_body
        ]
        d["parameter_sources"] = dict(self.parameter_sources)
        d["calibration_status"] = self.calibration_status
        return d


@dataclass(frozen=True)
class VehicleModel:
    geometry: VehicleGeometry = field(default_factory=VehicleGeometry)
    limits: VehicleLimits = field(default_factory=VehicleLimits)
    braking: BrakingModel = field(default_factory=BrakingModel)
    model_name: str = "AGV_V0_2_BASELINE"
    note: str = "Unified navigation vehicle model; unknown real-world values remain explicit."

    def to_dict(self) -> Dict[str, Any]:
        return {
            "model_name": self.model_name,
            "geometry": self.geometry.to_dict(),
            "limits": self.limits.to_dict(),
            "braking": {
                **asdict(self.braking),
                "stopping_distance_at_0_3_mps": self.braking.stopping_distance_m(0.3),
            },
            "note": self.note,
        }


DEFAULT_GEOM = VehicleGeometry(
    max_vx=0.40,
    min_vx=-0.18,
    max_w=0.45,
    acc_v=0.9,
    acc_w=0.55,
)
DEFAULT_LIMITS = VehicleLimits(
    max_vx_mps=DEFAULT_GEOM.max_vx,
    min_vx_mps=DEFAULT_GEOM.min_vx,
    max_accel_mps2=DEFAULT_GEOM.acc_v,
    max_decel_mps2=DEFAULT_GEOM.max_decel_mps2,
    max_omega_rad_s=DEFAULT_GEOM.max_w,
    max_alpha_rad_s2=DEFAULT_GEOM.acc_w,
)
DEFAULT_BRAKING = BrakingModel(
    reaction_latency_s=DEFAULT_GEOM.perception_latency_s,
    controller_latency_s=DEFAULT_GEOM.controller_latency_s,
    command_latency_s=DEFAULT_GEOM.command_latency_s,
    max_decel_mps2=DEFAULT_GEOM.max_decel_mps2,
    safety_margin_m=DEFAULT_GEOM.safety_margin_m,
)
DEFAULT_VEHICLE = VehicleModel(
    geometry=DEFAULT_GEOM,
    limits=DEFAULT_LIMITS,
    braking=DEFAULT_BRAKING,
)


def get_vehicle_geometry() -> VehicleGeometry:
    """Compatibility accessor for geometry consumers."""
    return DEFAULT_VEHICLE.geometry


def get_vehicle_model() -> VehicleModel:
    """Canonical V0.2 accessor for all navigation modules."""
    return DEFAULT_VEHICLE


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
