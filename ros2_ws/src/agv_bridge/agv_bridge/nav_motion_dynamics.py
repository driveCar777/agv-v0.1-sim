"""M3.9 motion state, curvature, jerk, oscillation / overshoot detectors.

Does not emit cmd_vel. Estimates that are not measured are tagged ESTIMATED.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple

from agv_bridge.nav_geometry import (
    CALIBRATION_REQUIRED,
    DERIVED,
    ENGINEERING_ASSUMPTION,
    get_vehicle_model,
)

TEMPORARY = "TEMPORARY"

EPS_V = 0.04
EPS_W = 0.02
PREVIEW_DS = (0.5, 1.0, 1.5, 2.0)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def curvature(vx: float, omega: float, *, eps: float = EPS_V) -> Optional[float]:
    """kappa = omega / v. None when |v| < eps (no division by zero)."""
    if abs(float(vx)) < eps:
        return None
    return float(omega) / float(vx)


def lateral_accel_vw(vx: float, omega: float) -> float:
    """a_y = v * omega."""
    return float(vx) * float(omega)


def lateral_accel_kappa(vx: float, kappa: Optional[float]) -> Optional[float]:
    """a_y = v^2 * kappa. None if kappa unknown."""
    if kappa is None:
        return None
    return float(vx) * float(vx) * float(kappa)


def curve_vmax(abs_kappa: float, ay_max: float, v_max: float, *, eps: float = 1e-4) -> float:
    """v <= sqrt(ay_max / |kappa|), capped at vehicle max_v."""
    k = abs(float(abs_kappa))
    if k < eps:
        return float(v_max)
    if ay_max <= 1e-9:
        return 0.0
    return min(float(v_max), math.sqrt(float(ay_max) / k))


def path_heading_and_kappa(
    path: List[Tuple[float, float]],
    x: float,
    y: float,
    preview_m: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Heading at nearest point and curvature ~preview_m ahead. ESTIMATED."""
    if not path or len(path) < 2:
        return None, None
    best_i, best_d = 0, 1e18
    for i, p in enumerate(path):
        d = math.hypot(p[0] - x, p[1] - y)
        if d < best_d:
            best_d, best_i = d, i
    acc = 0.0
    j = best_i
    while j + 1 < len(path) and acc < preview_m:
        acc += math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
        j += 1
    hdg = None
    if best_i + 1 < len(path):
        hdg = math.atan2(path[best_i + 1][1] - path[best_i][1], path[best_i + 1][0] - path[best_i][0])
    kappa = polyline_kappa_at(path, j)
    return hdg, kappa


def polyline_kappa_at(path: List[Tuple[float, float]], i: int) -> Optional[float]:
    if i <= 0 or i >= len(path) - 1:
        return None
    x0, y0 = path[i - 1]
    x1, y1 = path[i]
    x2, y2 = path[i + 1]
    d01 = math.hypot(x1 - x0, y1 - y0)
    d12 = math.hypot(x2 - x1, y2 - y1)
    if d01 < 1e-4 or d12 < 1e-4:
        return None
    h0 = math.atan2(y1 - y0, x1 - x0)
    h1 = math.atan2(y2 - y1, x2 - x1)
    dth = wrap_pi(h1 - h0)
    ds = 0.5 * (d01 + d12)
    if ds < 1e-4:
        return None
    return dth / ds


def preview_max_abs_kappa(
    path: List[Tuple[float, float]],
    x: float,
    y: float,
    distances: Tuple[float, ...] = PREVIEW_DS,
) -> Tuple[float, Dict[str, Optional[float]]]:
    samples: Dict[str, Optional[float]] = {}
    peak = 0.0
    for d in distances:
        _, k = path_heading_and_kappa(path, x, y, d)
        samples[f"kappa_{d:.1f}m"] = None if k is None else round(float(k), 4)
        if k is not None:
            peak = max(peak, abs(float(k)))
    return peak, samples


@dataclass
class MotionLimits:
    max_vx_mps: float
    max_accel_mps2: float
    max_decel_mps2: Optional[float]
    max_omega_rad_s: float
    max_alpha_rad_s2: float
    max_lateral_accel_mps2: float
    max_longitudinal_jerk_mps3: Optional[float]
    max_angular_jerk_rps3: Optional[float]
    sources: Dict[str, str] = field(default_factory=dict)

    def effective_decel(self) -> Tuple[float, str]:
        if self.max_decel_mps2 is not None and self.max_decel_mps2 > 1e-6:
            return float(self.max_decel_mps2), self.sources.get("max_decel_mps2", ENGINEERING_ASSUMPTION)
        return float(self.max_accel_mps2), TEMPORARY + "|DECEL_EQ_ACCEL"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "max_vx_mps": self.max_vx_mps,
            "max_accel_mps2": self.max_accel_mps2,
            "max_decel_mps2": self.max_decel_mps2,
            "max_omega_rad_s": self.max_omega_rad_s,
            "max_alpha_rad_s2": self.max_alpha_rad_s2,
            "max_lateral_accel_mps2": self.max_lateral_accel_mps2,
            "max_longitudinal_jerk_mps3": self.max_longitudinal_jerk_mps3,
            "max_angular_jerk_rps3": self.max_angular_jerk_rps3,
            "sources": dict(self.sources),
        }


def get_motion_limits() -> MotionLimits:
    """Central limits. Unknown real-vehicle values stay CALIBRATION_REQUIRED."""
    model = get_vehicle_model()
    lim = model.limits
    geom = model.geometry
    # DERIVED kinematic product: max |ay| at hardware vx*omega (not a comfort guess).
    ay_derived = abs(float(lim.max_vx_mps) * float(lim.max_omega_rad_s))
    return MotionLimits(
        max_vx_mps=float(lim.max_vx_mps),
        max_accel_mps2=float(lim.max_accel_mps2),
        max_decel_mps2=lim.max_decel_mps2,
        max_omega_rad_s=float(lim.max_omega_rad_s),
        max_alpha_rad_s2=float(lim.max_alpha_rad_s2),
        max_lateral_accel_mps2=ay_derived,
        max_longitudinal_jerk_mps3=None,
        max_angular_jerk_rps3=None,
        sources={
            "max_vx_mps": ENGINEERING_ASSUMPTION,
            "max_accel_mps2": ENGINEERING_ASSUMPTION,
            "max_decel_mps2": CALIBRATION_REQUIRED,
            "max_omega_rad_s": ENGINEERING_ASSUMPTION,
            "max_alpha_rad_s2": ENGINEERING_ASSUMPTION,
            "max_lateral_accel_mps2": DERIVED + "|max_vx*max_omega",
            "max_longitudinal_jerk_mps3": CALIBRATION_REQUIRED,
            "max_angular_jerk_rps3": CALIBRATION_REQUIRED,
            "acc_v_physics": ENGINEERING_ASSUMPTION,
            "acc_w_physics": ENGINEERING_ASSUMPTION,
            "geom_acc_v": str(geom.acc_v),
            "geom_acc_w": str(geom.acc_w),
        },
    )


def _sign(v: float, eps: float) -> int:
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


class OscillationDetector:
    """Omega / heading / cross-track sign chatter. Does not change command."""

    def __init__(self, window_s: float = 4.0, flip_limit: int = 6) -> None:
        self.window_s = window_s
        self.flip_limit = flip_limit
        self._w_flips: Deque[float] = deque()
        self._cte_flips: Deque[float] = deque()
        self._last_w = 0
        self._last_cte = 0

    def reset(self) -> None:
        self._w_flips.clear()
        self._cte_flips.clear()
        self._last_w = 0
        self._last_cte = 0

    def update(self, now: float, omega: float, cross_track: Optional[float] = None) -> Dict[str, Any]:
        ws = _sign(omega, EPS_W)
        if ws and self._last_w and ws != self._last_w:
            self._w_flips.append(now)
        if ws:
            self._last_w = ws
        while self._w_flips and now - self._w_flips[0] > self.window_s:
            self._w_flips.popleft()
        cte_n = 0
        if cross_track is not None:
            cs = _sign(float(cross_track), 0.04)
            if cs and self._last_cte and cs != self._last_cte:
                self._cte_flips.append(now)
            if cs:
                self._last_cte = cs
            while self._cte_flips and now - self._cte_flips[0] > self.window_s:
                self._cte_flips.popleft()
            cte_n = len(self._cte_flips)
        n = len(self._w_flips)
        rate = n / max(self.window_s, 1e-3)
        chatter = n >= self.flip_limit
        return {
            "omega_sign_changes": n,
            "omega_sign_changes_per_second": round(rate, 3),
            "cross_track_sign_changes": cte_n,
            "control_chatter": chatter,
            "oscillation_score": n,
        }


class OvershootDetector:
    def __init__(self, mag_rad: float = 0.25) -> None:
        self.mag_rad = mag_rad
        self._last_e: Optional[float] = None
        self._last_overshoot = False
        self._peak = 0.0

    def reset(self) -> None:
        self._last_e = None
        self._last_overshoot = False
        self._peak = 0.0

    def update(self, heading_error: float) -> Dict[str, Any]:
        e = wrap_pi(float(heading_error))
        over = False
        if self._last_e is not None:
            if _sign(self._last_e, 0.03) and _sign(e, 0.03) and _sign(self._last_e, 0.03) != _sign(e, 0.03):
                if abs(e) >= self.mag_rad and abs(self._last_e) >= self.mag_rad:
                    over = True
        self._last_e = e
        self._last_overshoot = over
        self._peak = max(self._peak, abs(e))
        return {
            "heading_error": round(e, 4),
            "heading_overshoot": over,
            "heading_error_peak_abs": round(self._peak, 4),
        }


def limit_cycle_suspected(*, progress_m: float, omega_sign_changes: int, window_s: float = 4.0) -> bool:
    return abs(float(progress_m)) < 0.15 and int(omega_sign_changes) >= 4 and window_s > 0.5


def classify_motion_health(
    *,
    emergency: bool = False,
    safe_stop: bool = False,
    chatter: bool = False,
    overshoot: bool = False,
    stale_planner: bool = False,
    dynamics_infeasible: bool = False,
    jerk_limited: bool = False,
    accel_limited: bool = False,
    curvature_limited: bool = False,
    oscillating: bool = False,
) -> str:
    if emergency or safe_stop:
        return "SAFE_STOP"
    if dynamics_infeasible:
        return "DYNAMICS_INFEASIBLE"
    if stale_planner:
        return "STALE_PLANNER"
    if oscillating or chatter:
        return "OSCILLATING"
    if overshoot:
        return "OVERSHOOT"
    if jerk_limited:
        return "JERK_LIMITED"
    if accel_limited:
        return "ACCELERATION_LIMITED"
    if curvature_limited:
        return "CURVATURE_LIMITED"
    return "NORMAL"


def derive_rates(
    *,
    vx: float,
    omega: float,
    prev_vx: Optional[float],
    prev_omega: Optional[float],
    prev_ax: Optional[float],
    prev_alpha: Optional[float],
    dt: float,
) -> Dict[str, Any]:
    """Finite-difference ax/alpha/jerk. ESTIMATED."""
    dt = max(1e-3, float(dt))
    ax = None if prev_vx is None else (float(vx) - float(prev_vx)) / dt
    alpha = None if prev_omega is None else (float(omega) - float(prev_omega)) / dt
    jx = None if ax is None or prev_ax is None else (float(ax) - float(prev_ax)) / dt
    jw = None if alpha is None or prev_alpha is None else (float(alpha) - float(prev_alpha)) / dt
    kap = curvature(vx, omega)
    ay = lateral_accel_vw(vx, omega)
    ay_k = lateral_accel_kappa(vx, kap)
    return {
        "vx": float(vx),
        "omega": float(omega),
        "ax": ax,
        "alpha": alpha,
        "longitudinal_jerk": jx,
        "angular_jerk": jw,
        "curvature": kap,
        "lateral_acceleration": ay,
        "lateral_acceleration_from_kappa": ay_k,
        "quality": {
            "vx": "MEASURED",
            "omega": "MEASURED",
            "ax": "ESTIMATED" if ax is not None else "UNAVAILABLE",
            "alpha": "ESTIMATED" if alpha is not None else "UNAVAILABLE",
            "longitudinal_jerk": "ESTIMATED" if jx is not None else "UNAVAILABLE",
            "angular_jerk": "ESTIMATED" if jw is not None else "UNAVAILABLE",
            "curvature": "DERIVED" if kap is not None else "UNAVAILABLE",
            "lateral_acceleration": "DERIVED",
        },
    }
