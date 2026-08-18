"""M3.9 DynamicsLimiter — physical rate / curvature / ay / jerk gates on command.

requested → limiter → limited → Safety Supervisor → approved.

Does not flip omega sign. Emergency only hardware-clamps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_motion_dynamics import (
    EPS_V,
    MotionLimits,
    classify_motion_health,
    curve_vmax,
    get_motion_limits,
    preview_max_abs_kappa,
)

Pt = Tuple[float, float]


@dataclass
class LimitedCommand:
    vx: float
    omega: float
    reasons: List[str] = field(default_factory=list)
    speed_limit_reason: str = "NORMAL"
    curve_vmax: Optional[float] = None
    future_max_abs_kappa: float = 0.0
    kappa_preview: Dict[str, Optional[float]] = field(default_factory=dict)
    ax: float = 0.0
    alpha: float = 0.0
    ay: float = 0.0
    curvature: Optional[float] = None
    longitudinal_jerk: Optional[float] = None
    angular_jerk: Optional[float] = None
    accel_limited: bool = False
    jerk_limited: bool = False
    curvature_limited: bool = False
    chatter_blocked: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "limited_vx": round(self.vx, 4),
            "limited_omega": round(self.omega, 4),
            "reasons": list(self.reasons),
            "speed_limit_reason": self.speed_limit_reason,
            "curve_vmax": None if self.curve_vmax is None else round(self.curve_vmax, 4),
            "future_max_abs_kappa": round(self.future_max_abs_kappa, 4),
            "kappa_preview": dict(self.kappa_preview),
            "ax": round(self.ax, 4),
            "alpha": round(self.alpha, 4),
            "ay": round(self.ay, 4),
            "curvature": None if self.curvature is None else round(self.curvature, 4),
            "longitudinal_jerk": None if self.longitudinal_jerk is None else round(self.longitudinal_jerk, 4),
            "angular_jerk": None if self.angular_jerk is None else round(self.angular_jerk, 4),
            "accel_limited": self.accel_limited,
            "jerk_limited": self.jerk_limited,
            "curvature_limited": self.curvature_limited,
        }


class DynamicsLimiter:
    """Stateful first-order + jerk gate. Physics still applies acc_v/acc_w to plant."""

    def __init__(self, limits: Optional[MotionLimits] = None) -> None:
        self.limits = limits or get_motion_limits()
        self.vx = 0.0
        self.omega = 0.0
        self.ax = 0.0
        self.alpha = 0.0
        self.last: Optional[LimitedCommand] = None

    def reset(self) -> None:
        self.vx = self.omega = self.ax = self.alpha = 0.0
        self.last = None

    def limit(
        self,
        requested_vx: float,
        requested_omega: float,
        *,
        dt: float,
        state_vx: Optional[float] = None,
        state_omega: Optional[float] = None,
        path: Optional[Sequence[Any]] = None,
        x: float = 0.0,
        y: float = 0.0,
        emergency: bool = False,
        maneuver_mode: str = "",
        policy_speed: Optional[float] = None,
        obstacle_vmax: Optional[float] = None,
    ) -> LimitedCommand:
        dt = max(1e-3, float(dt))
        lim = self.limits
        req_v = float(requested_vx)
        req_w = float(requested_omega)
        reasons: List[str] = []
        speed_reason = "NORMAL"
        mm = str(maneuver_mode or "").upper()

        # Seed from plant if first cycle
        if self.last is None:
            if state_vx is not None:
                self.vx = float(state_vx)
            if state_omega is not None:
                self.omega = float(state_omega)

        v_hw = float(lim.max_vx_mps)
        w_hw = float(lim.max_omega_rad_s)
        if emergency or mm in ("EMERGENCY", "SAFE_STOP"):
            v = max(-v_hw, min(v_hw, req_v))
            w = max(-w_hw, min(w_hw, req_w))
            out = LimitedCommand(vx=v, omega=w, reasons=["EMERGENCY_HARDWARE_CLAMP"], speed_limit_reason="SAFETY")
            self.vx, self.omega = v, w
            self.last = out
            return out

        pts = _as_xy(path)
        future_k, preview = preview_max_abs_kappa(pts, x, y) if pts else (0.0, {})
        v_curve = curve_vmax(future_k, lim.max_lateral_accel_mps2, v_hw)
        v_cap = v_hw
        if policy_speed is not None:
            v_cap = min(v_cap, max(0.0, float(policy_speed)))
        if obstacle_vmax is not None:
            v_cap = min(v_cap, max(0.0, float(obstacle_vmax)))
            if float(obstacle_vmax) + 1e-6 < v_hw:
                speed_reason = "OBSTACLE"
                reasons.append("OBSTACLE_VMAX")
        curvature_limited = False
        if future_k > 1e-4 and v_curve + 1e-6 < v_cap:
            v_cap = min(v_cap, v_curve)
            speed_reason = "CURVATURE"
            curvature_limited = True
            reasons.append("CURVATURE_PREVIEW")

        req_v = max(-v_hw, min(v_hw, req_v))
        if req_v > v_cap:
            req_v = v_cap
            if speed_reason == "NORMAL":
                speed_reason = "VEHICLE"
            reasons.append("VX_CAP")

        req_w = max(-w_hw, min(w_hw, req_w))

        # Lateral accel: prefer reducing |v| so turning capability remains
        ay_req = abs(req_v * req_w)
        if ay_req > lim.max_lateral_accel_mps2 + 1e-6 and abs(req_w) > EPS_V:
            v_ay = lim.max_lateral_accel_mps2 / abs(req_w)
            if abs(req_v) > v_ay + 1e-6:
                req_v = math.copysign(v_ay, req_v) if req_v != 0 else v_ay
                reasons.append("LATERAL_ACCEL")
                curvature_limited = True
                if speed_reason == "NORMAL":
                    speed_reason = "CURVATURE"

        decel, decel_src = lim.effective_decel()
        accel = float(lim.max_accel_mps2)
        if req_v >= self.vx:
            max_dv = accel * dt
        else:
            max_dv = decel * dt
            if "TEMPORARY" in decel_src:
                reasons.append("DECEL_UNCALIBRATED")
        dv = max(-max_dv, min(max_dv, req_v - self.vx))
        accel_limited = abs(req_v - self.vx) > max_dv + 1e-6
        if accel_limited:
            reasons.append("ACCEL")

        ax_cmd = dv / dt
        jx_lim = lim.max_longitudinal_jerk_mps3
        jerk_limited = False
        if jx_lim is not None and jx_lim > 1e-6:
            max_dax = jx_lim * dt
            dax = max(-max_dax, min(max_dax, ax_cmd - self.ax))
            if abs(ax_cmd - self.ax) > max_dax + 1e-6:
                jerk_limited = True
                reasons.append("LONGITUDINAL_JERK")
            ax_cmd = self.ax + dax
            dv = ax_cmd * dt

        v_next = self.vx + dv
        v_next = max(-v_hw, min(v_hw, v_next))

        alpha_max = float(lim.max_alpha_rad_s2)
        max_dw = alpha_max * dt
        dw = max(-max_dw, min(max_dw, req_w - self.omega))
        if abs(req_w - self.omega) > max_dw + 1e-6:
            accel_limited = True
            reasons.append("ALPHA")
        alpha_cmd = dw / dt
        jw_lim = lim.max_angular_jerk_rps3
        if jw_lim is not None and jw_lim > 1e-6:
            max_dalpha = jw_lim * dt
            da = max(-max_dalpha, min(max_dalpha, alpha_cmd - self.alpha))
            if abs(alpha_cmd - self.alpha) > max_dalpha + 1e-6:
                jerk_limited = True
                reasons.append("ANGULAR_JERK")
            alpha_cmd = self.alpha + da
            dw = alpha_cmd * dt
        w_next = self.omega + dw
        w_next = max(-w_hw, min(w_hw, w_next))

        # Do not invert sign in one step beyond ramp-through-zero
        if self.omega > EPS_V and w_next < -1e-6 and req_w < 0:
            # allowed: ramp through 0 within this dt
            pass
        ay = v_next * w_next
        kap = None if abs(v_next) < EPS_V else (w_next / v_next)

        out = LimitedCommand(
            vx=v_next,
            omega=w_next,
            reasons=reasons or ["NONE"],
            speed_limit_reason=speed_reason,
            curve_vmax=v_curve,
            future_max_abs_kappa=future_k,
            kappa_preview=preview,
            ax=ax_cmd,
            alpha=alpha_cmd,
            ay=ay,
            curvature=kap,
            longitudinal_jerk=None if dt <= 0 else (ax_cmd - self.ax) / dt,
            angular_jerk=None if dt <= 0 else (alpha_cmd - self.alpha) / dt,
            accel_limited=accel_limited,
            jerk_limited=jerk_limited,
            curvature_limited=curvature_limited,
        )
        self.vx, self.omega = v_next, w_next
        self.ax, self.alpha = ax_cmd, alpha_cmd
        self.last = out
        return out


def _as_xy(path: Optional[Sequence[Any]]) -> List[Pt]:
    out: List[Pt] = []
    if not path:
        return out
    for p in path:
        if isinstance(p, dict):
            out.append((float(p.get("x") or 0), float(p.get("y") or 0)))
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            out.append((float(p[0]), float(p[1])))
    return out


def motion_health_from_limited(
    lim: LimitedCommand,
    *,
    chatter: bool = False,
    overshoot: bool = False,
    stale: bool = False,
    emergency: bool = False,
) -> str:
    return classify_motion_health(
        emergency=emergency,
        chatter=chatter,
        overshoot=overshoot,
        stale_planner=stale,
        jerk_limited=lim.jerk_limited,
        accel_limited=lim.accel_limited,
        curvature_limited=lim.curvature_limited,
        oscillating=chatter,
    )
