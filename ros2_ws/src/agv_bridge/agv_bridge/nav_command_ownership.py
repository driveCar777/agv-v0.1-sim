"""M3.8.3 command ownership — classify who wrote approved vx/omega.

Does not emit cmd_vel. Read-only classification of an already-decided cycle.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

SRC_EMERGENCY_STOP = "EMERGENCY_STOP"
SRC_SAFETY_LIMIT = "SAFETY_LIMIT"
SRC_RECOVERY = "RECOVERY"
SRC_LOCAL_AVOIDANCE = "LOCAL_AVOIDANCE"
SRC_LOCAL_MPPI = "LOCAL_MPPI"
SRC_GLOBAL_PATH_TRACKER = "GLOBAL_PATH_TRACKER"
SRC_DYNAMIC_WAIT = "DYNAMIC_WAIT"
SRC_FALLBACK = "FALLBACK"
SRC_MANUAL = "MANUAL"
SRC_OTHER = "OTHER"

MODULE_SAFETY = "sim_api_ext.apply_safety"
MODULE_MPPI = "mppi_controller.DiffDriveMppi.step"
MODULE_MANEUVER = "maneuver.ManeuverFSM"
MODULE_PHYSICS = "sim_api_ext._physics_loop"
MODULE_MANUAL = "sim_api_ext.set_translate"


def omega_side(w: Optional[float], *, eps: float = 0.02) -> str:
    if w is None:
        return "UNKNOWN"
    if w > eps:
        return "LEFT"
    if w < -eps:
        return "RIGHT"
    return "STRAIGHT"


def _sign(v: float, *, eps: float = 0.02) -> int:
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


def safety_direction_override(requested_w: float, approved_w: float, *, eps: float = 0.02) -> bool:
    """True if Safety flipped omega sign (not mere magnitude clamp / zero)."""
    rs, as_ = _sign(requested_w, eps=eps), _sign(approved_w, eps=eps)
    return rs != 0 and as_ != 0 and rs != as_


def path_heading_rad(path: Sequence[Any], x: float, y: float) -> Optional[float]:
    if not path or len(path) < 2:
        return None
    pts: List[Tuple[float, float]] = []
    for p in path:
        if isinstance(p, dict):
            pts.append((float(p.get("x") or 0), float(p.get("y") or 0)))
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            pts.append((float(p[0]), float(p[1])))
    if len(pts) < 2:
        return None
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(pts):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_d, best_i = d, i
    j = min(best_i + 1, len(pts) - 1)
    if j == best_i:
        j = max(0, best_i - 1)
    dx = pts[j][0] - pts[best_i][0]
    dy = pts[j][1] - pts[best_i][1]
    if math.hypot(dx, dy) < 1e-6:
        return None
    return math.atan2(dy, dx)


def classify_command_source(
    *,
    nav_mode: str = "",
    emergency: bool = False,
    maneuver_mode: str = "",
    planner_state: str = "",
    follow_path_source: str = "",
    tracking_local_plan: bool = False,
    requested_vx: float = 0.0,
    requested_omega: float = 0.0,
    approved_vx: float = 0.0,
    approved_omega: float = 0.0,
    safe_vx_reason: str = "NORMAL",
    stop_reason: str = "NONE",
    local_plan_kinematic_valid: Optional[bool] = None,
    physical_control_eligible: Optional[bool] = None,
) -> Dict[str, Any]:
    mm = str(maneuver_mode or "").upper()
    follow = str(follow_path_source or "").upper()
    mode = str(nav_mode or "").lower()
    reason = "MPPI_OUTPUT"
    module = MODULE_MPPI
    source = SRC_LOCAL_MPPI
    fallback = "NONE"

    if mode not in ("tracking", "avoid") and str(stop_reason or "") not in ("FRONT_OBSTACLE",):
        if abs(approved_vx) > 1e-4 or abs(approved_omega) > 1e-4:
            source, module, reason = SRC_MANUAL, MODULE_MANUAL, "MANUAL_3055"
        else:
            source, module, reason = SRC_OTHER, MODULE_PHYSICS, "IDLE_OR_STOPPED"
    elif emergency or str(stop_reason or "") in ("EMERGENCY", "E_STOP"):
        source, module, reason = SRC_EMERGENCY_STOP, MODULE_SAFETY, "EMERGENCY"
    elif str(planner_state or "").upper() in ("NAVIGATION_FAILED", "SAFE_STOP"):
        source, module, reason = SRC_SAFETY_LIMIT, MODULE_SAFETY, str(planner_state)
    elif mm in ("REVERSE_ESCAPE",) or "RECOVERY" in str(planner_state or "").upper():
        source, module, reason = SRC_RECOVERY, MODULE_MANEUVER, mm or planner_state
    elif mm in ("LOCAL_LEFT", "LOCAL_RIGHT") or follow == "MANEUVER":
        source, module, reason = SRC_LOCAL_AVOIDANCE, MODULE_MANEUVER, mm or "MANEUVER_FORCE_W"
    elif mm in ("POST_TURN", "PATH_RECAPTURE"):
        source, module, reason = SRC_GLOBAL_PATH_TRACKER, MODULE_MANEUVER, "POST_TURN_RECAPTURE"
        fallback = "GLOBAL_TRACK"
    elif mm in ("WAIT_FOR_CLEARANCE", "SAFE_STOP"):
        source, module, reason = SRC_DYNAMIC_WAIT, MODULE_MANEUVER, mm
    elif tracking_local_plan or follow == "LOCAL_PLAN":
        source, module, reason = SRC_LOCAL_MPPI, MODULE_MPPI, "PP_PLUS_MPPI_ON_LOCAL_PLAN"
        if physical_control_eligible is False or local_plan_kinematic_valid is False:
            fallback = "STALE_TRAJECTORY_STILL_TRACKED"
            reason = "PP_ON_LOCAL_PLAN|PUBLISHED_TRAJECTORY_STALE"
    elif follow in ("GLOBAL_PATH", "GLOBAL", ""):
        source, module, reason = SRC_GLOBAL_PATH_TRACKER, MODULE_MPPI, "PP_ON_GLOBAL_PATH"
        if local_plan_kinematic_valid is False or physical_control_eligible is False:
            fallback = "STALE_LOCAL_TO_GLOBAL_TRACK"
            reason = "LOCAL_PLAN_NOT_TRACKING|PP_ON_GLOBAL_PATH"
        else:
            fallback = "GLOBAL_TRACK"
            reason = "AUTH_OR_NO_LOCAL_PLAN|PP_ON_GLOBAL_PATH"
    else:
        source, module, reason = SRC_OTHER, MODULE_MPPI, follow or "UNKNOWN_FOLLOW"

    safety_over = safety_direction_override(requested_omega, approved_omega)
    if safety_over:
        source, module, reason = SRC_SAFETY_LIMIT, MODULE_SAFETY, "SAFETY_DIRECTION_OVERRIDE"
    elif (
        source not in (SRC_EMERGENCY_STOP, SRC_MANUAL)
        and abs(requested_vx) + abs(requested_omega) > 0.04
        and abs(approved_vx) < 1e-4
        and abs(approved_omega) < 1e-4
        and str(safe_vx_reason or "NORMAL") not in ("NORMAL", "")
    ):
        # Magnitude/zero clamp — owner remains planner, safety is last writer of zeros.
        pass

    last_writer = MODULE_SAFETY if (
        abs(approved_omega - requested_omega) > 0.005 or abs(approved_vx - requested_vx) > 0.005
    ) else MODULE_MPPI

    trace = [
        f"MPPI wrote omega={requested_omega:+.4f} vx={requested_vx:+.4f} follow={follow or 'NONE'}",
        f"Safety {safe_vx_reason or 'NORMAL'} omega {requested_omega:+.4f}->{approved_omega:+.4f}",
        f"Final approved omega={approved_omega:+.4f} source={source}",
    ]
    if safety_over:
        trace.insert(1, "SAFETY_DIRECTION_OVERRIDE")

    return {
        "command_source": source,
        "command_source_module": module,
        "command_source_reason": reason,
        "command_reason": reason,
        "last_command_writer": last_writer,
        "last_command_reason": reason,
        "fallback": fallback,
        "safety_direction_override": safety_over,
        "command_side": omega_side(approved_omega),
        "requested_side": omega_side(requested_omega),
        "command_write_trace": trace,
        "follow_path_source": follow or "NONE",
        "tracking_local_plan": bool(tracking_local_plan),
    }


def command_actual_consistency(
    *,
    approved_omega: float,
    actual_omega: float,
    eps: float = 0.03,
) -> Dict[str, Any]:
    mismatch = _sign(approved_omega, eps=eps) != 0 and _sign(actual_omega, eps=eps) != 0 and _sign(approved_omega, eps=eps) != _sign(actual_omega, eps=eps)
    return {
        "command_actual_sign": "MISMATCH" if mismatch else "MATCH",
        "actuator_direction_mismatch": mismatch,
        "command_actual_magnitude_error": round(abs(approved_omega - actual_omega), 4),
    }


def trajectory_command_consistency(
    *,
    vx: float,
    omega: float,
    direction_dot: Optional[float],
    trajectory_control_eligible: Optional[bool],
) -> Dict[str, Any]:
    issues: List[str] = []
    if trajectory_control_eligible is False:
        issues.append("TRAJECTORY_NOT_CONTROL_ELIGIBLE")
    if vx > 0.04 and direction_dot is not None and direction_dot < -0.3 and trajectory_control_eligible:
        issues.append("COMMAND_TRAJECTORY_INCONSISTENCY")
    # omega vs curvature: +omega LEFT; negative direction_angle is RIGHT of heading
    return {
        "issues": issues,
        "command_trajectory_inconsistency": "COMMAND_TRAJECTORY_INCONSISTENCY" in issues,
    }
