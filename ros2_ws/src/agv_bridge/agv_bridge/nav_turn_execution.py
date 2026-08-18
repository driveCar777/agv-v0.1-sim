"""M3.9.1 turn execution latency / readiness / late-turn forensics."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_model

# Derived at 20 Hz control: |omega| must exceed noise floor and sustain 2 samples.
CONTROL_HZ = 20.0
TURN_OMEGA_EPSILON = 0.05  # rad/s — ~2.9 deg/s at 20 Hz
TURN_CONSECUTIVE_SAMPLES = 2
TURN_START_THRESHOLD = {
    "omega_rad_s": TURN_OMEGA_EPSILON,
    "consecutive_samples": TURN_CONSECUTIVE_SAMPLES,
    "control_hz": CONTROL_HZ,
    "basis": "20Hz control, 0.05 rad/s noise floor, 2 consecutive samples",
}

READINESS_READY = "READY"
READINESS_REQUIRED = "REQUIRED"
READINESS_LATE = "LATE"
READINESS_INFEASIBLE = "INFEASIBLE"

FEASIBLE = "FEASIBLE"
MARGINAL = "MARGINAL"
FEAS_LATE = "LATE"
INFEASIBLE = "INFEASIBLE"


def turn_required_time_s(
    *,
    omega_target: float,
    omega_current: float = 0.0,
    alpha_max: Optional[float] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> float:
    am = float(alpha_max if alpha_max is not None else geom.acc_w)
    if am <= 1e-6:
        return 0.0
    return abs(float(omega_target) - float(omega_current)) / am


def turn_required_distance_m(
    *,
    vx: float,
    omega_target: float,
    omega_current: float = 0.0,
    alpha_max: Optional[float] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> float:
    t = turn_required_time_s(
        omega_target=omega_target,
        omega_current=omega_current,
        alpha_max=alpha_max,
        geom=geom,
    )
    return max(0.0, abs(float(vx))) * t


def classify_turn_readiness(
    *,
    obstacle_distance_m: Optional[float],
    turn_required_distance_m: float,
    safety_margin_m: float = 0.35,
) -> str:
    if obstacle_distance_m is None:
        return READINESS_READY
    obs = float(obstacle_distance_m)
    req = float(turn_required_distance_m)
    margin = obs - req - float(safety_margin_m)
    if margin >= 0.5:
        return READINESS_READY
    if margin >= 0.0:
        return READINESS_REQUIRED
    if obs > 0.0:
        return READINESS_LATE
    return READINESS_INFEASIBLE


def classify_turn_feasibility(
    *,
    obstacle_distance_m: Optional[float],
    vx: float,
    omega_target: float,
    omega_current: float = 0.0,
    alpha_max: Optional[float] = None,
    ay_max: Optional[float] = None,
    safety_margin_m: float = 0.35,
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> str:
    req_d = turn_required_distance_m(
        vx=vx,
        omega_target=omega_target,
        omega_current=omega_current,
        alpha_max=alpha_max,
        geom=geom,
    )
    braking_d = 0.0
    try:
        braking = get_vehicle_model().braking
        stop_d = braking.stopping_distance_m(max(0.0, float(vx)))
        if stop_d is not None:
            braking_d = float(stop_d)
    except Exception:
        braking_d = 0.5 * float(vx) * float(vx) / max(float(geom.acc_v), 0.1)
    d_total = req_d + braking_d + float(safety_margin_m)
    if obstacle_distance_m is None:
        return FEASIBLE
    obs = float(obstacle_distance_m)
    if obs < d_total * 0.85:
        return INFEASIBLE
    if obs < d_total:
        return FEAS_LATE
    if obs < d_total + 0.35:
        return MARGINAL
    return FEASIBLE


def turn_margin_m(
    *,
    obstacle_distance_m: Optional[float],
    turn_required_distance_m: float,
    safety_margin_m: float = 0.35,
) -> Optional[float]:
    if obstacle_distance_m is None:
        return None
    return float(obstacle_distance_m) - float(turn_required_distance_m) - float(safety_margin_m)


def omega_sign_side(omega: float, *, eps: float = TURN_OMEGA_EPSILON) -> str:
    if omega > eps:
        return "LEFT"
    if omega < -eps:
        return "RIGHT"
    return "STRAIGHT"


def turn_started(omega: float, *, eps: float = TURN_OMEGA_EPSILON) -> bool:
    return abs(float(omega)) >= eps


def detect_turn_start_frames(
    omegas: Sequence[float],
    *,
    eps: float = TURN_OMEGA_EPSILON,
    consecutive: int = TURN_CONSECUTIVE_SAMPLES,
) -> Optional[int]:
    """Return index of first sustained turn start in omega series."""
    run = 0
    for i, w in enumerate(omegas):
        if abs(float(w)) >= eps:
            run += 1
            if run >= consecutive:
                return i - consecutive + 1
        else:
            run = 0
    return None


@dataclass
class TurnExecutionSnapshot:
    turn_readiness: str
    turn_feasibility: str
    turn_required_time_s: float
    turn_required_distance_m: float
    turn_margin_m: Optional[float]
    obstacle_distance_m: Optional[float]
    command_source: str = ""
    trajectory_valid: bool = False
    trajectory_stale: bool = False
    planner_age_ms: Optional[float] = None
    command_age_ms: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "turn_readiness": self.turn_readiness,
            "turn_feasibility": self.turn_feasibility,
            "turn_required_time_s": round(self.turn_required_time_s, 4),
            "turn_required_distance_m": round(self.turn_required_distance_m, 4),
            "turn_margin_m": None if self.turn_margin_m is None else round(self.turn_margin_m, 4),
            "obstacle_distance_m": self.obstacle_distance_m,
            "command_source": self.command_source,
            "trajectory_valid": self.trajectory_valid,
            "trajectory_stale": self.trajectory_stale,
            "planner_age_ms": self.planner_age_ms,
            "command_age_ms": self.command_age_ms,
            "turn_start_threshold": TURN_START_THRESHOLD,
        }


def build_turn_snapshot(
    *,
    obstacle_distance_m: Optional[float],
    vx: float,
    omega_target: float = 0.28,
    omega_current: float = 0.0,
    command_source: str = "",
    trajectory_control_eligible: Optional[bool] = None,
    trajectory_stale: bool = False,
    planner_age_ms: Optional[float] = None,
    command_age_ms: Optional[float] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> TurnExecutionSnapshot:
    req_t = turn_required_time_s(omega_target=omega_target, omega_current=omega_current, geom=geom)
    req_d = turn_required_distance_m(
        vx=vx,
        omega_target=omega_target,
        omega_current=omega_current,
        geom=geom,
    )
    margin = turn_margin_m(
        obstacle_distance_m=obstacle_distance_m,
        turn_required_distance_m=req_d,
    )
    readiness = classify_turn_readiness(
        obstacle_distance_m=obstacle_distance_m,
        turn_required_distance_m=req_d,
    )
    feasibility = classify_turn_feasibility(
        obstacle_distance_m=obstacle_distance_m,
        vx=vx,
        omega_target=omega_target,
        omega_current=omega_current,
        geom=geom,
    )
    return TurnExecutionSnapshot(
        turn_readiness=readiness,
        turn_feasibility=feasibility,
        turn_required_time_s=req_t,
        turn_required_distance_m=req_d,
        turn_margin_m=margin,
        obstacle_distance_m=obstacle_distance_m,
        command_source=str(command_source or ""),
        trajectory_valid=trajectory_control_eligible is True,
        trajectory_stale=bool(trajectory_stale),
        planner_age_ms=planner_age_ms,
        command_age_ms=command_age_ms,
    )
