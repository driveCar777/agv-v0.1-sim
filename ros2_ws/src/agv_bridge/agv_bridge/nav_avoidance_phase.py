"""P0-D.1 — Predictive avoidance phase machine (detect / probe / commit / reconnect).

Separates DETECTION from ACTION. Does NOT emit cmd_vel.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_obstacle_preview import required_avoidance_distance

# Avoidance phases
PHASE_OPEN = "OPEN"
PHASE_FUTURE_PREVIEW = "FUTURE_PREVIEW"
PHASE_OBSTACLE_APPROACH = "OBSTACLE_APPROACH"
PHASE_SIDE_PROBE = "SIDE_PROBE"
PHASE_SIDE_COMMIT = "SIDE_COMMIT"
PHASE_OBSTACLE_PASS = "OBSTACLE_PASS"
PHASE_GLOBAL_RECONNECT = "GLOBAL_RECONNECT"

# Action readiness signals (not immediate turn)
SIGNAL_NONE = "NONE"
SIGNAL_DETECTED = "DETECTED"
SIGNAL_PREDICTED = "PREDICTED"
SIGNAL_WARNING = "WARNING"
SIGNAL_MANEUVER_READY = "MANEUVER_READY"
SIGNAL_COMMITTED = "COMMITTED"

COMMIT_HOLD_S = 0.45


def compute_tier_distances(
    vx: float,
    preview_m: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    w: float = 0.0,
) -> Dict[str, float]:
    """d_detection > d_probe_start > d_commit > d_hard_stop."""
    d_hard = float(geom.front_stop_m)
    d_commit = required_avoidance_distance(vx, geom, w=w)
    v = max(abs(float(vx)), 0.08)
    d_probe = d_commit + max(0.75, v * 1.15 + 0.35)
    d_detection = max(float(preview_m), d_probe + 0.5)
    return {
        "d_detection_m": round(d_detection, 3),
        "d_probe_start_m": round(d_probe, 3),
        "d_commit_m": round(d_commit, 3),
        "d_hard_stop_m": round(d_hard, 3),
        "required_avoidance_distance_m": round(d_commit, 3),
        "maneuver_start_distance_m": round(d_commit, 3),
        "hard_stop_distance_m": round(d_hard, 3),
    }


def compute_readiness_signal(
    *,
    future_collision: bool,
    first_collision_m: Optional[float],
    tiers: Dict[str, float],
) -> str:
    if not future_collision or first_collision_m is None:
        return SIGNAL_NONE
    fc = float(first_collision_m)
    if fc <= tiers["d_commit_m"]:
        return SIGNAL_MANEUVER_READY
    if fc <= tiers["d_probe_start_m"]:
        return SIGNAL_WARNING
    if fc <= tiers["d_detection_m"]:
        return SIGNAL_PREDICTED
    return SIGNAL_DETECTED


def classify_strict_obstacle_passed(
    *,
    front_near: float,
    left_free: float,
    right_free: float,
    future_collision: bool,
    lateral_error: float,
    local_plan_active: bool,
    committed_side: Optional[str],
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> str:
    """Strict pass state — not front_near alone."""
    if future_collision and front_near < geom.front_cost_m + 0.4:
        if committed_side and abs(lateral_error) > 0.20:
            return "PASSING"
        return "APPROACHING"
    if not future_collision and front_near > geom.front_clear_m and min(left_free, right_free) > 0.55:
        return "PASSED"
    if local_plan_active and committed_side and front_near > geom.front_stop_m + 0.35:
        return "BESIDE"
    if front_near < geom.front_clear_m:
        return "BESIDE"
    return "UNKNOWN"


@dataclass
class AvoidancePhaseState:
    phase: str = PHASE_OPEN
    signal: str = SIGNAL_NONE
    probe_active: bool = False
    side_probe_active: bool = False
    commit_ready: bool = False
    committed_side: Optional[str] = None
    global_reconnect_blocked: bool = False
    obstacle_pass_state: str = "UNKNOWN"
    tiers: Dict[str, float] = field(default_factory=dict)
    lead_time_m: Optional[float] = None
    side_history: List[str] = field(default_factory=list)
    phase_enter_ts: float = 0.0
    commit_hold_until: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "phase": self.phase,
            "avoidance_phase": self.phase,
            "signal": self.signal,
            "readiness_signal": self.signal,
            "probe_active": self.probe_active,
            "side_probe_active": self.side_probe_active,
            "commit_ready": self.commit_ready,
            "committed_side": self.committed_side,
            "global_reconnect_blocked": self.global_reconnect_blocked,
            "obstacle_pass_state": self.obstacle_pass_state,
            "d_detection_m": self.tiers.get("d_detection_m"),
            "d_probe_start_m": self.tiers.get("d_probe_start_m"),
            "d_commit_m": self.tiers.get("d_commit_m"),
            "d_hard_stop_m": self.tiers.get("d_hard_stop_m"),
            "avoidance_lead_time_m": self.lead_time_m,
            "side_history": list(self.side_history[-6:]),
            "source": "AVOIDANCE_PHASE",
            "controls_vehicle": False,
        }


class AvoidancePhaseTracker:
    """Hysteresis-aware avoidance phase tracker."""

    def __init__(self) -> None:
        self.state = AvoidancePhaseState()
        self._side_hist: Deque[str] = deque(maxlen=8)
        self._turn_start_distance_m: Optional[float] = None

    def reset(self) -> None:
        self.state = AvoidancePhaseState()
        self._side_hist.clear()
        self._turn_start_distance_m = None

    def update(
        self,
        *,
        now: float,
        vx: float,
        preview_m: float,
        future_collision: bool,
        first_collision_m: Optional[float],
        front_near: float,
        left_free: float,
        right_free: float,
        lateral_error: float,
        side_probe_active: bool,
        probe_confidence_left: float,
        probe_confidence_right: float,
        preferred_side: Optional[str],
        commit_ready: bool,
        left_probe_valid: bool = False,
        right_probe_valid: bool = False,
        committed_side_external: Optional[str],
        obstacle_passed_external: bool,
        commitment_active: bool,
        w: float = 0.0,
        geom: Optional[VehicleGeometry] = None,
    ) -> AvoidancePhaseState:
        g = geom or get_vehicle_geometry()
        tiers = compute_tier_distances(vx, preview_m, g, w=w)
        signal = compute_readiness_signal(
            future_collision=future_collision,
            first_collision_m=first_collision_m,
            tiers=tiers,
        )

        prev_phase = self.state.phase
        phase = PHASE_OPEN
        committed = committed_side_external
        reconnect_blocked = False

        if obstacle_passed_external:
            phase = PHASE_GLOBAL_RECONNECT if commitment_active else PHASE_OBSTACLE_PASS
        elif not future_collision:
            phase = PHASE_OPEN
        elif signal in (SIGNAL_DETECTED, SIGNAL_PREDICTED):
            phase = PHASE_FUTURE_PREVIEW
        elif signal == SIGNAL_WARNING:
            phase = PHASE_SIDE_PROBE if side_probe_active else PHASE_OBSTACLE_APPROACH
        elif signal == SIGNAL_MANEUVER_READY:
            if not (left_probe_valid or right_probe_valid):
                phase = PHASE_OBSTACLE_APPROACH
            elif commit_ready and preferred_side:
                if self.state.committed_side == preferred_side and now < self.state.commit_hold_until:
                    phase = PHASE_SIDE_COMMIT
                    committed = preferred_side
                elif commit_ready:
                    phase = PHASE_SIDE_COMMIT
                    committed = preferred_side
                    self.state.commit_hold_until = now + COMMIT_HOLD_S
                    if self._turn_start_distance_m is None:
                        self._turn_start_distance_m = float(front_near)
                else:
                    phase = PHASE_SIDE_PROBE
            else:
                phase = PHASE_OBSTACLE_APPROACH
        else:
            phase = PHASE_FUTURE_PREVIEW

        if future_collision and not obstacle_passed_external:
            reconnect_blocked = True

        if preferred_side:
            self._side_hist.append(preferred_side)

        pass_st = classify_strict_obstacle_passed(
            front_near=front_near,
            left_free=left_free,
            right_free=right_free,
            future_collision=future_collision,
            lateral_error=lateral_error,
            local_plan_active=commit_ready or commitment_active,
            committed_side=committed,
            geom=g,
        )

        lead = None
        if self._turn_start_distance_m is not None and first_collision_m is not None:
            lead = float(first_collision_m) - float(self._turn_start_distance_m)

        if phase != prev_phase:
            self.state.phase_enter_ts = now

        self.state.phase = phase
        self.state.signal = SIGNAL_COMMITTED if phase == PHASE_SIDE_COMMIT else signal
        self.state.tiers = tiers
        self.state.probe_active = future_collision
        self.state.side_probe_active = side_probe_active or phase in (PHASE_SIDE_PROBE, PHASE_OBSTACLE_APPROACH)
        self.state.commit_ready = commit_ready
        self.state.committed_side = committed
        self.state.global_reconnect_blocked = reconnect_blocked
        self.state.obstacle_pass_state = pass_st
        self.state.lead_time_m = lead
        self.state.side_history = list(self._side_hist)
        return self.state
