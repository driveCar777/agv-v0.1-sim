"""Navigation Debug / Execution Black Box (read-only observability).

Extends prior dashboard with time-series, execution events, pose trace,
session/run summary and post-mortem. Does NOT change navigation algorithms.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_geometry import (
    DEFAULT_GEOM,
    MAX_RECOVERY_ATTEMPTS,
    RECOVERY_COOLDOWN_S,
    REVERSE_MAX_S,
)
from agv_bridge.nav_incident import ApproachAnalyzer, IncidentRecorder, INCIDENT_TRIGGERS

Pt = Tuple[float, float]

# Ring sizes: ~30s @ 20Hz
TELEM_MAX = 640
POSE_MAX = 640
EVENT_MAX = 300


@dataclass
class TelemetrySample:
    timestamp: float
    session_id: str
    x: float
    y: float
    theta: float
    mppi_vx: float
    cmd_vx: float
    safe_vx: float
    state_vx: float
    mppi_w: float
    cmd_w: float
    safe_w: float
    state_w: float
    ax: float
    alpha: float
    front_near: float
    rear_near: float
    collision: bool
    path_progress_s: float
    path_progress_rate: float
    lateral_error: float
    heading_error: float
    phase: str
    stop_reason: str
    recovery_attempts: int
    planner_status: str
    forward_trajectory: str
    best_cost: float
    motion_state: str
    nav_mode: str = "idle"
    control_mode: str = "mppi"
    goal_distance: float = 0.0
    stuck_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class EventRing:
    def __init__(self, maxlen: int = EVENT_MAX) -> None:
        self._events: Deque[Dict[str, Any]] = deque(maxlen=maxlen)
        self._last_keys: Dict[str, float] = {}
        self.on_push = None  # Optional[Callable[[str, Dict], None]]

    def clear(self) -> None:
        self._events.clear()
        self._last_keys.clear()

    def push(
        self,
        event: str,
        payload: Optional[Dict[str, Any]] = None,
        *,
        min_interval_s: float = 0.25,
        category: str = "INFO",
    ) -> Optional[Dict[str, Any]]:
        now = time.time()
        reason = str((payload or {}).get("stop_reason") or (payload or {}).get("message") or "")
        dedupe = f"{event}:{reason}"
        last2 = self._last_keys.get(dedupe, 0.0)
        critical = {
            "PLAN_SUCCESS",
            "PLAN_FAILED",
            "ARRIVED",
            "FAILED",
            "MANUAL_STOP",
            "CANDIDATE_SWITCH",
            "RECOVERY_EXHAUSTED",
            "ACCELERATION_START",
            "DECELERATION_START",
            "TURN_START",
            "REVERSE_START",
            "SAFETY_BLOCK",
            "OBSTACLE_DETECTED",
            "RECOVERY_START",
            "PATH_PROGRESS_STALLED",
            "DIVERGENCE_START",
            "ANGULAR_OSCILLATION",
            "REVERSE_WITH_FORWARD_AVAILABLE",
            "APPROACHING_OBSTACLE_WHILE_TRACKING",
            "APPROACHED_OBSTACLE_THEN_REVERSE",
            "RECOVERY_MADE_GEOMETRY_WORSE",
            "LATERAL_ERROR_DIVERGING",
            "STEERING_RESPONSE_WEAK",
        }
        if now - last2 < min_interval_s and event not in critical:
            return None
        self._last_keys[event] = now
        self._last_keys[dedupe] = now
        row: Dict[str, Any] = {"ts": now, "event": event, "type": category, "category": category}
        if payload:
            row.update(payload)
        self._events.append(row)
        if self.on_push:
            try:
                self.on_push(event, row)
            except Exception:
                pass
        return row

    def list(self, limit: int = 80) -> List[Dict[str, Any]]:
        return list(self._events)[-limit:]


class TelemetryRing:
    def __init__(self, maxlen: int = TELEM_MAX) -> None:
        self._rows: Deque[Dict[str, Any]] = deque(maxlen=maxlen)

    def clear(self) -> None:
        self._rows.clear()

    def push(self, row: Dict[str, Any]) -> None:
        self._rows.append(row)

    def list(self, limit: int = 300) -> List[Dict[str, Any]]:
        return list(self._rows)[-limit:]

    def __len__(self) -> int:
        return len(self._rows)

    @property
    def last(self) -> Optional[Dict[str, Any]]:
        return self._rows[-1] if self._rows else None


def _approx0(v: float, eps: float = 0.02) -> bool:
    return abs(float(v)) < eps


def classify_motion_state(
    *,
    state_vx: float,
    state_w: float,
    ax: float,
    phase: str,
    stop_reason: str,
) -> str:
    if phase in ("reverse_escape", "recover") or stop_reason in ("STUCK_RECOVERY", "REVERSE_ESCAPE"):
        if state_vx < -0.03:
            return "REVERSE"
        return "RECOVERY"
    if phase == "safe_stop" or stop_reason == "FAILED":
        return "STOPPED"
    if abs(state_vx) < 0.03 and abs(state_w) < 0.05:
        return "STOPPED"
    if state_vx < -0.03:
        return "REVERSE"
    if ax < -0.15:
        return "DECELERATING"
    if ax > 0.15:
        return "ACCELERATING"
    if abs(state_w) > 0.08:
        return "TURNING"
    if state_vx > 0.03:
        return "FORWARD"
    return "STOPPED"


def diagnose_why_not_moving(
    *,
    mppi_vx: float,
    safe_vx: float,
    final_vx: float,
    state_vx: float,
    front_near: float,
    front_stop: float,
    collision: bool,
    emergency: bool,
    phase: str,
    stop_reason: str,
    path_progress_s: float,
    path_progress_delta: float,
    recovery_attempts: int,
    max_recovery: int,
    forward_candidate_available: bool,
    best_forward: Optional[Dict[str, Any]],
    best_stop: Optional[Dict[str, Any]],
    path_progress_rate: float = 0.0,
    has_global_path: bool = False,
) -> Dict[str, Any]:
    secondary: List[str] = []
    primary = "NONE"
    explanation = "Vehicle can move or is idle without blockage."
    anomalies: List[str] = []

    if emergency:
        primary = "EMERGENCY"
        explanation = "Emergency / soft EMC active."
    elif phase == "safe_stop" or stop_reason == "FAILED" or (
        recovery_attempts >= max_recovery and phase in ("safe_stop", "recover")
    ):
        primary = "RECOVERY_EXHAUSTED"
        explanation = f"Recovery exhausted ({recovery_attempts}/{max_recovery})."
    elif stop_reason == "FRONT_OBSTACLE" or (front_near < front_stop and mppi_vx >= 0):
        primary = "SAFETY_FRONT_BLOCK"
        explanation = f"front_near={front_near:.2f}m threshold={front_stop:.2f}m."
    elif stop_reason == "REAR_OBSTACLE":
        primary = "SAFETY_REAR_BLOCK"
        explanation = "Rear gate blocked."
    elif stop_reason == "COLLISION_GUARD" or collision:
        primary = "COLLISION_GUARD"
        explanation = "Collision guard active."
        if front_near > 1.5:
            anomalies.append("CLEARANCE_COLLISION_MISMATCH")
    elif (not _approx0(mppi_vx, 0.03)) and _approx0(safe_vx, 0.03):
        primary = "SAFETY_SUPERVISOR_BLOCKED"
        explanation = "MPPI requested motion but safety zeroed vx."
        anomalies.append("SAFETY_SUPERVISOR_BLOCKED")
    elif _approx0(mppi_vx, 0.03) and front_near > front_stop + 0.15 and not collision:
        primary = "MPPI_NO_FORWARD_TRAJECTORY"
        explanation = "MPPI did not select forward motion while front is clear."
        anomalies.append("MPPI_NO_FORWARD_TRAJECTORY")
        if has_global_path and not forward_candidate_available:
            anomalies.append("GLOBAL_LOCAL_PLANNING_MISMATCH")
    elif (not _approx0(safe_vx, 0.03) or not _approx0(final_vx, 0.03)) and _approx0(state_vx, 0.03):
        primary = "CMD_BUT_STATE_STILL"
        explanation = "Command vx exists but state.vx≈0."
        anomalies.append("COMMAND_EXECUTION_MISMATCH")
    elif (not _approx0(state_vx, 0.03)) and path_progress_rate < 0.025:
        primary = "VELOCITY_BUT_PATH_STALLED"
        explanation = "state.vx>0 but path progress stalled."
        anomalies.append("VELOCITY_BUT_PATH_STALLED")
    elif _approx0(safe_vx, 0.03) and stop_reason == "NONE" and phase == "forward":
        if abs(mppi_vx) < 0.03:
            primary = "MPPI_NO_FORWARD_TRAJECTORY"
        else:
            primary = "UNEXPLAINED_ZERO_SPEED"
            anomalies.append("UNEXPLAINED_ZERO_SPEED")
            explanation = "safe_vx=0 but stop_reason=NONE."
    elif stop_reason in ("REVERSE_ESCAPE", "STUCK_RECOVERY"):
        primary = stop_reason
        explanation = f"In recovery ({stop_reason})."
    elif stop_reason == "REPLAN":
        primary = "REPLAN"
        explanation = "Global replan."
    elif stop_reason == "GOAL_REACHED":
        primary = "GOAL_REACHED"
        explanation = "Goal reached."
    elif stop_reason == "MANUAL_STOP":
        primary = "MANUAL_STOP"
        explanation = "Manual stop."
    elif stop_reason == "NO_GLOBAL_PATH":
        primary = "NO_GLOBAL_PATH"
        explanation = "No global path."

    if collision and front_near > 1.5 and "CLEARANCE_COLLISION_MISMATCH" not in anomalies:
        anomalies.append("CLEARANCE_COLLISION_MISMATCH")
    if has_global_path and not forward_candidate_available and primary == "MPPI_NO_FORWARD_TRAJECTORY":
        if "GLOBAL_LOCAL_PLANNING_MISMATCH" not in anomalies:
            anomalies.append("GLOBAL_LOCAL_PLANNING_MISMATCH")

    moving = not _approx0(state_vx, 0.03) and primary == "NONE"
    chain = {
        "mppi_vx": round(mppi_vx, 4),
        "cmd_vx": round(mppi_vx, 4),
        "safe_vx": round(safe_vx, 4),
        "state_vx": round(state_vx, 4),
    }
    vel_chain_note = "OK"
    if mppi_vx > 0.05 and safe_vx <= 0.01:
        vel_chain_note = "SAFETY CUT"
    elif abs(mppi_vx) <= 0.01:
        vel_chain_note = "LOCAL PLANNER STOP"
    elif safe_vx > 0.05 and abs(state_vx) <= 0.01:
        vel_chain_note = "DYNAMICS / SLEW DELAY"

    return {
        "primary_reason": primary if not moving else "NONE",
        "secondary_reasons": secondary,
        "anomalies": anomalies,
        "explanation": explanation if not moving else "Moving / no stop.",
        "why_stopped": primary if (primary != "NONE" or _approx0(state_vx, 0.03)) else "NONE",
        "command_chain": chain,
        "velocity_chain_note": vel_chain_note,
        "forward_candidate_available": forward_candidate_available,
        "best_forward": best_forward,
        "best_stopish": best_stop,
        "causal_chain": _causal_chain(primary, mppi_vx, safe_vx, state_vx, stop_reason, forward_candidate_available),
    }


def _causal_chain(
    primary: str,
    mppi_vx: float,
    safe_vx: float,
    state_vx: float,
    stop_reason: str,
    forward_ok: bool,
) -> List[Dict[str, str]]:
    steps = [{"label": "MPPI", "detail": f"vx={mppi_vx:.3f}" + ("" if forward_ok else " · forward NOT_AVAILABLE")}]
    if primary in ("SAFETY_FRONT_BLOCK", "SAFETY_REAR_BLOCK", "COLLISION_GUARD", "SAFETY_SUPERVISOR_BLOCKED", "EMERGENCY"):
        steps.append({"label": "Safety", "detail": stop_reason or primary})
        steps.append({"label": "safe_vx", "detail": f"{safe_vx:.3f}"})
    elif primary == "MPPI_NO_FORWARD_TRAJECTORY":
        steps.append({"label": "Local", "detail": "NO_FORWARD_TRAJECTORY"})
        steps.append({"label": "safe_vx", "detail": f"{safe_vx:.3f}"})
    else:
        steps.append({"label": "Safety", "detail": stop_reason or "ALLOW"})
        steps.append({"label": "safe_vx", "detail": f"{safe_vx:.3f}"})
    steps.append({"label": "state_vx", "detail": f"{state_vx:.3f}"})
    if primary in ("RECOVERY_EXHAUSTED", "STUCK_RECOVERY", "REVERSE_ESCAPE"):
        steps.append({"label": "Recovery", "detail": primary})
    return steps


def diagnose_path_quality(planning: Dict[str, Any]) -> Dict[str, Any]:
    if not planning:
        return {"status": "UNKNOWN", "reasons": ["no planning metrics"]}
    reasons: List[str] = []
    ratio = float(planning.get("path_ratio") or 0.0)
    turns90 = int(planning.get("turns_90ish") or 0)
    min_c = float(planning.get("min_clearance") or 0.0)
    direct = bool(planning.get("direct_path_safe"))
    snap = float(planning.get("goal_snap_error") or 0.0)
    if direct and ratio > 1.25:
        reasons.append("direct LOS safe but planner detoured")
    if ratio > 2.5:
        reasons.append("excessive detour (ratio>2.5)")
    if turns90 >= 3:
        reasons.append("many ~90° turns")
    if min_c < 0.40:
        reasons.append("low clearance")
    if snap > 0.35:
        reasons.append("large goal snap error")
    if not reasons and ratio <= 1.15 and min_c >= 0.45:
        status = "GOOD"
    elif reasons:
        status = "POOR" if (ratio > 2.5 or (direct and ratio > 1.25) or min_c < 0.35) else "WARNING"
    else:
        status = "OK"
    return {"status": status, "reasons": reasons or ["metrics within soft limits"]}


def diagnose_tracking(
    *,
    heading_err: float,
    lateral_err: float,
    w_osc: int,
    path_delta: float,
    state_vx: float,
) -> str:
    score = 0
    if abs(heading_err) > 0.8:
        score += 2
    elif abs(heading_err) > 0.4:
        score += 1
    if abs(lateral_err) > 0.45:
        score += 2
    elif abs(lateral_err) > 0.25:
        score += 1
    if w_osc >= 4:
        score += 2
    elif w_osc >= 2:
        score += 1
    if abs(state_vx) > 0.05 and path_delta < 0.05:
        score += 1
    if score >= 4:
        return "POOR"
    if score >= 2:
        return "WARNING"
    return "GOOD"


def safety_gate_trace(
    *,
    mppi_vx: float,
    mppi_w: float,
    safe_vx: float,
    safe_w: float,
    front_near: float,
    rear_near: float,
    front_stop: float,
    rear_stop: float,
    collision: bool,
    emergency: bool,
    phase: str,
    stop_reason: str,
) -> Dict[str, Any]:
    gates = []
    gates.append({"gate": "MPPI_CMD", "vx_before": mppi_vx, "w_before": mppi_w, "vx_after": mppi_vx, "w_after": mppi_w, "status": "PASS"})
    gates.append(
        {
            "gate": "EMERGENCY",
            "status": "BLOCK" if emergency else "PASS",
            "vx_before": mppi_vx,
            "vx_after": 0.0 if emergency else mppi_vx,
            "w_before": mppi_w,
            "w_after": 0.0 if emergency else mppi_w,
        }
    )
    front_block = stop_reason == "FRONT_OBSTACLE" or (front_near < front_stop and mppi_vx >= 0)
    rear_block = stop_reason == "REAR_OBSTACLE"
    coll_block = stop_reason == "COLLISION_GUARD"
    gates.append(
        {
            "gate": "FRONT",
            "status": "BLOCK" if front_block else "PASS",
            "front_near": round(front_near, 3),
            "threshold": front_stop,
            "vx_before": mppi_vx,
            "vx_after": safe_vx,
            "w_before": mppi_w,
            "w_after": safe_w,
        }
    )
    gates.append(
        {
            "gate": "REAR",
            "status": "BLOCK" if rear_block else "PASS",
            "rear_near": round(rear_near, 3),
            "threshold": rear_stop,
            "vx_before": mppi_vx,
            "vx_after": safe_vx,
            "w_before": mppi_w,
            "w_after": safe_w,
        }
    )
    gates.append(
        {
            "gate": "COLLISION",
            "status": "BLOCK" if coll_block else ("WARN" if collision else "PASS"),
            "vx_before": mppi_vx,
            "vx_after": safe_vx,
            "w_before": mppi_w,
            "w_after": safe_w,
        }
    )
    decision = "ALLOW"
    if stop_reason in ("FRONT_OBSTACLE", "REAR_OBSTACLE", "COLLISION_GUARD", "EMERGENCY", "FAILED"):
        decision = "BLOCK"
    elif _approx0(safe_vx, 0.03) and not _approx0(mppi_vx, 0.03):
        decision = "BLOCK"
    return {
        "decision": decision,
        "block_reason": stop_reason if decision == "BLOCK" else "NONE",
        "who_stopped": (
            "SAFETY"
            if decision == "BLOCK" and stop_reason not in ("NONE",)
            else ("MPPI" if abs(mppi_vx) < 0.02 else "NONE")
        ),
        "gates": gates,
        "mppi_cmd": {"vx": round(mppi_vx, 4), "w": round(mppi_w, 4)},
        "safe_cmd": {"vx": round(safe_vx, 4), "w": round(safe_w, 4)},
    }


class ExecutionEventDetector:
    """Stateful edge detectors on consecutive TelemetrySamples."""

    def __init__(self) -> None:
        self._prev: Optional[Dict[str, Any]] = None
        self._ax_pos_streak = 0
        self._ax_neg_streak = 0
        self._turn_on = False
        self._turn_off_since: Optional[float] = None
        self._rev_on = False
        self._obs_on = False
        self._safety_on = False
        self._stall_on = False
        self._stall_window: Deque[Tuple[float, float]] = deque(maxlen=40)  # ts, progress
        self._lat_hist: Deque[float] = deque(maxlen=10)
        self._replan_times: Deque[float] = deque(maxlen=20)
        self._rev_count_window: Deque[float] = deque(maxlen=20)

    def reset(self) -> None:
        self.__init__()

    def update(self, sample: Dict[str, Any], events: EventRing) -> None:
        now = float(sample["timestamp"])
        vx = float(sample["state_vx"])
        w = float(sample["state_w"])
        ax = float(sample["ax"])
        mppi_vx = float(sample["mppi_vx"])
        cmd_vx = float(sample["cmd_vx"])
        safe_vx = float(sample["safe_vx"])
        front = float(sample["front_near"])
        prog = float(sample["path_progress_s"])
        phase = str(sample.get("phase") or "")
        stop = str(sample.get("stop_reason") or "NONE")
        sid = str(sample.get("session_id") or "")

        def emit(ev: str, cat: str, msg: str, interval: float = 0.15, **extra: Any) -> None:
            events.push(
                ev,
                {"session_id": sid, "message": msg, "category": cat, "type": cat, **extra},
                min_interval_s=interval,
                category=cat,
            )

        # accel / decel
        if ax > 0.15:
            self._ax_pos_streak += 1
            self._ax_neg_streak = 0
        elif ax < -0.15:
            self._ax_neg_streak += 1
            self._ax_pos_streak = 0
        else:
            if self._ax_pos_streak >= 2:
                emit("ACCELERATION_END", "ACCEL", f"ax→{ax:.2f}")
            if self._ax_neg_streak >= 2:
                emit("DECELERATION_END", "DECEL", f"ax→{ax:.2f}")
            self._ax_pos_streak = 0
            self._ax_neg_streak = 0
        if self._ax_pos_streak == 2:
            emit("ACCELERATION_START", "ACCEL", f"vx {self._prev.get('state_vx', 0) if self._prev else 0:.2f}→{vx:.2f}")
        if self._ax_neg_streak == 2:
            emit("DECELERATION_START", "DECEL", f"vx {self._prev.get('state_vx', 0) if self._prev else 0:.2f}→{vx:.2f}")

        # turn
        if abs(w) > 0.08:
            if not self._turn_on:
                self._turn_on = True
                emit("TURN_START", "TURN", f"w→{w:.2f}")
            self._turn_off_since = None
        else:
            if self._turn_on:
                if self._turn_off_since is None:
                    self._turn_off_since = now
                elif now - self._turn_off_since >= 0.2 and abs(w) < 0.05:
                    self._turn_on = False
                    emit("TURN_END", "TURN", f"w→{w:.2f}")

        # reverse
        if vx < -0.03:
            if not self._rev_on:
                self._rev_on = True
                self._rev_count_window.append(now)
                emit("REVERSE_START", "REVERSE", f"vx={vx:.2f}")
        elif vx >= -0.01 and self._rev_on:
            self._rev_on = False
            emit("REVERSE_END", "REVERSE", f"vx={vx:.2f}")

        # obstacle
        if front <= 0.90:
            if not self._obs_on:
                self._obs_on = True
                emit("OBSTACLE_DETECTED", "OBSTACLE", f"front={front:.2f}m", front_near=front)
        elif front > 0.95 and self._obs_on:
            self._obs_on = False
            emit("OBSTACLE_CLEARED", "OBSTACLE", f"front={front:.2f}m", front_near=front)

        # safety block
        if cmd_vx > 0.03 and safe_vx <= 0.01:
            if not self._safety_on:
                self._safety_on = True
                emit(
                    "SAFETY_BLOCK",
                    "SAFETY",
                    f"cmd={cmd_vx:.2f}→safe={safe_vx:.2f}",
                    stop_reason=stop,
                    front_near=front,
                )
        elif self._safety_on and safe_vx > 0.03:
            self._safety_on = False
            emit("SAFETY_RELEASE", "SAFETY", f"safe_vx={safe_vx:.2f}")

        if abs(vx) < 0.02 and abs(mppi_vx) < 0.02 and phase == "forward":
            emit("FORWARD_STOP", "STOP", "forward speed ~0", interval=1.0)

        # path stall: 2s window growth < 0.05 while moving
        self._stall_window.append((now, prog))
        while self._stall_window and now - self._stall_window[0][0] > 2.0:
            self._stall_window.popleft()
        if len(self._stall_window) >= 2:
            gain = self._stall_window[-1][1] - self._stall_window[0][1]
            if abs(vx) > 0.03 and gain < 0.05:
                if not self._stall_on:
                    self._stall_on = True
                    emit("PATH_PROGRESS_STALLED", "MOVE", f"gain={gain:.3f}m in 2s vx={vx:.2f}")
            elif self._stall_on and gain >= 0.05:
                self._stall_on = False
                emit("PATH_PROGRESS_RESUMED", "MOVE", f"gain={gain:.3f}m")

        # lateral growth while turning
        self._lat_hist.append(float(sample.get("lateral_error") or 0.0))
        if abs(w) > 0.08 and len(self._lat_hist) >= 5:
            if self._lat_hist[-1] > self._lat_hist[0] + 0.08:
                emit(
                    "TURNING_BUT_TRACKING_ERROR_GROWING",
                    "ERROR",
                    f"lat {self._lat_hist[0]:.2f}→{self._lat_hist[-1]:.2f}",
                    interval=1.0,
                )

        # collision mismatch
        if sample.get("collision") and front > 1.5:
            emit("CLEARANCE_COLLISION_MISMATCH", "ERROR", f"collision=true front={front:.2f}", interval=1.0)

        # unexplained zero
        if abs(safe_vx) <= 0.01 and stop == "NONE" and phase == "forward" and abs(mppi_vx) > 0.05:
            emit("UNEXPLAINED_ZERO_SPEED", "ERROR", "safe=0 stop=NONE", interval=1.0)

        # recovery oscillation
        recent_rev = [t for t in self._rev_count_window if now - t < 15.0]
        if len(recent_rev) >= 3:
            emit("RECOVERY_OSCILLATION", "ERROR", f"reverse×{len(recent_rev)} in 15s", interval=2.0)

        self._prev = sample

    def note_replan(self, now: float, events: EventRing, session_id: str) -> None:
        self._replan_times.append(now)
        events.push(
            "REPLAN_START",
            {"session_id": session_id, "message": "replan", "category": "REPLAN", "type": "REPLAN"},
            min_interval_s=0.0,
            category="REPLAN",
        )
        recent = [t for t in self._replan_times if now - t < 10.0]
        if len(recent) > 4:
            events.push(
                "REPLAN_LOOP",
                {"session_id": session_id, "message": f"replan×{len(recent)}/10s", "category": "ERROR"},
                min_interval_s=2.0,
                category="ERROR",
            )


class NavDebugHub:
    """Owns rings + session + black-box diagnostics."""

    def __init__(self) -> None:
        self.events = EventRing(EVENT_MAX)
        self.telemetry = TelemetryRing(TELEM_MAX)
        self.pose_trace: Deque[Dict[str, Any]] = deque(maxlen=POSE_MAX)
        self.detector = ExecutionEventDetector()
        self.incident = IncidentRecorder()
        self.approach = ApproachAnalyzer()
        self.debug_level = "ADVANCED"
        self.freeze = False
        self.frozen_snapshot: Optional[Dict[str, Any]] = None
        self.last_capture: Optional[Dict[str, Any]] = None
        self.prev_candidate_id: Optional[int] = None
        self.candidate_switch_count = 0
        self.candidate_switch_log: List[Dict[str, Any]] = []
        self.prev_best_cost: float = 0.0
        self.prev_cand_vx = 0.0
        self.prev_cand_w = 0.0
        self.reverse_decisions: List[Dict[str, Any]] = []
        self.recovery_attempts_full: List[Dict[str, Any]] = []
        self._open_recovery: Optional[Dict[str, Any]] = None
        self._fwd_dist = 0.0
        self._rev_dist = 0.0
        self._min_actual_clr = 99.0
        self._min_path_clr = 99.0
        self._max_abs_w = 0.0
        self.prev_phase = "forward"
        self.phase_enter_ts = time.time()
        self.last_recovery_action = ""
        self._last_progress_s = 0.0
        self._path_delta = 0.0
        self._w_sign_hist: Deque[int] = deque(maxlen=20)
        self._prev_vx = 0.0
        self._prev_w = 0.0
        self._prev_t: Optional[float] = None
        self.session_id = ""
        self.session_start: Optional[float] = None
        self.session_end: Optional[float] = None
        self._stats: Dict[str, Any] = {}
        self._recovery_log: List[Dict[str, Any]] = []
        self._exec_state = "IDLE"
        self._exec_state_enter = time.time()
        self._exec_history: List[Dict[str, Any]] = []
        self._max_ax = 0.0
        self._min_ax = 0.0
        self._max_alpha = 0.0
        self._min_alpha = 0.0
        self._max_vx = 0.0
        self._min_vx = 0.0
        self._max_lat = 0.0
        self._max_herr = 0.0
        self._obstacle_events = 0
        self._safety_stops = 0
        self._actual_dist = 0.0
        self._last_pose: Optional[Pt] = None
        self._post_mortem: Optional[Dict[str, Any]] = None
        self._run_summary: Optional[Dict[str, Any]] = None
        self.events.on_push = self._on_event_push

    def _on_event_push(self, event: str, row: Dict[str, Any]) -> None:
        if event in INCIDENT_TRIGGERS:
            self.incident.note_event(event, row)

    def begin_session(self, goal: Optional[Pt] = None) -> str:
        now = time.time()
        self.reset_session(keep_level=True)
        self._run_summary = None
        self._post_mortem = None
        self.incident.clear_session()
        self.approach.reset()
        self.candidate_switch_log = []
        self.reverse_decisions = []
        self.recovery_attempts_full = []
        self._open_recovery = None
        self._fwd_dist = self._rev_dist = 0.0
        self._min_actual_clr = 99.0
        self._min_path_clr = 99.0
        self._max_abs_w = 0.0
        self.session_id = "NAV-" + datetime.now().strftime("%Y%m%d-%H%M%S")
        self.session_start = now
        self.session_end = None
        self._stats = {"goal": {"x": goal[0], "y": goal[1]} if goal else None}
        self.events.push(
            "PLAN_STARTED",
            {"session_id": self.session_id, "message": "session start", "category": "PLAN", "goal": self._stats["goal"]},
            min_interval_s=0.0,
            category="PLAN",
        )
        self._set_exec_state("PLAN", now)
        return self.session_id

    def end_session(self, final_reason: str = "") -> None:
        now = time.time()
        self.session_end = now
        failedish = final_reason in ("FAILED", "RECOVERY_EXHAUSTED", "UNEXPLAINED_ZERO_SPEED", "STOP") or (
            "EXHAUST" in (final_reason or "").upper() or "FAIL" in (final_reason or "").upper()
        )
        if failedish:
            self._post_mortem = self._build_post_mortem(final_reason)
            self.events.push(
                "RECOVERY_EXHAUSTED" if "RECOVERY" in final_reason or final_reason == "FAILED" else final_reason,
                {"session_id": self.session_id, "message": final_reason, "category": "ERROR"},
                min_interval_s=0.0,
                category="ERROR",
            )
        self._run_summary = self._build_run_summary(final_reason)

    def reset_session(self, keep_level: bool = False) -> None:
        lv = self.debug_level if keep_level else self.debug_level
        keep_summary = self._run_summary
        keep_post = self._post_mortem
        keep_cap = self.last_capture
        self.events.clear()
        self.telemetry.clear()
        self.pose_trace.clear()
        self.detector.reset()
        self.approach.reset()
        self.incident.reset()
        self.prev_candidate_id = None
        self.candidate_switch_count = 0
        self.prev_best_cost = 0.0
        self.prev_cand_vx = 0.0
        self.prev_cand_w = 0.0
        self.last_recovery_action = ""
        self._last_progress_s = 0.0
        self._path_delta = 0.0
        self._w_sign_hist.clear()
        self._prev_vx = 0.0
        self._prev_w = 0.0
        self._prev_t = None
        self.freeze = False
        self.frozen_snapshot = None
        self.prev_phase = "forward"
        self.phase_enter_ts = time.time()
        self._recovery_log = []
        self._exec_state = "IDLE"
        self._exec_state_enter = time.time()
        self._exec_history = []
        self._max_ax = self._min_ax = 0.0
        self._max_alpha = self._min_alpha = 0.0
        self._max_vx = self._min_vx = 0.0
        self._max_lat = self._max_herr = 0.0
        self._obstacle_events = 0
        self._safety_stops = 0
        self._actual_dist = 0.0
        self._last_pose = None
        # keep last run for post-mortem UI until next begin_session
        self._run_summary = keep_summary
        self._post_mortem = keep_post
        self.last_capture = keep_cap
        self.debug_level = lv

    def _set_exec_state(self, st: str, now: float) -> None:
        if st == self._exec_state:
            return
        dur = now - self._exec_state_enter
        self._exec_history.append({"state": self._exec_state, "enter": self._exec_state_enter, "duration_s": round(dur, 2)})
        if len(self._exec_history) > 40:
            self._exec_history = self._exec_history[-40:]
        self._exec_state = st
        self._exec_state_enter = now

    def note_candidate(
        self,
        cand_id: Optional[int],
        best_cost: float,
        *,
        vx: float = 0.0,
        w: float = 0.0,
        reason: str = "",
    ) -> None:
        if cand_id is None:
            return
        if self.prev_candidate_id is not None and cand_id != self.prev_candidate_id:
            self.candidate_switch_count += 1
            row = {
                "ts": time.time(),
                "session_id": self.session_id,
                "previous_candidate": self.prev_candidate_id,
                "new_candidate": cand_id,
                "previous_vx": self.prev_cand_vx,
                "new_vx": vx,
                "previous_w": self.prev_cand_w,
                "new_w": w,
                "previous_cost": self.prev_best_cost,
                "new_cost": best_cost,
                "reason": reason or "mppi_reselect",
            }
            self.candidate_switch_log.append(row)
            if len(self.candidate_switch_log) > 80:
                self.candidate_switch_log = self.candidate_switch_log[-80:]
            self.events.push(
                "CANDIDATE_SWITCH",
                {
                    "session_id": self.session_id,
                    "from": self.prev_candidate_id,
                    "to": cand_id,
                    "best_cost": best_cost,
                    "prev_best_cost": self.prev_best_cost,
                    "previous_vx": self.prev_cand_vx,
                    "new_vx": vx,
                    "previous_w": self.prev_cand_w,
                    "new_w": w,
                    "category": "MOVE",
                    "message": f"#{self.prev_candidate_id}→#{cand_id}",
                },
                min_interval_s=0.0,
                category="MOVE",
            )
        self.prev_candidate_id = cand_id
        self.prev_best_cost = best_cost
        self.prev_cand_vx = vx
        self.prev_cand_w = w

    def note_phase(self, phase: str, snapshot: Optional[Dict[str, Any]] = None) -> None:
        now = time.time()
        snap = snapshot or {}
        if phase != self.prev_phase:
            self.events.push(
                "PHASE_CHANGE",
                {"session_id": self.session_id, "from": self.prev_phase, "to": phase, "category": "RECOVERY", "message": f"{self.prev_phase}→{phase}"},
                min_interval_s=0.0,
                category="RECOVERY",
            )
            if phase in ("reverse_escape", "recover", "safe_stop"):
                self.last_recovery_action = phase
                self.events.push(
                    "RECOVERY_START",
                    {"session_id": self.session_id, "phase": phase, "category": "RECOVERY", "message": phase},
                    min_interval_s=0.0,
                    category="RECOVERY",
                )
                self.incident.note_event("RECOVERY_START", {"phase": phase})
                fwd_ok = str(snap.get("forward_trajectory") or "") == "AVAILABLE"
                decision = {
                    "ts": now,
                    "reason": "RECOVERY",
                    "phase_before": self.prev_phase,
                    "phase_after": phase,
                    "forward_feasible": fwd_ok,
                    "forward_candidate_count": snap.get("forward_candidate_count"),
                    "best_forward_cost": snap.get("best_forward_cost"),
                    "best_forward_vx": snap.get("best_forward_vx"),
                    "best_forward_w": snap.get("best_forward_w"),
                    "forward_collision": snap.get("forward_collision"),
                    "forward_clearance": snap.get("forward_clearance"),
                    "reverse_feasible": snap.get("reverse_feasible"),
                    "reverse_candidate_count": snap.get("reverse_candidate_count"),
                    "best_reverse_cost": snap.get("best_reverse_cost"),
                    "best_reverse_vx": snap.get("best_reverse_vx"),
                    "best_reverse_w": snap.get("best_reverse_w"),
                    "reverse_collision": snap.get("reverse_collision"),
                    "reverse_clearance": snap.get("reverse_clearance"),
                    "front": snap.get("front_near"),
                    "rear": snap.get("rear_near"),
                    "path_progress": snap.get("path_progress_s"),
                    "stuck_s": snap.get("stuck_s"),
                }
                if fwd_ok:
                    decision["anomaly"] = "REVERSE_WHILE_FORWARD_AVAILABLE"
                    self.events.push(
                        "REVERSE_WITH_FORWARD_AVAILABLE",
                        {**decision, "category": "ERROR", "message": "reverse while forward AVAILABLE"},
                        min_interval_s=0.5,
                        category="ERROR",
                    )
                    self.incident.note_event("REVERSE_WITH_FORWARD_AVAILABLE", decision)
                self.reverse_decisions.append(decision)
                if len(self.reverse_decisions) > 20:
                    self.reverse_decisions = self.reverse_decisions[-20:]
                attempt = {
                    "attempt_id": len(self.recovery_attempts_full) + 1,
                    "start_time": now,
                    "end_time": None,
                    "trigger": "PHASE",
                    "phase_before": self.prev_phase,
                    "action": phase,
                    "pose_before": {"x": snap.get("x"), "y": snap.get("y"), "theta": snap.get("theta")},
                    "pose_after": None,
                    "path_progress_before": snap.get("path_progress_s"),
                    "path_progress_after": None,
                    "front_before": snap.get("front_near"),
                    "front_after": None,
                    "rear_before": snap.get("rear_near"),
                    "rear_after": None,
                    "path_clearance_before": snap.get("path_clearance"),
                    "path_clearance_after": None,
                    "actual_clearance_before": snap.get("actual_clearance"),
                    "actual_clearance_after": None,
                    "vx_before": snap.get("state_vx"),
                    "vx_after": None,
                    "result": "OPEN",
                }
                self._open_recovery = attempt
                self._recovery_log.append(
                    {
                        "attempt": attempt["attempt_id"],
                        "trigger": "PHASE",
                        "action": phase,
                        "ts": now,
                        "result": "ENTERED",
                    }
                )
                self._set_exec_state("RECOVERY" if phase != "reverse_escape" else "REVERSE", now)
            if self.prev_phase in ("reverse_escape", "recover") and phase == "forward":
                self.events.push(
                    "RECOVERY_END",
                    {"session_id": self.session_id, "phase": phase, "category": "RECOVERY", "message": "back to forward"},
                    min_interval_s=0.0,
                    category="RECOVERY",
                )
                if self._open_recovery:
                    att = self._open_recovery
                    att["end_time"] = now
                    att["pose_after"] = {"x": snap.get("x"), "y": snap.get("y"), "theta": snap.get("theta")}
                    att["path_progress_after"] = snap.get("path_progress_s")
                    att["front_after"] = snap.get("front_near")
                    att["rear_after"] = snap.get("rear_near")
                    att["path_clearance_after"] = snap.get("path_clearance")
                    att["actual_clearance_after"] = snap.get("actual_clearance")
                    att["vx_after"] = snap.get("state_vx")
                    pb = float(att.get("path_progress_before") or 0)
                    pa = float(att.get("path_progress_after") or 0)
                    cb = float(att.get("actual_clearance_before") or att.get("front_before") or 0)
                    ca = float(att.get("actual_clearance_after") or att.get("front_after") or 0)
                    if pa > pb + 0.05 or ca > cb + 0.05:
                        att["result"] = "IMPROVED"
                    elif pa < pb - 0.02 or ca < cb - 0.08:
                        att["result"] = "WORSE"
                        self.events.push(
                            "RECOVERY_MADE_GEOMETRY_WORSE",
                            {"attempt_id": att["attempt_id"], "category": "ERROR", "message": "recovery worsened geometry"},
                            min_interval_s=0.5,
                            category="ERROR",
                        )
                    else:
                        att["result"] = "NO_CHANGE"
                    self.recovery_attempts_full.append(att)
                    self._open_recovery = None
                self._set_exec_state("FORWARD", now)
            self.prev_phase = phase
            self.phase_enter_ts = now

    def note_stop_reason(self, reason: str, payload: Dict[str, Any]) -> None:
        if reason and reason != "NONE":
            cat = "SAFETY" if reason in ("FRONT_OBSTACLE", "REAR_OBSTACLE", "COLLISION_GUARD", "EMERGENCY") else "STOP"
            pl = {"session_id": self.session_id, "category": cat, "message": reason, **payload}
            self.events.push(reason, pl, min_interval_s=0.4, category=cat)
            if reason in ("FRONT_OBSTACLE", "REAR_OBSTACLE", "COLLISION_GUARD"):
                self._safety_stops += 1
                self._set_exec_state("OBSTACLE" if "OBSTACLE" in reason else "STOP", time.time())
            if reason == "FAILED":
                self.events.push("FAILED", pl, min_interval_s=0.0, category="ERROR")
                self.end_session("FAILED")

    def note_replan(self) -> None:
        self.detector.note_replan(time.time(), self.events, self.session_id)
        self._set_exec_state("REPLAN", time.time())

    def ingest_sample(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        """Compute ax/alpha, push telem+pose, detect events. Returns enriched sample dict."""
        now = float(raw.get("timestamp") or time.time())
        state_vx = float(raw.get("state_vx") or 0.0)
        state_w = float(raw.get("state_w") or 0.0)
        dt = 0.05
        if self._prev_t is not None:
            dt = max(1e-3, now - self._prev_t)
        ax = (state_vx - self._prev_vx) / dt
        alpha = (state_w - self._prev_w) / dt
        self._prev_vx, self._prev_w, self._prev_t = state_vx, state_w, now

        ps = float(raw.get("path_progress_s") or 0.0)
        rate = (ps - self._last_progress_s) / dt if self._last_progress_s or ps else 0.0
        # smooth-ish for display
        if self.telemetry.last:
            rate = 0.6 * float(self.telemetry.last.get("path_progress_rate") or 0.0) + 0.4 * rate
        self._path_delta = ps - self._last_progress_s
        self._last_progress_s = ps

        motion = classify_motion_state(
            state_vx=state_vx,
            state_w=state_w,
            ax=ax,
            phase=str(raw.get("phase") or "forward"),
            stop_reason=str(raw.get("stop_reason") or "NONE"),
        )
        if motion == "FORWARD":
            self._set_exec_state("FORWARD", now)
        elif motion == "TURNING":
            self._set_exec_state("TURN", now)
        elif motion == "DECELERATING":
            self._set_exec_state("DECEL", now)
        elif motion == "ACCELERATING":
            self._set_exec_state("FORWARD", now)

        sample = {
            **raw,
            "timestamp": now,
            "session_id": self.session_id or raw.get("session_id") or "",
            "ax": round(ax, 4),
            "alpha": round(alpha, 4),
            "path_progress_rate": round(rate, 4),
            "motion_state": motion,
            "cmd_vx": float(raw.get("cmd_vx", raw.get("mppi_vx", 0.0)) or 0.0),
            "cmd_w": float(raw.get("cmd_w", raw.get("mppi_w", 0.0)) or 0.0),
            "desired_w": float(raw.get("desired_w", raw.get("mppi_w", 0.0)) or 0.0),
            "pp_w": float(raw.get("pp_w") or 0.0),
            "selected_candidate": raw.get("selected_candidate"),
            "actual_clearance": float(
                raw.get("actual_clearance")
                or raw.get("nearest_obstacle_distance")
                or raw.get("front_near")
                or 0.0
            ),
            "path_clearance": float(raw.get("path_clearance") or 0.0),
            "nearest_obstacle_distance": float(
                raw.get("nearest_obstacle_distance") or raw.get("actual_clearance") or raw.get("front_near") or 0.0
            ),
            "nearest_obstacle_direction": float(raw.get("nearest_obstacle_direction") or 0.0),
        }
        self.detector.update(sample, self.events)
        meta = self.approach.update(sample, self.events, self.session_start)
        sample.update(meta)
        sample["clearance_rate"] = meta.get("clearance_rate", 0.0)
        sample["lateral_error_rate"] = meta.get("lateral_error_rate", 0.0)
        sample["heading_error_rate"] = meta.get("heading_error_rate", 0.0)
        sample["steering_execution_ratio"] = meta.get("steering_execution_ratio", 0.0)

        self.telemetry.push(sample)
        self.incident.push_sample(sample)
        x, y = float(raw.get("x") or 0.0), float(raw.get("y") or 0.0)
        if self._last_pose is not None:
            step = ((x - self._last_pose[0]) ** 2 + (y - self._last_pose[1]) ** 2) ** 0.5
            self._actual_dist += step
            if state_vx >= 0:
                self._fwd_dist += step
            else:
                self._rev_dist += step
        self._last_pose = (x, y)
        self._min_actual_clr = min(self._min_actual_clr, float(sample.get("actual_clearance") or 99))
        pc = float(sample.get("path_clearance") or 99)
        if pc < 90:
            self._min_path_clr = min(self._min_path_clr, pc)
        self._max_abs_w = max(self._max_abs_w, abs(state_w))
        self.pose_trace.append(
            {
                "ts": now,
                "session_id": self.session_id,
                "x": x,
                "y": y,
                "theta": float(raw.get("theta") or 0.0),
                "vx": state_vx,
                "w": state_w,
            }
        )
        self._max_ax = max(self._max_ax, ax)
        self._min_ax = min(self._min_ax, ax)
        self._max_alpha = max(self._max_alpha, alpha)
        self._min_alpha = min(self._min_alpha, alpha)
        self._max_vx = max(self._max_vx, state_vx)
        self._min_vx = min(self._min_vx, state_vx)
        self._max_lat = max(self._max_lat, abs(float(raw.get("lateral_error") or 0.0)))
        self._max_herr = max(self._max_herr, abs(float(raw.get("heading_error") or 0.0)))

        sign = 0 if abs(state_w) < 0.03 else (1 if state_w > 0 else -1)
        self._w_sign_hist.append(sign)
        return sample

    def sample_telemetry(self, row: Dict[str, Any]) -> None:
        """Back-compat wrapper."""
        if "timestamp" not in row:
            row = {**row, "timestamp": row.get("ts", time.time())}
        self.ingest_sample(row)

    def w_oscillation_count(self) -> int:
        n = 0
        prev = 0
        for s in self._w_sign_hist:
            if s != 0 and prev != 0 and s != prev:
                n += 1
            if s != 0:
                prev = s
        return n

    def _build_run_summary(self, final_reason: str) -> Dict[str, Any]:
        dur = 0.0
        if self.session_start:
            dur = (self.session_end or time.time()) - self.session_start
        last = self.telemetry.last or {}
        evs = self.events.list(EVENT_MAX)
        return {
            "session_id": self.session_id,
            "duration_s": round(dur, 2),
            "distance_m": round(self._actual_dist, 3),
            "forward_distance_m": round(self._fwd_dist, 3),
            "reverse_distance_m": round(self._rev_dist, 3),
            "path_progress_m": round(float(last.get("path_progress_s") or 0.0), 3),
            "max_vx": round(self._max_vx, 3),
            "max_reverse": round(self._min_vx, 3),
            "max_abs_w": round(self._max_abs_w, 3),
            "max_lateral_error": round(self._max_lat, 3),
            "max_heading_error": round(self._max_herr, 3),
            "min_actual_clearance": round(self._min_actual_clr if self._min_actual_clr < 90 else 0.0, 3),
            "min_path_clearance": round(self._min_path_clr if self._min_path_clr < 90 else 0.0, 3),
            "obstacle_events": sum(1 for e in evs if e.get("event") == "OBSTACLE_DETECTED"),
            "turn_events": sum(1 for e in evs if e.get("event") == "TURN_START"),
            "candidate_switches": self.candidate_switch_count,
            "recovery_attempts": int(last.get("recovery_attempts") or 0),
            "replan_count": sum(1 for e in evs if e.get("event") == "REPLAN_START"),
            "safety_stops": self._safety_stops,
            "final": final_reason or last.get("stop_reason") or "NONE",
            "primary_diagnosis": (self._post_mortem or {}).get("primary")
            or (self.approach.first_divergence and "TRACKING_DIVERGENCE")
            or final_reason,
            "secondary_diagnosis": [
                e.get("event")
                for e in evs
                if e.get("event")
                in (
                    "RECOVERY_MADE_GEOMETRY_WORSE",
                    "REVERSE_WITH_FORWARD_AVAILABLE",
                    "ANGULAR_OSCILLATION",
                    "APPROACHED_OBSTACLE_THEN_REVERSE",
                )
            ][-3:],
            "first_divergence": self.approach.first_divergence,
        }

    def _build_post_mortem(self, final_reason: str) -> Dict[str, Any]:
        now = time.time()
        telem = [r for r in self.telemetry.list(TELEM_MAX) if now - float(r.get("timestamp") or 0) <= 12.0]
        evs = [e for e in self.events.list(EVENT_MAX) if now - float(e.get("ts") or 0) <= 12.0]
        first = None
        for e in evs:
            if e.get("event") in (
                "SAFETY_BLOCK",
                "OBSTACLE_DETECTED",
                "PATH_PROGRESS_STALLED",
                "MPPI_NO_FORWARD_TRAJECTORY",
                "CLEARANCE_COLLISION_MISMATCH",
                "UNEXPLAINED_ZERO_SPEED",
                "REVERSE_START",
                "RECOVERY_START",
            ):
                first = e
                break
        return {
            "session_id": self.session_id,
            "final": final_reason,
            "primary": (first or {}).get("event") or final_reason,
            "first_anomaly": first,
            "window_s": 10,
            "telemetry_tail": telem[-80:],
            "events_tail": evs[-40:],
        }

    def build(
        self,
        *,
        nav_mode: str,
        phase: str,
        control_mode: str,
        stop_reason: str,
        x: float,
        y: float,
        yaw: float,
        goal_xy: Optional[Pt],
        goal_distance: float,
        path_progress_s: float,
        lateral_err: float,
        heading_err: float,
        stuck_s: float,
        planning: Dict[str, Any],
        raw_path: Sequence[Pt],
        processed_path: Sequence[Pt],
        executed_path: Sequence[Pt],
        planned_path: Sequence[Pt],
        candidates: List[Dict[str, Any]],
        mppi_vx: float,
        mppi_w: float,
        pp_w: float,
        safe_vx: float,
        safe_w: float,
        final_vx: float,
        final_w: float,
        state_vx: float,
        state_w: float,
        front_near: float,
        rear_near: float,
        collision: bool,
        emergency: bool,
        recovery_attempts: int,
        global_replan_count: int,
        local_replan_count: int,
        confidence: float,
        selected_candidate: Optional[int],
        best_cost: float,
        scene: Dict[str, Any],
        lookahead_pt: Optional[Pt] = None,
        first_collision: Optional[Dict[str, Any]] = None,
        mppi_meta: Optional[Dict[str, Any]] = None,
        cmd_vx: Optional[float] = None,
        cmd_w: Optional[float] = None,
        maneuver: Optional[Dict[str, Any]] = None,
        cmd_source: Optional[str] = None,
    ) -> Dict[str, Any]:
        geom = DEFAULT_GEOM
        cands = list(candidates or [])
        man = dict(maneuver or {})
        # Prefer Maneuver-layer forward feasibility when present
        man_fwd_ok = man.get("forward_feasible")
        man_fwd_reason = man.get("forward_reason")
        stopish = None
        best_fwd = None
        best_rev = None
        for c in cands:
            row = {
                "id": c.get("id"),
                "vx": float(c.get("vx", 0.0) or 0.0),
                "w": float(c.get("w", 0.0) or 0.0),
                "cost": float(c.get("cost") or 0.0),
                "collision": bool(c.get("collision", False)),
                "mode": c.get("mode"),
            }
            if row["vx"] > 0.04:
                if best_fwd is None or row["cost"] < best_fwd["cost"]:
                    best_fwd = row
            if row["vx"] < -0.04:
                if best_rev is None or row["cost"] < best_rev["cost"]:
                    best_rev = row
            if abs(row["vx"]) <= 0.04:
                if stopish is None or row["cost"] < stopish["cost"]:
                    stopish = row
        forward_available = (
            bool(man_fwd_ok)
            if man_fwd_ok is not None
            else (
                any(float(c.get("vx") or 0.0) > 0.04 and not c.get("collision") for c in cands)
                if any("vx" in c for c in cands)
                else any(c.get("mode") == "forward" and not c.get("collision") for c in cands)
            )
        )
        forward_reason = man_fwd_reason or ("OK" if forward_available else "NOT_AVAILABLE")

        last = self.telemetry.last or {}
        ax = float(last.get("ax") or 0.0)
        alpha = float(last.get("alpha") or 0.0)
        rate = float(last.get("path_progress_rate") or 0.0)
        motion = str(last.get("motion_state") or classify_motion_state(
            state_vx=state_vx, state_w=state_w, ax=ax, phase=phase, stop_reason=stop_reason
        ))
        cmd_vx_v = float(cmd_vx if cmd_vx is not None else mppi_vx)
        cmd_w_v = float(cmd_w if cmd_w is not None else mppi_w)

        diag = diagnose_why_not_moving(
            mppi_vx=mppi_vx,
            safe_vx=safe_vx,
            final_vx=final_vx,
            state_vx=state_vx,
            front_near=front_near,
            front_stop=geom.front_stop_m,
            collision=collision,
            emergency=emergency,
            phase=phase,
            stop_reason=stop_reason,
            path_progress_s=path_progress_s,
            path_progress_delta=self._path_delta,
            recovery_attempts=recovery_attempts,
            max_recovery=MAX_RECOVERY_ATTEMPTS,
            forward_candidate_available=forward_available,
            best_forward=best_fwd,
            best_stop=stopish,
            path_progress_rate=rate,
            has_global_path=bool(processed_path),
        )
        pq = diagnose_path_quality(planning or {})
        tracking = diagnose_tracking(
            heading_err=heading_err,
            lateral_err=lateral_err,
            w_osc=self.w_oscillation_count(),
            path_delta=self._path_delta,
            state_vx=state_vx,
        )
        safety = safety_gate_trace(
            mppi_vx=mppi_vx,
            mppi_w=mppi_w,
            safe_vx=safe_vx,
            safe_w=safe_w,
            front_near=front_near,
            rear_near=rear_near,
            front_stop=geom.front_stop_m,
            rear_stop=geom.rear_stop_m,
            collision=collision,
            emergency=emergency,
            phase=phase,
            stop_reason=stop_reason,
        )
        selected = None
        for c in cands:
            if selected_candidate is not None and int(c.get("id") or -1) == int(selected_candidate):
                selected = c
                break
        if selected is None and cands:
            selected = cands[0]

        planned_len = float((planning or {}).get("final_path_length") or 0.0)
        level = self.debug_level
        telem = self.telemetry.list(200 if level == "FULL" else 120)
        pose = list(self.pose_trace)[-200:]
        map_events = [
            {
                "ts": e.get("ts"),
                "event": e.get("event"),
                "type": e.get("category") or e.get("type"),
                "message": e.get("message") or e.get("event"),
                "x": None,
                "y": None,
            }
            for e in self.events.list(60)
            if e.get("category") in ("ACCEL", "DECEL", "TURN", "OBSTACLE", "SAFETY", "REVERSE", "REPLAN", "STOP", "ERROR", "RECOVERY")
            or e.get("event")
            in (
                "ACCELERATION_START",
                "DECELERATION_START",
                "TURN_START",
                "OBSTACLE_DETECTED",
                "SAFETY_BLOCK",
                "REVERSE_START",
                "REPLAN_START",
                "RECOVERY_START",
                "FAILED",
            )
        ]
        # attach nearest pose for map markers
        for me in map_events:
            ts = float(me.get("ts") or 0)
            best = None
            best_d = 1e9
            for p in pose:
                d = abs(float(p.get("ts") or 0) - ts)
                if d < best_d:
                    best_d = d
                    best = p
            if best and best_d < 0.5:
                me["x"], me["y"] = best.get("x"), best.get("y")

        out: Dict[str, Any] = {
            "level": level,
            "freeze": self.freeze,
            "updated_at": time.time(),
            "session_id": self.session_id,
            "session_start": self.session_start,
            "session_end": self.session_end,
            "status": {
                "nav_mode": nav_mode,
                "phase": phase,
                "control_mode": control_mode,
                "stop_reason": stop_reason,
                "vx": round(state_vx, 4),
                "w": round(state_w, 4),
                "goal_distance": round(goal_distance, 3),
                "path_progress_s": round(path_progress_s, 3),
                "stuck_s": round(stuck_s, 2),
                "recovery_attempts": recovery_attempts,
                "max_recovery_attempts": MAX_RECOVERY_ATTEMPTS,
                "motion_state": motion,
            },
            "motion": {
                "motion_state": motion,
                "phase": phase,
                "control_mode": control_mode,
                "vx": round(state_vx, 4),
                "w": round(state_w, 4),
                "ax": round(ax, 4),
                "alpha": round(alpha, 4),
                "max_ax": round(self._max_ax, 4),
                "min_ax": round(self._min_ax, 4),
                "max_alpha": round(self._max_alpha, 4),
                "min_alpha": round(self._min_alpha, 4),
            },
            "velocity_chain": {
                "mppi_vx": round(mppi_vx, 4),
                "cmd_vx": round(cmd_vx_v, 4),
                "safe_vx": round(safe_vx, 4),
                "state_vx": round(state_vx, 4),
                "note": diag.get("velocity_chain_note"),
            },
            "diagnostics": diag,
            "paths": {
                "raw_global": [{"x": p[0], "y": p[1]} for p in list(raw_path)[:200]],
                "processed_global": [{"x": p[0], "y": p[1]} for p in list(processed_path)[:200]],
                "planned_local": [{"x": p[0], "y": p[1]} for p in list(planned_path)[:80]],
                "executed": [{"x": p[0], "y": p[1]} for p in list(executed_path)[:80]],
                "actual_trace": pose,
                "blue_band_means": "executed kinematic band (after safety)",
                "n_raw": len(raw_path),
                "n_processed": len(processed_path),
                "n_executed": len(executed_path),
                "planned_length": planned_len,
                "actual_distance": round(self._actual_dist, 3),
                "path_progress": round(path_progress_s, 3),
                "tracking_deviation": round(abs(self._actual_dist - path_progress_s), 3) if path_progress_s else round(self._actual_dist, 3),
            },
            "geometry": {
                "length": geom.length,
                "width": geom.width,
                "bumper_l": geom.bumper_l,
                "planner_radius": geom.planner_radius,
                "local_radius": geom.local_radius,
                "safety_radius": geom.safety_radius,
                "front_stop_m": geom.front_stop_m,
                "rear_stop_m": geom.rear_stop_m,
                "front_cost_m": geom.front_cost_m,
                "requested_inflate_m": scene.get("requested_inflate_m"),
                "actual_inflate_m": scene.get("actual_inflate_m"),
                "plan_res": scene.get("plan_res"),
            },
            "radar": {
                "front_near": round(front_near, 3),
                "rear_near": round(rear_near, 3),
                "front_metric": "second_nearest_in_sector",
                "rear_metric": "second_nearest_in_sector",
                "sector_half_deg": 22.0,
                "front_stop_m": geom.front_stop_m,
                "rear_stop_m": geom.rear_stop_m,
                "collision": collision,
                "clearance_collision_mismatch": bool(collision and front_near > 1.5),
            },
            "execution_state": {
                "current": self._exec_state,
                "enter_ts": self._exec_state_enter,
                "time_in_state_s": round(time.time() - self._exec_state_enter, 2),
                "history": self._exec_history[-12:],
            },
            "map_events": map_events[-40:],
            "run_summary": self._run_summary,
            "post_mortem": self._post_mortem,
            "recovery_log": self._recovery_log[-5:],
            "incident": self.incident.latest(),
            "incidents": [
                {"trigger": i.get("trigger"), "t0": i.get("t0"), "summary": i.get("summary"), "keyframes": i.get("keyframes")}
                for i in self.incident.incidents[-3:]
            ],
            "wall_approach": {
                "nearest_obstacle_distance": last.get("nearest_obstacle_distance"),
                "nearest_obstacle_direction": last.get("nearest_obstacle_direction"),
                "nearest_obstacle_point": last.get("nearest_obstacle_point"),
                "vehicle_to_obstacle_clearance": last.get("actual_clearance"),
                "path_clearance": last.get("path_clearance"),
                "clearance_rate": last.get("clearance_rate"),
                "actual_clearance": last.get("actual_clearance"),
                "approach_tag": last.get("approach_tag") or getattr(self.approach, "_approach_state", ""),
            },
            "tracking_divergence": {
                "first_divergence": self.approach.first_divergence,
                "lateral_error_rate": last.get("lateral_error_rate"),
                "heading_error_rate": last.get("heading_error_rate"),
            },
            "steering": {
                "pp_w": round(pp_w, 4),
                "mppi_w": round(mppi_w, 4),
                "desired_w": round(float(last.get("desired_w") or mppi_w), 4),
                "safe_w": round(safe_w, 4),
                "cmd_w": round(cmd_w_v, 4),
                "state_w": round(state_w, 4),
                "steering_execution_ratio": last.get("steering_execution_ratio"),
                "turn_flip_count": last.get("turn_flip_count"),
            },
            "path_vs_actual_clearance": {
                "planned_path_clearance": last.get("path_clearance"),
                "actual_vehicle_clearance": last.get("actual_clearance"),
            },
            "candidate_switch_log": self.candidate_switch_log[-12:],
            "reverse_decisions": self.reverse_decisions[-5:],
            "recovery_attempts": self.recovery_attempts_full[-5:],
            "anomalies": {
                "reverse_with_forward_available": phase == "reverse_escape" and forward_available,
                "divergence": self.approach.first_divergence,
            },
        }

        if level in ("BASIC", "ADVANCED", "FULL"):
            out["planner"] = {
                **(planning or {}),
                "algorithm": "A*",
                "status": "SUCCESS" if processed_path else "FAILED",
                "path_quality": pq,
                "start": {"x": x, "y": y},
                "goal": {"x": goal_xy[0], "y": goal_xy[1]} if goal_xy else None,
            }
            out["safety"] = {
                **safety,
                "front_near": round(front_near, 3),
                "rear_near": round(rear_near, 3),
                "collision": collision,
                "emergency": emergency,
                "obstacle_blocked": stop_reason in ("FRONT_OBSTACLE", "REAR_OBSTACLE", "COLLISION_GUARD"),
            }
            out["controller"] = {
                "pp_lookahead_m": 1.4,
                "pp_w": round(pp_w, 4),
                "mppi_w": round(mppi_w, 4),
                "mppi_dw": round(mppi_w - pp_w, 4),
                "final_vx": round(final_vx, 4),
                "final_w": round(final_w, 4),
                "state_vx": round(state_vx, 4),
                "state_w": round(state_w, 4),
                "heading_err": round(heading_err, 4),
                "lateral_err": round(lateral_err, 4),
                "path_progress_rate": round(rate, 4),
                "lookahead_point": {"x": lookahead_pt[0], "y": lookahead_pt[1]} if lookahead_pt else None,
                "tracking_quality": tracking,
                "acc_v": geom.acc_v,
                "acc_w": geom.acc_w,
            }
            out["recovery"] = {
                "phase": phase,
                "previous_phase": self.prev_phase,
                "phase_enter_ts": self.phase_enter_ts,
                "time_in_phase_s": round(time.time() - self.phase_enter_ts, 2),
                "recovery_attempts": recovery_attempts,
                "max_attempts": MAX_RECOVERY_ATTEMPTS,
                "cooldown_s": RECOVERY_COOLDOWN_S,
                "reverse_max_s": REVERSE_MAX_S,
                "stuck_s": round(stuck_s, 2),
                "last_recovery_action": self.last_recovery_action,
                "global_replan_count": global_replan_count,
                "local_replan_count": local_replan_count,
                "log": self._recovery_log[-3:],
            }

        if level in ("ADVANCED", "FULL"):
            meta = mppi_meta or {}
            out["local_planner"] = {
                "control_mode": control_mode,
                "mppi_vx": round(mppi_vx, 4),
                "mppi_w": round(mppi_w, 4),
                "best_cost": round(best_cost, 3),
                "previous_best_cost": round(self.prev_best_cost, 3),
                "confidence": confidence,
                "confidence_note": "cost normalization only (not safety probability)",
                "candidate_count": len(cands),
                "selected_candidate": selected_candidate,
                "previous_selected_candidate": self.prev_candidate_id,
                "candidate_switch_count": self.candidate_switch_count,
                "forward_trajectory": "AVAILABLE" if forward_available else "NOT_AVAILABLE",
                "forward_reason": forward_reason,
                "maneuver_required": man.get("maneuver_required"),
                "batch": meta.get("batch"),
                "time_steps": meta.get("time_steps"),
                "model_dt": meta.get("model_dt"),
                "horizon_s": meta.get("horizon_s"),
                "selected": selected,
                "cost_breakdown": (selected or {}).get("cost_breakdown"),
                "first_collision": first_collision or (selected or {}).get("first_collision"),
            }
            out["maneuver"] = {
                "mode": man.get("mode") or "IDLE",
                "reason": man.get("reason"),
                "target_heading": man.get("target_heading"),
                "heading_error": man.get("heading_error"),
                "capture": man.get("capture") or {},
                "rotation_safe": man.get("rotation_safe"),
                "turn_feasible": man.get("turn_feasible"),
                "left_free": man.get("left_free"),
                "right_free": man.get("right_free"),
                "front_free": man.get("front_free"),
                "rear_free": man.get("rear_free"),
                "turn_left_feasible": man.get("turn_left_feasible"),
                "turn_right_feasible": man.get("turn_right_feasible"),
                "pause_stuck": man.get("pause_stuck"),
                "history": man.get("history") or [],
                "max_heading_change_horizon": man.get("max_heading_change_horizon"),
                "cmd_source": cmd_source,
            }
            out["forward_reverse"] = {
                "forward_feasible": forward_available,
                "forward_reason": forward_reason,
                "best_forward_vx": (best_fwd or {}).get("vx") if best_fwd else man.get("forward_feasible"),
                "best_forward_w": (best_fwd or {}).get("w"),
                "best_forward_cost": (best_fwd or {}).get("cost"),
                "reverse_feasible": best_rev is not None,
                "best_reverse_vx": (best_rev or {}).get("vx"),
                "best_reverse_w": (best_rev or {}).get("w"),
                "best_reverse_cost": (best_rev or {}).get("cost"),
                "selected_maneuver": man.get("mode"),
                "reverse_before": man.get("reverse_before"),
                "reverse_after": man.get("reverse_after"),
            }
            out["path_capture"] = man.get("capture") or {"available": False}
            out["candidates"] = []
            for c in cands[:8]:
                out["candidates"].append(
                    {
                        "id": c.get("id"),
                        "label": c.get("label"),
                        "vx": c.get("vx"),
                        "w": c.get("w"),
                        "total_cost": c.get("cost"),
                        "cost_breakdown": c.get("cost_breakdown"),
                        "collision": c.get("collision"),
                        "first_collision": c.get("first_collision"),
                        "mode": c.get("mode"),
                        "confidence": c.get("confidence"),
                        "selected": selected_candidate is not None and int(c.get("id") or -1) == int(selected_candidate),
                        "path": c.get("path"),
                    }
                )

        out["events"] = self.events.list(80 if level == "FULL" else 40)
        out["telemetry"] = telem
        out["pose_trace"] = pose
        out["telem_ring_len"] = len(self.telemetry)
        out["pose_ring_len"] = len(self.pose_trace)
        # Always expose maneuver cards (even BASIC / empty)
        out["maneuver"] = {
            "mode": man.get("mode") or "IDLE",
            "reason": man.get("reason"),
            "target_heading": man.get("target_heading"),
            "heading_error": man.get("heading_error"),
            "capture": man.get("capture") or {},
            "rotation_safe": man.get("rotation_safe"),
            "turn_feasible": man.get("turn_feasible"),
            "left_free": man.get("left_free"),
            "right_free": man.get("right_free"),
            "front_free": man.get("front_free"),
            "rear_free": man.get("rear_free"),
            "turn_left_feasible": man.get("turn_left_feasible"),
            "turn_right_feasible": man.get("turn_right_feasible"),
            "pause_stuck": man.get("pause_stuck"),
            "history": man.get("history") or [],
            "max_heading_change_horizon": man.get("max_heading_change_horizon"),
            "cmd_source": cmd_source,
            "forward_feasible": man.get("forward_feasible"),
            "forward_reason": man.get("forward_reason") or forward_reason,
            "maneuver_required": man.get("maneuver_required"),
            "reverse_before": man.get("reverse_before"),
            "reverse_after": man.get("reverse_after"),
        }
        out["forward_reverse"] = {
            "forward_feasible": forward_available,
            "forward_reason": forward_reason,
            "best_forward_vx": (best_fwd or {}).get("vx") if best_fwd else None,
            "best_forward_w": (best_fwd or {}).get("w") if best_fwd else None,
            "best_forward_cost": (best_fwd or {}).get("cost") if best_fwd else None,
            "reverse_feasible": best_rev is not None,
            "best_reverse_vx": (best_rev or {}).get("vx") if best_rev else None,
            "best_reverse_w": (best_rev or {}).get("w") if best_rev else None,
            "best_reverse_cost": (best_rev or {}).get("cost") if best_rev else None,
            "selected_maneuver": man.get("mode"),
            "reverse_before": man.get("reverse_before"),
            "reverse_after": man.get("reverse_after"),
        }
        out["path_capture"] = man.get("capture") or {"available": False}
        # Local LEFT/RIGHT comparison (from ManeuverFSM.local_selector)
        lc = man.get("local_compare") or {}
        out["local_maneuver"] = {
            "decision": man.get("decision") or lc.get("selected") or "FORWARD",
            "reason": lc.get("reason") or man.get("reason"),
            "forward": next((r for r in (lc.get("rows") or []) if r.get("action") == "FORWARD"), {}),
            "left": next((r for r in (lc.get("rows") or []) if r.get("action") == "LEFT"), {}),
            "right": next((r for r in (lc.get("rows") or []) if r.get("action") == "RIGHT"), {}),
            "rows": lc.get("rows") or [],
            "left_path": lc.get("left_path") or [],
            "right_path": lc.get("right_path") or [],
            "forward_path": lc.get("forward_path") or [],
            "obstacle_passed": man.get("obstacle_passed"),
            "side_attempts": man.get("side_attempts"),
            "events": man.get("local_events") or [],
            "history": man.get("local_history") or [],
            "snapshot": lc.get("snapshot") or {},
        }
        return out
