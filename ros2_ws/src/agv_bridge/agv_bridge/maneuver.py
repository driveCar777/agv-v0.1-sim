"""Maneuver Decision Layer — explicit FSM for forward / turn / reverse.

Does NOT replace A* or Safety Supervisor. Constrains local planner action space.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_geometry import DEFAULT_GEOM, MAX_RECOVERY_ATTEMPTS, REVERSE_MAX_S

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]

# Heading bands (rad)
H_FORWARD = 0.45
H_TURN = 0.95
H_ALIGN = 1.15
ALIGN_TOL = 0.22
PATH_CAPTURE_TOL_M = 0.55

# Horizon / turn capability (match MPPI ~1.6s, wz_max~0.42)
HORIZON_S = 1.6
WZ_MAX = 0.42

# Timeouts
ALIGN_MAX_S = 6.0
TURN_MAX_S = 8.0
REPOSITION_MAX_S = 4.0
REVERSE_EVAL_S = 0.9  # early abort check
# STEP 3F-CORRECTIVE — recovery reverse must move, not spin
RECOVERY_MOVE_TIMEOUT_S = 3.5
RECOVERY_STALL_S = 1.25
RECOVERY_MIN_PROGRESS_M = 0.12
POLICY_REVERSE_VX = -0.12
POLICY_REVERSE_W = 0.0

# Modes
IDLE = "IDLE"
FORWARD_TRACK = "FORWARD_TRACK"
FORWARD_TURN = "FORWARD_TURN"
ALIGN = "ALIGN"
TURN_IN_PLACE = "TURN_IN_PLACE"
REPOSITION = "REPOSITION"
REVERSE_ESCAPE = "REVERSE_ESCAPE"
POST_TURN = "POST_TURN"
WAIT_FOR_CLEARANCE = "WAIT_FOR_CLEARANCE"
REPLAN = "REPLAN"
SAFE_STOP = "SAFE_STOP"
LOCAL_LEFT = "LOCAL_LEFT"
LOCAL_RIGHT = "LOCAL_RIGHT"

# Map legacy phase names
PHASE_MAP = {
    FORWARD_TRACK: "forward",
    FORWARD_TURN: "forward",
    LOCAL_LEFT: "local_avoid",
    LOCAL_RIGHT: "local_avoid",
    ALIGN: "align",
    TURN_IN_PLACE: "turn_in_place",
    REPOSITION: "reposition",
    REVERSE_ESCAPE: "reverse_escape",
    POST_TURN: "forward",
    WAIT_FOR_CLEARANCE: "forward",
    REPLAN: "recover",
    SAFE_STOP: "safe_stop",
    IDLE: "forward",
}


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def turning_radius_m(geom=DEFAULT_GEOM) -> float:
    return 0.5 * math.hypot(geom.length, geom.width)  # ~0.59m for AMB-150


def max_heading_change_in_horizon(wz_max: float = WZ_MAX, horizon_s: float = HORIZON_S) -> float:
    return abs(wz_max) * horizon_s


@dataclass
class PathCapture:
    available: bool
    point: Optional[Pt] = None
    heading: float = 0.0
    distance: float = 0.0
    heading_error: float = 0.0
    clearance: float = 0.0
    index: int = 0


@dataclass
class ForwardFeasibility:
    feasible: bool
    reason: str
    best_vx: float = 0.0
    best_w: float = 0.0
    best_cost: float = 0.0
    maneuver_required: str = "DIRECT_FORWARD"


@dataclass
class ReverseSnapshot:
    ts: float
    pose: Pt
    theta: float
    front_near: float
    rear_near: float
    path_progress: float
    lateral_error: float
    heading_error: float
    actual_clearance: float
    forward_feasible: bool
    forward_reason: str
    recovery_attempt: int
    phase_before: str


@dataclass
class ManeuverDecision:
    mode: str
    target_heading: float
    heading_error: float
    capture: PathCapture
    forward: ForwardFeasibility
    turn_feasible: bool
    rotation_safe: bool
    left_free: float
    right_free: float
    front_free: float
    rear_free: float
    turn_left_feasible: bool
    turn_right_feasible: bool
    reason: str
    vx_min: float
    vx_max: float
    allow_w: bool
    force_vx: Optional[float] = None
    force_w: Optional[float] = None
    pause_stuck: bool = False
    legacy_phase: str = "forward"
    decision: str = "FORWARD"
    local_compare: Optional[Dict[str, Any]] = None
    policy_state: str = ""
    policy_behavior: str = ""
    corridor_half_width: float = 0.3
    path_follow_weight: float = 5.0
    recovery_exec: Optional[Dict[str, Any]] = None


def find_best_path_capture(
    x: float,
    y: float,
    yaw: float,
    path: Sequence[Pt],
    *,
    clearance_at: Optional[Callable[[float, float], float]] = None,
    search_stride: int = 2,
) -> PathCapture:
    if not path or len(path) < 2:
        return PathCapture(available=False)
    best: Optional[Tuple[float, int, Pt, float, float, float]] = None
    n = len(path)
    for i in range(0, n - 1, max(1, search_stride)):
        px, py = path[i]
        qx, qy = path[min(i + 1, n - 1)]
        th = math.atan2(qy - py, qx - px)
        dist = math.hypot(px - x, py - y)
        herr = abs(wrap_pi(th - yaw))
        clr = float(clearance_at(px, py)) if clearance_at else 1.0
        # Prefer nearby points with moderate heading and good clearance
        cost = dist * 1.2 + herr * 0.85 + max(0.0, 0.45 - clr) * 3.0
        if best is None or cost < best[0]:
            best = (cost, i, (px, py), th, dist, clr)
    if best is None:
        return PathCapture(available=False)
    _, idx, pt, th, dist, clr = best
    return PathCapture(
        available=True,
        point=pt,
        heading=th,
        distance=dist,
        heading_error=wrap_pi(th - yaw),
        clearance=clr,
        index=idx,
    )


def assess_forward_feasibility(
    *,
    heading_error: float,
    front_near: float,
    front_stop: float,
    collision: bool,
    capture: PathCapture,
    candidates: Optional[List[Dict[str, Any]]] = None,
) -> ForwardFeasibility:
    if collision:
        return ForwardFeasibility(False, "COLLISION", maneuver_required="RECOVERY_REQUIRED")
    if front_near < front_stop:
        return ForwardFeasibility(False, "CLEARANCE_TOO_LOW", maneuver_required="RECOVERY_REQUIRED")

    max_dh = max_heading_change_in_horizon()
    abs_h = abs(heading_error)
    if abs_h > max_dh + 0.15:
        return ForwardFeasibility(
            False,
            "HEADING_UNACHIEVABLE",
            maneuver_required="ALIGN_REQUIRED",
        )
    if abs_h > H_ALIGN:
        return ForwardFeasibility(False, "CURRENT_HORIZON_CANNOT_CAPTURE_PATH", maneuver_required="ALIGN_REQUIRED")
    if not capture.available:
        return ForwardFeasibility(False, "PATH_CAPTURE_UNREACHABLE", maneuver_required="REPLAN")

    best_fwd = None
    if candidates:
        for c in candidates:
            vx = float(c.get("vx") or 0.0)
            if vx > 0.04 and not c.get("collision"):
                if best_fwd is None or float(c.get("cost") or 1e9) < float(best_fwd.get("cost") or 1e9):
                    best_fwd = c
        if candidates and best_fwd is None and any(float(c.get("vx") or 0) > 0.04 for c in candidates):
            return ForwardFeasibility(False, "COLLISION", maneuver_required="RECOVERY_REQUIRED")

    if abs_h <= H_FORWARD:
        req = "DIRECT_FORWARD"
        ok = True
        reason = "OK"
    elif abs_h <= H_TURN:
        req = "FORWARD_TURN"
        ok = True
        reason = "OK"
    else:
        req = "ALIGN_REQUIRED"
        ok = False
        reason = "HEADING_UNACHIEVABLE"

    return ForwardFeasibility(
        feasible=ok,
        reason=reason,
        best_vx=float((best_fwd or {}).get("vx") or 0.0),
        best_w=float((best_fwd or {}).get("w") or 0.0),
        best_cost=float((best_fwd or {}).get("cost") or 0.0),
        maneuver_required=req,
    )


def rotation_safe_at(
    x: float,
    y: float,
    *,
    clearance: float,
    collide: Optional[CollideFn] = None,
    geom=DEFAULT_GEOM,
) -> bool:
    r = turning_radius_m(geom) + 0.08
    if clearance < r:
        return False
    if collide is None:
        return True
    # sample ring around vehicle
    for k in range(8):
        a = (2 * math.pi * k) / 8
        px = x + math.cos(a) * r * 0.85
        py = y + math.sin(a) * r * 0.85
        if collide(px, py):
            return False
    return True


def sector_free(
    x: float,
    y: float,
    yaw: float,
    *,
    base_ang: float,
    half_width: float = 0.55,
    collide: Optional[CollideFn] = None,
    max_r: float = 2.5,
) -> float:
    """Approximate free distance in a body-relative sector."""
    if collide is None:
        return max_r
    ang = yaw + base_ang
    for d in [0.4, 0.8, 1.2, 1.6, 2.0, 2.5]:
        if d > max_r:
            break
        for off in (-half_width * 0.5, 0.0, half_width * 0.5):
            px = x + math.cos(ang) * d - math.sin(ang) * off
            py = y + math.sin(ang) * d + math.cos(ang) * off
            if collide(px, py):
                return max(0.0, d - 0.35)
    return max_r


def action_limits(mode: str) -> Tuple[float, float, bool]:
    """Returns vx_min, vx_max, allow_nonzero_w."""
    if mode in (ALIGN, TURN_IN_PLACE):
        return (-0.02, 0.02, True)
    if mode == FORWARD_TRACK:
        return (0.0, DEFAULT_GEOM.max_vx * 0.95, True)
    if mode == FORWARD_TURN:
        return (0.04, DEFAULT_GEOM.max_vx * 0.75, True)
    if mode == LOCAL_LEFT:
        return (0.04, DEFAULT_GEOM.max_vx * 0.75, True)
    if mode == LOCAL_RIGHT:
        return (0.04, DEFAULT_GEOM.max_vx * 0.75, True)
    if mode == REPOSITION:
        return (-0.08, 0.12, True)
    if mode == REVERSE_ESCAPE:
        return (-0.18, -0.04, True)
    if mode == SAFE_STOP:
        return (0.0, 0.0, False)
    if mode == WAIT_FOR_CLEARANCE:
        return (0.0, 0.0, False)
    return (0.0, DEFAULT_GEOM.max_vx * 0.95, True)


class ManeuverFSM:
    def __init__(self) -> None:
        self.mode = IDLE
        self.enter_ts = time.time()
        self.target_heading = 0.0
        self.last_decision: Optional[ManeuverDecision] = None
        self.reverse_before: Optional[ReverseSnapshot] = None
        self.reverse_after: Optional[Dict[str, Any]] = None
        self.history: List[Dict[str, Any]] = []
        self._align_sign = 1.0
        self._reposition_dir = 1.0
        from agv_bridge.local_maneuver import LocalManeuverSelector

        self.local_selector = LocalManeuverSelector()
        self.decision_label = "FORWARD"
        self._compare_invoked = False
        self._compare_fsm_reason = "NEVER"
        # STEP 3F-CORRECTIVE: longitudinal recovery progress (not yaw)
        self._recovery_start_pose: Optional[Tuple[float, float, float]] = None
        self._recovery_exec: Dict[str, Any] = {
            "status": "NONE",
            "action": "NONE",
            "signed_progress_m": 0.0,
            "distance_since_start_m": 0.0,
            "target_distance_m": 1.0,
            "target_remaining_m": 1.0,
            "stall_s": 0.0,
            "tracker": "TEMPORARY_REVERSE_TRACKER",
        }
        self._recovery_stall_since: Optional[float] = None

    def reset(self) -> None:
        self.mode = IDLE
        self.enter_ts = time.time()
        self.target_heading = 0.0
        self.last_decision = None
        self.reverse_before = None
        self.reverse_after = None
        self.history = []
        self.decision_label = "FORWARD"
        self.local_selector.reset()
        self._recovery_start_pose = None
        self._recovery_stall_since = None
        self._recovery_exec = {
            "status": "NONE",
            "action": "NONE",
            "signed_progress_m": 0.0,
            "distance_since_start_m": 0.0,
            "target_distance_m": 1.0,
            "target_remaining_m": 1.0,
            "stall_s": 0.0,
            "tracker": "TEMPORARY_REVERSE_TRACKER",
        }

    def _set_mode(self, mode: str, now: float, reason: str) -> None:
        if mode != self.mode:
            self.history.append(
                {"ts": now, "from": self.mode, "to": mode, "reason": reason, "duration_s": round(now - self.enter_ts, 2)}
            )
            if len(self.history) > 40:
                self.history = self.history[-40:]
            self.mode = mode
            self.enter_ts = now

    def time_in_mode(self, now: float) -> float:
        return max(0.0, now - self.enter_ts)

    def decide(
        self,
        *,
        now: float,
        x: float,
        y: float,
        yaw: float,
        path: Sequence[Pt],
        goal: Optional[Pt],
        front_near: float,
        rear_near: float,
        collision: bool,
        emergency: bool,
        path_progress: float,
        lateral_error: float,
        stuck_s: float,
        recovery_attempts: int,
        candidates: Optional[List[Dict[str, Any]]] = None,
        clearance_at: Optional[Callable[[float, float], float]] = None,
        collide: Optional[CollideFn] = None,
        actual_clearance: float = 1.0,
        nav_active: bool = True,
        policy_ctx: Optional[Dict[str, Any]] = None,
        dynamic_short: bool = False,
    ) -> ManeuverDecision:
        pol = policy_ctx or {}
        allow_side = bool(pol.get("allow_side_compare", True))
        allow_replan = bool(pol.get("allow_replan", True))
        allow_recovery = bool(pol.get("allow_recovery", True))
        require_capture_hard = bool(pol.get("require_capture_hard", True))
        path_follow_weight = float(pol.get("path_follow_weight", 5.0))
        policy_state = str(pol.get("state") or "")
        policy_behavior = str(pol.get("behavior") or "")
        corridor_hw = float(pol.get("max_deviation_m") or 0.3)
        commitment_active = bool(pol.get("commitment_active", False))
        committed_side = str(pol.get("committed_side") or "NONE")
        side_switch_authorized = bool(pol.get("side_switch_authorized", False))
        commitment_hard_fail = bool(pol.get("commitment_hard_fail", False))
        authorized_side = str(pol.get("authorized_side") or "").upper()
        if authorized_side not in ("LEFT", "RIGHT"):
            authorized_side = ""
        recovery_action = str(pol.get("recovery_action") or "NONE").upper()
        recovery_target_m = float(pol.get("recovery_target_distance_m") or 1.0)
        recovery_force_vx = float(pol.get("recovery_force_vx") or POLICY_REVERSE_VX)
        recovery_force_w = float(pol.get("recovery_force_w") or POLICY_REVERSE_W)
        # While committed, still evaluate L/R feasibility even if allow_side_compare=False
        eval_sides = allow_side or commitment_active
        prev_mode_for_switch = self.mode
        if emergency or not nav_active:
            self._set_mode(IDLE if not nav_active else SAFE_STOP, now, "emergency" if emergency else "idle")
            cap = PathCapture(available=False)
            fwd = ForwardFeasibility(False, "SIL_INPUT_INVALID" if emergency else "IDLE")
            vx0, vx1, aw = action_limits(self.mode)
            d = ManeuverDecision(
                mode=self.mode,
                target_heading=yaw,
                heading_error=0.0,
                capture=cap,
                forward=fwd,
                turn_feasible=False,
                rotation_safe=False,
                left_free=0.0,
                right_free=0.0,
                front_free=front_near,
                rear_free=rear_near,
                turn_left_feasible=False,
                turn_right_feasible=False,
                reason="emergency" if emergency else "idle",
                vx_min=vx0,
                vx_max=vx1,
                allow_w=aw,
                force_vx=0.0,
                force_w=0.0,
                pause_stuck=True,
                legacy_phase=PHASE_MAP.get(self.mode, "forward"),
            )
            self.last_decision = d
            return d

        if any(map(lambda v: v != v, [x, y, yaw, front_near])):  # NaN
            self._set_mode(SAFE_STOP, now, "SIL_INPUT_INVALID")
            return self._stop_decision(now, yaw, front_near, rear_near, "SIL_INPUT_INVALID")

        capture = find_best_path_capture(x, y, yaw, path, clearance_at=clearance_at)
        goal_herr = None
        if goal is not None:
            goal_herr = wrap_pi(math.atan2(goal[1] - y, goal[0] - x) - yaw)
        if capture.available:
            herr = capture.heading_error
            self.target_heading = capture.heading
            # When path progress is stalled / stuck and goal is clearly behind the vehicle,
            # prefer goal-facing align over blindly tracking a nearby path tangent.
            if (
                goal_herr is not None
                and abs(goal_herr) > H_ALIGN
                and (stuck_s >= 2.0 or path_progress < 0.05)
                and abs(herr) < H_TURN
            ):
                herr = goal_herr
                self.target_heading = wrap_pi(yaw + goal_herr)
                reason_hint = "GOAL_BEHIND_OVERRIDE"
            else:
                reason_hint = ""
        elif goal is not None:
            th = math.atan2(goal[1] - y, goal[0] - x)
            herr = wrap_pi(th - yaw)
            self.target_heading = th
            capture = PathCapture(
                available=True, point=goal, heading=th, distance=math.hypot(goal[0] - x, goal[1] - y), heading_error=herr
            )
            reason_hint = ""
        else:
            herr = 0.0
            reason_hint = ""

        rot_safe = rotation_safe_at(x, y, clearance=actual_clearance, collide=collide)
        left_free = sector_free(x, y, yaw, base_ang=math.pi / 2, collide=collide)
        right_free = sector_free(x, y, yaw, base_ang=-math.pi / 2, collide=collide)
        front_free = sector_free(x, y, yaw, base_ang=0.0, collide=collide)
        rear_free = sector_free(x, y, yaw, base_ang=math.pi, collide=collide)
        turn_left = left_free > 1.0 and rot_safe
        turn_right = right_free > 1.0 and rot_safe

        fwd = assess_forward_feasibility(
            heading_error=herr,
            front_near=front_near,
            front_stop=DEFAULT_GEOM.front_stop_m,
            collision=collision,
            capture=capture,
            candidates=candidates,
        )

        abs_h = abs(herr)
        reason = fwd.reason
        mode = self.mode

        policy_wants_reverse = (
            allow_recovery
            and recovery_action in ("LOCAL_REVERSE", "HISTORICAL_RETREAT")
            and recovery_attempts < MAX_RECOVERY_ATTEMPTS
            and rear_near > DEFAULT_GEOM.rear_stop_m + 0.08
        )

        def _signed_reverse_progress(px: float, py: float, pyaw: float) -> Tuple[float, float]:
            """Body-longitudinal retreat (+ = moved opposite heading from start)."""
            if self._recovery_start_pose is None:
                return 0.0, 0.0
            sx, sy, syaw = self._recovery_start_pose
            dx, dy = px - sx, py - sy
            # Displacement projected onto -heading(start) = backward body axis at start
            back_x = -math.cos(syaw)
            back_y = -math.sin(syaw)
            signed = dx * back_x + dy * back_y
            dist = math.hypot(dx, dy)
            return signed, dist

        # Active reverse evaluation / early abort
        reverse_aborting = False
        skip_fresh = False
        if pol.get("dynamic_resume_clear") and self.mode == WAIT_FOR_CLEARANCE:
            mode = FORWARD_TRACK
            reason = "DYNAMIC_RESUME"
            skip_fresh = True
            decision_label = "FORWARD"
        if self.mode == REVERSE_ESCAPE and self.reverse_before is not None:
            dt_rev = self.time_in_mode(now)
            if self._recovery_start_pose is None:
                self._recovery_start_pose = (
                    float(self.reverse_before.pose[0]),
                    float(self.reverse_before.pose[1]),
                    float(self.reverse_before.theta),
                )
            signed_prog, dist_prog = _signed_reverse_progress(x, y, yaw)
            tgt = max(0.35, min(2.5, recovery_target_m))
            remaining = max(0.0, tgt - signed_prog)
            self._recovery_exec.update(
                {
                    "status": "EXECUTING" if signed_prog < RECOVERY_MIN_PROGRESS_M else "PROGRESSING",
                    "action": recovery_action or "LOCAL_REVERSE",
                    "signed_progress_m": round(signed_prog, 3),
                    "distance_since_start_m": round(dist_prog, 3),
                    "target_distance_m": round(tgt, 3),
                    "target_remaining_m": round(remaining, 3),
                    "stall_s": 0.0,
                    "tracker": "TEMPORARY_REVERSE_TRACKER",
                }
            )
            # Stall: no longitudinal progress while supposedly reversing
            if signed_prog < RECOVERY_MIN_PROGRESS_M * 0.5:
                if self._recovery_stall_since is None:
                    self._recovery_stall_since = now
                stall_s = now - float(self._recovery_stall_since)
                self._recovery_exec["stall_s"] = round(stall_s, 2)
            else:
                self._recovery_stall_since = None
                self._recovery_exec["stall_s"] = 0.0

            clr_delta = actual_clearance - float(self.reverse_before.actual_clearance)
            prog_delta = path_progress - float(self.reverse_before.path_progress)
            herr_delta = abs(herr) - abs(float(self.reverse_before.heading_error))

            if policy_wants_reverse or recovery_action in ("LOCAL_REVERSE", "HISTORICAL_RETREAT"):
                # Policy reverse: progress = signed longitudinal only (yaw ≠ success)
                if signed_prog >= tgt * 0.85 or (signed_prog >= RECOVERY_MIN_PROGRESS_M and front_near > DEFAULT_GEOM.front_stop_m + 0.25):
                    self.reverse_after = {
                        "result": "SUCCESS",
                        "signed_progress_m": round(signed_prog, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    self._recovery_exec["status"] = "SUCCESS"
                    reverse_aborting = True
                    mode = POST_TURN
                    reason = "RECOVERY_STEP_COMPLETE"
                    decision_label = "FORWARD"
                    skip_fresh = True
                elif self._recovery_exec.get("stall_s", 0) >= RECOVERY_STALL_S and dt_rev >= RECOVERY_STALL_S:
                    self.reverse_after = {
                        "result": "STALLED",
                        "signed_progress_m": round(signed_prog, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    self._recovery_exec["status"] = "STALLED"
                    reverse_aborting = True
                    mode = SAFE_STOP if not allow_replan else REPLAN
                    reason = "RECOVERY_EXECUTION_STUCK"
                    decision_label = "REPLAN"
                    skip_fresh = True
                elif dt_rev >= RECOVERY_MOVE_TIMEOUT_S and signed_prog < RECOVERY_MIN_PROGRESS_M:
                    self.reverse_after = {
                        "result": "FAILED",
                        "signed_progress_m": round(signed_prog, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    self._recovery_exec["status"] = "FAILED"
                    reverse_aborting = True
                    mode = SAFE_STOP if not allow_replan else REPLAN
                    reason = "RECOVERY_EXECUTION_FAILED"
                    decision_label = "REPLAN"
                    skip_fresh = True
                else:
                    # Keep reversing — do not fall into TURN/REPOSITION
                    mode = REVERSE_ESCAPE
                    reason = f"POLICY_RECOVERY|{recovery_action or 'LOCAL_REVERSE'}"
                    decision_label = "REVERSE"
                    skip_fresh = True
            elif dt_rev >= REVERSE_EVAL_S:
                if clr_delta < -0.08 or (prog_delta < 0.02 and herr_delta > 0.15):
                    self.reverse_after = {
                        "result": "WORSE",
                        "clearance_delta": round(clr_delta, 3),
                        "progress_delta": round(prog_delta, 3),
                        "heading_delta": round(herr_delta, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    reverse_aborting = True
                    # Never endless TURN after reverse fail — reassess
                    mode = REPLAN if allow_replan else SAFE_STOP
                    reason = "REVERSE_WORSENING→REPLAN"
                elif clr_delta > 0.08 or signed_prog >= RECOVERY_MIN_PROGRESS_M:
                    self.reverse_after = {
                        "result": "IMPROVED",
                        "clearance_delta": round(clr_delta, 3),
                        "progress_delta": round(prog_delta, 3),
                        "signed_progress_m": round(signed_prog, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    reverse_aborting = True
                    mode = POST_TURN
                    reason = "REVERSE_IMPROVED→FORWARD"
                elif dt_rev >= REVERSE_MAX_S:
                    self.reverse_after = {
                        "result": "NO_CHANGE",
                        "clearance_delta": round(clr_delta, 3),
                        "progress_delta": round(prog_delta, 3),
                        "heading_delta": round(herr_delta, 3),
                        "duration_s": round(dt_rev, 2),
                    }
                    reverse_aborting = True
                    mode = REPLAN if recovery_attempts < MAX_RECOVERY_ATTEMPTS else SAFE_STOP
                    reason = "REVERSE_TIMEOUT"

        # STEP 3F-CORRECTIVE: Policy LOCAL_REVERSE/HISTORICAL_RETREAT owns mode
        # Must beat HEADING_ALIGN / STUCK→REPOSITION / TURN_IN_PLACE
        if policy_wants_reverse and not reverse_aborting:
            mode = REVERSE_ESCAPE
            reason = f"POLICY_RECOVERY|{recovery_action}"
            decision_label = "REVERSE"
            skip_fresh = True
            if self._recovery_start_pose is None or self.mode != REVERSE_ESCAPE:
                self._recovery_start_pose = (x, y, yaw)
                self._recovery_stall_since = None
                self._recovery_exec.update(
                    {
                        "status": "PLANNED",
                        "action": recovery_action,
                        "signed_progress_m": 0.0,
                        "distance_since_start_m": 0.0,
                        "target_distance_m": round(max(0.35, min(2.5, recovery_target_m)), 3),
                        "target_remaining_m": round(max(0.35, min(2.5, recovery_target_m)), 3),
                        "stall_s": 0.0,
                        "tracker": "TEMPORARY_REVERSE_TRACKER",
                    }
                )
            else:
                signed_prog, dist_prog = _signed_reverse_progress(x, y, yaw)
                tgt = float(self._recovery_exec.get("target_distance_m") or recovery_target_m)
                self._recovery_exec.update(
                    {
                        "status": "EXECUTING" if signed_prog < RECOVERY_MIN_PROGRESS_M else "PROGRESSING",
                        "signed_progress_m": round(signed_prog, 3),
                        "distance_since_start_m": round(dist_prog, 3),
                        "target_remaining_m": round(max(0.0, tgt - signed_prog), 3),
                    }
                )
        elif recovery_action == "SAFE_STOP" and not reverse_aborting:
            # S2 / NO_ESCAPE: stop — never endless TURN
            mode = SAFE_STOP
            reason = "POLICY_RECOVERY|SAFE_STOP"
            decision_label = "REPLAN"
            skip_fresh = True
            self._recovery_exec.update({"status": "FAILED", "action": "SAFE_STOP"})
        elif recovery_action == "REPLAN" and not reverse_aborting and not policy_wants_reverse:
            mode = REPLAN if allow_replan else SAFE_STOP
            reason = "POLICY_RECOVERY|REPLAN"
            decision_label = "REPLAN"
            skip_fresh = True
        elif not policy_wants_reverse and self.mode != REVERSE_ESCAPE:
            if self._recovery_exec.get("status") not in ("SUCCESS", "FAILED", "STALLED"):
                self._recovery_exec["status"] = "NONE"
            self._recovery_start_pose = None
            self._recovery_stall_since = None

        # Timeouts for align/turn/reposition
        if (not skip_fresh) and self.mode in (ALIGN, TURN_IN_PLACE) and self.time_in_mode(now) > (
            TURN_MAX_S if self.mode == TURN_IN_PLACE else ALIGN_MAX_S
        ):
            if rot_safe is False:
                mode = REPOSITION
                reason = "TURN_TIMEOUT→REPOSITION"
            else:
                mode = REPLAN
                reason = "TURN_TIMEOUT→REPLAN"
        if (not skip_fresh) and self.mode == REPOSITION and self.time_in_mode(now) > REPOSITION_MAX_S:
            mode = REPLAN
            reason = "REPOSITION_TIMEOUT"

        # Align / turn success → post turn → forward
        if (not skip_fresh) and self.mode in (ALIGN, TURN_IN_PLACE, POST_TURN):
            if abs_h <= ALIGN_TOL and capture.available and capture.distance <= PATH_CAPTURE_TOL_M + 0.4:
                mode = FORWARD_TRACK if self.mode == POST_TURN else POST_TURN
                reason = "ALIGN_OK" if self.mode != POST_TURN else "CAPTURE_OK"

        local_compare_tel: Optional[Dict[str, Any]] = None
        self._compare_invoked = False
        self._compare_fsm_reason = "NOT_EVALUATED"
        if not skip_fresh:
            decision_label = self.decision_label
        # else: decision_label already set by policy reverse / abort paths

        # Local LEFT/RIGHT comparison when corridor stressed AND policy allows
        need_side_compare = (
            (not skip_fresh)
            and eval_sides
            and not reverse_aborting
            and abs_h <= H_ALIGN
            and self.mode
            in (
                IDLE,
                FORWARD_TRACK,
                FORWARD_TURN,
                LOCAL_LEFT,
                LOCAL_RIGHT,
                POST_TURN,
                WAIT_FOR_CLEARANCE,
                REPLAN,
            )
            and (
                (not fwd.feasible)
                or front_near < DEFAULT_GEOM.front_cost_m + 0.45
                or self.mode in (LOCAL_LEFT, LOCAL_RIGHT)
                or self.local_selector.current in ("LEFT", "RIGHT")
                or policy_behavior in ("AVOID_LEFT", "AVOID_RIGHT", "CAUTION")
                or policy_state in ("LOCAL_AVOID", "OBSTACLE_APPROACH")
                or commitment_active
            )
        )

        # Fresh decision when idle/forward/recover-like
        if (not skip_fresh) and (not reverse_aborting) and (
            mode
            in (
                IDLE,
                FORWARD_TRACK,
                FORWARD_TURN,
                LOCAL_LEFT,
                LOCAL_RIGHT,
                POST_TURN,
                WAIT_FOR_CLEARANCE,
                REPLAN,
            )
            or (self.mode in (FORWARD_TRACK, FORWARD_TURN, LOCAL_LEFT, LOCAL_RIGHT, IDLE) and mode == self.mode)
        ):
            if collision and rear_near > DEFAULT_GEOM.rear_stop_m + 0.2:
                if not rot_safe and front_near < DEFAULT_GEOM.front_stop_m + 0.15:
                    if allow_recovery and recovery_attempts < MAX_RECOVERY_ATTEMPTS:
                        mode = REVERSE_ESCAPE
                        reason = "COLLISION_TRAP"
                        decision_label = "REVERSE"
                    else:
                        mode = SAFE_STOP
                        reason = "COLLISION_TRAP_NO_RECOVERY"
                        decision_label = "REPLAN"
                elif rot_safe:
                    mode = TURN_IN_PLACE if abs_h > H_ALIGN else ALIGN
                    reason = "COLLISION→ALIGN"
                    decision_label = "ALIGN"
                else:
                    mode = SAFE_STOP
                    reason = "COLLISION_NO_ESCAPE"
                    decision_label = "REPLAN"
            elif (
                eval_sides
                and (
                    front_near < DEFAULT_GEOM.front_cost_m + 0.45
                    and abs_h <= (H_ALIGN + 0.55)
                )
            ) or need_side_compare:
                # Front corridor stressed: compare LEFT/RIGHT before ALIGN (policy-gated)
                dyn = bool(dynamic_short)
                self._compare_invoked = True
                self._compare_fsm_reason = "INVOKED"
                cmp = self.local_selector.compare(
                    now=now,
                    x=x,
                    y=y,
                    yaw=yaw,
                    path=path,
                    goal=goal,
                    front_near=front_near,
                    rear_near=rear_near,
                    collide=collide or (lambda *_: False),
                    clearance_at=clearance_at,
                    forward_feasible=bool(fwd.feasible and front_near >= DEFAULT_GEOM.front_stop_m),
                    rotation_safe=rot_safe,
                    left_free=left_free,
                    right_free=right_free,
                    dynamic_short=dyn,
                    force=False,
                    require_capture=require_capture_hard,
                    max_deviation_m=corridor_hw,
                    path_follow_scale=path_follow_weight / 5.0,
                    commitment_active=commitment_active,
                    committed_side=committed_side if committed_side in ("LEFT", "RIGHT") else None,
                    side_switch_authorized=side_switch_authorized,
                    commitment_hard_fail=commitment_hard_fail,
                    authorized_side=authorized_side or None,
                )
                local_compare_tel = cmp.to_telemetry()
                sel = cmp.selected
                decision_label = sel
                if self.local_selector.obstacle_passed and sel in ("LEFT", "RIGHT"):
                    mode = POST_TURN
                    reason = "OBSTACLE_PASSED→CAPTURE"
                    decision_label = "FORWARD"
                    self.local_selector.obstacle_passed = False
                    self.local_selector.current = "FORWARD"
                    self.local_selector.side_attempts = 0
                elif sel == "FORWARD":
                    mode = FORWARD_TRACK if fwd.feasible else FORWARD_TURN
                    reason = f"LOCAL|{cmp.reason}"
                elif sel == "LEFT":
                    # STEP 3E: committed RIGHT cannot become LEFT without authorization
                    if (
                        commitment_active
                        and committed_side == "RIGHT"
                        and not side_switch_authorized
                    ):
                        mode = LOCAL_RIGHT
                        reason = "AUTH_GATE_BLOCK_LEFT"
                        decision_label = "RIGHT"
                        self.local_selector.current = "RIGHT"
                    else:
                        mode = LOCAL_LEFT
                        reason = f"LOCAL_LEFT|{cmp.reason}"
                elif sel == "RIGHT":
                    if (
                        commitment_active
                        and committed_side == "LEFT"
                        and not side_switch_authorized
                    ):
                        mode = LOCAL_LEFT
                        reason = "AUTH_GATE_BLOCK_RIGHT"
                        decision_label = "LEFT"
                        self.local_selector.current = "LEFT"
                    else:
                        mode = LOCAL_RIGHT
                        reason = f"LOCAL_RIGHT|{cmp.reason}"
                elif sel == "ALIGN":
                    mode = ALIGN if rot_safe else REPOSITION
                    reason = f"LOCAL|{cmp.reason}"
                elif sel == "REPOSITION":
                    mode = REPOSITION
                    reason = f"LOCAL|{cmp.reason}"
                elif sel == "WAIT":
                    mode = WAIT_FOR_CLEARANCE
                    reason = f"LOCAL|{cmp.reason}"
                elif sel == "REVERSE":
                    if allow_recovery and recovery_attempts < MAX_RECOVERY_ATTEMPTS:
                        mode = REVERSE_ESCAPE
                        reason = f"LOCAL|{cmp.reason}"
                    else:
                        mode = REPLAN if allow_replan else SAFE_STOP
                        reason = "LOCAL_REVERSE_BLOCKED_BY_POLICY"
                else:
                    mode = REPLAN if allow_replan else WAIT_FOR_CLEARANCE
                    reason = f"LOCAL|{cmp.reason}"
                # Stash switch intent for Policy consume (nav_models reads via policy_ctx callback fields)
                if local_compare_tel is not None and isinstance(local_compare_tel, dict):
                    local_compare_tel["prev_mode"] = prev_mode_for_switch
                    local_compare_tel["authorized_side"] = authorized_side or None
                    local_compare_tel["side_switch_authorized"] = side_switch_authorized
            elif fwd.maneuver_required == "ALIGN_REQUIRED" or abs_h > H_ALIGN:
                if rot_safe:
                    mode = TURN_IN_PLACE if abs_h > (H_ALIGN + 0.25) else ALIGN
                    reason = "HEADING_ALIGN"
                    decision_label = "ALIGN"
                else:
                    mode = REPOSITION
                    reason = "ALIGN_UNSAFE→REPOSITION"
                    decision_label = "REPOSITION"
            elif fwd.maneuver_required == "FORWARD_TURN" or (H_FORWARD < abs_h <= H_ALIGN and fwd.feasible):
                mode = FORWARD_TURN
                reason = "FORWARD_TURN"
                decision_label = "FORWARD"
            elif fwd.feasible:
                mode = FORWARD_TRACK
                reason = "FORWARD_OK"
                decision_label = "FORWARD"
            elif front_near < DEFAULT_GEOM.front_stop_m and not rot_safe:
                if (
                    allow_recovery
                    and rear_near > DEFAULT_GEOM.rear_stop_m + 0.15
                    and recovery_attempts < MAX_RECOVERY_ATTEMPTS
                ):
                    mode = REVERSE_ESCAPE
                    reason = "DEAD_END_REVERSE"
                    decision_label = "REVERSE"
                else:
                    mode = SAFE_STOP if not allow_replan else REPLAN
                    reason = "DEAD_END_NO_REAR"
            elif stuck_s >= 8.0:
                if abs_h > H_FORWARD and rot_safe:
                    mode = TURN_IN_PLACE if abs_h > H_ALIGN else ALIGN
                    reason = "STUCK→ALIGN"
                    decision_label = "ALIGN"
                elif left_free > 1.2 or right_free > 1.2 or front_free > 1.0:
                    mode = REPOSITION
                    reason = "STUCK→REPOSITION"
                    decision_label = "REPOSITION"
                elif (
                    allow_recovery
                    and front_near < DEFAULT_GEOM.front_stop_m
                    and not rot_safe
                    and rear_near > DEFAULT_GEOM.rear_stop_m + 0.2
                    and recovery_attempts < MAX_RECOVERY_ATTEMPTS
                ):
                    mode = REVERSE_ESCAPE
                    reason = "STUCK→REVERSE"
                    decision_label = "REVERSE"
                else:
                    mode = REPLAN if allow_replan else SAFE_STOP
                    reason = "STUCK→REPLAN"
                    decision_label = "REPLAN"
            elif not fwd.feasible and fwd.reason == "PATH_CAPTURE_UNREACHABLE":
                mode = REPLAN if allow_replan else WAIT_FOR_CLEARANCE
                reason = "NO_PATH_CAPTURE_POINT"
                decision_label = "REPLAN"
            else:
                mode = FORWARD_TRACK if fwd.feasible else WAIT_FOR_CLEARANCE
                reason = fwd.reason
                decision_label = "FORWARD" if fwd.feasible else "WAIT"
            if reason_hint and mode in (ALIGN, TURN_IN_PLACE):
                reason = f"{reason_hint}|{reason}"

            # Policy PATH_RECAPTURE prefers POST_TURN when heading OK
            if policy_state == "PATH_RECAPTURE" and mode in (FORWARD_TRACK, FORWARD_TURN, LOCAL_LEFT, LOCAL_RIGHT):
                if abs_h <= H_ALIGN and capture.available:
                    mode = POST_TURN
                    reason = f"POLICY_RECAPTURE|{reason}"
                    decision_label = "FORWARD"

        self.decision_label = decision_label

        # Prefer turn toward freer side when aligning
        self._align_sign = 1.0 if herr >= 0 else -1.0
        if abs_h > H_FORWARD:
            if herr > 0 and not turn_left and turn_right:
                self._align_sign = -1.0
            elif herr < 0 and not turn_right and turn_left:
                self._align_sign = 1.0

        if left_free >= right_free:
            self._reposition_dir = 1.0
        else:
            self._reposition_dir = -1.0

        # Snapshot on reverse enter
        entered_rev = mode == REVERSE_ESCAPE and self.mode != REVERSE_ESCAPE
        self._set_mode(mode, now, reason)
        if mode == REVERSE_ESCAPE and (entered_rev or self.reverse_before is None):
            self.reverse_before = ReverseSnapshot(
                ts=now,
                pose=(x, y),
                theta=yaw,
                front_near=front_near,
                rear_near=rear_near,
                path_progress=path_progress,
                lateral_error=lateral_error,
                heading_error=herr,
                actual_clearance=actual_clearance,
                forward_feasible=fwd.feasible,
                forward_reason=fwd.reason,
                recovery_attempt=recovery_attempts,
                phase_before=self.history[-1]["from"] if self.history else "forward",
            )
            self.reverse_after = None

        vx0, vx1, aw = action_limits(self.mode)
        force_vx = None
        force_w = None
        if self.mode in (ALIGN, TURN_IN_PLACE):
            force_vx = 0.0
            force_w = max(-WZ_MAX, min(WZ_MAX, 1.15 * herr))
            if abs(force_w) < 0.08:
                force_w = 0.12 * self._align_sign
        elif self.mode == LOCAL_LEFT:
            lc = None
            if local_compare_tel and local_compare_tel.get("candidates"):
                lc = (local_compare_tel["candidates"] or {}).get("LEFT") or {}
            force_vx = float(lc.get("vx") or 0.14)
            force_w = abs(float(lc.get("w") or 0.28))
        elif self.mode == LOCAL_RIGHT:
            lc = None
            if local_compare_tel and local_compare_tel.get("candidates"):
                lc = (local_compare_tel["candidates"] or {}).get("RIGHT") or {}
            force_vx = float(lc.get("vx") or 0.14)
            force_w = -abs(float(lc.get("w") or 0.28))
        elif self.mode == REVERSE_ESCAPE:
            # TEMPORARY_REVERSE_TRACKER: follow Probe/recovery straight reverse (no arbitrary yaw)
            force_vx = max(vx0, min(vx1, float(recovery_force_vx)))
            if force_vx >= -0.04:
                force_vx = max(vx0, min(vx1, POLICY_REVERSE_VX))
            force_w = float(recovery_force_w)
            if abs(force_w) > 0.05:
                # Only allow small w if historical retreat supplies validated curvature later
                force_w = max(-0.12, min(0.12, force_w))
            else:
                force_w = 0.0
            if self._recovery_exec.get("status") in ("NONE", "PLANNED"):
                self._recovery_exec["status"] = "EXECUTING"
        elif self.mode == REPOSITION:
            force_vx = 0.06 if front_free > 0.9 else -0.06
            force_w = 0.25 * self._reposition_dir
        elif self.mode == WAIT_FOR_CLEARANCE:
            force_vx = 0.0
            force_w = 0.0
        elif self.mode == SAFE_STOP:
            force_vx = 0.0
            force_w = 0.0

        pause_stuck = self.mode in (
            ALIGN,
            TURN_IN_PLACE,
            REPOSITION,
            REVERSE_ESCAPE,
            WAIT_FOR_CLEARANCE,
            POST_TURN,
            LOCAL_LEFT,
            LOCAL_RIGHT,
        )

        if local_compare_tel is None:
            if not eval_sides:
                fsm_r = "ALLOW_SIDE_COMPARE_FALSE"
            elif not need_side_compare:
                fsm_r = "NEED_SIDE_COMPARE_FALSE"
            else:
                fsm_r = "FSM_NOT_INVOKED"
            self._compare_fsm_reason = fsm_r
            try:
                sel_f = self.local_selector.forensics_dict(now)
            except Exception:
                sel_f = {}
            c_reason = sel_f.get("compare_reason") or "NONE_OPEN_FORWARD"
            if str(sel_f.get("current") or "FORWARD") == "FORWARD":
                c_reason = "NONE_OPEN_FORWARD"
            local_compare_tel = {
                "invoked": False,
                "compare_called": False,
                "compare_reason": c_reason,
                "fsm_skip_reason": fsm_r,
                "selector": sel_f,
                "candidates": {},
                "rows": [],
            }
        else:
            self._compare_fsm_reason = "INVOKED"
            local_compare_tel["invoked"] = True
            local_compare_tel["compare_called"] = self.local_selector.last_compare_called
            local_compare_tel["compare_reason"] = self.local_selector.last_compare_reason
            try:
                local_compare_tel["selector"] = self.local_selector.forensics_dict(now)
            except Exception:
                pass
            local_compare_tel["fsm_skip_reason"] = None

        d = ManeuverDecision(
            mode=self.mode,
            target_heading=self.target_heading,
            heading_error=herr,
            capture=capture,
            forward=fwd,
            turn_feasible=rot_safe and (turn_left or turn_right or abs_h > 0),
            rotation_safe=rot_safe,
            left_free=left_free,
            right_free=right_free,
            front_free=front_free,
            rear_free=rear_free,
            turn_left_feasible=turn_left,
            turn_right_feasible=turn_right,
            reason=reason,
            vx_min=vx0,
            vx_max=vx1,
            allow_w=aw,
            force_vx=force_vx,
            force_w=force_w,
            pause_stuck=pause_stuck,
            legacy_phase=PHASE_MAP.get(self.mode, "forward"),
            decision=self.decision_label,
            local_compare=local_compare_tel,
            policy_state=policy_state,
            policy_behavior=policy_behavior,
            corridor_half_width=corridor_hw,
            path_follow_weight=path_follow_weight,
            recovery_exec=dict(self._recovery_exec),
        )
        self.last_decision = d
        return d

    def _stop_decision(self, now, yaw, front_near, rear_near, reason: str) -> ManeuverDecision:
        vx0, vx1, aw = action_limits(SAFE_STOP)
        d = ManeuverDecision(
            mode=SAFE_STOP,
            target_heading=yaw,
            heading_error=0.0,
            capture=PathCapture(available=False),
            forward=ForwardFeasibility(False, reason),
            turn_feasible=False,
            rotation_safe=False,
            left_free=0.0,
            right_free=0.0,
            front_free=front_near,
            rear_free=rear_near,
            turn_left_feasible=False,
            turn_right_feasible=False,
            reason=reason,
            vx_min=vx0,
            vx_max=vx1,
            allow_w=False,
            force_vx=0.0,
            force_w=0.0,
            pause_stuck=True,
            legacy_phase="safe_stop",
        )
        self.last_decision = d
        return d

    def to_telemetry(self) -> Dict[str, Any]:
        d = self.last_decision
        if not d:
            return {"mode": self.mode}
        cap = d.capture
        out = {
            "mode": d.mode,
            "reason": d.reason,
            "target_heading": round(d.target_heading, 4),
            "heading_error": round(d.heading_error, 4),
            "time_in_mode_s": round(time.time() - self.enter_ts, 2),
            "capture": {
                "available": cap.available,
                "distance": round(cap.distance, 3),
                "heading_error": round(cap.heading_error, 4),
                "clearance": round(cap.clearance, 3),
                "point": {"x": cap.point[0], "y": cap.point[1]} if cap.point else None,
            },
            "forward_feasible": d.forward.feasible,
            "forward_reason": d.forward.reason,
            "maneuver_required": d.forward.maneuver_required,
            "rotation_safe": d.rotation_safe,
            "turn_feasible": d.turn_feasible,
            "left_free": round(d.left_free, 2),
            "right_free": round(d.right_free, 2),
            "front_free": round(d.front_free, 2),
            "rear_free": round(d.rear_free, 2),
            "turn_left_feasible": d.turn_left_feasible,
            "turn_right_feasible": d.turn_right_feasible,
            "vx_min": d.vx_min,
            "vx_max": d.vx_max,
            "pause_stuck": d.pause_stuck,
            "legacy_phase": d.legacy_phase,
            "decision": d.decision,
            "local_compare": d.local_compare,
            "local_events": self.local_selector.events[-12:],
            "local_history": self.local_selector.history[-8:],
            "obstacle_passed": self.local_selector.obstacle_passed,
            "side_attempts": self.local_selector.side_attempts,
            "history": self.history[-8:],
            "reverse_before": None
            if not self.reverse_before
            else {
                "front": self.reverse_before.front_near,
                "rear": self.reverse_before.rear_near,
                "progress": self.reverse_before.path_progress,
                "heading_error": self.reverse_before.heading_error,
                "forward_feasible": self.reverse_before.forward_feasible,
                "forward_reason": self.reverse_before.forward_reason,
            },
            "reverse_after": self.reverse_after,
            "recovery_exec": dict(self._recovery_exec),
            "force_vx": d.force_vx,
            "force_w": d.force_w,
            "max_heading_change_horizon": round(max_heading_change_in_horizon(), 3),
        }
        out["local_compare_invoked"] = bool(getattr(self, "_compare_invoked", False))
        out["local_compare_fsm_reason"] = getattr(self, "_compare_fsm_reason", None)
        try:
            out["local_selector"] = self.local_selector.forensics_dict()
        except Exception:
            out["local_selector"] = {}
        return out
