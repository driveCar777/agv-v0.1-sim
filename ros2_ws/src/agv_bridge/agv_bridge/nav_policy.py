"""Navigation Policy / Behavior Supervisor.

Owns WHY (behavior, corridor, cost profile, loop guards).
Does NOT write vx/w, integrate pose, or bypass Safety.
ManeuverFSM owns HOW TO MANEUVER; Local Planner owns trajectory under behavior.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.nav_commitment import (
    AUTH_LOCKED,
    AUTH_NONE,
    AUTH_REVALIDATION_REQUIRED,
    AvoidanceCommitment,
    FAIL_HARD_INFEASIBLE,
    FAIL_NONE,
    FAIL_SOFT_COST,
    PHASE_COMMITTED_LEFT,
    PHASE_COMMITTED_RIGHT,
    PHASE_EVALUATING,
    PHASE_NONE,
    PHASE_PASSING,
    PHASE_RELEASED,
    PHASE_REVALIDATING,
    PHASE_SELECTING,
    make_obstacle_signature,
)
from agv_bridge.nav_side_switch import (
    AUTH_STATUS_AUTHORIZED,
    AUTH_STATUS_DENIED,
    AUTH_STATUS_EXECUTED,
    AUTH_STATUS_EXPIRED,
    REASON_EXEC_FAILED,
    REASON_NONE,
    SideSwitchDecision,
    authorize_side_switch,
    validate_token_for_execution,
)

Pt = Tuple[float, float]

# ---- Policy states ----
IDLE = "IDLE"
FOLLOW_GLOBAL = "FOLLOW_GLOBAL"
CAUTION = "CAUTION"
OBSTACLE_APPROACH = "OBSTACLE_APPROACH"
FUTURE_PREVIEW = "FUTURE_PREVIEW"
SIDE_PROBE = "SIDE_PROBE"
LOCAL_AVOID = "LOCAL_AVOID"
PATH_RECAPTURE = "PATH_RECAPTURE"
WAIT_FOR_CLEARANCE = "WAIT_FOR_CLEARANCE"
ALIGN = "ALIGN"
TURN_IN_PLACE = "TURN_IN_PLACE"
REPOSITION = "REPOSITION"
REPLAN = "REPLAN"
RECOVERY = "RECOVERY"
SAFE_STOP = "SAFE_STOP"
ARRIVED = "ARRIVED"
FAILED = "FAILED"

# Behavior actions (what Local/Maneuver may execute)
BEH_FOLLOW = "FOLLOW_GLOBAL"
BEH_CAUTION = "CAUTION"
BEH_AVOID_LEFT = "AVOID_LEFT"
BEH_AVOID_RIGHT = "AVOID_RIGHT"
BEH_WAIT = "WAIT"
BEH_ALIGN = "ALIGN"
BEH_TURN = "TURN_IN_PLACE"
BEH_REPOSITION = "REPOSITION"
BEH_RECAPTURE = "RECAPTURE"
BEH_REPLAN = "REPLAN"
BEH_RECOVERY = "RECOVERY"
BEH_SAFE_STOP = "SAFE_STOP"

# Priority (lower number = higher priority)
PRIORITY = {
    SAFE_STOP: 1,
    FAILED: 1,
    RECOVERY: 2,
    TURN_IN_PLACE: 3,
    ALIGN: 3,
    LOCAL_AVOID: 4,
    SIDE_PROBE: 5,
    WAIT_FOR_CLEARANCE: 5,
    PATH_RECAPTURE: 6,
    OBSTACLE_APPROACH: 7,
    FUTURE_PREVIEW: 7,
    CAUTION: 8,
    FOLLOW_GLOBAL: 9,
    REPLAN: 10,
    REPOSITION: 3,
    ARRIVED: 0,
    IDLE: 11,
}

# Corridor half-widths (m) — scaled to vehicle width
_W = DEFAULT_GEOM.width
CORRIDOR_NORMAL = max(0.30, 0.55 * _W)          # ~0.30
CORRIDOR_CAUTION = max(0.45, 0.90 * _W)          # ~0.50
CORRIDOR_AVOID = max(0.80, 1.55 * _W)            # ~0.85
CORRIDOR_SEVERE = max(1.20, 2.20 * _W)           # ~1.21
CORRIDOR_RECAPTURE = max(0.55, 1.10 * _W)        # ~0.60

MAX_LOCAL_DEVIATION_TIME_S = 8.0
MIN_AVOID_HOLD_S = 1.0
MIN_BEHAVIOR_HOLD_S = 0.55
COMPARE_COOLDOWN_S = 0.20

# Loop guards
OSCILLATION_WINDOW_S = 4.0
OSCILLATION_FLIP_LIMIT = 6
REPLAN_WINDOW_S = 12.0
REPLAN_COUNT_LIMIT = 4
RECOVERY_WINDOW_S = 20.0
RECOVERY_COUNT_LIMIT = 3
DEADLOCK_HOLD_S = 2.5


@dataclass
class CostProfile:
    name: str
    path_follow: float = 1.0
    path_align: float = 1.0
    obstacle: float = 1.0
    clearance: float = 1.0
    progress: float = 1.0
    smoothness: float = 1.0
    capture: float = 1.0
    vx_scale: float = 1.0

    def to_dict(self) -> Dict[str, float]:
        return {
            "name": self.name,
            "path_follow": self.path_follow,
            "path_align": self.path_align,
            "obstacle": self.obstacle,
            "clearance": self.clearance,
            "progress": self.progress,
            "smoothness": self.smoothness,
            "capture": self.capture,
            "vx_scale": self.vx_scale,
        }


NORMAL_PROFILE = CostProfile("NORMAL", path_follow=1.0, path_align=1.0, obstacle=0.7, clearance=0.7, progress=1.0, smoothness=0.8, capture=0.8, vx_scale=1.0)
CAUTION_PROFILE = CostProfile("CAUTION", path_follow=0.85, path_align=0.9, obstacle=1.2, clearance=1.2, progress=0.9, smoothness=0.9, capture=0.9, vx_scale=0.75)
AVOID_PROFILE = CostProfile("AVOID", path_follow=0.25, path_align=0.35, obstacle=1.6, clearance=1.5, progress=0.85, smoothness=1.0, capture=1.1, vx_scale=0.65)
RECAPTURE_PROFILE = CostProfile("RECAPTURE", path_follow=0.7, path_align=1.1, obstacle=1.0, clearance=1.0, progress=0.95, capture=1.5, smoothness=0.9, vx_scale=0.7)
RECOVERY_PROFILE = CostProfile("RECOVERY", path_follow=0.15, path_align=0.2, obstacle=1.8, clearance=1.8, progress=0.4, capture=0.5, smoothness=0.6, vx_scale=0.5)


@dataclass
class PathCorridor:
    half_width: float
    lateral_error: float
    inside: bool
    exceeded: bool
    time_away_s: float
    max_time_s: float = MAX_LOCAL_DEVIATION_TIME_S
    mode: str = "NORMAL"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "half_width": round(self.half_width, 3),
            "lateral_error": round(self.lateral_error, 3),
            "inside": self.inside,
            "exceeded": self.exceeded,
            "time_away_s": round(self.time_away_s, 2),
            "max_time_s": self.max_time_s,
            "mode": self.mode,
        }


@dataclass
class PolicyDecision:
    state: str
    behavior: str
    reason: str
    profile: CostProfile
    corridor: PathCorridor
    allow_side_compare: bool = False
    allow_replan: bool = False
    allow_recovery: bool = False
    path_follow_weight: float = 5.0  # MPPI global_path_cost scale
    require_capture_hard: bool = True
    max_deviation_m: float = CORRIDOR_NORMAL
    scene: str = "OPEN"
    events: List[Dict[str, Any]] = field(default_factory=list)
    flags: Dict[str, Any] = field(default_factory=dict)
    # STEP 3C — commitment gate (Policy owns authorization)
    commitment_active: bool = False
    committed_side: str = "NONE"
    commitment_phase: str = PHASE_NONE
    commitment_hard_fail: bool = False
    commitment_failure_reason: str = FAIL_NONE
    side_switch_authorized: bool = False
    side_switch_authorization_status: str = AUTH_NONE

    def to_dict(self) -> Dict[str, Any]:
        return {
            "state": self.state,
            "behavior": self.behavior,
            "reason": self.reason,
            "scene": self.scene,
            "profile": self.profile.to_dict(),
            "corridor": self.corridor.to_dict(),
            "allow_side_compare": self.allow_side_compare,
            "allow_replan": self.allow_replan,
            "allow_recovery": self.allow_recovery,
            "path_follow_weight": round(self.path_follow_weight, 3),
            "require_capture_hard": self.require_capture_hard,
            "max_deviation_m": round(self.max_deviation_m, 3),
            "flags": self.flags,
            "events": self.events[-8:],
            "commitment_active": self.commitment_active,
            "committed_side": self.committed_side,
            "commitment_phase": self.commitment_phase,
            "commitment_hard_fail": self.commitment_hard_fail,
            "commitment_failure_reason": self.commitment_failure_reason,
            "side_switch_authorized": self.side_switch_authorized,
            "side_switch_authorization_status": self.side_switch_authorization_status,
        }


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class NavigationPolicy:
    """Single decision owner for WHY / corridor / profiles / loop guards."""

    def __init__(self) -> None:
        self.state = IDLE
        self.behavior = BEH_FOLLOW
        self.reason = "INIT"
        self.state_enter_ts = 0.0
        self.behavior_enter_ts = 0.0
        self.hold_until = 0.0
        self.avoid_side: Optional[str] = None  # LEFT / RIGHT
        self.avoid_hold_until = 0.0
        self.away_since: Optional[float] = None
        self.events: List[Dict[str, Any]] = []
        self.history: List[Dict[str, Any]] = []
        self.w_sign_flips: List[float] = []
        self.side_flips: List[Tuple[float, str]] = []
        self.replan_times: List[float] = []
        self.recovery_times: List[float] = []
        self.deadlock_since: Optional[float] = None
        self.last_decision: Optional[PolicyDecision] = None
        self.obstacle_passed = False
        self._was_dynamic_short = False
        self.last_w_sign = 0
        self.commitment = AvoidanceCommitment()
        self.switch_token = None  # SideSwitchAuthorizationToken | None
        self.last_switch_decision: Optional[SideSwitchDecision] = None
        self.executed_switch_times: List[float] = []

    def reset(self) -> None:
        self.__init__()

    def authorize_side_switch_gate(
        self,
        *,
        now: float,
        probe_bundle=None,
        raw_candidate_side: Optional[str] = None,
        dynamic_short: bool = False,
        emergency: bool = False,
        safety_zero: bool = False,
        planned_rejected_by_safety: bool = False,
        path_valid: bool = True,
    ) -> SideSwitchDecision:
        """STEP 3E — Policy-owned authorization (consumes Probe; no rollout)."""
        # Expire prior live token if stale
        if self.switch_token is not None and self.switch_token.authorized and not self.switch_token.consumed:
            if now > self.switch_token.expires_at:
                self._emit(
                    "SIDE_SWITCH_AUTH_EXPIRED",
                    from_side=self.switch_token.from_side,
                    to_side=self.switch_token.to_side,
                )
                self.switch_token.authorized = False
                self.switch_token.consumed = True

        decision = authorize_side_switch(
            now=now,
            commitment=self.commitment,
            probe_bundle=probe_bundle,
            raw_candidate_side=raw_candidate_side,
            dynamic_short=dynamic_short,
            emergency=emergency,
            safety_zero=safety_zero,
            planned_rejected_by_safety=planned_rejected_by_safety,
            path_valid=path_valid,
            switch_times=list(self.executed_switch_times),
        )
        self.last_switch_decision = decision

        if decision.authorized and decision.token is not None:
            self.switch_token = decision.token
            self.commitment.authorization_status = AUTH_STATUS_AUTHORIZED
            self._emit(
                "SIDE_SWITCH_AUTHORIZED",
                from_side=decision.from_side,
                to_side=decision.to_side,
                reason=decision.primary_reason,
                source="NavigationPolicy",
            )
            if self.commitment.phase == PHASE_REVALIDATING:
                self._emit(
                    "SIDE_SWITCH_REVALIDATION",
                    side=decision.from_side,
                    status="AUTHORIZED",
                )
        elif decision.status == AUTH_STATUS_DENIED:
            self.switch_token = None
            if self.commitment.active:
                self.commitment.authorization_status = AUTH_REVALIDATION_REQUIRED if (
                    self.commitment.hard_fail or decision.gates.get("current_side_failed")
                ) else AUTH_LOCKED
            self._emit(
                "SIDE_SWITCH_DENIED",
                from_side=decision.from_side,
                to_side=decision.to_side,
                reason=decision.primary_reason,
                failed_gates=list(decision.failed_gates),
                source="NavigationPolicy",
            )
            if self.commitment.hard_fail or decision.gates.get("current_side_failed"):
                self._emit(
                    "SIDE_SWITCH_REVALIDATION",
                    side=decision.from_side,
                    status="DENIED",
                    reason=decision.primary_reason,
                )
        else:
            # NONE / WAITING — clear live auth
            if not (self.switch_token and self.switch_token.is_live(now)):
                self.switch_token = None

        # Mirror into last PolicyDecision if present
        if self.last_decision is not None:
            live = bool(self.switch_token and self.switch_token.is_live(now))
            self.last_decision.side_switch_authorized = live
            self.last_decision.side_switch_authorization_status = (
                AUTH_STATUS_AUTHORIZED if live else str(decision.status)
            )
        return decision

    def note_side_switch_executed(
        self,
        *,
        now: float,
        from_side: str,
        to_side: str,
        maneuver_mode: str = "",
    ) -> bool:
        """FSM confirmed LOCAL_* transition — consume token + retarget commitment."""
        tok = self.switch_token
        ok, why = validate_token_for_execution(
            tok,
            now=now,
            current_side=from_side,
            commitment_side=self.commitment.side if self.commitment.active else from_side,
            obstacle_signature=self.commitment.obstacle_signature,
        )
        if not ok or tok is None:
            self._emit(
                "SIDE_SWITCH_EXECUTION_FAILED",
                from_side=from_side,
                to_side=to_side,
                reason=why or REASON_EXEC_FAILED,
                mode=maneuver_mode,
                source="ManeuverFSM",
            )
            return False
        if str(tok.to_side).upper() != str(to_side).upper():
            self._emit(
                "SIDE_SWITCH_EXECUTION_FAILED",
                reason=REASON_EXEC_FAILED,
                expected=tok.to_side,
                got=to_side,
            )
            return False

        tok.consumed = True
        self.executed_switch_times.append(now)
        if len(self.executed_switch_times) > 40:
            self.executed_switch_times = self.executed_switch_times[-40:]
        self.commitment.last_switch_at = now
        self.commitment.switch_count = int(self.commitment.switch_count or 0) + 1
        # Atomic retarget
        self.create_commitment(
            now=now,
            side=to_side,
            reason=f"AUTHORIZED_SWITCH|{tok.reason}",
            pose=self.commitment.start_pose,
            signature=tok.obstacle_signature or self.commitment.obstacle_signature,
        )
        self.commitment.last_switch_at = now
        self._emit(
            "SIDE_SWITCH_EXECUTED",
            from_side=from_side,
            to_side=to_side,
            reason=tok.reason,
            source="ManeuverFSM",
        )
        self._emit(
            "COMMITMENT_SWITCHED",
            from_side=from_side,
            to_side=to_side,
            source="NavigationPolicy",
        )
        if self.last_switch_decision is not None:
            self.last_switch_decision.status = AUTH_STATUS_EXECUTED
            self.last_switch_decision.authorized = False
        self.switch_token = None
        return True

    def release_commitment(self, *, now: float, reason: str) -> None:
        if not self.commitment.active and self.commitment.phase in (PHASE_NONE, PHASE_RELEASED):
            self.commitment.release_reason = reason
            return
        self.commitment.active = False
        self.commitment.phase = PHASE_RELEASED
        self.commitment.release_reason = reason
        self.commitment.authorization_status = AUTH_NONE
        self.commitment.hard_fail = False
        self.commitment.failure_reason = FAIL_NONE
        self._emit("COMMITMENT_RELEASED", side=self.commitment.side, reason=reason, ts=now)
        self.commitment.side = "NONE"

    def create_commitment(
        self,
        *,
        now: float,
        side: str,
        reason: str,
        pose: Optional[Tuple[float, float, float]] = None,
        signature: Optional[str] = None,
    ) -> None:
        side_u = str(side or "").upper()
        if side_u not in ("LEFT", "RIGHT"):
            return
        self.commitment.active = True
        self.commitment.side = side_u
        self.commitment.phase = PHASE_COMMITTED_LEFT if side_u == "LEFT" else PHASE_COMMITTED_RIGHT
        self.commitment.reason = reason or "LOCAL_AVOID_SELECT"
        self.commitment.failure_reason = FAIL_NONE
        self.commitment.hard_fail = False
        self.commitment.soft_fail_streak = 0
        self.commitment.soft_fail_since = None
        self.commitment.started_at = now
        self.commitment.start_pose = pose
        self.commitment.obstacle_signature = signature
        self.commitment.release_reason = "NONE"
        self.commitment.authorization_status = AUTH_LOCKED
        self.avoid_side = side_u
        self._emit(
            "COMMITMENT_CREATED",
            side=side_u,
            reason=self.commitment.reason,
            obstacle_signature=signature,
            phase=self.commitment.phase,
        )

    def update_commitment_after_selection(
        self,
        *,
        now: float,
        selected: Optional[str],
        left_feasible: Optional[bool],
        right_feasible: Optional[bool],
        left_cost: Optional[float] = None,
        right_cost: Optional[float] = None,
        obstacle_passed: bool = False,
        policy_state: str = "",
        maneuver_mode: str = "",
        front_near: float = 30.0,
        left_free: float = 0.0,
        right_free: float = 0.0,
        scene: str = "",
        pose: Optional[Tuple[float, float, float]] = None,
        emergency: bool = False,
        nav_active: bool = True,
    ) -> None:
        """Post-selector lifecycle update. Does not write vx/w."""
        try:
            if (not nav_active) or emergency:
                self.release_commitment(now=now, reason="MISSION_RESET" if not nav_active else "EMERGENCY")
                return
            mm = (maneuver_mode or "").upper()
            pst = (policy_state or "").upper()
            if obstacle_passed or pst == PATH_RECAPTURE or mm == "POST_TURN":
                if self.commitment.active:
                    self.release_commitment(now=now, reason="OBSTACLE_PASSED" if obstacle_passed else "PATH_RECAPTURE")
                return
            if pst in (SAFE_STOP, FAILED, IDLE):
                if self.commitment.active:
                    self.release_commitment(now=now, reason=pst)
                return
            # Clear stale commitment once back on open forward follow
            if (
                self.commitment.active
                and str(selected or "").upper() == "FORWARD"
                and mm in ("FORWARD_TRACK", "FORWARD_TURN", "IDLE")
                and pst in (FOLLOW_GLOBAL, CAUTION, IDLE)
            ):
                self.release_commitment(now=now, reason="RETURN_FORWARD")
                return

            sel = str(selected or "").upper()
            sig = make_obstacle_signature(
                front_near=front_near, left_free=left_free, right_free=right_free, scene=scene
            )

            # Create on first real local side while avoiding
            if (not self.commitment.active) and sel in ("LEFT", "RIGHT") and (
                mm in ("LOCAL_LEFT", "LOCAL_RIGHT") or pst == LOCAL_AVOID
            ):
                self.create_commitment(now=now, side=sel, reason=f"SELECT|{sel}", pose=pose, signature=sig)
                return

            if not self.commitment.active:
                return

            side = self.commitment.side
            side_feasible = left_feasible if side == "LEFT" else right_feasible
            other_feasible = right_feasible if side == "LEFT" else left_feasible
            side_cost = left_cost if side == "LEFT" else right_cost
            other_cost = right_cost if side == "LEFT" else left_cost

            # HARD failure of committed side
            if side_feasible is False:
                if not self.commitment.hard_fail:
                    self.commitment.hard_fail = True
                    self.commitment.failure_reason = FAIL_HARD_INFEASIBLE
                    self.commitment.phase = PHASE_REVALIDATING
                    self.commitment.authorization_status = AUTH_REVALIDATION_REQUIRED
                    self._emit(
                        "COMMITMENT_HARD_FAILED",
                        side=side,
                        reason=FAIL_HARD_INFEASIBLE,
                        other_feasible=other_feasible,
                    )
                    self._emit(
                        "COMMITMENT_REVALIDATION_REQUIRED",
                        side=side,
                        authorized=False,
                        note="STEP3E will authorize alternate side; 3C blocks direct switch",
                    )
                else:
                    self.commitment.phase = PHASE_REVALIDATING
                    self.commitment.authorization_status = AUTH_REVALIDATION_REQUIRED
                return

            # Soft degradation: other cheaper but committed still feasible
            if (
                side_feasible is True
                and other_feasible is True
                and side_cost is not None
                and other_cost is not None
                and float(other_cost) + 1e-6 < float(side_cost)
            ):
                self.commitment.soft_fail_streak += 1
                if self.commitment.soft_fail_since is None:
                    self.commitment.soft_fail_since = now
                self.commitment.failure_reason = FAIL_SOFT_COST
                if self.commitment.phase in (PHASE_COMMITTED_LEFT, PHASE_COMMITTED_RIGHT):
                    self.commitment.phase = PHASE_EVALUATING
                if self.commitment.soft_fail_streak == 1 or self.commitment.soft_fail_streak % 10 == 0:
                    self._emit(
                        "COMMITMENT_SOFT_DEGRADED",
                        side=side,
                        side_cost=side_cost,
                        other_cost=other_cost,
                        streak=self.commitment.soft_fail_streak,
                    )
                self.commitment.authorization_status = AUTH_LOCKED
                return

            # Healthy retain
            self.commitment.soft_fail_streak = 0
            self.commitment.soft_fail_since = None
            if not self.commitment.hard_fail:
                self.commitment.failure_reason = FAIL_NONE
                self.commitment.phase = (
                    PHASE_COMMITTED_LEFT if side == "LEFT" else PHASE_COMMITTED_RIGHT
                )
                self.commitment.authorization_status = AUTH_LOCKED
                if self.commitment.obstacle_signature != sig and sig:
                    # Same avoid event evolving — refresh signature quietly
                    self.commitment.obstacle_signature = sig
        except Exception as exc:
            self._emit("COMMITMENT_STATE_INVALID", error=str(exc))
            self.release_commitment(now=now, reason="STATE_ERROR")

    def _emit(self, kind: str, **kw: Any) -> Dict[str, Any]:
        ev = {"ts": time.time(), "type": kind, **kw}
        self.events.append(ev)
        if len(self.events) > 40:
            self.events = self.events[-40:]
        return ev

    def time_in_state(self, now: float) -> float:
        return max(0.0, now - self.state_enter_ts) if self.state_enter_ts else 0.0

    def note_replan(self, now: float) -> None:
        self.replan_times.append(now)
        self.replan_times = [t for t in self.replan_times if now - t <= REPLAN_WINDOW_S]
        self._emit("REPLAN_TRIGGERED", count=len(self.replan_times))

    def note_recovery(self, now: float) -> None:
        self.recovery_times.append(now)
        self.recovery_times = [t for t in self.recovery_times if now - t <= RECOVERY_WINDOW_S]
        self._emit("RECOVERY_TRIGGERED", count=len(self.recovery_times))

    def note_cmd_w(self, now: float, w: float) -> None:
        sgn = 0 if abs(w) < 0.05 else (1 if w > 0 else -1)
        if sgn and self.last_w_sign and sgn != self.last_w_sign:
            self.w_sign_flips.append(now)
        if sgn:
            self.last_w_sign = sgn
        self.w_sign_flips = [t for t in self.w_sign_flips if now - t <= OSCILLATION_WINDOW_S]

    def note_side(self, now: float, side: str) -> None:
        if self.side_flips and self.side_flips[-1][1] != side:
            self.side_flips.append((now, side))
        elif not self.side_flips:
            self.side_flips.append((now, side))
        self.side_flips = [(t, s) for t, s in self.side_flips if now - t <= OSCILLATION_WINDOW_S]

    def oscillation_loop(self, now: float) -> bool:
        flips = len(self.w_sign_flips)
        side_changes = max(0, len(self.side_flips) - 1)
        return flips >= OSCILLATION_FLIP_LIMIT or side_changes >= 4

    def replan_loop(self, now: float) -> bool:
        self.replan_times = [t for t in self.replan_times if now - t <= REPLAN_WINDOW_S]
        return len(self.replan_times) >= REPLAN_COUNT_LIMIT

    def recovery_loop(self, now: float) -> bool:
        self.recovery_times = [t for t in self.recovery_times if now - t <= RECOVERY_WINDOW_S]
        return len(self.recovery_times) >= RECOVERY_COUNT_LIMIT

    def _corridor_for(self, state: str, lateral: float, now: float) -> PathCorridor:
        if state in (LOCAL_AVOID, OBSTACLE_APPROACH):
            hw, mode = CORRIDOR_AVOID, "AVOID"
            if lateral > CORRIDOR_AVOID * 0.9:
                hw, mode = CORRIDOR_SEVERE, "SEVERE"
        elif state == CAUTION:
            hw, mode = CORRIDOR_CAUTION, "CAUTION"
        elif state == PATH_RECAPTURE:
            hw, mode = CORRIDOR_RECAPTURE, "RECAPTURE"
        elif state == RECOVERY:
            hw, mode = CORRIDOR_SEVERE, "RECOVERY"
        else:
            hw, mode = CORRIDOR_NORMAL, "NORMAL"

        inside = lateral <= hw + 1e-3
        if inside:
            self.away_since = None
            away = 0.0
        else:
            if self.away_since is None:
                self.away_since = now
            away = now - self.away_since
        exceeded = (not inside) and away >= MAX_LOCAL_DEVIATION_TIME_S
        if exceeded:
            self._emit("LOCAL_DEVIATION_EXCEEDED", lateral=lateral, half_width=hw, time_away=away)
        return PathCorridor(
            half_width=hw,
            lateral_error=lateral,
            inside=inside,
            exceeded=exceeded,
            time_away_s=away,
            mode=mode,
        )

    def _profile_for(self, state: str) -> CostProfile:
        if state in (LOCAL_AVOID, OBSTACLE_APPROACH):
            return AVOID_PROFILE
        if state == CAUTION:
            return CAUTION_PROFILE
        if state == PATH_RECAPTURE:
            return RECAPTURE_PROFILE
        if state == RECOVERY:
            return RECOVERY_PROFILE
        return NORMAL_PROFILE

    def _path_follow_weight(self, profile: CostProfile) -> float:
        # NORMAL uses historical 5.0; AVOID softens rail
        return 5.0 * max(0.12, profile.path_follow)

    def _classify_scene(
        self,
        *,
        front_near: float,
        left_free: float,
        right_free: float,
        dynamic_short: bool,
        dynamic_long: bool,
        collision: bool,
        goal_behind: bool,
        forward_feasible: bool,
        path_valid: bool,
    ) -> str:
        if collision:
            return "COLLISION"
        if not path_valid:
            return "PATH_INVALID"
        if goal_behind:
            return "GOAL_BEHIND"
        if dynamic_long:
            return "DYNAMIC_LONG"
        if dynamic_short:
            return "DYNAMIC_SHORT"
        if front_near < DEFAULT_GEOM.front_stop_m:
            return "FRONT_BLOCKED"
        if front_near < DEFAULT_GEOM.front_cost_m + 0.45:
            if left_free > right_free + 0.35:
                return "BLOCK_LEFT_WIDE"
            if right_free > left_free + 0.35:
                return "BLOCK_RIGHT_WIDE"
            if left_free > 1.0 and right_free > 1.0:
                return "BLOCK_BOTH_WIDE"
            return "BLOCK_NARROW"
        if front_near < DEFAULT_GEOM.front_clear_m + 0.3:
            return "APPROACH"
        if not forward_feasible:
            return "FORWARD_INFEASIBLE"
        return "OPEN"

    def step(
        self,
        *,
        now: float,
        nav_active: bool,
        arrived: bool = False,
        emergency: bool = False,
        sensor_invalid: bool = False,
        map_invalid: bool = False,
        path_valid: bool = True,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        front_near: float = 30.0,
        rear_near: float = 30.0,
        left_free: float = 2.0,
        right_free: float = 2.0,
        lateral_error: float = 0.0,
        heading_error: float = 0.0,
        path_progress_rate: float = 0.0,
        state_vx: float = 0.0,
        collision: bool = False,
        forward_feasible: bool = True,
        rotation_safe: bool = True,
        dynamic_short: bool = False,
        dynamic_long: bool = False,
        maneuver_mode: str = "IDLE",
        local_decision: str = "FORWARD",
        recovery_attempts: int = 0,
        stuck_s: float = 0.0,
        goal_herr: float = 0.0,
        planned_rejected_by_safety: bool = False,
        safety_zero: bool = False,
        future_collision: bool = False,
        approach_active: bool = False,
        first_collision_distance_m: Optional[float] = None,
        required_avoidance_distance_m: Optional[float] = None,
        avoidance_phase: str = "OPEN",
        readiness_signal: str = "NONE",
        side_probe_active: bool = False,
        commit_ready: bool = False,
        dynamic_resume_clear: bool = False,
        d_probe_start_m: Optional[float] = None,
    ) -> PolicyDecision:
        evs: List[Dict[str, Any]] = []
        flags: Dict[str, Any] = {}

        if arrived:
            return self._commit(now, ARRIVED, BEH_SAFE_STOP, "ARRIVED", CORRIDOR_NORMAL, lateral_error, evs, flags)

        if emergency or sensor_invalid or map_invalid:
            reason = "EMERGENCY" if emergency else ("SIL_INPUT_INVALID" if sensor_invalid else "MAP_INVALID")
            return self._commit(now, SAFE_STOP, BEH_SAFE_STOP, reason, CORRIDOR_NORMAL, lateral_error, evs, flags)

        if not nav_active:
            return self._commit(now, IDLE, BEH_FOLLOW, "NAV_INACTIVE", CORRIDOR_NORMAL, lateral_error, evs, flags)

        if self.recovery_loop(now):
            self._emit("RECOVERY_LOOP")
            return self._commit(now, SAFE_STOP, BEH_SAFE_STOP, "RECOVERY_LOOP", CORRIDOR_NORMAL, lateral_error, evs, flags)

        if self.replan_loop(now):
            self._emit("REPLAN_LOOP")
            flags["replan_loop"] = True
            return self._commit(now, SAFE_STOP, BEH_SAFE_STOP, "REPLAN_LOOP", CORRIDOR_NORMAL, lateral_error, evs, flags)

        if self.oscillation_loop(now):
            self._emit("OSCILLATION_LOOP")
            flags["oscillation"] = True
            # Hold wait briefly instead of thrashing
            return self._commit(
                now, WAIT_FOR_CLEARANCE, BEH_WAIT, "OSCILLATION_LOOP→WAIT", CORRIDOR_CAUTION, lateral_error, evs, flags
            )

        goal_behind = abs(_wrap_pi(goal_herr)) > 2.0 and abs(heading_error) > 1.0
        scene = self._classify_scene(
            front_near=front_near,
            left_free=left_free,
            right_free=right_free,
            dynamic_short=dynamic_short,
            dynamic_long=dynamic_long,
            collision=collision,
            goal_behind=goal_behind,
            forward_feasible=forward_feasible,
            path_valid=path_valid,
        )
        flags["scene"] = scene

        # Deadlock: planner wants motion, safety zeros, progress stalled
        if planned_rejected_by_safety and safety_zero and abs(state_vx) < 0.02 and stuck_s > 1.0:
            if self.deadlock_since is None:
                self.deadlock_since = now
            elif now - self.deadlock_since >= DEADLOCK_HOLD_S:
                self._emit("NAVIGATION_DEADLOCK", stuck_s=stuck_s)
                flags["deadlock"] = True
                return self._commit(now, REPLAN, BEH_REPLAN, "NAVIGATION_DEADLOCK→REPLAN", CORRIDOR_NORMAL, lateral_error, evs, flags)
        else:
            self.deadlock_since = None

        # Velocity but path stalled → approach / avoid cue
        if state_vx > 0.05 and path_progress_rate < 0.02 and front_near < DEFAULT_GEOM.front_clear_m + 0.5:
            flags["VELOCITY_BUT_PATH_STALLED"] = True

        abs_h = abs(heading_error)
        state = FOLLOW_GLOBAL
        behavior = BEH_FOLLOW
        reason = "FOLLOW_OK"
        allow_compare = False
        allow_replan = False
        allow_recovery = False

        # Priority ladder
        if collision and rear_near > DEFAULT_GEOM.rear_stop_m + 0.2 and not rotation_safe:
            state, behavior, reason = RECOVERY, BEH_RECOVERY, "COLLISION_TRAP"
            allow_recovery = True
        elif abs_h > 1.15 and scene == "GOAL_BEHIND":
            if rotation_safe:
                state = TURN_IN_PLACE if abs_h > 1.4 else ALIGN
                behavior = BEH_TURN if state == TURN_IN_PLACE else BEH_ALIGN
                reason = "GOAL_BEHIND→ALIGN"
            else:
                state, behavior, reason = REPOSITION, BEH_REPOSITION, "GOAL_BEHIND→REPOSITION"
        elif abs_h > 1.15 and scene not in ("FRONT_BLOCKED", "BLOCK_LEFT_WIDE", "BLOCK_RIGHT_WIDE", "BLOCK_BOTH_WIDE", "BLOCK_NARROW"):
            if rotation_safe:
                state = TURN_IN_PLACE if abs_h > 1.4 else ALIGN
                behavior = BEH_TURN if state == TURN_IN_PLACE else BEH_ALIGN
                reason = "HEADING_ALIGN"
            else:
                state, behavior, reason = REPOSITION, BEH_REPOSITION, "ALIGN_UNSAFE"
        elif dynamic_short and front_near < DEFAULT_GEOM.front_cost_m + 0.35 and not dynamic_resume_clear:
            state, behavior, reason = WAIT_FOR_CLEARANCE, BEH_WAIT, "DYNAMIC_SHORT→WAIT"
        elif dynamic_resume_clear and self.state == WAIT_FOR_CLEARANCE and forward_feasible:
            state, behavior, reason = FOLLOW_GLOBAL, BEH_FOLLOW, "DYNAMIC_CLEAR→RESUME"
            flags["dynamic_resume"] = True
        elif dynamic_long and not forward_feasible:
            state, behavior, reason = REPLAN, BEH_REPLAN, "DYNAMIC_LONG→REPLAN"
            allow_replan = True
        elif scene in ("BLOCK_LEFT_WIDE", "BLOCK_RIGHT_WIDE", "BLOCK_BOTH_WIDE", "BLOCK_NARROW", "FRONT_BLOCKED", "FORWARD_INFEASIBLE"):
            state = LOCAL_AVOID
            allow_compare = True
            reason = f"SCENE_{scene}"
            # Prefer hold current avoid side
            if self.avoid_side == "LEFT" and now < self.avoid_hold_until and left_free > 0.5:
                behavior = BEH_AVOID_LEFT
                reason = "HOLD_AVOID_LEFT"
            elif self.avoid_side == "RIGHT" and now < self.avoid_hold_until and right_free > 0.5:
                behavior = BEH_AVOID_RIGHT
                reason = "HOLD_AVOID_RIGHT"
            elif local_decision == "LEFT":
                behavior = BEH_AVOID_LEFT
            elif local_decision == "RIGHT":
                behavior = BEH_AVOID_RIGHT
            elif left_free > right_free + 0.45:
                behavior = BEH_AVOID_LEFT
            elif right_free > left_free + 0.45:
                behavior = BEH_AVOID_RIGHT
            else:
                behavior = BEH_AVOID_LEFT if left_free >= right_free else BEH_AVOID_RIGHT
                reason = f"LOCAL_AVOID|{scene}"
        elif scene == "APPROACH" or flags.get("VELOCITY_BUT_PATH_STALLED"):
            state, behavior, reason = OBSTACLE_APPROACH, BEH_CAUTION, "OBSTACLE_APPROACH"
            allow_compare = front_near < DEFAULT_GEOM.front_cost_m + 0.55
        elif str(avoidance_phase or "").upper() == "SIDE_COMMIT" and commit_ready:
            state = LOCAL_AVOID
            allow_compare = not self.commitment.active
            reason = "SIDE_COMMIT→LOCAL_AVOID"
            flags["side_commit"] = True
        elif str(avoidance_phase or "").upper() == "SIDE_PROBE" or (
            side_probe_active and future_collision and readiness_signal in ("WARNING", "PREDICTED")
        ):
            state, behavior, reason = SIDE_PROBE, BEH_CAUTION, "SIDE_PROBE"
            allow_compare = True
            flags["side_probe_active"] = True
            flags["future_collision"] = True
        elif str(avoidance_phase or "").upper() == "FUTURE_PREVIEW" or (
            future_collision
            and first_collision_distance_m is not None
            and d_probe_start_m is not None
            and first_collision_distance_m >= d_probe_start_m
        ):
            state, behavior, reason = FUTURE_PREVIEW, BEH_CAUTION, "FUTURE_PREVIEW"
            flags["future_collision"] = True
            flags["detected_only"] = True
        elif approach_active or (
            future_collision
            and first_collision_distance_m is not None
            and required_avoidance_distance_m is not None
            and first_collision_distance_m < required_avoidance_distance_m
        ):
            state, behavior, reason = OBSTACLE_APPROACH, BEH_CAUTION, "FUTURE_COLLISION→OBSTACLE_APPROACH"
            allow_compare = True
            flags["future_collision"] = True
            flags["first_collision_distance_m"] = first_collision_distance_m
            flags["required_avoidance_distance_m"] = required_avoidance_distance_m
        elif front_near < DEFAULT_GEOM.front_clear_m + 0.5:
            state, behavior, reason = CAUTION, BEH_CAUTION, "CAUTION"
        elif self.obstacle_passed or (
            self.state == LOCAL_AVOID and front_near > DEFAULT_GEOM.front_clear_m * 0.9 and lateral_error > CORRIDOR_NORMAL
        ):
            state, behavior, reason = PATH_RECAPTURE, BEH_RECAPTURE, "PATH_RECAPTURE"
            self.obstacle_passed = False
        elif not path_valid or stuck_s >= 8.0:
            state, behavior, reason = REPLAN, BEH_REPLAN, "STUCK_OR_PATH→REPLAN"
            allow_replan = True
        else:
            state, behavior, reason = FOLLOW_GLOBAL, BEH_FOLLOW, "FOLLOW_GLOBAL"

        self._was_dynamic_short = bool(dynamic_short)

        # Maneuver mode feedback (active turn / avoid wins if higher priority)
        mm = (maneuver_mode or "").upper()
        if mm in ("ALIGN", "TURN_IN_PLACE") and PRIORITY.get(ALIGN, 9) <= PRIORITY.get(state, 9):
            if state not in (SAFE_STOP, RECOVERY, LOCAL_AVOID) or abs_h > 1.0:
                state = TURN_IN_PLACE if mm == "TURN_IN_PLACE" else ALIGN
                behavior = BEH_TURN if state == TURN_IN_PLACE else BEH_ALIGN
                reason = f"ACTIVE_{mm}"
                allow_compare = False
        if mm in ("LOCAL_LEFT", "LOCAL_RIGHT"):
            state = LOCAL_AVOID
            behavior = BEH_AVOID_LEFT if mm == "LOCAL_LEFT" else BEH_AVOID_RIGHT
            # STEP 3C: active commitment locks ordinary side re-vote.
            # Feasibility still evaluated via commitment_active in ManeuverFSM.
            allow_compare = not self.commitment.active
            reason = f"ACTIVE_{mm}"
            self.avoid_side = "LEFT" if mm == "LOCAL_LEFT" else "RIGHT"
            if now >= self.avoid_hold_until:
                self.avoid_hold_until = now + MIN_AVOID_HOLD_S
        if mm == "REVERSE_ESCAPE":
            state, behavior, reason = RECOVERY, BEH_RECOVERY, "ACTIVE_REVERSE"
            allow_recovery = True
        if mm == "WAIT_FOR_CLEARANCE":
            state, behavior, reason = WAIT_FOR_CLEARANCE, BEH_WAIT, "ACTIVE_WAIT"
        if mm == "REPOSITION":
            state, behavior, reason = REPOSITION, BEH_REPOSITION, "ACTIVE_REPOSITION"
        if mm == "POST_TURN":
            state, behavior, reason = PATH_RECAPTURE, BEH_RECAPTURE, "POST_TURN→RECAPTURE"
        if mm == "SAFE_STOP":
            state, behavior, reason = SAFE_STOP, BEH_SAFE_STOP, "ACTIVE_SAFE_STOP"

        # Deviation exceeded → recapture or replan
        corridor_preview = self._corridor_for(state, lateral_error, now)
        if corridor_preview.exceeded and state not in (RECOVERY, SAFE_STOP, ALIGN, TURN_IN_PLACE):
            if path_valid:
                state, behavior, reason = PATH_RECAPTURE, BEH_RECAPTURE, "LOCAL_DEVIATION_EXCEEDED→RECAPTURE"
            else:
                state, behavior, reason = REPLAN, BEH_REPLAN, "LOCAL_DEVIATION_EXCEEDED→REPLAN"
                allow_replan = True

        # Recovery only when truly stuck and allowed
        if state == RECOVERY:
            allow_recovery = recovery_attempts < 3
            if not allow_recovery:
                state, behavior, reason = SAFE_STOP, BEH_SAFE_STOP, "RECOVERY_EXHAUSTED"

        # Behavior hysteresis
        if now < self.hold_until and self.state not in (SAFE_STOP, FAILED):
            # keep state unless higher priority emergency already handled
            if PRIORITY.get(state, 99) >= PRIORITY.get(self.state, 99):
                state = self.state
                behavior = self.behavior
                reason = f"HOLD|{reason}"

        return self._commit(
            now,
            state,
            behavior,
            reason,
            corridor_preview.half_width,
            lateral_error,
            evs,
            flags,
            allow_compare=allow_compare,
            allow_replan=allow_replan,
            allow_recovery=allow_recovery,
            scene=scene,
        )

    def _commit(
        self,
        now: float,
        state: str,
        behavior: str,
        reason: str,
        half_width: float,
        lateral: float,
        evs: List[Dict[str, Any]],
        flags: Dict[str, Any],
        *,
        allow_compare: bool = False,
        allow_replan: bool = False,
        allow_recovery: bool = False,
        scene: str = "OPEN",
    ) -> PolicyDecision:
        if state != self.state:
            evs.append(self._emit("POLICY_STATE_CHANGE", from_state=self.state, to_state=state, reason=reason))
            self.state_enter_ts = now
            self.hold_until = now + MIN_BEHAVIOR_HOLD_S
            self.history.append({"ts": now, "state": state, "reason": reason})
            if len(self.history) > 40:
                self.history = self.history[-40:]
        if behavior != self.behavior:
            evs.append(self._emit("BEHAVIOR_SWITCHED", from_b=self.behavior, to_b=behavior, reason=reason))
            self.behavior_enter_ts = now
            if behavior in (BEH_AVOID_LEFT, BEH_AVOID_RIGHT):
                side = "LEFT" if behavior == BEH_AVOID_LEFT else "RIGHT"
                self.note_side(now, side)
                self.avoid_side = side
                self.avoid_hold_until = max(self.avoid_hold_until, now + MIN_AVOID_HOLD_S)
                evs.append(self._emit("LOCAL_AVOID_STARTED", side=side))
                evs.append(self._emit(f"{side}_SELECTED", reason=reason))
            if behavior == BEH_RECAPTURE:
                evs.append(self._emit("RECAPTURE_STARTED"))
            if behavior == BEH_REPLAN:
                self.note_replan(now)
            if behavior == BEH_RECOVERY:
                self.note_recovery(now)

        self.state = state
        self.behavior = behavior
        self.reason = reason
        profile = self._profile_for(state)
        corridor = self._corridor_for(state, lateral, now)
        # Override half_width hint
        corridor.half_width = max(corridor.half_width, half_width) if state in (LOCAL_AVOID, OBSTACLE_APPROACH) else corridor.half_width

        hard_capture = state in (FOLLOW_GLOBAL, CAUTION, PATH_RECAPTURE, ALIGN, TURN_IN_PLACE)
        allow_side = allow_compare and state in (LOCAL_AVOID, OBSTACLE_APPROACH, CAUTION)
        if self.commitment.active:
            # Ordinary re-selection locked until STEP 3E authorization
            allow_side = False
        c = self.commitment
        d = PolicyDecision(
            state=state,
            behavior=behavior,
            reason=reason,
            profile=profile,
            corridor=corridor,
            allow_side_compare=allow_side,
            allow_replan=allow_replan or state == REPLAN,
            allow_recovery=allow_recovery or state == RECOVERY,
            path_follow_weight=self._path_follow_weight(profile),
            require_capture_hard=hard_capture,
            max_deviation_m=corridor.half_width,
            scene=scene,
            events=list(self.events[-8:]),
            flags=flags,
            commitment_active=bool(c.active),
            committed_side=str(c.side or "NONE"),
            commitment_phase=str(c.phase or PHASE_NONE),
            commitment_hard_fail=bool(c.hard_fail),
            commitment_failure_reason=str(c.failure_reason or FAIL_NONE),
            side_switch_authorized=bool(
                self.switch_token is not None and self.switch_token.is_live(now)
            ),
            side_switch_authorization_status=(
                AUTH_STATUS_AUTHORIZED
                if (self.switch_token is not None and self.switch_token.is_live(now))
                else (
                    str(self.last_switch_decision.status)
                    if self.last_switch_decision is not None
                    else str(c.authorization_status or AUTH_NONE)
                )
            ),
        )
        self.last_decision = d
        return d

    def mark_obstacle_passed(self) -> None:
        self.obstacle_passed = True
        self._emit("OBSTACLE_PASSED")

    def to_telemetry(self) -> Dict[str, Any]:
        d = self.last_decision
        now = time.time()
        sw = self.last_switch_decision.to_dict() if self.last_switch_decision else {}
        return {
            "state": self.state,
            "behavior": self.behavior,
            "reason": self.reason,
            "time_in_state": round(self.time_in_state(now), 2),
            "avoid_side": self.avoid_side,
            "history": self.history[-12:],
            "events": self.events[-12:],
            "decision": d.to_dict() if d else {},
            "commitment": self.commitment.to_dict(now),
            "side_switch": sw,
            "switch_token": self.switch_token.to_dict() if self.switch_token else None,
            "loops": {
                "oscillation_flips": len(self.w_sign_flips),
                "replan_count": len(self.replan_times),
                "recovery_count": len(self.recovery_times),
                "side_switch_count": len(self.executed_switch_times),
            },
        }
