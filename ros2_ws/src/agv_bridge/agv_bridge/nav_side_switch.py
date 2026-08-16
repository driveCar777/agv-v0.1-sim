"""Side Switch Authorization — STEP 3E (Policy-owned gate).

Consumes ProbeBundle + Commitment + Safety/dynamic flags.
Does NOT rollout trajectories. Does NOT write vx/w.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from agv_bridge.nav_commitment import AvoidanceCommitment
from agv_bridge.nav_probe import (
    STATUS_INVALID,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_VALID,
    ProbeBundle,
    ProbeResult,
)

# ---- Authorization status ----
AUTH_STATUS_NONE = "NONE"
AUTH_STATUS_DENIED = "DENIED"
AUTH_STATUS_AUTHORIZED = "AUTHORIZED"
AUTH_STATUS_WAITING = "WAITING_EVIDENCE"
AUTH_STATUS_EXPIRED = "EXPIRED"
AUTH_STATUS_EXECUTED = "EXECUTED"

# ---- Reasons (minimal set) ----
REASON_NONE = "NONE"
REASON_NO_COMMITMENT = "NO_COMMITMENT"
REASON_CURRENT_SIDE_STILL_VALID = "CURRENT_SIDE_STILL_VALID"
REASON_CURRENT_PROBE_INVALID = "CURRENT_SIDE_PROBE_INVALID"
REASON_CURRENT_HARD_INFEASIBLE = "CURRENT_SIDE_HARD_INFEASIBLE"
REASON_CURRENT_HARD_COLLISION = "CURRENT_SIDE_HARD_COLLISION"
REASON_ALT_PROBE_INVALID = "ALTERNATIVE_PROBE_INVALID"
REASON_UNKNOWN_PROBE = "UNKNOWN_PROBE"
REASON_STALE_PROBE = "STALE_PROBE"
REASON_TURN_NOT_SAFE = "TURN_NOT_SAFE"
REASON_SAFETY_DENIED = "SAFETY_DENIED"
REASON_DYNAMIC_WAIT = "DYNAMIC_WAIT_REQUIRED"
REASON_COOLDOWN = "SIDE_SWITCH_COOLDOWN"
REASON_OSCILLATION = "SIDE_SWITCH_OSCILLATION"
REASON_SCENE = "SCENE_CHANGED"
REASON_SIGNATURE = "OBSTACLE_SIGNATURE_MISMATCH"
REASON_PATH_INVALID = "PATH_INVALID"
REASON_EMERGENCY = "EMERGENCY"
REASON_TOKEN_EXPIRED = "AUTHORIZATION_EXPIRED"
REASON_STATE_MISMATCH = "AUTHORIZATION_STATE_MISMATCH"
REASON_EXEC_FAILED = "SIDE_SWITCH_EXECUTION_FAILED"

# ---- Config (centralized) ----
SIDE_SWITCH_COOLDOWN_S = 2.0
SIDE_SWITCH_AUTH_TTL_S = 0.40
OSCILLATION_MAX_SWITCHES = 3
OSCILLATION_WINDOW_S = 8.0
PROBE_FRESH_MAX_AGE_S = 0.55


@dataclass
class SideSwitchAuthorizationToken:
    authorized: bool = False
    from_side: str = "NONE"
    to_side: str = "NONE"
    reason: str = REASON_NONE
    obstacle_signature: Optional[str] = None
    created_at: float = 0.0
    expires_at: float = 0.0
    consumed: bool = False

    def is_live(self, now: float) -> bool:
        return (
            bool(self.authorized)
            and (not self.consumed)
            and float(now) <= float(self.expires_at)
            and self.from_side in ("LEFT", "RIGHT")
            and self.to_side in ("LEFT", "RIGHT")
            and self.from_side != self.to_side
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "authorized": self.authorized,
            "from_side": self.from_side,
            "to_side": self.to_side,
            "reason": self.reason,
            "obstacle_signature": self.obstacle_signature,
            "created_at": self.created_at,
            "expires_at": self.expires_at,
            "consumed": self.consumed,
        }


@dataclass
class SideSwitchDecision:
    status: str = AUTH_STATUS_NONE
    authorized: bool = False
    from_side: str = "NONE"
    to_side: str = "NONE"
    primary_reason: str = REASON_NONE
    failed_gates: List[str] = field(default_factory=list)
    gates: Dict[str, bool] = field(default_factory=dict)
    raw_candidate_side: str = "NONE"
    current_probe_status: str = "NONE"
    alternative_probe_status: str = "NONE"
    turn_probe_status: str = "NONE"
    evidence_consistent: bool = True
    token: Optional[SideSwitchAuthorizationToken] = None
    authorization_ms: float = 0.0
    timestamp: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "implemented": True,
            "status": self.status,
            "authorized": self.authorized,
            "authorization": self.authorized,
            "from_side": self.from_side,
            "to_side": self.to_side,
            "current_side": self.from_side,
            "candidate_side": self.raw_candidate_side or self.to_side,
            "reason": self.primary_reason,
            "primary_reason": self.primary_reason,
            "failed_gates": list(self.failed_gates),
            "gates": dict(self.gates),
            "current_probe_status": self.current_probe_status,
            "alternative_probe_status": self.alternative_probe_status,
            "candidate_probe": self.alternative_probe_status,
            "turn_probe_status": self.turn_probe_status,
            "evidence_consistent": self.evidence_consistent,
            "token": self.token.to_dict() if self.token else None,
            "authorization_ms": round(self.authorization_ms, 3),
            "timestamp": self.timestamp,
            "note": "Policy-owned gate; Probe INVALID never authorizes that side",
        }


def _side_probe(bundle: Optional[ProbeBundle], side: str) -> Optional[ProbeResult]:
    if bundle is None:
        return None
    s = str(side or "").upper()
    if s == "LEFT":
        return bundle.left
    if s == "RIGHT":
        return bundle.right
    if s == "FORWARD":
        return bundle.forward
    if s in ("BACKWARD", "REVERSE"):
        return bundle.backward
    if s in ("TURN", "TURN_IN_PLACE"):
        return bundle.turn_in_place
    return None


def _probe_status(pr: Optional[ProbeResult]) -> str:
    if pr is None:
        return STATUS_UNKNOWN
    return str(pr.status or STATUS_UNKNOWN)


def _core_sig(s: str) -> str:
    parts = str(s).split("_")
    return "_".join(parts[:3]) if len(parts) >= 3 else str(s)


def _current_side_hard_failed(
    *,
    commitment: AvoidanceCommitment,
    current_probe: Optional[ProbeResult],
) -> tuple:
    if commitment.hard_fail:
        return True, REASON_CURRENT_HARD_INFEASIBLE
    if current_probe is None:
        return False, REASON_NONE
    st = _probe_status(current_probe)
    if st == STATUS_INVALID:
        if current_probe.hard_collision or current_probe.collision:
            return True, REASON_CURRENT_HARD_COLLISION
        return True, REASON_CURRENT_PROBE_INVALID
    return False, REASON_NONE


def authorize_side_switch(
    *,
    now: float,
    commitment: AvoidanceCommitment,
    probe_bundle: Optional[ProbeBundle],
    raw_candidate_side: Optional[str] = None,
    dynamic_short: bool = False,
    emergency: bool = False,
    safety_zero: bool = False,
    planned_rejected_by_safety: bool = False,
    path_valid: bool = True,
    switch_times: Optional[List[float]] = None,
    cooldown_s: float = SIDE_SWITCH_COOLDOWN_S,
    auth_ttl_s: float = SIDE_SWITCH_AUTH_TTL_S,
    probe_fresh_s: float = PROBE_FRESH_MAX_AGE_S,
) -> SideSwitchDecision:
    """O(1) boolean gates over existing evidence — no new rollouts."""
    t0 = time.perf_counter()
    gates: Dict[str, bool] = {
        "commitment_active": False,
        "current_side_failed": False,
        "alternative_valid": False,
        "turn_valid": False,
        "probe_fresh": False,
        "safety_ok": False,
        "dynamic_ok": False,
        "cooldown_ok": False,
        "oscillation_ok": False,
        "scene_ok": False,
        "path_ok": False,
        "signature_ok": False,
    }
    failed: List[str] = []
    decision = SideSwitchDecision(timestamp=now, gates=gates, failed_gates=failed)

    if emergency:
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_EMERGENCY
        failed.append("emergency")
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    if not commitment.active or commitment.side not in ("LEFT", "RIGHT"):
        decision.status = AUTH_STATUS_NONE
        decision.primary_reason = REASON_NO_COMMITMENT
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    gates["commitment_active"] = True
    from_side = commitment.side
    to_side = "RIGHT" if from_side == "LEFT" else "LEFT"
    decision.from_side = from_side
    decision.to_side = to_side
    raw = str(raw_candidate_side or "").upper()
    if raw not in ("LEFT", "RIGHT"):
        raw = to_side
    decision.raw_candidate_side = raw

    if probe_bundle is None:
        decision.status = AUTH_STATUS_WAITING
        decision.primary_reason = REASON_UNKNOWN_PROBE
        failed.append("probe_missing")
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    try:
        age = float(probe_bundle.snapshot.age_s(now))
    except Exception:
        age = 0.0
    gates["probe_fresh"] = age <= probe_fresh_s
    if not gates["probe_fresh"]:
        failed.append("probe_fresh")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_STALE_PROBE
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    cur_p = _side_probe(probe_bundle, from_side)
    alt_p = _side_probe(probe_bundle, to_side)
    turn_p = probe_bundle.turn_in_place
    decision.current_probe_status = _probe_status(cur_p)
    decision.alternative_probe_status = _probe_status(alt_p)
    decision.turn_probe_status = _probe_status(turn_p)

    if raw in ("LEFT", "RIGHT") and raw != from_side:
        decision.evidence_consistent = _probe_status(alt_p) == STATUS_VALID
    else:
        decision.evidence_consistent = True

    failed_ok, fail_reason = _current_side_hard_failed(
        commitment=commitment, current_probe=cur_p
    )
    gates["current_side_failed"] = failed_ok
    if not failed_ok:
        if decision.current_probe_status == STATUS_VALID:
            decision.primary_reason = REASON_CURRENT_SIDE_STILL_VALID
            failed.append("current_side_failed")
        elif decision.current_probe_status == STATUS_UNKNOWN:
            decision.primary_reason = REASON_UNKNOWN_PROBE
            failed.append("current_probe")
        elif decision.current_probe_status == STATUS_STALE:
            decision.primary_reason = REASON_STALE_PROBE
            failed.append("current_probe")
        else:
            decision.primary_reason = REASON_CURRENT_SIDE_STILL_VALID
            failed.append("current_side_failed")
        decision.status = AUTH_STATUS_DENIED
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    alt_st = decision.alternative_probe_status
    gates["alternative_valid"] = alt_st == STATUS_VALID
    if alt_st == STATUS_INVALID:
        failed.append("alternative_probe")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_ALT_PROBE_INVALID
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision
    if alt_st == STATUS_UNKNOWN:
        failed.append("alternative_probe")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_UNKNOWN_PROBE
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision
    if alt_st == STATUS_STALE:
        failed.append("alternative_probe")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_STALE_PROBE
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision
    if not gates["alternative_valid"]:
        failed.append("alternative_probe")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_ALT_PROBE_INVALID
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    turn_st = decision.turn_probe_status
    gates["turn_valid"] = turn_st == STATUS_VALID
    if not gates["turn_valid"]:
        failed.append("turn_probe")
        decision.status = AUTH_STATUS_DENIED
        if turn_st == STATUS_STALE:
            decision.primary_reason = REASON_STALE_PROBE
        elif turn_st == STATUS_UNKNOWN:
            decision.primary_reason = REASON_UNKNOWN_PROBE
        else:
            decision.primary_reason = REASON_TURN_NOT_SAFE
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    gates["safety_ok"] = (not safety_zero) and (not planned_rejected_by_safety)
    if not gates["safety_ok"]:
        failed.append("safety")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_SAFETY_DENIED
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    gates["dynamic_ok"] = not bool(dynamic_short)
    if not gates["dynamic_ok"]:
        failed.append("dynamic")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_DYNAMIC_WAIT
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    gates["path_ok"] = bool(path_valid)
    if not gates["path_ok"]:
        failed.append("path")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_PATH_INVALID
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    sig = commitment.obstacle_signature
    try:
        snap_sig = probe_bundle.snapshot.signature()
    except Exception:
        snap_sig = None
    gates["scene_ok"] = True
    gates["signature_ok"] = True
    if sig and snap_sig:
        try:
            gates["scene_ok"] = abs(
                float(sig.split("_")[0][1:]) - float(snap_sig.split("_")[0][1:])
            ) < 1.5
        except Exception:
            gates["scene_ok"] = True
    if not gates["scene_ok"]:
        failed.append("scene")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_SCENE
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    last_sw = commitment.last_switch_at
    gates["cooldown_ok"] = last_sw is None or (now - float(last_sw)) >= cooldown_s
    if not gates["cooldown_ok"]:
        failed.append("cooldown")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_COOLDOWN
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    times = [t for t in (switch_times or []) if now - float(t) <= OSCILLATION_WINDOW_S]
    gates["oscillation_ok"] = len(times) < OSCILLATION_MAX_SWITCHES
    if not gates["oscillation_ok"]:
        failed.append("oscillation")
        decision.status = AUTH_STATUS_DENIED
        decision.primary_reason = REASON_OSCILLATION
        decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
        return decision

    token = SideSwitchAuthorizationToken(
        authorized=True,
        from_side=from_side,
        to_side=to_side,
        reason=fail_reason or REASON_CURRENT_PROBE_INVALID,
        obstacle_signature=sig or snap_sig,
        created_at=now,
        expires_at=now + auth_ttl_s,
        consumed=False,
    )
    decision.authorized = True
    decision.status = AUTH_STATUS_AUTHORIZED
    decision.primary_reason = fail_reason or REASON_CURRENT_PROBE_INVALID
    decision.token = token
    decision.authorization_ms = (time.perf_counter() - t0) * 1000.0
    return decision


def validate_token_for_execution(
    token: Optional[SideSwitchAuthorizationToken],
    *,
    now: float,
    current_side: str,
    commitment_side: str,
    obstacle_signature: Optional[str] = None,
) -> tuple:
    if token is None or not token.authorized:
        return False, REASON_NONE
    if token.consumed:
        return False, REASON_NONE
    if now > token.expires_at:
        return False, REASON_TOKEN_EXPIRED
    if str(current_side).upper() != str(token.from_side).upper():
        return False, REASON_STATE_MISMATCH
    if str(commitment_side).upper() != str(token.from_side).upper():
        return False, REASON_STATE_MISMATCH
    if token.obstacle_signature and obstacle_signature:
        if _core_sig(token.obstacle_signature) != _core_sig(obstacle_signature):
            return False, REASON_SIGNATURE
    return True, REASON_NONE
