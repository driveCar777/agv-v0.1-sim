"""Phase 4 STEP 3B — Telemetry First (read-only observability).

Observes existing Policy / Selector / FSM / Controller / Safety outputs.
Does NOT change navigation decisions.

Placeholders for Commitment / Probe / Side-switch authorization remain
NOT_IMPLEMENTED until STEP 3C–3E.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

EVENT_MAX = 200
NA = "NOT_AVAILABLE"
NI = "NOT_IMPLEMENTED"


def _finite(v: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if v is None:
            return default
        f = float(v)
        if math.isnan(f) or math.isinf(f):
            return default
        return f
    except Exception:
        return default


def _side_from_token(token: Any) -> str:
    s = str(token or "").upper()
    if "LEFT" in s:
        return "LEFT"
    if "RIGHT" in s:
        return "RIGHT"
    if s in ("NONE", "", "FORWARD", "NULL"):
        return "NONE"
    return "NONE"


def _side_from_w(w: Any, thr: float = 0.05) -> str:
    f = _finite(w, 0.0) or 0.0
    if abs(f) < thr:
        return "NONE"
    return "LEFT" if f > 0 else "RIGHT"


def _cand_row(row: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    r = dict(row or {})
    return {
        "feasible": r.get("feasible") if "feasible" in r else NA,
        "cost": _finite(r.get("cost")),
        "clearance": _finite(r.get("clr") if "clr" in r else r.get("clearance")),
        "progress": _finite(r.get("progress")),
        "capture": _finite(r.get("capture")),
        "heading": _finite(r.get("heading")),
        "quality": r.get("quality") if r.get("quality") is not None else NA,
        "reason": r.get("reason") if r.get("reason") is not None else NA,
    }


def placeholder_commitment() -> Dict[str, Any]:
    return {
        "implemented": False,
        "active": False,
        "side": "NONE",
        "phase": "NONE",
        "reason": NI,
        "started_at": None,
        "age_s": 0.0,
        "obstacle_signature": None,
        "failure_reason": "NONE",
        "soft_fail_streak": 0,
        "hard_fail": False,
        "last_switch_at": None,
        "switch_count": 0,
        "release_reason": "NONE",
        "note": "STEP3C will activate AvoidanceCommitment; do not treat avoid_side as commitment",
    }


def placeholder_probe() -> Dict[str, Any]:
    stub = {"status": NI}
    return {
        "implemented": False,
        "status": NI,
        "forward": dict(stub),
        "backward": dict(stub),
        "left": dict(stub),
        "right": dict(stub),
        "turn_in_place": dict(stub),
        "note": "STEP3D ProbeEngine will fill VALID/INVALID/UNKNOWN/STALE",
    }


def placeholder_side_switch() -> Dict[str, Any]:
    return {
        "implemented": False,
        "authorized": False,
        "authorization_status": NI,
        "status": NI,
        "current_side": "NONE",
        "candidate_side": "NONE",
        "reason": NI,
        "failure_reason": "NONE",
        "evidence": {},
        "last_switch_at": None,
        "switch_count": 0,
        "note": "authorized=false is PLACEHOLDER; not Policy gate yet. See SIDE_SWITCH_OBSERVED for actual flips",
    }


def placeholder_obstacle_snapshot() -> Dict[str, Any]:
    return {
        "available": False,
        "timestamp": None,
        "source": NI,
        "count": 0,
        "front": {},
        "rear": {},
        "left": {},
        "right": {},
        "signature": None,
        "note": "STEP3D will bind same-tick ObstacleSnapshot; 3B does not recompute obstacles",
    }


class Phase4ObserveTracker:
    """Detect state changes and emit structured observe-only events."""

    def __init__(self, maxlen: int = EVENT_MAX) -> None:
        self.events: Deque[Dict[str, Any]] = deque(maxlen=maxlen)
        self._prev_fsm: Optional[str] = None
        self._prev_sel_side: Optional[str] = None
        self._prev_stop: Optional[str] = None
        self._prev_safety_override: Optional[bool] = None
        self.switch_count = 0
        self.last_switch_at: Optional[float] = None
        self.side_selected_once = False
        self.trace_id: Optional[str] = None
        self.last_emitted: List[Dict[str, Any]] = []

    def reset(self, *, trace_id: Optional[str] = None) -> None:
        self.events.clear()
        self._prev_fsm = None
        self._prev_sel_side = None
        self._prev_stop = None
        self._prev_safety_override = None
        self.switch_count = 0
        self.last_switch_at = None
        self.side_selected_once = False
        self.trace_id = trace_id
        self.last_emitted = []

    def _push(self, event_type: str, *, source: str, old: Any, new: Any, reason: str = "", **meta: Any) -> None:
        row = {
            "timestamp": time.time(),
            "event_type": event_type,
            "event": event_type,
            "source": source,
            "old_value": old,
            "new_value": new,
            "reason": reason or "",
            "metadata": meta or {},
            "trace_id": self.trace_id,
        }
        self.events.append(row)

    def observe(
        self,
        *,
        fsm_mode: str,
        selector_side: str,
        selector_reason: str,
        stop_reason: str,
        safety_override: bool,
        mppi_vx: float,
        mppi_w: float,
        safe_vx: float,
        safe_w: float,
        front_near: float,
        rear_near: float,
        input_vx: float,
        input_w: float,
    ) -> List[Dict[str, Any]]:
        emitted: List[Dict[str, Any]] = []
        mode = (fsm_mode or "").upper()
        sel = _side_from_token(selector_side)
        stop = str(stop_reason or "NONE")

        if self._prev_fsm is not None and mode != self._prev_fsm:
            self._push(
                "FSM_TRANSITION",
                source="ManeuverFSM",
                old=self._prev_fsm,
                new=mode,
                reason=selector_reason or "",
            )
            emitted.append(self.events[-1])
        self._prev_fsm = mode

        if sel in ("LEFT", "RIGHT"):
            if self._prev_sel_side in (None, "NONE") and not self.side_selected_once:
                self._push(
                    "SIDE_SELECTED",
                    source="LocalManeuverSelector",
                    old="NONE",
                    new=sel,
                    reason=selector_reason or "",
                )
                emitted.append(self.events[-1])
                self.side_selected_once = True
            elif self._prev_sel_side in ("LEFT", "RIGHT") and sel != self._prev_sel_side:
                self.switch_count += 1
                self.last_switch_at = time.time()
                self._push(
                    "SIDE_SWITCH_OBSERVED",
                    source="LocalManeuverSelector",
                    old=self._prev_sel_side,
                    new=sel,
                    reason=selector_reason or "",
                    authority="LocalManeuverSelector",
                    switch_count=self.switch_count,
                )
                emitted.append(self.events[-1])
            self._prev_sel_side = sel
        elif self._prev_sel_side is None:
            self._prev_sel_side = "NONE"

        if safety_override and self._prev_safety_override is False:
            self._push(
                "SAFETY_OVERRIDE",
                source="apply_safety",
                old="ALLOW",
                new=stop,
                reason=stop,
                input_vx=input_vx,
                input_w=input_w,
                output_vx=safe_vx,
                output_w=safe_w,
                front_distance=front_near,
                rear_distance=rear_near,
                mppi_vx=mppi_vx,
                mppi_w=mppi_w,
            )
            emitted.append(self.events[-1])
        elif (not safety_override) and self._prev_safety_override is True:
            self._push(
                "SAFETY_RELEASE",
                source="apply_safety",
                old=self._prev_stop or "OVERRIDE",
                new=stop,
                reason=stop,
            )
            emitted.append(self.events[-1])
        self._prev_safety_override = bool(safety_override)
        self._prev_stop = stop
        self.last_emitted = emitted
        return emitted


def build_phase4_telemetry(
    *,
    nav_policy: Optional[Dict[str, Any]],
    maneuver: Optional[Dict[str, Any]],
    local_maneuver: Optional[Dict[str, Any]],
    mppi_vx: float,
    mppi_w: float,
    cmd_vx: float,
    cmd_w: float,
    safe_vx: float,
    safe_w: float,
    state_vx: float,
    state_w: float,
    stop_reason: str,
    front_near: float,
    rear_near: float,
    phase: str,
    control_mode: str,
    stuck_s: float,
    recovery_attempts: int,
    tracker: Optional[Phase4ObserveTracker] = None,
    pose_ts: Optional[float] = None,
    obstacle_ts: Optional[float] = None,
    debug_ms: Optional[float] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Assemble Phase4 debug block from already-computed structures (no recompute)."""
    t0 = time.perf_counter()
    pol = dict(nav_policy or {})
    man = dict(maneuver or {})
    lm = dict(local_maneuver or {})
    dec = pol.get("decision") if isinstance(pol.get("decision"), dict) else {}
    # nav_policy from hub.build may already be flattened
    policy_state = pol.get("state") or dec.get("state") or "IDLE"
    policy_behavior = pol.get("behavior") or dec.get("behavior") or "FOLLOW_GLOBAL"
    policy_reason = pol.get("reason") or dec.get("reason") or ""
    avoid_side = _side_from_token(pol.get("avoid_side"))
    allow_compare = pol.get("allow_side_compare")
    if allow_compare is None:
        allow_compare = dec.get("allow_side_compare")
    allow_replan = pol.get("allow_replan")
    if allow_replan is None:
        allow_replan = dec.get("allow_replan")
    allow_recovery = pol.get("allow_recovery")
    if allow_recovery is None:
        allow_recovery = dec.get("allow_recovery")

    sel_token = lm.get("decision") or man.get("decision") or "FORWARD"
    selector_side = _side_from_token(sel_token)
    selector_reason = lm.get("reason") or man.get("reason") or NA
    left_row = _cand_row(lm.get("left") if isinstance(lm.get("left"), dict) else {})
    right_row = _cand_row(lm.get("right") if isinstance(lm.get("right"), dict) else {})
    fwd_row = _cand_row(lm.get("forward") if isinstance(lm.get("forward"), dict) else {})

    fsm_mode = str(man.get("mode") or "").upper() or NA
    fsm_side = _side_from_token(fsm_mode)
    controller_side = _side_from_w(mppi_w)
    actual_side = _side_from_w(state_w)

    in_vx = _finite(cmd_vx, 0.0) or 0.0
    in_w = _finite(cmd_w, 0.0) or 0.0
    s_vx = _finite(safe_vx, 0.0) or 0.0
    s_w = _finite(safe_w, 0.0) or 0.0
    safety_override = abs(in_vx - s_vx) > 0.02 or abs(in_w - s_w) > 0.05
    safety_rejection_observed = abs(in_vx) > 0.05 and abs(s_vx) < 0.02

    # STEP 3C — real commitment from Policy (fallback placeholder if missing)
    commit_src = pol.get("commitment") if isinstance(pol.get("commitment"), dict) else {}
    if commit_src.get("implemented") or commit_src.get("active") is not None or dec.get("commitment_active") is not None:
        commitment = {
            "implemented": True,
            "active": bool(commit_src.get("active", dec.get("commitment_active"))),
            "side": commit_src.get("side") or dec.get("committed_side") or "NONE",
            "phase": commit_src.get("phase") or dec.get("commitment_phase") or "NONE",
            "reason": commit_src.get("reason") or "NONE",
            "started_at": commit_src.get("started_at"),
            "age_s": commit_src.get("age_s", 0.0),
            "obstacle_signature": commit_src.get("obstacle_signature"),
            "failure_reason": commit_src.get("failure_reason")
            or dec.get("commitment_failure_reason")
            or "NONE",
            "soft_fail_streak": commit_src.get("soft_fail_streak", 0),
            "hard_fail": bool(commit_src.get("hard_fail", dec.get("commitment_hard_fail"))),
            "last_switch_at": commit_src.get("last_switch_at"),
            "switch_count": commit_src.get("switch_count", 0),
            "release_reason": commit_src.get("release_reason") or "NONE",
            "authorization_status": commit_src.get("authorization_status")
            or dec.get("side_switch_authorization_status")
            or "NONE",
            "note": "STEP3C AvoidanceCommitment active; side switch auth = STEP3E",
        }
    else:
        commitment = placeholder_commitment()

    # STEP 3D — real ProbeBundle from Policy telemetry attach
    probe_src = pol.get("probe") if isinstance(pol.get("probe"), dict) else {}
    if probe_src.get("implemented"):
        probe = dict(probe_src)
        probe["implemented"] = True
        if "note" not in probe:
            probe["note"] = "STEP3D evidence-only"
    else:
        probe = placeholder_probe()

    # STEP 3E — real SideSwitchDecision from Policy
    sw_src = pol.get("side_switch") if isinstance(pol.get("side_switch"), dict) else {}
    if sw_src.get("implemented") or sw_src.get("status") or sw_src.get("primary_reason"):
        side_switch = dict(sw_src)
        side_switch["implemented"] = True
        side_switch["authorized"] = bool(
            sw_src.get("authorized") or dec.get("side_switch_authorized")
        )
        side_switch["authorization_status"] = (
            sw_src.get("status")
            or sw_src.get("authorization_status")
            or dec.get("side_switch_authorization_status")
            or "NONE"
        )
        side_switch["current_side"] = (
            sw_src.get("from_side")
            or sw_src.get("current_side")
            or (fsm_side if fsm_side != "NONE" else selector_side)
        )
        side_switch["candidate_side"] = sw_src.get("candidate_side") or sw_src.get("to_side") or "NONE"
        side_switch["failure_reason"] = sw_src.get("reason") or sw_src.get("primary_reason") or "NONE"
        if tracker is not None:
            side_switch["switch_count"] = tracker.switch_count
            side_switch["last_switch_at"] = tracker.last_switch_at
        side_switch["note"] = "STEP3E Policy gate; Probe INVALID never authorizes that side"
    else:
        side_switch = placeholder_side_switch()
        side_switch["implemented"] = True
        side_switch["authorized"] = bool(dec.get("side_switch_authorized", False))
        side_switch["authorization_status"] = (
            dec.get("side_switch_authorization_status")
            or commitment.get("authorization_status")
            or "NONE"
        )
        side_switch["current_side"] = fsm_side if fsm_side != "NONE" else selector_side
        side_switch["failure_reason"] = commitment.get("failure_reason") or "NONE"
        if tracker is not None:
            side_switch["switch_count"] = tracker.switch_count
            side_switch["last_switch_at"] = tracker.last_switch_at
        side_switch["note"] = "STEP3E awaiting decision payload"

    ownership = {
        "policy": "NavigationPolicy",
        "candidate_evaluation": "LocalManeuverSelector",
        "side_switch_authority": "NavigationPolicy",
        "side_switch_authority_current": "NavigationPolicy",
        "side_switch_authority_target": "NavigationPolicy",
        "maneuver_execution": "ManeuverFSM",
        "trajectory_control": "DiffDriveMppi",
        "safety": "apply_safety",
        "note": "Candidate≠Authorized≠Executed≠Safe",
    }

    sides = {
        "policy_intent_side": _side_from_token(policy_behavior),
        "policy_avoid_side": avoid_side,
        "commitment_side": commitment.get("side") or "NONE",
        "selector_side": selector_side,
        "fsm_side": fsm_side,
        "controller_side": controller_side,
        "actual_motion_side": actual_side,
        "sources": {
            "policy_intent_side": "NavigationPolicy.behavior",
            "policy_avoid_side": "NavigationPolicy.avoid_side",
            "commitment_side": "AvoidanceCommitment.side",
            "selector_side": "LocalManeuverSelector",
            "fsm_side": "ManeuverFSM.mode",
            "controller_side": "DiffDriveMppi.mppi_w sign",
            "actual_motion_side": "state.w sign",
        },
    }

    policy_block = {
        "state": policy_state,
        "behavior": policy_behavior,
        "reason": policy_reason,
        "scene": pol.get("scene") or (dec.get("scene") if dec else None),
        "profile": pol.get("profile") or dec.get("profile") or {},
        "corridor": pol.get("corridor") or dec.get("corridor") or {},
        "allow_side_compare": allow_compare,
        "allow_replan": allow_replan,
        "allow_recovery": allow_recovery,
        "path_follow_weight": pol.get("path_follow_weight") or dec.get("path_follow_weight"),
        "avoid_side": avoid_side,
        "vx_scale": (pol.get("profile") or dec.get("profile") or {}).get("vx_scale")
        if isinstance(pol.get("profile") or dec.get("profile"), dict)
        else NA,
        "source": "NavigationPolicy",
    }

    selector_block = {
        "selected_side": selector_side,
        "selected_token": sel_token,
        "reason": selector_reason,
        "compare_allowed": allow_compare,
        "hysteresis_state": NA,
        "hold_state": NA,
        "left": left_row,
        "right": right_row,
        "forward": fwd_row,
        "candidate": {"left": left_row, "right": right_row},
        "selected": {"side": selector_side, "reason": selector_reason},
        "source": "LocalManeuverSelector",
        "telemetry_missing_fields": ["hysteresis_state", "hold_state"],
    }

    maneuver_block = {
        "mode": fsm_mode,
        "phase": phase,
        "legacy_phase": man.get("legacy_phase") or phase,
        "force_vx": _finite(man.get("force_vx")),
        "force_w": _finite(man.get("force_w")),
        "side": fsm_side,
        "reason": man.get("reason") or selector_reason,
        "source": "ManeuverFSM",
    }

    controller_block = {
        "mode": control_mode,
        "mppi_vx": _finite(mppi_vx),
        "mppi_w": _finite(mppi_w),
        "cmd_vx_before_safety": _finite(cmd_vx),
        "cmd_w_before_safety": _finite(cmd_w),
        "force_vx": _finite(man.get("force_vx")),
        "force_w": _finite(man.get("force_w")),
        "side": controller_side,
        "source": "DiffDriveMppi",
    }

    safety_block = {
        "stop_reason": stop_reason or "NONE",
        "front_obstacle": str(stop_reason or "").upper() in ("FRONT_OBSTACLE", "STOP_FRONT")
        or ("FRONT" in str(stop_reason or "").upper()),
        "rear_obstacle": "REAR" in str(stop_reason or "").upper(),
        "safe_vx": _finite(safe_vx),
        "safe_w": _finite(safe_w),
        "input_vx": _finite(cmd_vx),
        "input_w": _finite(cmd_w),
        "override_applied": bool(safety_override),
        "safety_rejection_observed": bool(safety_rejection_observed),
        "commanded_vx": _finite(cmd_vx),
        "rejected": bool(safety_rejection_observed),
        "front_near": _finite(front_near),
        "rear_near": _finite(rear_near),
        "source": "apply_safety",
    }

    progress_block = {
        "global": NA,
        "local": NI,
        "obstacle": NA,
        "stuck": _finite(stuck_s),
        "stuck_age": _finite(stuck_s),
        "obstacle_passed": lm.get("obstacle_passed") if "obstacle_passed" in lm else man.get("obstacle_passed"),
        "note": "local progress NOT_IMPLEMENTED in 3B",
    }

    recovery_src = pol.get("recovery") if isinstance(pol.get("recovery"), dict) else {}
    recovery_exec = man.get("recovery_exec") if isinstance(man.get("recovery_exec"), dict) else {}
    if not recovery_exec and isinstance(recovery_src.get("execution"), dict):
        recovery_exec = recovery_src.get("execution") or {}
    recovery_block = {
        "implemented": True,
        "allowed": bool(recovery_src.get("allow_recovery", allow_recovery)),
        "active": "REVERSE" in fsm_mode
        or "RECOVERY" in str(policy_state).upper()
        or str(recovery_src.get("action") or "") in ("LOCAL_REVERSE", "HISTORICAL_RETREAT"),
        "classification": recovery_src.get("classification") or NA,
        "action": recovery_src.get("action") or NA,
        "phase": fsm_mode if "REVERSE" in fsm_mode else (recovery_src.get("classification") or NA),
        "attempt": recovery_attempts,
        "max_attempt": recovery_src.get("max_recovery_attempts") or 3,
        "reason": recovery_src.get("reason") or NA,
        "failure_reason": (recovery_src.get("retreat") or {}).get("failure_reason")
        if isinstance(recovery_src.get("retreat"), dict)
        else (recovery_exec.get("status") if recovery_exec.get("status") in ("FAILED", "STALLED") else NA),
        "release_commitment": bool(recovery_src.get("release_commitment")),
        "active_source": recovery_src.get("active_source") or NA,
        "retreat": recovery_src.get("retreat"),
        "gates": recovery_src.get("gates") or {},
        "execution": recovery_exec or {
            "status": NA,
            "signed_progress_m": NA,
            "target_distance_m": NA,
            "target_remaining_m": NA,
            "note": "awaiting ManeuverFSM.recovery_exec",
        },
        "progress": {
            "signed_progress_m": recovery_exec.get("signed_progress_m"),
            "distance_since_recovery_start": recovery_exec.get("distance_since_start_m"),
            "target_distance_remaining": recovery_exec.get("target_remaining_m"),
            "status": recovery_exec.get("status"),
        },
        "source": "nav_recovery.evaluate_recovery",
    }

    traj_src = pol.get("physical_trajectory") if isinstance(pol.get("physical_trajectory"), dict) else {}
    if not traj_src and isinstance(pol.get("trajectory"), dict):
        traj_src = pol.get("trajectory") or {}
    physical_trajectory = {
        "implemented": True,
        "active": traj_src or None,
        "note": "Swept footprint⊕margin from Probe poses (ONE source of truth)",
    }
    breadcrumb_block = pol.get("breadcrumb") if isinstance(pol.get("breadcrumb"), dict) else {
        "implemented": True,
        "count": 0,
        "points": [],
        "note": "awaiting LocalMppiModel.breadcrumb",
    }

    loops = pol.get("loops") if isinstance(pol.get("loops"), dict) else {}
    replan_block = {
        "allowed": allow_replan,
        "active": str(policy_state).upper() == "REPLAN",
        "reason": NA,
        "count": loops.get("replan_count", NA),
        "loop": bool(loops.get("replan_loop")) if "replan_loop" in loops else NA,
    }
    oscillation_block = {
        "active": bool(loops.get("oscillation")) if "oscillation" in loops else NA,
        "count": loops.get("oscillation_flips", NA),
        "window": NA,
        "last_sign": controller_side,
        "note": "side-switch oscillation guard is STEP3E; this is existing w-flip loop read-only",
    }

    skew = None
    if pose_ts is not None and obstacle_ts is not None:
        skew = abs(float(pose_ts) - float(obstacle_ts)) * 1000.0

    if tracker is not None:
        tracker.observe(
            fsm_mode=fsm_mode if fsm_mode != NA else "IDLE",
            selector_side=selector_side,
            selector_reason=str(selector_reason),
            stop_reason=str(stop_reason or "NONE"),
            safety_override=bool(safety_override),
            mppi_vx=_finite(mppi_vx, 0.0) or 0.0,
            mppi_w=_finite(mppi_w, 0.0) or 0.0,
            safe_vx=s_vx,
            safe_w=s_w,
            front_near=_finite(front_near, 0.0) or 0.0,
            rear_near=_finite(rear_near, 0.0) or 0.0,
            input_vx=in_vx,
            input_w=in_w,
        )

    build_ms = (time.perf_counter() - t0) * 1000.0
    obs_snap = (
        probe.get("obstacle_snapshot")
        if isinstance(probe.get("obstacle_snapshot"), dict)
        else placeholder_obstacle_snapshot()
    )
    # Prefer probe snapshot skew when available
    if isinstance(obs_snap, dict) and obs_snap.get("skew_ms") is not None and skew is None:
        skew = obs_snap.get("skew_ms")

    try:
        from agv_bridge.nav_footprint import geometry_telemetry

        vehicle_geometry = geometry_telemetry()
    except Exception:
        vehicle_geometry = {"implemented": False}

    return {
        "schema_version": "phase4_step3f_v1",
        "trace_id": (tracker.trace_id if tracker else None) or session_id,
        "behavior_unchanged_guarantee": False,
        "ownership": ownership,
        "sides": sides,
        "policy": policy_block,
        "commitment": commitment,
        "side_switch": side_switch,
        "probe": probe,
        "obstacle_snapshot": obs_snap,
        "selector": selector_block,
        "maneuver": maneuver_block,
        "controller": controller_block,
        "safety": safety_block,
        "progress": progress_block,
        "recovery": recovery_block,
        "physical_trajectory": physical_trajectory,
        "vehicle_geometry": vehicle_geometry,
        "breadcrumb": breadcrumb_block,
        "replan": replan_block,
        "oscillation": oscillation_block,
        "snapshot_skew_ms": skew,
        "pose_timestamp": pose_ts,
        "obstacle_timestamp": obstacle_ts,
        "performance": {
            "phase4_build_ms": round(build_ms, 3),
            "debug_ms": _finite(debug_ms),
            "physics_cycle_ms": NA,
            "policy_ms": NA,
            "selector_ms": NA,
            "probe_bundle_ms": probe.get("bundle_ms") if isinstance(probe, dict) else NA,
            "probe_forward_ms": probe.get("forward_ms") if isinstance(probe, dict) else NA,
            "probe_backward_ms": probe.get("backward_ms") if isinstance(probe, dict) else NA,
            "probe_left_ms": probe.get("left_ms") if isinstance(probe, dict) else NA,
            "probe_right_ms": probe.get("right_ms") if isinstance(probe, dict) else NA,
            "probe_turn_ms": probe.get("turn_ms") if isinstance(probe, dict) else NA,
            "side_switch_authorization_ms": side_switch.get("authorization_ms")
            if isinstance(side_switch, dict)
            else NA,
            "mppi_ms": NA,
            "safety_ms": NA,
            "total_ms": NA,
            "note": "auth_ms is O(1) gate checks; no extra rollout",
        },
        "events": list(tracker.events)[-40:] if tracker else [],
        "current_vs_target": {
            "commitment": {
                "current": "AvoidanceCommitment (STEP3C)",
                "target": "AvoidanceCommitment + authorize (3E)",
            },
            "probe": {"current": "ProbeEngine F/B/L/R/TURN (STEP3D)", "target": "consumed by 3E authorize"},
            "side_switch_authority": {
                "current": "NavigationPolicy.side_switch_authorized",
                "target": "NavigationPolicy.side_switch_authorized",
            },
        },
    }
