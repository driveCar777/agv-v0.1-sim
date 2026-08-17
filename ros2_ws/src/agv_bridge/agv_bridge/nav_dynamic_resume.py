"""P0-D.1 — Dynamic obstacle wait/resume forensics and resume hysteresis.

Diagnoses why vehicle stays stopped after moving obstacle clears.
Does NOT mask failures with goal reset.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

DYN_ENTER = "ENTER_DYNAMIC_WAIT"
DYN_BLOCKED = "DYNAMIC_BLOCKED"
DYN_CLEAR = "DYNAMIC_CLEAR"
DYN_RESUME = "DYNAMIC_RESUME"
DYN_REPLAN = "DYNAMIC_REPLAN"
DYN_STOP = "DYNAMIC_STOP"

RESUME_RAMP = (0.15, 0.20, 0.25, 0.30)


@dataclass
class DynamicResumeState:
    dynamic_short: bool = False
    dynamic_long: bool = False
    was_dynamic: bool = False
    in_dynamic_wait: bool = False
    dynamic_state: str = "IDLE"
    resume_allowed: bool = False
    resume_block_reason: Optional[str] = None
    resume_target_vx: Optional[float] = None
    wait_enter_ts: float = 0.0
    clear_ts: float = 0.0
    loop_flags: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "dynamic_short": self.dynamic_short,
            "dynamic_long": self.dynamic_long,
            "was_dynamic": self.was_dynamic,
            "in_dynamic_wait": self.in_dynamic_wait,
            "dynamic_state": self.dynamic_state,
            "resume_allowed": self.resume_allowed,
            "resume_block_reason": self.resume_block_reason,
            "resume_target_vx": self.resume_target_vx,
            "wait_enter_ts": self.wait_enter_ts,
            "clear_ts": self.clear_ts,
            "loop_flags": dict(self.loop_flags),
            "source": "DYNAMIC_RESUME",
            "controls_vehicle": False,
        }


class DynamicResumeTracker:
    """Track dynamic obstacle wait/resume with hysteresis."""

    def __init__(self) -> None:
        self.state = DynamicResumeState()
        self._resume_step = 0
        self._last_ramp_ts = 0.0

    def reset(self) -> None:
        self.state = DynamicResumeState()
        self._resume_step = 0
        self._last_ramp_ts = 0.0

    def update(
        self,
        *,
        now: float,
        dynamic_short: bool,
        dynamic_long: bool,
        policy_state: str,
        policy_reason: str,
        maneuver_mode: str,
        front_near: float,
        forward_feasible: bool,
        safety_zero: bool,
        local_plan_valid: bool,
        recovery_loop: bool = False,
        replan_loop: bool = False,
        oscillation_loop: bool = False,
    ) -> DynamicResumeState:
        st = self.state
        prev_short = st.dynamic_short
        st.dynamic_short = bool(dynamic_short)
        st.dynamic_long = bool(dynamic_long)

        pst = str(policy_state or "").upper()
        mm = str(maneuver_mode or "").upper()
        in_wait = pst == "WAIT_FOR_CLEARANCE" or mm == "WAIT_FOR_CLEARANCE" or "DYNAMIC" in str(policy_reason or "").upper()

        events: list[tuple[str, str]] = []

        if dynamic_short and not prev_short:
            st.in_dynamic_wait = True
            st.wait_enter_ts = now
            st.dynamic_state = "BLOCKED"
            events.append((DYN_ENTER, "dynamic_short_rising"))
        elif dynamic_short:
            st.in_dynamic_wait = True
            st.dynamic_state = "BLOCKED"
        elif prev_short and not dynamic_short:
            st.dynamic_state = "CLEAR"
            st.clear_ts = now
            events.append((DYN_CLEAR, "dynamic_short_cleared"))
        elif not dynamic_short and not dynamic_long:
            if st.dynamic_state == "CLEAR":
                st.dynamic_state = "CLEAR"

        st.was_dynamic = st.was_dynamic or dynamic_short or dynamic_long

        block_reason = None
        if dynamic_short:
            block_reason = "DYNAMIC_SHORT_ACTIVE"
        elif in_wait and not dynamic_short:
            if safety_zero:
                block_reason = "SAFETY_ZERO"
            elif recovery_loop:
                block_reason = "RECOVERY_LOOP"
            elif replan_loop:
                block_reason = "REPLAN_LOOP"
            elif oscillation_loop:
                block_reason = "OSCILLATION_LOOP"
            elif not forward_feasible:
                block_reason = "FORWARD_INFEASIBLE"
            elif not local_plan_valid:
                block_reason = "LOCAL_PLAN_STALE"
            elif mm in ("TURN_IN_PLACE", "ALIGN", "REPOSITION"):
                block_reason = f"FSM_{mm}"
            else:
                block_reason = "WAIT_STATE_HELD"

        resume_ok = (
            not dynamic_short
            and not dynamic_long
            and not safety_zero
            and forward_feasible
            and front_near > 0.85
            and not recovery_loop
            and not replan_loop
        )

        if resume_ok and (prev_short or st.dynamic_state == "CLEAR" or in_wait):
            if block_reason in (None, "WAIT_STATE_HELD"):
                st.resume_allowed = True
                st.in_dynamic_wait = False
                st.dynamic_state = "RESUMING"
                if now - self._last_ramp_ts > 0.35:
                    self._resume_step = min(len(RESUME_RAMP) - 1, self._resume_step + 1)
                    self._last_ramp_ts = now
                st.resume_target_vx = RESUME_RAMP[self._resume_step]
                events.append((DYN_RESUME, f"ramp_step_{self._resume_step}"))
            else:
                st.resume_allowed = False
                st.resume_block_reason = block_reason
                events.append((DYN_BLOCKED, block_reason or "unknown"))
        else:
            st.resume_allowed = False
            st.resume_block_reason = block_reason
            if dynamic_short:
                events.append((DYN_BLOCKED, "dynamic_short"))

        st.loop_flags = {
            "recovery_loop": recovery_loop,
            "replan_loop": replan_loop,
            "oscillation_loop": oscillation_loop,
            "policy_state": pst,
            "maneuver_mode": mm,
            "policy_reason": policy_reason,
        }

        self._emit(events, st)
        return st

    def _emit(self, events: list[tuple[str, str]], st: DynamicResumeState) -> None:
        if not events:
            return
        try:
            from agv_bridge.nav_observability import OBS

            for name, detail in events:
                OBS.emit(
                    name,
                    level="INFO" if "RESUME" in name or "CLEAR" in name else "NOTICE",
                    category="DYNAMIC",
                    component="dynamic_resume",
                    data={**st.to_dict(), "detail": detail},
                    min_interval_s=0.4 if name != DYN_RESUME else 0.8,
                )
        except Exception:
            pass


def diagnose_resume_block(st: DynamicResumeState) -> Dict[str, Any]:
    """Structured A–J checklist for post-mortem."""
    lf = st.loop_flags
    checks = {
        "A_wait_no_exit": lf.get("policy_state") == "WAIT_FOR_CLEARANCE" and not st.dynamic_short,
        "B_replan_loop": bool(lf.get("replan_loop")),
        "C_recovery_loop": bool(lf.get("recovery_loop")),
        "D_safety_zero": st.resume_block_reason == "SAFETY_ZERO",
        "E_forward_infeasible": st.resume_block_reason == "FORWARD_INFEASIBLE",
        "F_local_plan_stale": st.resume_block_reason == "LOCAL_PLAN_STALE",
        "G_replan_global": st.resume_block_reason == "REPLAN_LOOP",
        "H_oscillation": bool(lf.get("oscillation_loop")),
        "I_dynamic_stale": st.dynamic_short and st.dynamic_state == "CLEAR",
        "J_fsm_turn": (lf.get("maneuver_mode") or "") in ("TURN_IN_PLACE", "ALIGN", "REPOSITION"),
    }
    return {"checks": checks, "resume_block_reason": st.resume_block_reason, "resume_allowed": st.resume_allowed}
