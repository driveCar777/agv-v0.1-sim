"""V0.2 M2 — planner / failure semantics (distinct from navigation task failure).

Layered states:
  LOCAL_PLAN_INFEASIBLE — local trajectory generation failed (NOT task failure)
  LOCAL_RECOVERY        — recovery ladder active
  SAFE_STOP             — motion halted for safety (NOT task failure until exhausted)
  NAVIGATION_FAILED     — recovery exhausted / global impossible / critical fault
"""

from __future__ import annotations

from typing import Any, Dict

# Planner FSM states
PLANNER_NORMAL = "NORMAL"
LOCAL_PLAN_INFEASIBLE = "LOCAL_PLAN_INFEASIBLE"
LOCAL_RECOVERY = "LOCAL_RECOVERY"
PLANNER_SAFE_STOP = "SAFE_STOP"
NAVIGATION_FAILED = "NAVIGATION_FAILED"

# Unified safe_vx / stop reason codes (shared backend ↔ frontend)
REASON_NORMAL = "NORMAL"
REASON_DYNAMIC_WAIT = "DYNAMIC_WAIT"
REASON_MPPI_INFEASIBLE = "MPPI_NO_FEASIBLE_TRAJECTORY"
REASON_FP_CLEARANCE = "FOOTPRINT_CLEARANCE_VETO"
REASON_BRAKING_LIMIT = "BRAKING_LIMIT"
REASON_BRAKING_UNAVAILABLE = "BRAKING_MODEL_UNAVAILABLE"
REASON_STALE_SENSOR = "STALE_SENSOR"
REASON_LOC_INVALID = "LOCALIZATION_INVALID"
REASON_RECOVERY_ACTIVE = "RECOVERY_ACTIVE"
REASON_SAFE_STOP = "SAFE_STOP"
REASON_RECOVERY_EXHAUSTED = "RECOVERY_EXHAUSTED"
REASON_RECOVERY_REVERSE = "RECOVERY_REQUIRES_REVERSE"
REASON_NAV_FAILED = "NAVIGATION_FAILED"

# Stop reason strings (physics / telemetry)
STOP_NONE = "NONE"
STOP_SAFE = "SAFE_STOP"
STOP_RECOVERY = "RECOVERY"
STOP_NAV_FAILED = "NAVIGATION_FAILED"

# UI severity bands
UI_NORMAL = "NORMAL"
UI_WARNING = "WARNING"
UI_DEGRADED = "DEGRADED"
UI_RECOVERY = "RECOVERY"
UI_SAFETY_STOP = "SAFETY_STOP"
UI_FAILED = "FAILED"


def ui_severity(*, planner_state: str, safe_vx_reason: str, stop_reason: str) -> str:
    if planner_state == NAVIGATION_FAILED or safe_vx_reason == REASON_NAV_FAILED:
        return UI_FAILED
    if planner_state in (LOCAL_RECOVERY,) or safe_vx_reason == REASON_RECOVERY_ACTIVE:
        return UI_RECOVERY
    if planner_state == LOCAL_PLAN_INFEASIBLE or safe_vx_reason == REASON_MPPI_INFEASIBLE:
        return UI_DEGRADED
    if planner_state == PLANNER_SAFE_STOP or stop_reason == STOP_SAFE:
        return UI_SAFETY_STOP
    if safe_vx_reason not in (REASON_NORMAL, ""):
        return UI_WARNING
    return UI_NORMAL


def is_navigation_failed(planner_state: str, stop_reason: str) -> bool:
    return planner_state == NAVIGATION_FAILED or stop_reason == STOP_NAV_FAILED


def is_transient_planner_failure(planner_state: str) -> bool:
    return planner_state in (LOCAL_PLAN_INFEASIBLE, LOCAL_RECOVERY)


def planner_state_to_dict(state: str, *, failure_reason: str = "", recovery_action: str = "") -> Dict[str, Any]:
    return {
        "planner_state": state,
        "planner_failure_reason": failure_reason or "NONE",
        "recovery_action": recovery_action or "NONE",
        "ui_severity": ui_severity(
            planner_state=state,
            safe_vx_reason=failure_reason if state == LOCAL_PLAN_INFEASIBLE else "",
            stop_reason=STOP_NONE,
        ),
    }
