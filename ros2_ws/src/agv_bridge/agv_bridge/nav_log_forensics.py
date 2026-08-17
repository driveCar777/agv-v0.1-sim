"""P0-B-0 pure forensic helpers — no navigation side effects."""

from __future__ import annotations

from typing import Any, Dict, Optional

VX_EPS = 0.03
W_EPS = 0.12


def infer_likely_owner(
    *,
    candidate_count: int = 0,
    valid_candidate_count: int = 0,
    selected_candidate: Any = None,
    requested_vx: Optional[float] = None,
    requested_w: Optional[float] = None,
    safe_vx: Optional[float] = None,
    safe_w: Optional[float] = None,
    state_vx: Optional[float] = None,
    state_w: Optional[float] = None,
    recovery_active: bool = False,
    intentional_stop: bool = False,
    goal_reached: bool = False,
) -> str:
    """Infer STOP/stall owner from command + candidate facts."""
    if goal_reached or intentional_stop:
        return "NONE"

    sel = selected_candidate
    sel_none = sel is None or sel == "" or str(sel).upper() in ("NONE", "NULL", "N/A")

    rvx = None if requested_vx is None else float(requested_vx)
    svx = None if safe_vx is None else float(safe_vx)
    xv = None if state_vx is None else float(state_vx)
    sw = 0.0 if safe_w is None else abs(float(safe_w))
    xw = 0.0 if state_w is None else abs(float(state_w))
    rw = None if requested_w is None else abs(float(requested_w))

    if recovery_active and rvx is not None and rvx < -VX_EPS:
        if svx is not None and abs(svx) < VX_EPS and abs(rvx) > VX_EPS:
            return "SAFETY"
        if svx is not None and abs(svx) > VX_EPS and (xv is None or abs(xv) < VX_EPS):
            return "EXECUTION"
        return "RECOVERY"

    if candidate_count <= 0:
        return "LOCAL_PLANNING"
    if valid_candidate_count <= 0:
        return "CANDIDATE"
    if sel_none and valid_candidate_count > 0:
        return "CANDIDATE_SELECTOR"

    if rvx is not None and abs(rvx) > VX_EPS and svx is not None and abs(svx) < VX_EPS:
        return "SAFETY"

    if not sel_none and rvx is not None and abs(rvx) < VX_EPS and (rw is None or rw < W_EPS):
        return "FSM" if sw < W_EPS and xw < W_EPS else "POLICY"

    if not sel_none and rvx is not None and abs(rvx) < VX_EPS and rw is not None and rw > W_EPS:
        return "FSM"

    if svx is not None and abs(svx) > VX_EPS and (xv is None or abs(xv) < VX_EPS):
        return "EXECUTION"

    if xv is not None and abs(xv) < VX_EPS and xw > W_EPS:
        return "PHYSICS"

    return "UNKNOWN"


def classify_stop_case(
    *,
    candidate_count: int,
    valid_candidate_count: int,
    selected_candidate: Any,
    requested_vx: float,
    safe_vx: float,
    state_vx: float,
    state_w: float = 0.0,
    translation_delta: float = 0.0,
) -> Dict[str, Any]:
    """Map forensic cases A–F to owner + optional spin flag."""
    owner = infer_likely_owner(
        candidate_count=candidate_count,
        valid_candidate_count=valid_candidate_count,
        selected_candidate=selected_candidate,
        requested_vx=requested_vx,
        safe_vx=safe_vx,
        state_vx=state_vx,
        state_w=state_w,
    )
    spin = abs(state_vx) < VX_EPS and abs(state_w) > W_EPS and abs(translation_delta) < 0.05
    return {
        "likely_owner": owner,
        "spin_loop_suspected": spin,
        "event": "SPIN_LOOP_SUSPECTED" if spin else "NAVIGATION_STOP_DIAGNOSTIC",
    }
