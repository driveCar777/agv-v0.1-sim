"""P0-C.1 Open-space local planning forensics — assemble-only.

Does not change planning, Safety, FSM, MPPI, or P0-C validator behavior.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from agv_bridge.local_maneuver import NOMINAL_VX, SIDE_VX, selector_horizon_s, selector_nominal_distance_m
from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.mppi_controller import get_control_mode

MAX_VX = float(DEFAULT_GEOM.max_vx)
SIDE_NOM = float(SIDE_VX)


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _polyline_len(poses: Any) -> float:
    s = 0.0
    prev = None
    for p in poses or []:
        if isinstance(p, dict):
            xy = (float(p.get("x") or 0.0), float(p.get("y") or 0.0))
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            xy = (float(p[0]), float(p[1]))
        else:
            continue
        if prev is not None:
            s += ((xy[0] - prev[0]) ** 2 + (xy[1] - prev[1]) ** 2) ** 0.5
        prev = xy
    return s


def classify_short_horizon_reason(
    *,
    horizon_s: Optional[float],
    planned_m: Optional[float],
    survived_m: Optional[float],
    state_vx: Optional[float],
    safety_clamp: bool,
    capture_fail: bool,
    compare_called: Optional[bool],
) -> str:
    if capture_fail:
        return "PATH_CAPTURE"
    if planned_m is not None and survived_m is not None and planned_m > 0.08:
        if survived_m + 0.04 < 0.70 * planned_m:
            return "INVALIDATED_EARLY"
        if survived_m >= 0.85 * planned_m:
            if horizon_s is not None and abs(horizon_s - 1.5) < 0.08:
                return "FIXED_1_5S"
            return "FIXED_HORIZON"
    if safety_clamp:
        return "SAFETY_CLAMP"
    if state_vx is not None and state_vx < 0.70 * NOMINAL_VX:
        return "LOW_SPEED"
    if compare_called is False:
        return "FIXED_1_5S"
    if horizon_s is not None and abs(horizon_s - 1.5) < 0.08:
        return "FIXED_1_5S"
    return "UNKNOWN"


def assemble_open_space_forensics(
    *,
    dbg: Optional[Dict[str, Any]] = None,
    gref: Optional[Dict[str, Any]] = None,
    loc_layer: Optional[Dict[str, Any]] = None,
    kv: Optional[Dict[str, Any]] = None,
    maneuver: Optional[Dict[str, Any]] = None,
    policy: Optional[Dict[str, Any]] = None,
    mppi_meta: Optional[Dict[str, Any]] = None,
    physical: Optional[Dict[str, Any]] = None,
    planned_path: Optional[List[Any]] = None,
    cmd_vx: Optional[float] = None,
    safe_vx: Optional[float] = None,
    state_vx: Optional[float] = None,
    front_near: Optional[float] = None,
    rear_near: Optional[float] = None,
    left_near: Optional[float] = None,
    right_near: Optional[float] = None,
    stop_reason: Optional[str] = None,
    control_mode: Optional[str] = None,
    path_valid: Optional[bool] = None,
) -> Dict[str, Any]:
    dbg = dbg or {}
    gref = gref or {}
    loc_layer = loc_layer or {}
    kv = kv or {}
    maneuver = maneuver or (dbg.get("maneuver") if isinstance(dbg.get("maneuver"), dict) else {}) or {}
    policy = policy or (dbg.get("nav_policy") if isinstance(dbg.get("nav_policy"), dict) else {}) or {}
    if not policy:
        policy = dbg.get("policy") if isinstance(dbg.get("policy"), dict) else {}
    pol_dec = policy.get("decision") if isinstance(policy.get("decision"), dict) else policy
    mppi_meta = mppi_meta or {}
    physical = physical or {}
    lc = maneuver.get("local_compare") if isinstance(maneuver.get("local_compare"), dict) else {}
    sel_f = maneuver.get("local_selector") if isinstance(maneuver.get("local_selector"), dict) else {}
    if not sel_f:
        sel_f = lc.get("selector") if isinstance(lc.get("selector"), dict) else {}

    horizon_s = _f(sel_f.get("selector_horizon_s")) or selector_horizon_s()
    nom_vx = _f(sel_f.get("selector_nominal_vx")) or NOMINAL_VX
    nom_dist = _f(sel_f.get("selector_nominal_distance_m")) or selector_nominal_distance_m()
    compare_called = lc.get("compare_called")
    if compare_called is None:
        compare_called = sel_f.get("compare_called")
    compare_reason = lc.get("compare_reason") or sel_f.get("compare_reason")
    invoked = lc.get("invoked")
    if invoked is None:
        invoked = maneuver.get("local_compare_invoked")

    cands = lc.get("candidates") if isinstance(lc.get("candidates"), dict) else {}
    cand_rows: List[Dict[str, Any]] = []
    capture_fail = False
    planned_m = None
    survived_m = None
    for k, v in cands.items():
        if not isinstance(v, dict):
            continue
        row = {
            "candidate_id": v.get("type") or k,
            "type": v.get("type") or k,
            "feasible": v.get("feasible"),
            "reason": v.get("reason"),
            "vx": v.get("vx"),
            "w": v.get("w"),
            "duration_s": v.get("duration_s") if v.get("duration_s") is not None else v.get("duration"),
            "distance_m": v.get("distance_m"),
            "planned_distance_m": v.get("planned_distance_m"),
            "actual_survived_distance_m": v.get("actual_survived_distance_m"),
            "min_clearance": v.get("min_clearance"),
            "mean_clearance": v.get("mean_clearance"),
            "path_capture_available": v.get("path_capture_available"),
            "path_capture_distance": v.get("path_capture_distance"),
            "path_capture_heading_error": v.get("path_capture_heading_error"),
            "progress_gain": v.get("path_progress_gain"),
            "heading_error": v.get("heading_error"),
            "lateral_error": v.get("lateral_error"),
            "total_cost": v.get("total_cost"),
            "cost_breakdown": v.get("cost_breakdown"),
            "selected": v.get("selected"),
            "first_collision_t": v.get("first_collision_t"),
            "first_invalid_reason": v.get("first_invalid_reason"),
        }
        cand_rows.append(row)
        if v.get("selected"):
            planned_m = _f(v.get("planned_distance_m"))
            survived_m = _f(v.get("actual_survived_distance_m") or v.get("distance_m"))
            if (not v.get("feasible")) and str(v.get("reason") or "") == "NO_PATH_CAPTURE":
                capture_fail = True
        if str(v.get("reason") or "") == "NO_PATH_CAPTURE":
            capture_fail = True

    local_max = _f(loc_layer.get("max_distance_m"))
    if survived_m is None:
        survived_m = local_max
    if planned_m is None:
        svx = _f(state_vx) or nom_vx
        planned_m = abs(float(svx)) * float(horizon_s)

    gprev = _f(gref.get("preview_m"))
    coverage = None
    if gprev and gprev > 1e-6 and local_max is not None:
        coverage = round(local_max / gprev, 4)

    req = _f(cmd_vx)
    safe = _f(safe_vx)
    stv = _f(state_vx)
    safety_clamp = bool(req is not None and safe is not None and abs(req) > 0.02 and abs(safe) + 0.02 < abs(req))
    safety_zero = bool(req is not None and abs(req) > 0.02 and safe is not None and abs(safe) < 0.02)

    mppi_h = _f(mppi_meta.get("horizon_s")) or 1.6
    mppi_path_m = _polyline_len(planned_path) if planned_path else None
    phys_src = physical.get("source") if isinstance(physical, dict) else None
    loc_src = loc_layer.get("source") or ("MPPI" if not cands else "LOCAL_SELECTOR")

    kin_valid = kv.get("kinematic_valid")
    scene = pol_dec.get("scene") or policy.get("scene")
    pstate = pol_dec.get("state") or policy.get("state")
    profile = pol_dec.get("profile") if isinstance(pol_dec.get("profile"), dict) else {}
    vx_scale = _f(profile.get("vx_scale"))
    contradiction = bool(kin_valid is False and str(scene or "").upper() in ("OPEN", "OPEN_SPACE"))

    short_reason = classify_short_horizon_reason(
        horizon_s=horizon_s,
        planned_m=planned_m,
        survived_m=survived_m,
        state_vx=stv,
        safety_clamp=safety_clamp,
        capture_fail=capture_fail,
        compare_called=compare_called,
    )

    speed_ratio = None if stv is None else round(stv / MAX_VX, 4) if MAX_VX else None
    speed_ratio_nom = None if stv is None else round(stv / SIDE_NOM, 4) if SIDE_NOM else None

    return {
        "scene": "OPEN_SPACE" if str(scene or "").upper() in ("OPEN", "OPEN_SPACE") else scene,
        "policy_scene": scene,
        "global_preview_m": gprev,
        "local_selector": {
            "horizon_s": round(horizon_s, 3),
            "nominal_vx": nom_vx,
            "side_vx": SIDE_VX,
            "nominal_distance_m": round(float(nom_dist), 3),
            "max_distance_m": local_max,
            "compare_invoked": invoked,
            "compare_called": compare_called,
            "compare_reason": compare_reason,
            "fsm_skip_reason": lc.get("fsm_skip_reason") or maneuver.get("local_compare_fsm_reason"),
            "last_compare_age_s": sel_f.get("last_compare_age_s"),
            "current": sel_f.get("current") or maneuver.get("decision"),
            "candidate_count": loc_layer.get("count") if loc_layer.get("count") is not None else len(cand_rows),
            "valid_count": loc_layer.get("valid_count"),
            "selected_candidate": loc_layer.get("selected_candidate") or maneuver.get("decision"),
            "candidates": cand_rows,
        },
        "mppi": {
            "horizon_s": mppi_h,
            "time_steps": mppi_meta.get("time_steps"),
            "model_dt": mppi_meta.get("model_dt"),
            "batch_size": mppi_meta.get("batch_size") or mppi_meta.get("batch"),
            "temperature": mppi_meta.get("temperature"),
            "top_k": mppi_meta.get("top_k"),
            "mean_vx": mppi_meta.get("mean_vx_after") or mppi_meta.get("mean_vx"),
            "mean_vx_before": mppi_meta.get("mean_vx_before"),
            "mean_vx_after": mppi_meta.get("mean_vx_after") or mppi_meta.get("mean_vx"),
            "vx_raw": mppi_meta.get("vx_raw"),
            "vx_cmd": mppi_meta.get("vx_cmd"),
            "vx_scale": mppi_meta.get("vx_scale") if mppi_meta.get("vx_scale") is not None else vx_scale,
            "mean_dw": mppi_meta.get("mean_dw"),
            "pp_w": mppi_meta.get("pp_w"),
            "w_cmd": mppi_meta.get("w_cmd"),
            "a_vx_min": mppi_meta.get("a_vx_min") if mppi_meta.get("a_vx_min") is not None else mppi_meta.get("vx_min"),
            "a_vx_max": mppi_meta.get("a_vx_max") if mppi_meta.get("a_vx_max") is not None else mppi_meta.get("vx_max"),
            "wz_max": mppi_meta.get("wz_max"),
            "path_follow_weight": mppi_meta.get("path_follow_weight") or pol_dec.get("path_follow_weight"),
            # Soft preference from SpeedPolicy (OPEN cruise default 0.30). Not SIDE_VX.
            "speed_target": mppi_meta.get("target_vx") if mppi_meta.get("target_vx") is not None else 0.30,
            "speed_target_kind": "SPEED_POLICY_TARGET",
            "best_path_distance_m": None if mppi_path_m is None else round(mppi_path_m, 3),
            "control_mode": control_mode or mppi_meta.get("control_mode") or get_control_mode(),
            "local_replan_period_s": 0.35,
        },
        "policy": {
            "state": pstate,
            "behavior": pol_dec.get("behavior") or policy.get("behavior"),
            "scene": scene,
            "profile_name": profile.get("name"),
            "vx_scale": vx_scale,
            "path_follow_weight": pol_dec.get("path_follow_weight"),
            "max_deviation_m": pol_dec.get("max_deviation_m"),
            "require_capture_hard": pol_dec.get("require_capture_hard"),
            "allow_side_compare": pol_dec.get("allow_side_compare"),
        },
        "command": {
            "requested_vx": req,
            "safe_vx": safe,
            "state_vx": stv,
            "safety_clamp": safety_clamp,
            "safety_zero": safety_zero,
            "stop_reason": stop_reason,
            "front_near": _f(front_near),
            "rear_near": _f(rear_near),
            "left_near": _f(left_near),
            "right_near": _f(right_near),
        },
        "p0c": {
            "kinematic_valid": kin_valid,
            "kinematic_status": kv.get("kinematic_status") or kv.get("status"),
            "path_valid": path_valid,
            "path_valid_source": "global_path_exists_len>=2",
            "kinematic_valid_used_by_policy": False,
            "open_vs_invalid_contradiction": contradiction,
        },
        "render": {
            "local_candidates_source": loc_src,
            "selected_local_source": "LOCAL_SELECTOR" if cands else loc_src,
            "active_physical_source": ("PROBE:" + str(phys_src)) if phys_src else "PROBE",
            "mppi_best_path_source": "MPPI",
        },
        "diagnostics": {
            "coverage_ratio": coverage,
            "short_horizon_reason": short_reason,
            "safety_clamp": safety_clamp,
            "expected_distance_at_nominal_vx": round(selector_nominal_distance_m(NOMINAL_VX), 3),
            "expected_distance_at_state_vx": None if stv is None else round(abs(stv) * horizon_s, 3),
            "planned_distance_m": None if planned_m is None else round(planned_m, 3),
            "rendered_distance_m": local_max,
            "executed_distance_m": None if survived_m is None else round(survived_m, 3),
            "actual_survived_distance_m": None if survived_m is None else round(survived_m, 3),
            "speed_ratio": speed_ratio,
            "speed_ratio_to_nominal": speed_ratio_nom,
            "max_vx": MAX_VX,
            # P1-0 role classification (telemetry); does not change control
            "local_selector_role_in_open": (
                "OBSERVATION_ONLY"
                if (compare_called is False or str(compare_reason or "") == "NONE_OPEN_FORWARD")
                and str(pstate or "").upper() in ("FOLLOW_GLOBAL", "IDLE", "")
                else "MIXED"
            ),
            "open_space_cruise_speed": "NO_EXPLICIT_OPEN_SPACE_CRUISE_SPEED",
            "architecture_mode_open": "GLOBAL_TRACKING_PLUS_MPPI",
            "architecture_mode_avoid": "LOCAL_SELECTOR_PLUS_MANEUVER_PLUS_MPPI",
            "open_space_target_vx_code": {
                "selector_forward": NOMINAL_VX,
                "selector_side": SIDE_VX,
                "mppi_mean_init": 0.16,
                "mppi_speed_track_soft": 0.22,
                "mppi_vx_max_forward_track": round(MAX_VX * 0.95, 3),
                "policy_normal_vx_scale": 1.0,
                "pp_only_cruise": 0.22,
                "note": "OPEN FOLLOW_GLOBAL has no hard cruise; soft track 0.22 + mean init 0.16 + sample std 0.08 + EMA",
            },
        },
    }
