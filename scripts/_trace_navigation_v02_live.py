#!/usr/bin/env python3
"""V0.2 LIVE trace — deterministic M3.1 scenarios (JSONL).

Scenes LIVE-00 … LIVE-06 via nav_scenario_injector (LiDAR-visible obstacles).

Requires web sim at --base (default http://127.0.0.1:19999).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BRIDGE = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
if BRIDGE not in sys.path:
    sys.path.insert(0, BRIDGE)

from agv_bridge.nav_live_client import NavLiveClient  # noqa: E402
from agv_bridge.nav_scenario_injector import ALL_SCENES, M32_SCENES, M33_SCENES, MOTION_SCENES, OBS_OPEN_SCENES, ONLINE_SCENES, SCENARIOS, apply_scenario  # noqa: E402
from agv_bridge.nav_trajectory_integrity import enrich_trajectory_metadata, global_path_fingerprint  # noqa: E402
from agv_bridge.sim_world import SimWorld  # noqa: E402

_TRACK_KEYS = (
    "nav_state",
    "behavior_state",
    "avoidance_phase",
    "committed_side",
    "planner_state",
    "planner_failure_reason",
    "recovery_state",
    "safe_vx_reason",
    "maneuver_mode",
    "stop_reason",
    "nav_ui_severity",
)

_SCENE_FILE_TAG = {
    "LIVE-00": "LIVE-00-baseline",
    "LIVE-01": "LIVE-01-static-left",
    "LIVE-02": "LIVE-02-static-right",
    "LIVE-03": "LIVE-03-both-blocked",
    "LIVE-04": "LIVE-04-dynamic-cross",
    "LIVE-05": "LIVE-05-dynamic-away",
    "LIVE-06": "LIVE-06-field-p0d1",
    "M32-OPEN-STRAIGHT": "M32-OPEN-STRAIGHT-open",
    "OBS-OPEN-LEFT": "OBS-OPEN-LEFT-static-left",
    "OBS-OPEN-RIGHT": "OBS-OPEN-RIGHT-static-right",
    "OBS-OPEN-BOTH-BLOCKED": "OBS-OPEN-BOTH-blocked",
    "OBS-OPEN-DYNAMIC-CROSS": "OBS-OPEN-DYNAMIC-cross",
    "OBS-OPEN-DYNAMIC-AWAY": "OBS-OPEN-DYNAMIC-away",
    "OBS-OPEN-FIELD-P0D1": "OBS-OPEN-FIELD-p0d1",
    "ONLINE-LEFT": "ONLINE-LEFT-static-left",
    "ONLINE-RIGHT": "ONLINE-RIGHT-static-right",
    "ONLINE-BOTH-BLOCKED": "ONLINE-BOTH-blocked",
    "SCENE-CURVE-01": "SCENE-CURVE-01-approach",
}


def _dig(d: dict, *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


def _sample_v02(client: NavLiveClient, seq: int, scene: str, setup_meta: dict) -> dict:
    # Single lite snapshot — physics thread owns heavy work; HTTP is read-only.
    st = client.get("/api/state?lite=1")
    op = st.get("obstacle_preview") if isinstance(st.get("obstacle_preview"), dict) else {}
    sim_rt = st.get("sim_runtime") if isinstance(st.get("sim_runtime"), dict) else {}
    dbg = st.get("debug") if isinstance(st.get("debug"), dict) else {}
    if not dbg:
        dbg = {}
    safety = st.get("safety") or _dig(dbg, "safety") or {}
    recovery = _dig(dbg, "execution_recovery") or _dig(dbg, "recovery") or {}
    diag = {}

    nav = st.get("nav") or {}
    agv = st.get("agv") or {}
    status = dbg.get("status") or {}
    vc = dbg.get("velocity_chain") or {}
    lp = dbg.get("local_planner") or nav.get("local_plan") or {}
    mppi_meta = st.get("debug", {}).get("mppi_meta") if isinstance(st.get("debug"), dict) else {}
    if not mppi_meta:
        mppi_meta = lp if isinstance(lp, dict) else {}
    nav_pol = dbg.get("nav_policy") or {}

    ec = nav_pol.get("execution_corridor") or _dig(dbg, "nav_policy", "execution_corridor") or nav.get("execution_corridor") or {}
    if not ec:
        ec = _dig(op, "obstacle_preview", "execution_corridor") or {}

    goal = nav.get("goal") or setup_meta.get("goal")
    gpath = nav.get("path") or []
    pt_raw = nav.get("physical_trajectory") if isinstance(nav.get("physical_trajectory"), dict) else {}
    compute_s = None
    if pt_raw:
        pst = pt_raw.get("planner_start_timestamp")
        pft = pt_raw.get("planner_finish_timestamp")
        if pst and pft and float(pft) > float(pst):
            compute_s = float(pft) - float(pst)
    pt = (
        enrich_trajectory_metadata(
            dict(pt_raw),
            vehicle=agv,
            actual_planner_compute_s=compute_s,
            current_scene_id=nav.get("scene_id") or nav.get("nav_scene_id"),
            current_cycle_id=nav.get("planner_cycle_id") or None,
        )
        if pt_raw
        else {}
    )
    integ = pt.get("integrity") or {}
    retreat = nav.get("retreat_trajectory") or nav.get("historical_retreat") or {}
    lplan = nav.get("local_plan") or dbg.get("local_plan") or {}
    ghash = global_path_fingerprint(gpath)
    safety_collision = safety.get("collision")
    if safety_collision is None:
        safety_collision = dbg.get("collision")

    row = {
        "ts": time.time(),
        "seq": seq,
        "scene": scene,
        "schema": "navigation_v0.2_trace/2",
        "setup": setup_meta,
        "vehicle_pose": {"x": agv.get("x"), "y": agv.get("y"), "yaw": agv.get("angle")},
        "vehicle_vx": agv.get("vx"),
        "vehicle_omega": agv.get("w"),
        "state_vx": nav.get("state_vx") or status.get("vx"),
        "state_omega": nav.get("state_w") or status.get("w"),
        "nav_state": nav.get("mode") or status.get("nav_mode"),
        "behavior_state": _dig(dbg, "nav_policy", "state") or nav.get("policy_state") or status.get("policy_state"),
        "maneuver_mode": _dig(dbg, "maneuver", "mode") or nav.get("maneuver_mode") or status.get("maneuver_mode"),
        "avoidance_phase": op.get("avoidance_phase") or _dig(dbg, "nav_policy", "avoidance_phase", "phase"),
        "readiness_signal": op.get("readiness_signal") or op.get("signal"),
        "probe_confidence": op.get("probe_confidence"),
        "probe_confidence_left": op.get("probe_confidence_left") or _dig(op, "side_probe", "left_confidence"),
        "probe_confidence_right": op.get("probe_confidence_right") or _dig(op, "side_probe", "right_confidence"),
        "left_valid": op.get("left_valid") if op.get("left_valid") is not None else _dig(op, "side_probe", "left_valid"),
        "right_valid": op.get("right_valid") if op.get("right_valid") is not None else _dig(op, "side_probe", "right_valid"),
        "preferred_side": op.get("preferred_side") or op.get("probe_side") or _dig(op, "side_probe", "preferred_side"),
        "probe_side": op.get("probe_side") or op.get("preferred_side") or _dig(op, "side_probe", "preferred_side"),
        "commit_ready": op.get("commit_ready") if op.get("commit_ready") is not None else _dig(op, "side_probe", "commit_ready"),
        "d_detection_m": op.get("d_detection_m"),
        "d_probe_start_m": op.get("d_probe_start_m"),
        "d_commit_m": op.get("d_commit_m") or op.get("required_avoidance_distance_m") or op.get("maneuver_start_distance_m"),
        "committed_side": op.get("commit_side") or op.get("committed_side") or _dig(dbg, "nav_policy", "committed_side"),
        "commit_side": op.get("commit_side") or op.get("committed_side") or _dig(dbg, "nav_policy", "committed_side"),
        "execution_side": _corridor_side(ec) or op.get("execution_side"),
        "side_inconsistency": op.get("side_inconsistency"),
        "execution_corridor": ec if ec else None,
        "goal": goal,
        "global_path_len": len(gpath) if isinstance(gpath, list) else None,
        "obstacle_distance": st.get("front_near") or nav.get("safety_envelope", {}).get("front_near"),
        "front_near": nav.get("safety_envelope", {}).get("front_near") or st.get("front_near"),
        "footprint_clearance_m": nav.get("footprint_clearance_m") or safety.get("footprint_clearance_m") or status.get("footprint_clearance_m"),
        "predicted_min_clearance_m": nav.get("predicted_min_clearance_m") or safety.get("predicted_min_clearance_m") or mppi_meta.get("predicted_min_clearance_m"),
        "future_collision": _dig(op, "obstacle_preview", "future_collision") or op.get("future_collision"),
        "first_collision_m": op.get("first_collision_distance_m"),
        "candidate_count": mppi_meta.get("candidate_count") or lp.get("candidate_count") or nav.get("candidate_count"),
        "valid_candidate_count": mppi_meta.get("valid_candidate_count"),
        "collision_rejected_count": mppi_meta.get("collision_rejected_count"),
        "constraint_rejected_count": mppi_meta.get("constraint_rejected_count"),
        "clearance_rejected_count": mppi_meta.get("clearance_rejected_count"),
        "mppi_failure_reason": mppi_meta.get("failure_reason"),
        "mppi_vx": nav.get("mppi_vx") or vc.get("mppi_vx"),
        "mppi_omega": nav.get("mppi_w") or vc.get("requested_omega") or dbg.get("local_planner", {}).get("mppi_w"),
        "requested_vx": nav.get("requested_vx") or vc.get("requested_vx") or nav.get("cmd_vx_before_safety"),
        "approved_vx": nav.get("approved_vx") or vc.get("approved_vx") or nav.get("cmd_vx_after_safety"),
        "requested_omega": nav.get("requested_omega") or vc.get("requested_omega"),
        "approved_omega": nav.get("approved_omega") or vc.get("approved_omega") or nav.get("cmd_w_after_safety"),
        "safe_vx": vc.get("safe_vx") or nav.get("cmd_vx_after_safety"),
        "safe_vx_reason": nav.get("safe_vx_reason") or safety.get("safe_vx_reason") or status.get("safe_vx_reason") or vc.get("safe_vx_reason"),
        "braking_feasible": safety.get("braking_feasible"),
        "planner_state": nav.get("planner_state") or status.get("planner_state"),
        "planner_failure_reason": nav.get("planner_failure_reason") or status.get("planner_failure_reason"),
        "recovery_state": nav.get("recovery_state") or recovery.get("recovery_state") or status.get("recovery_state"),
        "recovery_attempt": nav.get("recovery_attempt") or recovery.get("recovery_attempt") or status.get("recovery_attempt"),
        "recovery_action": recovery.get("recovery_action") or recovery.get("action"),
        "recovery_result": recovery.get("recovery_result") or recovery.get("result"),
        "stop_reason": nav.get("stop_reason") or status.get("stop_reason"),
        "nav_ui_severity": nav.get("nav_ui_severity") or safety.get("nav_ui_severity") or status.get("nav_ui_severity"),
        "sensor_health": diag.get("sensor_health") or st.get("sensor_health") or "UNKNOWN",
        "localization_health": diag.get("localization_health") or ("OK" if float(agv.get("confidence") or 0) > 0.5 else "DEGRADED"),
        "selected_candidate": lp.get("selected_candidate") or dbg.get("local_planner", {}).get("selected_candidate"),
        "trajectory_omega_sign": _omega_sign(nav.get("mppi_w") or vc.get("requested_omega")),
        "corridor_side": _corridor_side(ec),
        "heading": agv.get("angle"),
        "obstacles": st.get("obstacles"),
        "scenario_movers": st.get("scenario_movers"),
        "build_commit": st.get("build_commit") or sim_rt.get("build_commit"),
        "physics_hz": sim_rt.get("physics_hz"),
        "physics_dt_ms": sim_rt.get("physics_dt_ms"),
        "physics_overrun_count": sim_rt.get("physics_overrun_count"),
        "sim_runtime": sim_rt,
        "scene_phase": setup_meta.get("scene_phase") or "BASELINE",
        "test_class": setup_meta.get("test_class"),
        "global_path_revision": nav.get("global_path_revision"),
        "global_replan_count": nav.get("global_replan_count"),
        "global_path_hash": ghash,
        "global_path_hash_at_setup": setup_meta.get("global_path_hash_at_setup"),
        "global_replan_occurred": (
            nav.get("global_path_revision") is not None
            and setup_meta.get("global_path_revision_at_setup") is not None
            and int(nav.get("global_path_revision") or 0) > int(setup_meta.get("global_path_revision_at_setup") or 0)
        ),
        "collision": safety_collision,
        "physical_trajectory": pt if pt else None,
        "retreat_trajectory": retreat if retreat else None,
        "trajectory_source": pt.get("trajectory_source") or pt.get("source") if pt else None,
        "trajectory_kind": pt.get("trajectory_kind") if pt else None,
        "trajectory_control_eligible": pt.get("control_eligible") if pt else None,
        "trajectory_visualization_only": pt.get("visualization_only") if pt else None,
        "trajectory_frame": pt.get("frame_id") if pt else None,
        "trajectory_timestamp": pt.get("trajectory_timestamp") if pt else None,
        "trajectory_generated_at": pt.get("trajectory_generated_at") if pt else None,
        "planner_input_timestamp": pt.get("planner_input_timestamp") or nav.get("planner_input_timestamp"),
        "planner_start_timestamp": pt.get("planner_start_timestamp") or nav.get("planner_start_timestamp"),
        "planner_finish_timestamp": pt.get("planner_finish_timestamp") or nav.get("planner_finish_timestamp"),
        "planner_cycle_id": pt.get("planner_cycle_id") or nav.get("planner_cycle_id"),
        "trajectory_cycle_id": pt.get("trajectory_cycle_id") or pt.get("planner_cycle_id"),
        "scene_id": pt.get("scene_id") if pt else nav.get("scene_id") or nav.get("nav_scene_id"),
        "nav_scene_id": nav.get("nav_scene_id") or nav.get("scene_id"),
        "anchor_error_m": integ.get("raw_anchor_error_m") if integ.get("raw_anchor_error_m") is not None else integ.get("anchor_error_m"),
        "anchor_yaw_error_deg": integ.get("anchor_yaw_error_deg"),
        "trajectory_age_ms": integ.get("trajectory_age_ms"),
        "max_anchor_error_m": integ.get("max_reanchor_shift_m") or integ.get("max_anchor_error_m"),
        "trajectory_behind_vehicle": integ.get("behind_vehicle"),
        "trajectory_stale": integ.get("stale"),
        "integrity_reject": pt.get("integrity_reject") or integ.get("integrity_reject"),
        "trajectory_ineligible_reason": pt.get("trajectory_ineligible_reason") or integ.get("trajectory_ineligible_reason"),
        "direction_dot": pt.get("direction_dot") or integ.get("direction_dot"),
        "direction_angle": pt.get("direction_angle") or integ.get("direction_angle_deg"),
        "trajectory_reanchored": pt.get("stale_reanchor_applied") or integ.get("stale_reanchor_applied"),
        "reanchor_shift_m": pt.get("reanchor_shift_m") or integ.get("reanchor_shift_m"),
        "small_reanchor_applied": pt.get("small_reanchor_applied") or integ.get("small_reanchor_applied"),
        "mppi_vx": nav.get("mppi_vx"),
        "mppi_omega": nav.get("mppi_w") or nav.get("requested_omega"),
        "requested_vx": nav.get("requested_vx") or nav.get("cmd_vx_before_safety"),
        "requested_omega": nav.get("requested_omega") or nav.get("cmd_w_before_safety"),
        "approved_vx": nav.get("approved_vx") or nav.get("cmd_vx_after_safety"),
        "approved_omega": nav.get("approved_omega") or nav.get("cmd_w_after_safety"),
        "actual_vx": agv.get("vx") or nav.get("state_vx"),
        "actual_omega": agv.get("w") or nav.get("state_w"),
        "actual_yaw": agv.get("angle"),
        "command_source": nav.get("command_source") or ((st.get("debug") or {}).get("command_ownership") or {}).get("command_source"),
        "command_reason": nav.get("command_reason") or nav.get("command_source_reason"),
        "command_source_module": nav.get("command_source_module"),
        "command_write_trace": nav.get("command_write_trace"),
        "command_fallback": nav.get("command_fallback"),
        "command_side": nav.get("command_side"),
        "follow_path_source": nav.get("follow_path_source"),
        "local_plan_status": nav.get("local_plan_status"),
        "visualization_control_mismatch": nav.get("visualization_control_mismatch"),
        "pp_w": nav.get("pp_w"),
        "safety_direction_override": nav.get("safety_direction_override"),
        "global_path_heading": nav.get("global_path_heading"),
        "command_actual_sign": nav.get("command_actual_sign"),
        "actuator_direction_mismatch": nav.get("actuator_direction_mismatch"),
        "motion_health": nav.get("motion_health"),
        "limited_omega": nav.get("limited_omega"),
        "speed_limit_reason": nav.get("speed_limit_reason"),
        "curve_vmax": nav.get("curve_vmax"),
        "future_max_abs_kappa": nav.get("future_max_abs_kappa"),
        "ax": (nav.get("motion") or {}).get("rates", {}).get("ax") if isinstance(nav.get("motion"), dict) else None,
        "alpha": (nav.get("motion") or {}).get("rates", {}).get("alpha") if isinstance(nav.get("motion"), dict) else None,
        "lateral_acceleration": (nav.get("motion") or {}).get("rates", {}).get("lateral_acceleration") if isinstance(nav.get("motion"), dict) else None,
        "oscillation_score": ((nav.get("motion") or {}).get("oscillation") or {}).get("oscillation_score") if isinstance(nav.get("motion"), dict) else None,
        "heading_overshoot": ((nav.get("motion") or {}).get("overshoot") or {}).get("heading_overshoot") if isinstance(nav.get("motion"), dict) else None,
        "limit_cycle_suspected": (nav.get("motion") or {}).get("limit_cycle_suspected") if isinstance(nav.get("motion"), dict) else None,
        "control_eligible": pt.get("control_eligible") if pt else None,
        "turn_readiness": _dig(nav, "motion", "turn", "turn_readiness") or _dig(nav, "turn", "turn_readiness"),
        "turn_feasibility": _dig(nav, "motion", "turn", "turn_feasibility") or _dig(nav, "turn", "turn_feasibility"),
        "turn_required_distance_m": _dig(nav, "motion", "turn", "turn_required_distance_m") or _dig(nav, "turn", "turn_required_distance_m"),
        "turn_margin_m": _dig(nav, "motion", "turn", "turn_margin_m") or _dig(nav, "turn", "turn_margin_m"),
        "command_age_ms": _dig(nav, "motion", "turn", "command_age_ms") or _dig(nav, "turn", "command_age_ms"),
        "max_trajectory_age_ms": integ.get("max_age_ms"),
        "planner_compute_ms": integ.get("planner_compute_ms"),
        "planner_compute_p50_ms": nav.get("planner_compute_p50"),
        "planner_compute_p95_ms": nav.get("planner_compute_p95"),
        "planner_compute_max_ms": nav.get("planner_compute_max"),
        "planner_inflight": nav.get("planner_inflight"),
        "planner_timeout_count": nav.get("planner_timeout_count"),
        "planner_drop_count": nav.get("planner_drop_count"),
        "planner_late_completion_count": nav.get("planner_late_completion_count"),
        "planner_apply_drop_count": nav.get("planner_apply_drop_count"),
    }
    row["trajectory_side"] = _traj_side(row.get("direction_angle") or (pt.get("direction_angle") if pt else None))
    return row


def _traj_side(ang: Any) -> Optional[str]:
    if ang is None:
        return None
    try:
        a = float(ang)
    except (TypeError, ValueError):
        return None
    if a > 15.0:
        return "LEFT"
    if a < -15.0:
        return "RIGHT"
    return "STRAIGHT"


def _omega_sign(w: Any) -> Optional[str]:
    if w is None:
        return None
    try:
        ww = float(w)
    except (TypeError, ValueError):
        return None
    if ww > 0.02:
        return "LEFT"
    if ww < -0.02:
        return "RIGHT"
    return "STRAIGHT"


def _corridor_side(ec: Any) -> Optional[str]:
    if not isinstance(ec, dict):
        return None
    mode = str(ec.get("mode") or "").upper()
    if mode in ("LEFT", "RIGHT"):
        return mode
    return ec.get("committed_side")


def _detect_transitions(prev: Optional[dict], cur: dict) -> List[dict]:
    if prev is None:
        return [{"event": "TRACE_START", "timestamp": cur["ts"], "scene": cur.get("scene")}]
    out = []
    for key in _TRACK_KEYS:
        old = prev.get(key)
        new = cur.get(key)
        if old != new and (old is not None or new is not None):
            out.append(
                {
                    "event": "STATE_TRANSITION",
                    "timestamp": cur["ts"],
                    "seq": cur.get("seq"),
                    "field": key,
                    "previous_state": old,
                    "new_state": new,
                    "reason": cur.get("safe_vx_reason") if key == "planner_state" else cur.get("planner_failure_reason"),
                }
            )
    return out


def _sync_world_scene(world: SimWorld, map_scene: str) -> None:
    if map_scene == "m32_open_straight":
        world.load_m32_open_straight()
    elif map_scene in ("indoor_office", "indoor", "office"):
        world.load_indoor()


def _baseline_pose(client: NavLiveClient, start: dict) -> Tuple[float, float, float]:
    st = client.get("/api/state?lite=1")
    agv = st.get("agv") or {}
    sx = float(start.get("x") or 0)
    sy = float(start.get("y") or 0)
    x = float(agv.get("x") or sx)
    y = float(agv.get("y") or sy)
    return x, y, math.hypot(x - sx, y - sy)


def _scene_phase(injected: bool, row: dict, spec_test_class: str) -> str:
    if not injected:
        return "BASELINE"
    phase = str(row.get("avoidance_phase") or "").upper()
    if "PASS" in phase or row.get("nav_state") == "arrived":
        return "OBSTACLE_PASSED"
    if row.get("recovery_state") not in (None, "", "NONE") or row.get("planner_state") not in (None, "", "NORMAL"):
        if spec_test_class == "LOCAL_AVOIDANCE_ONLINE" and row.get("global_replan_occurred"):
            return "GLOBAL_REPLAN"
        return "LOCAL_AVOIDANCE"
    if phase in ("SIDE_PROBE", "SIDE_DECISION", "SIDE_COMMIT", "EXECUTION_CORRIDOR", "OBSTACLE_APPROACH"):
        return "LOCAL_AVOIDANCE"
    return "OBSTACLE_INJECTED"


def run_scene(
    client: NavLiveClient,
    world: SimWorld,
    scene_id: str,
    seconds: float,
    hz: float,
    seq_start: int,
) -> Tuple[List[dict], int, dict, str]:
    spec = SCENARIOS[scene_id]
    _sync_world_scene(world, spec.map_scene)
    setup = apply_scenario(client, world, scene_id)
    if not setup.get("success"):
        return (
            [{"ts": time.time(), "seq": seq_start, "scene": scene_id, "error": setup.get("failure_class", "SETUP_FAILED"), "setup": setup, "schema": "navigation_v0.2_trace/2"}],
            seq_start + 1,
            setup,
            "",
        )

    st0 = client.get("/api/state?lite=1")
    nav0 = st0.get("nav") or {}
    setup["global_path_hash_at_setup"] = global_path_fingerprint(nav0.get("path") or [])
    setup["global_path_revision_at_setup"] = nav0.get("global_path_revision")
    setup["scene_phase"] = "BASELINE"
    setup["test_class"] = spec.test_class
    sx0, sy0 = float(spec.start["x"]), float(spec.start["y"])
    baseline_x0 = float((st0.get("agv") or {}).get("x") or sx0)
    baseline_y0 = float((st0.get("agv") or {}).get("y") or sy0)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = _SCENE_FILE_TAG.get(scene_id, scene_id)
    rows: List[dict] = []
    seq = seq_start
    prev_row: Optional[dict] = None
    t0 = time.time()
    injected = False
    critical_stop: Optional[str] = None
    period = 1.0 / max(0.2, hz)
    deadline = t0 + max(1.0, seconds)
    inject_rev_before: Optional[int] = None
    inject_hash_before: Optional[str] = None

    while time.time() < deadline and critical_stop is None:
        tick_start = time.time()
        elapsed = tick_start - t0
        st_pre = client.get("/api/state?lite=1")
        agv_pre = st_pre.get("agv") or {}
        nav_pre = st_pre.get("nav") or {}
        progress_m = math.hypot(float(agv_pre.get("x") or baseline_x0) - baseline_x0, float(agv_pre.get("y") or baseline_y0) - baseline_y0)
        vx_pre = float(agv_pre.get("vx") or 0)

        should_inject = False
        if (not injected) and spec.inject_fn:
            if spec.test_class == "LOCAL_AVOIDANCE_ONLINE":
                should_inject = progress_m >= float(spec.inject_min_progress_m or 0) and vx_pre >= float(spec.inject_min_vx or 0)
            elif elapsed >= spec.inject_delay_s:
                should_inject = True

        if should_inject:
            px, py = float(agv_pre.get("x") or 0), float(agv_pre.get("y") or 0)
            yaw = float(agv_pre.get("angle") or 0)
            inject_rev_before = int(nav_pre.get("global_path_revision") or 0)
            inject_hash_before = global_path_fingerprint(nav_pre.get("path") or [])
            _sync_world_scene(world, spec.map_scene)
            spec.inject_fn(client, world, (px, py), yaw)
            injected = True
            setup["scene_phase"] = "OBSTACLE_INJECTED"
            rows.append(
                {
                    "ts": time.time(),
                    "seq": seq,
                    "scene": scene_id,
                    "event": "ONLINE_OBSTACLE_INJECT",
                    "elapsed_s": round(elapsed, 3),
                    "vehicle_progress_m": round(progress_m, 4),
                    "vehicle_vx": vx_pre,
                    "global_path_revision_before": inject_rev_before,
                    "global_path_hash_before": inject_hash_before,
                    "schema": "navigation_v0.2_trace/3",
                }
            )
            seq += 1

        try:
            sample = _sample_v02(client, seq, scene_id, setup)
            sample["scene_phase"] = _scene_phase(injected, sample, spec.test_class)
            setup["scene_phase"] = sample["scene_phase"]
            if injected and spec.test_class == "LOCAL_AVOIDANCE_ONLINE":
                if sample.get("global_replan_occurred") and inject_rev_before is not None:
                    sample["global_replan_after_inject"] = True
                if inject_hash_before and sample.get("global_path_hash") != inject_hash_before:
                    sample["global_path_hash_changed"] = True
            transitions = _detect_transitions(prev_row, sample)
            if transitions:
                sample["transitions"] = transitions
            rows.append(sample)
            prev_row = sample
            seq += 1
            if sample.get("collision"):
                critical_stop = "P0_COLLISION"
                rows.append({"ts": time.time(), "seq": seq, "scene": scene_id, "event": critical_stop, "schema": "navigation_v0.2_trace/3"})
                seq += 1
            elif sample.get("trajectory_behind_vehicle"):
                pt_now = sample.get("physical_trajectory") if isinstance(sample.get("physical_trajectory"), dict) else {}
                still_control = bool(pt_now.get("control_eligible"))
                masked = bool((pt_now.get("integrity") or {}).get("stale_reanchor_applied") or pt_now.get("stale_reanchor_applied"))
                if still_control or masked:
                    critical_stop = "P0_TRAJECTORY_INTEGRITY_FAILURE"
                    rows.append({"ts": time.time(), "seq": seq, "scene": scene_id, "event": critical_stop, "schema": "navigation_v0.2_trace/3"})
                    seq += 1
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            rows.append({"ts": time.time(), "seq": seq, "scene": scene_id, "error": str(exc), "schema": "navigation_v0.2_trace/3"})
            seq += 1
        remain = period - (time.time() - tick_start)
        if remain > 0:
            time.sleep(remain)

    if critical_stop:
        print(f"  CRITICAL: {critical_stop} — trace stopped early")

    return rows, seq, setup, f"{tag}-{stamp}.jsonl"


def main() -> int:
    ap = argparse.ArgumentParser(description="V0.2 navigation LIVE trace (M3.1 deterministic scenarios)")
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--scene", choices=ALL_SCENES + M33_SCENES + MOTION_SCENES + ["ALL", "OBS-OPEN-ALL", "ONLINE-ALL"], default="LIVE-00")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "navigation_v02"))
    ap.add_argument("--navigate", action="store_true", help="Deprecated — scenarios always plan+confirm")
    args = ap.parse_args()

    client = NavLiveClient(base=args.base)
    if not client.ping():
        print("SIM_NOT_RUNNING")
        print(f"Cannot reach {args.base}/api/state")
        print("RESULT: NOT RUN")
        return 2

    world = SimWorld()
    os.makedirs(args.out_dir, exist_ok=True)
    if args.scene == "ALL":
        scenes = ALL_SCENES + M33_SCENES
    elif args.scene == "OBS-OPEN-ALL":
        scenes = OBS_OPEN_SCENES
    elif args.scene == "ONLINE-ALL":
        scenes = ONLINE_SCENES
    else:
        scenes = [args.scene]

    written: List[str] = []
    seq = 0
    for sc in scenes:
        print(f"--- {sc}: {SCENARIOS[sc].description}")
        rows, seq, setup, fname = run_scene(client, world, sc, args.seconds, args.hz, seq)
        if not fname:
            print(f"SETUP FAIL: {setup}")
            continue
        jsonl = os.path.join(args.out_dir, fname)
        with open(jsonl, "w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")
        n_trans = sum(len(r.get("transitions") or []) for r in rows if isinstance(r.get("transitions"), list))
        n_err = sum(1 for r in rows if r.get("error"))
        print(f"  → {len(rows)} samples ({n_trans} transitions, {n_err} errors) {jsonl}")
        written.append(jsonl)

    if not written:
        print("RESULT: FAIL (no traces written)")
        return 1
    print(f"Wrote {len(written)} trace file(s)")
    print("RESULT: PASS (trace captured; analyze with _analyze_navigation_v02_trace.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
