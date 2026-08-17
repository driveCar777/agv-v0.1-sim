"""SimWorld / 3055 / 设点导航。

责任链：
Local(MPPI|PP) → Safety Supervisor → Accel Limit → Kinematic Integrate
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from agv_bridge.mppi_controller import kinematic_band
from agv_bridge.nav_debug import EVENT_MAX, NavDebugHub, TELEM_MAX
from agv_bridge.nav_geometry import (
    DEFAULT_GEOM,
    MAX_RECOVERY_ATTEMPTS,
    STUCK_PROGRESS_MIN_M,
    STUCK_REVERSE_TRIGGER_S,
    get_vehicle_model,
)
from agv_bridge.nav_planner_state import (
    LOCAL_RECOVERY,
    NAVIGATION_FAILED,
    PLANNER_SAFE_STOP,
    REASON_BRAKING_LIMIT,
    REASON_BRAKING_UNAVAILABLE,
    REASON_LOC_INVALID,
    REASON_MPPI_INFEASIBLE,
    REASON_NAV_FAILED,
    REASON_RECOVERY_ACTIVE,
    REASON_RECOVERY_EXHAUSTED,
    REASON_STALE_SENSOR,
    STOP_NAV_FAILED,
    STOP_RECOVERY,
    STOP_SAFE,
    ui_severity,
)
from agv_bridge.nav_trajectory_validator import braking_feasible
from agv_bridge.nav_models import GlobalPlannerModel, LocalMppiModel
from agv_bridge.path_progress import ProgressTracker
from agv_bridge.sim_world import get_world

STOP_NONE = "NONE"
STOP_FRONT = "FRONT_OBSTACLE"
STOP_REAR = "REAR_OBSTACLE"
STOP_COLLISION = "COLLISION_GUARD"
STOP_EMERGENCY = "EMERGENCY"
STOP_STUCK = "STUCK_RECOVERY"
STOP_REVERSE = "REVERSE_ESCAPE"
STOP_REPLAN = "REPLAN"
STOP_GOAL = "GOAL_REACHED"
STOP_FAILED = "FAILED"
STOP_MANUAL = "MANUAL_STOP"
STOP_NO_GLOBAL = "NO_GLOBAL_PATH"

SAFE_VX_NORMAL = "NORMAL"
SAFE_VX_FRONT = "FRONT_CLEARANCE_VETO"
SAFE_VX_REAR = "REAR_CLEARANCE_VETO"
SAFE_VX_COLLISION = "COLLISION_GUARD"
SAFE_VX_EMERGENCY = "EMERGENCY_STOP"
SAFE_VX_FAILED = "PLANNER_FAILED"
SAFE_VX_FP = "FOOTPRINT_CLEARANCE_VETO"
SAFE_VX_MPPI = "MPPI_NO_FEASIBLE_TRAJECTORY"
SAFE_VX_RECOVERY = "RECOVERY_ACTIVE"
SAFE_VX_BRAKING = "BRAKING_LIMIT"
SAFE_VX_BRAKING_NA = "BRAKING_MODEL_UNAVAILABLE"
SAFE_VX_STALE = "STALE_SENSOR"
SAFE_VX_LOC = "LOCALIZATION_INVALID"
SAFE_VX_NAV_FAILED = "NAVIGATION_FAILED"
SAFE_VX_RECOVERY_EX = "RECOVERY_EXHAUSTED"


def patch_mock_state(state) -> None:
    world = get_world()
    state._sim_world = world
    geom = DEFAULT_GEOM

    global_planner = GlobalPlannerModel()
    local_mppi = LocalMppiModel()
    progress = ProgressTracker()
    debug_hub = NavDebugHub()
    debug_hub.debug_level = "ADVANCED"
    state._nav_debug_hub = debug_hub
    state._debug_pp_w = 0.0
    state._debug_selected_candidate = None
    state._debug_best_cost = 0.0
    state._path_lateral_m = 0.0
    state._path_heading_err = 0.0
    state._last_debug_sample_t = 0.0
    state._debug_snapshot: Dict[str, Any] = {}
    state._physical_corridor: Dict[str, Any] = {}
    state._safe_vx_reason = SAFE_VX_NORMAL
    state._planner_state = "NORMAL"
    state._planner_failure_reason = ""
    state._recovery_state = "NONE"
    state._recovery_attempt = 0
    state._predicted_min_clearance_m = None
    state._footprint_clearance_m = None

    state._cmd_vx = 0.0
    state._cmd_vy = 0.0
    state._cmd_w = 0.0
    state._cmd_stamp = 0.0
    state._cmd_timeout = 0.5
    state._path: List[Tuple[float, float]] = []
    state._planned_path: List[Tuple[float, float]] = []
    state._global_path: List[Tuple[float, float]] = []
    state._raw_global_path: List[Tuple[float, float]] = []
    state._planning_metrics: Dict[str, Any] = {}
    state._path_candidates: List[Dict[str, Any]] = []
    state._best_confidence = 0.0
    state._path_i = 0
    state._goal_xy: Optional[Tuple[float, float]] = None
    state._nav_mode = "idle"
    state._pending_confirm = False
    state._physics_started = False
    state._last_local_replan = 0.0
    state._last_global_replan = 0.0
    state._local_replan_count = 0
    state._global_replan_count = 0
    state._global_path_revision = 0
    state._global_reference: Dict[str, Any] = {}
    state._local_candidates_layer: Dict[str, Any] = {}
    state._selected_local: Dict[str, Any] = {}
    state._kinematic_validation: Dict[str, Any] = {}
    state._open_space_forensics: Dict[str, Any] = {}
    state._lookahead_forensics: Dict[str, Any] = {}
    state._obstacle_preview: Dict[str, Any] = {}
    state._local_plan: Dict[str, Any] = {}
    state._last_kv_id = ""
    state._last_kv_status = ""
    state._stuck_since = 0.0
    state._last_progress_dist = None
    state._track_mode = "forward"
    state._nav_phase = "forward"
    state._ctrl_note = ""
    state.max_vx = geom.max_vx
    state.max_w = geom.max_w

    # Telemetry / safety
    state._mppi_vx = 0.0
    state._mppi_w = 0.0
    state._cmd_vx_before_safety = 0.0
    state._cmd_w_before_safety = 0.0
    state._cmd_vx_after_safety = 0.0
    state._cmd_w_after_safety = 0.0
    state._front_near = 30.0
    state._rear_near = 30.0
    state._collision = False
    state._stop_reason = STOP_NONE
    state._path_progress_s = 0.0
    state._goal_distance = 0.0
    state._stuck_s = 0.0
    state._recovery_attempts = 0
    state._control_mode = local_mppi.control_mode
    state._last_log_key = ""

    sx, sy, syaw = world.spawn_pose()
    with state.lock:
        state.x, state.y, state.angle = sx, sy, syaw

    def _bump_global_revision() -> int:
        n = int(getattr(state, "_global_path_revision", 0) or 0) + 1
        state._global_path_revision = n
        return n

    def _clear_progress_and_recovery() -> None:
        progress.reset()
        local_mppi.reset()
        state._stuck_since = 0.0
        state._stuck_s = 0.0
        state._last_progress_dist = None
        state._path_progress_s = 0.0
        state._goal_distance = 0.0
        state._nav_phase = "forward"
        state._recovery_attempts = 0
        state._stop_reason = STOP_NONE
        state._mppi_vx = state._mppi_w = 0.0
        state._cmd_vx_before_safety = state._cmd_w_before_safety = 0.0
        state._cmd_vx_after_safety = state._cmd_w_after_safety = 0.0
        state._path_lateral_m = 0.0
        state._path_heading_err = 0.0
        debug_hub.reset_session()

    def set_translate(payload: Dict[str, Any]) -> Dict[str, Any]:
        with state.lock:
            if state.emergency or state.soft_emc:
                world.emit("api_stop", "急停中，平动被拒", level="danger")
                return {"ret_code": 1, "err_msg": "emergency"}
            if not state.locked_by:
                state.locked_by = "sim_auto"
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            w = float(payload.get("w", 0.0))
            state._cmd_vx = max(-state.max_vx, min(state.max_vx, vx))
            state._cmd_vy = max(-state.max_vx, min(state.max_vx, vy))
            state._cmd_w = max(-state.max_w, min(state.max_w, w))
            state._cmd_stamp = time.time()
            state.is_stop = abs(state._cmd_vx) < 1e-4 and abs(state._cmd_w) < 1e-4
            world.emit("api_3055", f"vx={state._cmd_vx:.2f} w={state._cmd_w:.2f}")
            return {"ret_code": 0}

    def set_rotate(payload: Dict[str, Any]) -> Dict[str, Any]:
        with state.lock:
            w = float(payload.get("w", payload.get("vw", 0.0)))
            state._cmd_vx = 0.0
            state._cmd_vy = 0.0
            state._cmd_w = max(-state.max_w, min(state.max_w, w))
            state._cmd_stamp = time.time()
            state.is_stop = abs(state._cmd_w) < 1e-4
            world.emit("api_3056", f"w={state._cmd_w:.2f}")
            return {"ret_code": 0}

    def soft_stop() -> Dict[str, Any]:
        with state.lock:
            state._cmd_vx = state._cmd_vy = state._cmd_w = 0.0
            state.vx = state.vy = state.w = 0.0
            state.r_vx = state.r_vy = state.r_w = 0.0
            state.is_stop = True
            state._path = []
            state._planned_path = []
            state._global_path = []
            state._raw_global_path = []
            _bump_global_revision()
            state._planning_metrics = {}
            state._path_candidates = []
            state._best_confidence = 0.0
            state._goal_xy = None
            state._nav_mode = "idle"
            state._pending_confirm = False
            state.task_status = 6 if state.task_status == 2 else state.task_status
            _clear_progress_and_recovery()
            state._stop_reason = STOP_MANUAL
            world.emit("api_stop", "MANUAL_STOP", level="warn")
            debug_hub.events.push("MANUAL_STOP", {"stop_reason": STOP_MANUAL}, min_interval_s=0.0)
            return {"ret_code": 0}

    state.set_translate = set_translate
    state.set_rotate = set_rotate
    state.soft_stop = soft_stop

    def laser() -> Dict[str, Any]:
        with state.lock:
            x, y, yaw = state.x, state.y, state.angle
        dual = world.dual_lidar(x, y, yaw, step_deg=2.0)
        near = float(dual["front_near"])
        if near < 0.6:
            world.emit("avoid", f"前向最近 {near:.2f} m", level="warn", min_interval_s=1.5)
        elif near < 1.5:
            world.emit("slow", f"前向最近 {near:.2f} m", level="warn", min_interval_s=2.0)
        else:
            world.emit("lidar_see", f"前向通畅≈{near:.1f} m", min_interval_s=4.0)
        return {
            "ret_code": 0,
            "lasers": [dual["front"], dual["rear"]],
            "front_near": near,
            "rear_near": float(dual.get("rear_near", 30.0)),
        }

    state.laser = laser

    def _local_once(
        sx,
        sy,
        syaw,
        gpath,
        goal,
        stuck_s: float,
        front_near: float,
        now: float,
        force_rev: bool,
        *,
        rear_near: float = 30.0,
        collision: bool = False,
        emergency: bool = False,
        path_progress: float = 0.0,
        lateral_error: float = 0.0,
        actual_clearance: float = 1.0,
        nav_active: bool = True,
        dynamic_short: bool = False,
        dynamic_long: bool = False,
        arrived: bool = False,
        state_vx: float = 0.0,
        safety_zero: bool = False,
        planned_rejected_by_safety: bool = False,
    ):
        return local_mppi.step(
            world,
            sx,
            sy,
            syaw,
            gpath,
            goal,
            front_near=front_near,
            stuck_s=stuck_s,
            now=now,
            force_reverse=force_rev,
            rear_near=rear_near,
            collision=collision,
            emergency=emergency,
            path_progress=path_progress,
            lateral_error=lateral_error,
            actual_clearance=actual_clearance,
            candidates_hint=list(getattr(state, "_path_candidates", []) or []),
            nav_active=nav_active,
            dynamic_short=dynamic_short,
            dynamic_long=dynamic_long,
            arrived=arrived,
            state_vx=state_vx,
            safety_zero=safety_zero,
            planned_rejected_by_safety=planned_rejected_by_safety,
        )

    def _log_decision(key: str, tip: str, level: str = "info") -> None:
        if key == state._last_log_key:
            return
        state._last_log_key = key
        world.emit("nav_decision", tip, level=level, min_interval_s=0.0)

    def apply_safety(
        cmd_vx: float,
        cmd_w: float,
        *,
        front_near: float,
        rear_near: float,
        footprint_clearance: Optional[float],
        predicted_min_clearance: Optional[float],
        mppi_failure_reason: Optional[str],
        colliding: bool,
        emergency: bool,
        phase: str,
        planner_state: str,
        state_vx: float,
        sensor_stale: bool = False,
        localization_invalid: bool = False,
    ) -> Tuple[float, float, str, str]:
        """最终安全裁决。返回 (vx, w, stop_reason, safe_vx_reason)。"""
        reason = STOP_NONE
        safe_vx_reason = SAFE_VX_NORMAL
        vx, w = cmd_vx, cmd_w
        turning = phase in ("align", "turn_in_place", "reposition", "local_avoid")
        braking = get_vehicle_model().braking

        if emergency:
            return 0.0, 0.0, STOP_EMERGENCY, SAFE_VX_EMERGENCY
        if sensor_stale:
            return 0.0, 0.0, STOP_SAFE, SAFE_VX_STALE
        if localization_invalid:
            return 0.0, 0.0, STOP_SAFE, SAFE_VX_LOC
        if planner_state == NAVIGATION_FAILED:
            return 0.0, 0.0, STOP_NAV_FAILED, SAFE_VX_NAV_FAILED
        if planner_state == PLANNER_SAFE_STOP:
            return 0.0, 0.0, STOP_SAFE, SAFE_VX_RECOVERY_EX
        if planner_state == LOCAL_RECOVERY:
            safe_vx_reason = SAFE_VX_RECOVERY
            if vx > 0.0:
                vx = min(vx, geom.max_vx * 0.55)
        if mppi_failure_reason == SAFE_VX_MPPI:
            return 0.0, 0.0, STOP_RECOVERY, SAFE_VX_MPPI
        if colliding:
            # SIL: do not invent reverse during align/turn; hard stop and let Maneuver decide
            if turning:
                return 0.0, 0.0, STOP_COLLISION, SAFE_VX_COLLISION
            if rear_near > geom.rear_stop_m + 0.15 and vx >= 0 and phase == "reverse_escape":
                return -0.10, w * 0.3, STOP_COLLISION, SAFE_VX_COLLISION
            if vx >= 0:
                return 0.0, 0.0, STOP_COLLISION, SAFE_VX_COLLISION
            # already reversing: keep rear gate below
        if footprint_clearance is not None and footprint_clearance < geom.safety_margin_m:
            return 0.0, 0.0, STOP_FRONT, SAFE_VX_FP
        clr_for_brake = predicted_min_clearance if predicted_min_clearance is not None else front_near
        if vx > 0.04:
            ok, br_reason = braking_feasible(
                max(vx, abs(state_vx)),
                float(clr_for_brake),
                braking,
                hard_stop_m=geom.front_stop_m,
            )
            if not ok:
                if br_reason == REASON_BRAKING_UNAVAILABLE:
                    cap = max(0.04, min(vx, front_near * 0.35))
                    if cap < vx - 0.02:
                        return cap, w * 0.5, STOP_NONE, SAFE_VX_BRAKING_NA
                else:
                    return 0.0, w * 0.2, STOP_FRONT, SAFE_VX_BRAKING
        if vx < 0 and rear_near < geom.rear_stop_m:
            return 0.0, 0.0, STOP_REAR, SAFE_VX_REAR
        # Front obstacle: allow pure yaw during align/turn (vx≈0) if not colliding
        if front_near < geom.front_stop_m and vx >= 0:
            if turning and abs(vx) < 0.04:
                return 0.0, w, STOP_NONE if abs(w) > 0.02 else STOP_FRONT, SAFE_VX_NORMAL if abs(w) > 0.02 else SAFE_VX_FRONT
            return 0.0, (w * 0.2 if abs(w) > 1e-6 else 0.0), STOP_FRONT, SAFE_VX_FRONT
        if phase == "reverse_escape":
            reason = STOP_REVERSE
        return vx, w, reason, safe_vx_reason

    def plan_nav_xy(gx: float, gy: float, gyaw: float = 0.0, target_id: str = "") -> Dict[str, Any]:
        with state.lock:
            sx, sy, syaw = state.x, state.y, state.angle
        world.emit("planning", f"全局 A* ({gx:.2f}, {gy:.2f})")
        gpath = global_planner.plan(world, (sx, sy), (gx, gy))
        if not gpath:
            world.emit("nav_stop", f"{STOP_NO_GLOBAL}: 目标无路", level="danger")
            debug_hub.events.push("PLAN_FAILED", {"stop_reason": STOP_NO_GLOBAL}, min_interval_s=0.0)
            return {"ret_code": 1, "err_msg": "no path", "waypoints": 0, "path": []}
        _clear_progress_and_recovery()
        debug_hub.begin_session((gx, gy))
        dual = world.dual_lidar(sx, sy, syaw, step_deg=6.0)
        res = _local_once(
            sx, sy, syaw, gpath, (gx, gy), 0.0, float(dual.get("front_near", 30.0)), time.time(), False
        )
        tid = target_id or f"XY_{gx:.2f}_{gy:.2f}"
        with state.lock:
            state._global_path = gpath
            state._raw_global_path = list(global_planner.last_raw_path)
            _bump_global_revision()
            state._planning_metrics = dict(global_planner.last_metrics)
            state._planned_path = list(res.best_path)
            state._path = list(res.best_path)
            state._path_candidates = res.candidates
            state._best_confidence = res.confidence
            state._track_mode = res.mode
            state._nav_phase = local_mppi.phase
            state._ctrl_note = res.control_mode
            state._control_mode = res.control_mode
            state._path_i = 0
            state._goal_xy = (gx, gy)
            state._nav_mode = "planned"
            state._pending_confirm = True
            state._last_local_replan = time.time()
            state._last_global_replan = time.time()
            state._local_replan_count = 1
            state._global_replan_count = 1
            state.vx = state.w = 0.0
            state._cmd_vx = state._cmd_w = 0.0
            state.task_status = 1
            state.task_type = 3
            state.target_id = tid
            state.unfinished_path = [tid]
            if not state.locked_by:
                state.locked_by = "sim_nav"
        world.emit("goal_set", f"目标 ({gx:.2f}, {gy:.2f})", level="success")
        world.emit(
            "planned",
            f"全局{len(gpath)} · 候选{len(res.candidates)} · {res.control_mode}",
            level="warn",
        )
        debug_hub.events.push(
            "PLAN_SUCCESS",
            {
                "waypoints": len(gpath),
                "candidates": len(res.candidates),
                "path_ratio": (global_planner.last_metrics or {}).get("path_ratio"),
            },
            min_interval_s=0.0,
        )
        return {
            "ret_code": 0,
            "waypoints": len(gpath),
            "path": [{"x": p[0], "y": p[1]} for p in gpath],
            "raw_path": [{"x": p[0], "y": p[1]} for p in global_planner.last_raw_path],
            "planning": dict(global_planner.last_metrics),
            "local_path": [{"x": p[0], "y": p[1]} for p in res.best_path],
            "candidates": res.candidates,
            "confidence": res.confidence,
            "pending_confirm": True,
            "target_id": tid,
            "planner": "astar_clearance+los+pp_mppi",
        }

    def plan_quality_debug(gx: float, gy: float, compare: bool = True) -> Dict[str, Any]:
        """Global-path-only diagnosis: no MPPI / PP / Safety / Recovery."""
        with state.lock:
            sx, sy = state.x, state.y
        start, goal = (sx, sy), (float(gx), float(gy))
        variants = world.compare_path_variants(start, goal, robot_r=geom.planner_radius) if compare else {}
        # Default processed path = D
        primary = world.plan_path_quality(
            start,
            goal,
            robot_r=geom.planner_radius,
            use_clearance=True,
            use_turn=True,
            use_los=True,
            use_smooth=True,
            variant="D",
        )
        with state.lock:
            state._raw_global_path = list(primary.raw_path)
            state._global_path = list(primary.path)
            _bump_global_revision()
            state._planning_metrics = primary.metrics.to_dict()
            state._planned_path = []
            state._path = []
            state._path_candidates = []
            state._nav_mode = "planner_debug"
            state._pending_confirm = False
            state._goal_xy = goal
            state.vx = state.w = 0.0
            state._cmd_vx = state._cmd_w = 0.0
        return {
            "ret_code": 0,
            "success": bool(primary.path),
            "planner_debug": True,
            "start": {"x": start[0], "y": start[1]},
            "goal": {"x": goal[0], "y": goal[1]},
            "raw_path": [{"x": p[0], "y": p[1]} for p in primary.raw_path],
            "path": [{"x": p[0], "y": p[1]} for p in primary.path],
            "planning": primary.metrics.to_dict(),
            "variants": variants,
        }

    def confirm_nav() -> Dict[str, Any]:
        with state.lock:
            if not state._pending_confirm or state._nav_mode != "planned" or not state._path:
                return {"ret_code": 1, "err_msg": "no pending plan"}
            state._pending_confirm = False
            state._nav_mode = "tracking"
            state.task_status = 2
            state._last_local_replan = 0.0
            progress.reset()
            state._stuck_s = 0.0
            state._stuck_since = 0.0
        world.emit("confirm_go", level="success")
        debug_hub.events.push("TRACKING_STARTED", {}, min_interval_s=0.0)
        return {"ret_code": 0}

    def start_nav_xy(gx: float, gy: float, gyaw: float = 0.0, auto_start: bool = False) -> Dict[str, Any]:
        resp = plan_nav_xy(gx, gy, gyaw)
        if resp.get("ret_code") != 0:
            return resp
        if auto_start:
            return confirm_nav()
        return resp

    def apply_scene(scene_id: str) -> Dict[str, Any]:
        r = world.set_scene(scene_id)
        if not r.get("success"):
            return r
        sx, sy, syaw = world.spawn_pose()
        with state.lock:
            state.x, state.y, state.angle = sx, sy, syaw
            state._path = []
            state._planned_path = []
            state._global_path = []
            state._raw_global_path = []
            _bump_global_revision()
            state._planning_metrics = {}
            state._path_candidates = []
            state._best_confidence = 0.0
            state._goal_xy = None
            state._nav_mode = "idle"
            state._pending_confirm = False
            state.vx = state.w = 0.0
            state._cmd_vx = state._cmd_w = 0.0
            state.task_status = 0
            _clear_progress_and_recovery()
        return r

    def set_control_mode(mode: str) -> Dict[str, Any]:
        m = local_mppi.set_control_mode(mode)
        with state.lock:
            state._control_mode = m
        return {"ret_code": 0, "control_mode": m}

    state.plan_nav_xy = plan_nav_xy
    state.confirm_nav = confirm_nav
    state.start_nav_xy = start_nav_xy
    state.apply_scene = apply_scene
    state.set_control_mode = set_control_mode
    state.plan_quality_debug = plan_quality_debug
    state._reset_nav_session = _clear_progress_and_recovery
    state._global_planner = global_planner

    def _lookahead_point(path: List[Tuple[float, float]], x: float, y: float, lookahead_m: float = 1.4):
        if not path:
            return None
        best_i, best_d = 0, 1e18
        for i, p in enumerate(path):
            d = math.hypot(p[0] - x, p[1] - y)
            if d < best_d:
                best_d, best_i = d, i
        acc = 0.0
        prev = path[best_i]
        for p in path[best_i:]:
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
            if acc >= lookahead_m:
                return p
        return path[-1]

    def _refresh_debug_snapshot(now: float) -> None:
        if debug_hub.freeze and debug_hub.frozen_snapshot is not None:
            state._debug_snapshot = debug_hub.frozen_snapshot
            return
        with state.lock:
            x, y, yaw = state.x, state.y, state.angle
            nav_mode = state._nav_mode
            phase = state._nav_phase
            control_mode = state._control_mode
            stop_reason = state._stop_reason
            goal_xy = state._goal_xy
            gpath = list(state._global_path or [])
            raw = list(state._raw_global_path or [])
            executed = list(state._path or [])
            planned = list(state._planned_path or [])
            cands = list(state._path_candidates or [])
            planning = dict(state._planning_metrics or {})
            mppi_vx = float(state._mppi_vx)
            mppi_w = float(state._mppi_w)
            cmd_vx = float(state._cmd_vx_before_safety)
            cmd_w = float(state._cmd_w_before_safety)
            safe_vx = float(state._cmd_vx_after_safety)
            safe_w = float(state._cmd_w_after_safety)
            state_vx = float(state.vx)
            state_w = float(state.w)
            front_near = float(state._front_near)
            rear_near = float(state._rear_near)
            collision = bool(state._collision)
            emergency = bool(state.emergency or state.soft_emc)
            path_progress_s = float(state._path_progress_s)
            goal_distance = float(state._goal_distance)
            path_i = int(getattr(state, "_path_i", 0) or 0)
            path_rev = int(getattr(state, "_global_path_revision", 0) or 0)
            stuck_s = float(state._stuck_s)
            recovery_attempts = int(state._recovery_attempts)
            safe_vx_reason = str(getattr(state, "_safe_vx_reason", SAFE_VX_NORMAL) or SAFE_VX_NORMAL)
            planner_state = str(getattr(state, "_planner_state", "NORMAL") or "NORMAL")
            planner_failure_reason = str(getattr(state, "_planner_failure_reason", "") or "")
            recovery_state = str(getattr(state, "_recovery_state", "NONE") or "NONE")
            recovery_attempt = int(getattr(state, "_recovery_attempt", 0) or 0)
            footprint_clearance_m = getattr(state, "_footprint_clearance_m", None)
            predicted_min_clearance_m = getattr(state, "_predicted_min_clearance_m", None)
            nav_ui_severity = str(getattr(state, "_nav_ui_severity", "NORMAL") or "NORMAL")
            global_n = int(state._global_replan_count)
            local_n = int(state._local_replan_count)
            confidence = float(state._best_confidence)
            lat = float(getattr(state, "_path_lateral_m", 0.0) or 0.0)
            herr = float(getattr(state, "_path_heading_err", 0.0) or 0.0)
            pp_w = float(getattr(state, "_debug_pp_w", 0.0) or 0.0)
            sel = getattr(state, "_debug_selected_candidate", None)
            best_cost = float(getattr(state, "_debug_best_cost", 0.0) or 0.0)
            first_col = getattr(state, "_debug_first_collision", None)
            mppi_meta = dict(getattr(state, "_debug_mppi_meta", {}) or {})
            maneuver = dict(getattr(state, "_maneuver", {}) or {})
            nav_policy = dict(getattr(state, "_nav_policy", {}) or {})
            cmd_source = str(getattr(state, "_cmd_source", "") or "")
            scene = world.scene_info()

        la = _lookahead_point(gpath, x, y)
        # clearance / nearest obstacle (diagnostics only)
        try:
            obs_info = world.nearest_obstacle_info(x, y, yaw)
            actual_clr = float(obs_info.get("vehicle_to_obstacle_clearance") or front_near)
        except Exception:
            obs_info = {}
            actual_clr = front_near
        path_clr = float((planning or {}).get("min_clearance") or 0.0)
        # candidate forward/reverse breakdown
        fwd_cands = [c for c in cands if float(c.get("vx") or 0) > 0.04]
        rev_cands = [c for c in cands if float(c.get("vx") or 0) < -0.04]
        best_fwd = min(fwd_cands, key=lambda c: float(c.get("cost") or 1e9), default=None)
        best_rev = min(rev_cands, key=lambda c: float(c.get("cost") or 1e9), default=None)
        forward_ok = any(float(c.get("vx") or 0) > 0.04 and not c.get("collision") for c in cands) if cands else False
        forward_traj = "AVAILABLE" if forward_ok else "NOT_AVAILABLE"

        phase_snap = {
            "x": x,
            "y": y,
            "theta": yaw,
            "forward_trajectory": forward_traj,
            "forward_candidate_count": len(fwd_cands),
            "best_forward_cost": (best_fwd or {}).get("cost"),
            "best_forward_vx": (best_fwd or {}).get("vx"),
            "best_forward_w": (best_fwd or {}).get("w"),
            "forward_collision": (best_fwd or {}).get("collision"),
            "forward_clearance": actual_clr,
            "reverse_feasible": bool(rev_cands),
            "reverse_candidate_count": len(rev_cands),
            "best_reverse_cost": (best_rev or {}).get("cost"),
            "best_reverse_vx": (best_rev or {}).get("vx"),
            "best_reverse_w": (best_rev or {}).get("w"),
            "reverse_collision": (best_rev or {}).get("collision"),
            "reverse_clearance": actual_clr,
            "front_near": front_near,
            "rear_near": rear_near,
            "path_progress_s": path_progress_s,
            "stuck_s": stuck_s,
            "path_clearance": path_clr,
            "actual_clearance": actual_clr,
            "state_vx": state_vx,
        }
        debug_hub.note_phase(phase, phase_snap)
        debug_hub.note_stop_reason(
            stop_reason,
            {
                "stop_reason": stop_reason,
                "phase": phase,
                "vx": state_vx,
                "w": state_w,
                "front_near": front_near,
                "rear_near": rear_near,
                "path_progress": path_progress_s,
            },
        )
        if abs(mppi_vx) < 0.02 and abs(safe_vx) < 0.02 and nav_mode in ("tracking", "avoid"):
            debug_hub.events.push(
                "SPEED_ZERO",
                {"mppi_vx": mppi_vx, "safe_vx": safe_vx, "front_near": front_near},
                min_interval_s=1.0,
            )

        # 20Hz-ish enrichment happens every refresh call; still compute ax via ingest
        debug_hub.ingest_sample(
            {
                "timestamp": now,
                "x": x,
                "y": y,
                "theta": yaw,
                "mppi_vx": mppi_vx,
                "cmd_vx": cmd_vx,
                "safe_vx": safe_vx,
                "state_vx": state_vx,
                "mppi_w": mppi_w,
                "cmd_w": cmd_w,
                "safe_w": safe_w,
                "state_w": state_w,
                "pp_w": pp_w,
                "desired_w": mppi_w,
                "front_near": front_near,
                "rear_near": rear_near,
                "collision": collision,
                "path_progress_s": path_progress_s,
                "lateral_error": lat,
                "heading_error": herr,
                "phase": phase,
                "stop_reason": stop_reason,
                "recovery_attempts": recovery_attempts,
                "forward_trajectory": forward_traj,
                "best_cost": best_cost,
                "best_forward_cost": (best_fwd or {}).get("cost"),
                "best_reverse_cost": (best_rev or {}).get("cost"),
                "best_forward_vx": (best_fwd or {}).get("vx"),
                "best_reverse_vx": (best_rev or {}).get("vx"),
                "selected_candidate": sel,
                "nav_mode": nav_mode,
                "control_mode": control_mode,
                "goal_distance": goal_distance,
                "stuck_s": stuck_s,
                "session_id": debug_hub.session_id,
                "actual_clearance": actual_clr,
                "path_clearance": path_clr,
                "nearest_obstacle_distance": obs_info.get("nearest_obstacle_distance", actual_clr),
                "nearest_obstacle_direction": obs_info.get("nearest_obstacle_direction", 0.0),
                "nearest_obstacle_point": obs_info.get("nearest_obstacle_point"),
                "maneuver_mode": maneuver.get("mode"),
                "maneuver_reason": maneuver.get("reason"),
                "forward_reason": maneuver.get("forward_reason"),
                "local_decision": maneuver.get("decision"),
                "policy_state": (nav_policy.get("state") if nav_policy else None),
                "policy_behavior": (nav_policy.get("behavior") if nav_policy else None),
                "safe_vx_reason": safe_vx_reason,
                "planner_state": planner_state,
                "planner_failure_reason": planner_failure_reason,
                "recovery_state": recovery_state,
                "recovery_attempt": recovery_attempt,
                "footprint_clearance_m": footprint_clearance_m,
                "predicted_min_clearance_m": predicted_min_clearance_m,
                "nav_ui_severity": nav_ui_severity,
                "path_follow_weight": ((nav_policy.get("decision") or {}).get("path_follow_weight") if nav_policy else None),
                "local_deviation": ((nav_policy.get("decision") or {}).get("corridor") or {}).get("lateral_error") if nav_policy else None,
                "fwd_cost": ((maneuver.get("local_compare") or {}).get("snapshot") or {}).get("forward_cost"),
                "left_cost": ((maneuver.get("local_compare") or {}).get("snapshot") or {}).get("left_cost"),
                "right_cost": ((maneuver.get("local_compare") or {}).get("snapshot") or {}).get("right_cost"),
            }
        )
        dbg = debug_hub.build(
            nav_mode=nav_mode,
            phase=phase,
            control_mode=control_mode,
            stop_reason=stop_reason,
            x=x,
            y=y,
            yaw=yaw,
            goal_xy=goal_xy,
            goal_distance=goal_distance,
            path_progress_s=path_progress_s,
            lateral_err=lat,
            heading_err=herr,
            stuck_s=stuck_s,
            planning=planning,
            raw_path=raw,
            processed_path=gpath,
            executed_path=executed,
            planned_path=planned,
            candidates=cands,
            mppi_vx=mppi_vx,
            mppi_w=mppi_w,
            pp_w=pp_w,
            safe_vx=safe_vx,
            safe_w=safe_w,
            final_vx=safe_vx,
            final_w=safe_w,
            state_vx=state_vx,
            state_w=state_w,
            front_near=front_near,
            rear_near=rear_near,
            collision=collision,
            emergency=emergency,
            recovery_attempts=recovery_attempts,
            global_replan_count=global_n,
            local_replan_count=local_n,
            confidence=confidence,
            selected_candidate=sel,
            best_cost=best_cost,
            scene=scene,
            lookahead_pt=la,
            first_collision=first_col,
            mppi_meta=mppi_meta,
            cmd_vx=cmd_vx,
            cmd_w=cmd_w,
            maneuver=maneuver,
            cmd_source=cmd_source,
            nav_policy=nav_policy,
        )
        if dbg.get("wall_approach") is not None:
            dbg["wall_approach"]["nearest_obstacle_point"] = obs_info.get("nearest_obstacle_point")
        # P0-B: Global Reference Preview — AFTER control, read-only (never cmd_vel)
        try:
            from agv_bridge.nav_global_preview import (
                build_global_reference,
                collect_local_candidates,
                expected_local_distance_m,
                preview_enabled,
            )

            if preview_enabled():
                gref = build_global_reference(
                    path=gpath,
                    x=x,
                    y=y,
                    yaw=yaw,
                    path_progress_s=path_progress_s,
                    path_index=path_i,
                    goal_distance_m=goal_distance,
                    path_revision=path_rev,
                    goal_reached=str(nav_mode) == "arrived" or str(stop_reason) in ("GOAL_REACHED", "STOP_GOAL"),
                )
            else:
                gref = {"status": "DISABLED", "preview_m": 0.0, "kinematic_valid": None, "controls_vehicle": False}
            loc_layer = collect_local_candidates(
                maneuver=maneuver,
                path_candidates=cands,
                selected=sel or maneuver.get("decision") or maneuver.get("mode"),
                rolling_layer=(
                    local_mppi.local_planner.ui_layer()
                    if hasattr(local_mppi, "local_planner")
                    else None
                ),
            )
            lp_obj = getattr(local_mppi, "last_local_plan", None)
            spd_obj = getattr(local_mppi, "last_speed", None)
            lp_dict = lp_obj.to_dict() if lp_obj is not None and hasattr(lp_obj, "to_dict") else {}
            selected_local = {
                "candidate_id": loc_layer.get("selected_candidate") or "NONE",
                "source": loc_layer.get("source") or "local_compare",
                "plan_id": loc_layer.get("plan_id") or (lp_dict.get("plan_id") if lp_dict else None),
                "horizon_m": loc_layer.get("horizon_m") or (lp_dict.get("horizon_m") if lp_dict else None),
                "horizon_s": loc_layer.get("horizon_s") or (lp_dict.get("horizon_s") if lp_dict else None),
            }
            gvl = {
                "global_preview_m": gref.get("preview_m"),
                "global_remaining_m": gref.get("remaining_m"),
                "global_preview_reason": gref.get("preview_reason"),
                "global_path_revision": gref.get("path_revision"),
                "local_candidate_count": loc_layer.get("count"),
                "local_valid_count": loc_layer.get("valid_count"),
                "local_max_distance_m": loc_layer.get("max_distance_m"),
                "local_mean_distance_m": loc_layer.get("mean_distance_m"),
                "selected_candidate": selected_local.get("candidate_id"),
                "requested_vx": cmd_vx,
                "safe_vx": safe_vx,
                "state_vx": state_vx,
                "expected_local_distance_m": round(
                    expected_local_distance_m(state_vx=state_vx, requested_vx=cmd_vx), 3
                ),
            }
            # P0-C: kinematic validation AFTER control — telemetry only
            kv_api: Dict[str, Any] = {
                "status": "NOT_VALIDATED",
                "kinematic_valid": None,
                "kinematic_status": "NOT_VALIDATED",
                "controls_vehicle": False,
            }
            try:
                from agv_bridge.nav_kinematic import empty_validation, validate_global_path, validator_enabled

                if validator_enabled():
                    src_path = raw or gpath

                    def _occ(px: float, py: float) -> bool:
                        return bool(world.collides(px, py, robot_r=0.02, include_actors=True))

                    kv_res = validate_global_path(
                        src_path,
                        path_revision=path_rev,
                        collide=_occ,
                        clearance_at=lambda px, py: float(world.clearance_at_xy(px, py)),
                        goal_reached=str(nav_mode) == "arrived" or str(stop_reason) in ("GOAL_REACHED", "STOP_GOAL"),
                        cache_token=len(world.dyn_obstacles),
                    )
                    kv_api = kv_res.to_api(include_poses=False)
                else:
                    kv_api = empty_validation(reason="NOT_VALIDATED", revision=path_rev)
                    kv_api["status"] = "NOT_VALIDATED"
                    kv_api["kinematic_valid"] = None
                    kv_api["kinematic_status"] = "NOT_VALIDATED"
                    kv_api["valid"] = None
            except Exception:
                kv_api = {
                    "status": "DEGRADED",
                    "kinematic_valid": None,
                    "kinematic_status": "DEGRADED",
                    "reason": "NUMERIC_FAILURE",
                    "controls_vehicle": False,
                }
            gref["geometry_status"] = gref.get("geometry_status") or "REFERENCE_ONLY"
            gref["kinematic_status"] = kv_api.get("kinematic_status") or kv_api.get("status")
            gref["kinematic_valid"] = kv_api.get("kinematic_valid")
            gref["first_invalid_distance_m"] = kv_api.get("first_invalid_distance_m")
            gref["speed_limited"] = kv_api.get("speed_limited")
            gref["max_curvature"] = kv_api.get("max_curvature")
            gref["min_turn_radius_m"] = kv_api.get("min_turn_radius_m")
            gvl["kinematic_status"] = gref["kinematic_status"]
            gvl["kinematic_valid"] = gref["kinematic_valid"]
            dbg["global_reference"] = gref
            dbg["local_candidates"] = loc_layer
            dbg["selected_local"] = selected_local
            dbg["local_plan"] = lp_dict or {}
            dbg["speed_policy"] = spd_obj.to_dict() if spd_obj is not None and hasattr(spd_obj, "to_dict") else {}
            dbg["maneuver_authority"] = getattr(local_mppi, "last_maneuver_authority", None)
            fp_obj = getattr(local_mppi, "last_future_preview", None)
            sp_obj = getattr(local_mppi, "last_side_probe", None)
            av_obj = getattr(local_mppi, "last_avoidance_state", None)
            dr_obj = getattr(local_mppi, "last_dynamic_state", None)
            if fp_obj is not None and hasattr(fp_obj, "to_dict"):
                op = fp_obj.to_dict()
                op["local_plan_id"] = lp_dict.get("plan_id") if isinstance(lp_dict, dict) else None
                op["local_plan_authority"] = dbg.get("maneuver_authority")
                op["lookahead_source"] = op.get("lookahead_source") or (
                    "LOCAL_PLAN" if lp_dict.get("active") else "GLOBAL_PATH"
                )
                if sp_obj is not None and hasattr(sp_obj, "to_dict"):
                    op["side_probe"] = sp_obj.to_dict()
                    op["probe_active"] = sp_obj.probe_active
                    op["probe_confidence_left"] = sp_obj.left_confidence
                    op["probe_confidence_right"] = sp_obj.right_confidence
                    op["committed_side"] = sp_obj.committed_side_hint or sp_obj.preferred_side
                if av_obj is not None and hasattr(av_obj, "to_dict"):
                    op.update(av_obj.to_dict())
                if dr_obj is not None and hasattr(dr_obj, "to_dict"):
                    op["dynamic_resume"] = dr_obj.to_dict()
                    op["dynamic_state"] = dr_obj.dynamic_state
                    op["resume_block_reason"] = dr_obj.resume_block_reason
                spd_d = dbg.get("speed_policy") if isinstance(dbg.get("speed_policy"), dict) else {}
                op["speed_reason"] = spd_d.get("reason")
                dbg["obstacle_preview"] = op
                dbg["side_probe"] = op.get("side_probe") or {}
                dbg["avoidance_phase"] = op.get("avoidance_phase") or op.get("phase")
                dbg["dynamic_resume"] = op.get("dynamic_resume") or {}
                with state.lock:
                    state._obstacle_preview = op
            else:
                dbg["obstacle_preview"] = {}
            dbg["global_vs_local"] = gvl
            dbg["kinematic_validation"] = kv_api
            with state.lock:
                state._global_reference = gref
                state._local_candidates_layer = loc_layer
                state._selected_local = selected_local
                state._kinematic_validation = kv_api
                state._local_plan = lp_dict or {}
            try:
                from agv_bridge.nav_observability import OBS

                vid = str(kv_api.get("validation_id") or "")
                stt = str(kv_api.get("status") or "")
                prev_id = str(getattr(state, "_last_kv_id", "") or "")
                prev_st = str(getattr(state, "_last_kv_status", "") or "")
                if (vid and vid != prev_id) or (stt and stt != prev_st):
                    state._last_kv_id = vid or prev_id
                    state._last_kv_status = stt
                    payload = {
                        "validation_id": vid,
                        "path_revision": kv_api.get("path_revision"),
                        "status": stt,
                        "valid": kv_api.get("valid"),
                        "reason": kv_api.get("reason"),
                        "max_curvature": kv_api.get("max_curvature"),
                        "min_turn_radius_m": kv_api.get("min_turn_radius_m"),
                        "max_required_w_rad_s": kv_api.get("max_required_w_rad_s"),
                        "max_feasible_speed_mps": kv_api.get("max_feasible_speed_mps"),
                        "min_clearance_m": kv_api.get("min_clearance_m"),
                        "collision": kv_api.get("swept_collision"),
                        "first_invalid_distance_m": kv_api.get("first_invalid_distance_m"),
                        "cache_hit": kv_api.get("cache_hit"),
                        "compute_ms": kv_api.get("compute_ms"),
                    }
                    OBS.emit(
                        "KINEMATIC_VALIDATION_STARTED",
                        level="INFO",
                        category="KINEMATIC",
                        component="kinematic_validator",
                        data={"validation_id": vid, "path_revision": kv_api.get("path_revision")},
                        force=True,
                    )
                    OBS.emit(
                        "KINEMATIC_VALIDATION_RESULT",
                        level="WARN" if stt == "INVALID" else "INFO",
                        category="KINEMATIC",
                        component="kinematic_validator",
                        reason=kv_api.get("reason"),
                        data=payload,
                        force=True,
                    )
                    if stt == "INVALID" and kv_api.get("reason") not in ("NO_PATH", None, ""):
                        OBS.emit(
                            "KINEMATIC_PATH_REJECTED",
                            level="WARN",
                            category="KINEMATIC",
                            component="kinematic_validator",
                            reason=kv_api.get("reason"),
                            data=payload,
                            force=True,
                        )
                    if kv_api.get("reason") == "CLEARANCE_TOO_LOW" or "CLEARANCE_TOO_LOW" in (kv_api.get("violations") or []):
                        OBS.emit(
                            "KINEMATIC_CLEARANCE_WARNING",
                            level="WARN",
                            category="KINEMATIC",
                            component="kinematic_validator",
                            data=payload,
                            force=True,
                        )
                    if kv_api.get("speed_limited"):
                        OBS.emit(
                            "KINEMATIC_SPEED_LIMITED",
                            level="NOTICE",
                            category="KINEMATIC",
                            component="kinematic_validator",
                            data={
                                "speed_limited_from_m": kv_api.get("speed_limited_from_m"),
                                "max_feasible_speed_mps": kv_api.get("max_feasible_speed_mps"),
                                "max_required_w_rad_s": kv_api.get("max_required_w_rad_s"),
                            },
                            force=True,
                        )
            except Exception:
                pass
        except Exception:
            pass
        # P0-C.1: open-space forensics AFTER control + P0-C (telemetry only)
        try:
            from agv_bridge.nav_open_space_forensics import assemble_open_space_forensics

            forensic = assemble_open_space_forensics(
                dbg=dbg,
                gref=dbg.get("global_reference") if isinstance(dbg.get("global_reference"), dict) else {},
                loc_layer=dbg.get("local_candidates") if isinstance(dbg.get("local_candidates"), dict) else {},
                kv=dbg.get("kinematic_validation") if isinstance(dbg.get("kinematic_validation"), dict) else {},
                maneuver=maneuver,
                policy=nav_policy,
                mppi_meta=mppi_meta,
                physical=dbg.get("physical_trajectory")
                if isinstance(dbg.get("physical_trajectory"), dict)
                else (getattr(state, "_physical_corridor", {}) or {}),
                planned_path=planned,
                cmd_vx=cmd_vx,
                safe_vx=safe_vx,
                state_vx=state_vx,
                front_near=front_near,
                rear_near=rear_near,
                stop_reason=stop_reason,
                control_mode=control_mode,
                path_valid=bool(gpath) and len(gpath) >= 2,
            )
            dbg["open_space_forensics"] = forensic
            with state.lock:
                state._open_space_forensics = forensic
        except Exception:
            pass
        # P1-2-OBSERVE: lookahead / reference conflict forensics (telemetry only)
        try:
            from agv_bridge.nav_lookahead_forensics import assemble_lookahead_forensics, emit_lookahead_events

            rec = dbg.get("recovery") if isinstance(dbg.get("recovery"), dict) else {}
            rec_active = bool(rec.get("active"))
            lp_dict = dbg.get("local_plan") if isinstance(dbg.get("local_plan"), dict) else {}

            def _clr_la(px: float, py: float) -> float:
                return float(world.clearance_at_xy(px, py))

            def _col_la(px: float, py: float) -> bool:
                return bool(world.collides(px, py, robot_r=0.02, include_actors=True))

            lf = assemble_lookahead_forensics(
                x=x,
                y=y,
                yaw=yaw,
                global_path=gpath,
                global_reference=dbg.get("global_reference") if isinstance(dbg.get("global_reference"), dict) else {},
                local_plan=lp_dict,
                mppi_meta=mppi_meta,
                display_lookahead_pt=la,
                display_lookahead_m=1.4,
                cmd_vx=cmd_vx,
                cmd_w=cmd_w,
                safe_w=safe_w,
                state_w=state_w,
                pp_w=pp_w,
                front_near=front_near,
                left_near=float(getattr(state, "_left_near", 0.0) or 0.0),
                right_near=float(getattr(state, "_right_near", 0.0) or 0.0),
                path_revision=path_rev,
                clearance_at=_clr_la,
                collide=_col_la,
                maneuver=maneuver,
                maneuver_authority=dbg.get("maneuver_authority"),
                fsm_mode=str(phase or maneuver.get("mode") or ""),
                recovery_active=rec_active,
                now=now,
            )
            emit_lookahead_events(lf)
            dbg["lookahead_forensics"] = lf
            with state.lock:
                state._lookahead_forensics = lf
        except Exception:
            pass
        # P0-B-0: read-only observability ingest (must not affect control)
        try:
            from agv_bridge.nav_observability import OBS

            t_obs0 = time.perf_counter()
            obs_out = OBS.ingest_debug_snapshot(dbg)
            overhead = (time.perf_counter() - t_obs0) * 1000.0
            dbg["observability"] = {
                "cycle_id": obs_out.get("cycle_id"),
                "trace_id": obs_out.get("trace_id"),
                "likely_owner": obs_out.get("likely_owner"),
                "logging_overhead_ms": obs_out.get("logging_overhead_ms", round(overhead, 3)),
                "summary": OBS.summary(),
            }
            # Attach decision slice for future UI without duplicating full dump every time
            if isinstance(obs_out.get("decision"), dict):
                dbg["nav_decision_trace"] = {
                    "selection": obs_out["decision"].get("selection"),
                    "command": obs_out["decision"].get("command"),
                    "safety": obs_out["decision"].get("safety"),
                    "diagnostics": obs_out["decision"].get("diagnostics"),
                    "candidates": {
                        "count": (obs_out["decision"].get("candidates") or {}).get("count"),
                        "valid_count": (obs_out["decision"].get("candidates") or {}).get("valid_count"),
                        "max_distance_m": (obs_out["decision"].get("candidates") or {}).get("max_distance_m"),
                        "items": (obs_out["decision"].get("candidates") or {}).get("items"),
                    },
                    "fsm": obs_out["decision"].get("fsm"),
                    "recovery": obs_out["decision"].get("recovery"),
                    "performance": obs_out["decision"].get("performance"),
                }
        except Exception:
            pass
        with state.lock:
            state._debug_snapshot = dbg
            state._nav_session_id = debug_hub.session_id

    def get_nav_debug() -> Dict[str, Any]:
        with state.lock:
            snap = dict(state._debug_snapshot or {})
        if not snap:
            _refresh_debug_snapshot(time.time())
            with state.lock:
                snap = dict(state._debug_snapshot or {})
        return {"success": True, "debug": snap}

    def set_debug_level(level: str) -> Dict[str, Any]:
        lv = (level or "BASIC").strip().upper()
        if lv not in ("OFF", "BASIC", "ADVANCED", "FULL"):
            lv = "BASIC"
        debug_hub.debug_level = lv
        _refresh_debug_snapshot(time.time())
        return {"success": True, "level": lv}

    def set_debug_freeze(freeze: bool) -> Dict[str, Any]:
        debug_hub.freeze = bool(freeze)
        if debug_hub.freeze:
            with state.lock:
                debug_hub.frozen_snapshot = dict(state._debug_snapshot or {})
        else:
            debug_hub.frozen_snapshot = None
            _refresh_debug_snapshot(time.time())
        return {"success": True, "freeze": debug_hub.freeze}

    def capture_debug() -> Dict[str, Any]:
        _refresh_debug_snapshot(time.time())
        with state.lock:
            cap = dict(state._debug_snapshot or {})
        cap["telemetry"] = debug_hub.telemetry.list(TELEM_MAX)
        cap["pose_trace"] = list(debug_hub.pose_trace)
        cap["events"] = debug_hub.events.list(EVENT_MAX)
        cap["session_id"] = debug_hub.session_id
        cap["session_start"] = debug_hub.session_start
        cap["session_end"] = debug_hub.session_end
        cap["run_summary"] = debug_hub._run_summary
        cap["post_mortem"] = debug_hub._post_mortem
        cap["incident"] = debug_hub.incident.latest()
        cap["incidents"] = debug_hub.incident.incidents[-3:]
        cap["recovery_attempts"] = debug_hub.recovery_attempts_full[-10:]
        cap["reverse_decisions"] = debug_hub.reverse_decisions[-10:]
        cap["candidate_switch_log"] = debug_hub.candidate_switch_log[-40:]
        debug_hub.last_capture = cap
        return {"success": True, "capture": cap, "timestamp": time.time()}

    def get_nav_logs(query: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        q = dict(query or {})
        return OBS.query_events(
            level=q.get("level"),
            category=q.get("category"),
            event=q.get("event"),
            source=q.get("source"),
            component=q.get("component"),
            trace_id=q.get("trace_id"),
            cycle_id=q.get("cycle_id"),
            from_ts=float(q["from_ts"]) if q.get("from_ts") not in (None, "") else None,
            to_ts=float(q["to_ts"]) if q.get("to_ts") not in (None, "") else None,
            since=q.get("since"),
            focus=q.get("focus") or q.get("diagnostic_focus"),
            limit=int(q.get("limit") or 200),
            cursor=q.get("cursor"),
            include_api=str(q.get("include_api") or "").lower() in ("1", "true", "yes"),
        )

    def get_nav_logs_summary() -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        return OBS.summary()

    def get_nav_logs_trace(trace_id: str) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        return OBS.get_trace(str(trace_id))

    def get_nav_logs_cycle(cycle_id: str) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        return OBS.get_cycle(str(cycle_id))

    def get_nav_logs_diagnostics(window_s: float = 10.0) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        return OBS.diagnostics_window(window_s=float(window_s or 10.0))

    def get_nav_logs_events(query: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return get_nav_logs(query)

    def get_nav_api_logs(limit: int = 100) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        with OBS._lock:
            rows = list(OBS.api_events)[-max(1, min(int(limit or 100), 200)) :]
        return {"success": True, "events": rows, "count": len(rows), "separated": True}

    def configure_nav_logs(cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        from agv_bridge.nav_observability import OBS

        c = dict(cfg or {})
        return {"success": True, **OBS.configure(
            level=c.get("level"),
            candidate_detail=c.get("candidate_detail"),
            enabled=c.get("enabled"),
            sample_hz=c.get("sample_hz"),
        )}

    def get_nav_preview() -> Dict[str, Any]:
        """GET-only Global/Local preview + kinematic validation. Never a control API."""
        with state.lock:
            gref = dict(getattr(state, "_global_reference", {}) or {})
            loc = dict(getattr(state, "_local_candidates_layer", {}) or {})
            sel = dict(getattr(state, "_selected_local", {}) or {})
            kv = dict(getattr(state, "_kinematic_validation", {}) or {})
            fos = dict(getattr(state, "_open_space_forensics", {}) or {})
            lp = dict(getattr(state, "_local_plan", {}) or {})
            meta = dict(getattr(state, "_debug_mppi_meta", {}) or {})
            gvl = (state._debug_snapshot or {}).get("global_vs_local") if isinstance(state._debug_snapshot, dict) else {}
            dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
        if not gref:
            _refresh_debug_snapshot(time.time())
            with state.lock:
                gref = dict(getattr(state, "_global_reference", {}) or {})
                loc = dict(getattr(state, "_local_candidates_layer", {}) or {})
                sel = dict(getattr(state, "_selected_local", {}) or {})
                kv = dict(getattr(state, "_kinematic_validation", {}) or {})
                fos = dict(getattr(state, "_open_space_forensics", {}) or {})
                lp = dict(getattr(state, "_local_plan", {}) or {})
                meta = dict(getattr(state, "_debug_mppi_meta", {}) or {})
                gvl = (state._debug_snapshot or {}).get("global_vs_local") if isinstance(state._debug_snapshot, dict) else {}
                dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
        mppi_summary = {
            "horizon_s": meta.get("horizon_s"),
            "mean_vx": meta.get("mean_vx_after") or meta.get("mean_vx"),
            "vx_raw": meta.get("vx_raw"),
            "vx_cmd": meta.get("vx_cmd"),
            "target_vx": meta.get("target_vx"),
            "local_plan_id": meta.get("local_plan_id"),
            "tracking_local_plan": meta.get("tracking_local_plan"),
            "control_mode": meta.get("control_mode"),
        }
        return {
            "success": True,
            "global_reference": gref,
            "local_candidates": loc,
            "selected_local": sel,
            "local_plan": lp or {},
            "mppi_summary": mppi_summary,
            "speed_policy": (dbg.get("speed_policy") if isinstance(dbg, dict) else {}) or {},
            "global_vs_local": gvl or {},
            "kinematic_validation": kv or {},
            "open_space_forensics": fos or {},
            "geometry_version": "p0a",
            "generated_at": time.time(),
            "controls_vehicle": False,
        }

    def get_nav_local_plan() -> Dict[str, Any]:
        """GET /api/nav/local-plan — same LocalPlan fact source as preview."""
        prev = get_nav_preview()
        lp = prev.get("local_plan") or {}
        return {
            "success": True,
            "plan_id": lp.get("plan_id"),
            "revision": lp.get("revision"),
            "horizon_s": lp.get("horizon_s"),
            "horizon_m": lp.get("horizon_m"),
            "selected_candidate": lp.get("selected_candidate"),
            "candidates": lp.get("candidates") or [],
            "active": lp.get("active"),
            "status": lp.get("status"),
            "speed_target": lp.get("speed_target"),
            "min_clearance": lp.get("min_clearance"),
            "kinematic_valid": lp.get("kinematic_valid"),
            "local_plan": lp,
            "global_reference": prev.get("global_reference") or {},
            "mppi_summary": prev.get("mppi_summary") or {},
            "generated_at": time.time(),
            "controls_vehicle": False,
        }

    def get_open_space_forensics() -> Dict[str, Any]:
        """GET-only P0-C.1 open-space local planning forensics. Never a control API."""
        with state.lock:
            blob = dict(getattr(state, "_open_space_forensics", {}) or {})
        if not blob:
            _refresh_debug_snapshot(time.time())
            with state.lock:
                blob = dict(getattr(state, "_open_space_forensics", {}) or {})
        return {
            "success": True,
            "open_space_forensics": blob or {},
            "generated_at": time.time(),
            "controls_vehicle": False,
        }

    def get_lookahead_forensics() -> Dict[str, Any]:
        """GET /api/nav/forensics/lookahead — pink point + reference conflict diagnostics."""
        with state.lock:
            blob = dict(getattr(state, "_lookahead_forensics", {}) or {})
            dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
        if not blob:
            _refresh_debug_snapshot(time.time())
            with state.lock:
                blob = dict(getattr(state, "_lookahead_forensics", {}) or {})
                dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
        lf = blob or (dbg.get("lookahead_forensics") if isinstance(dbg.get("lookahead_forensics"), dict) else {})
        return {
            "success": True,
            "lookahead": lf.get("display_lookahead") or lf.get("lookahead") or {},
            "pp_lookahead": lf.get("pp_lookahead") or {},
            "reference": lf.get("reference") or {},
            "controller": lf.get("controller") or {},
            "authority": lf.get("authority") or {},
            "diagnostics": lf.get("diagnostics") or {},
            "three_headings": lf.get("three_headings") or {},
            "paths": lf.get("paths") or {},
            "events": lf.get("events") or [],
            "lookahead_forensics": lf or {},
            "generated_at": time.time(),
            "controls_vehicle": False,
        }

    def get_obstacle_preview() -> Dict[str, Any]:
        """GET /api/nav/obstacle-preview — P0-D forward future obstacle preview."""
        with state.lock:
            blob = dict(getattr(state, "_obstacle_preview", {}) or {})
            dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
            lp = dict(getattr(state, "_local_plan", {}) or {})
        if not blob:
            _refresh_debug_snapshot(time.time())
            with state.lock:
                blob = dict(getattr(state, "_obstacle_preview", {}) or {})
                dbg = state._debug_snapshot if isinstance(state._debug_snapshot, dict) else {}
                lp = dict(getattr(state, "_local_plan", {}) or {})
        op = blob or (dbg.get("obstacle_preview") if isinstance(dbg.get("obstacle_preview"), dict) else {})
        return {
            "success": True,
            "future_preview_m": op.get("future_preview_m") or op.get("preview_distance_m"),
            "first_collision_distance_m": op.get("first_collision_distance_m"),
            "first_warning_distance_m": op.get("first_warning_distance_m"),
            "required_avoidance_distance_m": op.get("required_avoidance_distance_m"),
            "detection_distance_m": op.get("detection_distance_m"),
            "maneuver_start_distance_m": op.get("maneuver_start_distance_m"),
            "hard_stop_distance_m": op.get("hard_stop_distance_m"),
            "left_valid": op.get("left_valid"),
            "right_valid": op.get("right_valid"),
            "forward_valid": op.get("forward_valid"),
            "left_clearance": op.get("left_clearance"),
            "right_clearance": op.get("right_clearance"),
            "forward_clearance": op.get("forward_clearance"),
            "obstacle_pass_state": op.get("obstacle_pass_state"),
            "avoidance_phase": op.get("avoidance_phase") or op.get("phase"),
            "probe_active": op.get("probe_active"),
            "probe_confidence": max(
                float(op.get("probe_confidence_left") or 0.0),
                float(op.get("probe_confidence_right") or 0.0),
            ),
            "committed_side": op.get("committed_side"),
            "global_reconnect_blocked": op.get("global_reconnect_blocked"),
            "dynamic_state": op.get("dynamic_state"),
            "resume_block_reason": op.get("resume_block_reason"),
            "d_detection_m": op.get("d_detection_m"),
            "d_probe_start_m": op.get("d_probe_start_m"),
            "d_commit_m": op.get("d_commit_m"),
            "speed_reason": op.get("speed_reason"),
            "lookahead_source": op.get("lookahead_source"),
            "lookahead_distance": op.get("lookahead_distance_m"),
            "local_plan_id": lp.get("plan_id") or op.get("local_plan_id"),
            "local_plan_authority": op.get("local_plan_authority") or dbg.get("maneuver_authority"),
            "obstacle_preview": op or {},
            "generated_at": time.time(),
            "controls_vehicle": False,
        }

    state.get_nav_debug = get_nav_debug
    state.set_debug_level = set_debug_level
    state.set_debug_freeze = set_debug_freeze
    state.capture_debug = capture_debug
    state.get_nav_logs = get_nav_logs
    state.get_nav_logs_summary = get_nav_logs_summary
    state.get_nav_logs_trace = get_nav_logs_trace
    state.get_nav_logs_cycle = get_nav_logs_cycle
    state.get_nav_logs_diagnostics = get_nav_logs_diagnostics
    state.get_nav_logs_events = get_nav_logs_events
    state.get_nav_api_logs = get_nav_api_logs
    state.configure_nav_logs = configure_nav_logs
    state.get_nav_preview = get_nav_preview
    state.get_nav_local_plan = get_nav_local_plan
    state.get_open_space_forensics = get_open_space_forensics
    state.get_lookahead_forensics = get_lookahead_forensics
    state.get_obstacle_preview = get_obstacle_preview
    state._refresh_debug_snapshot = _refresh_debug_snapshot
    def _physics_loop() -> None:
        dt = 0.05
        local_period = 0.20  # P1-1: planner 5Hz; physics remains 20Hz. Do not reuse legacy 0.35s.
        while True:
            time.sleep(dt)
            world.step_actors(dt)

            with state.lock:
                x, y, yaw = state.x, state.y, state.angle
                nav_mode = state._nav_mode
                gpath = list(state._global_path)
                path_i = state._path_i
                cmd_vx, cmd_w = state._cmd_vx, state._cmd_w
                cmd_stamp = state._cmd_stamp
                goal_xy = state._goal_xy
                emergency = state.emergency or state.soft_emc
                last_local = state._last_local_replan
                last_global = state._last_global_replan

            now = time.time()
            dual = world.dual_lidar(x, y, yaw, step_deg=4.0)
            front_near = float(dual["front_near"])
            rear_near = float(dual.get("rear_near", 30.0))
            colliding = world.collides(x, y, robot_r=geom.safety_radius)

            mppi_vx = cmd_vx
            mppi_w = cmd_w
            planned_band: List[Tuple[float, float]] = []
            stop_reason = STOP_NONE
            stuck_s = 0.0
            force_rev = False

            if nav_mode in ("tracking", "avoid") and goal_xy and gpath:
                # Pause stuck accumulation during align / turn / reverse (Maneuver owns recovery)
                man = getattr(local_mppi, "maneuver", None)
                pause_stuck = False
                if man and man.last_decision is not None:
                    pause_stuck = bool(man.last_decision.pause_stuck)
                elif local_mppi.phase in ("align", "turn_in_place", "reposition", "reverse_escape"):
                    pause_stuck = True
                accumulate = (not pause_stuck) and local_mppi.phase in ("forward", "recover")
                prog = progress.update(
                    now=now,
                    x=x,
                    y=y,
                    yaw=yaw,
                    path=gpath,
                    goal=goal_xy,
                    accumulate_stuck=accumulate,
                    progress_min_m=STUCK_PROGRESS_MIN_M,
                )
                path_i = prog.index
                stuck_s = progress.no_progress_s if accumulate else 0.0
                need_global = False
                force_alt = False

                # Legacy stuck→reverse path REMOVED as primary: ManeuverFSM decides.
                # Only nudge stuck_s into local step; FSM may choose ALIGN / REPOSITION / REVERSE.
                if (
                    accumulate
                    and stuck_s >= STUCK_REVERSE_TRIGGER_S
                    and local_mppi.phase == "forward"
                ):
                    open_and_moving = front_near > 1.5 and abs(cmd_vx) > 0.05
                    if open_and_moving and (now - last_global) >= 3.0:
                        need_global = True
                        force_alt = True
                        stop_reason = STOP_REPLAN
                        _log_decision(
                            f"stuck_replan:{local_mppi.recovery_attempts}",
                            f"STUCK→REPLAN no_progress={stuck_s:.1f}s path_s={progress.path_progress_s:.2f}",
                            level="warn",
                        )
                        progress.last_progress_time = now
                        progress.no_progress_s = 0.0
                        local_mppi.recovery_attempts = min(
                            MAX_RECOVERY_ATTEMPTS, local_mppi.recovery_attempts + 1
                        )
                        if local_mppi.recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                            local_mppi.planner_state = NAVIGATION_FAILED
                            local_mppi.phase = "safe_stop"
                            if man:
                                man._set_mode("SAFE_STOP", now, "recovery_exhausted")
                            stop_reason = STOP_NAV_FAILED
                    else:
                        # Do NOT force begin_reverse_escape here — let Maneuver decide on next step
                        _log_decision(
                            f"stuck_maneuver:{int(stuck_s)}",
                            f"STUCK→MANEUVER_DECIDE no_progress={stuck_s:.1f}s "
                            f"(prefer ALIGN/TURN over reverse)",
                            level="warn",
                        )

                # force_rev only when Maneuver already in reverse_escape
                if local_mppi.phase == "reverse_escape":
                    force_rev = True

                need_local = (now - last_local) >= local_period or last_local <= 0
                if stuck_s >= global_planner.STUCK_FORCE_ALT_S and (now - last_global) >= 8.0:
                    need_global = True
                    force_alt = True
                elif stuck_s >= global_planner.STUCK_SPLICE_S and (now - last_global) >= 6.0:
                    need_global = True
                # Maneuver REPLAN mode also requests global
                if man and man.mode == "REPLAN" and (now - last_global) >= 2.0:
                    need_global = True

                if need_global and local_mppi.phase in (
                    "forward",
                    "recover",
                    "align",
                    "turn_in_place",
                    "reposition",
                ):
                    gr = global_planner.replan(
                        world,
                        (x, y),
                        goal_xy,
                        gpath,
                        no_progress_s=stuck_s,
                        force=force_alt,
                    )
                    new_g = gr.get("path") or []
                    mode = str(gr.get("mode", ""))
                    if new_g and mode != "keep":
                        if force_alt and float(gr.get("overlap", 0.0)) > 0.85 and mode != "splice":
                            gr2 = world.replan_global_alternative(
                                (x, y),
                                gpath,
                                goal_xy,
                                ban_front_m=8.0,
                                ban_radius=1.0,
                                max_overlap=0.55,
                                prefer_splice=False,
                            )
                            if gr2.get("path"):
                                gr = gr2
                                new_g = gr2["path"]
                                mode = str(gr2.get("mode", mode))
                        gpath = new_g
                        progress.reset()
                        with state.lock:
                            state._global_path = new_g
                            _bump_global_revision()
                            state._raw_global_path = list(getattr(world, "_last_raw_path", []) or [])
                            state._planning_metrics = dict(getattr(world, "_last_plan_metrics", {}) or {})
                            state._global_replan_count = int(state._global_replan_count) + 1
                            state._last_global_replan = now
                        stop_reason = STOP_REPLAN
                        debug_hub.note_replan()
                        _log_decision(f"replan:{mode}", f"REPLAN {mode}", level="warn")
                    else:
                        with state.lock:
                            state._last_global_replan = now

                clr_here = 1.0
                try:
                    clr_here = float(world.clearance_at_xy(x, y))
                except Exception:
                    clr_here = max(0.05, min(front_near, rear_near) * 0.5)

                # Dynamic obstacle cue from scene actors near forward cone
                dyn_short, dyn_long = False, False
                try:
                    ax = 0.0
                    nearest_act = 99.0
                    for a in list(getattr(world, "actors", []) or []):
                        dx = float(a.get("x", 0)) - x
                        dy = float(a.get("y", 0)) - y
                        dist = math.hypot(dx, dy)
                        bearing = math.atan2(dy, dx) - yaw
                        while bearing > math.pi:
                            bearing -= 2 * math.pi
                        while bearing < -math.pi:
                            bearing += 2 * math.pi
                        if abs(bearing) < 0.7 and dist < 4.0:
                            nearest_act = min(nearest_act, dist)
                    if nearest_act < 3.5:
                        seen = float(getattr(state, "_dyn_front_since", 0.0) or 0.0)
                        if seen <= 0:
                            state._dyn_front_since = now
                            seen = now
                        age = now - seen
                        dyn_short = age < 4.0
                        dyn_long = age >= 6.0
                    else:
                        state._dyn_front_since = 0.0
                except Exception:
                    dyn_short, dyn_long = False, False

                if need_local and local_mppi.phase != "safe_stop":
                    res = _local_once(
                        x,
                        y,
                        yaw,
                        gpath,
                        goal_xy,
                        stuck_s,
                        front_near,
                        now,
                        force_rev,
                        rear_near=rear_near,
                        collision=colliding,
                        emergency=emergency,
                        path_progress=float(progress.path_progress_s),
                        lateral_error=float(getattr(prog, "lateral_m", 0.0) or 0.0),
                        actual_clearance=clr_here,
                        nav_active=True,
                        dynamic_short=dyn_short,
                        dynamic_long=dyn_long,
                        state_vx=float(getattr(state, "vx", 0.0) or 0.0),
                        safety_zero=bool(getattr(state, "_last_safety_zero", False)),
                        planned_rejected_by_safety=bool(getattr(state, "_last_safety_blocked", False)),
                    )
                    mppi_vx, mppi_w = res.vx, res.w
                    planned_band = list(res.best_path)
                    cmd_stamp = now
                    m = local_mppi.mppi
                    debug_hub.note_candidate(
                        getattr(m, "_last_selected_id", None),
                        float(getattr(m, "_last_best_cost", 0.0) or 0.0),
                        vx=float(mppi_vx),
                        w=float(mppi_w),
                    )
                    with state.lock:
                        state._planned_path = planned_band
                        state._path_candidates = res.candidates
                        state._best_confidence = res.confidence
                        state._track_mode = res.mode
                        state._ctrl_note = f"{res.control_mode}:{res.mode}"
                        state._control_mode = res.control_mode
                        state._path_i = path_i
                        state._last_local_replan = now
                        state._local_replan_count = int(state._local_replan_count) + 1
                        state._debug_pp_w = float(getattr(m, "_last_pp_w", 0.0) or 0.0)
                        state._debug_selected_candidate = getattr(m, "_last_selected_id", None)
                        state._debug_best_cost = float(getattr(m, "_last_best_cost", 0.0) or 0.0)
                        state._debug_first_collision = getattr(m, "_last_first_collision", None)
                        state._debug_mppi_meta = dict(getattr(m, "_last_meta", {}) or {})
                        if getattr(local_mppi, "maneuver", None):
                            state._maneuver = local_mppi.maneuver.to_telemetry()
                        if getattr(local_mppi, "policy", None):
                            tel = local_mppi.policy.to_telemetry()
                            # STEP 3D evidence-only attach (does not affect control)
                            if getattr(local_mppi, "last_probe", None) is not None:
                                tel["probe"] = local_mppi.last_probe.to_dict(now)
                                tel["probe_events"] = list(local_mppi.probe.events[-12:])
                            if getattr(local_mppi, "last_execution_corridor", None) is not None:
                                tel["execution_corridor"] = local_mppi.last_execution_corridor.to_dict()
                            if getattr(local_mppi, "last_avoidance_state", None) is not None:
                                tel["avoidance_phase"] = local_mppi.last_avoidance_state.to_dict()
                            # STEP 3F — physical corridor / breadcrumb / recovery
                            if getattr(local_mppi, "last_recovery", None) is not None:
                                tel["recovery"] = local_mppi.last_recovery.to_dict()
                                rex = getattr(local_mppi.maneuver, "_recovery_exec", None)
                                if isinstance(rex, dict):
                                    tel["recovery"]["execution"] = dict(rex)
                            if getattr(local_mppi, "last_physical_trajectory", None) is not None:
                                tel["physical_trajectory"] = local_mppi.last_physical_trajectory
                                state._physical_corridor = local_mppi.last_physical_trajectory
                            else:
                                state._physical_corridor = {}
                            if getattr(local_mppi, "breadcrumb", None) is not None:
                                tel["breadcrumb"] = local_mppi.breadcrumb.to_dict()
                            # attach side_switch from policy if present
                            if getattr(local_mppi.policy, "last_switch_decision", None) is not None:
                                tel["side_switch"] = local_mppi.policy.last_switch_decision.to_dict()
                                tok = local_mppi.policy.switch_token
                                tel["switch_token"] = tok.to_dict() if tok else None
                            state._nav_policy = tel
                elif now - cmd_stamp > 0.40 and local_mppi.phase != "safe_stop":
                    res = _local_once(
                        x,
                        y,
                        yaw,
                        gpath,
                        goal_xy,
                        stuck_s,
                        front_near,
                        now,
                        force_rev,
                        rear_near=rear_near,
                        collision=colliding,
                        emergency=emergency,
                        path_progress=float(progress.path_progress_s),
                        lateral_error=float(getattr(prog, "lateral_m", 0.0) or 0.0),
                        actual_clearance=clr_here,
                        nav_active=True,
                        dynamic_short=dyn_short,
                        dynamic_long=dyn_long,
                        state_vx=float(getattr(state, "vx", 0.0) or 0.0),
                        safety_zero=bool(getattr(state, "_last_safety_zero", False)),
                        planned_rejected_by_safety=bool(getattr(state, "_last_safety_blocked", False)),
                    )
                    mppi_vx, mppi_w = res.vx, res.w
                    planned_band = list(res.best_path)
                    cmd_stamp = now
                    m = local_mppi.mppi
                    debug_hub.note_candidate(
                        getattr(m, "_last_selected_id", None),
                        float(getattr(m, "_last_best_cost", 0.0) or 0.0),
                        vx=float(mppi_vx),
                        w=float(mppi_w),
                    )
                    with state.lock:
                        state._debug_pp_w = float(getattr(m, "_last_pp_w", 0.0) or 0.0)
                        state._debug_selected_candidate = getattr(m, "_last_selected_id", None)
                        state._debug_best_cost = float(getattr(m, "_last_best_cost", 0.0) or 0.0)
                        state._debug_first_collision = getattr(m, "_last_first_collision", None)
                        state._debug_mppi_meta = dict(getattr(m, "_last_meta", {}) or {})
                        if getattr(local_mppi, "maneuver", None):
                            state._maneuver = local_mppi.maneuver.to_telemetry()
                        if getattr(local_mppi, "policy", None):
                            tel = local_mppi.policy.to_telemetry()
                            if getattr(local_mppi, "last_probe", None) is not None:
                                tel["probe"] = local_mppi.last_probe.to_dict(now)
                                tel["probe_events"] = list(local_mppi.probe.events[-12:])
                            if getattr(local_mppi, "last_execution_corridor", None) is not None:
                                tel["execution_corridor"] = local_mppi.last_execution_corridor.to_dict()
                            if getattr(local_mppi, "last_avoidance_state", None) is not None:
                                tel["avoidance_phase"] = local_mppi.last_avoidance_state.to_dict()
                            if getattr(local_mppi, "last_recovery", None) is not None:
                                tel["recovery"] = local_mppi.last_recovery.to_dict()
                                rex = getattr(local_mppi.maneuver, "_recovery_exec", None)
                                if isinstance(rex, dict):
                                    tel["recovery"]["execution"] = dict(rex)
                            if getattr(local_mppi, "last_physical_trajectory", None) is not None:
                                tel["physical_trajectory"] = local_mppi.last_physical_trajectory
                                state._physical_corridor = local_mppi.last_physical_trajectory
                            else:
                                state._physical_corridor = {}
                            if getattr(local_mppi, "breadcrumb", None) is not None:
                                tel["breadcrumb"] = local_mppi.breadcrumb.to_dict()
                            if getattr(local_mppi.policy, "last_switch_decision", None) is not None:
                                tel["side_switch"] = local_mppi.policy.last_switch_decision.to_dict()
                                tok = local_mppi.policy.switch_token
                                tel["switch_token"] = tok.to_dict() if tok else None
                            state._nav_policy = tel
                else:
                    mppi_vx, mppi_w = cmd_vx, cmd_w
                    planned_band = list(getattr(state, "_planned_path", []) or [])

                # progress geometry for debug
                with state.lock:
                    state._path_lateral_m = float(getattr(prog, "lateral_m", 0.0) or 0.0)
                    state._path_heading_err = float(getattr(prog, "heading_err", 0.0) or 0.0)
                    if getattr(local_mppi, "last_maneuver", None):
                        lm = local_mppi.last_maneuver
                        state._path_heading_err = float(lm.heading_error)
                        # Command audit when mode changes
                        prev_m = getattr(state, "_maneuver_mode_prev", "")
                        if lm.mode != prev_m:
                            state._maneuver_mode_prev = lm.mode
                            state._cmd_source = (
                                f"Maneuver:{lm.mode}|Local:MPPI|reason={lm.reason}"
                            )
                            _log_decision(
                                f"maneuver:{lm.mode}",
                                f"MANEUVER {lm.mode} reason={lm.reason} "
                                f"herr={lm.heading_error:.2f} fwd={lm.forward.reason}",
                                level="info",
                            )

            elif nav_mode not in ("tracking", "avoid"):
                if now - cmd_stamp > state._cmd_timeout:
                    mppi_vx = mppi_w = 0.0

            # Safety
            cmd_before_vx, cmd_before_w = mppi_vx, mppi_w
            mppi_failure_reason = None
            fp_clearance_now = None
            predicted_min_clr = None
            try:
                meta = dict(getattr(local_mppi.mppi, "_last_meta", {}) or {})
                if str(meta.get("failure_reason") or "") == SAFE_VX_MPPI:
                    mppi_failure_reason = SAFE_VX_MPPI
                predicted_min_clr = meta.get("predicted_min_clearance_m")
            except Exception:
                mppi_failure_reason = None
            try:
                corr = getattr(local_mppi, "last_execution_corridor", None)
                if corr is not None:
                    fp_clearance_now = (corr.metadata or {}).get("current_footprint_clearance_m")
            except Exception:
                fp_clearance_now = None
            planner_state = str(getattr(local_mppi, "planner_state", "NORMAL") or "NORMAL")
            safe_vx, safe_w, safety_reason, safe_vx_reason = apply_safety(
                mppi_vx,
                mppi_w,
                front_near=front_near,
                rear_near=rear_near,
                footprint_clearance=fp_clearance_now,
                predicted_min_clearance=predicted_min_clr,
                mppi_failure_reason=mppi_failure_reason,
                colliding=colliding,
                emergency=emergency,
                phase=local_mppi.phase,
                planner_state=planner_state,
                state_vx=float(getattr(state, "vx", 0.0) or 0.0),
            )
            if safety_reason != STOP_NONE:
                stop_reason = safety_reason
            if planner_state == NAVIGATION_FAILED:
                safe_vx = safe_w = 0.0
                stop_reason = STOP_NAV_FAILED

            with state.lock:
                state._last_safety_zero = abs(safe_vx) < 1e-4 and abs(safe_w) < 1e-4
                state._last_safety_blocked = safety_reason != STOP_NONE and (
                    abs(cmd_before_vx) > 0.02 or abs(cmd_before_w) > 0.02
                ) and abs(safe_vx) < abs(cmd_before_vx) - 0.01

            # avoid 模式标签（兼容旧 UI）
            if stop_reason == STOP_FRONT:
                nav_mode = "avoid"
            elif nav_mode == "avoid" and front_near > geom.front_clear_m:
                nav_mode = "tracking"

            # executed band：安全门后
            rev = safe_vx < -0.05
            executed = kinematic_band(
                x,
                y,
                yaw,
                safe_vx,
                safe_w,
                reverse=rev,
                bumper_l=geom.bumper_l,
                steps=12,
                dt=0.1,
            )

            if stop_reason not in (STOP_NONE, STOP_REVERSE, STOP_STUCK, STOP_REPLAN):
                _log_decision(
                    f"stop:{stop_reason}",
                    f"{stop_reason} front={front_near:.2f} rear={rear_near:.2f} "
                    f"mppi_vx={mppi_vx:.2f} safe_vx={safe_vx:.2f}",
                    level="warn",
                )

            with state.lock:
                if now - state._cmd_stamp <= state._cmd_timeout and state._nav_mode not in (
                    "tracking",
                    "avoid",
                ):
                    # 手动 3055 覆盖
                    safe_vx, safe_w = state._cmd_vx, state._cmd_w
                state._path_i = path_i
                if nav_mode in ("tracking", "avoid"):
                    state._nav_mode = nav_mode
                state.block_reason = 1 if stop_reason == STOP_FRONT else 0
                state._stuck_since = progress.last_progress_time if progress.initialized else 0.0
                state._stuck_s = float(stuck_s)
                state._last_progress_dist = progress.goal_distance
                state._nav_phase = local_mppi.phase
                state._recovery_attempts = local_mppi.recovery_attempts
                state._mppi_vx, state._mppi_w = float(mppi_vx), float(mppi_w)
                state._cmd_vx_before_safety = float(cmd_before_vx)
                state._cmd_w_before_safety = float(cmd_before_w)
                state._cmd_vx_after_safety = float(safe_vx)
                state._cmd_w_after_safety = float(safe_w)
                state._safe_vx_reason = str(safe_vx_reason)
                state._planner_state = planner_state
                state._planner_failure_reason = str(
                    getattr(local_mppi, "planner_failure_reason", "") or ""
                )
                rp = getattr(local_mppi, "last_recovery_plan", None)
                rs = getattr(local_mppi, "recovery_planner", None)
                state._recovery_state = (
                    rp.action if rp is not None else (rs.state.current_action if rs else "NONE")
                )
                state._recovery_attempt = int(
                    rs.state.attempt_count if rs is not None else getattr(local_mppi, "recovery_attempts", 0)
                )
                state._predicted_min_clearance_m = predicted_min_clr
                state._footprint_clearance_m = fp_clearance_now
                state._nav_ui_severity = ui_severity(
                    planner_state=planner_state,
                    safe_vx_reason=str(safe_vx_reason),
                    stop_reason=str(stop_reason),
                )
                state._front_near = float(front_near)
                state._rear_near = float(rear_near)
                state._collision = bool(colliding)
                state._stop_reason = stop_reason
                state._path_progress_s = float(progress.path_progress_s)
                state._goal_distance = float(progress.goal_distance)
                # Always refresh maneuver telemetry for debug cards
                if getattr(local_mppi, "maneuver", None) is not None:
                    try:
                        state._maneuver = local_mppi.maneuver.to_telemetry()
                    except Exception:
                        state._maneuver = {"mode": getattr(local_mppi.maneuver, "mode", "IDLE")}
                state._path = executed  # UI 默认看执行带
                if planned_band:
                    state._planned_path = planned_band
                state._cmd_vx, state._cmd_w = safe_vx, safe_w
                state._cmd_stamp = cmd_stamp if nav_mode in ("tracking", "avoid") else state._cmd_stamp
                state.vx += max(-geom.acc_v * dt, min(geom.acc_v * dt, safe_vx - state.vx))
                state.w += max(-geom.acc_w * dt, min(geom.acc_w * dt, safe_w - state.w))
                state.r_vx, state.r_w = state.vx, state.w
                state.is_stop = abs(state.vx) < 1e-3 and abs(state.w) < 1e-3
                state.x += state.vx * math.cos(state.angle) * dt
                state.y += state.vx * math.sin(state.angle) * dt
                state.angle = (state.angle + state.w * dt + math.pi) % (2 * math.pi) - math.pi
                if state._goal_xy is not None:
                    gx, gy = state._goal_xy
                    if math.hypot(gx - state.x, gy - state.y) < 0.28:
                        state.x, state.y = gx, gy
                        state.vx = state.w = 0.0
                        state._cmd_vx = state._cmd_w = 0.0
                        state._path = []
                        state._planned_path = []
                        state._global_path = []
                        state._raw_global_path = []
                        _bump_global_revision()
                        state._planning_metrics = {}
                        state._path_candidates = []
                        state._best_confidence = 0.0
                        state._goal_xy = None
                        state._nav_mode = "arrived"
                        state.task_status = 4
                        state.unfinished_path = []
                        state._stop_reason = STOP_GOAL
                        debug_hub.events.push(
                            "ARRIVED",
                            {"stop_reason": STOP_GOAL, "session_id": debug_hub.session_id, "category": "STOP"},
                            min_interval_s=0.0,
                            category="STOP",
                        )
                        debug_hub.end_session("GOAL_REACHED")
                        _clear_progress_and_recovery()
                        world.emit("arrived", level="success")

            # Debug observability (~20Hz telem via ingest; never affects control)
            if now - float(getattr(state, "_last_debug_sample_t", 0.0) or 0.0) >= 0.05:
                state._last_debug_sample_t = now
                _refresh_debug_snapshot(now)

    if not state._physics_started:
        state._physics_started = True
        threading.Thread(target=_physics_loop, daemon=True, name="sim_physics").start()
        world.emit("idle", "就绪 · path-progress stuck + Safety Supervisor")
