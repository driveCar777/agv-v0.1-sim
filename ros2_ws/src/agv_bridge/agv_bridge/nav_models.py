"""导航三层封装（Web 仿真）。

Pipeline:
  NavigationPolicy (WHY) → ManeuverFSM (HOW TO MANEUVER) → MPPI (trajectory) → Safety
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Tuple

from agv_bridge.mppi_controller import DiffDriveMppi, MppiResult, get_control_mode
from agv_bridge.maneuver import (
    ALIGN,
    LOCAL_LEFT,
    LOCAL_RIGHT,
    ManeuverFSM,
    POST_TURN,
    REVERSE_ESCAPE,
    SAFE_STOP,
    TURN_IN_PLACE,
)
from agv_bridge.nav_geometry import (
    DEFAULT_GEOM,
    GLOBAL_ALT_S,
    GLOBAL_SPLICE_S,
    MAX_RECOVERY_ATTEMPTS,
    RECOVERY_COOLDOWN_S,
    REVERSE_MAX_S,
)
from agv_bridge.nav_policy import NavigationPolicy
from agv_bridge.nav_probe import ProbeEngine, build_obstacle_snapshot
from agv_bridge.nav_breadcrumb import TrajectoryBreadcrumb
from agv_bridge.nav_local_planner import (
    AUTH_AVOIDANCE,
    AUTH_FSM,
    AUTH_RECOVERY,
    AUTH_ROLLING,
    LocalPlanRequest,
    RollingLocalPlanner,
)
from agv_bridge.nav_speed_policy import SpeedPolicy
from agv_bridge.nav_global_preview import GlobalPreviewCache
from agv_bridge.nav_obstacle_preview import (
    compute_future_preview,
    diagnose_late_avoidance,
    emit_obstacle_preview_events,
)
from agv_bridge.nav_side_probe import run_side_probe
from agv_bridge.nav_avoidance_phase import AvoidancePhaseTracker
from agv_bridge.nav_dynamic_resume import DynamicResumeTracker
from agv_bridge.nav_recovery import (
    ACT_CONTINUE,
    ACT_HISTORICAL_RETREAT,
    ACT_LOCAL_REVERSE,
    ACT_REPLAN,
    ACT_SAFE_STOP,
    ACT_SIDE_SWITCH,
    ACT_WAIT,
    evaluate_recovery,
)
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]


class GlobalPlannerModel:
    STUCK_SPLICE_S = GLOBAL_SPLICE_S
    STUCK_FORCE_ALT_S = GLOBAL_ALT_S

    def __init__(self) -> None:
        self.last_metrics: Dict[str, Any] = {}
        self.last_raw_path: List[Pt] = []

    def plan(self, world, start: Pt, goal: Pt) -> List[Pt]:
        result = world.plan_path_quality(
            start,
            goal,
            robot_r=DEFAULT_GEOM.planner_radius,
            avoid_dynamic=False,
            use_clearance=True,
            use_turn=True,
            use_los=True,
            use_smooth=True,
            variant="D",
        )
        self.last_raw_path = list(result.raw_path)
        self.last_metrics = result.metrics.to_dict() if result.path else {}
        return result.path

    def replan(
        self,
        world,
        start: Pt,
        goal: Pt,
        old_path: List[Pt],
        *,
        no_progress_s: float,
        force: bool = False,
    ) -> Dict[str, Any]:
        if force or no_progress_s >= self.STUCK_FORCE_ALT_S:
            r = world.replan_global_alternative(
                start, old_path, goal, prefer_splice=False, ban_front_m=6.0, max_overlap=0.65
            )
            r["trigger"] = "force_alt" if force else "stuck_alt"
            return r
        if no_progress_s >= self.STUCK_SPLICE_S:
            r = world.replan_global_repair(start, old_path, goal)
            r["trigger"] = "stuck_splice"
            return r
        return {"path": list(old_path), "mode": "keep", "kept_tail": True, "trigger": "keep"}


class LocalMppiModel:
    """Policy → Maneuver → Local trajectory. Safety remains final gate in sim_api_ext."""

    def __init__(self) -> None:
        self.geom = DEFAULT_GEOM
        self.mppi = DiffDriveMppi(geom=self.geom)
        self.maneuver = ManeuverFSM()
        self.policy = NavigationPolicy()
        self.phase = "forward"
        self._rev_cooldown_until = 0.0
        self._reverse_until = 0.0
        self.recovery_attempts = 0
        self.control_mode = get_control_mode()
        self.last_maneuver = None
        self.last_policy = None
        self.probe = ProbeEngine()
        self.last_probe = None
        self.last_obstacle_snapshot = None
        self._path_version = 0
        self.breadcrumb = TrajectoryBreadcrumb()
        self.last_recovery = None
        self.last_physical_trajectory = None  # active corridor dict for UI
        self._prev_progress_s = 0.0
        self._prev_progress_ts = 0.0
        self.last_path_valid = True
        self.speed_policy = SpeedPolicy(geom=self.geom)
        self.local_planner = RollingLocalPlanner(geom=self.geom)
        self.last_speed = None
        self.last_local_plan = None
        self.last_local_plan_result = None
        self.last_maneuver_authority = AUTH_ROLLING
        self.last_future_preview = None
        self._preview_cache = GlobalPreviewCache()
        self.last_side_probe = None
        self.avoidance_phase = AvoidancePhaseTracker()
        self.dynamic_resume = DynamicResumeTracker()
        self.last_avoidance_state = None
        self.last_dynamic_state = None

    def set_control_mode(self, mode: str) -> str:
        m = (mode or "mppi").strip().lower()
        self.control_mode = "pp_only" if m in ("pp", "pp_only", "pure_pursuit") else "mppi"
        os.environ["AGV_LOCAL_CONTROL"] = self.control_mode
        return self.control_mode

    def reset(self) -> None:
        self.mppi.reset()
        self.maneuver.reset()
        self.policy.reset()
        self.probe.reset()
        self.last_probe = None
        self.last_obstacle_snapshot = None
        self._path_version += 1
        self.phase = "forward"
        self._rev_cooldown_until = 0.0
        self._reverse_until = 0.0
        self.recovery_attempts = 0
        self.last_maneuver = None
        self.last_policy = None
        self._prev_progress_s = 0.0
        self._prev_progress_ts = 0.0
        self.breadcrumb.reset()
        self.last_recovery = None
        self.last_physical_trajectory = None
        self.speed_policy.reset()
        self.local_planner.reset()
        self.last_speed = None
        self.last_local_plan = None
        self.last_local_plan_result = None
        self.last_maneuver_authority = AUTH_ROLLING
        self.last_future_preview = None
        self._preview_cache = GlobalPreviewCache()
        self.last_side_probe = None
        self.avoidance_phase = AvoidancePhaseTracker()
        self.dynamic_resume = DynamicResumeTracker()
        self.last_avoidance_state = None
        self.last_dynamic_state = None

    def begin_reverse_escape(self, now: float) -> bool:
        """Legacy hook — prefer ManeuverFSM + Policy.allow_recovery."""
        if self.phase == "safe_stop":
            return False
        if now < self._rev_cooldown_until:
            return False
        if self.recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
            self.phase = "safe_stop"
            self.maneuver._set_mode(SAFE_STOP, now, "recovery_exhausted")
            return False
        self.recovery_attempts += 1
        self.phase = "reverse_escape"
        self._reverse_until = now + REVERSE_MAX_S
        self.policy.note_recovery(now)
        return True

    def tick_phase(self, now: float, cmd_vx: float) -> None:
        if self.phase == "reverse_escape" and self.maneuver.mode != REVERSE_ESCAPE:
            if now >= self._reverse_until or cmd_vx > -0.02:
                self.phase = "recover"
                self._rev_cooldown_until = now + RECOVERY_COOLDOWN_S
        elif self.phase == "recover" and now >= self._rev_cooldown_until:
            if self.maneuver.mode not in (ALIGN, TURN_IN_PLACE, REVERSE_ESCAPE, SAFE_STOP):
                self.phase = "forward"

    def step(
        self,
        world,
        x: float,
        y: float,
        yaw: float,
        global_path: List[Pt],
        goal: Pt,
        front_near: float,
        stuck_s: float,
        now: float,
        force_reverse: bool = False,
        *,
        rear_near: float = 30.0,
        collision: bool = False,
        emergency: bool = False,
        path_progress: float = 0.0,
        lateral_error: float = 0.0,
        actual_clearance: float = 1.0,
        candidates_hint: Optional[List[Dict[str, Any]]] = None,
        nav_active: bool = True,
        dynamic_short: bool = False,
        dynamic_long: bool = False,
        arrived: bool = False,
        left_free: Optional[float] = None,
        right_free: Optional[float] = None,
        state_vx: float = 0.0,
        safety_zero: bool = False,
        planned_rejected_by_safety: bool = False,
    ) -> MppiResult:
        def collide(px: float, py: float) -> bool:
            return world.collides(px, py, robot_r=self.geom.local_radius, include_actors=True)

        def clearance_at(px: float, py: float) -> float:
            try:
                return float(world.clearance_at_xy(px, py))
            except Exception:
                return 1.0

        # Side free estimates for policy scene classification
        if left_free is None or right_free is None:
            try:
                from agv_bridge.maneuver import sector_free

                lf = sector_free(x, y, yaw, base_ang=math.pi / 2, collide=collide)
                rf = sector_free(x, y, yaw, base_ang=-math.pi / 2, collide=collide)
            except Exception:
                lf, rf = 2.0, 2.0
            left_free = lf if left_free is None else left_free
            right_free = rf if right_free is None else right_free

        # Path progress rate
        dt_p = max(1e-3, now - self._prev_progress_ts) if self._prev_progress_ts else 0.2
        progress_rate = (path_progress - self._prev_progress_s) / dt_p if self._prev_progress_ts else 0.0
        self._prev_progress_s = path_progress
        self._prev_progress_ts = now

        path_valid = bool(global_path) and len(global_path) >= 2
        self.last_path_valid = path_valid
        proj = project_pose_to_path(x, y, yaw, global_path) if path_valid else None
        heading_error = float(proj.heading_err) if proj else 0.0
        lat = float(proj.lateral_m) if proj else float(lateral_error)
        goal_herr = 0.0
        if goal:
            goal_herr = math.atan2(goal[1] - y, goal[0] - x) - yaw

        forward_feasible = front_near >= DEFAULT_GEOM.front_stop_m and abs(heading_error) < 1.15

        try:
            from agv_bridge.maneuver import rotation_safe_at

            rot_safe = rotation_safe_at(x, y, clearance=actual_clearance, collide=collide)
        except Exception:
            rot_safe = True

        # STEP 3D — same-tick ObstacleSnapshot + ProbeBundle (evidence only)
        try:
            snap = build_obstacle_snapshot(
                now=now,
                x=x,
                y=y,
                yaw=yaw,
                vx=float(state_vx),
                w=0.0,
                front_near=front_near,
                rear_near=rear_near,
                left_free=float(left_free),
                right_free=float(right_free),
                collide=collide,
                clearance_at=clearance_at,
                global_path=global_path if path_valid else None,
                goal=goal,
                corridor_half_width=(
                    float(self.last_policy.max_deviation_m) if self.last_policy else None
                ),
                pose_ts=now,
                obstacle_ts=now,
                dynamic_short=dynamic_short,
                mode=str(self.maneuver.mode or ""),
                path_version=self._path_version,
            )
            self.last_obstacle_snapshot = snap
            self.last_probe = self.probe.evaluate(snap, now=now)
        except Exception:
            self.last_probe = None

        ext_passed = bool(getattr(self.maneuver.local_selector, "obstacle_passed", False))
        gprev_est = 5.0
        if path_valid and global_path:
            try:
                rem = 0.0
                for i in range(1, len(global_path)):
                    rem += math.hypot(
                        global_path[i][0] - global_path[i - 1][0],
                        global_path[i][1] - global_path[i - 1][1],
                    )
                gprev_est = min(5.0, max(0.4, rem))
            except Exception:
                gprev_est = 5.0
        goal_d_preview = math.hypot(goal[0] - x, goal[1] - y) if goal else None
        try:
            self.last_future_preview = compute_future_preview(
                x=x,
                y=y,
                yaw=yaw,
                vx=float(state_vx),
                global_path=global_path if path_valid else None,
                path_progress_s=float(path_progress) if path_progress else None,
                path_revision=int(self._path_version),
                global_preview_m=gprev_est,
                goal_distance_m=goal_d_preview,
                collide=collide,
                clearance_at=clearance_at,
                front_near=float(front_near),
                left_free=float(left_free),
                right_free=float(right_free),
                lateral_error=float(lat),
                geom=self.geom,
                cache=self._preview_cache,
                external_obstacle_passed=ext_passed,
                now=now,
            )
        except Exception:
            self.last_future_preview = None
        fp = self.last_future_preview

        side_probe_active = bool(
            fp is not None and fp.future_collision and fp.first_collision_distance_m is not None
        )
        try:
            self.last_side_probe = run_side_probe(
                x=x,
                y=y,
                yaw=yaw,
                vx=float(state_vx),
                horizon_m=float(fp.preview_distance_m) if fp is not None else 5.0,
                collide=collide,
                clearance_at=clearance_at,
                global_path=global_path if path_valid else None,
                geom=self.geom,
                probe_active=side_probe_active,
                side_history=list(self.avoidance_phase.state.side_history),
            )
        except Exception:
            self.last_side_probe = None
        sp = self.last_side_probe

        committed_side = None
        try:
            if self.policy.commitment.active:
                committed_side = getattr(self.policy.commitment, "side", None) or getattr(
                    self.policy.commitment, "committed_side", None
                )
        except Exception:
            committed_side = None

        try:
            self.last_avoidance_state = self.avoidance_phase.update(
                now=now,
                vx=float(state_vx),
                preview_m=float(fp.preview_distance_m) if fp else 5.0,
                future_collision=bool(fp.future_collision) if fp else False,
                first_collision_m=fp.first_collision_distance_m if fp else None,
                front_near=float(front_near),
                left_free=float(left_free),
                right_free=float(right_free),
                lateral_error=float(lat),
                side_probe_active=bool(sp.probe_active) if sp else False,
                probe_confidence_left=float(sp.left_confidence) if sp else 0.0,
                probe_confidence_right=float(sp.right_confidence) if sp else 0.0,
                preferred_side=sp.preferred_side if sp else None,
                commit_ready=bool(sp.commit_ready) if sp else False,
                committed_side_external=committed_side,
                obstacle_passed_external=ext_passed,
                commitment_active=bool(self.policy.commitment.active),
                geom=self.geom,
            )
        except Exception:
            self.last_avoidance_state = None
        av = self.last_avoidance_state

        dyn_resume_clear = False
        try:
            lp_valid = self.last_local_plan is not None and getattr(self.last_local_plan, "status", "") in (
                "CREATED", "ACTIVE", "REPLACED", "FALLBACK",
            )
            self.last_dynamic_state = self.dynamic_resume.update(
                now=now,
                dynamic_short=bool(dynamic_short),
                dynamic_long=bool(dynamic_long),
                policy_state=str(getattr(self.last_policy, "state", "") or ""),
                policy_reason=str(getattr(self.last_policy, "reason", "") or ""),
                maneuver_mode=str(self.maneuver.mode or ""),
                front_near=float(front_near),
                forward_feasible=forward_feasible,
                safety_zero=bool(safety_zero),
                local_plan_valid=lp_valid,
                recovery_loop=self.policy.recovery_loop(now),
                replan_loop=self.policy.replan_loop(now),
                oscillation_loop=self.policy.oscillation_loop(now),
            )
            dyn_resume_clear = bool(self.last_dynamic_state.resume_allowed)
        except Exception:
            self.last_dynamic_state = None

        pol = self.policy.step(
            now=now,
            nav_active=nav_active,
            arrived=arrived,
            emergency=emergency,
            sensor_invalid=False,
            map_invalid=False,
            path_valid=path_valid,
            x=x,
            y=y,
            yaw=yaw,
            front_near=front_near,
            rear_near=rear_near,
            left_free=float(left_free),
            right_free=float(right_free),
            lateral_error=lat,
            heading_error=heading_error,
            path_progress_rate=progress_rate,
            state_vx=state_vx,
            collision=collision,
            forward_feasible=forward_feasible,
            rotation_safe=rot_safe,
            dynamic_short=dynamic_short,
            dynamic_long=dynamic_long,
            maneuver_mode=self.maneuver.mode,
            local_decision=self.maneuver.decision_label,
            recovery_attempts=self.recovery_attempts,
            stuck_s=stuck_s,
            goal_herr=goal_herr,
            planned_rejected_by_safety=planned_rejected_by_safety,
            safety_zero=safety_zero,
            future_collision=bool(fp.future_collision) if fp is not None else False,
            approach_active=bool(fp.approach_active) if fp is not None else False,
            first_collision_distance_m=fp.first_collision_distance_m if fp is not None else None,
            required_avoidance_distance_m=fp.required_avoidance_distance_m if fp is not None else None,
            avoidance_phase=str(av.phase) if av is not None else "OPEN",
            readiness_signal=str(av.signal) if av is not None else "NONE",
            side_probe_active=bool(sp.probe_active) if sp else False,
            commit_ready=bool(sp.commit_ready) if sp else False,
            dynamic_resume_clear=dyn_resume_clear,
            d_probe_start_m=(av.tiers.get("d_probe_start_m") if av else None),
        )
        self.last_policy = pol

        if self.maneuver.local_selector.obstacle_passed:
            self.policy.mark_obstacle_passed()

        # STEP 3E — authorize BEFORE decide (consume ProbeBundle only; O(1) gates)
        raw_cand = None
        try:
            lr0 = self.maneuver.local_selector.last_result
            if lr0 is not None and isinstance(getattr(lr0, "snapshot", None), dict):
                raw_cand = lr0.snapshot.get("raw_preferred_side")
            if not raw_cand:
                raw_cand = self.maneuver.local_selector.current
        except Exception:
            raw_cand = None
        sw_dec = self.policy.authorize_side_switch_gate(
            now=now,
            probe_bundle=self.last_probe,
            raw_candidate_side=raw_cand,
            dynamic_short=dynamic_short,
            emergency=emergency,
            safety_zero=safety_zero,
            planned_rejected_by_safety=planned_rejected_by_safety,
            path_valid=path_valid,
        )
        live_auth = bool(
            self.policy.switch_token is not None and self.policy.switch_token.is_live(now)
        )
        auth_side = (
            self.policy.switch_token.to_side
            if live_auth and self.policy.switch_token is not None
            else None
        )

        # STEP 3F — Recovery ladder (event-ish; consumes Probe + breadcrumb; O(segments) only when dead)
        progress_low = bool(stuck_s >= 3.0 or (progress_rate is not None and progress_rate < 0.02))
        try:
            rec = evaluate_recovery(
                probe=self.last_probe,
                breadcrumb=self.breadcrumb,
                collide=collide,
                clearance_at=clearance_at,
                side_switch_authorized=live_auth,
                safety_zero=safety_zero,
                planned_rejected_by_safety=planned_rejected_by_safety,
                stuck_s=stuck_s,
                progress_low=progress_low,
                recovery_attempts=self.recovery_attempts,
                dynamic_short=dynamic_short,
                allow_replan=bool(getattr(pol, "allow_replan", True)),
                emergency=emergency,
            )
            self.last_recovery = rec
            if rec.release_commitment and self.policy.commitment.active:
                self.policy.release_commitment(now=now, reason=f"RECOVERY|{rec.action}")
            if rec.allow_recovery:
                # Mirror onto decision flags for ManeuverFSM
                if self.policy.last_decision is not None:
                    self.policy.last_decision.allow_recovery = True
                    pol = self.policy.last_decision
            if rec.action == ACT_REPLAN:
                self.policy.note_replan(now)
        except Exception:
            self.last_recovery = None

        # Active physical trajectory = prefer authorized/active maneuver side corridor from Probe
        try:
            self.last_physical_trajectory = self._select_active_corridor(
                live_auth=live_auth, auth_side=auth_side, recovery=self.last_recovery
            )
        except Exception:
            self.last_physical_trajectory = None

        # Refresh decision flags after authorize
        if self.policy.last_decision is not None:
            pol = self.policy.last_decision

        prev_mode = self.maneuver.mode
        policy_ctx = {
            "state": pol.state,
            "behavior": pol.behavior,
            "allow_side_compare": pol.allow_side_compare,
            "allow_replan": pol.allow_replan,
            "allow_recovery": bool(pol.allow_recovery)
            or bool(self.last_recovery and self.last_recovery.allow_recovery),
            "require_capture_hard": pol.require_capture_hard,
            "path_follow_weight": pol.path_follow_weight,
            "max_deviation_m": pol.max_deviation_m,
            "scene": pol.scene,
            "commitment_active": pol.commitment_active,
            "committed_side": pol.committed_side,
            "commitment_phase": pol.commitment_phase,
            "commitment_hard_fail": pol.commitment_hard_fail,
            "commitment_failure_reason": pol.commitment_failure_reason,
            "side_switch_authorized": live_auth,
            "side_switch_authorization_status": pol.side_switch_authorization_status,
            "authorized_side": auth_side,
            "recovery_action": (self.last_recovery.action if self.last_recovery else "NONE"),
            "recovery_class": (self.last_recovery.classification if self.last_recovery else "NORMAL"),
            "recovery_target_distance_m": self._recovery_target_distance_m(),
            "recovery_force_vx": -0.12,
            "recovery_force_w": 0.0,
            "dynamic_resume_clear": dyn_resume_clear,
            "avoidance_phase": str(av.phase) if av is not None else "OPEN",
        }

        decision = self.maneuver.decide(
            now=now,
            x=x,
            y=y,
            yaw=yaw,
            path=global_path,
            goal=goal,
            front_near=front_near,
            rear_near=rear_near,
            collision=collision,
            emergency=emergency,
            path_progress=path_progress,
            lateral_error=lat,
            stuck_s=stuck_s,
            recovery_attempts=self.recovery_attempts,
            candidates=candidates_hint,
            clearance_at=clearance_at,
            collide=collide,
            actual_clearance=actual_clearance,
            nav_active=nav_active,
            policy_ctx=policy_ctx,
            dynamic_short=dynamic_short,
        )
        self.last_maneuver = decision
        self.phase = decision.legacy_phase

        # Consume authorization only when FSM actually switched sides
        try:
            def _side_of_mode(m: str) -> str:
                mu = (m or "").upper()
                if "LEFT" in mu:
                    return "LEFT"
                if "RIGHT" in mu:
                    return "RIGHT"
                return "NONE"

            prev_side = _side_of_mode(prev_mode)
            new_side = _side_of_mode(decision.mode)
            if (
                live_auth
                and prev_side in ("LEFT", "RIGHT")
                and new_side in ("LEFT", "RIGHT")
                and prev_side != new_side
            ):
                self.policy.note_side_switch_executed(
                    now=now,
                    from_side=prev_side,
                    to_side=new_side,
                    maneuver_mode=decision.mode,
                )
            elif live_auth and auth_side and new_side != auth_side:
                # Only flag hard failure if selector claimed authorized switch but mode stayed
                lr = self.maneuver.local_selector.last_result
                reason = getattr(lr, "reason", "") if lr is not None else ""
                if str(reason) == "POLICY_AUTHORIZED_SWITCH" and prev_side == new_side:
                    self.policy._emit(
                        "SIDE_SWITCH_EXECUTION_FAILED",
                        from_side=prev_side,
                        to_side=auth_side,
                        reason="FSM_DID_NOT_TRANSITION",
                        mode=decision.mode,
                        source="ManeuverFSM",
                    )
        except Exception:
            pass

        # STEP 3C — post-selector commitment lifecycle (does not write vx/w)
        try:
            lr = self.maneuver.local_selector.last_result
            left_feas = right_feas = None
            left_cost = right_cost = None
            if lr is not None and getattr(lr, "candidates", None):
                lc = lr.candidates.get("LEFT")
                rc = lr.candidates.get("RIGHT")
                if lc is not None:
                    left_feas = bool(lc.feasible)
                    left_cost = float(lc.total_cost)
                if rc is not None:
                    right_feas = bool(rc.feasible)
                    right_cost = float(rc.total_cost)
            sel = (
                self.maneuver.local_selector.current
                or getattr(decision, "decision_label", None)
                or self.maneuver.decision_label
            )
            self.policy.update_commitment_after_selection(
                now=now,
                selected=sel,
                left_feasible=left_feas,
                right_feasible=right_feas,
                left_cost=left_cost,
                right_cost=right_cost,
                obstacle_passed=bool(self.maneuver.local_selector.obstacle_passed)
                or bool(self.policy.obstacle_passed),
                policy_state=pol.state,
                maneuver_mode=decision.mode,
                front_near=front_near,
                left_free=float(left_free),
                right_free=float(right_free),
                scene=pol.scene,
                pose=(x, y, yaw),
                emergency=emergency,
                nav_active=nav_active,
            )
        except Exception:
            pass

        if decision.mode == REVERSE_ESCAPE and force_reverse is False:
            if self.maneuver.time_in_mode(now) < 0.08:
                if self.recovery_attempts < MAX_RECOVERY_ATTEMPTS and now >= self._rev_cooldown_until:
                    self.recovery_attempts = min(MAX_RECOVERY_ATTEMPTS, self.recovery_attempts + 1)
                    self._reverse_until = now + REVERSE_MAX_S
                    self.policy.note_recovery(now)

        if decision.mode == REVERSE_ESCAPE:
            self.phase = "reverse_escape"
        elif decision.mode == SAFE_STOP:
            self.phase = "safe_stop"

        want_rev = decision.mode == REVERSE_ESCAPE
        if force_reverse and decision.mode not in (ALIGN, TURN_IN_PLACE, POST_TURN, SAFE_STOP):
            want_rev = True
        if decision.mode in (ALIGN, TURN_IN_PLACE, POST_TURN):
            want_rev = False
        # Policy must authorize recovery reverse
        if want_rev and not pol.allow_recovery and decision.mode == REVERSE_ESCAPE:
            want_rev = False
        # STEP 3F — never reverse without BACKWARD Probe VALID (or explicit recovery action)
        rec = self.last_recovery
        if want_rev:
            back_ok = bool(
                self.last_probe is not None and self.last_probe.backward.status == "VALID"
            )
            rec_wants = bool(
                rec is not None and rec.action in (ACT_LOCAL_REVERSE, ACT_HISTORICAL_RETREAT)
            )
            if not (back_ok and (rec_wants or pol.allow_recovery)):
                if rec is not None and rec.action in (ACT_SIDE_SWITCH, ACT_WAIT, ACT_CONTINUE):
                    want_rev = False
                elif not back_ok:
                    want_rev = False
            if rec is not None and rec.action == ACT_SAFE_STOP:
                want_rev = False

        # Prefer reverse when recovery ladder selected local reverse / retreat
        # STEP 3F-CORRECTIVE: FSM should already be REVERSE_ESCAPE; still force want_rev
        if (
            rec is not None
            and rec.action in (ACT_LOCAL_REVERSE, ACT_HISTORICAL_RETREAT)
            and rec.allow_recovery
            and self.last_probe is not None
            and self.last_probe.backward.status == "VALID"
            and decision.mode == REVERSE_ESCAPE
        ):
            want_rev = True

        # Safety reject telemetry: reverse requested but rear blocked for safety
        if (
            want_rev
            and decision.mode == REVERSE_ESCAPE
            and rear_near < self.geom.rear_stop_m
        ):
            try:
                if decision.recovery_exec is not None:
                    decision.recovery_exec = dict(decision.recovery_exec)
                    decision.recovery_exec["status"] = "RECOVERY_SAFETY_REJECT"
                    decision.recovery_exec["safety_note"] = "rear_near_blocks_reverse"
            except Exception:
                pass

        # P1-1: SpeedPolicy + RollingLocalPlanner (recommendation only; FSM owns mode)
        rec = self.last_recovery
        rec_act = str(getattr(rec, "action", "") or "")
        mm_pre = str(decision.mode or "")
        if want_rev or mm_pre == REVERSE_ESCAPE or rec_act in (ACT_LOCAL_REVERSE, ACT_HISTORICAL_RETREAT):
            authority = AUTH_RECOVERY
        elif mm_pre in (LOCAL_LEFT, LOCAL_RIGHT):
            authority = AUTH_AVOIDANCE
        elif mm_pre in (ALIGN, TURN_IN_PLACE, SAFE_STOP, POST_TURN):
            authority = AUTH_FSM
        else:
            authority = AUTH_ROLLING
        self.last_maneuver_authority = authority

        gprev = 5.0
        if path_valid and global_path:
            try:
                rem = 0.0
                for i in range(1, len(global_path)):
                    rem += math.hypot(
                        global_path[i][0] - global_path[i - 1][0],
                        global_path[i][1] - global_path[i - 1][1],
                    )
                gprev = min(5.0, max(0.4, rem))
            except Exception:
                gprev = 5.0
        goal_d = math.hypot(goal[0] - x, goal[1] - y) if goal else None
        spd = self.speed_policy.compute(
            scene=str(getattr(pol, "scene", None) or "OPEN"),
            policy_state=str(getattr(pol, "state", None) or ""),
            vx_scale=float(getattr(pol.profile, "vx_scale", 1.0) or 1.0),
            state_vx=float(state_vx),
            front_near=float(front_near),
            rear_near=float(rear_near),
            left_free=float(left_free or 2.0),
            right_free=float(right_free or 2.0),
            min_clearance=float(actual_clearance) if actual_clearance is not None else None,
            goal_distance_m=goal_d,
            heading_error=float(heading_error),
            recovery_active=authority == AUTH_RECOVERY,
            force_reverse=bool(want_rev),
            avoidance_phase=str(av.phase) if av is not None else "OPEN",
            probe_active=bool(sp.probe_active) if sp else False,
            commit_ready=bool(sp.commit_ready) if sp else False,
            dynamic_resume_vx=(
                self.last_dynamic_state.resume_target_vx
                if self.last_dynamic_state is not None and self.last_dynamic_state.resume_allowed
                else None
            ),
        )
        self.last_speed = spd

        try:
            lp_res = self.local_planner.update(
                LocalPlanRequest(
                    x=x,
                    y=y,
                    yaw=yaw,
                    vx=float(state_vx),
                    global_path=global_path if path_valid else None,
                    global_preview_m=gprev,
                    goal=goal,
                    collide=collide,
                    clearance_at=clearance_at,
                    front_near=float(front_near),
                    left_free=float(left_free or 2.0),
                    right_free=float(right_free or 2.0),
                    min_clearance=float(actual_clearance) if actual_clearance is not None else None,
                    scene=str(getattr(pol, "scene", None) or "OPEN"),
                    policy_state=str(getattr(pol, "state", None) or ""),
                    speed=spd,
                    path_revision=int(self._path_version),
                    now=now,
                    geom=self.geom,
                    maneuver_mode=mm_pre,
                    authority=authority,
                    future_preview=fp,
                    obstacle_pass_state=str(fp.obstacle_pass_state) if fp is not None else "UNKNOWN",
                    obstacle_passed=bool(fp.obstacle_passed) if fp is not None else ext_passed,
                    preferred_side_hint=sp.preferred_side if sp is not None else (fp.preferred_side if fp else None),
                    side_commit_ready=bool(sp.commit_ready) if sp else False,
                    avoidance_phase=str(av.phase) if av is not None else "OPEN",
                )
            )
            self.last_local_plan_result = lp_res
            self.last_local_plan = lp_res.plan
            try:
                if fp is not None:
                    emit_obstacle_preview_events(
                        fp,
                        local_plan_id=None if lp_res.plan is None else lp_res.plan.plan_id,
                        local_plan_authority=authority,
                        maneuver_mode=mm_pre,
                        selected_side=(
                            lp_res.plan.selected_candidate_id if lp_res.plan is not None else None
                        ),
                    )
                    if diagnose_late_avoidance(fp, front_near=float(front_near)):
                        from agv_bridge.nav_observability import OBS

                        OBS.emit(
                            "LATE_AVOIDANCE_SUSPECTED",
                            level="WARN",
                            category="PLANNING",
                            component="obstacle_preview",
                            data={**fp.to_dict(), "front_near": float(front_near)},
                            min_interval_s=2.0,
                        )
                    if sp is not None and sp.oscillation_suspected:
                        from agv_bridge.nav_observability import OBS

                        OBS.emit(
                            "SIDE_PROBE_OSCILLATION",
                            level="WARN",
                            category="PLANNING",
                            component="side_probe",
                            data=sp.to_dict(),
                            min_interval_s=2.0,
                        )
                    if sp is not None and sp.wall_hugging_suspected:
                        from agv_bridge.nav_observability import OBS

                        OBS.emit(
                            "WALL_HUGGING_SUSPECTED",
                            level="NOTICE",
                            category="PLANNING",
                            component="side_probe",
                            data=sp.to_dict(),
                            min_interval_s=2.5,
                        )
                    if av is not None and av.global_reconnect_blocked and av.phase not in ("OBSTACLE_PASS", "GLOBAL_RECONNECT"):
                        from agv_bridge.nav_observability import OBS

                        OBS.emit(
                            "EARLY_GLOBAL_RECONNECT",
                            level="NOTICE",
                            category="PLANNING",
                            component="avoidance_phase",
                            data=av.to_dict(),
                            min_interval_s=1.5,
                        )
            except Exception:
                pass
            try:
                from agv_bridge.nav_observability import OBS

                for ev in lp_res.events or []:
                    OBS.emit(
                        ev,
                        level="INFO",
                        category="LOCAL_PLANNING",
                        component="rolling_local_planner",
                        data={
                            "plan_id": None if lp_res.plan is None else lp_res.plan.plan_id,
                            "horizon_m": None if lp_res.plan is None else lp_res.plan.horizon_m,
                            "horizon_s": None if lp_res.plan is None else lp_res.plan.horizon_s,
                            "selected": None if lp_res.plan is None else lp_res.plan.selected_candidate_id,
                            "authority": authority,
                            "target_vx": spd.target_vx,
                        },
                        min_interval_s=0.35,
                    )
                if authority == AUTH_ROLLING and lp_res.plan is not None:
                    OBS.emit(
                        "MPPI_TRACKING_LOCAL_PLAN",
                        level="INFO",
                        category="LOCAL_PLANNING",
                        component="mppi",
                        data={"plan_id": lp_res.plan.plan_id, "target_vx": spd.target_vx},
                        min_interval_s=1.0,
                    )
                OBS.emit(
                    "SPEED_TARGET_UPDATED",
                    level="INFO",
                    category="LOCAL_PLANNING",
                    component="speed_policy",
                    data=spd.to_dict(),
                    min_interval_s=0.8,
                )
            except Exception:
                pass
        except Exception:
            self.last_local_plan_result = None

        track_plan = (
            authority == AUTH_ROLLING
            and self.last_local_plan is not None
            and bool(self.last_local_plan.kinematic_valid)
            and len(self.last_local_plan.poses or []) >= 2
        )
        local_xy = self.last_local_plan.as_xy() if track_plan else None

        res = self.mppi.step(
            x,
            y,
            yaw,
            global_path,
            goal,
            collide,
            front_near=front_near,
            stuck_s=stuck_s,
            force_reverse=want_rev,
            control_mode=self.control_mode,
            maneuver_mode=decision.mode,
            vx_min=decision.vx_min,
            vx_max=decision.vx_max,
            force_vx=decision.force_vx,
            force_w=decision.force_w,
            path_follow_weight=pol.path_follow_weight,
            vx_scale=pol.profile.vx_scale,
            target_vx=None if authority != AUTH_ROLLING else (None if self.last_speed is None else self.last_speed.target_vx),
            local_plan_path=local_xy,
            local_plan_id=None if not track_plan else self.last_local_plan.plan_id,
            local_plan_horizon_m=None if not track_plan else self.last_local_plan.horizon_m,
        )
        self.policy.note_cmd_w(now, res.w)
        self.tick_phase(now, res.vx)

        # STEP 3F — record trusted breadcrumb from executed pose
        try:
            self.breadcrumb.record(
                now=now,
                x=x,
                y=y,
                yaw=yaw,
                vx=float(state_vx),
                w=float(res.w),
                mode=str(decision.mode or ""),
                emergency=emergency,
                collision=collision,
                commitment_side=str(getattr(pol, "committed_side", None) or "NONE"),
            )
        except Exception:
            pass
        return res

    def _recovery_target_distance_m(self) -> float:
        """Prefer Probe.backward clearance / corridor length as reverse target."""
        pb = self.last_probe
        if pb is None or pb.backward is None:
            return 1.0
        b = pb.backward
        if b.corridor and isinstance(b.corridor, dict):
            length = float(b.corridor.get("length_m") or b.corridor.get("length") or 0.0)
            if length >= 0.35:
                return min(2.0, max(0.5, length * 0.85))
        clr = b.min_clearance
        if clr is not None and float(clr) > 0.4:
            return min(1.5, max(0.5, float(clr) * 0.7))
        return 1.0

    def _select_active_corridor(
        self,
        *,
        live_auth: bool,
        auth_side: Optional[str],
        recovery,
    ) -> Optional[Dict[str, Any]]:
        """Pick UI/active PhysicalTrajectoryCorridor from Probe (shared poses)."""
        pb = self.last_probe
        if pb is None:
            return None
        src = "FORWARD"
        pr = pb.forward
        if recovery is not None and recovery.action == ACT_HISTORICAL_RETREAT and recovery.retreat:
            if recovery.retreat.corridor is not None:
                return recovery.retreat.corridor.to_dict()
        if recovery is not None and recovery.action == ACT_LOCAL_REVERSE:
            src, pr = "BACKWARD", pb.backward
        elif live_auth and auth_side == "LEFT":
            src, pr = "LEFT", pb.left
        elif live_auth and auth_side == "RIGHT":
            src, pr = "RIGHT", pb.right
        else:
            mm = str(self.maneuver.mode or "").upper()
            if "LEFT" in mm:
                src, pr = "LEFT", pb.left
            elif "RIGHT" in mm:
                src, pr = "RIGHT", pb.right
            elif "REVERSE" in mm:
                src, pr = "BACKWARD", pb.backward
        if pr is None:
            return None
        if pr.corridor:
            return pr.corridor
        return {
            "source": src,
            "status": pr.status,
            "valid": pr.status == "VALID",
            "collision": pr.collision,
            "min_clearance": pr.min_clearance,
            "poses": list(pr.poses or [])[:24],
            "note": "ProbeResult without corridor attach",
        }


LocalPathPlannerModel = LocalMppiModel
PathSelectionModel = LocalMppiModel
ControlModel = LocalMppiModel
