"""导航三层封装（Web 仿真）。"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

from agv_bridge.mppi_controller import DiffDriveMppi, MppiResult, get_control_mode
from agv_bridge.maneuver import (
    ALIGN,
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
    STUCK_REVERSE_TRIGGER_S,
)

Pt = Tuple[float, float]


class GlobalPlannerModel:
    STUCK_SPLICE_S = GLOBAL_SPLICE_S
    STUCK_FORCE_ALT_S = GLOBAL_ALT_S

    def __init__(self) -> None:
        self.last_metrics: Dict[str, Any] = {}
        self.last_raw_path: List[Pt] = []

    def plan(self, world, start: Pt, goal: Pt) -> List[Pt]:
        # Processed global path (clearance+turn+LOS+smooth) for PP/MPPI tracking
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
    """局部控制 + Maneuver FSM（真正裁决在 sim_api_ext Safety）。"""

    def __init__(self) -> None:
        self.geom = DEFAULT_GEOM
        self.mppi = DiffDriveMppi(geom=self.geom)
        self.maneuver = ManeuverFSM()
        self.phase = "forward"  # legacy phase string for telemetry compatibility
        self._rev_cooldown_until = 0.0
        self._reverse_until = 0.0
        self.recovery_attempts = 0
        self.control_mode = get_control_mode()
        self.last_maneuver = None

    def set_control_mode(self, mode: str) -> str:
        m = (mode or "mppi").strip().lower()
        self.control_mode = "pp_only" if m in ("pp", "pp_only", "pure_pursuit") else "mppi"
        os.environ["AGV_LOCAL_CONTROL"] = self.control_mode
        return self.control_mode

    def reset(self) -> None:
        self.mppi.reset()
        self.maneuver.reset()
        self.phase = "forward"
        self._rev_cooldown_until = 0.0
        self._reverse_until = 0.0
        self.recovery_attempts = 0
        self.last_maneuver = None

    def begin_reverse_escape(self, now: float) -> bool:
        """Legacy hook — prefer ManeuverFSM. Kept for recovery attempt accounting."""
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
        return True

    def tick_phase(self, now: float, cmd_vx: float) -> None:
        # Maneuver FSM owns transitions; keep light legacy sync for reverse timeout
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
    ) -> MppiResult:
        def collide(px: float, py: float) -> bool:
            return world.collides(px, py, robot_r=self.geom.local_radius, include_actors=True)

        def clearance_at(px: float, py: float) -> float:
            try:
                return float(world.clearance_at_xy(px, py))
            except Exception:
                return 1.0

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
            lateral_error=lateral_error,
            stuck_s=stuck_s,
            recovery_attempts=self.recovery_attempts,
            candidates=candidates_hint,
            clearance_at=clearance_at,
            collide=collide,
            actual_clearance=actual_clearance,
            nav_active=nav_active,
        )
        self.last_maneuver = decision
        self.phase = decision.legacy_phase

        # Count recovery when FSM newly enters reverse
        if decision.mode == REVERSE_ESCAPE and force_reverse is False:
            # enter via FSM — ensure attempt counted once per enter
            if self.maneuver.time_in_mode(now) < 0.08:
                if self.recovery_attempts < MAX_RECOVERY_ATTEMPTS and now >= self._rev_cooldown_until:
                    self.recovery_attempts = min(MAX_RECOVERY_ATTEMPTS, self.recovery_attempts + 1)
                    self._reverse_until = now + REVERSE_MAX_S

        want_rev = decision.mode == REVERSE_ESCAPE
        # Legacy force_reverse only if FSM is not in active align/turn/stop
        if force_reverse and decision.mode not in (ALIGN, TURN_IN_PLACE, POST_TURN, SAFE_STOP):
            want_rev = True
        if decision.mode in (ALIGN, TURN_IN_PLACE, POST_TURN):
            want_rev = False

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
        )
        self.tick_phase(now, res.vx)
        return res


LocalPathPlannerModel = LocalMppiModel
PathSelectionModel = LocalMppiModel
ControlModel = LocalMppiModel
