"""P1-1 Open-space / scene speed policy.

Produces a desired target_vx. Does NOT emit cmd_vel. Does NOT bypass
Safety, Physics acceleration, Recovery, or ManeuverFSM.

Layers (downstream):
  geom.max_vx → this target → Local Planner candidate bounds → MPPI → Safety → Physics
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_kinematic import W_MAX_CONTROL

# Configurable — NOT a final vehicle calibration.
OPEN_CRUISE_VX = float(os.environ.get("NAV_OPEN_CRUISE_VX", "0.30") or 0.30)
TURN_MODERATE_VX = float(os.environ.get("NAV_TURN_MODERATE_VX", "0.25") or 0.25)
TIGHT_VX = float(os.environ.get("NAV_TIGHT_VX", "0.18") or 0.18)
GOAL_NEAR_M = 1.20
GOAL_NEAR_VX = 0.12
CLEARANCE_SLOW_M = 1.40
FRONT_SLOW_M = 1.80

LIMIT_NONE = "NONE"
LIMIT_HARDWARE = "HARDWARE"
LIMIT_CURVATURE = "CURVATURE"
LIMIT_CLEARANCE = "CLEARANCE"
LIMIT_FRONT = "FRONT_NEAR"
LIMIT_GOAL = "GOAL_NEAR"
LIMIT_SCENE = "SCENE_PROFILE"
LIMIT_KINEMATIC = "KINEMATIC_FEASIBLE"

# P0-D.1 speed reasons
REASON_NORMAL_CRUISE = "NORMAL_CRUISE"
REASON_FUTURE_PREVIEW = "FUTURE_OBSTACLE_PREVIEW"
REASON_SIDE_PROBE = "SIDE_PROBE"
REASON_SIDE_COMMIT = "SIDE_COMMIT"
REASON_CURVATURE_LIMIT = "CURVATURE_LIMIT"
REASON_CLEARANCE_LIMIT = "CLEARANCE_LIMIT"
REASON_SAFETY_LIMIT = "SAFETY_LIMIT"
REASON_DYNAMIC_RESUME = "DYNAMIC_RESUME"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class SpeedPolicyResult:
    target_vx: float
    v_max_allowed: float
    v_min_allowed: float
    cruise_vx: float
    reason: str
    limits: List[str] = field(default_factory=list)
    scene: str = "OPEN"
    policy_state: str = "FOLLOW_GLOBAL"
    vx_scale: float = 1.0
    abs_kappa: float = 0.0
    front_near: float = 30.0
    min_clearance: float = 30.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "target_vx": round(self.target_vx, 4),
            "v_max_allowed": round(self.v_max_allowed, 4),
            "v_min_allowed": round(self.v_min_allowed, 4),
            "cruise_vx": round(self.cruise_vx, 4),
            "reason": self.reason,
            "limits": list(self.limits),
            "scene": self.scene,
            "policy_state": self.policy_state,
            "vx_scale": round(self.vx_scale, 4),
            "abs_kappa": round(self.abs_kappa, 4),
            "front_near": round(self.front_near, 3),
            "min_clearance": None if self.min_clearance is None else round(self.min_clearance, 3),
        }


class SpeedPolicy:
    """Scene-aware desired speed. Recovery / reverse are not owned here."""

    def __init__(self, geom: Optional[VehicleGeometry] = None) -> None:
        self.geom = geom or get_vehicle_geometry()
        self.last: Optional[SpeedPolicyResult] = None

    def reset(self) -> None:
        self.last = None

    def compute(
        self,
        *,
        scene: str = "OPEN",
        policy_state: str = "FOLLOW_GLOBAL",
        vx_scale: float = 1.0,
        state_vx: float = 0.0,
        front_near: float = 30.0,
        rear_near: float = 30.0,
        left_free: float = 5.0,
        right_free: float = 5.0,
        min_clearance: Optional[float] = None,
        goal_distance_m: Optional[float] = None,
        abs_kappa: float = 0.0,
        heading_error: float = 0.0,
        kinematic_feasible_vx: Optional[float] = None,
        recovery_active: bool = False,
        force_reverse: bool = False,
        avoidance_phase: str = "OPEN",
        probe_active: bool = False,
        commit_ready: bool = False,
        future_max_abs_kappa: float = 0.0,
        dynamic_resume_vx: Optional[float] = None,
    ) -> SpeedPolicyResult:
        g = self.geom or DEFAULT_GEOM
        hw = float(g.max_vx)
        cruise = _clamp(float(OPEN_CRUISE_VX), 0.08, hw)
        limits: List[str] = []
        target = cruise
        reason = "OPEN_CRUISE"

        sc = str(scene or "OPEN").upper()
        pst = str(policy_state or "").upper()
        clr = float(min_clearance) if min_clearance is not None else min(
            float(front_near), float(left_free), float(right_free)
        )

        if force_reverse or recovery_active or "RECOVERY" in pst or "REVERSE" in pst:
            # Do not invent a forward cruise during recovery — FSM/MPPI force path owns vx.
            res = SpeedPolicyResult(
                target_vx=0.0,
                v_max_allowed=hw,
                v_min_allowed=-0.18,
                cruise_vx=cruise,
                reason="RECOVERY_DEFER",
                limits=[LIMIT_SCENE],
                scene=sc,
                policy_state=pst,
                vx_scale=float(vx_scale),
                abs_kappa=float(abs_kappa),
                front_near=float(front_near),
                min_clearance=clr,
            )
            self.last = res
            return res

        ap = str(avoidance_phase or "OPEN").upper()
        # P0-D.1 soft slowdown — gradual, not 0.30→0.10 cliff
        if dynamic_resume_vx is not None and float(dynamic_resume_vx) > 0.04:
            target = min(cruise, float(dynamic_resume_vx))
            reason = REASON_DYNAMIC_RESUME
            limits.append(LIMIT_SCENE)
        elif ap == "FUTURE_PREVIEW":
            target = min(target, max(0.27, cruise - 0.02))
            reason = REASON_FUTURE_PREVIEW
            limits.append(LIMIT_SCENE)
        elif ap in ("SIDE_PROBE", "OBSTACLE_APPROACH"):
            target = min(target, max(0.24, cruise - 0.05))
            reason = REASON_SIDE_PROBE
            limits.append(LIMIT_SCENE)
        elif ap == "SIDE_COMMIT":
            target = min(target, max(0.22, cruise - 0.07))
            reason = REASON_SIDE_COMMIT
            limits.append(LIMIT_SCENE)
        elif probe_active and ap not in ("OPEN", "OBSTACLE_PASS", "GLOBAL_RECONNECT"):
            target = min(target, max(0.26, cruise - 0.03))
            reason = REASON_FUTURE_PREVIEW
            limits.append(LIMIT_SCENE)

        if sc in ("TIGHT", "NARROW") or min(float(left_free), float(right_free)) < 0.55:
            target = min(target, TIGHT_VX)
            reason = "TIGHT"
            limits.append(LIMIT_SCENE)
        elif abs(float(heading_error)) > 0.55 or float(abs_kappa) > 0.55:
            target = min(target, TURN_MODERATE_VX)
            reason = "MODERATE_TURN"
            limits.append(LIMIT_CURVATURE)
        elif sc not in ("OPEN", "OPEN_SPACE", ""):
            if "CAUTION" in sc or "CAUTION" in pst:
                target = min(target, 0.22)
                reason = "CAUTION"
                limits.append(LIMIT_SCENE)
            elif "AVOID" in sc or "AVOID" in pst or "OBSTACLE" in pst:
                target = min(target, TIGHT_VX)
                reason = "OBSTACLE_SCENE"
                limits.append(LIMIT_SCENE)

        # Existing obstacle evidence only (not P0-D unrecoverable distance)
        if float(front_near) < FRONT_SLOW_M:
            scale = max(0.35, float(front_near) / FRONT_SLOW_M)
            target = min(target, cruise * scale)
            limits.append(LIMIT_FRONT)
            if float(front_near) < g.front_cost_m:
                reason = "FRONT_NEAR"

        if clr < CLEARANCE_SLOW_M:
            scale = max(0.40, clr / CLEARANCE_SLOW_M)
            target = min(target, cruise * scale)
            limits.append(LIMIT_CLEARANCE)
            if reason == "OPEN_CRUISE":
                reason = "CLEARANCE"

        if goal_distance_m is not None and float(goal_distance_m) < GOAL_NEAR_M:
            target = min(target, GOAL_NEAR_VX)
            limits.append(LIMIT_GOAL)
            reason = "GOAL_NEAR"

        # Curvature → v <= w_max / |κ|  (current + future preview)
        kappa_use = max(abs(float(abs_kappa)), abs(float(future_max_abs_kappa)))
        if kappa_use > 1e-4:
            v_k = float(min(W_MAX_CONTROL, g.max_w)) / max(kappa_use, 1e-4)
            if v_k + 1e-6 < target:
                target = min(target, v_k)
                limits.append(LIMIT_CURVATURE)
                reason = "CURVATURE"

        if kinematic_feasible_vx is not None and float(kinematic_feasible_vx) > 0.04:
            if float(kinematic_feasible_vx) + 1e-6 < target:
                target = min(target, float(kinematic_feasible_vx))
                limits.append(LIMIT_KINEMATIC)
                reason = "KINEMATIC_FEASIBLE"

        scale = _clamp(float(vx_scale), 0.2, 1.0)
        if scale < 0.999:
            target *= scale
            limits.append(LIMIT_SCENE)

        target = _clamp(target, 0.06, hw)
        if target + 1e-6 >= hw:
            limits.append(LIMIT_HARDWARE)

        v_max_allowed = _clamp(max(target, cruise * 0.5), 0.08, hw)
        # Candidate generator may sample up to v_max_allowed, not only target
        v_max_allowed = min(hw, max(v_max_allowed, target))

        res = SpeedPolicyResult(
            target_vx=round(target, 4),
            v_max_allowed=round(v_max_allowed, 4),
            v_min_allowed=0.06,
            cruise_vx=cruise,
            reason=reason,
            limits=limits or [LIMIT_NONE],
            scene=sc,
            policy_state=pst,
            vx_scale=scale,
            abs_kappa=float(kappa_use),
            front_near=float(front_near),
            min_clearance=clr,
        )
        self.last = res
        return res
