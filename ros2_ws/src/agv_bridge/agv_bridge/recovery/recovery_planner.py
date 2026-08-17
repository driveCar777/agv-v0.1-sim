"""V0.2 M2 RecoveryPlanner — decide next recovery strategy (does NOT drive).

Priority ladder (adjusted by obstacle class):
  RECHECK → REPROBE → REBUILD_CORRIDOR → REPLAN → ALTERNATE_SIDE → WAIT_DYNAMIC → GLOBAL_REPLAN → SAFE_STOP
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from agv_bridge.nav_geometry import MAX_RECOVERY_ATTEMPTS
from agv_bridge.nav_planner_state import (
    LOCAL_PLAN_INFEASIBLE,
    LOCAL_RECOVERY,
    NAVIGATION_FAILED,
    PLANNER_NORMAL,
    PLANNER_SAFE_STOP,
    REASON_MPPI_INFEASIBLE,
    REASON_RECOVERY_REVERSE,
    REASON_STALE_SENSOR,
    REASON_LOC_INVALID,
)
from agv_bridge.nav_probe import STATUS_INVALID, STATUS_VALID, ProbeBundle

# Recovery actions
ACT_RECHECK = "RECHECK_PERCEPTION"
ACT_REPROBE = "REPROBE"
ACT_REBUILD = "REBUILD_CORRIDOR"
ACT_REPLAN = "REPLAN"
ACT_ALT_SIDE = "ALTERNATE_SIDE"
ACT_WAIT = "WAIT_DYNAMIC"
ACT_GLOBAL = "GLOBAL_REPLAN"
ACT_SAFE_STOP = "SAFE_STOP"
ACT_REQUIRES_REVERSE = "RECOVERY_REQUIRES_REVERSE"

RECOVERY_TIMEOUT_S = 30.0
RECOVERY_VX_SCALE = 0.55


@dataclass
class RecoveryState:
    attempt_count: int = 0
    max_attempts: int = MAX_RECOVERY_ATTEMPTS
    state_enter_time: float = 0.0
    timeout_s: float = RECOVERY_TIMEOUT_S
    last_failure_reason: str = "NONE"
    current_action: str = "NONE"
    planner_state: str = PLANNER_NORMAL
    alternate_side_tried: bool = False
    dynamic_wait_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "attempt_count": self.attempt_count,
            "max_attempts": self.max_attempts,
            "state_enter_time": round(self.state_enter_time, 3),
            "timeout_s": self.timeout_s,
            "last_failure_reason": self.last_failure_reason,
            "current_action": self.current_action,
            "planner_state": self.planner_state,
            "alternate_side_tried": self.alternate_side_tried,
            "dynamic_wait_s": round(self.dynamic_wait_s, 2),
        }


@dataclass
class RecoveryPlanResult:
    planner_state: str = PLANNER_NORMAL
    action: str = "NONE"
    reason: str = "NONE"
    vx_scale: float = 1.0
    release_commitment: bool = False
    request_global_replan: bool = False
    request_reprobe: bool = False
    request_alternate_side: bool = False
    navigation_failed: bool = False
    requires_reverse: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "planner_state": self.planner_state,
            "action": self.action,
            "reason": self.reason,
            "vx_scale": round(self.vx_scale, 3),
            "release_commitment": self.release_commitment,
            "request_global_replan": self.request_global_replan,
            "request_reprobe": self.request_reprobe,
            "request_alternate_side": self.request_alternate_side,
            "navigation_failed": self.navigation_failed,
            "requires_reverse": self.requires_reverse,
        }


class RecoveryPlanner:
    """Maps planner failures → recovery strategy. Does not emit vx/w."""

    def __init__(self, max_attempts: int = MAX_RECOVERY_ATTEMPTS) -> None:
        self.state = RecoveryState(max_attempts=max_attempts)

    def reset(self) -> None:
        self.state = RecoveryState(max_attempts=self.state.max_attempts)

    def on_success(self) -> None:
        self.state.attempt_count = 0
        self.state.current_action = "NONE"
        self.state.planner_state = PLANNER_NORMAL
        self.state.last_failure_reason = "NONE"
        self.state.alternate_side_tried = False

    def on_planner_failure(
        self,
        *,
        now: float,
        failure_reason: str,
        probe: Optional[ProbeBundle] = None,
        dynamic_short: bool = False,
        dynamic_long: bool = False,
        sensor_stale: bool = False,
        localization_invalid: bool = False,
        committed_side: Optional[str] = None,
        alternate_side_available: bool = False,
        requires_reverse: bool = False,
    ) -> RecoveryPlanResult:
        res = RecoveryPlanResult()
        self.state.last_failure_reason = failure_reason or REASON_MPPI_INFEASIBLE

        if sensor_stale:
            self.state.planner_state = PLANNER_SAFE_STOP
            self.state.current_action = ACT_SAFE_STOP
            res.planner_state = PLANNER_SAFE_STOP
            res.action = ACT_SAFE_STOP
            res.reason = REASON_STALE_SENSOR
            return res

        if localization_invalid:
            self.state.planner_state = PLANNER_SAFE_STOP
            self.state.current_action = ACT_SAFE_STOP
            res.planner_state = PLANNER_SAFE_STOP
            res.action = ACT_SAFE_STOP
            res.reason = REASON_LOC_INVALID
            return res

        if requires_reverse:
            self.state.planner_state = PLANNER_SAFE_STOP
            self.state.current_action = ACT_REQUIRES_REVERSE
            res.planner_state = PLANNER_SAFE_STOP
            res.action = ACT_REQUIRES_REVERSE
            res.reason = REASON_RECOVERY_REVERSE
            res.requires_reverse = True
            return res

        if self.state.state_enter_time <= 0.0:
            self.state.state_enter_time = now

        self.state.attempt_count += 1
        self.state.planner_state = LOCAL_RECOVERY if self.state.attempt_count > 0 else LOCAL_PLAN_INFEASIBLE

        if self.state.attempt_count > self.state.max_attempts:
            self.state.planner_state = NAVIGATION_FAILED
            self.state.current_action = ACT_SAFE_STOP
            res.planner_state = NAVIGATION_FAILED
            res.action = ACT_SAFE_STOP
            res.reason = "RECOVERY_EXHAUSTED"
            res.navigation_failed = True
            return res

        if now - self.state.state_enter_time > self.state.timeout_s:
            self.state.planner_state = NAVIGATION_FAILED
            res.planner_state = NAVIGATION_FAILED
            res.action = ACT_SAFE_STOP
            res.reason = "RECOVERY_TIMEOUT"
            res.navigation_failed = True
            return res

        res.planner_state = LOCAL_RECOVERY
        res.vx_scale = RECOVERY_VX_SCALE

        static_blocked = _probe_static_blocked(probe)
        if dynamic_short and not dynamic_long:
            self.state.current_action = ACT_WAIT
            res.action = ACT_WAIT
            res.reason = "DYNAMIC_OBSTACLE_WAIT"
            self.state.dynamic_wait_s = now - self.state.state_enter_time
            return res

        # Ladder by attempt count
        attempt = self.state.attempt_count
        if attempt == 1:
            self.state.current_action = ACT_REPROBE
            res.action = ACT_REPROBE
            res.reason = "MPPI_INFEASIBLE_REPROBE"
            res.request_reprobe = True
        elif attempt == 2:
            self.state.current_action = ACT_REBUILD
            res.action = ACT_REBUILD
            res.reason = "REBUILD_EXECUTION_CORRIDOR"
            res.request_reprobe = True
        elif attempt == 3:
            self.state.current_action = ACT_REPLAN
            res.action = ACT_REPLAN
            res.reason = "LOCAL_REPLAN"
            res.request_global_replan = False
        elif attempt == 4 and alternate_side_available and not self.state.alternate_side_tried:
            self.state.current_action = ACT_ALT_SIDE
            self.state.alternate_side_tried = True
            res.action = ACT_ALT_SIDE
            res.reason = f"ALTERNATE_FROM_{committed_side or 'NONE'}"
            res.request_alternate_side = True
            res.release_commitment = True
        elif dynamic_long or (dynamic_short and attempt >= 5):
            self.state.current_action = ACT_WAIT
            res.action = ACT_WAIT
            res.reason = "DYNAMIC_WAIT"
        elif attempt >= 5 or static_blocked:
            self.state.current_action = ACT_GLOBAL
            res.action = ACT_GLOBAL
            res.reason = "GLOBAL_REPLAN"
            res.request_global_replan = True
            res.release_commitment = True
        else:
            self.state.current_action = ACT_RECHECK
            res.action = ACT_RECHECK
            res.reason = "RECHECK_PERCEPTION"

        return res


def _probe_static_blocked(probe: Optional[ProbeBundle]) -> bool:
    if probe is None:
        return False
    return (
        probe.forward.status == STATUS_INVALID
        and probe.left.status == STATUS_INVALID
        and probe.right.status == STATUS_INVALID
    )
