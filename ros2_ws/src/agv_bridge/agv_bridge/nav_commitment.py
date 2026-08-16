"""Avoidance Commitment — STEP 3C behavior memory (not a second FSM).

Owns: whether ordinary side re-selection is allowed.
Does NOT write vx/w. Does NOT authorize side switches (STEP 3E).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

# Phases (Policy sub-state only)
PHASE_NONE = "NONE"
PHASE_SELECTING = "SELECTING"
PHASE_COMMITTED_LEFT = "COMMITTED_LEFT"
PHASE_COMMITTED_RIGHT = "COMMITTED_RIGHT"
PHASE_EVALUATING = "EVALUATING"
PHASE_PASSING = "PASSING"
PHASE_REVALIDATING = "REVALIDATING"
PHASE_FAILED = "FAILED"
PHASE_RELEASED = "RELEASED"

FAIL_NONE = "NONE"
FAIL_HARD_INFEASIBLE = "HARD_INFEASIBLE"
FAIL_COLLISION = "COLLISION_PREDICTED"
FAIL_SOFT_COST = "TEMPORARY_COST_WORSE"
FAIL_SOFT_CLEARANCE = "SOFT_CLEARANCE_DEGRADED"

AUTH_LOCKED = "LOCKED_BY_COMMITMENT"
AUTH_REVALIDATION_REQUIRED = "REVALIDATION_REQUIRED"
AUTH_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # full authorize = 3E
AUTH_NONE = "NONE"


@dataclass
class AvoidanceCommitment:
    active: bool = False
    side: str = "NONE"  # LEFT / RIGHT / NONE
    phase: str = PHASE_NONE
    reason: str = "NONE"
    failure_reason: str = FAIL_NONE
    hard_fail: bool = False
    soft_fail_streak: int = 0
    soft_fail_since: Optional[float] = None
    started_at: Optional[float] = None
    start_pose: Optional[Tuple[float, float, float]] = None
    obstacle_signature: Optional[str] = None
    last_switch_at: Optional[float] = None
    switch_count: int = 0
    release_reason: str = "NONE"
    authorization_status: str = AUTH_NONE

    def age_s(self, now: float) -> float:
        if not self.started_at:
            return 0.0
        return max(0.0, float(now) - float(self.started_at))

    def to_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        ts = now if now is not None else time.time()
        return {
            "implemented": True,
            "active": self.active,
            "side": self.side,
            "phase": self.phase,
            "reason": self.reason,
            "failure_reason": self.failure_reason,
            "hard_fail": self.hard_fail,
            "soft_fail_streak": self.soft_fail_streak,
            "soft_fail_since": self.soft_fail_since,
            "started_at": self.started_at,
            "age_s": round(self.age_s(ts), 3),
            "obstacle_signature": self.obstacle_signature,
            "last_switch_at": self.last_switch_at,
            "switch_count": self.switch_count,
            "release_reason": self.release_reason,
            "authorization_status": self.authorization_status,
            "start_pose": (
                {
                    "x": self.start_pose[0],
                    "y": self.start_pose[1],
                    "yaw": self.start_pose[2],
                }
                if self.start_pose
                else None
            ),
        }

    def clear(self, *, reason: str = "RESET") -> None:
        self.active = False
        self.side = "NONE"
        self.phase = PHASE_NONE
        self.reason = "NONE"
        self.failure_reason = FAIL_NONE
        self.hard_fail = False
        self.soft_fail_streak = 0
        self.soft_fail_since = None
        self.started_at = None
        self.start_pose = None
        self.obstacle_signature = None
        self.release_reason = reason
        self.authorization_status = AUTH_NONE


def make_obstacle_signature(
    *,
    front_near: float,
    left_free: float,
    right_free: float,
    scene: str = "",
) -> str:
    """Cheap coarse signature — binds commitment to a local obstacle event."""
    return (
        f"f{round(float(front_near), 1)}"
        f"_L{round(float(left_free), 1)}"
        f"_R{round(float(right_free), 1)}"
        f"_{scene or 'NA'}"
    )
