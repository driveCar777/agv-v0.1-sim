"""Recovery ladder + Historical Retreat Probe — STEP 3F.

Policy-facing pure evaluation. Does NOT write vx/w. Does NOT bypass Safety.
Consumes ProbeBundle + Breadcrumb; may footprint-check breadcrumb segments
using existing _collide_body (no new kinematics family).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.local_maneuver import _collide_body
from agv_bridge.nav_breadcrumb import BreadcrumbPoint, TrajectoryBreadcrumb
from agv_bridge.nav_geometry import DEFAULT_GEOM, MAX_RECOVERY_ATTEMPTS
from agv_bridge.nav_probe import (
    STATUS_INVALID,
    STATUS_UNKNOWN,
    STATUS_VALID,
    ProbeBundle,
)
from agv_bridge.nav_trajectory import (
    SRC_BACKWARD,
    SRC_RETREAT,
    PhysicalTrajectoryCorridor,
    TrajectorySample,
    build_corridor_from_poses,
)

CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

# Classification
CLASS_NORMAL = "NORMAL"
CLASS_DEGRADED = "DEGRADED"
CLASS_DEAD_END = "DEAD_END"
CLASS_DEADLOCK = "DEADLOCK"
CLASS_RECOVERY_EVAL = "RECOVERY_EVALUATE"
CLASS_RECOVERY_ACTIVE = "RECOVERY_ACTIVE"
CLASS_RECOVERY_FAILED = "RECOVERY_FAILED"
CLASS_SAFE_STOP = "SAFE_STOP"

# Actions
ACT_NONE = "NONE"
ACT_CONTINUE = "CONTINUE_LOCAL"
ACT_SIDE_SWITCH = "AUTHORIZED_SIDE_SWITCH"
ACT_LOCAL_REVERSE = "LOCAL_REVERSE"
ACT_HISTORICAL_RETREAT = "HISTORICAL_RETREAT"
ACT_REPLAN = "REPLAN"
ACT_WAIT = "WAIT"
ACT_SAFE_STOP = "SAFE_STOP"

MAX_RETREAT_DISTANCE_M = 6.0
MAX_RETREAT_SEGMENTS = 80
RETREAT_STEP_CHECK_M = 0.12


@dataclass
class HistoricalRetreatResult:
    status: str = STATUS_UNKNOWN
    safe_distance_m: float = 0.0
    target_index: int = -1  # into retreat polyline (newest=0)
    blocked_index: int = -1
    min_clearance: Optional[float] = None
    collision: bool = False
    segments_checked: int = 0
    target_pose: Optional[Dict[str, float]] = None
    corridor: Optional[PhysicalTrajectoryCorridor] = None
    failure_reason: str = "NONE"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "safe_distance_m": round(self.safe_distance_m, 3),
            "target_index": self.target_index,
            "blocked_index": self.blocked_index,
            "min_clearance": None if self.min_clearance is None else round(float(self.min_clearance), 3),
            "collision": self.collision,
            "segments_checked": self.segments_checked,
            "target_pose": self.target_pose,
            "failure_reason": self.failure_reason,
            "corridor": self.corridor.to_dict() if self.corridor else None,
        }


@dataclass
class RecoveryDecision:
    classification: str = CLASS_NORMAL
    action: str = ACT_NONE
    reason: str = "NONE"
    allow_recovery: bool = False
    release_commitment: bool = False
    retreat: Optional[HistoricalRetreatResult] = None
    active_source: str = "NONE"
    gates: Dict[str, bool] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "implemented": True,
            "classification": self.classification,
            "action": self.action,
            "reason": self.reason,
            "allow_recovery": self.allow_recovery,
            "release_commitment": self.release_commitment,
            "active_source": self.active_source,
            "gates": dict(self.gates),
            "retreat": self.retreat.to_dict() if self.retreat else None,
            "max_recovery_attempts": MAX_RECOVERY_ATTEMPTS,
        }


def classify_situation(
    *,
    probe: Optional[ProbeBundle],
    safety_zero: bool = False,
    planned_rejected_by_safety: bool = False,
    stuck_s: float = 0.0,
    progress_low: bool = False,
) -> str:
    if probe is None:
        return CLASS_DEGRADED
    f = probe.forward.status
    l = probe.left.status
    r = probe.right.status
    b = probe.backward.status
    sides_blocked = f == STATUS_INVALID and l == STATUS_INVALID and r == STATUS_INVALID
    # Deadlock: geometry may look open but execution blocked
    if (f == STATUS_VALID or l == STATUS_VALID or r == STATUS_VALID) and (
        safety_zero or planned_rejected_by_safety
    ) and stuck_s >= 2.0:
        return CLASS_DEADLOCK
    if sides_blocked and progress_low:
        if b == STATUS_VALID:
            return CLASS_DEAD_END
        return CLASS_DEAD_END
    if sides_blocked:
        return CLASS_DEAD_END if b != STATUS_VALID else CLASS_DEAD_END
    if f == STATUS_INVALID and (l == STATUS_VALID or r == STATUS_VALID):
        return CLASS_DEGRADED
    return CLASS_NORMAL


def evaluate_historical_retreat(
    *,
    breadcrumb: TrajectoryBreadcrumb,
    collide: Optional[CollideFn],
    clearance_at: Optional[ClearanceFn] = None,
    max_distance_m: float = MAX_RETREAT_DISTANCE_M,
    now: Optional[float] = None,
) -> HistoricalRetreatResult:
    """Segment-wise revalidation of breadcrumb (newest → older). Partial success OK."""
    res = HistoricalRetreatResult()
    if collide is None:
        res.status = STATUS_UNKNOWN
        res.failure_reason = "NO_COLLIDE"
        return res
    poly = breadcrumb.retreat_polyline(trusted_only=True, max_m=max_distance_m)
    if len(poly) < 2:
        res.status = STATUS_INVALID
        res.failure_reason = "NO_BREADCRUMB"
        return res

    # poly is newest-first; retreat walks toward older
    safe_poses: List[TrajectorySample] = []
    safe_dist = 0.0
    min_clr = 99.0
    last_safe_i = 0
    blocked_i = -1
    for i in range(len(poly) - 1):
        a = poly[i]
        b = poly[i + 1]
        seg_len = math.hypot(b.x - a.x, b.y - a.y)
        if seg_len < 1e-4:
            continue
        steps = max(1, int(math.ceil(seg_len / RETREAT_STEP_CHECK_M)))
        hit = False
        for k in range(1, steps + 1):
            t = k / steps
            x = a.x + (b.x - a.x) * t
            y = a.y + (b.y - a.y) * t
            # yaw faces along segment direction of travel (toward older = retreat heading)
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            res.segments_checked += 1
            if _collide_body(x, y, yaw, collide):
                hit = True
                blocked_i = i + 1
                break
            if clearance_at:
                min_clr = min(min_clr, float(clearance_at(x, y)))
        if hit:
            res.collision = True
            break
        safe_dist += seg_len
        last_safe_i = i + 1
        safe_poses.append(
            TrajectorySample(t=float(i) * 0.1, x=b.x, y=b.y, yaw=math.atan2(b.y - a.y, b.x - a.x))
        )
        if safe_dist >= max_distance_m:
            break
        if res.segments_checked >= MAX_RETREAT_SEGMENTS:
            break

    res.blocked_index = blocked_i
    res.target_index = last_safe_i
    res.safe_distance_m = safe_dist
    res.min_clearance = None if min_clr >= 98 else min_clr
    if last_safe_i > 0 and safe_dist >= 0.25:
        tgt = poly[last_safe_i]
        res.target_pose = {"x": tgt.x, "y": tgt.y, "yaw": tgt.yaw}
        res.status = STATUS_VALID
        # Corridor along validated retreat (newest → target)
        corridor_pts = [
            TrajectorySample(t=float(j) * 0.1, x=poly[j].x, y=poly[j].y, yaw=poly[j].yaw)
            for j in range(0, last_safe_i + 1)
        ]
        res.corridor = build_corridor_from_poses(
            corridor_pts,
            source=SRC_RETREAT,
            status=STATUS_VALID,
            min_clearance=res.min_clearance,
        )
    else:
        res.status = STATUS_INVALID
        res.failure_reason = "BLOCKED" if res.collision else "TOO_SHORT"
    return res


def evaluate_recovery(
    *,
    probe: Optional[ProbeBundle],
    breadcrumb: Optional[TrajectoryBreadcrumb],
    collide: Optional[CollideFn],
    clearance_at: Optional[ClearanceFn] = None,
    side_switch_authorized: bool = False,
    safety_zero: bool = False,
    planned_rejected_by_safety: bool = False,
    stuck_s: float = 0.0,
    progress_low: bool = False,
    recovery_attempts: int = 0,
    dynamic_short: bool = False,
    allow_replan: bool = True,
    emergency: bool = False,
) -> RecoveryDecision:
    """Priority ladder (Policy evidence). Safety remains final gate at execution."""
    dec = RecoveryDecision()
    if emergency:
        dec.classification = CLASS_SAFE_STOP
        dec.action = ACT_SAFE_STOP
        dec.reason = "EMERGENCY"
        return dec

    cls = classify_situation(
        probe=probe,
        safety_zero=safety_zero,
        planned_rejected_by_safety=planned_rejected_by_safety,
        stuck_s=stuck_s,
        progress_low=progress_low,
    )
    dec.classification = cls
    gates = {
        "probe_ok": probe is not None,
        "forward_invalid": bool(probe and probe.forward.status == STATUS_INVALID),
        "left_invalid": bool(probe and probe.left.status == STATUS_INVALID),
        "right_invalid": bool(probe and probe.right.status == STATUS_INVALID),
        "backward_valid": bool(probe and probe.backward.status == STATUS_VALID),
        "side_switch_authorized": side_switch_authorized,
        "attempts_ok": recovery_attempts < MAX_RECOVERY_ATTEMPTS,
        "dynamic_wait": dynamic_short,
    }
    dec.gates = gates

    if probe is None:
        dec.action = ACT_WAIT
        dec.reason = "NO_PROBE"
        return dec

    # 1) Continue / side switch when alternatives exist — NOT reverse first
    if probe.forward.status == STATUS_VALID:
        dec.action = ACT_CONTINUE
        dec.reason = "FORWARD_VALID"
        dec.classification = CLASS_NORMAL
        return dec
    if side_switch_authorized:
        dec.action = ACT_SIDE_SWITCH
        dec.reason = "POLICY_AUTHORIZED_SWITCH"
        return dec
    if probe.left.status == STATUS_VALID or probe.right.status == STATUS_VALID:
        dec.action = ACT_CONTINUE
        dec.reason = "SIDE_STILL_VALID"
        dec.classification = CLASS_DEGRADED
        return dec

    # Dynamic: prefer WAIT before recovery motion
    if dynamic_short and cls in (CLASS_DEAD_END, CLASS_DEADLOCK, CLASS_DEGRADED):
        dec.action = ACT_WAIT
        dec.reason = "DYNAMIC_WAIT"
        return dec

    if not gates["attempts_ok"]:
        dec.classification = CLASS_RECOVERY_FAILED
        dec.action = ACT_SAFE_STOP
        dec.reason = "RECOVERY_EXHAUSTED"
        return dec

    # Enter recovery evaluate when F/L/R all invalid (or deadlock)
    if cls in (CLASS_DEAD_END, CLASS_DEADLOCK) or (
        gates["forward_invalid"] and gates["left_invalid"] and gates["right_invalid"]
    ):
        dec.classification = CLASS_RECOVERY_EVAL
        # Local reverse first if BACKWARD VALID
        if gates["backward_valid"] and probe.backward.escape_available is not False:
            dec.action = ACT_LOCAL_REVERSE
            dec.reason = "BACKWARD_PROBE_VALID"
            dec.allow_recovery = True
            dec.release_commitment = True
            dec.active_source = SRC_BACKWARD
            return dec

        # Historical retreat
        if breadcrumb is not None and collide is not None:
            retreat = evaluate_historical_retreat(
                breadcrumb=breadcrumb,
                collide=collide,
                clearance_at=clearance_at,
            )
            dec.retreat = retreat
            if retreat.status == STATUS_VALID:
                dec.action = ACT_HISTORICAL_RETREAT
                dec.reason = "HISTORICAL_RETREAT_VALID"
                dec.allow_recovery = True
                dec.release_commitment = True
                dec.active_source = SRC_RETREAT
                return dec

        if allow_replan:
            dec.action = ACT_REPLAN
            dec.reason = "NO_LOCAL_ESCAPE→REPLAN"
            dec.release_commitment = True
            return dec

        dec.classification = CLASS_RECOVERY_FAILED
        dec.action = ACT_SAFE_STOP
        dec.reason = "NO_ESCAPE"
        return dec

    dec.action = ACT_CONTINUE
    dec.reason = "DEFAULT"
    return dec
