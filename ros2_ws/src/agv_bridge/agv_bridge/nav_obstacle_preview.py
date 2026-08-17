"""P0-D — Forward Future Obstacle Preview (predictive approach trigger).

Predicts swept-footprint collision along global reference BEFORE front_near reactive threshold.
Does NOT replace Safety (front_stop_m). Does NOT extend display pink lookahead.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import trajectory_collision
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_global_preview import (
    GLOBAL_PREVIEW_MAX_M,
    GLOBAL_PREVIEW_MIN_M,
    GlobalPreviewCache,
    densify_polyline,
    slice_forward,
)
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

MIN_FUTURE_PREVIEW_M = float(os.environ.get("NAV_FUTURE_PREVIEW_MIN_M", "2.0") or 2.0)
NORMAL_FUTURE_PREVIEW_M = float(os.environ.get("NAV_FUTURE_PREVIEW_M", "5.0") or 5.0)
MAX_FUTURE_PREVIEW_M = float(os.environ.get("NAV_FUTURE_PREVIEW_MAX_M", "8.0") or 8.0)

# Control / planner / sensor latency budget (seconds)
DEFAULT_LATENCY_S = float(os.environ.get("NAV_APPROACH_LATENCY_S", "0.25") or 0.25)
CLEARANCE_WARNING_M = float(os.environ.get("NAV_CLEARANCE_WARNING_M", "0.55") or 0.55)

PASS_APPROACHING = "APPROACHING"
PASS_BESIDE = "BESIDE"
PASS_PASSING = "PASSING"
PASS_PASSED = "PASSED"
PASS_UNKNOWN = "UNKNOWN"

SOURCE_GLOBAL = "GLOBAL_REFERENCE"
SOURCE_DEGRADED = "DEGRADED"
SOURCE_NONE = "NONE"


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def hard_stop_distance_m(geom: VehicleGeometry = DEFAULT_GEOM) -> float:
    return float(geom.front_stop_m)


def required_avoidance_distance(
    vx: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    *,
    latency_s: float = DEFAULT_LATENCY_S,
    w: float = 0.0,
) -> float:
    """Speed-aware distance needed before maneuver must begin."""
    v = max(abs(float(vx)), 0.08)
    d_latency = v * max(0.05, float(latency_s))
    d_setup = 0.45 * float(geom.length) + 0.25
    # Turn setup grows with speed and heading rate demand
    d_turn = float(geom.width) * 1.15 + v * 0.65 + 0.15 * abs(float(w))
    d_safety = float(geom.front_stop_m) + float(getattr(geom, "safety_margin_m", 0.08) or 0.08) + 0.20
    a_dec = max(0.25, float(geom.acc_v) * 0.65)
    d_brake = (v * v) / (2.0 * a_dec) if v > 0.06 else 0.0
    return d_latency + d_setup + d_turn + d_safety + d_brake


def adaptive_preview_distance_m(
    *,
    vx: float,
    global_preview_m: Optional[float] = None,
    goal_distance_m: Optional[float] = None,
    local_horizon_m: Optional[float] = None,
    max_curvature: Optional[float] = None,
) -> float:
    v = max(abs(float(vx)), 0.10)
    tight = max_curvature is not None and float(max_curvature) > 0.45
    base = MIN_FUTURE_PREVIEW_M if tight else NORMAL_FUTURE_PREVIEW_M
    speed_m = min(MAX_FUTURE_PREVIEW_M, max(MIN_FUTURE_PREVIEW_M, v * 5.0))
    preview = max(base, speed_m)
    if global_preview_m is not None and global_preview_m > 0.5:
        preview = max(preview, min(float(global_preview_m), MAX_FUTURE_PREVIEW_M))
    if local_horizon_m is not None and local_horizon_m > 0.5:
        preview = max(preview, min(float(local_horizon_m) * 1.5, MAX_FUTURE_PREVIEW_M))
    if goal_distance_m is not None:
        preview = min(preview, max(MIN_FUTURE_PREVIEW_M, float(goal_distance_m)))
    return max(MIN_FUTURE_PREVIEW_M, min(MAX_FUTURE_PREVIEW_M, preview))


def compute_dynamic_lookahead_m(
    *,
    vx: float,
    follow_path_length_m: Optional[float] = None,
    local_plan_horizon_m: Optional[float] = None,
    first_collision_distance_m: Optional[float] = None,
    geom: VehicleGeometry = DEFAULT_GEOM,
) -> Tuple[float, str]:
    """Controller reference design — PP lookahead from active follow_path (not display pink)."""
    v = max(abs(float(vx)), 0.08)
    la = max(0.55, min(2.2, max(0.70, v * 1.25)))
    src = "SPEED_SCALED"
    if local_plan_horizon_m is not None and float(local_plan_horizon_m) > 0.4:
        la = min(la, max(0.6, float(local_plan_horizon_m) * 0.70))
        src = "LOCAL_PLAN_HORIZON"
    if follow_path_length_m is not None and follow_path_length_m > 0.3:
        la = min(la, max(0.5, float(follow_path_length_m) * 0.85))
    if first_collision_distance_m is not None and first_collision_distance_m > 0.4:
        safe_cap = max(0.45, float(first_collision_distance_m) * 0.85)
        if la > safe_cap:
            la = safe_cap
            src = "COLLISION_CLAMP"
    la = max(0.45, min(2.2, la))
    return la, src


def _rollout_arc(
    x: float,
    y: float,
    yaw: float,
    *,
    vx: float,
    kappa: float,
    horizon_m: float,
    dt: float = 0.10,
) -> List[Dict[str, float]]:
    w = float(vx) * float(kappa)
    steps = max(2, int(math.ceil(horizon_m / max(abs(vx) * dt, 1e-3))))
    poses: List[Dict[str, float]] = [{"x": x, "y": y, "yaw": yaw}]
    cx, cy, cyaw = x, y, yaw
    for _ in range(steps):
        cx += vx * math.cos(cyaw) * dt
        cy += vx * math.sin(cyaw) * dt
        cyaw = _wrap(cyaw + w * dt)
        poses.append({"x": cx, "y": cy, "yaw": cyaw})
        if math.hypot(cx - x, cy - y) >= horizon_m:
            break
    return poses


def _corridor_check(
    poses: Sequence[Dict[str, float]],
    collide: CollideFn,
    clearance_at: Optional[ClearanceFn],
    geom: VehicleGeometry,
    *,
    margin_m: float = 0.0,
) -> Tuple[bool, Optional[float], bool]:
    hit = trajectory_collision(poses, collide, geom, margin_m=margin_m)
    if hit.collision:
        return False, 0.0, True
    clrs: List[float] = []
    if clearance_at is not None:
        for p in poses[::2]:
            try:
                clrs.append(float(clearance_at(p["x"], p["y"])))
            except Exception:
                pass
    mc = min(clrs) if clrs else None
    warn = mc is not None and mc < CLEARANCE_WARNING_M
    return True, mc, warn


def classify_obstacle_pass_state(
    *,
    front_near: float,
    left_free: float,
    right_free: float,
    future_collision: bool,
    approach_active: bool,
    lateral_error: float,
    geom: VehicleGeometry = DEFAULT_GEOM,
    external_passed: bool = False,
) -> str:
    if external_passed:
        return PASS_PASSED
    if not future_collision and front_near > geom.front_clear_m:
        return PASS_PASSED
    if approach_active and front_near > geom.front_cost_m + 0.35:
        return PASS_APPROACHING
    if front_near < geom.front_cost_m + 0.55 and min(left_free, right_free) < 1.2:
        return PASS_BESIDE
    if future_collision and front_near < geom.front_clear_m and abs(lateral_error) > 0.25:
        return PASS_PASSING
    if future_collision:
        return PASS_APPROACHING
    if front_near < geom.front_clear_m + 0.4:
        return PASS_BESIDE
    return PASS_UNKNOWN


@dataclass
class FuturePreviewResult:
    preview_distance_m: float = 0.0
    first_collision_distance_m: Optional[float] = None
    first_clearance_warning_distance_m: Optional[float] = None
    min_clearance_m: Optional[float] = None
    collision: bool = False
    collision_index: int = -1
    collision_pose: Optional[Dict[str, float]] = None
    collision_reason: Optional[str] = None
    left_clearance: Optional[float] = None
    right_clearance: Optional[float] = None
    forward_clearance: Optional[float] = None
    left_valid: bool = True
    right_valid: bool = True
    forward_valid: bool = True
    obstacle_count: int = 0
    reference_source: str = SOURCE_NONE
    reference_revision: int = 0
    future_collision: bool = False
    required_avoidance_distance_m: float = 0.0
    detection_distance_m: Optional[float] = None
    maneuver_start_distance_m: Optional[float] = None
    hard_stop_distance_m: float = 0.70
    approach_active: bool = False
    obstacle_pass_state: str = PASS_UNKNOWN
    obstacle_passed: bool = False
    future_global_blocked: bool = False
    preview_poses: List[Dict[str, float]] = field(default_factory=list)
    left_corridor_poses: List[Dict[str, float]] = field(default_factory=list)
    right_corridor_poses: List[Dict[str, float]] = field(default_factory=list)
    evidence_quality: str = "OK"
    left_reconnect_m: Optional[float] = None
    right_reconnect_m: Optional[float] = None
    preferred_side: Optional[str] = None
    generated_at: float = 0.0
    lookahead_distance_m: Optional[float] = None
    lookahead_source: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "preview_distance_m": round(self.preview_distance_m, 3),
            "future_preview_m": round(self.preview_distance_m, 3),
            "first_collision_distance_m": None
            if self.first_collision_distance_m is None
            else round(self.first_collision_distance_m, 3),
            "first_warning_distance_m": None
            if self.first_clearance_warning_distance_m is None
            else round(self.first_clearance_warning_distance_m, 3),
            "first_clearance_warning_distance_m": None
            if self.first_clearance_warning_distance_m is None
            else round(self.first_clearance_warning_distance_m, 3),
            "min_clearance_m": None if self.min_clearance_m is None else round(self.min_clearance_m, 3),
            "collision": self.collision,
            "collision_index": self.collision_index,
            "collision_pose": self.collision_pose,
            "collision_reason": self.collision_reason,
            "left_clearance": None if self.left_clearance is None else round(self.left_clearance, 3),
            "right_clearance": None if self.right_clearance is None else round(self.right_clearance, 3),
            "forward_clearance": None if self.forward_clearance is None else round(self.forward_clearance, 3),
            "left_valid": self.left_valid,
            "right_valid": self.right_valid,
            "forward_valid": self.forward_valid,
            "obstacle_count": int(self.obstacle_count),
            "reference_source": self.reference_source,
            "reference_revision": int(self.reference_revision),
            "future_collision": self.future_collision,
            "required_avoidance_distance_m": round(self.required_avoidance_distance_m, 3),
            "detection_distance_m": None
            if self.detection_distance_m is None
            else round(self.detection_distance_m, 3),
            "maneuver_start_distance_m": None
            if self.maneuver_start_distance_m is None
            else round(self.maneuver_start_distance_m, 3),
            "hard_stop_distance_m": round(self.hard_stop_distance_m, 3),
            "approach_active": self.approach_active,
            "obstacle_pass_state": self.obstacle_pass_state,
            "obstacle_passed": self.obstacle_passed,
            "future_global_blocked": self.future_global_blocked,
            "preview_poses": self.preview_poses[:48],
            "left_corridor_poses": self.left_corridor_poses[:32],
            "right_corridor_poses": self.right_corridor_poses[:32],
            "evidence_quality": self.evidence_quality,
            "left_reconnect_m": None if self.left_reconnect_m is None else round(self.left_reconnect_m, 3),
            "right_reconnect_m": None if self.right_reconnect_m is None else round(self.right_reconnect_m, 3),
            "preferred_side": self.preferred_side,
            "generated_at": self.generated_at,
            "lookahead_distance_m": self.lookahead_distance_m,
            "lookahead_source": self.lookahead_source,
            "controls_vehicle": False,
            "source": "FUTURE_OBSTACLE_PREVIEW",
        }


def compute_future_preview(
    *,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    w: float = 0.0,
    global_path: Optional[Sequence[Pt]] = None,
    path_progress_s: Optional[float] = None,
    path_revision: int = 0,
    global_preview_m: Optional[float] = None,
    goal_distance_m: Optional[float] = None,
    local_horizon_m: Optional[float] = None,
    collide: Optional[CollideFn] = None,
    clearance_at: Optional[ClearanceFn] = None,
    front_near: float = 30.0,
    left_free: float = 5.0,
    right_free: float = 5.0,
    lateral_error: float = 0.0,
    geom: Optional[VehicleGeometry] = None,
    cache: Optional[GlobalPreviewCache] = None,
    external_obstacle_passed: bool = False,
    sensor_invalid: bool = False,
    now: Optional[float] = None,
) -> FuturePreviewResult:
    """Forward swept-footprint preview along global reference + side corridors."""
    g = geom or get_vehicle_geometry()
    ts = float(now or time.time())
    collide_fn = collide or (lambda _x, _y: False)
    req_avoid = required_avoidance_distance(vx, g, w=w)
    d_hard = hard_stop_distance_m(g)

    if sensor_invalid:
        return FuturePreviewResult(
            preview_distance_m=0.0,
            required_avoidance_distance_m=req_avoid,
            hard_stop_distance_m=d_hard,
            evidence_quality="UNKNOWN",
            reference_source=SOURCE_DEGRADED,
            reference_revision=int(path_revision),
            generated_at=ts,
        )

    preview_m = adaptive_preview_distance_m(
        vx=vx,
        global_preview_m=global_preview_m,
        goal_distance_m=goal_distance_m,
        local_horizon_m=local_horizon_m,
    )

    res = FuturePreviewResult(
        preview_distance_m=preview_m,
        required_avoidance_distance_m=req_avoid,
        maneuver_start_distance_m=req_avoid,
        hard_stop_distance_m=d_hard,
        reference_revision=int(path_revision),
        generated_at=ts,
    )

    path = list(global_path or [])
    if len(path) < 2:
        res.evidence_quality = "NO_PATH"
        res.reference_source = SOURCE_NONE
        pass_st = classify_obstacle_pass_state(
            front_near=front_near,
            left_free=left_free,
            right_free=right_free,
            future_collision=False,
            approach_active=False,
            lateral_error=lateral_error,
            geom=g,
            external_passed=external_obstacle_passed,
        )
        res.obstacle_pass_state = pass_st
        res.obstacle_passed = pass_st == PASS_PASSED
        return res

    c = cache or GlobalPreviewCache()
    densified = c.ensure(path)
    if not densified:
        res.evidence_quality = "DEGRADED"
        res.reference_source = SOURCE_DEGRADED
        return res

    proj = project_pose_to_path(x, y, yaw, path)
    s0 = float(path_progress_s) if path_progress_s is not None else float(proj.s)
    poses = slice_forward(densified, s_start=s0, preview_m=preview_m)
    if not poses:
        poses = [{"x": x, "y": y, "yaw": yaw, "distance_along_path": 0.0}]
    res.preview_poses = poses
    res.reference_source = SOURCE_GLOBAL

    # Forward swept collision along global reference (segment-wise for accurate distance)
    first_col_dist: Optional[float] = None
    warn_dist: Optional[float] = None
    min_clr: Optional[float] = None
    col_pose: Optional[Dict[str, float]] = None
    col_idx = -1
    prev_seg: List[Dict[str, float]] = []
    for i, p in enumerate(poses):
        seg = prev_seg + [p] if prev_seg else [p]
        if len(seg) >= 2:
            seg_hit = trajectory_collision(seg[-2:], collide_fn, g, margin_m=0.0)
            if seg_hit.collision and first_col_dist is None:
                first_col_dist = float(p.get("distance_along_path") or 0.0)
                col_pose = seg_hit.first_pose
                col_idx = i
        if clearance_at is not None:
            try:
                c = float(clearance_at(p["x"], p["y"]))
                min_clr = c if min_clr is None else min(min_clr, c)
                if c < CLEARANCE_WARNING_M and warn_dist is None:
                    warn_dist = float(p.get("distance_along_path") or 0.0)
            except Exception:
                pass
        prev_seg = [p]

    # Full-path confirmation (swept along entire preview)
    hit = trajectory_collision(poses, collide_fn, g, margin_m=0.0)
    if min_clr is not None:
        res.min_clearance_m = min_clr
        res.forward_clearance = min_clr
    if warn_dist is not None:
        res.first_clearance_warning_distance_m = warn_dist

    if hit.collision or first_col_dist is not None:
        res.collision = True
        res.future_collision = True
        res.collision_index = col_idx if col_idx >= 0 else int(hit.first_index)
        res.collision_pose = col_pose or hit.first_pose
        res.collision_reason = "SWEPT_FOOTPRINT"
        res.first_collision_distance_m = first_col_dist if first_col_dist is not None else float(
            poses[max(0, min(hit.first_index, len(poses) - 1))].get("distance_along_path") or 0.0
        )
        res.obstacle_count = 1
        res.forward_valid = False
        res.future_global_blocked = True
        res.detection_distance_m = res.first_collision_distance_m
    else:
        res.forward_valid = True
        res.detection_distance_m = preview_m

    # Side corridors — swept arc validation (not blind LEFT)
    v_cor = max(0.12, min(float(g.max_vx), max(abs(vx), 0.18)))
    left_poses = _rollout_arc(x, y, yaw, vx=v_cor, kappa=0.55, horizon_m=preview_m)
    right_poses = _rollout_arc(x, y, yaw, vx=v_cor, kappa=-0.55, horizon_m=preview_m)
    res.left_corridor_poses = left_poses
    res.right_corridor_poses = right_poses
    l_ok, l_clr, _ = _corridor_check(left_poses, collide_fn, clearance_at, g)
    r_ok, r_clr, _ = _corridor_check(right_poses, collide_fn, clearance_at, g)
    res.left_valid = l_ok
    res.right_valid = r_ok
    res.left_clearance = l_clr
    res.right_clearance = r_clr

    # Global reconnect hint after obstacle (endpoint lateral to global path)
    if path and len(path) >= 2:
        try:
            le = left_poses[-1]
            re = right_poses[-1]
            pl = project_pose_to_path(le["x"], le["y"], le["yaw"], path)
            pr = project_pose_to_path(re["x"], re["y"], re["yaw"], path)
            res.left_reconnect_m = abs(float(pl.lateral_m))
            res.right_reconnect_m = abs(float(pr.lateral_m))
            if l_ok and r_ok:
                if res.left_reconnect_m < res.right_reconnect_m - 0.15:
                    res.preferred_side = "LEFT"
                elif res.right_reconnect_m < res.left_reconnect_m - 0.15:
                    res.preferred_side = "RIGHT"
            elif l_ok:
                res.preferred_side = "LEFT"
            elif r_ok:
                res.preferred_side = "RIGHT"
        except Exception:
            pass

    res.approach_active = bool(
        res.future_collision
        and res.first_collision_distance_m is not None
        and res.first_collision_distance_m < req_avoid
    )
    pass_st = classify_obstacle_pass_state(
        front_near=front_near,
        left_free=left_free,
        right_free=right_free,
        future_collision=res.future_collision,
        approach_active=res.approach_active,
        lateral_error=lateral_error,
        geom=g,
        external_passed=external_obstacle_passed,
    )
    res.obstacle_pass_state = pass_st
    res.obstacle_passed = pass_st == PASS_PASSED or external_obstacle_passed

    la, la_src = compute_dynamic_lookahead_m(
        vx=vx,
        local_plan_horizon_m=local_horizon_m,
        first_collision_distance_m=res.first_collision_distance_m,
        geom=g,
    )
    res.lookahead_distance_m = la
    res.lookahead_source = la_src
    return res


def emit_obstacle_preview_events(
    preview: FuturePreviewResult,
    *,
    local_plan_id: Optional[str] = None,
    local_plan_authority: Optional[str] = None,
    maneuver_mode: Optional[str] = None,
    pp_follow_source: Optional[str] = None,
    selected_side: Optional[str] = None,
) -> None:
    """Emit P0-D diagnostic events via nav_observability (no second log bus)."""
    try:
        from agv_bridge.nav_observability import OBS

        d = preview.to_dict()
        d["local_plan_id"] = local_plan_id
        d["local_plan_authority"] = local_plan_authority
        d["maneuver_mode"] = maneuver_mode
        d["pp_follow_source"] = pp_follow_source

        if preview.future_collision:
            OBS.emit(
                "FUTURE_OBSTACLE_DETECTED",
                level="NOTICE",
                category="PLANNING",
                component="obstacle_preview",
                data=d,
                min_interval_s=0.4,
            )
        if preview.first_clearance_warning_distance_m is not None:
            OBS.emit(
                "FUTURE_CLEARANCE_WARNING",
                level="NOTICE",
                category="PLANNING",
                component="obstacle_preview",
                data=d,
                min_interval_s=0.5,
            )
        if preview.approach_active:
            OBS.emit(
                "OBSTACLE_APPROACH_ENTER",
                level="INFO",
                category="PLANNING",
                component="obstacle_preview",
                data=d,
                min_interval_s=0.35,
            )
        elif preview.obstacle_pass_state == PASS_PASSED:
            OBS.emit(
                "OBSTACLE_APPROACH_EXIT",
                level="INFO",
                category="PLANNING",
                component="obstacle_preview",
                data=d,
                min_interval_s=0.8,
            )

        # Late avoidance diagnostic (emit from caller with front_near via diagnose_late_avoidance)
        if (
            preview.future_collision
            and preview.first_collision_distance_m is not None
            and preview.first_collision_distance_m > preview.required_avoidance_distance_m + 0.3
            and not preview.approach_active
            and maneuver_mode in (None, "", "FORWARD", "IDLE")
        ):
            OBS.emit(
                "PLANNER_DELAYED_AVOIDANCE",
                level="WARN",
                category="PLANNING",
                component="obstacle_preview",
                data=d,
                min_interval_s=1.5,
            )

        if (
            selected_side
            and pp_follow_source == "GLOBAL_PATH"
            and preview.approach_active
            and maneuver_mode not in ("LOCAL_LEFT", "LOCAL_RIGHT")
        ):
            OBS.emit(
                "LOCAL_PLAN_EXECUTION_MISMATCH",
                level="WARN",
                category="PLANNING",
                component="obstacle_preview",
                data={**d, "selected_side": selected_side},
                min_interval_s=1.0,
            )
    except Exception:
        pass


def front_near_late(preview: FuturePreviewResult, *, front_near: Optional[float]) -> bool:
    if front_near is None:
        return False
    return (
        preview.future_collision
        and front_near < preview.hard_stop_distance_m + 0.25
        and preview.first_collision_distance_m is not None
        and preview.first_collision_distance_m > preview.hard_stop_distance_m + 0.8
    )


def diagnose_late_avoidance(
    preview: FuturePreviewResult,
    *,
    front_near: float,
    turn_start_distance_m: Optional[float] = None,
) -> bool:
    """True when future collision was early but maneuver started near hard stop."""
    if not preview.future_collision or preview.first_collision_distance_m is None:
        return False
    ts = turn_start_distance_m if turn_start_distance_m is not None else front_near
    early = preview.first_collision_distance_m > preview.required_avoidance_distance_m * 0.6
    late = ts <= preview.hard_stop_distance_m + 0.25
    return early and late
