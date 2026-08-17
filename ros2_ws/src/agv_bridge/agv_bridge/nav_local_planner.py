"""P1-1 Rolling Local Planner — medium-horizon physical trajectory.

Does NOT replace LocalManeuverSelector (obstacle-triggered side compare).
Does NOT emit cmd_vel. Does NOT bypass MPPI or Safety.
Does NOT own Recovery / REVERSE_ESCAPE / TURN_IN_PLACE.

GLOBAL (4-8m) → this module (2-4s / ~1-3m spatial) → MPPI (1-2s) → Safety.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import trajectory_collision
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_kinematic import W_MAX_CONTROL, control_limits
from agv_bridge.nav_speed_policy import OPEN_CRUISE_VX, SpeedPolicyResult
from agv_bridge.path_progress import project_pose_to_path

try:
    from agv_bridge.nav_obstacle_preview import FuturePreviewResult
except ImportError:
    FuturePreviewResult = Any  # type: ignore

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]
Pose = Dict[str, float]

MIN_LOCAL_HORIZON_S = float(os.environ.get("NAV_LOCAL_HORIZON_MIN_S", "2.0") or 2.0)
NORMAL_LOCAL_HORIZON_S = float(os.environ.get("NAV_LOCAL_HORIZON_S", "3.0") or 3.0)
MAX_LOCAL_HORIZON_S = float(os.environ.get("NAV_LOCAL_HORIZON_MAX_S", "4.0") or 4.0)
PLANNING_PERIOD_S = float(os.environ.get("NAV_LOCAL_PLAN_PERIOD_S", "0.20") or 0.20)
MAX_STALE_AGE_S = 0.80
OPEN_SPATIAL_FLOOR_M = 2.0
TIGHT_SPATIAL_FLOOR_M = 1.0
HARD_TIME_CAP_S = 8.0
ROLLOUT_DT = 0.10
DEVIATION_REPLAN_M = 0.40

KIND_FORWARD = "FORWARD"
KIND_LEFT_ARC = "LEFT_ARC"
KIND_RIGHT_ARC = "RIGHT_ARC"

STATUS_CREATED = "CREATED"
STATUS_ACTIVE = "ACTIVE"
STATUS_REPLACED = "REPLACED"
STATUS_EXPIRED = "EXPIRED"
STATUS_REJECTED = "REJECTED"
STATUS_FALLBACK = "FALLBACK"
STATUS_NONE = "NONE"

AUTH_ROLLING = "ROLLING_LOCAL"
AUTH_AVOIDANCE = "LOCAL_SELECTOR"
AUTH_RECOVERY = "RECOVERY"
AUTH_FSM = "FSM"


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _polyline_len(poses: Sequence[Any]) -> float:
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
            s += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        prev = xy
    return s


def _nearest_dist(path: Sequence[Pt], x: float, y: float) -> float:
    if not path:
        return 1e6
    return min(math.hypot(p[0] - x, p[1] - y) for p in path)


@dataclass
class LocalPlanRequest:
    x: float
    y: float
    yaw: float
    vx: float
    w: float = 0.0
    global_path: Optional[List[Pt]] = None
    global_preview_m: Optional[float] = None
    goal: Optional[Pt] = None
    collide: Optional[CollideFn] = None
    clearance_at: Optional[ClearanceFn] = None
    front_near: float = 30.0
    left_free: float = 5.0
    right_free: float = 5.0
    min_clearance: Optional[float] = None
    scene: str = "OPEN"
    policy_state: str = "FOLLOW_GLOBAL"
    speed: Optional[SpeedPolicyResult] = None
    path_revision: int = 0
    now: float = 0.0
    geom: Optional[VehicleGeometry] = None
    kinematic_valid_global: Optional[bool] = None
    maneuver_mode: str = ""
    authority: str = AUTH_ROLLING
    future_preview: Optional["FuturePreviewResult"] = None
    obstacle_pass_state: str = "UNKNOWN"
    obstacle_passed: bool = False
    preferred_side_hint: Optional[str] = None
    side_commit_ready: bool = False
    avoidance_phase: str = "OPEN"


@dataclass
class LocalCandidate:
    candidate_id: str
    kind: str
    valid: bool
    vx: float
    w: float
    kappa: float
    poses: List[Pose]
    distance_m: float
    duration_s: float
    endpoint: Optional[Pose] = None
    yaw_end: float = 0.0
    min_clearance: Optional[float] = None
    collision: bool = False
    kinematic_valid: bool = True
    reject_reason: Optional[str] = None
    score: float = 0.0
    cost_breakdown: Dict[str, float] = field(default_factory=dict)
    progress_m: float = 0.0
    global_deviation_m: float = 0.0
    heading_error: float = 0.0
    reconnect_m: float = 0.0
    recoverability: float = 1.0
    selected: bool = False

    def to_dict(self, pose_limit: int = 64) -> Dict[str, Any]:
        poses = self.poses[: max(8, int(pose_limit))]
        return {
            "candidate_id": self.candidate_id,
            "kind": self.kind,
            "valid": self.valid,
            "selected": self.selected,
            "vx": round(self.vx, 4),
            "w": round(self.w, 4),
            "kappa": round(self.kappa, 4),
            "distance_m": round(self.distance_m, 3),
            "duration_s": round(self.duration_s, 3),
            "poses": poses,
            "endpoint": self.endpoint,
            "yaw_end": round(self.yaw_end, 4),
            "min_clearance": None if self.min_clearance is None else round(self.min_clearance, 3),
            "collision": self.collision,
            "kinematic_valid": self.kinematic_valid,
            "reject_reason": self.reject_reason,
            "score": round(self.score, 3),
            "score_breakdown": {k: round(v, 3) for k, v in self.cost_breakdown.items()},
            "progress_m": round(self.progress_m, 3),
            "global_deviation_m": round(self.global_deviation_m, 3),
            "heading_error": round(self.heading_error, 4),
            "reconnect_m": round(self.reconnect_m, 3),
            "recoverability": round(self.recoverability, 3),
            "source": "ROLLING_LOCAL_PLANNER",
        }


@dataclass
class LocalPlan:
    plan_id: str
    revision: int
    timestamp: float
    generated_at: float
    expires_at: float
    horizon_s: float
    horizon_m: float
    status: str
    selected_candidate_id: str
    poses: List[Pose]
    curvature: List[float]
    speed_profile: List[float]
    min_clearance: Optional[float]
    kinematic_valid: bool
    speed_target: float
    authority: str = AUTH_ROLLING
    candidates: List[LocalCandidate] = field(default_factory=list)
    compute_ms: float = 0.0
    note: str = ""
    fallback: bool = False

    def to_dict(self, *, include_candidates: bool = True) -> Dict[str, Any]:
        sel = next((c for c in self.candidates if c.selected), None)
        return {
            "plan_id": self.plan_id,
            "revision": self.revision,
            "timestamp": self.timestamp,
            "generated_at": self.generated_at,
            "expires_at": self.expires_at,
            "horizon_s": round(self.horizon_s, 3),
            "horizon_m": round(self.horizon_m, 3),
            "status": self.status,
            "active": self.status in (STATUS_CREATED, STATUS_ACTIVE, STATUS_REPLACED, STATUS_FALLBACK),
            "selected_candidate": self.selected_candidate_id,
            "selected_kind": None if sel is None else sel.kind,
            "poses": self.poses[:64],
            "curvature": [round(k, 4) for k in self.curvature[:32]],
            "speed_profile": [round(v, 4) for v in self.speed_profile[:32]],
            "min_clearance": None if self.min_clearance is None else round(self.min_clearance, 3),
            "kinematic_valid": self.kinematic_valid,
            "speed_target": round(self.speed_target, 4),
            "authority": self.authority,
            "candidate_count": len(self.candidates),
            "valid_count": sum(1 for c in self.candidates if c.valid),
            "compute_ms": round(self.compute_ms, 3),
            "note": self.note,
            "fallback": self.fallback,
            "source": "ROLLING_LOCAL_PLANNER",
            "controls_vehicle": False,
            "candidates": [c.to_dict() for c in self.candidates] if include_candidates else [],
        }

    def as_xy(self) -> List[Pt]:
        out: List[Pt] = []
        for p in self.poses:
            out.append((float(p["x"]), float(p["y"])))
        return out


@dataclass
class LocalPlanResult:
    plan: Optional[LocalPlan]
    status: str
    events: List[str] = field(default_factory=list)
    refresh_hz: float = 0.0
    age_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "status": self.status,
            "events": list(self.events),
            "refresh_hz": round(self.refresh_hz, 3),
            "age_s": None if self.age_s is None else round(self.age_s, 3),
            "plan": None if self.plan is None else self.plan.to_dict(),
        }
        return d


class RollingLocalPlanner:
    """Continuous rolling-horizon local planner (OPEN / FOLLOW_GLOBAL / APPROACH)."""

    def __init__(self, geom: Optional[VehicleGeometry] = None) -> None:
        self.geom = geom or get_vehicle_geometry()
        self._n = 0
        self._revision = 0
        self.last_plan: Optional[LocalPlan] = None
        self.last_result: Optional[LocalPlanResult] = None
        self.last_update_ts: float = 0.0
        self.last_compute_ms: float = 0.0

    def reset(self) -> None:
        self.last_plan = None
        self.last_result = None
        self.last_update_ts = 0.0
        self.last_compute_ms = 0.0
        self._revision = 0

    def _next_id(self) -> str:
        self._n += 1
        return f"LP-{self._n:06d}"

    def adaptive_horizon(
        self,
        *,
        scene: str,
        vx: float,
        target_vx: float,
        front_near: float,
        global_preview_m: Optional[float],
        goal_distance_m: Optional[float],
        left_free: float,
        right_free: float,
    ) -> Tuple[float, float]:
        sc = str(scene or "OPEN").upper()
        v = max(abs(float(vx)), abs(float(target_vx)), 0.12)
        tight = sc in ("TIGHT", "NARROW") or min(float(left_free), float(right_free)) < 0.55
        approach = float(front_near) < 2.4
        if tight:
            hs = MIN_LOCAL_HORIZON_S
            floor_m = TIGHT_SPATIAL_FLOOR_M
        elif approach:
            hs = 0.5 * (MIN_LOCAL_HORIZON_S + NORMAL_LOCAL_HORIZON_S)
            floor_m = 1.40
        elif v >= 0.26:
            hs = MAX_LOCAL_HORIZON_S
            floor_m = OPEN_SPATIAL_FLOOR_M
        else:
            hs = NORMAL_LOCAL_HORIZON_S
            floor_m = OPEN_SPATIAL_FLOOR_M
        hs = max(MIN_LOCAL_HORIZON_S, min(MAX_LOCAL_HORIZON_S, hs))
        spatial = max(floor_m, v * hs)
        cap = float(global_preview_m) if global_preview_m and global_preview_m > 0.2 else 8.0
        if goal_distance_m is not None:
            cap = min(cap, max(0.4, float(goal_distance_m)))
        spatial = min(spatial, cap)
        return hs, spatial

    def _should_refresh(self, req: LocalPlanRequest) -> bool:
        if self.last_plan is None:
            return True
        now = float(req.now or time.time())
        if now - self.last_update_ts >= PLANNING_PERIOD_S:
            return True
        # Pose left the previous plan
        pts = self.last_plan.as_xy()
        if _nearest_dist(pts, req.x, req.y) > DEVIATION_REPLAN_M:
            return True
        if int(req.path_revision) != int(getattr(self.last_plan, "_path_revision", req.path_revision)):
            return True
        return False

    def update(self, req: LocalPlanRequest) -> LocalPlanResult:
        t0 = time.perf_counter()
        now = float(req.now or time.time())
        events: List[str] = []
        if not self._should_refresh(req) and self.last_plan is not None:
            age = now - self.last_plan.generated_at
            hz = 1.0 / PLANNING_PERIOD_S
            self.last_plan.status = STATUS_ACTIVE
            res = LocalPlanResult(
                plan=self.last_plan, status=STATUS_ACTIVE, events=[], refresh_hz=hz, age_s=age
            )
            self.last_result = res
            return res

        plan = self._plan_once(req, now)
        self.last_compute_ms = (time.perf_counter() - t0) * 1000.0
        if plan is not None:
            plan.compute_ms = self.last_compute_ms
            setattr(plan, "_path_revision", int(req.path_revision))
            if self.last_plan is None:
                plan.status = STATUS_CREATED
                events.append("LOCAL_PLAN_CREATED")
                events.append("LOCAL_PLAN_SELECTED")
            else:
                plan.status = STATUS_REPLACED
                events.append("LOCAL_PLAN_REPLACED")
                events.append("LOCAL_PLAN_SELECTED")
            self.last_plan = plan
            self.last_update_ts = now
            age = 0.0
            status = plan.status
        else:
            # Fallback to last valid if fresh enough
            if self.last_plan is not None and (now - self.last_plan.generated_at) <= MAX_STALE_AGE_S:
                self.last_plan.status = STATUS_FALLBACK
                self.last_plan.fallback = True
                events.append("LOCAL_PLAN_FALLBACK")
                events.append("LOCAL_PLAN_REJECTED")
                plan = self.last_plan
                status = STATUS_FALLBACK
                age = now - plan.generated_at
            else:
                if self.last_plan is not None:
                    self.last_plan.status = STATUS_EXPIRED
                    events.append("LOCAL_PLAN_EXPIRED")
                events.append("LOCAL_PLAN_REJECTED")
                status = STATUS_REJECTED
                age = 0.0
                plan = None

        hz = 1.0 / max(1e-3, PLANNING_PERIOD_S)
        res = LocalPlanResult(plan=plan, status=status, events=events, refresh_hz=hz, age_s=age)
        self.last_result = res
        return res

    def _plan_once(self, req: LocalPlanRequest, now: float) -> Optional[LocalPlan]:
        geom = req.geom or self.geom
        lim = control_limits(geom)
        w_max = float(lim["w_max"])
        collide = req.collide or (lambda _x, _y: False)
        speed = req.speed
        target = float(speed.target_vx) if speed is not None else float(OPEN_CRUISE_VX)
        v_max = float(speed.v_max_allowed) if speed is not None else min(geom.max_vx, target + 0.05)
        goal_d = None
        if req.goal:
            goal_d = math.hypot(req.goal[0] - req.x, req.goal[1] - req.y)
        hs, hm = self.adaptive_horizon(
            scene=req.scene,
            vx=req.vx,
            target_vx=target,
            front_near=req.front_near,
            global_preview_m=req.global_preview_m,
            goal_distance_m=goal_d,
            left_free=req.left_free,
            right_free=req.right_free,
        )
        v_roll = max(0.12, min(v_max, max(abs(req.vx), target, 0.18)))
        # Spatial floor may require more steps than hs at current v (OPEN ≥2m).
        steps_time = max(2, int(round(hs / ROLLOUT_DT)))
        steps_dist = max(2, int(math.ceil(hm / max(v_roll * ROLLOUT_DT, 1e-3))))
        steps = min(int(HARD_TIME_CAP_S / ROLLOUT_DT), max(steps_time, steps_dist))
        actual_h_s = steps * ROLLOUT_DT

        kappas = (-0.90, -0.55, -0.28, 0.0, 0.28, 0.55, 0.90)
        speeds = []
        for f in (0.55, 0.80, 1.00):
            vv = max(0.10, min(v_max, target * f if target > 0.08 else v_roll * f))
            if all(abs(vv - x) > 0.02 for x in speeds):
                speeds.append(vv)
        if v_max > target + 0.03:
            speeds.append(min(v_max, target + 0.06))

        gpath = list(req.global_path or [])
        cands: List[LocalCandidate] = []
        n = 0
        for kappa in kappas:
            for vx in speeds:
                n += 1
                w = float(vx) * float(kappa)
                if abs(w) > w_max + 1e-6:
                    # Speed-limit this sample rather than drop the curvature family
                    if abs(kappa) < 1e-6:
                        w = 0.0
                    else:
                        vx = w_max / abs(kappa)
                        w = math.copysign(w_max, kappa)
                        if vx < 0.08:
                            continue
                kind = KIND_FORWARD
                if kappa > 0.08:
                    kind = KIND_LEFT_ARC
                elif kappa < -0.08:
                    kind = KIND_RIGHT_ARC
                cid = f"{kind}_{n}"
                cand = self._rollout(
                    cid=cid,
                    kind=kind,
                    x=req.x,
                    y=req.y,
                    yaw=req.yaw,
                    vx=float(vx),
                    w=float(w),
                    kappa=float(kappa),
                    steps=steps,
                    collide=collide,
                    clearance_at=req.clearance_at,
                    geom=geom,
                    gpath=gpath,
                    goal=req.goal,
                    target_vx=target,
                    scene=req.scene,
                    reconnect_gate=bool(
                        not req.obstacle_passed
                        and req.future_preview is not None
                        and getattr(req.future_preview, "future_collision", False)
                    ),
                )
                if (
                    req.future_preview is not None
                    and getattr(req.future_preview, "future_global_blocked", False)
                    and kind == KIND_FORWARD
                ):
                    cand.valid = False
                    cand.reject_reason = "FUTURE_GLOBAL_BLOCKED"
                    cand.score = 1e6
                cands.append(cand)

        valid = [c for c in cands if c.valid]
        selected: Optional[LocalCandidate] = None
        if valid:
            valid.sort(key=lambda c: c.score)
            sc = str(req.scene or "OPEN").upper()
            fp = req.future_preview
            future_blocked = bool(
                fp is not None
                and getattr(fp, "future_global_blocked", False)
                and getattr(fp, "future_collision", False)
            )
            approach_early = bool(fp is not None and getattr(fp, "approach_active", False))
            side_commit = bool(req.side_commit_ready or str(req.avoidance_phase or "").upper() == "SIDE_COMMIT")
            fwd_ok = float(req.front_near) >= float(geom.front_stop_m) + 0.15
            if future_blocked or (approach_early and side_commit):
                fwd_ok = False
            if sc in ("OPEN", "OPEN_SPACE", "") and fwd_ok and not future_blocked:
                best = valid[0].score
                fwds = [c for c in valid if c.kind == KIND_FORWARD and c.score <= best + 1.15]
                selected = fwds[0] if fwds else valid[0]
            else:
                side_pref = req.preferred_side_hint
                if side_pref is None and fp is not None:
                    side_pref = getattr(fp, "preferred_side", None)
                arcs = [c for c in valid if c.kind in (KIND_LEFT_ARC, KIND_RIGHT_ARC)]
                if future_blocked and side_commit and arcs:
                    if side_pref == "LEFT":
                        lefts = [c for c in arcs if c.kind == KIND_LEFT_ARC]
                        selected = lefts[0] if lefts else arcs[0]
                    elif side_pref == "RIGHT":
                        rights = [c for c in arcs if c.kind == KIND_RIGHT_ARC]
                        selected = rights[0] if rights else arcs[0]
                    else:
                        selected = arcs[0]
                else:
                    selected = valid[0]
            for c in cands:
                c.selected = c is selected
        else:
            for c in cands:
                c.selected = False

        if selected is None:
            return None

        self._revision += 1
        poses = list(selected.poses)
        actual_m = selected.distance_m
        plan = LocalPlan(
            plan_id=self._next_id(),
            revision=self._revision,
            timestamp=now,
            generated_at=now,
            expires_at=now + max(PLANNING_PERIOD_S * 3.0, MAX_STALE_AGE_S),
            horizon_s=actual_h_s,
            horizon_m=actual_m,
            status=STATUS_CREATED,
            selected_candidate_id=selected.candidate_id,
            poses=poses,
            curvature=[selected.kappa] * max(1, len(poses)),
            speed_profile=[selected.vx] * max(1, len(poses)),
            min_clearance=selected.min_clearance,
            kinematic_valid=bool(selected.kinematic_valid and not selected.collision),
            speed_target=target,
            authority=req.authority or AUTH_ROLLING,
            candidates=cands,
            note="rolling local plan",
        )
        return plan

    def _rollout(
        self,
        *,
        cid: str,
        kind: str,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        w: float,
        kappa: float,
        steps: int,
        collide: CollideFn,
        clearance_at: Optional[ClearanceFn],
        geom: VehicleGeometry,
        gpath: List[Pt],
        goal: Optional[Pt],
        target_vx: float,
        scene: str,
        reconnect_gate: bool = False,
    ) -> LocalCandidate:
        poses: List[Pose] = [{"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5)}]
        cx, cy, cyaw = x, y, yaw
        dt = ROLLOUT_DT
        for _ in range(steps):
            cx += vx * math.cos(cyaw) * dt
            cy += vx * math.sin(cyaw) * dt
            cyaw = _wrap(cyaw + w * dt)
            poses.append({"x": round(cx, 4), "y": round(cy, 4), "yaw": round(cyaw, 5)})
        dist = _polyline_len(poses)
        end = poses[-1]
        cand = LocalCandidate(
            candidate_id=cid,
            kind=kind,
            valid=True,
            vx=vx,
            w=w,
            kappa=kappa,
            poses=poses,
            distance_m=dist,
            duration_s=steps * dt,
            endpoint=end,
            yaw_end=float(end["yaw"]),
        )
        # Hard kinematic: |w| already clipped by caller
        if abs(w) > min(W_MAX_CONTROL, geom.max_w) + 1e-3:
            cand.valid = False
            cand.kinematic_valid = False
            cand.reject_reason = "OMEGA_LIMIT"
            cand.score = 1e6
            return cand

        hit = trajectory_collision(poses, collide, geom, margin_m=0.0)
        if hit.collision:
            cand.valid = False
            cand.collision = True
            cand.reject_reason = "FOOTPRINT_COLLISION"
            cand.score = 1e6
            cand.cost_breakdown = {"collision_cost": 700.0, "total": 700.0}
            return cand

        # P0-D: global reconnect gate — block forward while obstacle not passed
        if kind == KIND_FORWARD and reconnect_gate:
            cand.valid = False
            cand.reject_reason = "FUTURE_GLOBAL_BLOCKED"
            cand.score = 1e6
            return cand

        clrs: List[float] = []
        if clearance_at is not None:
            for p in poses[::2]:
                try:
                    clrs.append(float(clearance_at(p["x"], p["y"])))
                except Exception:
                    pass
        if clrs:
            cand.min_clearance = min(clrs)
            margin = float(getattr(geom, "safety_margin_m", 0.08) or 0.08)
            if cand.min_clearance < margin:
                cand.valid = False
                cand.reject_reason = "CLEARANCE_TOO_LOW"
                cand.score = 1e6
                return cand

        # Costs — collision already hard-invalid. Do not use huge scores as a substitute.
        s0 = 0.0
        s1 = 0.0
        lat_mean = 0.0
        if gpath and len(gpath) >= 2:
            p0 = project_pose_to_path(x, y, yaw, gpath)
            p1 = project_pose_to_path(end["x"], end["y"], end["yaw"], gpath)
            s0, s1 = p0.s, p1.s
            nlat = 0
            acc = 0.0
            for p in poses[::3]:
                pp = project_pose_to_path(p["x"], p["y"], p.get("yaw") or 0.0, gpath)
                acc += abs(pp.lateral_m)
                nlat += 1
            lat_mean = acc / max(1, nlat)
            cand.heading_error = float(p1.heading_err)
            cand.reconnect_m = abs(float(p1.lateral_m))
        cand.progress_m = max(0.0, s1 - s0)
        cand.global_deviation_m = lat_mean
        # Future reconnect = lateral at endpoint (not "must capture in 0.3m")
        if goal:
            d0 = math.hypot(goal[0] - x, goal[1] - y)
            d1 = math.hypot(goal[0] - end["x"], goal[1] - end["y"])
            goal_gain = max(0.0, d0 - d1)
        else:
            goal_gain = cand.progress_m

        future_clr = cand.min_clearance if cand.min_clearance is not None else 1.0
        # Simple recoverability: later-half clearance + not heading into a wall-ish dead end
        later = clrs[len(clrs) // 2 :] if clrs else []
        future_clr2 = min(later) if later else future_clr
        cand.recoverability = max(0.0, min(1.0, future_clr2 / 1.2))

        progress_cost = -3.4 * (cand.progress_m + 0.35 * goal_gain)
        global_cost = 2.2 * lat_mean
        heading_cost = 1.4 * abs(cand.heading_error)
        clr_cost = 0.0 if cand.min_clearance is None else 1.8 * max(0.0, 0.55 - cand.min_clearance)
        if cand.min_clearance is not None and cand.min_clearance < 0.45:
            clr_cost += 2.5 * max(0.0, 0.45 - cand.min_clearance) ** 2
        curv_cost = 0.55 * abs(kappa)
        speed_cost = 0.90 * abs(vx - target_vx)
        reconnect_mult = 3.6 if reconnect_gate else 1.0
        reconnect_cost = 1.1 * cand.reconnect_m * reconnect_mult
        recover_cost = 2.0 * (1.0 - cand.recoverability)
        total = (
            progress_cost
            + global_cost
            + heading_cost
            + clr_cost
            + curv_cost
            + speed_cost
            + reconnect_cost
            + recover_cost
        )
        cand.score = float(total)
        cand.cost_breakdown = {
            "progress_cost": progress_cost,
            "global_deviation": global_cost,
            "heading_error": heading_cost,
            "clearance_cost": clr_cost,
            "curvature_cost": curv_cost,
            "speed_cost": speed_cost,
            "reconnect_cost": reconnect_cost,
            "recoverability_cost": recover_cost,
            "collision_cost": 0.0,
            "total": total,
        }
        cand.kinematic_valid = True
        return cand

    def ui_layer(self) -> Dict[str, Any]:
        """Packaging for collect_local_candidates / map overlay."""
        plan = self.last_plan
        if plan is None:
            return {
                "count": 0,
                "valid_count": 0,
                "max_distance_m": 0.0,
                "mean_distance_m": 0.0,
                "items": [],
                "selected_candidate": "NONE",
                "source": "ROLLING_LOCAL_PLANNER",
            }
        items = [c.to_dict() for c in plan.candidates]
        dists = [float(it.get("distance_m") or 0.0) for it in items]
        return {
            "count": len(items),
            "valid_count": sum(1 for c in plan.candidates if c.valid),
            "max_distance_m": round(max(dists) if dists else 0.0, 3),
            "mean_distance_m": round(sum(dists) / len(dists), 3) if dists else 0.0,
            "items": items,
            "selected_candidate": plan.selected_candidate_id,
            "source": "ROLLING_LOCAL_PLANNER",
            "plan_id": plan.plan_id,
            "horizon_s": plan.horizon_s,
            "horizon_m": plan.horizon_m,
        }
