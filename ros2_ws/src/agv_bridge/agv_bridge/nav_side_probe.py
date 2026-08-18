"""P0-D.1 — Multi-kappa side corridor probe (virtual evaluation, not physical swing).

Evaluates LEFT/RIGHT swept footprints over full preview horizon with multiple curvatures.
Does NOT emit cmd_vel. Does NOT bypass 3C/3E commitment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import trajectory_collision
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_kinematic import W_MAX_CONTROL, control_limits
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

PROBE_KAPPA_MAGS = (0.25, 0.45, 0.65, 0.85)
COMMIT_CONFIDENCE_THRESHOLD = 0.55
PREFERRED_CLEARANCE_M = 0.55


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


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
    poses: List[Dict[str, float]] = [{"x": x, "y": y, "yaw": yaw, "s": 0.0}]
    cx, cy, cyaw = x, y, yaw
    dist = 0.0
    for _ in range(steps):
        nx = cx + vx * math.cos(cyaw) * dt
        ny = cy + vx * math.sin(cyaw) * dt
        dist += math.hypot(nx - cx, ny - cy)
        cyaw = _wrap(cyaw + w * dt)
        cx, cy = nx, ny
        poses.append({"x": cx, "y": cy, "yaw": cyaw, "s": round(dist, 3)})
        if dist >= horizon_m:
            break
    return poses


def clearance_cost(min_clearance: Optional[float], geom: VehicleGeometry) -> float:
    """Nonlinear wall-hugging cost — hard invalid handled separately."""
    margin = float(getattr(geom, "safety_margin_m", 0.08) or 0.08)
    if min_clearance is None:
        return 0.35
    c = float(min_clearance)
    if c < margin:
        return 10.0
    if c >= PREFERRED_CLEARANCE_M:
        return 0.0
    if c >= margin + 0.08:
        return 0.4 * (PREFERRED_CLEARANCE_M - c)
    return 1.5 + 3.0 * (margin + 0.08 - c)


@dataclass
class SideArcProbe:
    side: str
    kappa: float
    valid: bool
    hard_invalid: bool
    collision: bool
    future_blocked: bool
    min_clearance: Optional[float] = None
    first_block_distance_m: Optional[float] = None
    reconnect_m: Optional[float] = None
    progress_m: float = 0.0
    soft_cost: float = 0.0
    confidence: float = 0.0
    reject_reason: Optional[str] = None
    poses: List[Dict[str, float]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "side": self.side,
            "kappa": round(self.kappa, 3),
            "valid": self.valid,
            "hard_invalid": self.hard_invalid,
            "collision": self.collision,
            "future_blocked": self.future_blocked,
            "min_clearance": None if self.min_clearance is None else round(self.min_clearance, 3),
            "first_block_distance_m": self.first_block_distance_m,
            "reconnect_m": None if self.reconnect_m is None else round(self.reconnect_m, 3),
            "progress_m": round(self.progress_m, 3),
            "soft_cost": round(self.soft_cost, 3),
            "confidence": round(self.confidence, 3),
            "reject_reason": self.reject_reason,
            "pose_count": len(self.poses),
        }


@dataclass
class SideProbeResult:
    probe_active: bool = False
    left_valid: bool = False
    right_valid: bool = False
    left_confidence: float = 0.0
    right_confidence: float = 0.0
    left_min_clearance: Optional[float] = None
    right_min_clearance: Optional[float] = None
    left_future_blocked: bool = False
    right_future_blocked: bool = False
    preferred_side: Optional[str] = None
    commit_ready: bool = False
    committed_side_hint: Optional[str] = None
    left_arcs: List[SideArcProbe] = field(default_factory=list)
    right_arcs: List[SideArcProbe] = field(default_factory=list)
    best_left: Optional[SideArcProbe] = None
    best_right: Optional[SideArcProbe] = None
    oscillation_suspected: bool = False
    wall_hugging_suspected: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "probe_active": self.probe_active,
            "left_valid": self.left_valid,
            "right_valid": self.right_valid,
            "left_confidence": round(self.left_confidence, 3),
            "right_confidence": round(self.right_confidence, 3),
            "probe_confidence_left": round(self.left_confidence, 3),
            "probe_confidence_right": round(self.right_confidence, 3),
            "left_min_clearance": None if self.left_min_clearance is None else round(self.left_min_clearance, 3),
            "right_min_clearance": None if self.right_min_clearance is None else round(self.right_min_clearance, 3),
            "left_future_blocked": self.left_future_blocked,
            "right_future_blocked": self.right_future_blocked,
            "preferred_side": self.preferred_side,
            "commit_ready": self.commit_ready,
            "committed_side_hint": self.committed_side_hint,
            "best_left_kappa": None if self.best_left is None else round(self.best_left.kappa, 3),
            "best_right_kappa": None if self.best_right is None else round(self.best_right.kappa, 3),
            "left_arcs": [a.to_dict() for a in self.left_arcs[:8]],
            "right_arcs": [a.to_dict() for a in self.right_arcs[:8]],
            "oscillation_suspected": self.oscillation_suspected,
            "wall_hugging_suspected": self.wall_hugging_suspected,
            "source": "SIDE_PROBE",
            "controls_vehicle": False,
        }


def _probe_one_arc(
    *,
    side: str,
    kappa: float,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    horizon_m: float,
    collide: CollideFn,
    clearance_at: Optional[ClearanceFn],
    geom: VehicleGeometry,
    gpath: Optional[Sequence[Pt]],
) -> SideArcProbe:
    poses = _rollout_arc(x, y, yaw, vx=vx, kappa=kappa, horizon_m=horizon_m)
    arc = SideArcProbe(side=side, kappa=kappa, valid=True, hard_invalid=False, collision=False, future_blocked=False, poses=poses)
    arc.progress_m = float(poses[-1].get("s") or 0.0) if poses else 0.0

    hit = trajectory_collision(poses, collide, geom, margin_m=0.0)
    if hit.collision:
        arc.valid = False
        arc.hard_invalid = True
        arc.collision = True
        arc.future_blocked = True
        arc.reject_reason = "FOOTPRINT_COLLISION"
        fi = max(0, min(int(hit.first_index), len(poses) - 1))
        arc.first_block_distance_m = float(poses[fi].get("s") or 0.0)
        return arc

    clrs: List[float] = []
    block_dist: Optional[float] = None
    margin = float(getattr(geom, "safety_margin_m", 0.08) or 0.08)
    if clearance_at is not None:
        for p in poses:
            try:
                c = float(clearance_at(p["x"], p["y"]))
                clrs.append(c)
                if c < margin and block_dist is None:
                    block_dist = float(p.get("s") or 0.0)
            except Exception:
                pass
    if clrs:
        arc.min_clearance = min(clrs)
        if arc.min_clearance < margin:
            arc.valid = False
            arc.hard_invalid = True
            arc.future_blocked = True
            arc.reject_reason = "CLEARANCE_HARD"
            arc.first_block_distance_m = block_dist
            return arc
        if block_dist is not None and block_dist < horizon_m * 0.85:
            arc.future_blocked = True

    if gpath and len(gpath) >= 2 and poses:
        try:
            end = poses[-1]
            pp = project_pose_to_path(end["x"], end["y"], end["yaw"], gpath)
            arc.reconnect_m = abs(float(pp.lateral_m))
        except Exception:
            pass

    clr_c = clearance_cost(arc.min_clearance, geom)
    rec_c = 0.0 if arc.reconnect_m is None else 0.9 * arc.reconnect_m
    blk_c = 2.5 if arc.future_blocked else 0.0
    arc.soft_cost = clr_c + rec_c + blk_c

    c_score = 0.0 if arc.min_clearance is None else min(1.0, arc.min_clearance / 1.2)
    r_score = 0.0 if arc.reconnect_m is None else max(0.0, 1.0 - arc.reconnect_m / 3.5)
    p_score = min(1.0, arc.progress_m / max(horizon_m, 0.5))
    arc.confidence = max(0.0, min(1.0, 0.40 * c_score + 0.30 * r_score + 0.20 * p_score + 0.10 * (0.0 if arc.future_blocked else 1.0)))
    return arc


def run_side_probe(
    *,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    horizon_m: float,
    collide: CollideFn,
    clearance_at: Optional[ClearanceFn] = None,
    global_path: Optional[Sequence[Pt]] = None,
    geom: Optional[VehicleGeometry] = None,
    probe_active: bool = True,
    side_history: Optional[List[str]] = None,
) -> SideProbeResult:
    """Virtual multi-kappa LEFT/RIGHT evaluation over full preview horizon."""
    g = geom or get_vehicle_geometry()
    lim = control_limits(g)
    w_max = float(lim["w_max"])
    v = max(0.12, min(float(g.max_vx), max(abs(vx), 0.15)))
    hm = max(1.5, float(horizon_m))

    res = SideProbeResult(probe_active=bool(probe_active))
    if not probe_active:
        return res

    left_arcs: List[SideArcProbe] = []
    right_arcs: List[SideArcProbe] = []
    for km in PROBE_KAPPA_MAGS:
        k_l = min(km, w_max / max(v, 0.08))
        k_r = -k_l
        left_arcs.append(
            _probe_one_arc(
                side="LEFT", kappa=k_l, x=x, y=y, yaw=yaw, vx=v, horizon_m=hm,
                collide=collide, clearance_at=clearance_at, geom=g, gpath=global_path,
            )
        )
        right_arcs.append(
            _probe_one_arc(
                side="RIGHT", kappa=k_r, x=x, y=y, yaw=yaw, vx=v, horizon_m=hm,
                collide=collide, clearance_at=clearance_at, geom=g, gpath=global_path,
            )
        )

    res.left_arcs = left_arcs
    res.right_arcs = right_arcs
    valid_left = [a for a in left_arcs if a.valid]
    valid_right = [a for a in right_arcs if a.valid]
    res.left_valid = len(valid_left) > 0
    res.right_valid = len(valid_right) > 0

    if valid_left:
        res.best_left = min(valid_left, key=lambda a: a.soft_cost)
        res.left_confidence = res.best_left.confidence
        res.left_min_clearance = res.best_left.min_clearance
        res.left_future_blocked = res.best_left.future_blocked
    if valid_right:
        res.best_right = min(valid_right, key=lambda a: a.soft_cost)
        res.right_confidence = res.best_right.confidence
        res.right_min_clearance = res.best_right.min_clearance
        res.right_future_blocked = res.best_right.future_blocked

    if res.left_valid and res.right_valid and res.best_left and res.best_right:
        l_score = res.left_confidence - 0.15 * res.best_left.soft_cost
        r_score = res.right_confidence - 0.15 * res.best_right.soft_cost
        if l_score > r_score + 0.08:
            res.preferred_side = "LEFT"
        elif r_score > l_score + 0.08:
            res.preferred_side = "RIGHT"
        else:
            l_clr = float(res.best_left.min_clearance or 0.0)
            r_clr = float(res.best_right.min_clearance or 0.0)
            if l_clr > r_clr + 0.06:
                res.preferred_side = "LEFT"
            elif r_clr > l_clr + 0.06:
                res.preferred_side = "RIGHT"
    elif res.left_valid:
        res.preferred_side = "LEFT"
    elif res.right_valid:
        res.preferred_side = "RIGHT"

    best_conf = max(res.left_confidence, res.right_confidence)
    res.commit_ready = best_conf >= COMMIT_CONFIDENCE_THRESHOLD and res.preferred_side is not None
    if res.commit_ready:
        res.committed_side_hint = res.preferred_side

    if side_history and len(side_history) >= 4:
        flips = sum(1 for i in range(1, len(side_history)) if side_history[i] != side_history[i - 1])
        res.oscillation_suspected = flips >= 3

    min_clr = min(
        [c for c in (res.left_min_clearance, res.right_min_clearance) if c is not None],
        default=None,
    )
    if min_clr is not None and min_clr < PREFERRED_CLEARANCE_M * 0.85:
        res.wall_hugging_suspected = True

    return res
