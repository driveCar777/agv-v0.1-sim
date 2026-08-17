"""Local Maneuver Selection — short-horizon LEFT/RIGHT/FORWARD comparison.

Extends ManeuverFSM; does not replace global A* or MPPI. Cost scales align with
DiffDriveMppi._score (collision≈700, global_path_cost≈5*gdev, w_cost≈2.4*|w|).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.maneuver import find_best_path_capture, wrap_pi
from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

ROLLOUT_DT = 0.1
ROLLOUT_STEPS = 15  # 1.5s short-horizon compare
NOMINAL_VX = 0.18
SIDE_VX = 0.22
WZ_NOM = 0.35  # side swerve (still below wz_max 0.42)
WZ_MAX = 0.42

COLLISION_HARD = 700.0
CLR_HARD = 0.18  # below ~local footprint; soft cost handles 0.18–0.85
CLR_SOFT = 18.0
CAPTURE_SOFT = 8.0
HEADING_SOFT = 6.0
PROGRESS_SOFT = 12.0
TURN_SOFT = 2.4
DEV_SOFT = 5.0
DURATION_SOFT = 1.5
REVERSE_SOFT = 20.0
SWITCH_SOFT = 8.0

HYSTERESIS_RATIO = 0.12
HYSTERESIS_ABS = 6.0
MIN_HOLD_S = 0.85
COMPARE_PERIOD_S = 0.18
MAX_SIDE_ATTEMPTS = 3

QUALITY_GOOD = "GOOD"
QUALITY_WARNING = "WARNING"
QUALITY_POOR = "POOR"
QUALITY_BLOCKED = "BLOCKED"

DEC_FORWARD = "FORWARD"
DEC_LEFT = "LEFT"
DEC_RIGHT = "RIGHT"
DEC_ALIGN = "ALIGN"
DEC_REPOSITION = "REPOSITION"
DEC_REVERSE = "REVERSE"
DEC_WAIT = "WAIT"
DEC_REPLAN = "REPLAN"


def _xy_of(p: Any) -> Optional[Pt]:
    if isinstance(p, dict):
        return float(p.get("x") or 0.0), float(p.get("y") or 0.0)
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return float(p[0]), float(p[1])
    return None


def _endpoint_distance(path: Sequence[Any], ex: float = 0.0, ey: float = 0.0) -> float:
    if path:
        a = _xy_of(path[0])
        b = _xy_of(path[-1]) if len(path) >= 2 else a
        if a and b:
            return math.hypot(b[0] - a[0], b[1] - a[1])
    return math.hypot(float(ex or 0.0), float(ey or 0.0))


def _survived_distance(path: Sequence[Any]) -> float:
    s = 0.0
    prev: Optional[Pt] = None
    for p in path or []:
        xy = _xy_of(p)
        if xy is None:
            continue
        if prev is not None:
            s += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        prev = xy
    return s


def selector_horizon_s() -> float:
    return float(ROLLOUT_STEPS) * float(ROLLOUT_DT)


def selector_nominal_distance_m(vx: Optional[float] = None) -> float:
    v = float(NOMINAL_VX if vx is None else vx)
    return abs(v) * selector_horizon_s()


@dataclass
class LocalManeuverCandidate:
    type: str
    feasible: bool
    reason: str = "OK"
    vx: float = 0.0
    w: float = 0.0
    duration: float = 0.0
    endpoint_x: float = 0.0
    endpoint_y: float = 0.0
    endpoint_theta: float = 0.0
    collision: bool = False
    first_collision_t: Optional[float] = None
    first_collision_x: Optional[float] = None
    first_collision_y: Optional[float] = None
    min_clearance: float = 0.0
    mean_clearance: float = 0.0
    path_clearance: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    path_capture_distance: float = 0.0
    path_capture_heading_error: float = 0.0
    path_capture_available: bool = False
    path_progress_gain: float = 0.0
    turn_cost: float = 0.0
    deviation_cost: float = 0.0
    obstacle_cost: float = 0.0
    capture_cost: float = 0.0
    heading_cost: float = 0.0
    progress_cost: float = 0.0
    clearance_cost: float = 0.0
    switch_cost: float = 0.0
    total_cost: float = 1e6
    confidence: float = 0.0
    route_quality: str = QUALITY_BLOCKED
    selected: bool = False
    path: List[Dict[str, float]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type,
            "feasible": self.feasible,
            "reason": self.reason,
            "vx": round(self.vx, 4),
            "w": round(self.w, 4),
            "duration": round(self.duration, 3),
            "duration_s": round(self.duration, 3),
            "distance_m": round(_endpoint_distance(self.path, self.endpoint_x, self.endpoint_y), 3),
            "planned_distance_m": round(abs(float(self.vx or 0.0)) * float(self.duration or 0.0), 3),
            "actual_survived_distance_m": round(_survived_distance(self.path), 3),
            "first_collision_t": self.first_collision_t,
            "first_invalid_reason": None if self.feasible else (self.reason or "UNKNOWN"),
            "endpoint": {
                "x": round(self.endpoint_x, 3),
                "y": round(self.endpoint_y, 3),
                "theta": round(self.endpoint_theta, 4),
            },
            "collision": self.collision,
            "first_collision": None
            if self.first_collision_t is None
            else {"t": self.first_collision_t, "x": self.first_collision_x, "y": self.first_collision_y},
            "min_clearance": round(self.min_clearance, 3),
            "mean_clearance": round(self.mean_clearance, 3),
            "path_capture_distance": round(self.path_capture_distance, 3),
            "path_capture_heading_error": round(self.path_capture_heading_error, 4),
            "path_capture_available": self.path_capture_available,
            "path_progress_gain": round(self.path_progress_gain, 3),
            "heading_error": round(self.heading_error, 4),
            "lateral_error": round(self.lateral_error, 3),
            "total_cost": round(self.total_cost, 2),
            "route_quality": self.route_quality,
            "selected": self.selected,
            "confidence": round(self.confidence, 3),
            "cost_breakdown": {
                "clearance": round(self.clearance_cost, 2),
                "capture": round(self.capture_cost, 2),
                "heading": round(self.heading_cost, 2),
                "progress": round(self.progress_cost, 2),
                "turn": round(self.turn_cost, 2),
                "deviation": round(self.deviation_cost, 2),
                "obstacle": round(self.obstacle_cost, 2),
                "switch": round(self.switch_cost, 2),
                "total": round(self.total_cost, 2),
            },
            "path": self.path[:16],
        }


def _footprint_points(x: float, y: float, yaw: float, geom=DEFAULT_GEOM) -> List[Pt]:
    """Probe/rollout body samples — sizes from VehicleGeometry ONLY (no second table).

    Uses the pre-P0-A 0.45 shrink set so Probe / Local / 3F recovery semantics
    stay unchanged. Full rectangle polygon truth lives in nav_footprint
    (swept corridor / trajectory_collision / rotation sweep). Wiring Probe to
    full-polygon narrow-phase is deferred (P0-C/E) — not this phase.
    """
    hl, hw = 0.45 * float(geom.length), 0.45 * float(geom.width)
    local = [
        (hl, 0.0),
        (-hl, 0.0),
        (0.0, hw),
        (0.0, -hw),
        (hl * 0.7, hw * 0.7),
        (hl * 0.7, -hw * 0.7),
        (-hl * 0.7, hw * 0.7),
        (-hl * 0.7, -hw * 0.7),
        (0.0, 0.0),
    ]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]


def _collide_body(x: float, y: float, yaw: float, collide: CollideFn, geom=DEFAULT_GEOM) -> bool:
    return any(collide(px, py) for px, py in _footprint_points(x, y, yaw, geom))


def _in_map(x: float, y: float, map_bounds: Optional[Tuple[float, float, float, float]]) -> bool:
    if map_bounds is None:
        return True
    xmin, ymin, xmax, ymax = map_bounds
    return xmin <= x <= xmax and ymin <= y <= ymax


def rollout_candidate(
    *,
    ctype: str,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    w: float,
    path: Sequence[Pt],
    goal: Optional[Pt],
    collide: CollideFn,
    clearance_at: Optional[ClearanceFn],
    front_near: float,
    prev_w: float = 0.0,
    map_bounds: Optional[Tuple[float, float, float, float]] = None,
    s_current: float = 0.0,
    require_capture: bool = True,
) -> LocalManeuverCandidate:
    cand = LocalManeuverCandidate(type=ctype, feasible=True, vx=vx, w=w, duration=ROLLOUT_STEPS * ROLLOUT_DT)
    if ctype == DEC_REVERSE and vx >= 0:
        vx = -0.10
        cand.vx = vx
    if ctype in (DEC_LEFT, DEC_RIGHT, DEC_FORWARD) and vx < 0:
        vx = abs(vx) if vx != 0 else NOMINAL_VX
        cand.vx = vx
    if ctype == DEC_LEFT and w <= 0:
        w = WZ_NOM
        cand.w = w
    if ctype == DEC_RIGHT and w >= 0:
        w = -WZ_NOM
        cand.w = w

    cx, cy, cyaw = x, y, yaw
    clrs: List[float] = []
    gdev_acc = 0.0
    gdev_n = 0
    # STEP 3F: store yaw so PhysicalTrajectoryCorridor / Probe share ONE rollout
    traj: List[Dict[str, float]] = [{"x": cx, "y": cy, "yaw": round(cyaw, 4)}]

    for i in range(ROLLOUT_STEPS):
        cx += vx * math.cos(cyaw) * ROLLOUT_DT
        cy += vx * math.sin(cyaw) * ROLLOUT_DT
        cyaw = wrap_pi(cyaw + w * ROLLOUT_DT)
        traj.append({"x": round(cx, 3), "y": round(cy, 3), "yaw": round(cyaw, 4)})
        if not _in_map(cx, cy, map_bounds):
            cand.feasible = False
            cand.reason = "OUT_OF_MAP"
            cand.total_cost = COLLISION_HARD
            cand.route_quality = QUALITY_BLOCKED
            cand.path = traj
            return cand
        if _collide_body(cx, cy, cyaw, collide):
            cand.feasible = False
            cand.collision = True
            cand.reason = "COLLISION"
            cand.first_collision_t = round((i + 1) * ROLLOUT_DT, 3)
            cand.first_collision_x = round(cx, 3)
            cand.first_collision_y = round(cy, 3)
            cand.total_cost = COLLISION_HARD
            cand.route_quality = QUALITY_BLOCKED
            cand.path = traj
            return cand
        if clearance_at:
            clrs.append(float(clearance_at(cx, cy)))
        if path:
            step = max(1, len(path) // 24)
            gpts = path[::step]
            gdev_acc += min(math.hypot(cx - g[0], cy - g[1]) for g in gpts)
            gdev_n += 1

    cand.endpoint_x, cand.endpoint_y, cand.endpoint_theta = cx, cy, cyaw
    cand.path = traj
    cand.min_clearance = min(clrs) if clrs else (front_near if ctype == DEC_FORWARD else 1.0)
    cand.mean_clearance = (sum(clrs) / len(clrs)) if clrs else cand.min_clearance

    cap = find_best_path_capture(cx, cy, cyaw, path, clearance_at=clearance_at)
    cand.path_capture_available = bool(cap.available)
    cand.path_capture_distance = float(cap.distance) if cap.available else 99.0
    cand.path_capture_heading_error = float(cap.heading_error) if cap.available else math.pi
    cand.path_clearance = float(cap.clearance) if cap.available else 0.0
    if require_capture and ctype in (DEC_LEFT, DEC_RIGHT, DEC_FORWARD) and not cap.available:
        cand.feasible = False
        cand.reason = "NO_PATH_CAPTURE"
        cand.total_cost = COLLISION_HARD * 0.5
        cand.route_quality = QUALITY_BLOCKED
        return cand
    if require_capture and ctype in (DEC_LEFT, DEC_RIGHT) and cand.path_capture_distance > 4.5:
        cand.feasible = False
        cand.reason = "NO_PATH_CAPTURE"
        cand.total_cost = 400.0
        cand.route_quality = QUALITY_BLOCKED
        return cand

    prog = project_pose_to_path(cx, cy, cyaw, path) if path else None
    s_end = float(prog.s) if prog else s_current
    cand.path_progress_gain = s_end - s_current
    cand.lateral_error = float(prog.lateral_m) if prog else 0.0
    cand.heading_error = float(prog.heading_err) if prog else 0.0

    if cand.min_clearance < CLR_HARD:
        cand.feasible = False
        cand.reason = "LOW_CLEARANCE"
        cand.total_cost = 500.0
        cand.route_quality = QUALITY_BLOCKED
        return cand

    cand.clearance_cost = CLR_SOFT * max(0.0, 0.85 - cand.min_clearance)
    cand.capture_cost = CAPTURE_SOFT * cand.path_capture_distance + 3.0 * abs(cand.path_capture_heading_error)
    cand.heading_cost = HEADING_SOFT * abs(cand.heading_error)
    if cand.path_progress_gain <= 0.02:
        cand.progress_cost = PROGRESS_SOFT * 0.55
    else:
        cand.progress_cost = max(0.0, PROGRESS_SOFT * (0.25 - cand.path_progress_gain))
    cand.turn_cost = TURN_SOFT * abs(w)
    gdev = (gdev_acc / max(1, gdev_n)) if gdev_n else 0.0
    cand.deviation_cost = DEV_SOFT * gdev
    if front_near < DEFAULT_GEOM.front_cost_m and vx > 0.04 and ctype == DEC_FORWARD:
        cand.obstacle_cost = 40.0 * (1.0 - front_near / max(DEFAULT_GEOM.front_cost_m, 1e-3))
    if vx < -0.05:
        cand.obstacle_cost += REVERSE_SOFT * abs(vx)
    if prev_w * w < 0 and abs(prev_w) > 0.05 and abs(w) > 0.05:
        cand.switch_cost = SWITCH_SOFT
    duration_cost = DURATION_SOFT * cand.duration

    cand.total_cost = (
        cand.clearance_cost
        + cand.capture_cost
        + cand.heading_cost
        + cand.progress_cost
        + cand.turn_cost
        + cand.deviation_cost
        + cand.obstacle_cost
        + cand.switch_cost
        + duration_cost
    )

    if not cand.feasible:
        cand.route_quality = QUALITY_BLOCKED
    elif cand.min_clearance >= 0.65 and cand.path_capture_distance <= 1.4 and abs(cand.heading_error) < 0.6:
        cand.route_quality = QUALITY_GOOD
    elif cand.min_clearance >= 0.45 and cand.path_capture_distance <= 2.5:
        cand.route_quality = QUALITY_WARNING
    else:
        cand.route_quality = QUALITY_POOR

    cand.confidence = max(0.05, min(0.99, 1.0 - cand.total_cost / 120.0))
    return cand


def _nominal_side_w(side: str, yaw: float, path: Sequence[Pt], x: float, y: float) -> float:
    cap = find_best_path_capture(x, y, yaw, path)
    if side == DEC_LEFT:
        target = wrap_pi((cap.heading if cap.available else yaw) + 0.55)
        err = wrap_pi(target - yaw)
        return max(0.12, min(WZ_NOM, 0.9 * abs(err) + 0.12))
    if side == DEC_RIGHT:
        target = wrap_pi((cap.heading if cap.available else yaw) - 0.55)
        err = wrap_pi(target - yaw)
        return -max(0.12, min(WZ_NOM, 0.9 * abs(err) + 0.12))
    return 0.0


@dataclass
class ManeuverComparisonResult:
    selected: str
    reason: str
    candidates: Dict[str, LocalManeuverCandidate]
    trigger: bool
    snapshot: Dict[str, Any] = field(default_factory=dict)

    def to_telemetry(self) -> Dict[str, Any]:
        rows = []
        for k in (DEC_FORWARD, DEC_LEFT, DEC_RIGHT, DEC_ALIGN, DEC_REPOSITION, DEC_REVERSE, DEC_WAIT):
            c = self.candidates.get(k)
            if not c:
                continue
            rows.append(
                {
                    "action": k,
                    "feasible": c.feasible,
                    "cost": round(c.total_cost, 1) if c.feasible or c.total_cost < 1e5 else None,
                    "clr": round(c.min_clearance, 2),
                    "capture": round(c.path_capture_distance, 2) if c.path_capture_available else None,
                    "progress": round(c.path_progress_gain, 2),
                    "heading": round(c.heading_error, 2),
                    "quality": c.route_quality,
                    "reason": c.reason,
                    "selected": c.selected,
                }
            )
        return {
            "selected": self.selected,
            "reason": self.reason,
            "trigger": self.trigger,
            "compare_called": (self.snapshot or {}).get("compare_called"),
            "compare_reason": (self.snapshot or {}).get("compare_reason"),
            "rows": rows,
            "candidates": {k: v.to_dict() for k, v in self.candidates.items()},
            "snapshot": self.snapshot,
            "left_path": (self.candidates[DEC_LEFT].path if DEC_LEFT in self.candidates else []),
            "right_path": (self.candidates[DEC_RIGHT].path if DEC_RIGHT in self.candidates else []),
            "forward_path": (self.candidates[DEC_FORWARD].path if DEC_FORWARD in self.candidates else []),
        }


class LocalManeuverSelector:
    def __init__(self) -> None:
        self.current: str = DEC_FORWARD
        self.current_cost: float = 0.0
        self.hold_until: float = 0.0
        self.last_compare_ts: float = 0.0
        self.last_result: Optional[ManeuverComparisonResult] = None
        self.side_attempts: int = 0
        self.obstacle_passed: bool = False
        self.prev_w: float = 0.0
        self.enter_front: float = 30.0
        self.enter_progress: float = 0.0
        self.history: List[Dict[str, Any]] = []
        self.events: List[Dict[str, Any]] = []
        self.regression_score: float = 0.0
        # P0-C.1 diagnostics only — does not change compare gates
        self.last_compare_called: bool = False
        self.last_compare_trigger: str = "NEVER"
        self.last_compare_reason: str = "NEVER"

    def reset(self) -> None:
        self.current = DEC_FORWARD
        self.current_cost = 0.0
        self.hold_until = 0.0
        self.last_compare_ts = 0.0
        self.last_result = None
        self.side_attempts = 0
        self.obstacle_passed = False
        self.prev_w = 0.0
        self.history = []
        self.events = []
        self.regression_score = 0.0
        self.last_compare_called = False
        self.last_compare_trigger = "NEVER"
        self.last_compare_reason = "NEVER"

    def _emit(self, event: str, **payload: Any) -> None:
        self.events.append({"ts": time.time(), "event": event, **payload})
        if len(self.events) > 40:
            self.events = self.events[-40:]

    def needs_compare(
        self,
        *,
        now: float,
        front_near: float,
        forward_feasible: bool,
        force: bool = False,
    ) -> bool:
        if force:
            return True
        if now - self.last_compare_ts < COMPARE_PERIOD_S and self.last_result is not None:
            return False
        if self.current in (DEC_LEFT, DEC_RIGHT):
            return True
        if not forward_feasible:
            return True
        if front_near < DEFAULT_GEOM.front_cost_m + 0.15:
            return True
        return False

    def explain_compare_reason(
        self,
        *,
        now: float,
        front_near: float,
        forward_feasible: bool,
        force: bool = False,
    ) -> str:
        """Diagnostic mirror of needs_compare + fallthrough. Does not change gates."""
        if force:
            return "FORCED"
        if now - self.last_compare_ts < COMPARE_PERIOD_S and self.last_result is not None:
            return "COMPARE_PERIOD"
        if self.current in (DEC_LEFT, DEC_RIGHT):
            return "SIDE_ACTIVE"
        if not forward_feasible:
            return "FORWARD_INFEASIBLE"
        if front_near < DEFAULT_GEOM.front_cost_m + 0.15:
            return "FRONT_NEAR"
        if self.last_result is None:
            return "NO_PREVIOUS_RESULT"
        return "NONE_OPEN_FORWARD"

    def _note_compare(self, called: bool, reason: str) -> None:
        self.last_compare_called = bool(called)
        self.last_compare_reason = str(reason or "UNKNOWN")
        self.last_compare_trigger = self.last_compare_reason

    def forensics_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        ts = time.time() if now is None else float(now)
        age = None if float(self.last_compare_ts or 0.0) <= 0.0 else round(ts - float(self.last_compare_ts), 3)
        return {
            "selector_horizon_s": round(selector_horizon_s(), 3),
            "selector_nominal_vx": NOMINAL_VX,
            "selector_side_vx": SIDE_VX,
            "selector_nominal_distance_m": round(selector_nominal_distance_m(), 3),
            "compare_called": self.last_compare_called,
            "compare_trigger": self.last_compare_trigger,
            "compare_reason": self.last_compare_reason,
            "last_compare_age_s": age,
            "current": self.current,
            "has_last_result": self.last_result is not None,
        }

    def compare(
        self,
        *,
        now: float,
        x: float,
        y: float,
        yaw: float,
        path: Sequence[Pt],
        goal: Optional[Pt],
        front_near: float,
        rear_near: float,
        collide: CollideFn,
        clearance_at: Optional[ClearanceFn],
        forward_feasible: bool,
        rotation_safe: bool,
        left_free: float,
        right_free: float,
        map_bounds: Optional[Tuple[float, float, float, float]] = None,
        dynamic_short: bool = False,
        force: bool = False,
        require_capture: bool = True,
        max_deviation_m: float = 1.2,
        path_follow_scale: float = 1.0,
        commitment_active: bool = False,
        committed_side: Optional[str] = None,
        side_switch_authorized: bool = False,
        commitment_hard_fail: bool = False,
        authorized_side: Optional[str] = None,
    ) -> ManeuverComparisonResult:
        if not self.needs_compare(now=now, front_near=front_near, forward_feasible=forward_feasible, force=force):
            reason = self.explain_compare_reason(
                now=now, front_near=front_near, forward_feasible=forward_feasible, force=force
            )
            if self.last_result is not None:
                self._note_compare(False, reason)
                try:
                    self.last_result.snapshot = dict(self.last_result.snapshot or {})
                    self.last_result.snapshot["compare_called"] = False
                    self.last_result.snapshot["compare_reason"] = reason
                except Exception:
                    pass
                return self.last_result
            self._note_compare(True, "NO_PREVIOUS_RESULT")
        else:
            self._note_compare(
                True,
                self.explain_compare_reason(
                    now=now, front_near=front_near, forward_feasible=forward_feasible, force=force
                ),
            )
        self.last_compare_ts = now
        # Soften capture gate during LOCAL_AVOID (policy require_capture_hard=False)
        capt = bool(require_capture)
        cap_dist_limit = 4.5 if capt else 7.5
        dev_soft_scale = max(0.2, float(path_follow_scale))

        s_cur = project_pose_to_path(x, y, yaw, path).s if path else 0.0
        cands: Dict[str, LocalManeuverCandidate] = {}

        cands[DEC_FORWARD] = rollout_candidate(
            ctype=DEC_FORWARD,
            x=x,
            y=y,
            yaw=yaw,
            vx=NOMINAL_VX,
            w=0.0,
            path=path,
            goal=goal,
            collide=collide,
            clearance_at=clearance_at,
            front_near=front_near,
            prev_w=self.prev_w,
            map_bounds=map_bounds,
            s_current=s_cur,
            require_capture=capt,
        )
        if front_near < DEFAULT_GEOM.front_stop_m:
            cands[DEC_FORWARD].feasible = False
            cands[DEC_FORWARD].reason = "COLLISION" if cands[DEC_FORWARD].collision else "LOW_CLEARANCE"
            cands[DEC_FORWARD].route_quality = QUALITY_BLOCKED
            cands[DEC_FORWARD].total_cost = max(cands[DEC_FORWARD].total_cost, 500.0)

        w_l = _nominal_side_w(DEC_LEFT, yaw, path, x, y)
        w_r = _nominal_side_w(DEC_RIGHT, yaw, path, x, y)
        cands[DEC_LEFT] = rollout_candidate(
            ctype=DEC_LEFT,
            x=x,
            y=y,
            yaw=yaw,
            vx=SIDE_VX,
            w=w_l,
            path=path,
            goal=goal,
            collide=collide,
            clearance_at=clearance_at,
            front_near=front_near,
            prev_w=self.prev_w,
            map_bounds=map_bounds,
            s_current=s_cur,
            require_capture=capt,
        )
        cands[DEC_RIGHT] = rollout_candidate(
            ctype=DEC_RIGHT,
            x=x,
            y=y,
            yaw=yaw,
            vx=SIDE_VX,
            w=w_r,
            path=path,
            goal=goal,
            collide=collide,
            clearance_at=clearance_at,
            front_near=front_near,
            prev_w=self.prev_w,
            map_bounds=map_bounds,
            s_current=s_cur,
            require_capture=capt,
        )
        # Close-range: primary arc may not clear before inflated obstacle — aggressive retry
        if front_near < 1.35 and (not cands[DEC_LEFT].feasible or not cands[DEC_RIGHT].feasible):
            if not cands[DEC_LEFT].feasible:
                alt = rollout_candidate(
                    ctype=DEC_LEFT,
                    x=x,
                    y=y,
                    yaw=yaw,
                    vx=0.10,
                    w=WZ_MAX,
                    path=path,
                    goal=goal,
                    collide=collide,
                    clearance_at=clearance_at,
                    front_near=front_near,
                    prev_w=self.prev_w,
                    map_bounds=map_bounds,
                    s_current=s_cur,
                    require_capture=capt,
                )
                if alt.feasible or alt.total_cost < cands[DEC_LEFT].total_cost:
                    cands[DEC_LEFT] = alt
            if not cands[DEC_RIGHT].feasible:
                alt = rollout_candidate(
                    ctype=DEC_RIGHT,
                    x=x,
                    y=y,
                    yaw=yaw,
                    vx=0.10,
                    w=-WZ_MAX,
                    path=path,
                    goal=goal,
                    collide=collide,
                    clearance_at=clearance_at,
                    front_near=front_near,
                    prev_w=self.prev_w,
                    map_bounds=map_bounds,
                    s_current=s_cur,
                    require_capture=capt,
                )
                if alt.feasible or alt.total_cost < cands[DEC_RIGHT].total_cost:
                    cands[DEC_RIGHT] = alt

        align = LocalManeuverCandidate(
            type=DEC_ALIGN,
            feasible=rotation_safe,
            reason="OK" if rotation_safe else "SIL_BLOCKED",
            vx=0.0,
            w=0.25,
            duration=1.0,
            min_clearance=min(left_free, right_free, front_near) * 0.5,
            path_capture_available=True,
            path_capture_distance=0.5,
            total_cost=55.0 if rotation_safe else 900.0,
            route_quality=QUALITY_GOOD if rotation_safe else QUALITY_BLOCKED,
        )
        repos = LocalManeuverCandidate(
            type=DEC_REPOSITION,
            feasible=left_free > 0.8 or right_free > 0.8 or rear_near > 0.8,
            reason="OK",
            vx=0.06,
            w=0.2,
            duration=1.2,
            min_clearance=max(left_free, right_free) * 0.4,
            path_capture_available=True,
            path_capture_distance=1.2,
            total_cost=70.0,
            route_quality=QUALITY_WARNING,
        )
        wait = LocalManeuverCandidate(
            type=DEC_WAIT,
            feasible=bool(dynamic_short and front_near < DEFAULT_GEOM.front_cost_m),
            reason="DYNAMIC_OBSTACLE" if dynamic_short else "N/A",
            vx=0.0,
            w=0.0,
            duration=1.0,
            min_clearance=front_near,
            total_cost=48.0 if dynamic_short else 950.0,
            route_quality=QUALITY_WARNING if dynamic_short else QUALITY_BLOCKED,
        )
        rev = LocalManeuverCandidate(
            type=DEC_REVERSE,
            feasible=rear_near > DEFAULT_GEOM.rear_stop_m + 0.15,
            reason="OK" if rear_near > DEFAULT_GEOM.rear_stop_m + 0.15 else "LOW_CLEARANCE",
            vx=-0.10,
            w=0.0,
            duration=1.0,
            min_clearance=rear_near,
            total_cost=94.0 if rear_near > DEFAULT_GEOM.rear_stop_m + 0.15 else 900.0,
            route_quality=QUALITY_POOR if rear_near > DEFAULT_GEOM.rear_stop_m + 0.15 else QUALITY_BLOCKED,
            path_capture_available=False,
        )
        cands[DEC_ALIGN] = align
        cands[DEC_REPOSITION] = repos
        cands[DEC_WAIT] = wait
        cands[DEC_REVERSE] = rev

        # Soft free-space bias (not sole selector): reward genuinely freer side
        # Scale path-alignment soft costs by policy path_follow_scale during AVOID
        for k in (DEC_LEFT, DEC_RIGHT, DEC_FORWARD):
            if k in cands and cands[k].feasible:
                cands[k].deviation_cost *= dev_soft_scale
                cands[k].total_cost = (
                    cands[k].clearance_cost
                    + cands[k].capture_cost
                    + cands[k].heading_cost
                    + cands[k].progress_cost
                    + cands[k].turn_cost
                    + cands[k].deviation_cost
                    + cands[k].obstacle_cost
                    + cands[k].switch_cost
                    + DURATION_SOFT * cands[k].duration
                )
                # Soft corridor: penalize endpoint lateral beyond max_deviation
                if abs(cands[k].lateral_error) > max_deviation_m:
                    over = abs(cands[k].lateral_error) - max_deviation_m
                    cands[k].total_cost += 12.0 * over
                    if over > 0.6 and capt:
                        cands[k].feasible = False
                        cands[k].reason = "LOCAL_DEVIATION_EXCEEDED"
                        cands[k].route_quality = QUALITY_BLOCKED

        if cands[DEC_LEFT].feasible:
            cands[DEC_LEFT].total_cost -= 4.0 * min(1.8, max(0.0, left_free - right_free))
            cands[DEC_LEFT].total_cost = max(1.0, cands[DEC_LEFT].total_cost)
        if cands[DEC_RIGHT].feasible:
            cands[DEC_RIGHT].total_cost -= 4.0 * min(1.8, max(0.0, right_free - left_free))
            cands[DEC_RIGHT].total_cost = max(1.0, cands[DEC_RIGHT].total_cost)

        # Soft fallback: both rollouts hard-fail but one side is clearly freer
        if (
            (not cands[DEC_LEFT].feasible)
            and (not cands[DEC_RIGHT].feasible)
            and front_near < 1.4
        ):
            if left_free > right_free + 0.45 and left_free > 1.0:
                cands[DEC_LEFT].feasible = True
                cands[DEC_LEFT].reason = "SOFT_FREE_SPACE"
                cands[DEC_LEFT].route_quality = QUALITY_WARNING
                cands[DEC_LEFT].total_cost = 85.0 - 5.0 * min(2.0, left_free)
                cands[DEC_LEFT].vx = 0.10
                cands[DEC_LEFT].w = WZ_MAX
            elif right_free > left_free + 0.45 and right_free > 1.0:
                cands[DEC_RIGHT].feasible = True
                cands[DEC_RIGHT].reason = "SOFT_FREE_SPACE"
                cands[DEC_RIGHT].route_quality = QUALITY_WARNING
                cands[DEC_RIGHT].total_cost = 85.0 - 5.0 * min(2.0, right_free)
                cands[DEC_RIGHT].vx = 0.10
                cands[DEC_RIGHT].w = -WZ_MAX

        # Raw preference (ignore commitment) for debug — not authoritative
        raw_pref = None
        raw_cost = 1e18
        for k in (DEC_LEFT, DEC_RIGHT):
            if cands[k].feasible and cands[k].total_cost < raw_cost:
                raw_pref, raw_cost = k, cands[k].total_cost
        if cands[DEC_FORWARD].feasible and cands[DEC_FORWARD].route_quality in (
            QUALITY_GOOD,
            QUALITY_WARNING,
        ):
            if raw_pref is None or cands[DEC_FORWARD].total_cost + 1e-6 < raw_cost:
                raw_pref = DEC_FORWARD

        selected, reason = self._select(
            now,
            cands,
            forward_feasible=forward_feasible,
            dynamic_short=dynamic_short,
            commitment_active=commitment_active,
            committed_side=committed_side,
            side_switch_authorized=side_switch_authorized,
            commitment_hard_fail=commitment_hard_fail,
            authorized_side=authorized_side,
        )
        for c in cands.values():
            c.selected = c.type == selected

        snap = {
            "pose": {"x": x, "y": y, "theta": yaw},
            "front_near": front_near,
            "rear_near": rear_near,
            "left_free": left_free,
            "right_free": right_free,
            "forward_feasible": forward_feasible,
            "selected": selected,
            "reason": reason,
            "raw_preferred_side": raw_pref,
            "commitment_active": bool(commitment_active),
            "committed_side": committed_side,
            "side_switch_authorized": bool(side_switch_authorized),
            "authorized_side": authorized_side,
            "commitment_decision": reason if str(reason).startswith("COMMIT_") or str(reason).startswith("POLICY_") else None,
            "forward_cost": cands[DEC_FORWARD].total_cost,
            "left_cost": cands[DEC_LEFT].total_cost,
            "right_cost": cands[DEC_RIGHT].total_cost,
            "left_feasible": cands[DEC_LEFT].feasible,
            "right_feasible": cands[DEC_RIGHT].feasible,
            "forward_quality": cands[DEC_FORWARD].route_quality,
            "left_quality": cands[DEC_LEFT].route_quality,
            "right_quality": cands[DEC_RIGHT].route_quality,
            "compare_called": True,
            "compare_reason": self.last_compare_reason,
        }
        result = ManeuverComparisonResult(
            selected=selected, reason=reason, candidates=cands, trigger=True, snapshot=snap
        )
        self.last_result = result

        if selected != self.current:
            self._emit(
                "MANEUVER_SWITCH",
                from_dec=self.current,
                to_dec=selected,
                reason=reason,
                left_cost=cands[DEC_LEFT].total_cost,
                right_cost=cands[DEC_RIGHT].total_cost,
                forward_cost=cands[DEC_FORWARD].total_cost,
            )
            self._emit(
                f"{selected}_SELECTED",
                reason=reason,
                cost=cands[selected].total_cost if selected in cands else None,
            )
            if self.current in (DEC_LEFT, DEC_RIGHT) and selected in (DEC_LEFT, DEC_RIGHT) and selected != self.current:
                self.side_attempts += 1
            self.current = selected
            self.current_cost = cands[selected].total_cost if selected in cands else 1e6
            self.hold_until = now + MIN_HOLD_S
            self.enter_front = front_near
            self.enter_progress = s_cur
            self.obstacle_passed = False
            self.history.append({"ts": now, "selected": selected, "reason": reason})
            if len(self.history) > 30:
                self.history = self.history[-30:]
        else:
            self.current_cost = cands[selected].total_cost if selected in cands else self.current_cost

        if self.current in (DEC_LEFT, DEC_RIGHT):
            if front_near > self.enter_front + 0.35 and front_near > DEFAULT_GEOM.front_clear_m * 0.85:
                gain = s_cur - self.enter_progress
                if gain > 0.15 or cands[DEC_FORWARD].feasible:
                    self.obstacle_passed = True
                    self._emit("OBSTACLE_PASSED", front_near=front_near, progress_gain=gain)

        self.prev_w = cands[selected].w if selected in cands else self.prev_w
        return result

    def _select(
        self,
        now: float,
        cands: Dict[str, LocalManeuverCandidate],
        *,
        forward_feasible: bool,
        dynamic_short: bool,
        commitment_active: bool = False,
        committed_side: Optional[str] = None,
        side_switch_authorized: bool = False,
        commitment_hard_fail: bool = False,
        authorized_side: Optional[str] = None,
    ) -> Tuple[str, str]:
        cur = self.current
        # STEP 3E — authorized switch bypasses min-hold (one-shot Policy gate)
        auth_side = str(authorized_side or "").upper()
        if (
            commitment_active
            and side_switch_authorized
            and auth_side in (DEC_LEFT, DEC_RIGHT)
        ):
            return auth_side, "POLICY_AUTHORIZED_SWITCH"

        if cur in cands and cands[cur].feasible and now < self.hold_until:
            if cur != DEC_FORWARD or forward_feasible or cands[DEC_FORWARD].feasible:
                return cur, "HOLD_MIN_TIME"

        if dynamic_short and cands[DEC_WAIT].feasible and not (
            cands[DEC_LEFT].feasible or cands[DEC_RIGHT].feasible
        ):
            return DEC_WAIT, "DYNAMIC_WAIT"

        if cands[DEC_FORWARD].feasible and cands[DEC_FORWARD].route_quality in (QUALITY_GOOD, QUALITY_WARNING):
            if cur in (DEC_LEFT, DEC_RIGHT) and not self.obstacle_passed and cands[cur].feasible:
                if not self._better(cands[DEC_FORWARD].total_cost, cands[cur].total_cost):
                    return cur, "HOLD_SIDE_UNTIL_PASSED"
            return DEC_FORWARD, "FORWARD_OK"

        # STEP 3C — Commitment gate: block unauthorized LEFT↔RIGHT switches
        cs = str(committed_side or "").upper()
        if (
            commitment_active
            and cs in (DEC_LEFT, DEC_RIGHT)
            and not side_switch_authorized
        ):
            if cands[cs].feasible:
                return cs, "COMMIT_KEEP"
            _ = commitment_hard_fail
            return cs, "COMMIT_REVALIDATION_REQUIRED"

        left_ok = cands[DEC_LEFT].feasible
        right_ok = cands[DEC_RIGHT].feasible
        if left_ok or right_ok:
            if self.side_attempts >= MAX_SIDE_ATTEMPTS:
                if cands[DEC_REPOSITION].feasible:
                    return DEC_REPOSITION, "SIDE_ATTEMPTS_EXCEEDED→REPOSITION"
                return DEC_REPLAN, "SIDE_ATTEMPTS_EXCEEDED→REPLAN"

            best = None
            best_cost = 1e18
            for k in (DEC_LEFT, DEC_RIGHT):
                if cands[k].feasible and cands[k].total_cost < best_cost:
                    best, best_cost = k, cands[k].total_cost
            assert best is not None
            if cur in (DEC_LEFT, DEC_RIGHT) and cands[cur].feasible:
                if not self._better(best_cost, cands[cur].total_cost):
                    return cur, "HYSTERESIS_KEEP"
                if best != cur:
                    return best, "LOWER_TOTAL_COST"
                return cur, "HYSTERESIS_KEEP"
            return best, "LOWER_TOTAL_COST" if left_ok and right_ok else ("LEFT_ONLY" if left_ok else "RIGHT_ONLY")

        if cands[DEC_ALIGN].feasible and not forward_feasible:
            return DEC_ALIGN, "SIDES_BLOCKED→ALIGN"
        if cands[DEC_REPOSITION].feasible:
            return DEC_REPOSITION, "SIDES_BLOCKED→REPOSITION"
        if cands[DEC_REVERSE].feasible:
            return DEC_REVERSE, "LAST_RESORT_REVERSE"
        return DEC_REPLAN, "NO_LOCAL_MANEUVER"

    def _better(self, new_cost: float, cur_cost: float) -> bool:
        thresh = max(HYSTERESIS_ABS, abs(cur_cost) * HYSTERESIS_RATIO)
        return new_cost + thresh < cur_cost
