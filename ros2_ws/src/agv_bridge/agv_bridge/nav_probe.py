"""ProbeEngine — local predictive feasibility evidence (STEP 3D).

Evidence only. Does NOT write vx/w, change FSM mode, authorize side switches,
or trigger recovery/reverse/replan.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.local_maneuver import (
    DEC_FORWARD,
    DEC_LEFT,
    DEC_RIGHT,
    DEC_REVERSE,
    NOMINAL_VX,
    ROLLOUT_DT,
    SIDE_VX,
    WZ_NOM,
    _collide_body,
    _footprint_points,
    _nominal_side_w,
    rollout_candidate,
)
from agv_bridge.maneuver import wrap_pi
from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]


def _attach_poses_corridor(
    r: "ProbeResult",
    path: Sequence[Any],
    *,
    yaw0: float,
    vx: float,
    w: float,
    dt: float = ROLLOUT_DT,
) -> None:
    """Attach shared poses + PhysicalTrajectoryCorridor (no new rollout)."""
    from agv_bridge.nav_trajectory import build_corridor_from_poses, poses_from_rollout_path

    poses = poses_from_rollout_path(path, yaw0=yaw0, vx=vx, w=w, dt=dt)
    r.poses = [p.to_dict() for p in poses]
    corr = build_corridor_from_poses(
        poses,
        source=r.direction,
        status=r.status,
        collision=r.collision,
        soft_risk=r.soft_risk,
        min_clearance=r.min_clearance,
        failure_reason=r.failure_reason,
        progress=r.predicted_progress,
    )
    r.corridor = corr.to_dict()


# ---- Status / reasons (shared vocabulary) ----
STATUS_VALID = "VALID"
STATUS_INVALID = "INVALID"
STATUS_UNKNOWN = "UNKNOWN"
STATUS_STALE = "STALE"

FAIL_NONE = "NONE"
FAIL_COLLISION = "COLLISION"
FAIL_FOOTPRINT = "FOOTPRINT_COLLISION"
FAIL_CLEARANCE = "INSUFFICIENT_CLEARANCE"
FAIL_STOPPING = "STOPPING_MARGIN_NEGATIVE"
FAIL_CAPTURE = "PATH_CAPTURE_UNAVAILABLE"
FAIL_CORRIDOR = "CORRIDOR_BLOCKED"
FAIL_NO_PROGRESS = "NO_PROGRESS"
FAIL_DEAD_END = "DEAD_END_RISK"
FAIL_TURN_SPACE = "TURNING_SPACE_INSUFFICIENT"
FAIL_NO_DATA = "NO_DATA"
FAIL_STALE = "STALE_DATA"
FAIL_GEOMETRY = "INVALID_GEOMETRY"
FAIL_SKEW = "INPUT_SKEW"
FAIL_NO_REAR = "NO_REAR_GEOMETRY"
FAIL_NAN = "NAN_INPUT"

DIR_FORWARD = "FORWARD"
DIR_BACKWARD = "BACKWARD"
DIR_LEFT = "LEFT"
DIR_RIGHT = "RIGHT"
DIR_TURN = "TURN_IN_PLACE"


@dataclass(frozen=True)
class ProbeConfig:
    """Centralized Probe parameters (meters / seconds). Do not scatter magic numbers."""

    # Horizons (m) — calibrated to DEFAULT_GEOM + cruise ~0.15–0.22 m/s
    horizon_short_m: float = 0.50
    horizon_medium_m: float = 1.05
    horizon_long_m: float = 1.80
    horizon_min_m: float = 0.40
    horizon_max_m: float = 2.20
    # Speed → horizon: base + k*|vx|, then clip
    horizon_vx_gain: float = 2.5  # m per (m/s)
    # Sampling
    sample_distance_m: float = 0.08
    sample_time_s: float = 0.10
    # Clearance / safety (reuse geom intent)
    min_clearance_m: float = 0.18  # matches CLR_HARD in local_maneuver
    safety_margin_m: float = 0.08
    localization_margin_m: float = 0.05
    control_margin_m: float = 0.05
    soft_risk_clearance_m: float = 0.35
    # Stopping model: v^2/(2*a) + reaction*v
    reaction_time_s: float = 0.15
    # Stale / skew
    stale_timeout_s: float = 0.80  # ~16 ticks @20Hz; sim updates every cycle
    skew_warn_ms: float = 120.0
    skew_stale_ms: float = 400.0
    # Cache
    cache_ttl_static_s: float = 0.25
    cache_ttl_dynamic_s: float = 0.08
    pose_bucket_m: float = 0.05
    yaw_bucket_rad: float = 0.08
    vx_bucket: float = 0.05
    # Turn probe
    turn_yaw_step_rad: float = math.radians(15.0)
    turn_yaw_max_rad: float = math.radians(90.0)
    # Backward
    reverse_vx: float = -0.12
    reverse_steer_w: float = 0.28
    escape_min_m: float = 0.35
    # Horizon tiers frequency (cycles)
    medium_every_n: int = 2
    long_every_n: int = 5


DEFAULT_PROBE_CFG = ProbeConfig()


@dataclass
class ObstacleSnapshot:
    """Same-tick world view for all probe directions."""

    timestamp: float
    pose_ts: Optional[float]
    obstacle_ts: Optional[float]
    x: float
    y: float
    yaw: float
    vx: float
    w: float
    front_distance: float
    rear_distance: float
    left_clearance_raw: float
    right_clearance_raw: float
    collide: Optional[CollideFn] = None
    clearance_at: Optional[ClearanceFn] = None
    global_path: Optional[Sequence[Pt]] = None
    goal: Optional[Pt] = None
    corridor_half_width: Optional[float] = None
    source: str = "nav_models"
    validity: str = "OK"  # OK / NO_DATA / INVALID_GEOMETRY
    dynamic_short: bool = False
    map_bounds: Optional[Tuple[float, float, float, float]] = None
    path_version: int = 0
    mode: str = ""
    obstacle_key: str = "default"

    def age_s(self, now: Optional[float] = None) -> float:
        ts = now if now is not None else time.time()
        return max(0.0, float(ts) - float(self.timestamp))

    def skew_ms(self) -> Optional[float]:
        if self.pose_ts is None or self.obstacle_ts is None:
            return None
        return abs(float(self.pose_ts) - float(self.obstacle_ts)) * 1000.0

    def signature(self, cfg: ProbeConfig = DEFAULT_PROBE_CFG) -> str:
        if any(v != v for v in (self.x, self.y, self.yaw, self.vx)):
            return "NAN_INPUT"
        pb = cfg.pose_bucket_m
        yb = cfg.yaw_bucket_rad
        vb = cfg.vx_bucket
        return (
            f"p{round(self.x / pb) * pb:.2f},{round(self.y / pb) * pb:.2f}"
            f"_y{round(self.yaw / yb)}"
            f"_v{round(self.vx / vb)}"
            f"_f{round(self.front_distance, 1)}"
            f"_r{round(self.rear_distance, 1)}"
            f"_L{round(self.left_clearance_raw, 1)}"
            f"_R{round(self.right_clearance_raw, 1)}"
            f"_d{int(self.dynamic_short)}_pv{self.path_version}_{self.mode}"
            f"_o{self.obstacle_key}"
        )

    def to_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        return {
            "available": self.validity == "OK" and self.collide is not None,
            "timestamp": self.timestamp,
            "pose_timestamp": self.pose_ts,
            "obstacle_timestamp": self.obstacle_ts,
            "age_s": round(self.age_s(now), 4),
            "skew_ms": self.skew_ms(),
            "source": self.source,
            "validity": self.validity,
            "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
            "vx": self.vx,
            "w": self.w,
            "front": {"distance": self.front_distance},
            "rear": {"distance": self.rear_distance},
            "left": {"clearance_raw": self.left_clearance_raw},
            "right": {"clearance_raw": self.right_clearance_raw},
            "signature": self.signature(),
            "dynamic_short": self.dynamic_short,
            "note": "same-tick snapshot for F/B/L/R/TURN",
        }


@dataclass
class ProbeResult:
    direction: str
    status: str = STATUS_UNKNOWN
    collision: bool = False
    hard_collision: bool = False
    soft_risk: bool = False
    min_clearance: Optional[float] = None
    collision_distance: Optional[float] = None
    time_to_collision: Optional[float] = None
    predicted_progress: Optional[float] = None
    predicted_goal_delta: Optional[float] = None
    path_capture_valid: Optional[bool] = None
    path_capture_distance: Optional[float] = None
    corridor_valid: Optional[bool] = None
    corridor_lateral_error: Optional[float] = None
    stopping_distance: Optional[float] = None
    stopping_margin: Optional[float] = None
    escape_available: Optional[bool] = None
    escape_distance: Optional[float] = None
    reverse_distance: Optional[float] = None
    turning_space: Optional[float] = None
    rotation_valid: Optional[bool] = None
    max_safe_yaw_delta: Optional[float] = None
    collision_yaw: Optional[float] = None
    failure_reason: str = FAIL_NONE
    confidence: Optional[float] = None  # null unless computed honestly
    horizon_s: Optional[float] = None
    horizon_m: Optional[float] = None
    sample_count: int = 0
    computation_ms: float = 0.0
    timestamp: Optional[float] = None
    obstacle_type: str = "UNKNOWN"  # STATIC/DYNAMIC/UNKNOWN — no fake labels
    # STEP 3F — same rollout poses for Blue Band (no second integrator)
    poses: List[Dict[str, float]] = field(default_factory=list)
    corridor: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "direction": self.direction,
            "status": self.status,
            "collision": self.collision,
            "hard_collision": self.hard_collision,
            "soft_risk": self.soft_risk,
            "min_clearance": _r(self.min_clearance),
            "collision_distance": _r(self.collision_distance),
            "time_to_collision": _r(self.time_to_collision),
            "predicted_progress": _r(self.predicted_progress),
            "predicted_goal_delta": _r(self.predicted_goal_delta),
            "path_capture_valid": self.path_capture_valid,
            "path_capture_distance": _r(self.path_capture_distance),
            "corridor_valid": self.corridor_valid,
            "corridor_lateral_error": _r(self.corridor_lateral_error),
            "stopping_distance": _r(self.stopping_distance),
            "stopping_margin": _r(self.stopping_margin),
            "escape_available": self.escape_available,
            "escape_distance": _r(self.escape_distance),
            "reverse_distance": _r(self.reverse_distance),
            "turning_space": _r(self.turning_space),
            "rotation_valid": self.rotation_valid,
            "max_safe_yaw_delta": _r(self.max_safe_yaw_delta, 4),
            "collision_yaw": _r(self.collision_yaw, 4),
            "failure_reason": self.failure_reason,
            "confidence": self.confidence,
            "horizon_s": _r(self.horizon_s),
            "horizon_m": _r(self.horizon_m),
            "sample_count": self.sample_count,
            "computation_ms": round(self.computation_ms, 3),
            "timestamp": self.timestamp,
            "obstacle_type": self.obstacle_type,
            "poses": self.poses[:24],
            "corridor": self.corridor,
        }


@dataclass
class ProbeBundle:
    snapshot: ObstacleSnapshot
    forward: ProbeResult
    backward: ProbeResult
    left: ProbeResult
    right: ProbeResult
    turn_in_place: ProbeResult
    bundle_ms: float = 0.0
    cache_hit: bool = False
    input_signature: str = ""
    no_valid_escape: bool = False
    implemented: bool = True
    note: str = "STEP3D evidence-only; does not authorize side switch / reverse"

    def to_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        dirs = {
            "forward": self.forward.to_dict(),
            "backward": self.backward.to_dict(),
            "left": self.left.to_dict(),
            "right": self.right.to_dict(),
            "turn_in_place": self.turn_in_place.to_dict(),
        }
        # Aggregate status for UI header
        statuses = [self.forward.status, self.backward.status, self.left.status, self.right.status, self.turn_in_place.status]
        if any(s == STATUS_STALE for s in statuses):
            agg = STATUS_STALE
        elif any(s == STATUS_UNKNOWN for s in statuses):
            agg = STATUS_UNKNOWN
        elif any(s == STATUS_INVALID for s in statuses):
            agg = "MIXED"
        else:
            agg = STATUS_VALID
        return {
            "implemented": True,
            "status": agg,
            "forward": dirs["forward"],
            "backward": dirs["backward"],
            "left": dirs["left"],
            "right": dirs["right"],
            "turn_in_place": dirs["turn_in_place"],
            "bundle_ms": round(self.bundle_ms, 3),
            "forward_ms": round(self.forward.computation_ms, 3),
            "backward_ms": round(self.backward.computation_ms, 3),
            "left_ms": round(self.left.computation_ms, 3),
            "right_ms": round(self.right.computation_ms, 3),
            "turn_ms": round(self.turn_in_place.computation_ms, 3),
            "cache_hit": self.cache_hit,
            "input_signature": self.input_signature,
            "no_valid_escape": self.no_valid_escape,
            "obstacle_snapshot": self.snapshot.to_dict(now),
            "note": self.note,
        }


def _r(v: Optional[float], n: int = 3) -> Optional[float]:
    if v is None:
        return None
    try:
        if v != v:  # NaN
            return None
        return round(float(v), n)
    except Exception:
        return None


def stopping_distance_m(vx: float, cfg: ProbeConfig = DEFAULT_PROBE_CFG, geom=DEFAULT_GEOM) -> float:
    """Kinematic stop distance for |vx| using vehicle accel + reaction."""
    v = abs(float(vx))
    a = max(0.15, float(geom.acc_v))
    return (v * v) / (2.0 * a) + cfg.reaction_time_s * v


def horizon_for_speed(vx: float, base_m: float, cfg: ProbeConfig = DEFAULT_PROBE_CFG) -> float:
    h = float(base_m) + cfg.horizon_vx_gain * abs(float(vx))
    return max(cfg.horizon_min_m, min(cfg.horizon_max_m, h))


def build_obstacle_snapshot(
    *,
    now: float,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    w: float,
    front_near: float,
    rear_near: float,
    left_free: float,
    right_free: float,
    collide: Optional[CollideFn],
    clearance_at: Optional[ClearanceFn] = None,
    global_path: Optional[Sequence[Pt]] = None,
    goal: Optional[Pt] = None,
    corridor_half_width: Optional[float] = None,
    pose_ts: Optional[float] = None,
    obstacle_ts: Optional[float] = None,
    dynamic_short: bool = False,
    source: str = "nav_models",
    mode: str = "",
    path_version: int = 0,
    map_bounds: Optional[Tuple[float, float, float, float]] = None,
) -> ObstacleSnapshot:
    validity = "OK"
    if collide is None:
        validity = "NO_DATA"
    if any(v != v for v in (x, y, yaw, vx, w, front_near, rear_near)):
        validity = "INVALID_GEOMETRY"
    return ObstacleSnapshot(
        timestamp=now,
        pose_ts=pose_ts if pose_ts is not None else now,
        obstacle_ts=obstacle_ts if obstacle_ts is not None else now,
        x=x,
        y=y,
        yaw=yaw,
        vx=vx,
        w=w,
        front_distance=float(front_near),
        rear_distance=float(rear_near),
        left_clearance_raw=float(left_free),
        right_clearance_raw=float(right_free),
        collide=collide,
        clearance_at=clearance_at,
        global_path=global_path,
        goal=goal,
        corridor_half_width=corridor_half_width,
        source=source,
        validity=validity,
        dynamic_short=dynamic_short,
        map_bounds=map_bounds,
        path_version=path_version,
        mode=mode,
        obstacle_key=str(id(collide)) if collide is not None else "none",
    )


class ProbeEngine:
    """Evaluate F/B/L/R/TURN on one ObstacleSnapshot. Evidence only."""

    def __init__(self, cfg: ProbeConfig = DEFAULT_PROBE_CFG) -> None:
        self.cfg = cfg
        self._cache_sig: Optional[str] = None
        self._cache_bundle: Optional[ProbeBundle] = None
        self._cache_ts: float = 0.0
        self._cycle: int = 0
        self.events: List[Dict[str, Any]] = []
        self._prev_status: Dict[str, str] = {}

    def reset(self) -> None:
        self._cache_sig = None
        self._cache_bundle = None
        self._cache_ts = 0.0
        self._cycle = 0
        self.events.clear()
        self._prev_status.clear()

    def evaluate(self, snap: ObstacleSnapshot, *, now: Optional[float] = None) -> ProbeBundle:
        t0 = time.perf_counter()
        ts = now if now is not None else time.time()
        self._cycle += 1
        cfg = self.cfg
        sig = snap.signature(cfg)

        # Cache (short TTL; dynamic shorter)
        ttl = cfg.cache_ttl_dynamic_s if snap.dynamic_short else cfg.cache_ttl_static_s
        if (
            self._cache_bundle is not None
            and self._cache_sig == sig
            and (ts - self._cache_ts) <= ttl
        ):
            b = self._cache_bundle
            b.cache_hit = True
            return b

        # Preflight: validity / stale / skew
        gate = self._gate_snapshot(snap, ts)
        if gate is not None:
            bundle = self._uniform_bundle(snap, gate, ts, t0, sig, cache_hit=False)
            self._store_cache(sig, bundle, ts)
            self._emit_status_events(bundle, ts)
            return bundle

        do_medium = (self._cycle % max(1, cfg.medium_every_n)) == 0
        do_long = (self._cycle % max(1, cfg.long_every_n)) == 0
        # Always SHORT; MEDIUM/LONG extend horizon for side/backward escape
        fwd = self._probe_forward(snap, ts, do_medium=do_medium)
        back = self._probe_backward(snap, ts, do_long=do_long)
        left = self._probe_side(snap, DIR_LEFT, ts, do_medium=do_medium)
        right = self._probe_side(snap, DIR_RIGHT, ts, do_medium=do_medium)
        turn = self._probe_turn(snap, ts)

        no_escape = all(
            r.status == STATUS_INVALID
            for r in (fwd, back, left, right)
        )
        bundle = ProbeBundle(
            snapshot=snap,
            forward=fwd,
            backward=back,
            left=left,
            right=right,
            turn_in_place=turn,
            bundle_ms=(time.perf_counter() - t0) * 1000.0,
            cache_hit=False,
            input_signature=sig,
            no_valid_escape=no_escape,
        )
        self._store_cache(sig, bundle, ts)
        self._emit_status_events(bundle, ts)
        return bundle

    def _store_cache(self, sig: str, bundle: ProbeBundle, ts: float) -> None:
        self._cache_sig = sig
        self._cache_bundle = bundle
        self._cache_ts = ts

    def _gate_snapshot(self, snap: ObstacleSnapshot, ts: float) -> Optional[ProbeResult]:
        """Return a template ProbeResult if entire bundle must be UNKNOWN/STALE."""
        cfg = self.cfg
        if snap.validity == "NO_DATA" or snap.collide is None:
            return ProbeResult(
                direction="*",
                status=STATUS_UNKNOWN,
                failure_reason=FAIL_NO_DATA,
                timestamp=ts,
            )
        if snap.validity == "INVALID_GEOMETRY":
            return ProbeResult(
                direction="*",
                status=STATUS_UNKNOWN,
                failure_reason=FAIL_GEOMETRY,
                timestamp=ts,
            )
        if any(v != v for v in (snap.x, snap.y, snap.yaw)):
            return ProbeResult(
                direction="*",
                status=STATUS_UNKNOWN,
                failure_reason=FAIL_NAN,
                timestamp=ts,
            )
        age = snap.age_s(ts)
        if age > cfg.stale_timeout_s:
            return ProbeResult(
                direction="*",
                status=STATUS_STALE,
                failure_reason=FAIL_STALE,
                timestamp=ts,
            )
        skew = snap.skew_ms()
        if skew is not None and skew > cfg.skew_stale_ms:
            return ProbeResult(
                direction="*",
                status=STATUS_STALE,
                failure_reason=FAIL_SKEW,
                timestamp=ts,
            )
        return None

    def _uniform_bundle(
        self,
        snap: ObstacleSnapshot,
        gate: ProbeResult,
        ts: float,
        t0: float,
        sig: str,
        *,
        cache_hit: bool,
    ) -> ProbeBundle:
        def one(d: str) -> ProbeResult:
            r = ProbeResult(
                direction=d,
                status=gate.status,
                failure_reason=gate.failure_reason,
                timestamp=ts,
                computation_ms=0.0,
            )
            return r

        return ProbeBundle(
            snapshot=snap,
            forward=one(DIR_FORWARD),
            backward=one(DIR_BACKWARD),
            left=one(DIR_LEFT),
            right=one(DIR_RIGHT),
            turn_in_place=one(DIR_TURN),
            bundle_ms=(time.perf_counter() - t0) * 1000.0,
            cache_hit=cache_hit,
            input_signature=sig,
            no_valid_escape=False,
        )

    def _probe_forward(self, snap: ObstacleSnapshot, ts: float, *, do_medium: bool) -> ProbeResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        base = cfg.horizon_medium_m if do_medium else cfg.horizon_short_m
        horizon_m = horizon_for_speed(snap.vx, base, cfg)
        stop_d = stopping_distance_m(max(0.0, snap.vx), cfg)
        r = ProbeResult(
            direction=DIR_FORWARD,
            timestamp=ts,
            horizon_m=horizon_m,
            stopping_distance=stop_d,
            obstacle_type="DYNAMIC" if snap.dynamic_short else "UNKNOWN",
        )
        assert snap.collide is not None
        vx = max(0.06, abs(snap.vx) if snap.vx > 0.02 else NOMINAL_VX * 0.85)
        path = list(snap.global_path or [])
        cand = rollout_candidate(
            ctype=DEC_FORWARD,
            x=snap.x,
            y=snap.y,
            yaw=snap.yaw,
            vx=vx,
            w=0.0,
            path=path,
            goal=snap.goal,
            collide=snap.collide,
            clearance_at=snap.clearance_at,
            front_near=snap.front_distance,
            require_capture=False,
            map_bounds=snap.map_bounds,
        )
        r.sample_count = max(1, len(cand.path) - 1)
        r.horizon_s = r.sample_count * ROLLOUT_DT
        r.min_clearance = cand.min_clearance
        r.predicted_progress = cand.path_progress_gain
        r.path_capture_valid = cand.path_capture_available
        r.path_capture_distance = cand.path_capture_distance
        r.corridor_lateral_error = cand.lateral_error
        if snap.corridor_half_width is not None:
            r.corridor_valid = abs(cand.lateral_error) <= float(snap.corridor_half_width) + 0.35

        # Stopping margin vs front distance (and rollout clearance)
        front_clr = float(snap.front_distance)
        margin = front_clr - stop_d - cfg.safety_margin_m - cfg.localization_margin_m - cfg.control_margin_m
        r.stopping_margin = margin

        if cand.collision or (not cand.feasible and cand.reason in ("COLLISION",)):
            r.status = STATUS_INVALID
            r.collision = True
            r.hard_collision = True
            r.failure_reason = FAIL_FOOTPRINT
            r.time_to_collision = cand.first_collision_t
            if cand.first_collision_x is not None:
                r.collision_distance = math.hypot(
                    cand.first_collision_x - snap.x, (cand.first_collision_y or snap.y) - snap.y
                )
        elif margin < 0.0:
            r.status = STATUS_INVALID
            r.failure_reason = FAIL_STOPPING
        elif not cand.feasible and cand.reason == "LOW_CLEARANCE":
            r.status = STATUS_INVALID
            r.failure_reason = FAIL_CLEARANCE
        elif cand.feasible:
            r.status = STATUS_VALID
            if (cand.min_clearance is not None and cand.min_clearance < cfg.soft_risk_clearance_m) or margin < 0.15:
                r.soft_risk = True
        else:
            r.status = STATUS_INVALID
            r.failure_reason = cand.reason or FAIL_COLLISION

        _attach_poses_corridor(r, cand.path, yaw0=snap.yaw, vx=vx, w=0.0)
        r.computation_ms = (time.perf_counter() - t0) * 1000.0
        return r

    def _probe_backward(self, snap: ObstacleSnapshot, ts: float, *, do_long: bool) -> ProbeResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        r = ProbeResult(direction=DIR_BACKWARD, timestamp=ts, obstacle_type="UNKNOWN")
        # Honest UNKNOWN if rear geometry looks unavailable (sentinel huge + no collide samples)
        if snap.rear_distance > 50.0 and snap.clearance_at is None:
            r.status = STATUS_UNKNOWN
            r.failure_reason = FAIL_NO_REAR
            r.computation_ms = (time.perf_counter() - t0) * 1000.0
            return r

        assert snap.collide is not None
        horizon_m = cfg.horizon_long_m if do_long else cfg.horizon_medium_m
        horizon_m = max(cfg.horizon_min_m, min(cfg.horizon_max_m, horizon_m))
        r.horizon_m = horizon_m
        vx = cfg.reverse_vx
        dt = cfg.sample_time_s
        step_dist = max(0.02, abs(vx) * dt)
        steps = max(4, int(math.ceil(horizon_m / step_dist)))
        path = list(snap.global_path or [])

        # Straight reverse first
        straight = self._reverse_rollout(
            snap, vx=vx, w=0.0, steps=steps, path=path
        )
        r.sample_count = straight["samples"]
        r.min_clearance = straight["min_clearance"]
        r.reverse_distance = straight["distance"]
        r.escape_distance = straight["distance"] if not straight["collision"] else straight.get("collision_dist")
        r.horizon_s = steps * cfg.sample_time_s
        if straight.get("path"):
            _attach_poses_corridor(
                r, straight["path"], yaw0=snap.yaw, vx=vx, w=0.0, dt=cfg.sample_time_s
            )

        if straight["collision"]:
            r.status = STATUS_INVALID
            r.collision = True
            r.hard_collision = True
            r.failure_reason = FAIL_FOOTPRINT
            r.time_to_collision = straight.get("ttc")
            r.collision_distance = straight.get("collision_dist")
            r.escape_available = False
            if straight.get("path"):
                _attach_poses_corridor(
                    r, straight["path"], yaw0=snap.yaw, vx=vx, w=0.0, dt=cfg.sample_time_s
                )
            r.computation_ms = (time.perf_counter() - t0) * 1000.0
            return r

        # Turning space after reverse: sample small steer reverse
        turn_l = self._reverse_rollout(snap, vx=vx, w=cfg.reverse_steer_w, steps=max(3, steps // 2), path=path)
        turn_r = self._reverse_rollout(snap, vx=vx, w=-cfg.reverse_steer_w, steps=max(3, steps // 2), path=path)
        turn_ok = (not turn_l["collision"]) or (not turn_r["collision"])
        r.turning_space = max(
            turn_l.get("min_clearance") or 0.0,
            turn_r.get("min_clearance") or 0.0,
        )

        # Escape: moved enough. Steered-reverse is optional enrichment — NOT required
        # for three-side pocket with open rear (straight reverse is the escape).
        dist = float(straight["distance"] or 0.0)
        escape = dist >= cfg.escape_min_m
        # STEP 3F-CORRECTIVE: sealed L/R must NOT invalidate straight-open rear.
        # Old DEAD_END rule wrongly set B=INVALID whenever sides were tight,
        # blocking LOCAL_REVERSE even with clear backward corridor.
        if dist >= cfg.escape_min_m and (not turn_ok) and snap.left_clearance_raw < 0.45 and snap.right_clearance_raw < 0.45:
            r.status = STATUS_VALID
            r.escape_available = True
            r.soft_risk = True
            r.failure_reason = "LIMITED_TURN_AFTER_REVERSE"
            if straight.get("path"):
                _attach_poses_corridor(
                    r, straight["path"], yaw0=snap.yaw, vx=vx, w=0.0, dt=cfg.sample_time_s
                )
            r.computation_ms = (time.perf_counter() - t0) * 1000.0
            return r

        if not turn_ok and dist < cfg.escape_min_m:
            r.status = STATUS_INVALID
            r.failure_reason = FAIL_TURN_SPACE
            r.escape_available = False
        elif escape:
            r.status = STATUS_VALID
            r.escape_available = True
            if (r.min_clearance or 0) < cfg.soft_risk_clearance_m:
                r.soft_risk = True
        else:
            r.status = STATUS_VALID
            r.escape_available = False
            r.soft_risk = True
            if dist < cfg.escape_min_m:
                r.failure_reason = FAIL_NONE  # short reverse still geometrically clear
        if straight.get("path"):
            _attach_poses_corridor(
                r, straight["path"], yaw0=snap.yaw, vx=vx, w=0.0, dt=cfg.sample_time_s
            )
        r.computation_ms = (time.perf_counter() - t0) * 1000.0
        return r

    def _reverse_rollout(
        self,
        snap: ObstacleSnapshot,
        *,
        vx: float,
        w: float,
        steps: int,
        path: Sequence[Pt],
    ) -> Dict[str, Any]:
        cfg = self.cfg
        assert snap.collide is not None
        cx, cy, cyaw = snap.x, snap.y, snap.yaw
        clrs: List[float] = []
        dt = cfg.sample_time_s
        traj: List[Dict[str, float]] = [{"x": cx, "y": cy, "yaw": round(cyaw, 4)}]
        for i in range(steps):
            cx += vx * math.cos(cyaw) * dt
            cy += vx * math.sin(cyaw) * dt
            cyaw = wrap_pi(cyaw + w * dt)
            traj.append({"x": round(cx, 3), "y": round(cy, 3), "yaw": round(cyaw, 4)})
            if _collide_body(cx, cy, cyaw, snap.collide):
                dist = math.hypot(cx - snap.x, cy - snap.y)
                return {
                    "collision": True,
                    "samples": i + 1,
                    "distance": dist,
                    "collision_dist": dist,
                    "ttc": (i + 1) * dt,
                    "min_clearance": min(clrs) if clrs else 0.0,
                    "path": traj,
                }
            if snap.clearance_at:
                clrs.append(float(snap.clearance_at(cx, cy)))
            else:
                # footprint-proxied clearance via radial samples
                clrs.append(max(0.05, min(snap.rear_distance, 2.0) - 0.1 * (i + 1)))
        dist = math.hypot(cx - snap.x, cy - snap.y)
        return {
            "collision": False,
            "samples": steps,
            "distance": dist,
            "min_clearance": min(clrs) if clrs else float(snap.rear_distance),
            "path": traj,
        }

    def _probe_side(
        self, snap: ObstacleSnapshot, direction: str, ts: float, *, do_medium: bool
    ) -> ProbeResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        r = ProbeResult(direction=direction, timestamp=ts, obstacle_type="UNKNOWN")
        assert snap.collide is not None
        ctype = DEC_LEFT if direction == DIR_LEFT else DEC_RIGHT
        path = list(snap.global_path or [])
        w = _nominal_side_w(ctype, snap.yaw, path, snap.x, snap.y)
        # Horizon via rollout steps (fixed 1.5s) — medium uses same kinematic; soft capture
        cand = rollout_candidate(
            ctype=ctype,
            x=snap.x,
            y=snap.y,
            yaw=snap.yaw,
            vx=SIDE_VX,
            w=w,
            path=path,
            goal=snap.goal,
            collide=snap.collide,
            clearance_at=snap.clearance_at,
            front_near=snap.front_distance,
            require_capture=False,
            map_bounds=snap.map_bounds,
        )
        # Secondary arc if primary hard-fails — still footprint evidence, not a vote
        if (not cand.feasible) or cand.collision:
            from agv_bridge.local_maneuver import WZ_MAX

            w2 = WZ_MAX if ctype == DEC_LEFT else -WZ_MAX
            cand2 = rollout_candidate(
                ctype=ctype,
                x=snap.x,
                y=snap.y,
                yaw=snap.yaw,
                vx=SIDE_VX * 0.75,
                w=w2,
                path=path,
                goal=snap.goal,
                collide=snap.collide,
                clearance_at=snap.clearance_at,
                front_near=snap.front_distance,
                require_capture=False,
                map_bounds=snap.map_bounds,
            )
            if cand2.feasible and not cand2.collision:
                cand = cand2
        r.sample_count = max(1, len(cand.path) - 1)
        r.horizon_s = r.sample_count * ROLLOUT_DT
        r.horizon_m = cfg.horizon_medium_m if do_medium else cfg.horizon_short_m
        r.min_clearance = cand.min_clearance
        r.predicted_progress = cand.path_progress_gain
        r.path_capture_valid = cand.path_capture_available
        r.path_capture_distance = cand.path_capture_distance
        r.corridor_lateral_error = cand.lateral_error
        if snap.corridor_half_width is not None:
            # Soft corridor during avoid — warn only
            r.corridor_valid = abs(cand.lateral_error) <= float(snap.corridor_half_width) + 0.85

        if cand.collision or cand.reason == "COLLISION":
            r.status = STATUS_INVALID
            r.collision = True
            r.hard_collision = True
            r.failure_reason = FAIL_FOOTPRINT
            r.time_to_collision = cand.first_collision_t
            if cand.first_collision_x is not None:
                r.collision_distance = math.hypot(
                    cand.first_collision_x - snap.x, (cand.first_collision_y or snap.y) - snap.y
                )
        elif not cand.feasible and cand.reason == "LOW_CLEARANCE":
            r.status = STATUS_INVALID
            r.failure_reason = FAIL_CLEARANCE
        elif cand.feasible:
            r.status = STATUS_VALID
            if cand.min_clearance is not None and cand.min_clearance < cfg.soft_risk_clearance_m:
                r.soft_risk = True
        else:
            r.status = STATUS_INVALID
            r.failure_reason = cand.reason or FAIL_COLLISION

        _attach_poses_corridor(r, cand.path, yaw0=snap.yaw, vx=float(cand.vx), w=float(cand.w))
        r.computation_ms = (time.perf_counter() - t0) * 1000.0
        return r

    def _probe_turn(self, snap: ObstacleSnapshot, ts: float) -> ProbeResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        r = ProbeResult(direction=DIR_TURN, timestamp=ts, obstacle_type="UNKNOWN")
        assert snap.collide is not None
        # Pose fixed; sweep yaw — footprint must clear
        max_safe = 0.0
        collision_yaw = None
        min_clr = 99.0
        samples = 0
        step = cfg.turn_yaw_step_rad
        limit = cfg.turn_yaw_max_rad
        yaw0 = snap.yaw
        safes: List[float] = []
        for sign in (1.0, -1.0):
            safe = 0.0
            for k in range(1, int(limit / step) + 1):
                dyaw = sign * k * step
                yaw = wrap_pi(yaw0 + dyaw)
                samples += 1
                if _collide_body(snap.x, snap.y, yaw, snap.collide):
                    if collision_yaw is None:
                        collision_yaw = dyaw
                    break
                if snap.clearance_at:
                    for px, py in _footprint_points(snap.x, snap.y, yaw):
                        min_clr = min(min_clr, float(snap.clearance_at(px, py)))
                safe = abs(dyaw)
            safes.append(safe)
        max_safe = min(safes) if safes else 0.0

        r.sample_count = samples
        r.max_safe_yaw_delta = max_safe
        r.collision_yaw = collision_yaw
        r.min_clearance = None if min_clr >= 98 else min_clr
        r.turning_space = r.min_clearance
        # Also check current pose body
        if _collide_body(snap.x, snap.y, yaw0, snap.collide):
            r.status = STATUS_INVALID
            r.collision = True
            r.hard_collision = True
            r.rotation_valid = False
            r.failure_reason = FAIL_FOOTPRINT
        elif collision_yaw is not None and abs(collision_yaw) <= step * 1.01:
            r.status = STATUS_INVALID
            r.rotation_valid = False
            r.failure_reason = FAIL_TURN_SPACE
            r.hard_collision = True
            r.collision = True
        elif max_safe < step:
            r.status = STATUS_INVALID
            r.rotation_valid = False
            r.failure_reason = FAIL_TURN_SPACE
        else:
            r.status = STATUS_VALID
            r.rotation_valid = True
            if max_safe < math.radians(30.0):
                r.soft_risk = True
        # Turn corridor = footprint yaw samples at fixed base pose
        turn_path = []
        for k in range(0, int(max(max_safe, step) / step) + 1):
            yaw = wrap_pi(yaw0 + k * step)
            turn_path.append({"x": snap.x, "y": snap.y, "yaw": round(yaw, 4)})
        if turn_path:
            _attach_poses_corridor(r, turn_path, yaw0=yaw0, vx=0.0, w=0.0, dt=0.0)
        r.computation_ms = (time.perf_counter() - t0) * 1000.0
        return r

    def _emit_status_events(self, bundle: ProbeBundle, ts: float) -> None:
        mapping = {
            DIR_FORWARD: bundle.forward,
            DIR_BACKWARD: bundle.backward,
            DIR_LEFT: bundle.left,
            DIR_RIGHT: bundle.right,
            DIR_TURN: bundle.turn_in_place,
        }
        for d, res in mapping.items():
            prev = self._prev_status.get(d)
            if prev == res.status:
                continue
            self._prev_status[d] = res.status
            ev = {
                "event": "PROBE_STATUS_CHANGE",
                "direction": d,
                "old": prev,
                "new": res.status,
                "reason": res.failure_reason,
                "min_clearance": res.min_clearance,
                "timestamp": ts,
            }
            if res.status == STATUS_INVALID and res.hard_collision:
                ev["event"] = "PROBE_HARD_FAIL"
            elif res.status == STATUS_VALID and prev == STATUS_INVALID:
                ev["event"] = "PROBE_RECOVERED"
            elif res.status == STATUS_STALE:
                ev["event"] = "PROBE_STALE"
            elif res.status == STATUS_UNKNOWN:
                ev["event"] = "PROBE_UNKNOWN"
            self.events.append(ev)
            if len(self.events) > 80:
                self.events = self.events[-80:]


# Re-export footprint helpers for tests
footprint_points = _footprint_points
collide_body = _collide_body
