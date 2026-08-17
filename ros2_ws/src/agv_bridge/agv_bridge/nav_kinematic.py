"""P0-C Kinematic Path Validator — differential-drive feasibility + swept footprint.

Telemetry / reference only. Does NOT emit cmd_vel. Does NOT bypass Safety / FSM / Recovery.
Does NOT rewrite Global A*. Does NOT invent VehicleGeometry sizes or control limits.

Kinematic model (existing DD integrator):
    ω = v * κ
    κ > 0 = LEFT (CCW, toward body +y)
    κ < 0 = RIGHT

High |κ| is not automatically INVALID: if v_allowed = ω_max / |κ| >= v_min_forward,
the path is VALID with speed_limited=True.
"""

from __future__ import annotations

import copy
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import (
    SWEPT_SPATIAL_STEP_M,
    footprint_points,
    footprint_polygon_body,
    sample_rotation_sweep,
    trajectory_collision,
)
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_geometry
from agv_bridge.nav_global_preview import densify_polyline

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]
ClearanceFn = Callable[[float, float], float]

# Control limits: cited from existing modules, not invented.
#   geom.max_vx = 0.40, geom.max_w = 0.45, geom.acc_v = 0.9  (nav_geometry.py)
#   MPPI wz_max = min(0.42, geom.max_w); forward floor vx_cmd = max(0.04, ...)  (mppi_controller.py)
#   local_maneuver.WZ_MAX = 0.42; maneuver.WZ_MAX = 0.42
W_MAX_CONTROL = 0.42
V_MIN_FORWARD = 0.04
KAPPA_EPS = 1e-6
CROSS_EPS = 1e-9
DUP_EPS_M = 0.01
REVERSAL_RAD = math.radians(150.0)
HARD_CORNER_RAD = math.radians(15.0)
TIGHT_TURN_RAD = math.radians(60.0)
DENSIFY_M = 0.10
ARC_DS = 0.08

STATUS_VALID = "VALID"
STATUS_INVALID = "INVALID"
STATUS_DEGRADED = "DEGRADED"
STATUS_NOT_VALIDATED = "NOT_VALIDATED"

REASON_NO_PATH = "NO_PATH"
REASON_HARD_CORNER = "HARD_CORNER"
REASON_CURVATURE_UNBOUNDED = "CURVATURE_UNBOUNDED"
REASON_OMEGA_LIMIT = "OMEGA_LIMIT"
REASON_MIN_SPEED_INFEASIBLE = "MIN_SPEED_INFEASIBLE"
REASON_FOOTPRINT_COLLISION = "FOOTPRINT_COLLISION"
REASON_CLEARANCE_TOO_LOW = "CLEARANCE_TOO_LOW"
REASON_ROTATION_SWEEP_COLLISION = "ROTATION_SWEEP_COLLISION"
REASON_NO_FEASIBLE_TRANSITION = "NO_BOUNDED_CURVATURE_TRANSITION"
REASON_NUMERIC = "NUMERIC_FAILURE"
REASON_MISSING = "MISSING_CONSTRAINT"
REASON_HEADING_REVERSAL = "HEADING_REVERSAL"


def validator_enabled() -> bool:
    v = (os.environ.get("NAV_KINEMATIC_VALIDATOR", "1") or "1").strip().lower()
    return v not in ("0", "false", "off", "no")


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _as_xy(p: Any) -> Optional[Pt]:
    if isinstance(p, dict):
        return float(p.get("x") or 0.0), float(p.get("y") or 0.0)
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return float(p[0]), float(p[1])
    return None


def _dedupe(path: Sequence[Any]) -> List[Pt]:
    out: List[Pt] = []
    for p in path or []:
        xy = _as_xy(p)
        if xy is None:
            continue
        if out and math.hypot(xy[0] - out[-1][0], xy[1] - out[-1][1]) < DUP_EPS_M:
            out[-1] = xy
            continue
        out.append(xy)
    return out


def control_limits(geom: Optional[VehicleGeometry] = None) -> Dict[str, float]:
    g = geom or get_vehicle_geometry()
    return {
        "v_max": float(g.max_vx),
        "w_max": min(W_MAX_CONTROL, float(g.max_w)),
        "v_min_forward": V_MIN_FORWARD,
        "acc_v": float(g.acc_v),
        "acc_w": float(g.acc_w),
        "v_ref": float(g.max_vx),
        "safety_margin_m": float(getattr(g, "safety_margin_m", 0.08) or 0.08),
    }


def polygon_extents(geom: Optional[VehicleGeometry] = None) -> Dict[str, float]:
    """Narrow-phase body extents from P0-A polygon (length-clamped), not bumper_l."""
    g = geom or get_vehicle_geometry()
    poly = footprint_polygon_body(g)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return {
        "front_extent_m": max(xs) if xs else 0.5 * float(g.length),
        "rear_extent_m": min(xs) if xs else -0.5 * float(g.length),
        "left_extent_m": max(ys) if ys else 0.5 * float(g.width),
        "right_extent_m": min(ys) if ys else -0.5 * float(g.width),
        "length_m": float(g.length),
        "width_m": float(g.width),
        "bumper_sample_m": float(g.bumper_l),
    }


def discrete_curvature(p0: Pt, p1: Pt, p2: Pt) -> float:
    """Signed κ from circumcircle. κ>0 LEFT. Colinear → 0 (no 1e6 noise)."""
    ab = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    bc = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    ca = math.hypot(p0[0] - p2[0], p0[1] - p2[1])
    if ab < DUP_EPS_M or bc < DUP_EPS_M or ca < DUP_EPS_M:
        return 0.0
    cross = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0])
    if abs(cross) < CROSS_EPS:
        return 0.0
    den = ab * bc * ca
    if den < 1e-12:
        return 0.0
    return 2.0 * cross / den


def turn_radius(kappa: Optional[float]) -> Optional[float]:
    if kappa is None or abs(float(kappa)) < KAPPA_EPS:
        return None
    return 1.0 / abs(float(kappa))


@dataclass
class KinematicSpeedProfile:
    s_m: List[float] = field(default_factory=list)
    v_max_mps: List[float] = field(default_factory=list)
    kappa: List[float] = field(default_factory=list)
    omega_at_vref: List[float] = field(default_factory=list)

    def to_compact(self, max_n: int = 80) -> Dict[str, Any]:
        n = len(self.s_m)
        if n == 0:
            return {"count": 0, "s_m": [], "v_max_mps": []}
        step = max(1, n // max_n)
        idx = list(range(0, n, step))
        if idx[-1] != n - 1:
            idx.append(n - 1)
        return {
            "count": n,
            "s_m": [round(self.s_m[i], 3) for i in idx],
            "v_max_mps": [round(self.v_max_mps[i], 3) for i in idx],
        }


@dataclass
class KinematicValidationResult:
    status: str = STATUS_NOT_VALIDATED
    valid: Optional[bool] = None
    reason: Optional[str] = None
    primary_reason: Optional[str] = None
    violations: List[str] = field(default_factory=list)

    raw_path_length_m: float = 0.0
    validated_path_length_m: float = 0.0
    path_revision: int = 0
    validation_revision: int = 0
    validation_id: str = ""

    max_abs_curvature: Optional[float] = None
    mean_abs_curvature: Optional[float] = None
    min_turn_radius_m: Optional[float] = None
    max_required_w_at_reference_speed: Optional[float] = None
    reference_speed_mps: Optional[float] = None
    feasible_speed_min_mps: Optional[float] = None
    feasible_speed_max_mps: Optional[float] = None
    max_feasible_speed_mps: Optional[float] = None
    speed_limited: bool = False
    speed_limited_from_m: Optional[float] = None
    speed_limited_distance_m: float = 0.0

    min_clearance_m: Optional[float] = None
    clearance_at_index: Optional[int] = None
    clearance_at_distance_m: Optional[float] = None
    swept_collision: bool = False
    collision: bool = False
    rotation_sweep_valid: Optional[bool] = None

    first_invalid_index: Optional[int] = None
    first_invalid_distance_m: Optional[float] = None
    first_kinematic_issue_distance_m: Optional[float] = None

    raw_turn_count: int = 0
    turns_gt15: int = 0
    turns_gt30: int = 0
    turns_gt60: int = 0
    turns_gt90: int = 0
    hard_corner_count: int = 0
    transition_applied: bool = False
    needs_reverse_maneuver: bool = False
    executable: bool = False
    controls_vehicle: bool = False
    source: str = "RAW_GLOBAL"
    geometry_status: str = "REFERENCE_ONLY"
    kinematic_status: str = STATUS_NOT_VALIDATED
    kinematic_valid: Optional[bool] = None

    cache_hit: bool = False
    compute_ms: float = 0.0
    path_points: int = 0
    swept_samples: int = 0
    validated_poses: List[Dict[str, Any]] = field(default_factory=list)
    speed_profile: Optional[KinematicSpeedProfile] = None
    note: str = ""

    def to_api(self, *, include_poses: bool = False) -> Dict[str, Any]:
        d = {
            "status": self.status,
            "valid": self.valid,
            "kinematic_valid": self.kinematic_valid,
            "kinematic_status": self.kinematic_status,
            "reason": self.reason,
            "primary_reason": self.primary_reason,
            "violations": list(self.violations),
            "raw_path_length_m": round(self.raw_path_length_m, 3),
            "validated_path_length_m": round(self.validated_path_length_m, 3),
            "path_revision": self.path_revision,
            "validation_revision": self.validation_revision,
            "validation_id": self.validation_id,
            "max_curvature": None if self.max_abs_curvature is None else round(self.max_abs_curvature, 4),
            "mean_curvature": None if self.mean_abs_curvature is None else round(self.mean_abs_curvature, 4),
            "min_turn_radius_m": None if self.min_turn_radius_m is None else round(self.min_turn_radius_m, 3),
            "reference_speed_mps": self.reference_speed_mps,
            "max_required_w_rad_s": None
            if self.max_required_w_at_reference_speed is None
            else round(self.max_required_w_at_reference_speed, 4),
            "max_feasible_speed_mps": None
            if self.max_feasible_speed_mps is None
            else round(self.max_feasible_speed_mps, 3),
            "feasible_speed_min_mps": None
            if self.feasible_speed_min_mps is None
            else round(self.feasible_speed_min_mps, 3),
            "speed_limited": self.speed_limited,
            "speed_limited_from_m": self.speed_limited_from_m,
            "speed_limited_distance_m": round(self.speed_limited_distance_m, 3),
            "min_clearance_m": None if self.min_clearance_m is None else round(self.min_clearance_m, 3),
            "clearance_at_distance_m": self.clearance_at_distance_m,
            "swept_collision": self.swept_collision,
            "collision": self.collision,
            "rotation_sweep_valid": self.rotation_sweep_valid,
            "first_invalid_index": self.first_invalid_index,
            "first_invalid_distance_m": self.first_invalid_distance_m,
            "first_kinematic_issue_distance_m": self.first_kinematic_issue_distance_m,
            "raw_turn_count": self.raw_turn_count,
            "turns_gt15": self.turns_gt15,
            "turns_gt30": self.turns_gt30,
            "turns_gt60": self.turns_gt60,
            "turns_gt90": self.turns_gt90,
            "hard_corner_count": self.hard_corner_count,
            "transition_applied": self.transition_applied,
            "needs_reverse_maneuver": self.needs_reverse_maneuver,
            "executable": self.executable,
            "controls_vehicle": False,
            "source": self.source,
            "geometry_status": self.geometry_status,
            "cache_hit": self.cache_hit,
            "compute_ms": round(self.compute_ms, 3),
            "path_points": self.path_points,
            "swept_samples": self.swept_samples,
            "note": self.note,
        }
        if self.speed_profile is not None:
            d["speed_profile"] = self.speed_profile.to_compact()
        if include_poses:
            d["validated_poses"] = self.validated_poses
        else:
            poses = self.validated_poses
            if len(poses) > 80:
                step = max(1, len(poses) // 80)
                thin = poses[::step]
                if thin[-1] is not poses[-1]:
                    thin = list(thin) + [poses[-1]]
                d["validated_poses"] = thin
            else:
                d["validated_poses"] = poses
        return d


def _path_len(pts: Sequence[Pt]) -> float:
    s = 0.0
    for i in range(1, len(pts)):
        s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    return s


def _heading(a: Pt, b: Pt) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _raw_turn_stats(pts: Sequence[Pt]) -> Dict[str, int]:
    n15 = n30 = n60 = n90 = hard = 0
    total = 0
    for i in range(1, len(pts) - 1):
        h0 = _heading(pts[i - 1], pts[i])
        h1 = _heading(pts[i], pts[i + 1])
        d = abs(_wrap(h1 - h0))
        if d < math.radians(5.0):
            continue
        total += 1
        deg = math.degrees(d)
        if deg > 15:
            n15 += 1
        if deg > 30:
            n30 += 1
        if deg > 60:
            n60 += 1
        if deg > 90:
            n90 += 1
        if d >= HARD_CORNER_RAD:
            hard += 1
    return {
        "raw_turn_count": total,
        "turns_gt15": n15,
        "turns_gt30": n30,
        "turns_gt60": n60,
        "turns_gt90": n90,
        "hard_corner_count": hard,
    }


def _unit(dx: float, dy: float) -> Optional[Pt]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    return (dx / n, dy / n)


def _collapse_colinear(pts: Sequence[Pt], *, deg: float = 2.0) -> List[Pt]:
    """Drop near-colinear samples so fillet sees real edge lengths, not grid spacing."""
    if len(pts) < 3:
        return list(pts)
    thr = math.radians(deg)
    out: List[Pt] = [pts[0]]
    for i in range(1, len(pts) - 1):
        h0 = _heading(out[-1], pts[i])
        h1 = _heading(pts[i], pts[i + 1])
        if abs(_wrap(h1 - h0)) < thr:
            continue
        out.append(pts[i])
    out.append(pts[-1])
    return out


def insert_circular_fillets(
    pts: Sequence[Pt],
    *,
    geom: VehicleGeometry,
    w_max: float,
    v_ref: float,
) -> Tuple[List[Pt], bool, Optional[str], bool]:
    """Tangent-in / arc / tangent-out at polyline corners. Does not mutate pts.

    Returns (new_pts, applied, fail_reason, needs_reverse).
    path_quality.smooth_path_collision_checked is a linear chamfer — not used here.
    """
    if len(pts) < 3:
        return list(pts), False, None, False
    pts = _collapse_colinear(pts)
    if len(pts) < 3:
        return list(pts), False, None, False
    r_min = 0.5 * float(geom.width)
    r_pref = v_ref / max(w_max, 1e-6) if v_ref > 0 else r_min
    out: List[Pt] = [pts[0]]
    applied = False
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        u_in = _unit(b[0] - a[0], b[1] - a[1])
        u_out = _unit(c[0] - b[0], c[1] - b[1])
        if u_in is None or u_out is None:
            continue
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        dot = max(-1.0, min(1.0, u_in[0] * u_out[0] + u_in[1] * u_out[1]))
        alpha = math.atan2(cross, dot)
        abs_a = abs(alpha)
        if abs_a < HARD_CORNER_RAD:
            if not out or math.hypot(b[0] - out[-1][0], b[1] - out[-1][1]) >= DUP_EPS_M:
                out.append(b)
            continue
        if abs_a >= REVERSAL_RAD:
            return list(pts), False, REASON_HEADING_REVERSAL, True
        len_in = math.hypot(b[0] - a[0], b[1] - a[1])
        len_out = math.hypot(c[0] - b[0], c[1] - b[1])
        # keep a stub so neighboring corners do not consume the whole edge
        max_t = min(0.45 * len_in, 0.45 * len_out)
        half = abs_a / 2.0
        tan_h = math.tan(half) if half > 1e-6 else 1e-6
        r_fit = max_t / tan_h if tan_h > 1e-9 else 0.0
        r = min(r_pref, r_fit) if r_fit > 0 else 0.0
        if r + 1e-9 < r_min and abs_a >= TIGHT_TURN_RAD:
            return list(pts), False, REASON_NO_FEASIBLE_TRANSITION, False
        if r < 0.05:
            if not out or math.hypot(b[0] - out[-1][0], b[1] - out[-1][1]) >= DUP_EPS_M:
                out.append(b)
            continue
        t = r * tan_h
        p_enter = (b[0] - u_in[0] * t, b[1] - u_in[1] * t)
        p_exit = (b[0] + u_out[0] * t, b[1] + u_out[1] * t)
        # center: from enter, along inward normal
        sgn = 1.0 if alpha > 0 else -1.0
        nrm = (-u_in[1] * sgn, u_in[0] * sgn)
        center = (p_enter[0] + nrm[0] * r, p_enter[1] + nrm[1] * r)
        ang0 = math.atan2(p_enter[1] - center[1], p_enter[0] - center[0])
        ang1 = math.atan2(p_exit[1] - center[1], p_exit[0] - center[0])
        d_ang = _wrap(ang1 - ang0)
        arc_len = abs(d_ang) * r
        n_arc = max(3, int(math.ceil(arc_len / ARC_DS)))
        if not out or math.hypot(p_enter[0] - out[-1][0], p_enter[1] - out[-1][1]) >= DUP_EPS_M:
            out.append(p_enter)
        for k in range(1, n_arc):
            frac = k / n_arc
            ang = ang0 + d_ang * frac
            out.append((center[0] + r * math.cos(ang), center[1] + r * math.sin(ang)))
        out.append(p_exit)
        applied = True
    if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) >= DUP_EPS_M:
        out.append(pts[-1])
    return out, applied, None, False


def _poses_from_xy(xy: Sequence[Pt]) -> List[Dict[str, float]]:
    dens = densify_polyline([{"x": p[0], "y": p[1]} for p in xy], spacing_m=DENSIFY_M)
    return dens


def _annotate_curvature(poses: List[Dict[str, float]]) -> None:
    """κ = Δyaw / Δs (unwrap). Linear densify does not invent finite curvature."""
    n = len(poses)
    for i, p in enumerate(poses):
        if i == 0 or i == n - 1:
            p["kappa"] = 0.0
            continue
        ds = float(poses[i + 1].get("s") or 0.0) - float(poses[i - 1].get("s") or 0.0)
        dth = _wrap(float(poses[i + 1].get("yaw") or 0.0) - float(poses[i - 1].get("yaw") or 0.0))
        if ds < 1e-4 or abs(dth) < 1e-6:
            p["kappa"] = 0.0
        else:
            p["kappa"] = round(dth / ds, 5)


def _build_speed_profile(
    poses: List[Dict[str, float]],
    *,
    v_max: float,
    w_max: float,
    v_ref: float,
    acc_v: float,
) -> Tuple[KinematicSpeedProfile, bool, Optional[float], float]:
    n = len(poses)
    s_list = [float(p.get("s") or p.get("distance_along_path") or 0.0) for p in poses]
    if n >= 2 and abs(s_list[-1] - s_list[0]) < 1e-6:
        acc = 0.0
        s_list = [0.0]
        for i in range(1, n):
            acc += math.hypot(float(poses[i]["x"]) - float(poses[i - 1]["x"]), float(poses[i]["y"]) - float(poses[i - 1]["y"]))
            s_list.append(acc)
    kappas = [float(p.get("kappa") or 0.0) for p in poses]
    v_om = []
    w_ref = []
    for k in kappas:
        ak = abs(k)
        w_ref.append(v_ref * k)
        if ak < KAPPA_EPS:
            v_om.append(v_max)
        else:
            v_om.append(min(v_max, w_max / ak))
    # accel/decel along path (existing geom.acc_v)
    v = list(v_om)
    for i in range(n - 2, -1, -1):
        ds = max(0.0, s_list[i + 1] - s_list[i])
        v[i] = min(v[i], math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * acc_v * ds)))
    for i in range(1, n):
        ds = max(0.0, s_list[i] - s_list[i - 1])
        v[i] = min(v[i], math.sqrt(max(0.0, v[i - 1] ** 2 + 2.0 * acc_v * ds)))
    limited = False
    limited_from: Optional[float] = None
    limited_ds = 0.0
    for i, (vv, k) in enumerate(zip(v, kappas)):
        need = abs(v_ref * k)
        if need > w_max + 1e-6:
            limited = True
            if limited_from is None:
                limited_from = s_list[i] - s_list[0]
            if i + 1 < n:
                limited_ds += max(0.0, s_list[i + 1] - s_list[i])
        poses[i]["v_feasible"] = round(vv, 4)
        poses[i]["omega_at_vref"] = round(w_ref[i], 4)
    prof = KinematicSpeedProfile(s_m=s_list, v_max_mps=v, kappa=kappas, omega_at_vref=w_ref)
    return prof, limited, limited_from, limited_ds


class KinematicPathValidator:
    """Full-path validator with revision cache. Pure navigation module (no HTTP)."""

    def __init__(self) -> None:
        self._cache_key: Optional[Tuple[Any, ...]] = None
        self._cache: Optional[KinematicValidationResult] = None
        self._n = 0

    def _next_id(self) -> str:
        self._n += 1
        return f"VAL-{self._n:06d}"

    def validate(
        self,
        raw_path: Sequence[Any],
        *,
        path_revision: int = 0,
        collide: Optional[CollideFn] = None,
        clearance_at: Optional[ClearanceFn] = None,
        geom: Optional[VehicleGeometry] = None,
        goal_reached: bool = False,
        cache_token: Any = None,
    ) -> KinematicValidationResult:
        t0 = time.perf_counter()
        g = geom or get_vehicle_geometry()
        lim = control_limits(g)
        pts = _dedupe(raw_path)
        key = (
            int(path_revision),
            len(pts),
            pts[0] if pts else None,
            pts[len(pts) // 2] if pts else None,
            pts[-1] if pts else None,
            cache_token,
        )
        if self._cache is not None and self._cache_key == key:
            hit = copy.copy(self._cache)
            hit.cache_hit = True
            hit.compute_ms = (time.perf_counter() - t0) * 1000.0
            return hit

        res = KinematicValidationResult(
            path_revision=int(path_revision),
            validation_revision=int(path_revision),
            validation_id=self._next_id(),
            geometry_status="REFERENCE_ONLY",
            source="RAW_GLOBAL",
            reference_speed_mps=lim["v_ref"],
            note="P0-C kinematic validation — does not control the vehicle",
        )
        stats = _raw_turn_stats(pts)
        for k, v in stats.items():
            setattr(res, k, v)
        res.raw_path_length_m = _path_len(pts)

        if goal_reached or not pts:
            res.status = STATUS_VALID if goal_reached else STATUS_INVALID
            res.reason = None if goal_reached else REASON_NO_PATH
            res.primary_reason = res.reason
            res.valid = bool(goal_reached)
            res.kinematic_valid = True if goal_reached else False
            res.kinematic_status = res.status
            res.executable = bool(goal_reached)
            if not goal_reached:
                res.violations = [REASON_NO_PATH]
            res.compute_ms = (time.perf_counter() - t0) * 1000.0
            self._store(key, res)
            return res

        if len(pts) == 1:
            res.status = STATUS_VALID
            res.valid = True
            res.kinematic_valid = True
            res.kinematic_status = STATUS_VALID
            res.executable = True
            res.max_abs_curvature = 0.0
            res.min_turn_radius_m = None
            res.max_feasible_speed_mps = lim["v_max"]
            res.feasible_speed_min_mps = lim["v_max"]
            res.max_required_w_at_reference_speed = 0.0
            res.validated_poses = [{"x": pts[0][0], "y": pts[0][1], "yaw": 0.0, "s": 0.0, "kappa": 0.0}]
            res.path_points = 1
            res.compute_ms = (time.perf_counter() - t0) * 1000.0
            self._store(key, res)
            return res

        filleted, applied, fail, needs_rev = insert_circular_fillets(
            pts, geom=g, w_max=lim["w_max"], v_ref=lim["v_ref"]
        )
        res.transition_applied = applied
        res.needs_reverse_maneuver = needs_rev
        if fail:
            res.status = STATUS_INVALID
            res.valid = False
            res.kinematic_valid = False
            res.kinematic_status = STATUS_INVALID
            res.executable = False
            res.reason = fail
            res.primary_reason = fail
            res.violations = [fail]
            res.first_invalid_index = 1 if len(pts) > 1 else 0
            res.first_invalid_distance_m = 0.0
            res.first_kinematic_issue_distance_m = 0.0
            # Rotation may exist but must not prove the path VALID
            if len(pts) >= 2:
                yaw0 = _heading(pts[0], pts[1])
                yaw1 = yaw0 + math.pi / 2
                if collide is not None:
                    rot_ok = True
                    for fp in sample_rotation_sweep(pts[0][0], pts[0][1], yaw0, yaw1, g, margin_m=lim["safety_margin_m"]):
                        if any(collide(px, py) for px, py in fp.samples):
                            rot_ok = False
                            break
                    res.rotation_sweep_valid = rot_ok
                    if not rot_ok:
                        res.violations.append(REASON_ROTATION_SWEEP_COLLISION)
            res.compute_ms = (time.perf_counter() - t0) * 1000.0
            self._store(key, res)
            return res

        poses = _poses_from_xy(filleted)
        if len(poses) < 2:
            res.status = STATUS_INVALID
            res.valid = False
            res.kinematic_valid = False
            res.kinematic_status = STATUS_INVALID
            res.reason = REASON_NUMERIC
            res.primary_reason = REASON_NUMERIC
            res.violations = [REASON_NUMERIC]
            res.compute_ms = (time.perf_counter() - t0) * 1000.0
            self._store(key, res)
            return res
        _annotate_curvature(poses)
        # reject remaining unbounded spikes (should be gone after fillet)
        for i, p in enumerate(poses):
            k = abs(float(p.get("kappa") or 0.0))
            if k > 50.0:
                res.status = STATUS_INVALID
                res.valid = False
                res.kinematic_valid = False
                res.kinematic_status = STATUS_INVALID
                res.reason = REASON_CURVATURE_UNBOUNDED
                res.primary_reason = REASON_CURVATURE_UNBOUNDED
                res.violations = [REASON_CURVATURE_UNBOUNDED]
                res.first_invalid_index = i
                res.first_invalid_distance_m = round(float(p.get("s") or 0.0) - float(poses[0].get("s") or 0.0), 3)
                res.compute_ms = (time.perf_counter() - t0) * 1000.0
                self._store(key, res)
                return res

        kappas = [abs(float(p.get("kappa") or 0.0)) for p in poses]
        res.max_abs_curvature = max(kappas) if kappas else 0.0
        res.mean_abs_curvature = (sum(kappas) / len(kappas)) if kappas else 0.0
        res.min_turn_radius_m = turn_radius(res.max_abs_curvature)
        res.max_required_w_at_reference_speed = res.max_abs_curvature * lim["v_ref"]
        prof, limited, lim_from, lim_ds = _build_speed_profile(
            poses, v_max=lim["v_max"], w_max=lim["w_max"], v_ref=lim["v_ref"], acc_v=lim["acc_v"]
        )
        res.speed_profile = prof
        res.speed_limited = limited
        res.speed_limited_from_m = None if lim_from is None else round(lim_from, 3)
        res.speed_limited_distance_m = lim_ds
        res.feasible_speed_min_mps = min(prof.v_max_mps) if prof.v_max_mps else lim["v_max"]
        res.feasible_speed_max_mps = max(prof.v_max_mps) if prof.v_max_mps else lim["v_max"]
        res.max_feasible_speed_mps = res.feasible_speed_max_mps
        if res.feasible_speed_min_mps is not None and res.feasible_speed_min_mps + 1e-9 < lim["v_min_forward"]:
            if res.max_abs_curvature and res.max_abs_curvature > KAPPA_EPS:
                res.status = STATUS_INVALID
                res.valid = False
                res.kinematic_valid = False
                res.kinematic_status = STATUS_INVALID
                res.reason = REASON_MIN_SPEED_INFEASIBLE
                res.primary_reason = REASON_MIN_SPEED_INFEASIBLE
                res.violations = [REASON_MIN_SPEED_INFEASIBLE]
                res.executable = False
                res.compute_ms = (time.perf_counter() - t0) * 1000.0
                self._store(key, res)
                return res
        if limited:
            res.violations.append(REASON_OMEGA_LIMIT)

        # Swept footprint (P0-A APIs)
        collide_fn = collide if collide is not None else (lambda _x, _y: False)
        col = trajectory_collision(
            poses,
            collide_fn,
            g,
            margin_m=lim["safety_margin_m"],
            spatial_step_m=SWEPT_SPATIAL_STEP_M,
        )
        res.swept_samples = int(col.samples_checked)
        res.swept_collision = bool(col.collision)
        res.collision = bool(col.collision)
        if col.collision:
            res.status = STATUS_INVALID
            res.valid = False
            res.kinematic_valid = False
            res.kinematic_status = STATUS_INVALID
            res.reason = REASON_FOOTPRINT_COLLISION
            res.primary_reason = REASON_FOOTPRINT_COLLISION
            res.violations.append(REASON_FOOTPRINT_COLLISION)
            res.executable = False
            res.first_invalid_index = col.first_index if col.first_index >= 0 else None
            if col.first_pose is not None:
                s0 = float(poses[0].get("s") or 0.0)
                # nearest pose s
                best_i = 0
                best_d = 1e9
                fx, fy = float(col.first_pose["x"]), float(col.first_pose["y"])
                for i, p in enumerate(poses):
                    d = math.hypot(float(p["x"]) - fx, float(p["y"]) - fy)
                    if d < best_d:
                        best_d = d
                        best_i = i
                res.first_invalid_distance_m = round(float(poses[best_i].get("s") or 0.0) - s0, 3)
                res.first_kinematic_issue_distance_m = res.first_invalid_distance_m
            res.validated_path_length_m = _path_len(filleted)
            res.path_points = len(poses)
            res.validated_poses = []
            res.compute_ms = (time.perf_counter() - t0) * 1000.0
            self._store(key, res)
            return res

        min_cl: Optional[float] = None
        cl_i = None
        if clearance_at is not None:
            for i, p in enumerate(poses):
                for px, py in footprint_points(float(p["x"]), float(p["y"]), float(p.get("yaw") or 0.0), g):
                    c = float(clearance_at(px, py))
                    if min_cl is None or c < min_cl:
                        min_cl = c
                        cl_i = i
        res.min_clearance_m = min_cl
        res.clearance_at_index = cl_i
        if cl_i is not None:
            res.clearance_at_distance_m = round(float(poses[cl_i].get("s") or 0.0) - float(poses[0].get("s") or 0.0), 3)

        degraded = False
        if min_cl is not None and 0.0 <= min_cl < lim["safety_margin_m"]:
            degraded = True
            res.violations.append(REASON_CLEARANCE_TOO_LOW)

        res.validated_path_length_m = float(poses[-1].get("s") or 0.0) - float(poses[0].get("s") or 0.0)
        if res.validated_path_length_m <= 0:
            res.validated_path_length_m = _path_len(filleted)
        res.path_points = len(poses)
        res.validated_poses = [
            {
                "x": round(float(p["x"]), 3),
                "y": round(float(p["y"]), 3),
                "yaw": round(float(p.get("yaw") or 0.0), 4),
                "s": round(float(p.get("s") or 0.0), 3),
                "kappa": round(float(p.get("kappa") or 0.0), 5),
                "v_feasible": p.get("v_feasible"),
            }
            for p in poses
        ]
        res.rotation_sweep_valid = True
        res.executable = not degraded
        if degraded:
            res.status = STATUS_DEGRADED
            res.valid = None
            res.kinematic_valid = None
            res.kinematic_status = STATUS_DEGRADED
            res.reason = REASON_CLEARANCE_TOO_LOW
            res.primary_reason = REASON_CLEARANCE_TOO_LOW
            res.first_kinematic_issue_distance_m = res.clearance_at_distance_m
        else:
            res.status = STATUS_VALID
            res.valid = True
            res.kinematic_valid = True
            res.kinematic_status = STATUS_VALID
            res.reason = REASON_OMEGA_LIMIT if limited else None
            res.primary_reason = res.reason
            if limited:
                res.first_kinematic_issue_distance_m = res.speed_limited_from_m
        res.compute_ms = (time.perf_counter() - t0) * 1000.0
        self._store(key, res)
        return res

    def _store(self, key: Tuple[Any, ...], res: KinematicValidationResult) -> None:
        self._cache_key = key
        self._cache = res


_VALIDATOR = KinematicPathValidator()


def empty_validation(*, reason: str = REASON_NO_PATH, revision: int = 0) -> Dict[str, Any]:
    r = KinematicValidationResult(
        status=STATUS_INVALID if reason == REASON_NO_PATH else STATUS_NOT_VALIDATED,
        valid=False if reason == REASON_NO_PATH else None,
        kinematic_valid=False if reason == REASON_NO_PATH else None,
        kinematic_status=STATUS_INVALID if reason == REASON_NO_PATH else STATUS_NOT_VALIDATED,
        reason=reason,
        primary_reason=reason,
        path_revision=revision,
        validation_revision=revision,
        executable=False,
    )
    return r.to_api()


def validate_global_path(
    raw_path: Sequence[Any],
    *,
    path_revision: int = 0,
    collide: Optional[CollideFn] = None,
    clearance_at: Optional[ClearanceFn] = None,
    goal_reached: bool = False,
    cache_token: Any = None,
) -> KinematicValidationResult:
    return _VALIDATOR.validate(
        raw_path,
        path_revision=path_revision,
        collide=collide,
        clearance_at=clearance_at,
        goal_reached=goal_reached,
        cache_token=cache_token,
    )
