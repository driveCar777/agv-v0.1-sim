"""M3.8 trajectory integrity — anchor, age, frame semantics (map-frame only)."""

from __future__ import annotations

import hashlib
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]

TRAJ_KIND_FUTURE_PHYSICAL = "FUTURE_LOCAL_PHYSICAL"
TRAJ_KIND_FUTURE_ROLLING = "FUTURE_ROLLING_LOCAL_PLAN"
TRAJ_KIND_EXECUTED_KINEMATIC = "EXECUTED_KINEMATIC_BAND"
TRAJ_KIND_HISTORY_BREADCRUMB = "HISTORY_BREADCRUMB"
TRAJ_KIND_GLOBAL = "GLOBAL_REFERENCE"

FRAME_MAP = "map"


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def global_path_fingerprint(waypoints: Sequence[Any], *, max_pts: int = 32) -> str:
    """Stable hash of global path geometry for LOCAL_AVOIDANCE unchanged checks."""
    if not waypoints:
        return "empty"
    n = len(waypoints)
    if n <= max_pts:
        idx = list(range(n))
    else:
        step = max(1, n // max_pts)
        idx = list(range(0, n, step))
        if idx[-1] != n - 1:
            idx.append(n - 1)
    parts: List[str] = []
    for i in idx:
        p = waypoints[i]
        if isinstance(p, dict):
            x, y = float(p.get("x") or 0), float(p.get("y") or 0)
        else:
            x, y = float(p[0]), float(p[1])
        parts.append(f"{x:.3f},{y:.3f}")
    blob = "|".join(parts)
    return hashlib.sha1(blob.encode("utf-8")).hexdigest()[:12]


def max_anchor_tolerance_m(
    *,
    vehicle_speed_mps: float,
    planner_period_s: float = 0.20,
    control_latency_s: float = 0.10,
    min_m: float = 0.08,
    max_m: float = 1.50,
) -> float:
    """Derived anchor slack — not a magic constant."""
    spd = max(0.0, float(vehicle_speed_mps))
    tol = spd * (planner_period_s + control_latency_s) + 0.05
    return max(min_m, min(max_m, tol))


def max_anchor_yaw_deg(
    *,
    vehicle_omega_rad_s: float,
    planner_period_s: float = 0.20,
    control_latency_s: float = 0.10,
    min_deg: float = 5.0,
    max_deg: float = 45.0,
) -> float:
    w = abs(float(vehicle_omega_rad_s))
    tol = math.degrees(w * (planner_period_s + control_latency_s)) + 3.0
    return max(min_deg, min(max_deg, tol))


def _first_pose(traj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    poses = traj.get("poses") or traj.get("centerline") or []
    if not poses:
        return None
    p0 = poses[0]
    if isinstance(p0, dict):
        return p0
    if isinstance(p0, (list, tuple)) and len(p0) >= 2:
        return {"x": p0[0], "y": p0[1], "yaw": 0.0}
    return None


def anchor_error_m(vehicle: Dict[str, Any], traj: Dict[str, Any]) -> Optional[float]:
    p0 = _first_pose(traj)
    if not p0:
        return None
    vx = float(vehicle.get("x") or 0)
    vy = float(vehicle.get("y") or 0)
    return math.hypot(float(p0.get("x") or 0) - vx, float(p0.get("y") or 0) - vy)


def anchor_yaw_error_deg(vehicle: Dict[str, Any], traj: Dict[str, Any]) -> Optional[float]:
    p0 = _first_pose(traj)
    if not p0:
        return None
    vyaw = float(vehicle.get("angle") or vehicle.get("yaw") or 0)
    tyaw = float(p0.get("yaw") or p0.get("theta") or vyaw)
    return abs(math.degrees(_wrap(tyaw - vyaw)))


def trajectory_age_ms(traj: Dict[str, Any], now: Optional[float] = None) -> Optional[float]:
    ts = traj.get("trajectory_timestamp") or traj.get("generated_at") or traj.get("timestamp")
    if ts is None:
        return None
    return max(0.0, (float(now or time.time()) - float(ts)) * 1000.0)


def classify_display_source(snap: Dict[str, Any]) -> str:
    nav = snap.get("nav") or {}
    pt = nav.get("physical_trajectory") or {}
    if pt and (pt.get("poses") or pt.get("centerline")):
        return TRAJ_KIND_FUTURE_PHYSICAL
    lp = nav.get("local_plan") or {}
    if lp.get("poses") and lp.get("active") is not False:
        return TRAJ_KIND_FUTURE_ROLLING
    if nav.get("local_path"):
        return TRAJ_KIND_EXECUTED_KINEMATIC
    return "NONE"


def reanchor_trajectory_poses(traj: Dict[str, Any], vehicle: Dict[str, Any]) -> Dict[str, Any]:
    """Shift map-frame trajectory so poses[0] aligns with current vehicle (planner lag fix)."""
    out = dict(traj or {})
    p0 = _first_pose(out)
    if not p0:
        return out
    vx = float(vehicle.get("x") or 0)
    vy = float(vehicle.get("y") or 0)
    dx = vx - float(p0.get("x") or 0)
    dy = vy - float(p0.get("y") or 0)
    if abs(dx) < 1e-4 and abs(dy) < 1e-4:
        return out

    def _shift_pts(key: str) -> None:
        pts = out.get(key)
        if not isinstance(pts, list):
            return
        shifted = []
        for p in pts:
            if isinstance(p, dict):
                shifted.append({**p, "x": round(float(p.get("x") or 0) + dx, 4), "y": round(float(p.get("y") or 0) + dy, 4)})
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                shifted.append([float(p[0]) + dx, float(p[1]) + dy])
            else:
                shifted.append(p)
        out[key] = shifted

    for key in ("poses", "centerline", "left_edge", "right_edge", "swept_polygon"):
        _shift_pts(key)
    out["reanchored"] = True
    out["reanchor_shift_m"] = round(math.hypot(dx, dy), 4)
    return out


def enrich_trajectory_metadata(
    traj: Dict[str, Any],
    *,
    vehicle: Dict[str, Any],
    now: Optional[float] = None,
    planner_period_s: float = 0.20,
) -> Dict[str, Any]:
    """Attach M3.8 integrity fields to a trajectory dict (in-place copy)."""
    out = dict(traj or {})
    tnow = float(now or time.time())
    out.setdefault("frame_id", FRAME_MAP)
    out.setdefault("trajectory_kind", TRAJ_KIND_FUTURE_PHYSICAL)
    out.setdefault("trajectory_timestamp", out.get("generated_at") or tnow)
    out["vehicle_timestamp"] = tnow
    out["sample_timestamp"] = tnow
    spd = abs(float(vehicle.get("vx") or vehicle.get("speed") or 0))
    omega = float(vehicle.get("w") or 0)
    ae0 = anchor_error_m(vehicle, out)
    tol_m = max_anchor_tolerance_m(vehicle_speed_mps=spd, planner_period_s=planner_period_s)
    if ae0 is not None and tol_m is not None and ae0 > tol_m and spd > 0.04:
        out = reanchor_trajectory_poses(out, vehicle)
        out["stale_reanchor_applied"] = True
    ae = anchor_error_m(vehicle, out)
    ay = anchor_yaw_error_deg(vehicle, out)
    age = trajectory_age_ms(out, tnow)
    tol_m = max_anchor_tolerance_m(vehicle_speed_mps=spd, planner_period_s=planner_period_s)
    tol_y = max_anchor_yaw_deg(vehicle_omega_rad_s=omega, planner_period_s=planner_period_s)
    out["integrity"] = {
        "anchor_error_m": None if ae is None else round(ae, 4),
        "anchor_yaw_error_deg": None if ay is None else round(ay, 2),
        "trajectory_age_ms": None if age is None else round(age, 1),
        "planner_age_ms": None if age is None else round(age, 1),
        "max_anchor_error_m": round(tol_m, 4),
        "max_anchor_yaw_deg": round(tol_y, 2),
        "stale": bool(age is not None and age > (planner_period_s + 0.35) * 1000.0),
        "stale_reanchor_applied": bool(out.get("stale_reanchor_applied")),
        "reanchor_shift_m": out.get("reanchor_shift_m"),
        "behind_vehicle": _trajectory_behind_vehicle(vehicle, out),
    }
    return out


def _trajectory_behind_vehicle(vehicle: Dict[str, Any], traj: Dict[str, Any]) -> bool:
    """True if trajectory[0] is predominantly behind vehicle heading."""
    p0 = _first_pose(traj)
    if not p0:
        return False
    vx = float(vehicle.get("x") or 0)
    vy = float(vehicle.get("y") or 0)
    yaw = float(vehicle.get("angle") or vehicle.get("yaw") or 0)
    dx = float(p0.get("x") or 0) - vx
    dy = float(p0.get("y") or 0) - vy
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    return fwd < -0.15


def audit_snapshot_trajectory(snap: Dict[str, Any], *, now: Optional[float] = None) -> Dict[str, Any]:
    """Audit trajectory integrity from /api/state snapshot."""
    tnow = float(now or time.time())
    agv = snap.get("agv") or {}
    nav = snap.get("nav") or {}
    dbg = snap.get("debug") or {}
    pt = dict(nav.get("physical_trajectory") or dbg.get("physical_trajectory") or {})
    lp = dict(nav.get("local_plan") or dbg.get("local_plan") or {})
    exec_path = nav.get("local_path") or []
    breadcrumb = dbg.get("breadcrumb") or (dbg.get("phase4") or {}).get("breadcrumb") or {}

    display = classify_display_source(snap)
    issues: List[str] = []
    status = "PASS"

    if display == TRAJ_KIND_EXECUTED_KINEMATIC:
        issues.append("DISPLAY_FALLBACK_EXECUTED_KINEMATIC_NOT_FUTURE")
        status = "WARN"

    enriched_pt = None
    if pt and (pt.get("poses") or pt.get("centerline")):
        enriched_pt = enrich_trajectory_metadata(pt, vehicle=agv, now=tnow)
        integ = enriched_pt.get("integrity") or {}
        ae = integ.get("anchor_error_m")
        tol = integ.get("max_anchor_error_m")
        if ae is not None and tol is not None and ae > tol:
            issues.append("ANCHOR_ERROR_EXCEEDED")
            status = "FAIL"
        if integ.get("behind_vehicle") and not integ.get("stale_reanchor_applied"):
            issues.append("TRAJECTORY_BEHIND_VEHICLE")
            status = "FAIL"
        if integ.get("stale"):
            issues.append("STALE_TRAJECTORY")
            if status != "FAIL":
                status = "WARN"
    elif nav.get("mode") in ("tracking", "avoid"):
        issues.append("NO_FUTURE_PHYSICAL_TRAJECTORY")
        status = "UNVERIFIED"

    lp_enriched = None
    if lp.get("poses"):
        lp_enriched = enrich_trajectory_metadata(
            {**lp, "trajectory_kind": TRAJ_KIND_FUTURE_ROLLING, "trajectory_timestamp": lp.get("generated_at")},
            vehicle=agv,
            now=tnow,
        )

    gpath = nav.get("path") or []
    return {
        "status": status,
        "display_source": display,
        "frame_id": FRAME_MAP,
        "issues": issues,
        "physical_trajectory": enriched_pt,
        "local_plan": lp_enriched,
        "executed_kinematic_len": len(exec_path) if isinstance(exec_path, list) else 0,
        "breadcrumb_points": len((breadcrumb or {}).get("points") or []),
        "global_path_fingerprint": global_path_fingerprint(gpath),
        "global_path_revision": nav.get("global_path_revision"),
        "vehicle": {"x": agv.get("x"), "y": agv.get("y"), "yaw": agv.get("angle"), "vx": agv.get("vx"), "w": agv.get("w")},
        "collision": (snap.get("safety") or {}).get("collision"),
        "stop_reason": nav.get("stop_reason") or (snap.get("safety") or {}).get("stop_reason"),
    }
