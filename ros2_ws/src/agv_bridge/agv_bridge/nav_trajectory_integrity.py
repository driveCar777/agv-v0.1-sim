"""M3.8 trajectory integrity — anchor, age, frame semantics (map-frame only)."""

from __future__ import annotations

import hashlib
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]

TRAJ_KIND_FUTURE_PHYSICAL = "FUTURE_LOCAL_PHYSICAL"
TRAJ_KIND_FORWARD_FUTURE = "FORWARD_FUTURE"
TRAJ_KIND_BACKWARD_FUTURE = "BACKWARD_FUTURE"
TRAJ_KIND_HISTORICAL_RETREAT = "HISTORICAL_RETREAT"
TRAJ_KIND_FUTURE_ROLLING = "FUTURE_ROLLING_LOCAL_PLAN"
TRAJ_KIND_EXECUTED_KINEMATIC = "EXECUTED_KINEMATIC_BAND"
TRAJ_KIND_HISTORY_BREADCRUMB = "HISTORY_BREADCRUMB"
TRAJ_KIND_GLOBAL = "GLOBAL_REFERENCE"
TRAJ_KIND_NONE = "NONE"
TRAJ_KIND_STALE = "STALE_TRAJECTORY"

FRAME_MAP = "map"
REVERSE_ARC_IMPLEMENTED = False
PLANNER_PERIOD_S = 0.20
CONTROL_LATENCY_S = 0.10
LOCAL_PLANNER_STALE_MS = 5000.0  # moving + age above → LOCAL_PLANNER_STALE


def max_trajectory_age_ms(
    *,
    planner_period_s: float = PLANNER_PERIOD_S,
    actual_planner_compute_s: Optional[float] = None,
    planner_start_timestamp: Optional[float] = None,
    planner_finish_timestamp: Optional[float] = None,
    control_latency_s: float = CONTROL_LATENCY_S,
) -> float:
    """Freshness gate derived from measured planner compute + period + control latency."""
    compute_s = actual_planner_compute_s
    if compute_s is None and planner_start_timestamp and planner_finish_timestamp:
        pst, pft = float(planner_start_timestamp), float(planner_finish_timestamp)
        if pft > pst:
            compute_s = pft - pst
    if compute_s is not None and compute_s > 0:
        # Valid until next planner period completes + control delivery slack.
        return (float(compute_s) + float(planner_period_s) + float(control_latency_s)) * 1000.0
    return (float(planner_period_s) + 0.35) * 1000.0

_FUTURE_SOURCES = {"FORWARD", "LEFT", "RIGHT", "FORWARD_FUTURE"}
_BACKWARD_SOURCES = {"BACKWARD", "REVERSE", "BACKWARD_FUTURE"}
_HISTORY_SOURCES = {"HISTORICAL_RETREAT", "EXECUTED", "BREADCRUMB", "HISTORY"}


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


def classify_source_kind(source: Any) -> str:
    src = str(source or "").upper()
    if src in _HISTORY_SOURCES or "RETREAT" in src:
        return TRAJ_KIND_HISTORICAL_RETREAT
    if src in _BACKWARD_SOURCES:
        return TRAJ_KIND_BACKWARD_FUTURE
    if src in _FUTURE_SOURCES:
        return TRAJ_KIND_FORWARD_FUTURE
    if not src or src in ("NONE", "NULL"):
        return TRAJ_KIND_NONE
    return TRAJ_KIND_NONE


def control_eligible_for_kind(kind: str) -> bool:
    if kind == TRAJ_KIND_FORWARD_FUTURE:
        return True
    if kind == TRAJ_KIND_BACKWARD_FUTURE:
        return False  # REVERSE_ARC = NOT IMPLEMENTED
    return False


def classify_display_source(snap: Dict[str, Any]) -> str:
    nav = snap.get("nav") or {}
    pt = nav.get("physical_trajectory") or {}
    kind = str(pt.get("trajectory_kind") or "")
    if (
        pt
        and (pt.get("poses") or pt.get("centerline"))
        and pt.get("control_eligible") is True
        and kind == TRAJ_KIND_FORWARD_FUTURE
    ):
        return TRAJ_KIND_FORWARD_FUTURE
    retreat = nav.get("retreat_trajectory") or nav.get("historical_retreat") or {}
    if retreat and (retreat.get("poses") or retreat.get("centerline")):
        rk = str(retreat.get("trajectory_kind") or classify_source_kind(retreat.get("source")))
        if rk == TRAJ_KIND_HISTORICAL_RETREAT:
            return TRAJ_KIND_HISTORICAL_RETREAT
        if rk == TRAJ_KIND_BACKWARD_FUTURE:
            return TRAJ_KIND_BACKWARD_FUTURE
    lp = nav.get("local_plan") or {}
    if lp.get("poses") and lp.get("active") is not False:
        return TRAJ_KIND_FUTURE_ROLLING
    if nav.get("local_path"):
        return TRAJ_KIND_EXECUTED_KINEMATIC
    return TRAJ_KIND_NONE


def max_reanchor_shift_m(
    *,
    vehicle_speed_mps: float,
    planner_period_s: float = PLANNER_PERIOD_S,
    control_latency_s: float = CONTROL_LATENCY_S,
) -> float:
    """Derived max translation. 2m-class shifts are STALE, not reanchorable."""
    return max_anchor_tolerance_m(
        vehicle_speed_mps=vehicle_speed_mps,
        planner_period_s=planner_period_s,
        control_latency_s=control_latency_s,
        min_m=0.08,
        max_m=0.40,
    )


def reanchor_trajectory_poses(traj: Dict[str, Any], vehicle: Dict[str, Any]) -> Dict[str, Any]:
    """Optional small map-frame nudge. Caller must already have passed freshness checks."""
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
    planner_period_s: float = PLANNER_PERIOD_S,
    actual_planner_compute_s: Optional[float] = None,
    current_scene_id: Optional[Any] = None,
    current_cycle_id: Optional[Any] = None,
) -> Dict[str, Any]:
    """Attach integrity fields. Does NOT reanchor stale / history / backward into a fake future."""
    out = dict(traj or {})
    tnow = float(now or time.time())
    src = out.get("source") or out.get("trajectory_source")
    kind = classify_source_kind(src) if src else classify_source_kind(out.get("trajectory_kind"))
    if str(out.get("trajectory_kind") or "").upper() in (
        TRAJ_KIND_HISTORICAL_RETREAT,
        TRAJ_KIND_BACKWARD_FUTURE,
        TRAJ_KIND_FORWARD_FUTURE,
    ):
        kind = str(out["trajectory_kind"]).upper()
    eligible = control_eligible_for_kind(kind)
    viz_only = not eligible
    out["frame_id"] = FRAME_MAP
    out["trajectory_kind"] = kind
    out["trajectory_source"] = str(src or kind)
    out["control_eligible"] = eligible
    out["visualization_only"] = viz_only
    generated = (
        out.get("trajectory_generated_at")
        or out.get("planner_finish_timestamp")
        or out.get("generated_at")
        or out.get("trajectory_timestamp")
    )
    out["sample_timestamp"] = tnow
    out["vehicle_timestamp"] = tnow
    if generated is not None:
        out["trajectory_generated_at"] = float(generated)
        out["trajectory_timestamp"] = float(generated)
    else:
        out["trajectory_timestamp"] = None
        out["timestamp_status"] = "CALIBRATION_REQUIRED"
    spd = abs(float(vehicle.get("vx") or vehicle.get("speed") or 0))
    omega = float(vehicle.get("w") or 0)
    ae_raw = anchor_error_m(vehicle, out)
    max_shift = max_reanchor_shift_m(vehicle_speed_mps=spd, planner_period_s=planner_period_s)
    age = trajectory_age_ms(out, tnow) if generated is not None else None
    compute_s = actual_planner_compute_s
    if compute_s is None:
        pst = out.get("planner_start_timestamp")
        pft = out.get("planner_finish_timestamp")
        if pst and pft and float(pft) > float(pst):
            compute_s = float(pft) - float(pst)
    max_age_ms = max_trajectory_age_ms(
        planner_period_s=planner_period_s,
        actual_planner_compute_s=compute_s,
        planner_start_timestamp=out.get("planner_start_timestamp"),
        planner_finish_timestamp=out.get("planner_finish_timestamp"),
    )
    stale_reasons: List[str] = []
    if age is not None and age > LOCAL_PLANNER_STALE_MS:
        stale_reasons.append("PLANNER_REALTIME")
        out["local_planner_stale"] = True
    if kind in (TRAJ_KIND_HISTORICAL_RETREAT, TRAJ_KIND_BACKWARD_FUTURE, TRAJ_KIND_NONE):
        eligible = False
        viz_only = True
        out["control_eligible"] = False
        out["visualization_only"] = True
    if age is not None and age > max_age_ms:
        stale_reasons.append("AGE")
    if ae_raw is not None and ae_raw > max_shift:
        stale_reasons.append("ANCHOR")
    scene_ok = True
    if current_scene_id is not None and out.get("scene_id") is not None:
        if str(out.get("scene_id")) != str(current_scene_id):
            stale_reasons.append("SCENE")
            scene_ok = False
            out["integrity_reject"] = "TRAJECTORY_STALE_SCENE"
    if current_scene_id is not None and out.get("scene_id") is None:
        stale_reasons.append("SCENE")
        scene_ok = False
        out["integrity_reject"] = "TRAJECTORY_STALE_SCENE"
    if current_cycle_id is not None and out.get("planner_cycle_id") is not None:
        if int(out.get("planner_cycle_id") or -1) != int(current_cycle_id):
            out["cycle_mismatch"] = True
            stale_reasons.append("CYCLE")
            out["integrity_reject"] = out.get("integrity_reject") or "TRAJECTORY_CYCLE_MISMATCH"
            eligible = False
            out["control_eligible"] = False
            out["visualization_only"] = True
    # Optional small reanchor only for valid forward future within derived shift.
    if (
        eligible
        and not stale_reasons
        and ae_raw is not None
        and 1e-4 < ae_raw <= max_shift
        and spd > 0.04
    ):
        out = reanchor_trajectory_poses(out, vehicle)
        out["stale_reanchor_applied"] = False
        out["small_reanchor_applied"] = True
    elif stale_reasons:
        out["stale_reanchor_applied"] = False
        out["control_eligible"] = False
        out["visualization_only"] = True
        if "SCENE" in stale_reasons:
            out["integrity_reject"] = "TRAJECTORY_STALE_SCENE"
        elif "CYCLE" in stale_reasons:
            out["integrity_reject"] = "TRAJECTORY_CYCLE_MISMATCH"
        elif "ANCHOR" in stale_reasons or "AGE" in stale_reasons:
            out["trajectory_kind"] = TRAJ_KIND_STALE if kind == TRAJ_KIND_FORWARD_FUTURE else kind
            out["integrity_reject"] = "STALE_TRAJECTORY"
    ae = anchor_error_m(vehicle, out)
    ay = anchor_yaw_error_deg(vehicle, out)
    direction_dot = None
    direction_angle = None
    try:
        from agv_bridge.nav_trajectory_direction import direction_metrics

        raw_poses = out.get("poses") or out.get("centerline") or []
        dposes = []
        for p in raw_poses:
            if isinstance(p, dict) and p.get("x") is not None and p.get("y") is not None:
                dposes.append(p)
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                dposes.append({"x": float(p[0]), "y": float(p[1])})
        if len(dposes) >= 2:
            dm = direction_metrics(vehicle, dposes)
            direction_dot = dm.get("direction_dot")
            direction_angle = dm.get("direction_angle_deg")
    except Exception:
        pass
    out["control_eligible"] = bool(out.get("control_eligible"))
    out["visualization_only"] = not bool(out.get("control_eligible"))
    out["trajectory_is_control_eligible"] = out["control_eligible"]
    out["trajectory_is_visualization_only"] = out["visualization_only"]
    out["direction_dot"] = direction_dot
    out["direction_angle"] = direction_angle
    out["integrity"] = {
        "anchor_error_m": None if ae is None else round(ae, 4),
        "raw_anchor_error_m": None if ae_raw is None else round(ae_raw, 4),
        "anchor_yaw_error_deg": None if ay is None else round(ay, 2),
        "trajectory_age_ms": None if age is None else round(age, 1),
        "planner_age_ms": None if age is None else round(age, 1),
        "max_anchor_error_m": round(max_shift, 4),
        "max_reanchor_shift_m": round(max_shift, 4),
        "max_anchor_yaw_deg": round(
            max_anchor_yaw_deg(vehicle_omega_rad_s=omega, planner_period_s=planner_period_s), 2
        ),
        "max_age_ms": round(max_age_ms, 1),
        "planner_compute_ms": None if compute_s is None else round(float(compute_s) * 1000.0, 1),
        "local_planner_stale": bool(out.get("local_planner_stale")),
        "stale": bool(stale_reasons),
        "stale_reasons": stale_reasons,
        "stale_reanchor_applied": False,
        "small_reanchor_applied": bool(out.get("small_reanchor_applied")),
        "reanchor_shift_m": out.get("reanchor_shift_m"),
        "behind_vehicle": _trajectory_behind_vehicle(vehicle, out),
        "scene_ok": scene_ok,
        "control_eligible": bool(out.get("control_eligible")),
        "integrity_reject": out.get("integrity_reject"),
        "direction_dot": direction_dot,
        "direction_angle_deg": direction_angle,
        "planner_cycle_id": out.get("planner_cycle_id"),
        "scene_id": out.get("scene_id"),
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
    if pt and (pt.get("poses") or pt.get("centerline") or pt.get("trajectory_kind")):
        enriched_pt = enrich_trajectory_metadata(
            pt,
            vehicle=agv,
            now=tnow,
            current_scene_id=nav.get("nav_scene_id") or snap.get("nav_scene_id"),
            current_cycle_id=nav.get("planner_cycle_id") or snap.get("planner_cycle_id"),
        )
        integ = enriched_pt.get("integrity") or {}
        kind = str(enriched_pt.get("trajectory_kind") or "")
        if kind == TRAJ_KIND_HISTORICAL_RETREAT and enriched_pt.get("control_eligible"):
            issues.append("HISTORY_MARKED_CONTROL_ELIGIBLE")
            status = "FAIL"
        if kind == TRAJ_KIND_BACKWARD_FUTURE and enriched_pt.get("control_eligible"):
            issues.append("BACKWARD_MARKED_CONTROL_ELIGIBLE")
            status = "FAIL"
        if kind == TRAJ_KIND_STALE:
            issues.append("STALE_TRAJECTORY")
            if status != "FAIL":
                status = "WARN"
        if integ.get("stale_reanchor_applied"):
            issues.append("STALE_REANCHOR_MASK")
            status = "FAIL"
        ae = integ.get("raw_anchor_error_m") if integ.get("raw_anchor_error_m") is not None else integ.get("anchor_error_m")
        tol = integ.get("max_reanchor_shift_m") or integ.get("max_anchor_error_m")
        if (
            enriched_pt.get("control_eligible")
            and ae is not None
            and tol is not None
            and ae > tol
        ):
            issues.append("ANCHOR_ERROR_EXCEEDED")
            status = "FAIL"
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
