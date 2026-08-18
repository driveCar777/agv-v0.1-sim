"""M3.8.1 trajectory direction forensics.

Map frame: heading = (cos(yaw), sin(yaw)) — confirmed by
nav_geometry body +x forward / +y left, and physics integrate:
  x += vx * cos(angle) * dt
  y += vx * sin(angle) * dt
+omega = CCW = LEFT (nav_kinematic.py).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _xy(p: Any) -> Optional[Pt]:
    if p is None:
        return None
    if isinstance(p, dict):
        if p.get("x") is None or p.get("y") is None:
            return None
        return float(p["x"]), float(p["y"])
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return float(p[0]), float(p[1])
    return None


def _norm(dx: float, dy: float) -> Optional[Pt]:
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return None
    return dx / n, dy / n


def heading_vector(yaw: float) -> Pt:
    return math.cos(float(yaw)), math.sin(float(yaw))


def extract_poses(traj: Any) -> List[Dict[str, Any]]:
    if not isinstance(traj, dict):
        return []
    raw = traj.get("poses") or traj.get("centerline") or []
    out: List[Dict[str, Any]] = []
    for p in raw:
        xy = _xy(p)
        if xy is None:
            continue
        item = {"x": xy[0], "y": xy[1]}
        if isinstance(p, dict):
            if p.get("yaw") is not None:
                item["yaw"] = float(p["yaw"])
            if p.get("t") is not None:
                item["t"] = float(p["t"])
        out.append(item)
    return out


def trajectory_vector(poses: Sequence[Dict[str, Any]], *, use_index: int = 1) -> Optional[Pt]:
    if len(poses) < 2:
        return None
    i = min(max(1, use_index), len(poses) - 1)
    p0 = poses[0]
    pi = poses[i]
    dx = pi["x"] - p0["x"]
    dy = pi["y"] - p0["y"]
    if math.hypot(dx, dy) < 1e-4 and len(poses) > i + 1:
        pi = poses[i + 1]
        dx = pi["x"] - p0["x"]
        dy = pi["y"] - p0["y"]
    return dx, dy


def direction_metrics(vehicle: Dict[str, Any], poses: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    vx = float(vehicle.get("x") or 0)
    vy = float(vehicle.get("y") or 0)
    yaw = float(vehicle.get("yaw") or vehicle.get("angle") or 0)
    hx, hy = heading_vector(yaw)
    v1 = trajectory_vector(poses, use_index=1)
    v2 = trajectory_vector(poses, use_index=2)
    vec = v2 if v2 is not None and math.hypot(*(v2)) >= 1e-4 else v1
    nvec = _norm(*(vec)) if vec else None
    dot = None
    ang = None
    if nvec is not None:
        dot = nvec[0] * hx + nvec[1] * hy
        cross = hx * nvec[1] - hy * nvec[0]
        ang = math.degrees(math.atan2(cross, dot))
    n = max(1, len(poses))
    fwd = 0
    back = 0
    lat = 0
    for p in poses:
        rx = p["x"] - vx
        ry = p["y"] - vy
        along = rx * hx + ry * hy
        if along > 0.05:
            fwd += 1
        elif along < -0.05:
            back += 1
        else:
            lat += 1
    p0 = poses[0] if poses else None
    ae = None
    ay = None
    if p0 is not None:
        ae = math.hypot(p0["x"] - vx, p0["y"] - vy)
        if p0.get("yaw") is not None:
            ay = abs(math.degrees(_wrap(float(p0["yaw"]) - yaw)))
    return {
        "anchor_error_m": None if ae is None else round(ae, 4),
        "anchor_yaw_error_deg": None if ay is None else round(ay, 2),
        "direction_dot": None if dot is None else round(dot, 4),
        "direction_angle_deg": None if ang is None else round(ang, 2),
        "forward_fraction": round(fwd / n, 3),
        "backward_fraction": round(back / n, 3),
        "lateral_fraction": round(lat / n, 3),
        "n_poses": len(poses),
        "vec_p1": None if v1 is None else (round(v1[0], 4), round(v1[1], 4)),
        "vec_p2": None if v2 is None else (round(v2[0], 4), round(v2[1], 4)),
    }


def classify_frame(
    *,
    metrics: Dict[str, Any],
    age_ms: Optional[float],
    frame_id: Optional[str],
    has_poses: bool,
    planner_period_s: float = 0.20,
) -> str:
    if frame_id and str(frame_id).lower() not in ("map", "world", ""):
        return "TRAJECTORY_FRAME_INVALID"
    if not has_poses:
        return "TRAJECTORY_UNKNOWN"
    if age_ms is not None and age_ms > (planner_period_s + 0.35) * 1000.0:
        return "TRAJECTORY_STALE"
    dot = metrics.get("direction_dot")
    if dot is None:
        return "TRAJECTORY_UNKNOWN"
    if dot < -0.3:
        return "TRAJECTORY_BACKWARD"
    if abs(dot) <= 0.3:
        return "TRAJECTORY_SIDEWAYS"
    return "TRAJECTORY_FORWARD"


HISTORY_SOURCES = ("HISTORICAL_RETREAT", "EXECUTED", "BREADCRUMB", "HISTORY")


def analyze_jsonl_row(row: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if row.get("event") and "vehicle_pose" not in row:
        return None
    if "vehicle_pose" not in row:
        return None
    pt = row.get("physical_trajectory") if isinstance(row.get("physical_trajectory"), dict) else {}
    poses = extract_poses(pt)
    vehicle = row.get("vehicle_pose") or {}
    metrics = direction_metrics(vehicle, poses) if poses else {
        "anchor_error_m": None,
        "anchor_yaw_error_deg": None,
        "direction_dot": None,
        "direction_angle_deg": None,
        "forward_fraction": None,
        "backward_fraction": None,
        "lateral_fraction": None,
        "n_poses": 0,
        "vec_p1": None,
        "vec_p2": None,
    }
    age = row.get("trajectory_age_ms")
    if age is None:
        integ = pt.get("integrity") or {}
        age = integ.get("trajectory_age_ms")
    src = str(pt.get("source") or row.get("trajectory_source") or "")
    kind = str(pt.get("trajectory_kind") or row.get("trajectory_kind") or "")
    cls = classify_frame(
        metrics=metrics,
        age_ms=float(age) if age is not None else None,
        frame_id=pt.get("frame_id") or row.get("trajectory_frame"),
        has_poses=bool(poses),
    )
    vx = float(row.get("vehicle_vx") or 0)
    omega = float(row.get("vehicle_omega") or row.get("mppi_omega") or 0)
    cmd_inc = False
    if vx > 0.04 and metrics.get("direction_dot") is not None and metrics["direction_dot"] < -0.3:
        cmd_inc = True
    omega_inc = False
    ang = metrics.get("direction_angle_deg")
    if omega is not None and ang is not None and abs(omega) > 0.02 and abs(ang) > 25.0:
        # +omega = LEFT = positive direction_angle (CCW)
        if omega > 0.02 and ang < -25.0:
            omega_inc = True
        if omega < -0.02 and ang > 25.0:
            omega_inc = True
    return {
        "seq": row.get("seq"),
        "ts": row.get("ts"),
        "scene": row.get("scene"),
        "scene_phase": row.get("scene_phase"),
        "classification": cls,
        "source": src,
        "trajectory_kind": kind,
        "history_source": src.upper() in HISTORY_SOURCES,
        "frame_id": pt.get("frame_id") or row.get("trajectory_frame"),
        "vehicle": {"x": vehicle.get("x"), "y": vehicle.get("y"), "yaw": vehicle.get("yaw"), "vx": vx, "omega": omega},
        "mppi_vx": row.get("mppi_vx"),
        "mppi_omega": row.get("mppi_omega"),
        "approved_vx": row.get("approved_vx"),
        "probe_side": row.get("probe_side"),
        "commit_side": row.get("commit_side"),
        "execution_side": row.get("execution_side") or row.get("corridor_side"),
        "reanchored": bool(row.get("trajectory_reanchored") or (pt.get("integrity") or {}).get("stale_reanchor_applied")),
        "reanchor_shift_m": row.get("reanchor_shift_m") or (pt.get("integrity") or {}).get("reanchor_shift_m"),
        "raw_age_ms": age,
        "collision": row.get("collision"),
        "command_trajectory_inconsistency": cmd_inc,
        "omega_trajectory_sign_inconsistency": omega_inc,
        **metrics,
    }


def summarize_rows(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    analyzed = [analyze_jsonl_row(r) for r in rows]
    analyzed = [a for a in analyzed if a is not None]
    counts: Dict[str, int] = {}
    src_counts: Dict[str, int] = {}
    first_lag = None
    first_dir_fail = None
    first_cmd_inc = None
    dots: List[float] = []
    ages: List[float] = []
    anchors: List[float] = []
    near_180 = 0
    vx_back = 0
    hist = 0
    reanchor_n = 0
    max_shift = 0.0
    collisions = 0
    pose_rows = 0
    unknown = 0
    for a in analyzed:
        counts[a["classification"]] = counts.get(a["classification"], 0) + 1
        src = a.get("source") or ""
        src_counts[src] = src_counts.get(src, 0) + 1
        if a.get("history_source"):
            hist += 1
        if a.get("reanchored"):
            reanchor_n += 1
            max_shift = max(max_shift, float(a.get("reanchor_shift_m") or 0))
        if a.get("collision"):
            collisions += 1
        if a["n_poses"] > 0:
            pose_rows += 1
        else:
            unknown += 1
        if a.get("direction_dot") is not None:
            dots.append(float(a["direction_dot"]))
        if a.get("raw_age_ms") is not None:
            ages.append(float(a["raw_age_ms"]))
        if a.get("anchor_error_m") is not None:
            anchors.append(float(a["anchor_error_m"]))
        ang = a.get("direction_angle_deg")
        if ang is not None and abs(abs(ang) - 180.0) < 25.0:
            near_180 += 1
        if a.get("command_trajectory_inconsistency"):
            vx_back += 1
            if first_cmd_inc is None:
                first_cmd_inc = a["seq"]
        if first_dir_fail is None and a["classification"] == "TRAJECTORY_BACKWARD":
            first_dir_fail = a["seq"]
        if first_lag is None and (
            (a.get("reanchor_shift_m") or 0) > 0.25
            or a["classification"] == "TRAJECTORY_STALE"
            or (a.get("backward_fraction") or 0) > 0.5
        ):
            first_lag = {
                "seq": a["seq"],
                "ts": a["ts"],
                "classification": a["classification"],
                "source": a.get("source"),
                "anchor_error_m": a.get("anchor_error_m"),
                "direction_angle_deg": a.get("direction_angle_deg"),
                "direction_dot": a.get("direction_dot"),
                "raw_age_ms": a.get("raw_age_ms"),
                "reanchor_shift_m": a.get("reanchor_shift_m"),
                "event": "TRAJECTORY_LAG_EVENT",
            }
    return {
        "samples_with_pose": len(analyzed),
        "samples_with_trajectory_poses": pose_rows,
        "classification_counts": counts,
        "source_counts": src_counts,
        "history_source_frames": hist,
        "reanchored_frames": reanchor_n,
        "max_reanchor_shift_m": round(max_shift, 4),
        "direction_dot_min": None if not dots else round(min(dots), 4),
        "direction_dot_mean": None if not dots else round(sum(dots) / len(dots), 4),
        "direction_angle_near_180_count": near_180,
        "vx_positive_backward_count": vx_back,
        "trajectory_age_ms_max": None if not ages else round(max(ages), 1),
        "anchor_error_m_max": None if not anchors else round(max(anchors), 4),
        "collisions": collisions,
        "first_trajectory_lag": first_lag,
        "first_direction_failure_seq": first_dir_fail,
        "first_command_inconsistency_seq": first_cmd_inc,
        "frames": analyzed,
    }
