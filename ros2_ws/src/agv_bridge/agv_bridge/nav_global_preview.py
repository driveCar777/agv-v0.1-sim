"""P0-B Global Reference Preview — REFERENCE ONLY.

Does NOT produce cmd_vel. Does NOT bypass Safety / FSM / Local Planner.
Kinematic feasibility is P0-C: kinematic_valid is always None here.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_footprint import SWEPT_SPATIAL_STEP_M, swept_boundary_edges
from agv_bridge.nav_geometry import DEFAULT_GEOM, get_vehicle_geometry
from agv_bridge.path_progress import project_pose_to_path

Pt = Tuple[float, float]

GLOBAL_PREVIEW_MIN_M = 2.0
GLOBAL_PREVIEW_NORMAL_M = 5.0
GLOBAL_PREVIEW_MAX_M = 8.0
DENSIFY_SPACING_M = 0.10
FIRST_TURN_DEG = 15.0
FIRST_TURN_PERSIST_M = 0.40
DISPLAY_SWEPT_STEP_M = 0.28  # lightweight display geometry, not P0-C proof

STATUS_REFERENCE = "REFERENCE_ONLY"
STATUS_NO_PATH = "NO_GLOBAL_PATH"
STATUS_INVALID = "INVALID_SOURCE"
STATUS_GOAL = "GOAL_REACHED"
STATUS_DEGRADED = "DEGRADED"

REASON_NORMAL = "NORMAL_5M"
REASON_GOAL = "GOAL_LIMITED"
REASON_PATH = "PATH_LIMITED"


def preview_enabled() -> bool:
    v = (os.environ.get("NAV_GLOBAL_PREVIEW", "1") or "1").strip().lower()
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


def densify_polyline(path: Sequence[Any], *, spacing_m: float = DENSIFY_SPACING_M) -> List[Dict[str, float]]:
    """Insert samples so consecutive points are <= spacing_m. Yaw = path tangent."""
    pts: List[Pt] = []
    for p in path or []:
        xy = _as_xy(p)
        if xy is not None:
            pts.append(xy)
    if not pts:
        return []
    out: List[Dict[str, float]] = []
    s = 0.0
    step = max(0.04, float(spacing_m))
    for i, (x, y) in enumerate(pts):
        if i == 0:
            yaw = 0.0
            if len(pts) > 1:
                yaw = math.atan2(pts[1][1] - y, pts[1][0] - x)
            out.append({"x": x, "y": y, "yaw": yaw, "s": 0.0})
            continue
        px, py = pts[i - 1]
        dx, dy = x - px, y - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            continue
        yaw = math.atan2(dy, dx)
        n = max(1, int(math.ceil(dist / step)))
        for k in range(1, n + 1):
            t = k / n
            xx = px + dx * t
            yy = py + dy * t
            s += dist / n
            out.append({"x": xx, "y": yy, "yaw": yaw, "s": s})
    # fix last yaw
    if len(out) >= 2:
        out[-1]["yaw"] = out[-2]["yaw"]
    return out


def adaptive_preview_m(
    *,
    remaining_path_m: float,
    remaining_goal_m: Optional[float],
    min_m: float = GLOBAL_PREVIEW_MIN_M,
    normal_m: float = GLOBAL_PREVIEW_NORMAL_M,
    max_m: float = GLOBAL_PREVIEW_MAX_M,
) -> Tuple[float, str]:
    """Return (preview_m, reason). Never invents path beyond remaining."""
    rem = max(0.0, float(remaining_path_m))
    goal = None if remaining_goal_m is None else max(0.0, float(remaining_goal_m))
    target = float(normal_m)
    reason = REASON_NORMAL
    cap = rem
    if goal is not None:
        cap = min(cap, goal)
    if cap + 1e-6 < target:
        target = cap
        if goal is not None and goal <= rem + 1e-6 and goal < float(normal_m):
            reason = REASON_GOAL
        else:
            reason = REASON_PATH
    target = min(target, float(max_m), cap)
    # MIN is a normal-planning floor, not a reason to fabricate path
    if cap >= float(min_m) and target + 1e-9 < float(min_m) and reason == REASON_NORMAL:
        target = float(min_m)
    return max(0.0, target), reason


def first_meaningful_turn(
    poses: Sequence[Dict[str, Any]],
    *,
    angle_deg: float = FIRST_TURN_DEG,
    persist_m: float = FIRST_TURN_PERSIST_M,
) -> Tuple[Optional[float], Optional[float]]:
    """Ignore A* sawtooth: require heading delta > angle_deg sustained over persist_m.

    Returns (first_turn_distance_m, heading_change_deg) from preview start.
    """
    if not poses or len(poses) < 3:
        return None, None
    thr = math.radians(float(angle_deg))
    persist = max(0.15, float(persist_m))
    yaw0 = float(poses[0].get("yaw") or 0.0)
    run_s0: Optional[float] = None
    run_abs = 0.0
    for p in poses[1:]:
        yaw = float(p.get("yaw") or 0.0)
        s = float(p.get("s") or 0.0) - float(poses[0].get("s") or 0.0)
        d = abs(_wrap(yaw - yaw0))
        if d >= thr:
            if run_s0 is None:
                run_s0 = s
                run_abs = d
            else:
                run_abs = max(run_abs, d)
            if s - run_s0 >= persist:
                return round(run_s0, 3), round(math.degrees(run_abs), 1)
        else:
            run_s0 = None
            run_abs = 0.0
    return None, None


def slice_forward(
    densified: Sequence[Dict[str, float]],
    *,
    s_start: float,
    preview_m: float,
) -> List[Dict[str, float]]:
    """Take densified samples from s_start forward by preview_m (not behind)."""
    if not densified:
        return []
    s0 = max(0.0, float(s_start))
    s1 = s0 + max(0.0, float(preview_m))
    # start at first sample with s >= s0 (monotonic forward)
    i0 = 0
    for i, p in enumerate(densified):
        if float(p.get("s") or 0.0) >= s0 - 1e-6:
            i0 = i
            break
    else:
        i0 = len(densified) - 1
    out: List[Dict[str, float]] = []
    for p in densified[i0:]:
        s = float(p.get("s") or 0.0)
        if s > s1 + 1e-6:
            break
        out.append(
            {
                "x": round(float(p["x"]), 3),
                "y": round(float(p["y"]), 3),
                "yaw": round(float(p.get("yaw") or 0.0), 4),
                "s": round(s, 3),
                "distance_along_path": round(s - s0, 3),
            }
        )
    return out


def _path_fingerprint(path: Sequence[Any]) -> Tuple[Any, ...]:
    pts: List[Pt] = []
    for p in path or []:
        xy = _as_xy(p)
        if xy is not None:
            pts.append((round(xy[0], 3), round(xy[1], 3)))
    if not pts:
        return (0,)
    mid = pts[len(pts) // 2]
    return (len(pts), pts[0], mid, pts[-1])


@dataclass
class GlobalPreviewCache:
    fingerprint: Tuple[Any, ...] = ()
    densified: List[Dict[str, float]] = field(default_factory=list)
    path_length_m: float = 0.0

    def ensure(self, path: Sequence[Any]) -> List[Dict[str, float]]:
        fp = _path_fingerprint(path)
        if fp != self.fingerprint:
            self.densified = densify_polyline(path)
            self.fingerprint = fp
            self.path_length_m = float(self.densified[-1]["s"]) if self.densified else 0.0
        return self.densified


_CACHE = GlobalPreviewCache()


def empty_reference(*, status: str, reason: str = "NONE", revision: int = 0) -> Dict[str, Any]:
    return {
        "status": status,
        "geometry_status": STATUS_REFERENCE,
        "preview_m": 0.0,
        "remaining_m": 0.0,
        "path_length_m": 0.0,
        "path_exists": status not in (STATUS_NO_PATH, STATUS_GOAL),
        "preview_reason": reason,
        "preview_point_count": 0,
        "poses": [],
        "centerline": [],
        "left_edge": [],
        "right_edge": [],
        "first_turn_distance_m": None,
        "heading_change_deg": None,
        "max_heading_change_deg": None,
        "max_curvature": None,
        "kinematic_valid": None,
        "max_curvature_validated": None,
        "required_w_validated": None,
        "swept_collision_validated": None,
        "path_revision": int(revision),
        "note": "Global Reference Preview — unvalidated; not a certified safe/feasible path",
        "controls_vehicle": False,
    }


def build_global_reference(
    *,
    path: Sequence[Any],
    x: float,
    y: float,
    yaw: float,
    path_progress_s: Optional[float] = None,
    path_index: int = 0,
    goal_distance_m: Optional[float] = None,
    path_revision: int = 0,
    goal_reached: bool = False,
    cache: Optional[GlobalPreviewCache] = None,
) -> Dict[str, Any]:
    """Slice adaptive 2–8 m forward preview. Never used as cmd_vel."""
    if goal_reached:
        return empty_reference(status=STATUS_GOAL, reason="GOAL_REACHED", revision=path_revision)
    if not path:
        return empty_reference(status=STATUS_NO_PATH, reason="NO_GLOBAL_PATH", revision=path_revision)
    try:
        c = cache or _CACHE
        densified = c.ensure(path)
        if len(densified) < 2:
            return empty_reference(status=STATUS_INVALID, reason="INVALID_SOURCE", revision=path_revision)
        orig: List[Pt] = []
        for p in path:
            xy = _as_xy(p)
            if xy is not None:
                orig.append(xy)
        prog = project_pose_to_path(x, y, yaw, orig, hint_index=max(0, int(path_index)))
        # Monotonic preference: do not roll preview backward along the path
        s_raw = float(prog.s)
        if path_progress_s is not None:
            s_use = max(s_raw, float(path_progress_s) - 0.15)
        else:
            s_use = s_raw
        remaining_path = max(0.0, float(c.path_length_m) - s_use)
        preview_m, reason = adaptive_preview_m(
            remaining_path_m=remaining_path, remaining_goal_m=goal_distance_m
        )
        if goal_distance_m is not None and float(goal_distance_m) < 0.28:
            return empty_reference(status=STATUS_GOAL, reason="GOAL_REACHED", revision=path_revision)
        poses = slice_forward(densified, s_start=s_use, preview_m=preview_m)
        if not poses:
            return empty_reference(status=STATUS_DEGRADED, reason="PATH_LIMITED", revision=path_revision)
        actual_len = 0.0
        if len(poses) >= 2:
            actual_len = float(poses[-1]["s"]) - float(poses[0]["s"])
        turn_d, turn_deg = first_meaningful_turn(poses)
        # Lightweight display swept (downsampled) — REFERENCE_ONLY
        left: List[Pt] = []
        right: List[Pt] = []
        try:
            geom = get_vehicle_geometry() or DEFAULT_GEOM
            sample = poses[:: max(1, int(DISPLAY_SWEPT_STEP_M / max(DENSIFY_SPACING_M, 1e-3)))]
            if sample[-1] is not poses[-1]:
                sample = list(sample) + [poses[-1]]

            class _P:
                __slots__ = ("x", "y", "yaw")

                def __init__(self, q: Dict[str, float]) -> None:
                    self.x = float(q["x"])
                    self.y = float(q["y"])
                    self.yaw = float(q.get("yaw") or 0.0)

            left, right, _poly = swept_boundary_edges(
                [_P(q) for q in sample],
                geom,
                margin_m=float(getattr(geom, "safety_margin_m", 0.08) or 0.08),
                spatial_step_m=max(SWEPT_SPATIAL_STEP_M, DISPLAY_SWEPT_STEP_M),
            )
        except Exception:
            left, right = [], []
        status = STATUS_REFERENCE
        return {
            "status": status,
            "geometry_status": STATUS_REFERENCE,
            "preview_m": round(actual_len if actual_len > 0 else preview_m, 3),
            "requested_preview_m": round(preview_m, 3),
            "remaining_m": round(remaining_path, 3),
            "path_length_m": round(float(c.path_length_m), 3),
            "path_exists": True,
            "preview_reason": reason,
            "preview_point_count": len(poses),
            "poses": poses,
            "centerline": [{"x": p["x"], "y": p["y"]} for p in poses],
            "left_edge": [{"x": round(a, 3), "y": round(b, 3)} for a, b in left[:80]],
            "right_edge": [{"x": round(a, 3), "y": round(b, 3)} for a, b in right[:80]],
            "first_turn_distance_m": turn_d,
            "heading_change_deg": turn_deg,
            "max_heading_change_deg": turn_deg,
            "max_curvature": None,
            "kinematic_valid": None,
            "max_curvature_validated": None,
            "required_w_validated": None,
            "swept_collision_validated": None,
            "path_revision": int(path_revision),
            "s_start": round(s_use, 3),
            "start_offset_m": round(math.hypot(float(poses[0]["x"]) - x, float(poses[0]["y"]) - y), 3),
            "note": "Global Reference Preview — unvalidated; not a certified safe/feasible path",
            "controls_vehicle": False,
        }
    except Exception:
        out = empty_reference(status=STATUS_DEGRADED, reason="INVALID_SOURCE", revision=path_revision)
        out["path_exists"] = True
        return out


def collect_local_candidates(
    *,
    maneuver: Optional[Dict[str, Any]] = None,
    path_candidates: Optional[Sequence[Any]] = None,
    selected: Any = None,
) -> Dict[str, Any]:
    """Read-only packaging of existing local candidates. Does not rescore."""
    man = maneuver or {}
    lc = man.get("local_compare") or {}
    items: List[Dict[str, Any]] = []
    cands = lc.get("candidates")
    path_by_kind = {
        "LEFT": lc.get("left_path") or [],
        "RIGHT": lc.get("right_path") or [],
        "FORWARD": lc.get("forward_path") or [],
    }
    if isinstance(cands, dict):
        for k, v in cands.items():
            if not isinstance(v, dict):
                continue
            kind = str(v.get("type") or k)
            poses = v.get("path") or path_by_kind.get(kind.upper(), []) or []
            dist = _polyline_len(poses)
            valid = bool(v.get("feasible", True)) and not bool(v.get("collision"))
            items.append(
                {
                    "candidate_id": kind,
                    "kind": kind,
                    "valid": valid,
                    "selected": bool(v.get("selected")) or str(selected or "").upper() == kind.upper(),
                    "reject_reasons": []
                    if valid
                    else ([v.get("reason")] if v.get("reason") else ["UNKNOWN"]),
                    "invalid_reason": None if valid else (v.get("reason") or "UNKNOWN"),
                    "score": v.get("total_cost"),
                    "score_breakdown": v.get("cost_breakdown") or {},
                    "distance_m": round(dist, 3) if dist else v.get("path_progress_gain"),
                    "duration_s": v.get("duration"),
                    "poses": _poses_xy(poses),
                    "requested_vx": v.get("vx"),
                    "requested_w": v.get("w"),
                    "collision": bool(v.get("collision")),
                }
            )
    if not items:
        for row in lc.get("rows") or []:
            if not isinstance(row, dict):
                continue
            kind = str(row.get("action") or row.get("type") or "UNK")
            poses = path_by_kind.get(kind.upper(), [])
            valid = bool(row.get("feasible", True))
            items.append(
                {
                    "candidate_id": kind,
                    "kind": kind,
                    "valid": valid,
                    "selected": bool(row.get("selected")) or str(selected or "").upper() == kind.upper(),
                    "reject_reasons": [] if valid else ([row.get("reason")] if row.get("reason") else []),
                    "invalid_reason": None if valid else row.get("reason"),
                    "score": row.get("cost"),
                    "distance_m": round(_polyline_len(poses), 3),
                    "poses": _poses_xy(poses),
                }
            )
    if not items and path_candidates:
        for i, c in enumerate(path_candidates):
            if not isinstance(c, dict):
                continue
            poses = c.get("path") or []
            items.append(
                {
                    "candidate_id": str(c.get("id") or c.get("type") or f"C{i}"),
                    "kind": str(c.get("type") or c.get("mode") or "LOCAL"),
                    "valid": not bool(c.get("collision")),
                    "selected": bool(c.get("selected")),
                    "reject_reasons": ["COLLISION"] if c.get("collision") else [],
                    "distance_m": round(_polyline_len(poses), 3),
                    "poses": _poses_xy(poses),
                    "requested_vx": c.get("vx"),
                    "requested_w": c.get("w"),
                }
            )
    dists = [float(it.get("distance_m") or 0.0) for it in items]
    valid_n = sum(1 for it in items if it.get("valid"))
    sel = next((it for it in items if it.get("selected")), None)
    return {
        "count": len(items),
        "valid_count": valid_n,
        "max_distance_m": round(max(dists) if dists else 0.0, 3),
        "mean_distance_m": round(sum(dists) / len(dists), 3) if dists else 0.0,
        "items": items,
        "selected_candidate": (sel or {}).get("candidate_id") or (str(selected) if selected else "NONE"),
    }


def _polyline_len(poses: Sequence[Any]) -> float:
    s = 0.0
    prev = None
    for p in poses or []:
        xy = _as_xy(p)
        if xy is None:
            continue
        if prev is not None:
            s += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        prev = xy
    return s


def _poses_xy(poses: Sequence[Any], limit: int = 24) -> List[Dict[str, float]]:
    out: List[Dict[str, float]] = []
    for p in list(poses or [])[:limit]:
        xy = _as_xy(p)
        if xy is None:
            continue
        row: Dict[str, float] = {"x": round(xy[0], 3), "y": round(xy[1], 3)}
        if isinstance(p, dict) and p.get("yaw") is not None:
            row["yaw"] = round(float(p["yaw"]), 4)
        out.append(row)
    return out


def expected_local_distance_m(
    *,
    state_vx: Optional[float],
    requested_vx: Optional[float],
    horizon_s: float = 1.5,
    fallback_speed: float = 0.15,
) -> float:
    sp = abs(float(state_vx)) if state_vx is not None else 0.0
    if sp < 0.05 and requested_vx is not None:
        sp = abs(float(requested_vx))
    if sp < 0.05:
        sp = float(fallback_speed)
    return max(0.0, sp * float(horizon_s))
