"""P1-2-OBSERVE — Pink lookahead / global-local reference conflict forensics.

Diagnostics only. Does NOT change planning, MPPI, Pure Pursuit, Safety, or FSM behavior.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]
ClearanceFn = Callable[[float, float], float]
CollideFn = Callable[[float, float], bool]

LOOKAHEAD_DISPLAY_M = 1.4
JUMP_DIST_THRESHOLD_M = 0.35
JUMP_S_THRESHOLD_M = 0.35
TURNBACK_W_THRESHOLD = 0.06
MARGIN_TOO_CLOSE_M = 0.12

SOURCE_GLOBAL_PATH = "GLOBAL_PATH"
SOURCE_GLOBAL_REFERENCE = "GLOBAL_REFERENCE"
SOURCE_LOCAL_PLAN = "LOCAL_PLAN"
SOURCE_MPPI_BEST = "MPPI_BEST_PATH"
SOURCE_PP_FOLLOW = "PP_FOLLOW_PATH"
SOURCE_DISPLAY = "DISPLAY_LOOKAHEAD"


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _deg(r: float) -> float:
    return math.degrees(r)


def _as_xy(path: Any) -> List[Pt]:
    out: List[Pt] = []
    for p in path or []:
        if isinstance(p, dict):
            out.append((float(p.get("x") or 0.0), float(p.get("y") or 0.0)))
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            out.append((float(p[0]), float(p[1])))
    return out


def _polyline_len(path: Sequence[Pt]) -> float:
    s = 0.0
    prev = None
    for p in path:
        if prev is not None:
            s += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return s


def _nearest_index(path: Sequence[Pt], x: float, y: float) -> int:
    best_i, best_d = 0, 1e18
    for i, p in enumerate(path):
        d = math.hypot(p[0] - x, p[1] - y)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def lookahead_on_polyline(
    path: Sequence[Pt],
    x: float,
    y: float,
    lookahead_m: float = LOOKAHEAD_DISPLAY_M,
) -> Dict[str, Any]:
    """Same nearest-index + arc-length walk as sim_api_ext._lookahead_point (rich metadata)."""
    out: Dict[str, Any] = {
        "exists": False,
        "x": None,
        "y": None,
        "lookahead_m": round(float(lookahead_m), 3),
        "path_index": None,
        "s_along_m": None,
        "distance_from_vehicle_m": None,
    }
    if not path or len(path) < 2:
        return out
    i0 = _nearest_index(path, x, y)
    acc = 0.0
    prev = path[i0]
    ti = i0
    target = path[min(i0 + 1, len(path) - 1)]
    for j, p in enumerate(path[i0:], start=i0):
        acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        ti = j
        target = p
        if acc >= lookahead_m:
            break
    tx, ty = float(target[0]), float(target[1])
    dist = math.hypot(tx - x, ty - y)
    out.update(
        {
            "exists": True,
            "x": round(tx, 4),
            "y": round(ty, 4),
            "path_index": int(ti),
            "s_along_m": round(acc, 4),
            "distance_from_vehicle_m": round(dist, 4),
        }
    )
    return out


def body_frame(x: float, y: float, yaw: float, px: float, py: float) -> Dict[str, float]:
    dx = px - x
    dy = py - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    dist = math.hypot(local_x, local_y)
    heading_err = math.atan2(local_y, local_x) if dist > 1e-6 else 0.0
    return {
        "body_x": round(local_x, 4),
        "body_y": round(local_y, 4),
        "distance_m": round(dist, 4),
        "heading_error": round(heading_err, 5),
        "heading_error_deg": round(_deg(heading_err), 3),
    }


def heading_toward(x: float, y: float, tx: float, ty: float, yaw: float) -> Dict[str, float]:
    dx = tx - x
    dy = ty - y
    if math.hypot(dx, dy) < 1e-6:
        return {"heading_rad": yaw, "heading_deg": round(_deg(yaw), 3), "error_deg": 0.0}
    h = math.atan2(dy, dx)
    err = _wrap(h - yaw)
    return {"heading_rad": round(h, 5), "heading_deg": round(_deg(h), 3), "error_deg": round(_deg(err), 3)}


def path_identity(
    *,
    source: str,
    path_id: Optional[str] = None,
    revision: Optional[int] = None,
    horizon_m: Optional[float] = None,
    generated_at: Optional[float] = None,
) -> Dict[str, Any]:
    return {
        "path_source": source,
        "path_id": path_id,
        "path_revision": revision,
        "generated_at": generated_at,
        "horizon_m": None if horizon_m is None else round(float(horizon_m), 3),
    }


def point_obstacle_diag(
    px: Optional[float],
    py: Optional[float],
    *,
    clearance_at: Optional[ClearanceFn] = None,
    collide: Optional[CollideFn] = None,
) -> Dict[str, Any]:
    if px is None or py is None:
        return {
            "lookahead_clearance_m": None,
            "lookahead_collision": None,
            "lookahead_inside_obstacle": None,
        }
    clr = float(clearance_at(px, py)) if clearance_at else None
    col = bool(collide(px, py)) if collide else None
    inside = bool(col) if col is not None else (clr is not None and clr < 0.0)
    return {
        "lookahead_clearance_m": None if clr is None else round(clr, 4),
        "lookahead_collision": col,
        "lookahead_inside_obstacle": inside,
    }


def distance_to_path_end(px: Optional[float], py: Optional[float], path: Sequence[Pt]) -> Optional[float]:
    if px is None or py is None or len(path) < 2:
        return None
    ex, ey = path[-1]
    return round(math.hypot(px - ex, py - ey), 4)


def classify_obstacle_pass_state(
    *,
    front_near: Optional[float],
    side_near: Optional[float],
    obstacle_clearance: Optional[float],
    local_kind: Optional[str],
) -> str:
    fn = _f(front_near)
    sn = _f(side_near)
    oc = _f(obstacle_clearance)
    kind = str(local_kind or "").upper()
    if fn is not None and fn > 2.5 and (oc is None or oc > 1.0):
        return "PASSED"
    if kind in ("LEFT_ARC", "RIGHT_ARC") and sn is not None and sn < 1.2:
        return "BESIDE"
    if fn is not None and fn < 1.0:
        if kind in ("LEFT_ARC", "RIGHT_ARC"):
            return "PASSING"
        return "APPROACHING"
    if fn is not None and fn < 2.0:
        return "APPROACHING"
    return "UNKNOWN"


def classify_reference_authority(
    *,
    tracking_local_plan: bool,
    follow_path_source: str,
    local_plan_active: bool,
    maneuver_authority: Optional[str],
    fsm_mode: str,
    recovery_active: bool,
) -> str:
    if recovery_active:
        return "RECOVERY"
    auth = str(maneuver_authority or "").upper()
    if auth in ("AVOIDANCE", "LOCAL_SELECTOR"):
        return "LOCAL_AVOIDANCE_OVERRIDE"
    if auth == "FSM":
        return "FSM_OVERRIDE"
    if tracking_local_plan or (local_plan_active and follow_path_source == SOURCE_LOCAL_PLAN):
        return "ROLLING_LOCAL_PLAN"
    if follow_path_source == SOURCE_GLOBAL_PATH:
        return "GLOBAL_REFERENCE"
    return "UNKNOWN"


@dataclass
class _PrevLookahead:
    display: Optional[Dict[str, Any]] = None
    pp: Optional[Dict[str, Any]] = None
    pp_source: Optional[str] = None
    vehicle: Optional[Tuple[float, float, float]] = None
    w_cmd: Optional[float] = None
    local_plan_id: Optional[str] = None
    local_revision: Optional[int] = None
    events: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=400))


_TRACKER = _PrevLookahead()


def assemble_lookahead_forensics(
    *,
    x: float,
    y: float,
    yaw: float,
    global_path: Optional[List[Any]] = None,
    global_reference: Optional[Dict[str, Any]] = None,
    local_plan: Optional[Dict[str, Any]] = None,
    mppi_meta: Optional[Dict[str, Any]] = None,
    display_lookahead_pt: Optional[Pt] = None,
    display_lookahead_m: float = LOOKAHEAD_DISPLAY_M,
    cmd_vx: Optional[float] = None,
    cmd_w: Optional[float] = None,
    safe_w: Optional[float] = None,
    state_w: Optional[float] = None,
    pp_w: Optional[float] = None,
    front_near: Optional[float] = None,
    left_near: Optional[float] = None,
    right_near: Optional[float] = None,
    path_revision: int = 0,
    clearance_at: Optional[ClearanceFn] = None,
    collide: Optional[CollideFn] = None,
    maneuver: Optional[Dict[str, Any]] = None,
    maneuver_authority: Optional[str] = None,
    fsm_mode: str = "",
    recovery_active: bool = False,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """Build per-cycle lookahead / reference forensics blob."""
    now = now or time.time()
    gpath = _as_xy(global_path)
    gref = global_reference or {}
    lp = local_plan or {}
    meta = mppi_meta or {}
    man = maneuver or {}

    g_len = _polyline_len(gpath)
    lp_xy = _as_xy(lp.get("poses"))
    lp_len = _polyline_len(lp_xy)

    g_id = f"GP-{path_revision or gref.get('path_revision') or 0}"
    g_rev = int(path_revision or gref.get("path_revision") or 0)
    lp_id = lp.get("plan_id") or meta.get("local_plan_id")
    lp_rev = lp.get("revision")

    # --- Display lookahead (UI pink point source) ---
    if display_lookahead_pt is not None:
        disp_pt = {"exists": True, "x": display_lookahead_pt[0], "y": display_lookahead_pt[1]}
    else:
        disp_la = lookahead_on_polyline(gpath, x, y, display_lookahead_m)
        disp_pt = disp_la
    disp_body = (
        body_frame(x, y, yaw, float(disp_pt["x"]), float(disp_pt["y"]))
        if disp_pt.get("exists")
        else {}
    )
    disp_obs = point_obstacle_diag(
        disp_pt.get("x") if disp_pt.get("exists") else None,
        disp_pt.get("y") if disp_pt.get("exists") else None,
        clearance_at=clearance_at,
        collide=collide,
    )
    display_block = {
        **path_identity(
            source=SOURCE_GLOBAL_PATH,
            path_id=g_id,
            revision=g_rev,
            horizon_m=gref.get("preview_m") or g_len,
            generated_at=gref.get("generated_at"),
        ),
        "exists": bool(disp_pt.get("exists")),
        "x": disp_pt.get("x"),
        "y": disp_pt.get("y"),
        "lookahead_m": round(float(display_lookahead_m), 3),
        "distance_m": disp_body.get("distance_m"),
        "body_x": disp_body.get("body_x"),
        "body_y": disp_body.get("body_y"),
        "heading_error_deg": disp_body.get("heading_error_deg"),
        "s_along_m": disp_pt.get("s_along_m") if "s_along_m" in disp_pt else None,
        **disp_obs,
        "role": "UI_PINK_POINT",
        "note": "sim3d.js dbg.controller.lookahead_point — always from global_path via sim_api_ext._lookahead_point",
    }

    # --- PP / MPPI follow-path lookahead (control reference) ---
    follow_src = str(meta.get("follow_path_source") or meta.get("pp_follow_path_source") or SOURCE_GLOBAL_PATH)
    pp_la_m = _f(meta.get("pp_lookahead_m")) or display_lookahead_m
    pp_pt_raw = meta.get("pp_lookahead_point") if isinstance(meta.get("pp_lookahead_point"), dict) else {}
    if pp_pt_raw.get("x") is not None and pp_pt_raw.get("y") is not None:
        pp_pt = {"exists": True, "x": pp_pt_raw["x"], "y": pp_pt_raw["y"]}
    elif follow_src == SOURCE_LOCAL_PLAN and lp_xy:
        pp_la = lookahead_on_polyline(lp_xy, x, y, pp_la_m)
        pp_pt = pp_la
    elif gpath:
        pp_la = lookahead_on_polyline(gpath, x, y, pp_la_m)
        pp_pt = pp_la
    else:
        pp_pt = {"exists": False}

    pp_body = (
        body_frame(x, y, yaw, float(pp_pt["x"]), float(pp_pt["y"])) if pp_pt.get("exists") else {}
    )
    pp_obs = point_obstacle_diag(
        pp_pt.get("x") if pp_pt.get("exists") else None,
        pp_pt.get("y") if pp_pt.get("exists") else None,
        clearance_at=clearance_at,
        collide=collide,
    )
    follow_id = lp_id if follow_src == SOURCE_LOCAL_PLAN else g_id
    follow_rev = lp_rev if follow_src == SOURCE_LOCAL_PLAN else g_rev
    follow_horizon = lp_len if follow_src == SOURCE_LOCAL_PLAN else (gref.get("preview_m") or g_len)

    pp_block = {
        **path_identity(
            source=follow_src,
            path_id=follow_id,
            revision=follow_rev,
            horizon_m=follow_horizon,
            generated_at=lp.get("generated_at") if follow_src == SOURCE_LOCAL_PLAN else gref.get("generated_at"),
        ),
        "exists": bool(pp_pt.get("exists")),
        "x": pp_pt.get("x"),
        "y": pp_pt.get("y"),
        "lookahead_m": round(float(pp_la_m), 3),
        "distance_m": pp_body.get("distance_m") or meta.get("pp_local_x") and meta.get("pp_local_y") and round(
            math.hypot(_f(meta.get("pp_local_x")) or 0, _f(meta.get("pp_local_y")) or 0), 4
        ),
        "body_x": pp_body.get("body_x") or meta.get("pp_local_x"),
        "body_y": pp_body.get("body_y") or meta.get("pp_local_y"),
        "heading_error_deg": pp_body.get("heading_error_deg")
        or (round(_deg(_f(meta.get("pp_heading_error")) or 0), 3) if meta.get("pp_heading_error") is not None else None),
        "alpha": meta.get("pp_alpha"),
        "kappa": meta.get("pp_kappa"),
        "w_raw": meta.get("pp_w"),
        "follow_path_length_m": meta.get("follow_path_length_m"),
        **pp_obs,
        "role": "PP_CONTROL_LOOKAHEAD",
    }

    # Primary exported lookahead = display (matches UI); pp in separate block
    lookahead = {
        **display_block,
        "pp_control": pp_block,
        "display_vs_pp_separation_m": None
        if not (display_block.get("exists") and pp_block.get("exists"))
        else round(
            math.hypot(
                float(display_block["x"]) - float(pp_block["x"]),
                float(display_block["y"]) - float(pp_block["y"]),
            ),
            4,
        ),
    }

    # --- Three heading lines ---
    g_tgt = lookahead_on_polyline(gpath, x, y, min(2.0, g_len or 2.0)) if gpath else {}
    l_tgt = lookahead_on_polyline(lp_xy, x, y, min(1.5, lp_len or 1.5)) if lp_xy else {}
    global_h = (
        heading_toward(x, y, float(g_tgt["x"]), float(g_tgt["y"]), yaw) if g_tgt.get("exists") else {}
    )
    local_h = heading_toward(x, y, float(l_tgt["x"]), float(l_tgt["y"]), yaw) if l_tgt.get("exists") else {}
    la_h = (
        heading_toward(x, y, float(display_block["x"]), float(display_block["y"]), yaw)
        if display_block.get("exists")
        else {}
    )
    pp_h = (
        heading_toward(x, y, float(pp_block["x"]), float(pp_block["y"]), yaw) if pp_block.get("exists") else {}
    )

    tracking_local = bool(meta.get("tracking_local_plan"))
    local_active = bool(lp.get("active")) or str(lp.get("status") or "").upper() in ("ACTIVE", "CREATED", "FALLBACK")
    active_auth = classify_reference_authority(
        tracking_local_plan=tracking_local,
        follow_path_source=follow_src,
        local_plan_active=local_active,
        maneuver_authority=maneuver_authority,
        fsm_mode=fsm_mode,
        recovery_active=recovery_active,
    )

    sel_cand = next((c for c in (lp.get("candidates") or []) if c.get("selected")), None)
    if sel_cand is None and lp.get("selected_candidate"):
        sel_cand = {"candidate_id": lp.get("selected_candidate"), "kind": lp.get("selected_kind")}

    reference = {
        "global": {
            **path_identity(
                source=SOURCE_GLOBAL_REFERENCE,
                path_id=g_id,
                revision=g_rev,
                horizon_m=gref.get("preview_m") or g_len,
            ),
            "target_heading_deg": global_h.get("heading_deg"),
            "heading_error_deg": global_h.get("error_deg"),
            "preview_m": gref.get("preview_m"),
            "remaining_m": gref.get("remaining_m"),
        },
        "local": {
            **path_identity(
                source=SOURCE_LOCAL_PLAN,
                path_id=lp_id,
                revision=lp_rev,
                horizon_m=lp.get("horizon_m") or lp_len,
            ),
            "target_heading_deg": local_h.get("heading_deg"),
            "heading_error_deg": local_h.get("error_deg"),
            "plan_id": lp_id,
            "revision": lp_rev,
            "status": lp.get("status"),
            "selected_candidate": lp.get("selected_candidate"),
            "selected_kind": lp.get("selected_kind"),
            "horizon_m": lp.get("horizon_m"),
            "endpoint": (lp_xy[-1] if lp_xy else None),
            "authority": lp.get("authority") or maneuver_authority,
            "global_deviation_m": None if sel_cand is None else sel_cand.get("global_deviation_m"),
            "heading_error": None if sel_cand is None else sel_cand.get("heading_error"),
            "reconnect_m": None if sel_cand is None else sel_cand.get("reconnect_m"),
            "progress_m": None if sel_cand is None else sel_cand.get("progress_m"),
        },
        "lookahead_display": {
            "source": SOURCE_GLOBAL_PATH,
            "target_heading_deg": la_h.get("heading_deg"),
            "heading_error_deg": la_h.get("error_deg"),
        },
        "pp_lookahead": {
            "source": follow_src,
            "target_heading_deg": pp_h.get("heading_deg"),
            "heading_error_deg": pp_h.get("error_deg"),
        },
        "mppi": {
            "tracking_local_plan": tracking_local,
            "local_plan_id": meta.get("local_plan_id"),
            "local_plan_horizon_m": lp.get("horizon_m"),
            "follow_path_source": follow_src,
            "follow_path_length_m": meta.get("follow_path_length_m"),
        },
    }

    controller = {
        "pp_w": meta.get("pp_w") if pp_w is None else pp_w,
        "mean_dw": meta.get("mean_dw"),
        "w_des": meta.get("w_des"),
        "w_blend": meta.get("w_blend"),
        "w_cmd": meta.get("w_cmd") if cmd_w is None else cmd_w,
        "safe_w": safe_w,
        "state_w": state_w,
        "pp_lookahead_m": meta.get("pp_lookahead_m"),
    }

    authority = {
        "active_reference": active_auth,
        "last_maneuver_authority": maneuver_authority,
        "local_plan_authority": lp.get("authority"),
        "fsm_mode": fsm_mode or man.get("mode"),
        "tracking_local_plan": tracking_local,
        "pp_follow_path_source": follow_src,
        "display_lookahead_source": SOURCE_GLOBAL_PATH,
    }

    dist_lp_end = distance_to_path_end(pp_block.get("x"), pp_block.get("y"), lp_xy) if lp_xy else None
    outside_local = bool(
        follow_src == SOURCE_LOCAL_PLAN
        and lp_xy
        and pp_block.get("exists")
        and dist_lp_end is not None
        and dist_lp_end < 0.05
        and _polyline_len(lp_xy) > 0.5
    )

    obstacle_pass = classify_obstacle_pass_state(
        front_near=front_near,
        side_near=min(_f(left_near) or 99, _f(right_near) or 99),
        obstacle_clearance=front_near,
        local_kind=lp.get("selected_kind"),
    )

    events: List[Dict[str, Any]] = []
    prev = _TRACKER

    veh_moved = 0.0
    if prev.vehicle is not None:
        veh_moved = math.hypot(x - prev.vehicle[0], y - prev.vehicle[1],)

    # Jump detection on display lookahead
    if display_block.get("exists") and prev.display and prev.display.get("exists"):
        d_jump = math.hypot(
            float(display_block["x"]) - float(prev.display["x"]),
            float(display_block["y"]) - float(prev.display["y"]),
        )
        ds = abs((_f(display_block.get("s_along_m")) or 0) - ((_f(prev.display.get("s_along_m")) or 0)))
        expected = max(0.05, veh_moved + 0.05)
        if d_jump > max(JUMP_DIST_THRESHOLD_M, expected * 2.5) and veh_moved < d_jump * 0.5:
            events.append(
                {
                    "event": "LOOKAHEAD_JUMP",
                    "target": "DISPLAY",
                    "delta_distance_m": round(d_jump, 4),
                    "delta_s_along_path": round(ds, 4),
                    "vehicle_moved_m": round(veh_moved, 4),
                }
            )

    if follow_src != prev.pp_source and prev.pp_source is not None:
        events.append(
            {
                "event": "LOOKAHEAD_SOURCE_SWITCH",
                "old_source": prev.pp_source,
                "new_source": follow_src,
                "old_path_id": prev.local_plan_id if prev.pp_source == SOURCE_LOCAL_PLAN else g_id,
                "new_path_id": follow_id,
                "old_revision": prev.local_revision if prev.pp_source == SOURCE_LOCAL_PLAN else g_rev,
                "new_revision": follow_rev,
                "reason": "PP_FOLLOW_PATH_CHANGE",
            }
        )

    if lp_id != prev.local_plan_id and prev.local_plan_id and lp_id:
        events.append(
            {
                "event": "LOCAL_PLAN_REPLACED",
                "old_plan_id": prev.local_plan_id,
                "new_plan_id": lp_id,
                "old_revision": prev.local_revision,
                "new_revision": lp_rev,
            }
        )

    if disp_obs.get("lookahead_inside_obstacle"):
        events.append(
            {
                "event": "LOOKAHEAD_INSIDE_OBSTACLE",
                "target": "DISPLAY",
                "x": display_block.get("x"),
                "y": display_block.get("y"),
                "clearance_m": disp_obs.get("lookahead_clearance_m"),
            }
        )
    elif disp_obs.get("lookahead_clearance_m") is not None and disp_obs["lookahead_clearance_m"] < MARGIN_TOO_CLOSE_M:
        events.append(
            {
                "event": "LOOKAHEAD_TOO_CLOSE_TO_OBSTACLE",
                "target": "DISPLAY",
                "clearance_m": disp_obs["lookahead_clearance_m"],
            }
        )

    if pp_obs.get("lookahead_inside_obstacle"):
        events.append(
            {
                "event": "LOOKAHEAD_INSIDE_OBSTACLE",
                "target": "PP_CONTROL",
                "x": pp_block.get("x"),
                "y": pp_block.get("y"),
                "clearance_m": pp_obs.get("lookahead_clearance_m"),
            }
        )

    if outside_local:
        events.append(
            {
                "event": "LOOKAHEAD_OUTSIDE_LOCAL_PLAN",
                "distance_to_local_end_m": dist_lp_end,
                "pp_x": pp_block.get("x"),
                "pp_y": pp_block.get("y"),
            }
        )

    ref_conflict = False
    if (
        global_h.get("error_deg") is not None
        and local_h.get("error_deg") is not None
        and la_h.get("error_deg") is not None
    ):
        g_err = float(global_h["error_deg"])
        l_err = float(local_h["error_deg"])
        la_err = float(la_h["error_deg"])
        if abs(g_err - l_err) > 15 and abs(la_err - g_err) < 8 and abs(la_err - l_err) > 15:
            ref_conflict = True
            events.append(
                {
                    "event": "REFERENCE_AUTHORITY_MISMATCH",
                    "detail": "display_lookahead aligns with global, diverges from local plan",
                    "global_error_deg": g_err,
                    "local_error_deg": l_err,
                    "lookahead_error_deg": la_err,
                    "pp_follow_source": follow_src,
                    "active_reference": active_auth,
                }
            )

    if tracking_local and follow_src == SOURCE_GLOBAL_PATH:
        events.append(
            {
                "event": "REFERENCE_AUTHORITY_MISMATCH",
                "detail": "tracking_local_plan=true but PP follow_path=GLOBAL_PATH",
                "active_reference": active_auth,
            }
        )

    if local_active and follow_src == SOURCE_LOCAL_PLAN and display_block.get("source") == SOURCE_GLOBAL_PATH:
        sep = lookahead.get("display_vs_pp_separation_m")
        if sep is not None and sep > 0.25:
            events.append(
                {
                    "event": "REFERENCE_AUTHORITY_MISMATCH",
                    "detail": "UI pink (global) vs PP control (local) separation",
                    "separation_m": sep,
                }
            )

    # Global pull suspected
    sel_dev = _f(sel_cand.get("global_deviation_m") if sel_cand else None)
    sel_rec = _f(sel_cand.get("reconnect_m") if sel_cand else None)
    kind = str(lp.get("selected_kind") or "").upper()
    if kind in ("LEFT_ARC", "RIGHT_ARC") and sel_dev is not None and sel_rec is not None:
        if sel_rec < 0.8 and abs(global_h.get("error_deg") or 0) > abs(local_h.get("error_deg") or 0):
            events.append(
                {
                    "event": "LOCAL_PLAN_GLOBAL_PULL_SUSPECTED",
                    "global_deviation_m": sel_dev,
                    "reconnect_m": sel_rec,
                    "global_error_deg": global_h.get("error_deg"),
                    "local_error_deg": local_h.get("error_deg"),
                }
            )

    w_now = _f(controller.get("w_cmd"))
    if (
        w_now is not None
        and prev.w_cmd is not None
        and obstacle_pass in ("PASSING", "BESIDE")
        and abs(w_now - prev.w_cmd) > TURNBACK_W_THRESHOLD
        and la_h.get("error_deg") is not None
        and local_h.get("error_deg") is not None
    ):
        if abs(float(la_h["error_deg"])) < abs(float(local_h["error_deg"])) - 5:
            events.append(
                {
                    "event": "LOOKAHEAD_TURNBACK",
                    "w_cmd_prev": prev.w_cmd,
                    "w_cmd": w_now,
                    "lookahead_heading_error_deg": la_h.get("error_deg"),
                    "local_plan_heading_error_deg": local_h.get("error_deg"),
                    "global_path_heading_error_deg": global_h.get("error_deg"),
                    "pp_w": controller.get("pp_w"),
                    "mean_dw": controller.get("mean_dw"),
                    "safe_w": safe_w,
                    "state_w": state_w,
                }
            )

    prev.display = dict(display_block)
    prev.pp = dict(pp_block)
    prev.pp_source = follow_src
    prev.vehicle = (x, y, yaw)
    prev.w_cmd = w_now
    prev.local_plan_id = lp_id
    prev.local_revision = lp_rev

    for ev in events:
        row = {"ts": now, **ev}
        prev.events.append(row)

    diagnostics = {
        "global_heading_deg": global_h.get("heading_deg"),
        "local_heading_deg": local_h.get("heading_deg"),
        "lookahead_heading_deg": la_h.get("heading_deg"),
        "pp_heading_deg": pp_h.get("heading_deg"),
        "global_error_deg": global_h.get("error_deg"),
        "local_error_deg": local_h.get("error_deg"),
        "lookahead_error_deg": la_h.get("error_deg"),
        "pp_error_deg": pp_h.get("error_deg"),
        "reference_conflict": ref_conflict,
        "obstacle_pass_state": obstacle_pass,
        "follow_path_length_m": meta.get("follow_path_length_m"),
        "local_plan_horizon_m": lp.get("horizon_m"),
        "global_path_length_m": round(g_len, 3),
        "distance_to_local_plan_end_m": dist_lp_end,
        "display_vs_pp_separation_m": lookahead.get("display_vs_pp_separation_m"),
        "vehicle": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)},
        "events_this_cycle": [e["event"] for e in events],
    }

    return {
        "ts": now,
        "lookahead": lookahead,
        "display_lookahead": display_block,
        "pp_lookahead": pp_block,
        "reference": reference,
        "controller": controller,
        "authority": authority,
        "diagnostics": diagnostics,
        "events": events,
        "paths": {
            "GLOBAL_PATH": path_identity(source=SOURCE_GLOBAL_PATH, path_id=g_id, revision=g_rev, horizon_m=g_len),
            "GLOBAL_REFERENCE": path_identity(
                source=SOURCE_GLOBAL_REFERENCE, path_id=g_id, revision=g_rev, horizon_m=gref.get("preview_m")
            ),
            "LOCAL_PLAN": path_identity(
                source=SOURCE_LOCAL_PLAN, path_id=lp_id, revision=lp_rev, horizon_m=lp.get("horizon_m")
            ),
            "PP_FOLLOW_PATH": path_identity(
                source=follow_src, path_id=follow_id, revision=follow_rev, horizon_m=follow_horizon
            ),
            "LOOKAHEAD_POINT": path_identity(
                source=SOURCE_DISPLAY, path_id=g_id, revision=g_rev, horizon_m=display_lookahead_m
            ),
        },
        "three_headings": {
            "global": global_h,
            "local": local_h,
            "lookahead_display": la_h,
            "pp_control": pp_h,
        },
        "reference_authority_timeline_entry": active_auth,
        "controls_vehicle": False,
    }


def emit_lookahead_events(forensic: Dict[str, Any]) -> None:
    """Emit OBS events from forensics (diagnostic only)."""
    try:
        from agv_bridge.nav_observability import OBS
    except Exception:
        return
    if not OBS.enabled:
        return
    for ev in forensic.get("events") or []:
        name = str(ev.get("event") or "")
        if not name:
            continue
        payload = {k: v for k, v in ev.items() if k not in ("event",)}
        OBS.emit(
            name,
            level="WARN" if "MISMATCH" in name or "INSIDE" in name or "TURNBACK" in name else "NOTICE",
            category="DIAGNOSTIC",
            component="lookahead_forensics",
            data=payload,
            force=True,
        )


def forensics_event_window(event_name: str, center_ts: float, window_s: float = 3.0) -> List[Dict[str, Any]]:
    """Return buffered cycle snippets ±window_s around matching event."""
    half = max(0.5, float(window_s))
    t0, t1 = center_ts - half, center_ts + half
    return [e for e in _TRACKER.events if t0 <= float(e.get("ts") or 0) <= t1 and e.get("event") == event_name]


def offline_obstacle_intrusion_test(
    obstacle_xy: Pt,
    obstacle_r: float = 0.35,
    vehicle_xy: Pt = (0.0, 0.0),
    vehicle_yaw: float = 0.0,
    global_path: Optional[List[Pt]] = None,
) -> Dict[str, Any]:
    """Unit-style test: global lookahead through circular obstacle."""
    ox, oy = obstacle_xy
    r = float(obstacle_r)

    def clearance_at(px: float, py: float) -> float:
        return math.hypot(px - ox, py - oy) - r

    def collide(px: float, py: float) -> bool:
        return clearance_at(px, py) < 0.0

    gpath = global_path or [(0.0, 0.0), (0.7, 0.0), (1.4, 0.0), (5.0, 0.0)]
    la = lookahead_on_polyline(gpath, vehicle_xy[0], vehicle_xy[1], LOOKAHEAD_DISPLAY_M)
    obs = point_obstacle_diag(la.get("x"), la.get("y"), clearance_at=clearance_at, collide=collide)
    events = []
    if obs.get("lookahead_inside_obstacle"):
        events.append("LOOKAHEAD_INSIDE_OBSTACLE")
    return {
        "obstacle": {"x": ox, "y": oy, "r": r},
        "lookahead": la,
        "obstacle_diag": obs,
        "events": events,
        "confirmed_inside": bool(obs.get("lookahead_inside_obstacle")),
    }
