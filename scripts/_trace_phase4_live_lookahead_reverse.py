#!/usr/bin/env python3
"""P1-2-LIVE — Lookahead distance + turn-back + breadcrumb/reverse forensics.

Requires web sim at BASE (default 127.0.0.1:19999).
OBSERVE ONLY — does not change planning / Safety / FSM / MPPI behavior.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BASE_DEFAULT = "http://127.0.0.1:19999"


def api(base: str, method: str, path: str, body=None, timeout: float = 15.0) -> Dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(base + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def pose(base: str) -> Tuple[float, float, float]:
    a = (api(base, "GET", "/api/state").get("agv") or {})
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def bf(x: float, y: float, yaw: float, fwd: float, lat: float) -> Tuple[float, float]:
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def cancel_clear(base: str) -> None:
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            api(base, "POST", p, {})
        except Exception:
            pass
    try:
        api(base, "POST", "/api/obstacles/clear", {})
    except Exception:
        pass
    time.sleep(0.35)


def add_obs(base: str, x: float, y: float, r: float = 0.38, name: str = "obs") -> None:
    api(base, "POST", "/api/obstacles/add", {"x": x, "y": y, "r": r, "name": name})


def plan_start(base: str, goal_m: float = 10.0, lateral: float = 0.0) -> Dict[str, Any]:
    cancel_clear(base)
    try:
        api(base, "POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.65)
    except Exception:
        pass
    x, y, yaw = pose(base)
    gx, gy = bf(x, y, yaw, goal_m, lateral)
    wrap = api(base, "POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api(base, "POST", "/api/nav/confirm", {})
    time.sleep(0.85)
    return {"start": (x, y, yaw), "goal": (gx, gy)}


def seal_three_side(base: str, x: float, y: float, yaw: float) -> None:
    fx, fy = bf(x, y, yaw, 1.15, 0.0)
    add_obs(base, fx, fy, 0.42, "f3_front")
    for i, (fwd, lat, r) in enumerate([(1.0, 0.85, 0.34), (1.25, 1.05, 0.34)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        add_obs(base, bx, by, r, f"f3_L{i}")
    for i, (fwd, lat, r) in enumerate([(1.0, -0.85, 0.34), (1.25, -1.05, 0.34)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        add_obs(base, bx, by, r, f"f3_R{i}")


def sample_row(base: str, t: float, phase: str) -> Dict[str, Any]:
    st = api(base, "GET", "/api/state")
    agv = st.get("agv") or {}
    nav = st.get("nav") or {}
    safety = st.get("safety") or {}
    try:
        dbg_wrap = api(base, "GET", "/api/nav/debug")
        dbg = dbg_wrap.get("debug") if isinstance(dbg_wrap.get("debug"), dict) else dbg_wrap
    except Exception:
        dbg = {}
    try:
        lf = api(base, "GET", "/api/nav/forensics/lookahead")
    except Exception:
        lf = {}
    try:
        diag_win = api(base, "GET", "/api/logs/diagnostics?window_s=3")
    except Exception:
        diag_win = {}

    disp = lf.get("lookahead") or lf.get("display_lookahead") or {}
    pp = lf.get("pp_lookahead") or {}
    ref = lf.get("reference") or {}
    ctrl_lf = lf.get("controller") or {}
    auth = lf.get("authority") or {}
    lfd = lf.get("diagnostics") or {}
    th = lf.get("three_headings") or {}

    ctrl_dbg = dbg.get("controller") if isinstance(dbg.get("controller"), dict) else {}
    mppi_meta = dbg.get("mppi_meta") if isinstance(dbg.get("mppi_meta"), dict) else {}
    if not mppi_meta:
        mppi_meta = nav.get("mppi_meta") if isinstance(nav.get("mppi_meta"), dict) else {}

    # Fallback: pink point always from debug controller when forensics API empty
    la_pt = ctrl_dbg.get("lookahead_point") if isinstance(ctrl_dbg.get("lookahead_point"), dict) else {}
    if not disp.get("x") and la_pt.get("x") is not None:
        disp = {
            "x": la_pt.get("x"),
            "y": la_pt.get("y"),
            "path_source": "GLOBAL_PATH",
            "lookahead_m": ctrl_dbg.get("pp_lookahead_m"),
        }
    if not pp.get("x") and isinstance(mppi_meta.get("pp_lookahead_point"), dict):
        pp = {
            **mppi_meta.get("pp_lookahead_point", {}),
            "path_source": mppi_meta.get("follow_path_source") or mppi_meta.get("pp_follow_path_source"),
            "lookahead_m": mppi_meta.get("pp_lookahead_m"),
        }

    ref = ref or {}
    ctrl = {**ctrl_lf, **{k: v for k, v in {
        "pp_w": ctrl_dbg.get("pp_w") or mppi_meta.get("pp_w"),
        "mean_dw": mppi_meta.get("mean_dw"),
        "w_des": mppi_meta.get("w_des"),
        "w_blend": mppi_meta.get("w_blend"),
        "w_cmd": mppi_meta.get("w_cmd") or ctrl_dbg.get("mppi_w"),
        "pp_lookahead_m": ctrl_dbg.get("pp_lookahead_m") or mppi_meta.get("pp_lookahead_m"),
    }.items() if v is not None}}
    if not auth.get("pp_follow_path_source") and mppi_meta.get("follow_path_source"):
        auth = {
            **auth,
            "display_lookahead_source": "GLOBAL_PATH",
            "pp_follow_path_source": mppi_meta.get("follow_path_source"),
            "active_reference": "ROLLING_LOCAL_PLAN" if mppi_meta.get("tracking_local_plan") else "GLOBAL_REFERENCE",
            "tracking_local_plan": mppi_meta.get("tracking_local_plan"),
        }
    local = ref.get("local") or dbg.get("local_plan") or nav.get("local_plan") or {}
    mppi_ref = ref.get("mppi") or mppi_meta or {}
    if not mppi_ref.get("tracking_local_plan") and mppi_meta.get("tracking_local_plan") is not None:
        mppi_ref = {**mppi_ref, "tracking_local_plan": mppi_meta.get("tracking_local_plan"), "target_vx": mppi_meta.get("target_vx")}
    man = dbg.get("maneuver") or {}
    pol = dbg.get("nav_policy") or dbg.get("policy") or {}
    pol_dec = pol.get("decision") if isinstance(pol.get("decision"), dict) else pol
    rec = dbg.get("recovery") or (dbg.get("phase4") or {}).get("recovery") or {}
    pr = dbg.get("probe") or (dbg.get("phase4") or {}).get("probe") or {}
    bc = dbg.get("breadcrumb") or (dbg.get("phase4") or {}).get("breadcrumb") or {}
    radar = dbg.get("radar") or {}
    geom = dbg.get("geometry") or {}
    vc = dbg.get("velocity_chain") or {}
    gref = dbg.get("global_reference") or {}

    vx = _f(agv.get("vx")) or _f(nav.get("state_vx")) or 0.0
    x, y, yaw = float(agv.get("x") or 0), float(agv.get("y") or 0), float(agv.get("angle") or 0)
    disp_x, disp_y = _f(disp.get("x")), _f(disp.get("y"))
    pp_x, pp_y = _f(pp.get("x")), _f(pp.get("y"))
    disp_dist = math.hypot(disp_x - x, disp_y - y) if disp_x is not None and disp_y is not None else None
    pp_dist = math.hypot(pp_x - x, pp_y - y) if pp_x is not None and pp_y is not None else None
    sep_m = None
    if disp_x is not None and disp_y is not None and pp_x is not None and pp_y is not None:
        sep_m = round(math.hypot(disp_x - pp_x, disp_y - pp_y), 4)

    retreat = rec.get("retreat") if isinstance(rec.get("retreat"), dict) else {}
    sel_cand = None
    for c in local.get("candidates") or []:
        if c.get("selected"):
            sel_cand = c
            break

    return {
        "t": round(t, 3),
        "phase": phase,
        "vehicle_x": round(x, 3),
        "vehicle_y": round(y, 3),
        "vehicle_yaw": round(yaw, 4),
        "vehicle_speed": round(vx, 4),
        "display_lookahead_m": disp_dist,
        "display_lookahead_source": disp.get("path_source"),
        "control_lookahead_m": pp_dist,
        "control_lookahead_source": pp.get("path_source"),
        "pp_lookahead_m_param": _f(ctrl.get("pp_lookahead_m")) or _f(pp.get("lookahead_m")),
        "local_plan_horizon_m": _f(local.get("horizon_m")),
        "global_preview_m": _f(gref.get("preview_m")),
        "front_near": _f(safety.get("front_near")) or _f(nav.get("front_near")) or _f(dbg.get("front_near")),
        "rear_near": _f(safety.get("rear_near")) or _f(nav.get("rear_near")) or _f(dbg.get("rear_near")),
        "left_near": _f(dbg.get("left_near")),
        "right_near": _f(dbg.get("right_near")),
        "front_stop_m": _f(radar.get("front_stop_m")) or _f(geom.get("front_stop_m")),
        "front_cost_m": _f(geom.get("front_cost_m")),
        "display_la_x": disp_x,
        "display_la_y": disp_y,
        "display_clearance_m": disp.get("lookahead_clearance_m"),
        "display_inside_obstacle": disp.get("lookahead_inside_obstacle"),
        "pp_inside_obstacle": pp.get("lookahead_inside_obstacle"),
        "global_heading_deg": lfd.get("global_heading_deg") or (th.get("global") or {}).get("heading_deg"),
        "local_heading_deg": lfd.get("local_heading_deg") or (th.get("local") or {}).get("heading_deg"),
        "lookahead_heading_deg": lfd.get("lookahead_heading_deg") or (th.get("lookahead_display") or {}).get("heading_deg"),
        "global_error_deg": lfd.get("global_error_deg"),
        "local_error_deg": lfd.get("local_error_deg"),
        "lookahead_error_deg": lfd.get("lookahead_error_deg"),
        "reference_conflict": lfd.get("reference_conflict"),
        "display_vs_pp_m": lfd.get("display_vs_pp_separation_m") or sep_m,
        "display_authority": auth.get("display_lookahead_source"),
        "control_authority": auth.get("pp_follow_path_source"),
        "active_reference": auth.get("active_reference"),
        "tracking_local_plan": mppi_ref.get("tracking_local_plan"),
        "local_plan_id": local.get("plan_id"),
        "local_plan_revision": local.get("revision"),
        "local_selected_kind": local.get("selected_kind"),
        "local_plan_authority": local.get("authority") or auth.get("local_plan_authority"),
        "maneuver_authority": auth.get("last_maneuver_authority"),
        "maneuver_mode": man.get("mode"),
        "reconnect_m": _f(local.get("reconnect_m")) or (_f(sel_cand.get("reconnect_m")) if sel_cand else None),
        "global_deviation_m": _f(local.get("global_deviation_m")) or (_f(sel_cand.get("global_deviation_m")) if sel_cand else None),
        "policy_state": pol_dec.get("state") or pol.get("state"),
        "policy_behavior": pol_dec.get("behavior") or pol.get("behavior"),
        "target_vx": _f(mppi_ref.get("target_vx")) or _f((dbg.get("speed_policy") or {}).get("target_vx")),
        "requested_vx": _f(vc.get("cmd_vx")) or _f(nav.get("cmd_vx_before_safety")),
        "safe_vx": _f(vc.get("safe_vx")) or _f(nav.get("cmd_vx_after_safety")),
        "state_vx": _f(vc.get("state_vx")) or _f(nav.get("state_vx")),
        "pp_w": _f(ctrl.get("pp_w")),
        "mean_dw": _f(ctrl.get("mean_dw")),
        "w_cmd": _f(ctrl.get("w_cmd")) or _f(nav.get("cmd_w_before_safety")),
        "safe_w": _f(vc.get("safe_w")) or _f(nav.get("cmd_w_after_safety")),
        "state_w": _f(vc.get("state_w")) or _f(nav.get("state_w")),
        "stop_reason": nav.get("stop_reason") or dbg.get("stop_reason"),
        "obstacle_pass_state": lfd.get("obstacle_pass_state"),
        "recovery_action": rec.get("action"),
        "recovery_class": rec.get("classification"),
        "recovery_allowed": rec.get("allowed") if "allowed" in rec else rec.get("allow_recovery"),
        "historical_retreat_status": retreat.get("status"),
        "historical_retreat_safe_m": retreat.get("safe_distance_m"),
        "historical_retreat_failure": retreat.get("failure_reason"),
        "probe_forward": (pr.get("forward") or {}).get("status"),
        "probe_backward": (pr.get("backward") or {}).get("status"),
        "probe_left": (pr.get("left") or {}).get("status"),
        "probe_right": (pr.get("right") or {}).get("status"),
        "breadcrumb_count": bc.get("count"),
        "breadcrumb_length_m": bc.get("length_m"),
        "pose_trace_len": len((dbg.get("paths") or {}).get("actual_trace") or dbg.get("pose_trace") or []),
        "events": json.dumps(lf.get("events") or []),
        "recent_log_events": json.dumps([e.get("event") for e in (diag_win.get("events") or [])[-5:]]),
    }


def run_trace(
    base: str,
    name: str,
    setup: Callable[[], None],
    duration_s: float,
    hz: float = 5.0,
    inject_fn: Optional[Callable[[float, List[Dict[str, Any]]], None]] = None,
) -> Dict[str, Any]:
    print(f"[{name}] setup...")
    setup()
    rows: List[Dict[str, Any]] = []
    dt = 1.0 / max(1.0, hz)
    t0 = time.time()
    while time.time() - t0 < duration_s:
        t = time.time() - t0
        try:
            row = sample_row(base, t, name)
            rows.append(row)
            if inject_fn:
                inject_fn(t, rows)
        except Exception as exc:
            rows.append({"t": round(t, 3), "phase": name, "error": str(exc)})
        time.sleep(dt)

    summary = analyze_scene(name, rows)
    summary["rows"] = len(rows)
    return {"name": name, "rows": rows, "summary": summary}


def analyze_scene(name: str, rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    valid = [r for r in rows if "error" not in r]
    if not valid:
        return {"error": "no valid rows"}

    def first_when(key: str, pred) -> Optional[float]:
        for r in valid:
            v = r.get(key)
            if v is not None and pred(v):
                return r["t"]
        return None

    disp_dists = [_f(r.get("display_lookahead_m")) for r in valid if _f(r.get("display_lookahead_m")) is not None]
    ctrl_dists = [_f(r.get("control_lookahead_m")) for r in valid if _f(r.get("control_lookahead_m")) is not None]
    speeds = [_f(r.get("vehicle_speed")) or 0 for r in valid]

    s: Dict[str, Any] = {
        "display_lookahead_mean_m": round(statistics.mean(disp_dists), 3) if disp_dists else None,
        "display_lookahead_std_m": round(statistics.pstdev(disp_dists), 4) if len(disp_dists) > 1 else 0,
        "control_lookahead_mean_m": round(statistics.mean(ctrl_dists), 3) if ctrl_dists else None,
        "fixed_display_lookahead": bool(disp_dists and max(disp_dists) - min(disp_dists) < 0.15),
        "reference_conflict_any": any(r.get("reference_conflict") for r in valid),
        "display_inside_any": any(r.get("display_inside_obstacle") for r in valid),
        "turnback_events": sum(1 for r in valid if "LOOKAHEAD_TURNBACK" in str(r.get("events") or "")),
        "mismatch_events": sum(1 for r in valid if "REFERENCE_AUTHORITY_MISMATCH" in str(r.get("events") or "")),
        "global_pull_events": sum(1 for r in valid if "LOCAL_PLAN_GLOBAL_PULL_SUSPECTED" in str(r.get("events") or "")),
        "max_display_vs_pp_m": max((_f(r.get("display_vs_pp_m")) or 0 for r in valid), default=0),
    }

    if name.startswith("L0"):
        s["lookahead_vs_speed"] = [
            {"speed": round(sp, 3), "display_m": round(dd, 3)}
            for sp, dd in zip(speeds[::max(1, len(speeds)//8)], disp_dists[::max(1, len(disp_dists)//8)])
        ]

    if name.startswith("L1") or name.startswith("L2") or name.startswith("L5"):
        s["d_first_obstacle_aware"] = first_when("front_near", lambda v: _f(v) is not None and _f(v) < 2.5)
        s["d_first_side_arc"] = first_when("local_selected_kind", lambda v: str(v or "").upper() in ("LEFT_ARC", "RIGHT_ARC"))
        s["d_first_turn"] = first_when("state_w", lambda v: abs(_f(v) or 0) > 0.06)
        s["d_front_stop"] = first_when("stop_reason", lambda v: "FRONT" in str(v or "").upper())
        s["d_safety_zero"] = first_when("safe_vx", lambda v: _f(v) is not None and abs(_f(v) or 0) < 0.02 and (_f(valid[-1].get("requested_vx")) or 0) > 0.04)

    if name.startswith("L2") or name.startswith("L4"):
        # Phase detection for turnback
        bypass = [r for r in valid if str(r.get("local_selected_kind") or "").upper() in ("LEFT_ARC", "RIGHT_ARC")]
        s["bypass_rows"] = len(bypass)
        if bypass:
            post = valid[valid.index(bypass[-1]) :]
            w_flips = sum(
                1
                for i in range(1, len(post))
                if (_f(post[i].get("w_cmd")) or 0) * (_f(post[i - 1].get("w_cmd")) or 0) < -0.001
            )
            s["w_cmd_sign_flips_after_bypass"] = w_flips
            s["turnback_likely"] = w_flips > 0 or s["turnback_events"] > 0

    if name.startswith("R"):
        s["recovery_action_seen"] = sorted({str(r.get("recovery_action") or "") for r in valid if r.get("recovery_action")})
        s["max_breadcrumb_count"] = max((_f(r.get("breadcrumb_count")) or 0 for r in valid), default=0)
        s["max_breadcrumb_length_m"] = max((_f(r.get("breadcrumb_length_m")) or 0 for r in valid), default=0)
        s["historical_retreat_valid"] = any(str(r.get("historical_retreat_status") or "").upper() == "VALID" for r in valid)
        s["reverse_state_vx"] = min((_f(r.get("state_vx")) or 0 for r in valid), default=0)
        s["reverse_requested"] = any((_f(r.get("requested_vx")) or 0) < -0.03 for r in valid)

    return s


def build_scenes(base: str) -> Dict[str, Callable[[], None]]:
    scenes: Dict[str, Callable[[], None]] = {}

    scenes["L0_open"] = lambda: plan_start(base, goal_m=12.0)

    def l1():
        plan_start(base, goal_m=12.0)
        x, y, yaw = pose(base)
        add_obs(base, *bf(x, y, yaw, 2.8, 0.0), 0.45, "l1_front")

    scenes["L1_obstacle_straight"] = l1

    def l2():
        plan_start(base, goal_m=12.0)
        x, y, yaw = pose(base)
        add_obs(base, *bf(x, y, yaw, 2.8, 0.0), 0.45, "l2_front")

    scenes["L2_left_bypass"] = l2

    def l3():
        plan_start(base, goal_m=12.0)
        x, y, yaw = pose(base)
        add_obs(base, *bf(x, y, yaw, 2.8, 0.05), 0.45, "l3_front")

    scenes["L3_right_bypass"] = l3

    def l4():
        plan_start(base, goal_m=12.0)
        x, y, yaw = pose(base)
        add_obs(base, *bf(x, y, yaw, 2.5, 0.0), 0.42, "l4_front")

    scenes["L4_bypass_turnback"] = l4

    def l5():
        plan_start(base, goal_m=12.0)
        x, y, yaw = pose(base)
        add_obs(base, *bf(x, y, yaw, 4.0, 0.0), 0.55, "l5_global_through")

    scenes["L5_global_through_obstacle"] = l5

    scenes["R1_three_side_rear_open"] = lambda: plan_start(base, goal_m=8.0)

    return scenes


def run_r1_inject(base: str, t: float, rows: List[Dict[str, Any]]) -> None:
    if t > 1.5 and not getattr(run_r1_inject, "_done", False):
        x, y, yaw = pose(base)
        seal_three_side(base, x, y, yaw)
        run_r1_inject._done = True  # type: ignore[attr-defined]
        print(f"[R1] injected three-side at t={t:.2f}")


def run_r2_inject(base: str, t: float, rows: List[Dict[str, Any]]) -> None:
    if t > 2.0 and not getattr(run_r2_inject, "_moved", False):
        # let vehicle move ~2m first
        if rows and (_f(rows[-1].get("vehicle_speed")) or 0) > 0.05:
            run_r2_inject._moved = True  # type: ignore[attr-defined]
    if t > 4.0 and getattr(run_r2_inject, "_moved", False) and not getattr(run_r2_inject, "_done", False):
        x, y, yaw = pose(base)
        seal_three_side(base, x, y, yaw)
        run_r2_inject._done = True  # type: ignore[attr-defined]
        print(f"[R2] moved then sealed at t={t:.2f}")


def run_r3_inject(base: str, t: float, rows: List[Dict[str, Any]]) -> None:
    if t > 3.0 and not getattr(run_r3_inject, "_snap", False):
        bc_before = rows[-1].get("breadcrumb_count") if rows else 0
        run_r3_inject._bc_before = bc_before  # type: ignore[attr-defined]
        run_r3_inject._snap = True  # type: ignore[attr-defined]
    if t > 5.0 and not getattr(run_r3_inject, "_replanned", False):
        x, y, yaw = pose(base)
        gx, gy = bf(x, y, yaw, 6.0, 0.0)
        try:
            api(base, "POST", "/api/nav/plan", {"x": gx, "y": gy})
            api(base, "POST", "/api/nav/confirm", {})
            run_r3_inject._replanned = True  # type: ignore[attr-defined]
            print(f"[R3] replan triggered at t={t:.2f}")
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE_DEFAULT)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--scenes", default="L0,L1,L2,L3,L4,L5,R1,R2,R3")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "live_lookahead_reverse"))
    args = ap.parse_args()
    base = args.base.rstrip("/")
    os.makedirs(args.out_dir, exist_ok=True)

    try:
        api(base, "GET", "/api/state", timeout=3.0)
    except Exception as exc:
        print(f"FAIL: web sim not reachable at {base}: {exc}")
        print("Start: python scripts/run_web_sim.py")
        return 2

    scene_fns = build_scenes(base)
    selected = [s.strip() for s in args.scenes.split(",") if s.strip()]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    all_rows: List[Dict[str, Any]] = []
    summaries: Dict[str, Any] = {"stamp": stamp, "base": base, "scenes": {}}

    inject_map = {
        "R1_three_side_rear_open": lambda t, rows: run_r1_inject(base, t, rows),
        "R2_historical_retreat": lambda t, rows: run_r2_inject(base, t, rows),
        "R3_breadcrumb_reset": lambda t, rows: run_r3_inject(base, t, rows),
    }

    for key in selected:
        if key in ("R1", "R2", "R3"):
            name = {"R1": "R1_three_side_rear_open", "R2": "R2_historical_retreat", "R3": "R3_breadcrumb_reset"}[key]
        elif key.startswith("L"):
            name = {
                "L0": "L0_open",
                "L1": "L1_obstacle_straight",
                "L2": "L2_left_bypass",
                "L3": "L3_right_bypass",
                "L4": "L4_bypass_turnback",
                "L5": "L5_global_through_obstacle",
            }.get(key, key)
        else:
            name = key
        if name not in scene_fns and name not in inject_map:
            print(f"WARN: unknown scene {name}")
            continue
        dur = 15.0 if name.startswith(("L1", "L2", "L4", "L5", "R")) else args.duration
        if name == "R2_historical_retreat":
            scene_fns[name] = lambda: plan_start(base, goal_m=10.0)
        if name == "R3_breadcrumb_reset":
            scene_fns[name] = lambda: plan_start(base, goal_m=10.0)
        result = run_trace(
            base,
            name,
            scene_fns.get(name, lambda: plan_start(base)),
            duration_s=dur,
            inject_fn=inject_map.get(name),
        )
        summaries["scenes"][name] = result["summary"]
        all_rows.extend(result["rows"])
        if name == "R3_breadcrumb_reset":
            summaries["scenes"][name]["bc_before_replan"] = getattr(run_r3_inject, "_bc_before", None)
            summaries["scenes"][name]["bc_after_replan"] = result["summary"].get("max_breadcrumb_count")

    jsonl = os.path.join(args.out_dir, f"live_forensics_{stamp}.jsonl")
    csv_path = os.path.join(args.out_dir, f"live_forensics_{stamp}.csv")
    summary_path = os.path.join(args.out_dir, f"live_forensics_{stamp}_summary.json")
    with open(jsonl, "w", encoding="utf-8") as fh:
        for row in all_rows:
            fh.write(json.dumps(row) + "\n")
    if all_rows:
        keys = sorted({k for r in all_rows for k in r.keys()})
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(all_rows)
    with open(summary_path, "w", encoding="utf-8") as fh:
        json.dump(summaries, fh, indent=2)

    print(f"\nLIVE trace complete: {len(all_rows)} rows")
    print(f"  jsonl: {jsonl}")
    print(f"  summary: {summary_path}")
    for sn, meta in summaries["scenes"].items():
        print(f"  {sn}: {json.dumps(meta, ensure_ascii=False)[:200]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
