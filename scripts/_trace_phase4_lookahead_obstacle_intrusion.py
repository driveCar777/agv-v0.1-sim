#!/usr/bin/env python3
"""P1-2-OBSERVE — Pink lookahead / obstacle intrusion LIVE trace (READ/TRACE only).

Scenes L1–L5. Requires web sim at BASE (default 127.0.0.1:19999) unless --offline-only.
Does NOT change planning / Safety / FSM / MPPI behavior.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from agv_bridge.nav_lookahead_forensics import (  # noqa: E402
    assemble_lookahead_forensics,
    offline_obstacle_intrusion_test,
)

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 12.0) -> Dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def pose() -> Tuple[float, float, float]:
    a = (api("GET", "/api/state").get("agv") or {})
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def bf(x: float, y: float, yaw: float, fwd: float, lat: float) -> Tuple[float, float]:
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def cancel_clear() -> None:
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            api("POST", p, {})
        except Exception:
            pass
    try:
        api("POST", "/api/obstacles/clear", {})
    except Exception:
        pass
    time.sleep(0.3)


def add_box(cx: float, cy: float, w: float = 0.8, h: float = 0.6) -> None:
    api(
        "POST",
        "/api/obstacles",
        {
            "type": "box",
            "x": cx,
            "y": cy,
            "width": w,
            "height": h,
            "rotation": 0,
        },
    )


def plan_goal(goal_m: float = 8.0, lateral: float = 0.0) -> Dict[str, Any]:
    cancel_clear()
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.6)
    except Exception:
        pass
    x, y, yaw = pose()
    gx, gy = bf(x, y, yaw, goal_m, lateral)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.8)
    return {"start": (x, y, yaw), "goal": (gx, gy)}


def sample_row(t: float, phase: str) -> Dict[str, Any]:
    st = api("GET", "/api/state")
    agv = st.get("agv") or {}
    try:
        lf = api("GET", "/api/nav/forensics/lookahead")
    except Exception:
        lf = {}
    nav = st.get("nav") or {}
    disp = lf.get("lookahead") or {}
    pp = lf.get("pp_lookahead") or {}
    ref = lf.get("reference") or {}
    ctrl = lf.get("controller") or {}
    auth = lf.get("authority") or {}
    diag = lf.get("diagnostics") or {}
    th = lf.get("three_headings") or {}
    lp = ref.get("local") or nav.get("local_plan") or {}
    return {
        "t": round(t, 3),
        "phase": phase,
        "vehicle_x": agv.get("x"),
        "vehicle_y": agv.get("y"),
        "vehicle_yaw": agv.get("angle"),
        "display_la_x": disp.get("x"),
        "display_la_y": disp.get("y"),
        "display_la_source": disp.get("path_source"),
        "display_inside_obstacle": disp.get("lookahead_inside_obstacle"),
        "display_clearance_m": disp.get("lookahead_clearance_m"),
        "pp_la_x": pp.get("x"),
        "pp_la_y": pp.get("y"),
        "pp_la_source": pp.get("path_source"),
        "pp_inside_obstacle": pp.get("lookahead_inside_obstacle"),
        "global_heading_deg": diag.get("global_heading_deg"),
        "local_heading_deg": diag.get("local_heading_deg"),
        "lookahead_heading_deg": diag.get("lookahead_heading_deg"),
        "global_error_deg": diag.get("global_error_deg"),
        "local_error_deg": diag.get("local_error_deg"),
        "lookahead_error_deg": diag.get("lookahead_error_deg"),
        "reference_conflict": diag.get("reference_conflict"),
        "obstacle_pass_state": diag.get("obstacle_pass_state"),
        "active_reference": auth.get("active_reference"),
        "local_plan_id": lp.get("plan_id"),
        "local_plan_horizon_m": lp.get("horizon_m"),
        "local_selected_kind": lp.get("selected_kind"),
        "tracking_local_plan": (ref.get("mppi") or {}).get("tracking_local_plan"),
        "pp_w": ctrl.get("pp_w"),
        "mean_dw": ctrl.get("mean_dw"),
        "w_cmd": ctrl.get("w_cmd"),
        "safe_w": ctrl.get("safe_w"),
        "state_w": ctrl.get("state_w"),
        "front_near": nav.get("front_near"),
        "display_vs_pp_m": diag.get("display_vs_pp_separation_m"),
        "events": json.dumps(lf.get("events") or []),
    }


def run_scene(name: str, setup_fn, duration_s: float = 10.0, hz: float = 5.0) -> List[Dict[str, Any]]:
    print(f"[scene {name}] setup...")
    setup_fn()
    api("POST", "/api/nav/start", {})
    rows: List[Dict[str, Any]] = []
    dt = 1.0 / max(1.0, hz)
    t0 = time.time()
    while time.time() - t0 < duration_s:
        t = time.time() - t0
        try:
            rows.append(sample_row(t, name))
        except Exception as exc:
            rows.append({"t": round(t, 3), "phase": name, "error": str(exc)})
        time.sleep(dt)
    return rows


def offline_suite() -> Dict[str, Any]:
    """Offline geometry tests — no sim required."""
    # Lookahead through obstacle on straight global path
    t1 = offline_obstacle_intrusion_test(obstacle_xy=(1.4, 0.0), obstacle_r=0.15)
    # Local bypass vs global straight — synthetic forensics
    gpath = [(0.0, 0.0), (6.0, 0.0)]
    lposes = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.3}, {"x": 2.0, "y": 0.8}, {"x": 3.0, "y": 0.4}]
    lf = assemble_lookahead_forensics(
        x=0.5,
        y=0.0,
        yaw=0.0,
        global_path=gpath,
        global_reference={"preview_m": 5.0, "path_revision": 1},
        local_plan={
            "plan_id": "LP-OFF-1",
            "revision": 1,
            "status": "ACTIVE",
            "active": True,
            "horizon_m": 2.0,
            "selected_kind": "LEFT_ARC",
            "poses": lposes,
            "authority": "ROLLING_LOCAL",
        },
        mppi_meta={
            "tracking_local_plan": True,
            "follow_path_source": "LOCAL_PLAN",
            "local_plan_id": "LP-OFF-1",
            "pp_lookahead_m": 1.4,
            "pp_lookahead_point": {"x": 1.35, "y": 0.45},
            "follow_path_length_m": 2.0,
            "pp_w": 0.08,
            "w_cmd": 0.07,
        },
        display_lookahead_pt=(1.4, 0.0),
        front_near=1.2,
        path_revision=1,
        clearance_at=lambda px, py: math.hypot(px - 2.5, py) - 0.4,
        collide=lambda px, py: math.hypot(px - 2.5, py) < 0.4,
    )
    events = [e.get("event") for e in lf.get("events") or []]
    return {
        "offline_obstacle_intrusion": t1,
        "offline_l5_mismatch_events": events,
        "offline_reference_conflict": lf.get("diagnostics", {}).get("reference_conflict"),
        "offline_display_source": lf.get("display_lookahead", {}).get("path_source"),
        "offline_pp_source": lf.get("pp_lookahead", {}).get("path_source"),
    }


def live_available(base: str = BASE) -> bool:
    try:
        api("GET", "/api/state", timeout=2.0)
        return True
    except Exception:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--offline-only", action="store_true")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "lookahead_forensics"))
    args = ap.parse_args()
    base = args.base.rstrip("/")

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    summary: Dict[str, Any] = {"stamp": stamp, "scenes": {}, "offline": offline_suite()}

    if args.offline_only or not live_available(base):
        if not args.offline_only:
            print("WARN: sim not reachable — running offline suite only")
        out = os.path.join(args.out_dir, f"lookahead_offline_{stamp}.json")
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2)
        print(f"offline PASS — wrote {out}")
        obs = summary["offline"]["offline_obstacle_intrusion"]
        print(f"  LOOKAHEAD_INSIDE_OBSTACLE synthetic: {obs.get('confirmed_inside')}")
        print(f"  L5 mismatch events: {summary['offline']['offline_l5_mismatch_events']}")
        return 0

    scenes = {
        "L1_open": lambda: plan_goal(10.0),
        "L2_left_bypass": lambda: (plan_goal(10.0), add_box(*bf(*pose()[:2], pose()[2], 3.5, -0.5))),
        "L3_right_bypass": lambda: (plan_goal(10.0), add_box(*bf(*pose()[:2], pose()[2], 3.5, 0.5))),
        "L4_bypass_turnback": lambda: (plan_goal(10.0), add_box(*bf(*pose()[:2], pose()[2], 3.0, 0.0))),
        "L5_global_through_obstacle": lambda: (
            plan_goal(10.0),
            add_box(*bf(*pose()[:2], pose()[2], 4.0, 0.0), w=1.2, h=0.8),
        ),
    }

    all_rows: List[Dict[str, Any]] = []
    for name, setup in scenes.items():
        try:
            rows = run_scene(name, setup, duration_s=args.duration)
            summary["scenes"][name] = {
                "rows": len(rows),
                "reference_conflict_any": any(r.get("reference_conflict") for r in rows),
                "display_inside_any": any(r.get("display_inside_obstacle") for r in rows),
                "mismatch_events": sum(
                    1 for r in rows if "REFERENCE_AUTHORITY_MISMATCH" in str(r.get("events") or "")
                ),
                "turnback_events": sum(1 for r in rows if "LOOKAHEAD_TURNBACK" in str(r.get("events") or "")),
            }
            all_rows.extend(rows)
        except Exception as exc:
            summary["scenes"][name] = {"error": str(exc)}

    jsonl = os.path.join(args.out_dir, f"lookahead_trace_{stamp}.jsonl")
    csv_path = os.path.join(args.out_dir, f"lookahead_trace_{stamp}.csv")
    with open(jsonl, "w", encoding="utf-8") as fh:
        for row in all_rows:
            fh.write(json.dumps(row) + "\n")
    if all_rows:
        keys = sorted({k for r in all_rows for k in r.keys()})
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(all_rows)

    summary_path = os.path.join(args.out_dir, f"lookahead_summary_{stamp}.json")
    with open(summary_path, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)

    print(f"LIVE trace complete — {len(all_rows)} rows")
    print(f"  jsonl: {jsonl}")
    print(f"  summary: {summary_path}")
    for sn, meta in summary["scenes"].items():
        print(f"  {sn}: {meta}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
