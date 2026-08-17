#!/usr/bin/env python3
"""P0-C LIVE — Path Feasibility Timeline (read-only HTTP; no control change).

Prints GLOBAL vs KINEMATIC vs STOP over scenes A–F.
Requires web sim at BASE (default 127.0.0.1:19999).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 8.0) -> Dict[str, Any]:
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
    time.sleep(0.25)


def plan_to(gx: float, gy: float) -> None:
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.7)


def add_obs(x: float, y: float, r: float, name: str) -> None:
    api("POST", "/api/obstacles/add", {"x": x, "y": y, "r": r, "name": name})


def row(t: float, scene: str) -> str:
    st = api("GET", "/api/state")
    prev = api("GET", "/api/nav/preview")
    try:
        summ = api("GET", "/api/logs/summary")
    except Exception:
        summ = {}
    nav = st.get("nav") or {}
    gref = prev.get("global_reference") or nav.get("global_reference") or {}
    kv = prev.get("kinematic_validation") or nav.get("kinematic_validation") or {}
    g_m = _f(gref.get("preview_m"))
    kst = kv.get("kinematic_status") or kv.get("status") or gref.get("kinematic_status") or "—"
    reason = kv.get("reason") or "—"
    kappa = kv.get("max_curvature")
    radius = kv.get("min_turn_radius_m")
    wreq = kv.get("max_required_w_rad_s")
    vfeas = kv.get("max_feasible_speed_mps")
    clr = kv.get("min_clearance_m")
    coll = kv.get("swept_collision")
    fid = kv.get("first_invalid_distance_m")
    limited = kv.get("speed_limited")
    owner = summ.get("immediate_owner") or summ.get("likely_owner") or "—"
    stop = nav.get("stop_reason") or summ.get("stop_reason") or "NONE"
    rev = kv.get("path_revision") or gref.get("path_revision")
    cache = kv.get("cache_hit")
    return (
        f"T+{t:5.2f}  SCENE={scene}  GLOBAL={g_m if g_m is not None else '—'}m  "
        f"KINEMATIC={kst}  reason={reason}  "
        f"κ={kappa}  R={radius}  w_req={wreq}  v_feas={vfeas}  "
        f"clr={clr}  coll={coll}  first_invalid={fid}  "
        f"speed_limited={limited}  rev={rev}  cache={cache}  "
        f"STOP={stop}  OWNER={owner}"
    )


def sample_scene(scene: str, seconds: float, hz: float) -> None:
    dt = 1.0 / max(0.5, float(hz))
    t0 = time.time()
    while time.time() - t0 <= float(seconds):
        t = time.time() - t0
        try:
            print(row(t, scene), flush=True)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            print(f"T+{t:5.2f}  SCENE={scene}  FETCH_FAIL {e}", flush=True)
        time.sleep(dt)


def main() -> int:
    global BASE
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()
    BASE = args.base.rstrip("/")
    print("=== P0-C Kinematic Validation LIVE Timeline ===")
    print(f"BASE={BASE} window={args.seconds}s/scene")
    print("TIMELINE")
    try:
        api("GET", "/api/state")
    except Exception as e:
        print(f"SIM_UNREACHABLE {e}")
        return 2

    x, y, yaw = pose()
    scenes = [
        ("A", lambda: plan_to(*bf(x, y, yaw, 8.0, 0.0)), "open straight"),
        ("B", lambda: plan_to(*bf(x, y, yaw, 6.0, 1.6)), "gentle turn"),
        ("C", lambda: plan_to(*bf(x, y, yaw, 1.2, 6.0)), "90-ish path"),
    ]
    for name, setup, note in scenes:
        print(f"-- SCENE {name} {note}")
        cancel_clear()
        x, y, yaw = pose()
        try:
            setup()
        except Exception as e:
            print(f"SCENE {name} PLAN_FAIL {e}")
            continue
        sample_scene(name, args.seconds, args.hz)

    print("-- SCENE D narrow curved corridor (inject after plan)")
    cancel_clear()
    x, y, yaw = pose()
    try:
        plan_to(*bf(x, y, yaw, 6.0, 1.8))
        for i, (fwd, lat, r) in enumerate(
            [(1.2, 0.55, 0.22), (2.0, 0.55, 0.22), (2.8, 0.55, 0.22), (1.2, -0.55, 0.22), (2.0, -0.55, 0.22), (2.8, -0.55, 0.22)]
        ):
            px, py = bf(x, y, yaw, fwd, lat)
            add_obs(px, py, r, f"d_wall_{i}")
        time.sleep(0.5)
        sample_scene("D", args.seconds, args.hz)
    except Exception as e:
        print(f"SCENE D FAIL {e}")

    print("-- SCENE E inner-turn obstacle")
    cancel_clear()
    x, y, yaw = pose()
    try:
        plan_to(*bf(x, y, yaw, 5.5, 2.2))
        px, py = bf(x, y, yaw, 2.4, 0.85)
        add_obs(px, py, 0.38, "e_inner")
        time.sleep(0.5)
        sample_scene("E", args.seconds, args.hz)
    except Exception as e:
        print(f"SCENE E FAIL {e}")

    print("-- SCENE F outer-turn obstacle")
    cancel_clear()
    x, y, yaw = pose()
    try:
        plan_to(*bf(x, y, yaw, 5.5, 2.2))
        px, py = bf(x, y, yaw, 2.4, -0.95)
        add_obs(px, py, 0.38, "f_outer")
        time.sleep(0.5)
        sample_scene("F", args.seconds, args.hz)
    except Exception as e:
        print(f"SCENE F FAIL {e}")

    cancel_clear()
    print("P0-C LIVE TIMELINE DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
