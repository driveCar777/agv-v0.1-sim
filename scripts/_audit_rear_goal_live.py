#!/usr/bin/env python3
"""Live rear-goal maneuver check against running web sim (http://127.0.0.1:19999)."""

from __future__ import annotations

import json
import math
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=12) as r:
        return json.loads(r.read().decode("utf-8"))


def main() -> int:
    try:
        st = api("GET", "/api/state")
    except Exception as e:
        print("SKIP: web sim not reachable:", e)
        return 0

    agv = st.get("agv") or st
    x = float(agv.get("x") or 0)
    y = float(agv.get("y") or 0)
    yaw = float(agv.get("angle") or agv.get("yaw") or 0)
    gx = x - 3.0 * math.cos(yaw)
    gy = y - 3.0 * math.sin(yaw)
    print(f"pose=({x:.2f},{y:.2f},{yaw:.2f}) rear_goal=({gx:.2f},{gy:.2f})")

    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.4)
    plan_wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = plan_wrap.get("api") or plan_wrap
    rc = plan.get("ret_code")
    if rc is None:
        rc = plan_wrap.get("ret_code")
    print("plan", rc, "waypoints", plan.get("waypoints") or plan_wrap.get("waypoints"))
    if rc is None or int(rc) != 0:
        print("FAIL plan", plan_wrap)
        return 1
    conf = api("POST", "/api/nav/confirm", {})
    print("confirm", conf)

    modes = []
    reverse_streak = 0
    max_rev = 0
    saw_align = False
    saw_forward_after = False
    for i in range(48):
        time.sleep(0.25)
        wrap = api("GET", "/api/nav/debug")
        dbg = wrap.get("debug") or wrap
        man = dbg.get("maneuver") or {}
        status = dbg.get("status") or {}
        mode = man.get("mode") or status.get("phase")
        phase = status.get("phase")
        vx = status.get("vx")
        modes.append(mode)
        if phase == "reverse_escape" or mode == "REVERSE_ESCAPE":
            reverse_streak += 1
            max_rev = max(max_rev, reverse_streak)
        else:
            reverse_streak = 0
        if mode in ("ALIGN", "TURN_IN_PLACE", "REPOSITION", "FORWARD_TURN"):
            saw_align = True
        if saw_align and mode in ("FORWARD_TRACK", "POST_TURN"):
            saw_forward_after = True
        if i % 4 == 0:
            fr = dbg.get("forward_reverse") or {}
            print(
                f"t=+{i*0.25:.1f}s mode={mode} phase={phase} vx={vx} "
                f"herr={man.get('heading_error')} reason={man.get('reason')} "
                f"fwd={fr.get('forward_reason')}"
            )
        if status.get("nav_mode") == "arrived":
            print("ARRIVED")
            break
        if status.get("stop_reason") == "FAILED":
            print("FAILED early stop_reason=FAILED")
            break

    print("modes_unique", sorted(set(m for m in modes if m)))
    print("saw_align/turn/repos", saw_align)
    print("saw_forward_after_align", saw_forward_after)
    print("max_reverse_streak_samples", max_rev)
    ok = saw_align and max_rev < 12
    print("RESULT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
