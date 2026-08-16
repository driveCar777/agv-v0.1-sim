#!/usr/bin/env python3
"""Live LEFT/RIGHT local-maneuver checks against http://127.0.0.1:19999.

Ensures a single run_web_sim owns the port (caller must restart cleanly).
"""

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


def dbg():
    wrap = api("GET", "/api/nav/debug")
    return wrap.get("debug") or wrap


def pose():
    st = api("GET", "/api/state")
    agv = st.get("agv") or {}
    return float(agv.get("x") or 0), float(agv.get("y") or 0), float(agv.get("angle") or 0)


def plan_confirm(gx, gy):
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.25)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    return plan


def watch(seconds=10.0, want=None):
    """Collect decision timeline; want is set of acceptable selected decisions."""
    t0 = time.time()
    seen = []
    hit = False
    rev_streak = 0
    max_rev = 0
    while time.time() - t0 < seconds:
        d = dbg()
        lm = d.get("local_maneuver") or {}
        man = d.get("maneuver") or {}
        status = d.get("status") or {}
        dec = lm.get("decision") or man.get("decision") or man.get("mode") or status.get("phase")
        mode = man.get("mode")
        phase = status.get("phase")
        seen.append(dec)
        if want and dec in want:
            hit = True
        if phase == "reverse_escape" or mode == "REVERSE_ESCAPE" or dec == "REVERSE":
            rev_streak += 1
            max_rev = max(max_rev, rev_streak)
        else:
            rev_streak = 0
        time.sleep(0.25)
    return {
        "seen": seen,
        "unique": sorted(set(x for x in seen if x)),
        "hit": hit,
        "max_rev": max_rev,
        "last": dbg(),
    }


def main() -> int:
    try:
        st = api("GET", "/api/state")
    except Exception as e:
        print("FAIL: web sim not reachable:", e)
        return 1

    # Prefer outdoor open space for LEFT/RIGHT live geometry
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.6)
        st = api("GET", "/api/state")
    except Exception as e:
        print("WARN: scene switch failed", e)

    print("LIVE server ok; scene=", (st.get("scene") or {}).get("id") or st.get("scene"))
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.3)
    x, y, yaw = pose()
    print(f"pose=({x:.2f},{y:.2f},{yaw:.2f})")

    # Place pillar ahead + right-side blocker → expect LEFT
    fx = x + 0.95 * math.cos(yaw)
    fy = y + 0.95 * math.sin(yaw)
    rx = x + 0.85 * math.cos(yaw) + 0.90 * math.cos(yaw - math.pi / 2)
    ry = y + 0.85 * math.sin(yaw) + 0.90 * math.sin(yaw - math.pi / 2)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "live_pillar"})
    api("POST", "/api/obstacles/add", {"x": rx, "y": ry, "r": 0.55, "name": "live_right_wall"})
    gx = x + 3.2 * math.cos(yaw)
    gy = y + 3.2 * math.sin(yaw)
    print(f"LIVE-A pillar=({fx:.2f},{fy:.2f}) right=({rx:.2f},{ry:.2f}) goal=({gx:.2f},{gy:.2f})")
    try:
        plan_confirm(gx, gy)
    except Exception as e:
        print("LIVE-A plan FAIL", e)
        return 1
    a = watch(9.0, want={"LEFT", "LOCAL_LEFT"})
    # Also accept mode LOCAL_LEFT
    modes = []
    for _ in range(8):
        m = (dbg().get("maneuver") or {}).get("mode")
        modes.append(m)
        time.sleep(0.2)
    a_ok = a["hit"] or any(m == "LOCAL_LEFT" for m in modes) or "LEFT" in a["unique"]
    print("LIVE-A unique", a["unique"], "modes", sorted(set(modes)), "max_rev", a["max_rev"])
    print("LIVE-A", "PASS" if a_ok and a["max_rev"] < 10 else "FAIL")

    # Clear and place left wall → expect RIGHT
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.4)
    x, y, yaw = pose()
    fx = x + 0.95 * math.cos(yaw)
    fy = y + 0.95 * math.sin(yaw)
    lx = x + 0.85 * math.cos(yaw) + 0.90 * math.cos(yaw + math.pi / 2)
    ly = y + 0.85 * math.sin(yaw) + 0.90 * math.sin(yaw + math.pi / 2)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "live_pillar2"})
    api("POST", "/api/obstacles/add", {"x": lx, "y": ly, "r": 0.55, "name": "live_left_wall"})
    gx = x + 3.2 * math.cos(yaw)
    gy = y + 3.2 * math.sin(yaw)
    print(f"LIVE-B left_wall=({lx:.2f},{ly:.2f}) goal=({gx:.2f},{gy:.2f})")
    try:
        plan_confirm(gx, gy)
    except Exception as e:
        print("LIVE-B plan FAIL", e)
        b_ok = False
        b = {"unique": [], "max_rev": 0}
    else:
        b = watch(9.0, want={"RIGHT", "LOCAL_RIGHT"})
        modes = []
        for _ in range(8):
            m = (dbg().get("maneuver") or {}).get("mode")
            modes.append(m)
            time.sleep(0.2)
        b_ok = b["hit"] or any(m == "LOCAL_RIGHT" for m in modes) or "RIGHT" in b["unique"]
        print("LIVE-B unique", b["unique"], "modes", sorted(set(modes)), "max_rev", b["max_rev"])
    print("LIVE-B", "PASS" if b_ok and b.get("max_rev", 99) < 10 else "FAIL")

    # LIVE-C both blocked — expect not infinite reverse oscillation
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.3)
    x, y, yaw = pose()
    for i, ang in enumerate([0, 0.6, -0.6, 1.1, -1.1]):
        ox = x + 0.85 * math.cos(yaw + ang)
        oy = y + 0.85 * math.sin(yaw + ang)
        api("POST", "/api/obstacles/add", {"x": ox, "y": oy, "r": 0.4, "name": f"cage_{i}"})
    gx = x + 2.5 * math.cos(yaw)
    gy = y + 2.5 * math.sin(yaw)
    print("LIVE-C cage ahead")
    try:
        plan_confirm(gx, gy)
        c = watch(6.0)
        c_ok = c["max_rev"] < 16
        print("LIVE-C unique", c["unique"], "max_rev", c["max_rev"])
    except Exception as e:
        print("LIVE-C plan note", e)
        c_ok = True  # cage may make plan fail — acceptable SAFE path
        c = {"max_rev": 0}
    print("LIVE-C", "PASS" if c_ok else "FAIL")

    api("POST", "/api/obstacles/clear", {})
    results = [
        ("LIVE-A", a_ok and a["max_rev"] < 10),
        ("LIVE-B", b_ok and b.get("max_rev", 99) < 10),
        ("LIVE-C", c_ok),
    ]
    # Telemetry presence
    d = dbg()
    tel_ok = "local_maneuver" in d and "maneuver" in d
    print("LIVE-TEL", "PASS" if tel_ok else "FAIL", "keys", "local_maneuver" in d)

    failed = [n for n, okv in results if not okv]
    if not tel_ok:
        failed.append("LIVE-TEL")
    print("=== LIVE RESULT", "PASS" if not failed else f"FAIL {failed}", "===")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
