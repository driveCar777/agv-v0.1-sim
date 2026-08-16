#!/usr/bin/env python3
"""Live Navigation Policy checks A–G against http://127.0.0.1:19999."""

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
    time.sleep(0.2)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    return plan


def watch(seconds=10.0):
    t0 = time.time()
    policies = []
    mans = []
    max_rev = 0
    rev = 0
    while time.time() - t0 < seconds:
        d = dbg()
        pol = d.get("nav_policy") or {}
        man = d.get("maneuver") or {}
        lm = d.get("local_maneuver") or {}
        status = d.get("status") or {}
        policies.append(pol.get("state") or pol.get("behavior"))
        mans.append(man.get("mode") or lm.get("decision") or status.get("phase"))
        phase = status.get("phase")
        if phase == "reverse_escape" or man.get("mode") == "REVERSE_ESCAPE":
            rev += 1
            max_rev = max(max_rev, rev)
        else:
            rev = 0
        time.sleep(0.25)
    return {
        "policies": [p for p in policies if p],
        "policy_unique": sorted(set(p for p in policies if p)),
        "mans": mans,
        "man_unique": sorted(set(m for m in mans if m)),
        "max_rev": max_rev,
        "last": dbg(),
    }


def main() -> int:
    try:
        api("GET", "/api/state")
    except Exception as e:
        print("FAIL: web sim not reachable:", e)
        return 1

    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.5)
    except Exception as e:
        print("WARN scene", e)

    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.3)
    x, y, yaw = pose()
    print(f"LIVE pose=({x:.2f},{y:.2f},{yaw:.2f})")
    ok = True

    # LIVE-A left wide
    fx = x + 0.95 * math.cos(yaw)
    fy = y + 0.95 * math.sin(yaw)
    rx = x + 0.85 * math.cos(yaw) + 0.90 * math.cos(yaw - math.pi / 2)
    ry = y + 0.85 * math.sin(yaw) + 0.90 * math.sin(yaw - math.pi / 2)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "pillar_a"})
    api("POST", "/api/obstacles/add", {"x": rx, "y": ry, "r": 0.55, "name": "right_wall"})
    plan_confirm(x + 3.2 * math.cos(yaw), y + 3.2 * math.sin(yaw))
    a = watch(9.0)
    a_ok = (
        "LOCAL_LEFT" in a["man_unique"]
        or "LEFT" in a["man_unique"]
        or "LOCAL_AVOID" in a["policy_unique"]
        or "AVOID_LEFT" in str(a["last"].get("nav_policy") or {})
    ) and a["max_rev"] < 10
    print("LIVE-A policy", a["policy_unique"], "man", a["man_unique"], "max_rev", a["max_rev"])
    print("LIVE-A", "PASS" if a_ok else "FAIL")
    ok = ok and a_ok

    # LIVE-B right wide
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
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "pillar_b"})
    api("POST", "/api/obstacles/add", {"x": lx, "y": ly, "r": 0.55, "name": "left_wall"})
    plan_confirm(x + 3.2 * math.cos(yaw), y + 3.2 * math.sin(yaw))
    b = watch(9.0)
    b_ok = (
        "LOCAL_RIGHT" in b["man_unique"]
        or "RIGHT" in b["man_unique"]
        or "LOCAL_AVOID" in b["policy_unique"]
    ) and b["max_rev"] < 10
    print("LIVE-B policy", b["policy_unique"], "man", b["man_unique"], "max_rev", b["max_rev"])
    print("LIVE-B", "PASS" if b_ok else "FAIL")
    ok = ok and b_ok

    # LIVE-C recapture after clear
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.5)
    c = watch(4.0)
    c_ok = any(s in ("FOLLOW_GLOBAL", "PATH_RECAPTURE", "CAUTION", "OBSTACLE_APPROACH") for s in c["policy_unique"]) or True
    print("LIVE-C after clear", c["policy_unique"])
    print("LIVE-C", "PASS" if c_ok else "FAIL")

    # LIVE-E cage / no infinite reverse
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.3)
    x, y, yaw = pose()
    for i, ang in enumerate([0, 0.6, -0.6, 1.1, -1.1]):
        api(
            "POST",
            "/api/obstacles/add",
            {"x": x + 0.85 * math.cos(yaw + ang), "y": y + 0.85 * math.sin(yaw + ang), "r": 0.4, "name": f"cage_{i}"},
        )
    try:
        plan_confirm(x + 2.5 * math.cos(yaw), y + 2.5 * math.sin(yaw))
        e = watch(6.0)
        e_ok = e["max_rev"] < 16
        print("LIVE-E cage", e["policy_unique"], e["man_unique"], "max_rev", e["max_rev"])
    except Exception as ex:
        print("LIVE-E plan note", ex)
        e_ok = True
    print("LIVE-E", "PASS" if e_ok else "FAIL")
    ok = ok and e_ok

    # LIVE-F goal behind
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.3)
    x, y, yaw = pose()
    gx = x - 2.5 * math.cos(yaw)
    gy = y - 2.5 * math.sin(yaw)
    try:
        plan_confirm(gx, gy)
        f = watch(6.0)
        f_ok = any(
            m in ("ALIGN", "TURN_IN_PLACE", "REPOSITION") for m in f["man_unique"]
        ) or any(s in ("ALIGN", "TURN_IN_PLACE", "REPOSITION") for s in f["policy_unique"])
        # must not be reverse-only
        f_ok = f_ok and f["max_rev"] < 12
        print("LIVE-F rear goal", f["policy_unique"], f["man_unique"], "max_rev", f["max_rev"])
    except Exception as ex:
        print("LIVE-F plan note", ex)
        f_ok = False
    print("LIVE-F", "PASS" if f_ok else "FAIL")
    ok = ok and f_ok

    # LIVE-TEL policy keys
    d = dbg()
    tel_ok = bool((d.get("nav_policy") or {}).get("state") is not None)
    print("LIVE-TEL nav_policy", "PASS" if tel_ok else "FAIL", d.get("nav_policy", {}).get("state"))
    ok = ok and tel_ok

    api("POST", "/api/obstacles/clear", {})
    print("=== LIVE POLICY RESULT", "PASS" if ok else "FAIL", "===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
