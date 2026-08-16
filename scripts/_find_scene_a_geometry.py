# -*- coding: utf-8 -*-
"""LIVE geometry finder for Scene A stage2: need L=INVALID R=VALID T=VALID."""
from __future__ import annotations

import json
import math
import time
import urllib.request

BASE = "http://127.0.0.1:19999"


def api(method, path, body=None):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode("utf-8"))


def bf(x, y, yaw, fwd, lat):
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def pose():
    st = api("GET", "/api/state")
    a = st.get("agv") or {}
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def probe_statuses():
    d = api("GET", "/api/nav/debug")
    dbg = d.get("debug") if isinstance(d.get("debug"), dict) else d
    pr = dbg.get("probe") or (dbg.get("phase4") or {}).get("probe") or {}
    sw = dbg.get("side_switch") or (dbg.get("phase4") or {}).get("side_switch") or {}
    man = (dbg.get("maneuver") or {}).get("mode")
    c = dbg.get("commitment") or (dbg.get("phase4") or {}).get("commitment") or {}

    def st(key_a, key_b=None):
        v = pr.get(key_a) or (pr.get(key_b) if key_b else None) or {}
        if isinstance(v, dict):
            return v.get("status")
        return v

    return {
        "mode": man,
        "commit": c.get("side") or c.get("committed_side"),
        "L": st("left", "LEFT"),
        "R": st("right", "RIGHT"),
        "T": st("turn_in_place") or st("turn", "TURN_IN_PLACE"),
        "F": st("forward", "FORWARD"),
        "B": st("backward", "BACKWARD"),
        "auth": sw.get("authorized"),
        "reason": sw.get("reason") or sw.get("primary_reason"),
        "from": sw.get("from_side"),
        "to": sw.get("to_side"),
        "cur_p": sw.get("current_probe_status"),
        "alt_p": sw.get("alternative_probe_status"),
        "turn_p": sw.get("turn_probe_status"),
    }


def setup():
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    try:
        api("POST", "/api/nav/stop", {})
    except Exception:
        pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.3)
    wrap = api("POST", "/api/nav/plan", {"x": -20.0, "y": 0.0})
    plan = wrap.get("api") or wrap
    if plan.get("ret_code") is None or int(plan.get("ret_code")) != 0:
        raise RuntimeError(plan)
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.8)


def stage1(x, y, yaw):
    px, py = bf(x, y, yaw, 1.15, -0.12)
    r1x, r1y = bf(x, y, yaw, 1.05, -0.95)
    r2x, r2y = bf(x, y, yaw, 1.45, -1.15)
    r3x, r3y = bf(x, y, yaw, 0.75, -0.85)
    api("POST", "/api/obstacles/add", {"x": px, "y": py, "r": 0.42, "name": "s1_pillar"})
    api("POST", "/api/obstacles/add", {"x": r1x, "y": r1y, "r": 0.55, "name": "s1_ra"})
    api("POST", "/api/obstacles/add", {"x": r2x, "y": r2y, "r": 0.48, "name": "s1_rb"})
    api("POST", "/api/obstacles/add", {"x": r3x, "y": r3y, "r": 0.40, "name": "s1_rc"})
    return {"pillar": {"x": px, "y": py}}


def try_stage2(name, x, y, yaw, pillar, left_specs, remnant):
    api("POST", "/api/obstacles/clear", {})
    if pillar:
        ox = float(pillar["x"]) + 0.12 * math.cos(yaw + math.pi / 2)
        oy = float(pillar["y"]) + 0.12 * math.sin(yaw + math.pi / 2)
        api("POST", "/api/obstacles/add", {"x": ox, "y": oy, "r": 0.44, "name": "s2_pillar"})
    for i, (fwd, lat, r) in enumerate(left_specs):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"s2_l_{i}"})
    if remnant:
        rx, ry = bf(x, y, yaw, remnant[0], remnant[1])
        api("POST", "/api/obstacles/add", {"x": rx, "y": ry, "r": remnant[2], "name": "s2_rr"})
    time.sleep(0.55)
    st = probe_statuses()
    print(f"[{name}] {st}")
    return st


def main():
    setup()
    # wait motion
    t0 = time.time()
    while time.time() - t0 < 3.0:
        x, y, yaw = pose()
        st = probe_statuses()
        if st["mode"] in ("FORWARD_TRACK", "FORWARD_TURN") or time.time() - t0 > 1.5:
            break
        time.sleep(0.1)
    x, y, yaw = pose()
    s1 = stage1(x, y, yaw)
    pillar = s1.get("pillar") or {}
    print("stage1 at", round(x, 2), round(y, 2), "mode wait LEFT...")
    left_at = None
    while time.time() - t0 < 12.0:
        st = probe_statuses()
        if "LEFT" in str(st["mode"] or "") and left_at is None:
            left_at = time.time()
            print("LOCAL_LEFT", st)
        if left_at and (time.time() - left_at) >= 1.0:
            break
        time.sleep(0.12)
    x, y, yaw = pose()
    print("pose@s2", round(x, 3), round(y, 3), round(yaw, 3))
    variants = [
        (
            "wide_right_far",
            [(0.85, 0.70, 0.36), (1.15, 0.90, 0.38), (1.45, 1.10, 0.34), (0.70, 0.85, 0.30)],
            (2.2, -1.9, 0.18),
        ),
        (
            "wide_right_none",
            [(0.85, 0.70, 0.36), (1.15, 0.90, 0.38), (1.45, 1.10, 0.34), (0.70, 0.85, 0.30)],
            None,
        ),
        (
            "mild_left_open_right",
            [(1.0, 0.85, 0.34), (1.3, 1.05, 0.36), (0.75, 0.95, 0.30)],
            (2.0, -1.8, 0.16),
        ),
        (
            "left_arc_only",
            [(0.9, 0.75, 0.32), (1.2, 0.95, 0.34), (1.5, 1.15, 0.32)],
            None,
        ),
        (
            "hard_clog_open_r",
            [
                (0.85, 0.55, 0.36),
                (1.15, 0.75, 0.38),
                (1.40, 0.95, 0.34),
                (0.65, 0.70, 0.30),
                (1.55, 0.55, 0.32),
            ],
            (2.4, -2.0, 0.15),
        ),
        (
            "pillar_only_open_r",
            [],
            None,
        ),
        (
            "front_nudge_left_wall_far",
            [(1.2, 1.2, 0.40), (1.5, 1.4, 0.40)],
            None,
        ),
    ]
    for name, specs, rem in variants:
        try_stage2(name, x, y, yaw, pillar, specs, rem)
        time.sleep(0.25)


if __name__ == "__main__":
    main()
