# -*- coding: utf-8 -*-
"""Tune Scene A stage2 until L=INVALID R=VALID T=VALID with commitment held."""
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
    a = (api("GET", "/api/state").get("agv") or {})
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def pst():
    d = api("GET", "/api/nav/debug")
    dbg = d.get("debug") if isinstance(d.get("debug"), dict) else d
    pr = dbg.get("probe") or (dbg.get("phase4") or {}).get("probe") or {}
    sw = dbg.get("side_switch") or {}
    c = dbg.get("commitment") or (dbg.get("phase4") or {}).get("commitment") or {}
    man = (dbg.get("maneuver") or {}).get("mode")

    def st(k):
        v = pr.get(k) or {}
        return v.get("status") if isinstance(v, dict) else v

    obs = (api("GET", "/api/state").get("obstacles") or api("GET", "/api/state").get("dyn_obstacles") or [])
    return {
        "mode": man,
        "active": c.get("active"),
        "side": c.get("side"),
        "L": st("left"),
        "R": st("right"),
        "T": st("turn_in_place") or st("turn"),
        "auth": sw.get("authorized"),
        "reason": sw.get("reason"),
        "n_obs": len(obs) if isinstance(obs, list) else obs,
        "obs": obs if isinstance(obs, list) else [],
    }


def setup():
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            api("POST", p, {})
        except Exception:
            pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.3)
    wrap = api("POST", "/api/nav/plan", {"x": -20.0, "y": 0.0})
    plan = wrap.get("api") or wrap
    assert int(plan.get("ret_code")) == 0
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.9)


def main():
    setup()
    t0 = time.time()
    while time.time() - t0 < 2.5:
        time.sleep(0.1)
    x, y, yaw = pose()
    px, py = bf(x, y, yaw, 1.15, -0.12)
    api("POST", "/api/obstacles/add", {"x": px, "y": py, "r": 0.42, "name": "e_a_pillar"})
    for name, fwd, lat, r in (
        ("e_a_ra", 1.05, -0.95, 0.55),
        ("e_a_rb", 1.45, -1.15, 0.48),
        ("e_a_rc", 0.75, -0.85, 0.40),
    ):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": name})
    print("S1", pst())
    left_at = None
    while time.time() - t0 < 10:
        st = pst()
        if "LEFT" in str(st["mode"] or "") and left_at is None:
            left_at = time.time()
            print("LEFT", st)
        # stage2 very early: 0.25s after LEFT
        if left_at and (time.time() - left_at) >= 0.25:
            break
        time.sleep(0.05)
    x, y, yaw = pose()
    print("pose", x, y, yaw)
    # remove seals
    for name in ("e_a_ra", "e_a_rb", "e_a_rc"):
        print("remove", name, api("POST", "/api/obstacles/remove", {"name": name}))
    # try several pillar nudges without left walls first
    for nudge in (0.35, 0.55, 0.75, 1.0, 1.2):
        api("POST", "/api/obstacles/remove", {"name": "e_a_pillar"})
        # also remove prior left walls
        for i in range(5):
            try:
                api("POST", "/api/obstacles/remove", {"name": f"e_a_left_{i}"})
            except Exception:
                pass
        ox = px + nudge * math.cos(yaw + math.pi / 2)
        oy = py + nudge * math.sin(yaw + math.pi / 2)
        api("POST", "/api/obstacles/add", {"x": ox, "y": oy, "r": 0.46, "name": "e_a_pillar"})
        time.sleep(0.45)
        st = pst()
        print(f"nudge={nudge} LRT=({st['L']},{st['R']},{st['T']}) auth={st['auth']} reason={st['reason']} mode={st['mode']} active={st['active']} n_obs={st['n_obs']}")
        if st["L"] == "INVALID" and st["R"] == "VALID" and st["T"] == "VALID":
            print("HIT geometry with pillar only")
            # add left walls carefully
            for i, (fwd, lat, r) in enumerate([(1.2, 1.3, 0.36), (1.5, 1.5, 0.36)]):
                bx, by = bf(x, y, yaw, fwd, lat)
                api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"e_a_left_{i}"})
            time.sleep(0.45)
            st2 = pst()
            print("with left walls", st2)
            break


if __name__ == "__main__":
    main()
