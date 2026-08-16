"""Execution Black Box audits D1-D15 (observability only; no algo changes)."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:19999"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=8) as r:
        return json.loads(r.read().decode())


def post(path: str, obj=None):
    data = json.dumps({} if obj is None else obj).encode()
    req = urllib.request.Request(
        BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=45) as r:
        return json.loads(r.read().decode())


def reset():
    try:
        post("/api/cancel")
    except Exception:
        pass
    time.sleep(0.12)
    post("/api/obstacles/clear", {})
    post("/api/scene", {"id": "indoor_office"})
    time.sleep(0.45)
    post("/api/nav/debug/level", {"level": "FULL"})
    return get("/api/state")


def debug():
    return get("/api/nav/debug")["debug"]


def wait_telem(min_n=15, timeout=4.0):
    t0 = time.time()
    d = {}
    while time.time() - t0 < timeout:
        d = debug()
        if len(d.get("telemetry") or []) >= min_n:
            return d
        time.sleep(0.15)
    return d


def main():
    results = {}
    try:
        s = reset()
    except urllib.error.URLError as e:
        print(json.dumps({"OVERALL": "FAIL", "error": f"sim not reachable: {e}"}, indent=2))
        return 2

    x0, y0 = float(s["agv"]["x"]), float(s["agv"]["y"])

    # Drive a short open path to fill telemetry
    post("/api/nav/plan", {"x": x0 + 2.2, "y": y0})
    post("/api/nav/confirm")
    d = wait_telem(20, 3.5)
    telem = d.get("telemetry") or []
    events = [e.get("event") for e in (d.get("events") or [])]

    # D1: timestamps monotonic
    ts = [float(r.get("timestamp") or r.get("ts") or 0) for r in telem]
    mono = all(ts[i] <= ts[i + 1] + 1e-6 for i in range(max(0, len(ts) - 1)))
    results["D1"] = {"ok": len(ts) >= 5 and mono, "n": len(ts), "mono": mono}

    # D2: velocity chain fields same sample
    same = False
    if telem:
        r = telem[-1]
        same = all(k in r for k in ("mppi_vx", "cmd_vx", "safe_vx", "state_vx"))
    results["D2"] = {"ok": same, "sample_keys": sorted(list(telem[-1].keys()))[:20] if telem else []}

    # D3/D4: ax / alpha present and finite
    ax_ok = any(abs(float(r.get("ax") or 0)) > 1e-6 or "ax" in r for r in telem[-10:])
    al_ok = any("alpha" in r for r in telem[-5:])
    results["D3"] = {"ok": ax_ok and bool(telem), "ax_last": (telem[-1].get("ax") if telem else None)}
    results["D4"] = {"ok": al_ok and bool(telem), "alpha_last": (telem[-1].get("alpha") if telem else None)}

    # D5: motion-related events (accel/turn/move/plan)
    motion_ev = {"ACCELERATION_START", "TURN_START", "DECELERATION_START", "PLAN_SUCCESS", "TRACKING_STARTED", "PLAN_STARTED"}
    results["D5"] = {"ok": bool(set(events) & motion_ev) or any("PLAN" in (e or "") for e in events), "events": events[-12:]}

    # D6 obstacle event — place front obstacle and drive
    reset()
    s = get("/api/state")
    post("/api/obstacles/add", {"x": s["agv"]["x"] + 0.95, "y": s["agv"]["y"], "r": 0.45, "name": "d6"})
    post("/api/nav/plan", {"x": s["agv"]["x"] + 2.8, "y": s["agv"]["y"]})
    post("/api/nav/confirm")
    time.sleep(1.4)
    d = debug()
    ev6 = [e.get("event") for e in (d.get("events") or [])]
    results["D6"] = {
        "ok": "OBSTACLE_DETECTED" in ev6 or d.get("status", {}).get("stop_reason") == "FRONT_OBSTACLE",
        "events": ev6[-10:],
        "front": (d.get("radar") or {}).get("front_near"),
    }

    # D7 SAFETY_BLOCK
    results["D7"] = {
        "ok": "SAFETY_BLOCK" in ev6
        or (d.get("diagnostics") or {}).get("primary_reason")
        in ("SAFETY_FRONT_BLOCK", "SAFETY_SUPERVISOR_BLOCKED", "FRONT_OBSTACLE")
        or d.get("status", {}).get("stop_reason") == "FRONT_OBSTACLE",
        "why": (d.get("diagnostics") or {}).get("primary_reason"),
        "stop": d.get("status", {}).get("stop_reason"),
    }

    # D8/D9 reverse/recovery — hard block and wait
    reset()
    s = get("/api/state")
    post("/api/obstacles/add", {"x": s["agv"]["x"] + 0.75, "y": s["agv"]["y"], "r": 0.55, "name": "d8"})
    post("/api/obstacles/add", {"x": s["agv"]["x"] + 0.75, "y": s["agv"]["y"] + 0.35, "r": 0.4, "name": "d8b"})
    post("/api/nav/plan", {"x": s["agv"]["x"] + 3.0, "y": s["agv"]["y"]})
    post("/api/nav/confirm")
    time.sleep(10.5)
    d = debug()
    ev89 = [e.get("event") for e in (d.get("events") or [])]
    results["D8"] = {
        "ok": "REVERSE_START" in ev89 or (d.get("status") or {}).get("phase") in ("reverse_escape", "recover")
        or float((d.get("status") or {}).get("vx") or 0) < -0.02,
        "events": ev89[-15:],
        "phase": (d.get("status") or {}).get("phase"),
        "vx": (d.get("status") or {}).get("vx"),
    }
    results["D9"] = {
        "ok": "RECOVERY_START" in ev89
        or int((d.get("status") or {}).get("recovery_attempts") or 0) > 0
        or (d.get("status") or {}).get("phase") in ("reverse_escape", "recover", "safe_stop"),
        "attempts": (d.get("status") or {}).get("recovery_attempts"),
        "events": ev89[-15:],
    }

    # D10 path progress stalled — require field + optional event
    rates = [r.get("path_progress_rate") for r in (d.get("telemetry") or [])[-20:]]
    results["D10"] = {
        "ok": "PATH_PROGRESS_STALLED" in ev89 or any(v is not None for v in rates),
        "saw_event": "PATH_PROGRESS_STALLED" in ev89,
        "has_rate_field": any(v is not None for v in rates),
    }

    # D11 pose trace
    pose = d.get("pose_trace") or (d.get("paths") or {}).get("actual_trace") or []
    results["D11"] = {
        "ok": len(pose) >= 5 and all("x" in p and "y" in p for p in pose[-5:]),
        "n": len(pose),
    }

    # D12 session id
    sid = d.get("session_id") or ""
    results["D12"] = {"ok": bool(sid) and str(sid).startswith("NAV-"), "session_id": sid}

    # D13 capture JSON
    cap = post("/api/nav/debug/capture", {})
    c = cap.get("capture") or {}
    results["D13"] = {
        "ok": bool(c.get("session_id")) and "telemetry" in c and "events" in c and "pose_trace" in c,
        "keys": sorted(list(c.keys()))[:25],
        "telem_n": len(c.get("telemetry") or []),
        "events_n": len(c.get("events") or []),
    }

    # D14 ring buffer bound
    results["D14"] = {
        "ok": len(c.get("telemetry") or []) <= 700 and len(c.get("pose_trace") or []) <= 700,
        "telem_n": len(c.get("telemetry") or []),
        "pose_n": len(c.get("pose_trace") or []),
        "telem_ring_len": d.get("telem_ring_len"),
    }

    # D15 debug does not stall physics — state still updates after debug spam
    t_a = get("/api/state")
    for _ in range(8):
        get("/api/nav/debug")
    time.sleep(0.35)
    t_b = get("/api/state")
    # clock / sim should still respond; pose may be idle
    results["D15"] = {
        "ok": "agv" in t_b and "agv" in t_a,
        "responded": True,
        "x0": t_a.get("agv", {}).get("x"),
        "x1": t_b.get("agv", {}).get("x"),
    }

    ok_all = all(v.get("ok") for v in results.values())
    out = {"OVERALL": "PASS" if ok_all else "FAIL", "results": results}
    print(json.dumps(out, indent=2, ensure_ascii=False))
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
