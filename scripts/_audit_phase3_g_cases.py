"""Phase-3 Global path quality cases G1-G8 (+ phase2 sanity)."""
from __future__ import annotations

import json
import math
import time
import urllib.request

BASE = "http://127.0.0.1:19999"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.loads(r.read().decode())


def post(path: str, obj=None):
    data = json.dumps({} if obj is None else obj).encode()
    req = urllib.request.Request(
        BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read().decode())


def reset():
    post("/api/cancel")
    time.sleep(0.15)
    post("/api/scene", {"id": "indoor_office"})
    time.sleep(0.5)
    return get("/api/state")


def sample(n=18, dt=0.35):
    rows = []
    for i in range(n):
        time.sleep(dt)
        s = get("/api/state")
        a, nav, safety = s["agv"], s["nav"], s.get("safety") or {}
        rows.append(
            {
                "t": round((i + 1) * dt, 1),
                "vx": round(a["vx"], 3),
                "w": round(a["w"], 3),
                "path_s": nav.get("path_progress_s"),
                "stop": safety.get("stop_reason") or nav.get("stop_reason"),
                "phase": nav.get("phase"),
                "rec": nav.get("recovery_attempts"),
                "front": safety.get("front_near"),
            }
        )
    return rows


def w_sign_changes(rows) -> int:
    n = 0
    prev = None
    for r in rows:
        s = 0 if abs(r["w"]) < 0.02 else (1 if r["w"] > 0 else -1)
        if prev is not None and s != 0 and prev != 0 and s != prev:
            n += 1
        if s != 0:
            prev = s
    return n


def main():
    results = {}
    s0 = reset()
    x0, y0 = s0["agv"]["x"], s0["agv"]["y"]
    print("spawn", x0, y0)

    # G1 open straight via plan_quality
    pq = post("/api/nav/plan_quality", {"x": x0 + 2.0, "y": y0, "compare": True})
    assert pq.get("success"), pq
    v = pq["variants"]
    print("G1 variants", {k: (v[k]["ratio"], v[k]["turns"], v[k]["min_clearance"]) for k in "ABCD"})
    results["G1"] = {
        "ok": v["D"]["direct_safe"] and v["D"]["ratio"] < 1.05 and v["D"]["turns"] <= 1,
        "ratio": v["D"]["ratio"],
        "turns": v["D"]["turns"],
        "min_c": v["D"]["min_clearance"],
        "direct": v["D"]["direct_safe"],
    }

    # G2 south
    reset()
    s = get("/api/state")
    pq = post("/api/nav/plan_quality", {"x": s["agv"]["x"], "y": s["agv"]["y"] - 4.0, "compare": True})
    v = pq["variants"]
    results["G2"] = {
        "ok": v["D"]["direct_safe"] and v["D"]["ratio"] < 1.05 and v["D"]["turns"] <= 2,
        "ratio": v["D"]["ratio"],
        "turns": v["D"]["turns"],
        "min_c": v["D"]["min_clearance"],
        "direct": v["D"]["direct_safe"],
    }

    # G3 two-way / natural: far east goal; D should not hug worse than A on mean clearance
    reset()
    s = get("/api/state")
    pq = post("/api/nav/plan_quality", {"x": 16.0, "y": -2.0, "compare": True})
    v = pq["variants"]
    results["G3"] = {
        "ok": v["D"]["min_clearance"] + 1e-6 >= v["A"]["min_clearance"] and v["D"]["turns"] < v["A"]["turns"],
        "ratio": v["D"]["ratio"],
        "turns": v["D"]["turns"],
        "min_c": v["D"]["min_clearance"],
        "A_min_c": v["A"]["min_clearance"],
        "A_turns": v["A"]["turns"],
        "direct": v["D"]["direct_safe"],
    }

    # G4 blocked straight
    reset()
    pq = post("/api/nav/plan_quality", {"x": 16.0, "y": -2.0, "compare": False})
    pl = pq.get("planning") or {}
    results["G4"] = {
        "ok": (not pl.get("direct_path_safe", True)) and pl.get("path_ratio", 99) < 4.5,
        "ratio": pl.get("path_ratio"),
        "turns": pl.get("final_turn_count"),
        "min_c": pl.get("min_clearance"),
        "direct": pl.get("direct_path_safe"),
        "len": pl.get("final_path_length"),
    }

    # G5 narrow-ish: if path exists min clearance > 0.25
    reset()
    s = get("/api/state")
    pois = s.get("pois") or []
    far = max(pois, key=lambda p: math.hypot(p["x"] - s["agv"]["x"], p["y"] - s["agv"]["y"])) if pois else {
        "x": 10.57,
        "y": -12.0,
    }
    pq = post("/api/nav/plan_quality", {"x": far["x"], "y": far["y"], "compare": False})
    pl = pq.get("planning") or {}
    results["G5"] = {
        "ok": bool(pq.get("success")) and float(pl.get("min_clearance") or 0) >= 0.30,
        "ratio": pl.get("path_ratio"),
        "turns": pl.get("final_turn_count"),
        "min_c": pl.get("min_clearance"),
        "direct": pl.get("direct_path_safe"),
    }

    # G8 repeatability
    reset()
    s = get("/api/state")
    ratios = []
    for _ in range(3):
        pq = post("/api/nav/plan_quality", {"x": s["agv"]["x"] + 5.0, "y": s["agv"]["y"] - 5.0, "compare": False})
        ratios.append(round(float((pq.get("planning") or {}).get("path_ratio") or 0), 4))
    results["G8"] = {"ok": len(set(ratios)) == 1, "ratios": ratios}

    # G7 PP-only vs MPPI on short south
    for mode, key in (("mppi", "G7_mppi"), ("pp_only", "G7_pp")):
        reset()
        s = get("/api/state")
        post("/api/nav/control_mode", {"mode": mode})
        p = post("/api/nav/plan", {"x": s["agv"]["x"], "y": s["agv"]["y"] - 3.5})
        assert p.get("ret_code") == 0 or p.get("success") is not False, p
        post("/api/nav/confirm")
        rows = sample(16, 0.35)
        osc = w_sign_changes(rows)
        results[key] = {
            "ok": any(r["vx"] > 0.03 for r in rows) and all(r["phase"] != "reverse_escape" for r in rows[:12]),
            "osc_w": osc,
            "path_s0": rows[0]["path_s"] if rows else None,
            "path_s1": rows[-1]["path_s"] if rows else None,
            "stop": rows[-1]["stop"] if rows else None,
        }

    # G6: dynamic obstacle should not force global every frame — just plan once then confirm brief
    reset()
    s = get("/api/state")
    post("/api/obstacles/add", {"x": s["agv"]["x"] + 1.5, "y": s["agv"]["y"], "r": 0.4})
    time.sleep(0.2)
    p = post("/api/nav/plan", {"x": s["agv"]["x"] + 3.0, "y": s["agv"]["y"]})
    st = get("/api/state")
    gcount1 = st["nav"].get("global_replan_count")
    post("/api/nav/confirm")
    time.sleep(1.5)
    st2 = get("/api/state")
    gcount2 = st2["nav"].get("global_replan_count")
    results["G6"] = {
        "ok": (gcount2 or 0) - (gcount1 or 0) <= 1,
        "g1": gcount1,
        "g2": gcount2,
        "plan_ok": bool(p.get("path") or p.get("ret_code") == 0),
    }

    print("\n=== SUMMARY ===")
    all_ok = True
    for k, v in results.items():
        ok = bool(v.get("ok"))
        all_ok = all_ok and ok
        print(f"{k}: {'PASS' if ok else 'FAIL'} {v}")
    print("OVERALL", "PASS" if all_ok else "FAIL")
    out = os_path = __file__.replace(".py", "_out.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
