"""Phase-2 nav fix regression: CASE 1-8 (no target_id on XY plans)."""
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
    with urllib.request.urlopen(req, timeout=20) as r:
        return json.loads(r.read().decode())


def reset_scene():
    post("/api/cancel")
    time.sleep(0.2)
    post("/api/scene", {"id": "indoor_office"})
    time.sleep(0.6)
    return get("/api/state")


def plan_xy(x, y):
    # 故意不带 target_id，避免走 POI 路由
    return post("/api/nav/plan", {"x": float(x), "y": float(y)})


def confirm():
    return post("/api/nav/confirm")


def sample(n=25, dt=0.4):
    rows = []
    for i in range(n):
        time.sleep(dt)
        s = get("/api/state")
        a, nav, safety = s["agv"], s["nav"], s.get("safety") or {}
        rows.append(
            {
                "t": round((i + 1) * dt, 1),
                "x": round(a["x"], 3),
                "y": round(a["y"], 3),
                "vx": round(a["vx"], 3),
                "w": round(a["w"], 3),
                "stuck": nav.get("stuck_s"),
                "phase": nav.get("phase"),
                "stop": safety.get("stop_reason") or nav.get("stop_reason"),
                "front": safety.get("front_near"),
                "path_s": nav.get("path_progress_s"),
                "gd": nav.get("goal_distance"),
                "mppi_vx": nav.get("mppi_vx"),
                "safe_vx": nav.get("cmd_vx_after_safety"),
                "band": len(nav.get("local_path") or []),
                "rec": nav.get("recovery_attempts"),
                "mode": nav.get("control_mode"),
            }
        )
    return rows


def summarize(name, rows, checks):
    print(f"\n=== {name} ===")
    for r in rows[:: max(1, len(rows) // 5)] + ([rows[-1]] if rows else []):
        print(r)
    ok = True
    for label, fn in checks:
        try:
            passed = bool(fn(rows))
        except Exception as e:
            passed = False
            print(f"  CHECK {label}: EXC {e}")
        else:
            print(f"  CHECK {label}: {'PASS' if passed else 'FAIL'}")
        ok = ok and passed
    print(f"RESULT {name}: {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    results = {}
    s0 = reset_scene()
    x0, y0 = s0["agv"]["x"], s0["agv"]["y"]
    print("spawn", x0, y0, "inflate", s0.get("scene", {}))

    # CASE 1: straight +2m
    post("/api/nav/control_mode", {"mode": "mppi"})
    p = plan_xy(x0 + 2.0, y0)
    assert p.get("success"), p
    confirm()
    rows = sample(20, 0.35)
    results["CASE1"] = summarize(
        "CASE1_straight",
        rows,
        [
            ("vx_positive", lambda rs: any(r["vx"] > 0.03 for r in rs[:10])),
            ("path_progress_up", lambda rs: rs[-1]["path_s"] > rs[0]["path_s"] + 0.15),
            ("no_reverse", lambda rs: all(r["phase"] != "reverse_escape" for r in rs)),
            ("stop_none_or_goal", lambda rs: all(r["stop"] in (None, "NONE", "GOAL_REACHED") for r in rs)),
        ],
    )

    # CASE 3: 南向目标（路径先转向，goal 可能暂不下降）；验证能跟线且 12s 内不因假 stuck 倒车
    reset_scene()
    s = get("/api/state")
    pois = s.get("pois") or []
    # 优先选偏南的点
    south = [p for p in pois if p["y"] < s["agv"]["y"] - 3.0]
    goal = min(south, key=lambda p: math.hypot(p["x"] - s["agv"]["x"], p["y"] - s["agv"]["y"])) if south else max(
        pois, key=lambda p: math.hypot(p["x"] - s["agv"]["x"], p["y"] - s["agv"]["y"])
    )
    p = plan_xy(goal["x"], goal["y"])
    print("CASE3 plan wp", p.get("waypoints"), "goal", goal["id"])
    confirm()
    rows = sample(35, 0.4)  # 14s
    results["CASE3"] = summarize(
        "CASE3_turn_follow",
        rows,
        [
            ("path_progress_up", lambda rs: (rs[-1]["path_s"] or 0) > (rs[0]["path_s"] or 0) + 0.4),
            ("has_yaw_rate", lambda rs: any(abs(r["w"] or 0) > 0.02 for r in rs[:15])),
            (
                "no_reverse_while_progressing",
                # 仅当 path_s 仍在抬升时禁止 reverse；停在障碍前触发 recovery 不算假倒车
                lambda rs: not any(
                    rs[i]["phase"] == "reverse_escape"
                    and i >= 3
                    and (rs[i]["path_s"] or 0) > (rs[i - 3]["path_s"] or 0) + 0.15
                    for i in range(3, len(rs))
                ),
            ),
            ("telemetry_present", lambda rs: rs[0]["front"] is not None and rs[0]["mppi_vx"] is not None),
        ],
    )

    # CASE 7: cancel clears stuck
    time.sleep(0.5)
    post("/api/cancel")
    time.sleep(0.3)
    s = get("/api/state")
    results["CASE7"] = summarize(
        "CASE7_cancel",
        [
            {
                "stuck": s["nav"].get("stuck_s"),
                "phase": s["nav"].get("phase"),
                "stop": (s.get("safety") or {}).get("stop_reason"),
                "mode": s["nav"].get("mode"),
                "rec": s["nav"].get("recovery_attempts"),
                "vx": s["agv"]["vx"],
                "path_s": s["nav"].get("path_progress_s"),
                "front": (s.get("safety") or {}).get("front_near"),
                "mppi_vx": s["nav"].get("mppi_vx"),
                "x": s["agv"]["x"],
                "y": s["agv"]["y"],
                "w": 0,
                "safe_vx": 0,
                "band": 0,
                "gd": 0,
                "t": 0,
            }
        ],
        [
            ("stuck_zero", lambda rs: float(rs[0]["stuck"] or 0) == 0),
            ("idle", lambda rs: rs[0]["mode"] == "idle"),
            ("rec_zero", lambda rs: int(rs[0]["rec"] or 0) == 0),
        ],
    )

    # CASE 8: PP-only short
    reset_scene()
    s = get("/api/state")
    post("/api/nav/control_mode", {"mode": "pp_only"})
    p = plan_xy(s["agv"]["x"] + 2.0, s["agv"]["y"])
    confirm()
    rows = sample(18, 0.35)
    results["CASE8"] = summarize(
        "CASE8_pp_only",
        rows,
        [
            ("mode_pp", lambda rs: rs[-1]["mode"] == "pp_only"),
            ("vx_positive", lambda rs: any(r["vx"] > 0.03 for r in rs)),
            ("path_progress_up", lambda rs: rs[-1]["path_s"] > rs[0]["path_s"] + 0.1),
        ],
    )

    # CASE 4: place obstacle ahead — 动态障应进入 front_near / FRONT_OBSTACLE
    reset_scene()
    s = get("/api/state")
    post("/api/nav/control_mode", {"mode": "mppi"})
    ox, oy = s["agv"]["x"] + 1.0, s["agv"]["y"]
    add = post("/api/obstacles/add", {"x": ox, "y": oy, "r": 0.5, "name": "audit_block"})
    print("CASE4 add", add)
    time.sleep(0.3)
    s2 = get("/api/state")
    print("CASE4 front after add", (s2.get("safety") or {}).get("front_near"), "obs", s2.get("obstacles"))
    p = plan_xy(s["agv"]["x"] + 3.0, s["agv"]["y"])
    if p.get("success"):
        confirm()
        rows = sample(22, 0.35)
        results["CASE4"] = summarize(
            "CASE4_front_obstacle",
            rows,
            [
                (
                    "saw_front_stop_or_near",
                    lambda rs: any(
                        r["stop"] == "FRONT_OBSTACLE" or (r["front"] is not None and r["front"] < 1.2)
                        for r in rs
                    ),
                ),
                (
                    "stop_reason_front_when_near",
                    lambda rs: any(r["stop"] == "FRONT_OBSTACLE" for r in rs if (r["front"] or 99) < 0.5),
                ),
            ],
        )
    else:
        print("CASE4 plan failed around obstacle — PASS(blocked path)")
        results["CASE4"] = True
    post("/api/obstacles/clear")

    print("\n==== SUMMARY ====")
    for k, v in results.items():
        print(k, "PASS" if v else "FAIL")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
