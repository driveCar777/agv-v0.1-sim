#!/usr/bin/env python3
"""STEP 3F LIVE scenes on http://127.0.0.1:19999 (surgical obstacles, no clear mid-scene)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from typing import Any, Dict, List, Optional

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 10.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def debug() -> Dict[str, Any]:
    wrap = api("GET", "/api/nav/debug")
    return wrap.get("debug") if isinstance(wrap.get("debug"), dict) else wrap


def pose():
    st = api("GET", "/api/state")
    a = st.get("agv") or {}
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def bf(x, y, yaw, fwd, lat):
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def setup_nav(gx: float = -20.0, gy: float = 0.0) -> None:
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            api("POST", p, {})
        except Exception:
            pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.35)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.8)


def sample(t0: float) -> Dict[str, Any]:
    d = debug()
    pr = d.get("probe") or (d.get("phase4") or {}).get("probe") or {}
    rec = d.get("recovery") or (d.get("phase4") or {}).get("recovery") or {}
    traj = d.get("physical_trajectory") or (d.get("phase4") or {}).get("physical_trajectory") or {}
    bc = d.get("breadcrumb") or (d.get("phase4") or {}).get("breadcrumb") or {}
    man = d.get("maneuver") or {}
    active = traj.get("active") if isinstance(traj, dict) else None
    return {
        "type": "sample",
        "t": round(time.time() - t0, 4),
        "mode": man.get("mode"),
        "probe": {
            "F": (pr.get("forward") or {}).get("status"),
            "L": (pr.get("left") or {}).get("status"),
            "R": (pr.get("right") or {}).get("status"),
            "B": (pr.get("backward") or {}).get("status"),
            "T": (pr.get("turn_in_place") or {}).get("status"),
        },
        "recovery": {
            "class": rec.get("classification"),
            "action": rec.get("action"),
            "reason": rec.get("reason"),
            "allow": rec.get("allowed") if "allowed" in rec else rec.get("allow_recovery"),
        },
        "traj": {
            "source": (active or {}).get("source") if isinstance(active, dict) else None,
            "status": (active or {}).get("status") if isinstance(active, dict) else None,
            "half_w": (active or {}).get("half_width_m") if isinstance(active, dict) else None,
            "len": (active or {}).get("length_m") if isinstance(active, dict) else None,
        },
        "breadcrumb_n": bc.get("count"),
    }


def seal_front_left_right(x, y, yaw) -> None:
    """Three-side box; leave rear open."""
    fx, fy = bf(x, y, yaw, 1.05, 0.0)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.45, "name": "f3_front"})
    for i, (fwd, lat, r) in enumerate([(0.9, 0.75, 0.40), (1.2, 0.95, 0.38), (0.7, 0.90, 0.36)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_L{i}"})
    for i, (fwd, lat, r) in enumerate([(0.9, -0.75, 0.40), (1.2, -0.95, 0.38), (0.7, -0.90, 0.36)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_R{i}"})


def seal_rear_too(x, y, yaw) -> None:
    for i, (fwd, lat, r) in enumerate([(-0.85, 0.0, 0.42), (-1.1, 0.35, 0.36), (-1.1, -0.35, 0.36)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_rear{i}"})


def run_scene1(out_dir: str) -> Dict[str, Any]:
    """F/L/R invalid, B valid → LOCAL_REVERSE."""
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_scene1_{int(t0)}.jsonl")
    setup_nav()
    injected = False
    saw_rev = False
    saw_action = False
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", "scene": 1, "goal": "local_reverse"}, ensure_ascii=False) + "\n")
        end = time.time() + 16.0
        while time.time() < end:
            s = sample(t0)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            if (not injected) and s["t"] > 1.3:
                x, y, yaw = pose()
                seal_front_left_right(x, y, yaw)
                injected = True
                f.write(json.dumps({"type": "inject", "t": s["t"], "pose": {"x": x, "y": y, "yaw": yaw}}, ensure_ascii=False) + "\n")
                print(f"[S1] inject three-side at t={s['t']:.2f}")
            if (s.get("recovery") or {}).get("action") == "LOCAL_REVERSE":
                saw_action = True
            if "REVERSE" in str(s.get("mode") or ""):
                saw_rev = True
            time.sleep(0.1)
        summary = {
            "type": "summary",
            "scene": 1,
            "injected": injected,
            "recovery_local_reverse": saw_action,
            "mode_reverse": saw_rev,
            "pass": bool(injected and saw_action),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_S1", summary)
    return summary


def run_scene2(out_dir: str) -> Dict[str, Any]:
    """F/L/R/B invalid → no escape / SAFE_STOP or REPLAN."""
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_scene2_{int(t0)}.jsonl")
    setup_nav()
    injected = False
    saw_stopish = False
    saw_rev = False
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", "scene": 2, "goal": "no_escape"}, ensure_ascii=False) + "\n")
        end = time.time() + 14.0
        while time.time() < end:
            s = sample(t0)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            if (not injected) and s["t"] > 1.3:
                x, y, yaw = pose()
                seal_front_left_right(x, y, yaw)
                seal_rear_too(x, y, yaw)
                injected = True
                print(f"[S2] inject cage at t={s['t']:.2f}")
            act = (s.get("recovery") or {}).get("action")
            if act in ("SAFE_STOP", "REPLAN") or "STOP" in str(s.get("mode") or ""):
                saw_stopish = True
            if "REVERSE" in str(s.get("mode") or ""):
                saw_rev = True
            time.sleep(0.1)
        summary = {
            "type": "summary",
            "scene": 2,
            "injected": injected,
            "stop_or_replan": saw_stopish,
            "mode_reverse": saw_rev,
            "pass": bool(injected and saw_stopish and not saw_rev),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_S2", summary)
    return summary


def run_scene6(out_dir: str) -> Dict[str, Any]:
    """Forward blocked but RIGHT open → must NOT reverse while R Probe VALID."""
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_scene6_{int(t0)}.jsonl")
    setup_nav()
    injected = False
    illegal_rev = 0
    saw_r_valid = False
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", "scene": 6, "goal": "no_blind_reverse"}, ensure_ascii=False) + "\n")
        end = time.time() + 14.0
        while time.time() < end:
            s = sample(t0)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            if (not injected) and s["t"] > 1.3:
                x, y, yaw = pose()
                fx, fy = bf(x, y, yaw, 1.1, 0.0)
                api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "f6_front"})
                for i, (fwd, lat, r) in enumerate([(0.9, 0.7, 0.38), (1.2, 0.9, 0.36)]):
                    bx, by = bf(x, y, yaw, fwd, lat)
                    api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f6_L{i}"})
                injected = True
                print(f"[S6] front+left seal at t={s['t']:.2f}")
            pr = s.get("probe") or {}
            if pr.get("R") == "VALID":
                saw_r_valid = True
            # Illegal: reverse while a side Probe is still VALID
            if (s.get("recovery") or {}).get("action") == "LOCAL_REVERSE" and (
                pr.get("R") == "VALID" or pr.get("L") == "VALID"
            ):
                illegal_rev += 1
            time.sleep(0.1)
        summary = {
            "type": "summary",
            "scene": 6,
            "injected": injected,
            "illegal_reverse_while_side_valid": illegal_rev,
            "right_valid_seen": saw_r_valid,
            "pass": bool(injected and illegal_rev == 0),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_S6", summary)
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=["1", "2", "6", "all"], default="all")
    ap.add_argument("--out", default=r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    results = []
    if args.scene in ("1", "all"):
        results.append(run_scene1(args.out))
    if args.scene in ("2", "all"):
        results.append(run_scene2(args.out))
    if args.scene in ("6", "all"):
        results.append(run_scene6(args.out))
    ok = all(r.get("pass") for r in results)
    print("OVERALL", "PASS" if ok else "FAIL", results)
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
