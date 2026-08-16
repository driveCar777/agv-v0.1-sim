#!/usr/bin/env python3
"""STEP 3F-CORRECTIVE LIVE — require real reverse execution (vx<0 + pose progress), not action alone."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

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


def state() -> Dict[str, Any]:
    return api("GET", "/api/state")


def pose_from(st: Dict[str, Any]) -> Tuple[float, float, float]:
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
    # Outdoor corridor keeps rear open for S1 reverse escape
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.6)
    except Exception:
        pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.35)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        st = state()
        x, y, yaw = pose_from(st)
        wrap = api(
            "POST",
            "/api/nav/plan",
            {"x": x + 10.0 * math.cos(yaw), "y": y + 10.0 * math.sin(yaw)},
        )
        plan = wrap.get("api") or wrap
        if plan.get("ret_code") is None or int(plan.get("ret_code") or 1) != 0:
            raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.9)


def sample(t0: float) -> Dict[str, Any]:
    d = debug()
    st = state()
    nav = st.get("nav") or {}
    a = st.get("agv") or {}
    pr = d.get("probe") or (d.get("phase4") or {}).get("probe") or {}
    rec = d.get("recovery") or (d.get("phase4") or {}).get("recovery") or {}
    traj = d.get("physical_trajectory") or (d.get("phase4") or {}).get("physical_trajectory") or {}
    man = d.get("maneuver") or {}
    vc = d.get("velocity_chain") or {}
    active = traj.get("active") if isinstance(traj, dict) else None
    if not isinstance(active, dict):
        active = nav.get("physical_trajectory") if isinstance(nav.get("physical_trajectory"), dict) else {}
    exec_rec = rec.get("execution") or rec.get("progress") or man.get("recovery_exec") or {}
    x, y, yaw = pose_from(st)
    return {
        "type": "sample",
        "t": round(time.time() - t0, 4),
        "mode": man.get("mode") or nav.get("phase"),
        "pose": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)},
        "probe": {
            "F": (pr.get("forward") or {}).get("status"),
            "L": (pr.get("left") or {}).get("status"),
            "R": (pr.get("right") or {}).get("status"),
            "B": (pr.get("backward") or {}).get("status"),
        },
        "recovery": {
            "class": rec.get("classification"),
            "action": rec.get("action"),
            "reason": rec.get("reason"),
            "allow": rec.get("allowed") if "allowed" in rec else rec.get("allow_recovery"),
            "exec_status": exec_rec.get("status"),
            "signed_progress_m": exec_rec.get("signed_progress_m"),
            "target_remaining_m": exec_rec.get("target_remaining_m")
            or (exec_rec.get("target_distance_remaining") if isinstance(exec_rec, dict) else None),
        },
        "cmd": {
            "force_vx": man.get("force_vx"),
            "force_w": man.get("force_w"),
            "mppi_vx": nav.get("mppi_vx"),
            "mppi_w": nav.get("mppi_w"),
            "cmd_before_vx": nav.get("cmd_vx_before_safety"),
            "cmd_before_w": nav.get("cmd_w_before_safety"),
            "safe_vx": nav.get("cmd_vx_after_safety"),
            "safe_w": nav.get("cmd_w_after_safety"),
            "state_vx": nav.get("state_vx") if nav.get("state_vx") is not None else a.get("vx"),
            "state_w": nav.get("state_w") if nav.get("state_w") is not None else a.get("w"),
            "vc_mppi": vc.get("mppi_vx"),
            "vc_safe": vc.get("safe_vx") or vc.get("cmd_vx_after_safety"),
        },
        "traj": {
            "source": (active or {}).get("source"),
            "status": (active or {}).get("status"),
            "half_w": (active or {}).get("half_width_m"),
            "has_edges": bool((active or {}).get("left_edge") and (active or {}).get("right_edge")),
            "len": (active or {}).get("length_m"),
        },
        "front_near": (st.get("safety") or {}).get("front_near"),
        "rear_near": (st.get("safety") or {}).get("rear_near"),
    }


def seal_front_left_right(x, y, yaw) -> None:
    """Three-side pocket; keep rearward corridor clear for Backward Probe."""
    # Front wall (ahead only)
    fx, fy = bf(x, y, yaw, 1.15, 0.0)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.42, "name": "f3_front"})
    # Side seals: forward-biased so reverse footprint stays clear
    for i, (fwd, lat, r) in enumerate([(1.0, 0.85, 0.34), (1.25, 1.05, 0.34)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_L{i}"})
    for i, (fwd, lat, r) in enumerate([(1.0, -0.85, 0.34), (1.25, -1.05, 0.34)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_R{i}"})


def seal_rear_too(x, y, yaw) -> None:
    for i, (fwd, lat, r) in enumerate([(-0.85, 0.0, 0.42), (-1.1, 0.35, 0.36), (-1.1, -0.35, 0.36)]):
        bx, by = bf(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": f"f3_rear{i}"})


def signed_back(p0, p1) -> float:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    yaw = p0[2]
    return dx * (-math.cos(yaw)) + dy * (-math.sin(yaw))


def run_scene1(out_dir: str) -> Dict[str, Any]:
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_cor_scene1_{int(t0)}.jsonl")
    setup_nav()
    injected = False
    inject_pose = None
    saw_action = False
    saw_mode_rev = False
    saw_neg_req = False
    saw_neg_safe = False
    saw_neg_state = False
    max_back = 0.0
    endless_spin = 0
    timeline: List[str] = []
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", "scene": 1, "goal": "escape_with_real_reverse"}, ensure_ascii=False) + "\n")
        end = time.time() + 20.0
        while time.time() < end:
            s = sample(t0)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            if (not injected) and s["t"] > 1.3:
                st = state()
                x, y, yaw = pose_from(st)
                seal_front_left_right(x, y, yaw)
                injected = True
                inject_pose = (x, y, yaw)
                timeline.append(f"T+{s['t']:.2f}: INJECT three-side")
                print(f"[S1] inject three-side at t={s['t']:.2f}")
            act = (s.get("recovery") or {}).get("action")
            if act == "LOCAL_REVERSE":
                if not saw_action:
                    timeline.append(f"T+{s['t']:.2f}: RECOVERY=LOCAL_REVERSE")
                saw_action = True
            if "REVERSE" in str(s.get("mode") or ""):
                if not saw_mode_rev:
                    timeline.append(f"T+{s['t']:.2f}: mode=REVERSE_ESCAPE")
                saw_mode_rev = True
            cmd = s.get("cmd") or {}
            req = cmd.get("cmd_before_vx")
            if req is None:
                req = cmd.get("mppi_vx")
            if req is None:
                req = cmd.get("force_vx")
            safe = cmd.get("safe_vx")
            stv = cmd.get("state_vx")
            if req is not None and float(req) < -0.03:
                if not saw_neg_req:
                    timeline.append(f"T+{s['t']:.2f}: requested_vx={float(req):.3f}")
                saw_neg_req = True
            if safe is not None and float(safe) < -0.03:
                if not saw_neg_safe:
                    timeline.append(f"T+{s['t']:.2f}: safe_vx={float(safe):.3f}")
                saw_neg_safe = True
            if stv is not None and float(stv) < -0.02:
                if not saw_neg_state:
                    timeline.append(f"T+{s['t']:.2f}: state_vx={float(stv):.3f}")
                saw_neg_state = True
            # spin detect during recovery
            if act == "LOCAL_REVERSE" and req is not None and abs(float(req)) < 0.02:
                w = cmd.get("cmd_before_w") or cmd.get("mppi_w") or cmd.get("state_w") or 0
                if abs(float(w)) > 0.15:
                    endless_spin += 1
            if inject_pose is not None:
                p = s.get("pose") or {}
                mb = signed_back(inject_pose, (p.get("x", 0), p.get("y", 0), inject_pose[2]))
                if mb > max_back:
                    max_back = mb
                    if max_back >= 0.12 and f"pose moved" not in "".join(timeline):
                        timeline.append(f"T+{s['t']:.2f}: pose moved -{max_back:.2f}m (signed back)")
            time.sleep(0.1)
        escaped = bool(max_back >= 0.12 and saw_neg_state)
        summary = {
            "type": "summary",
            "scene": 1,
            "injected": injected,
            "recovery_local_reverse": saw_action,
            "mode_reverse": saw_mode_rev,
            "requested_vx_neg": saw_neg_req,
            "safe_vx_neg": saw_neg_safe,
            "state_vx_neg": saw_neg_state,
            "max_signed_back_m": round(max_back, 3),
            "endless_spin_ticks": endless_spin,
            "timeline": timeline,
            "pass": bool(
                injected
                and saw_action
                and saw_mode_rev
                and saw_neg_req
                and saw_neg_safe
                and saw_neg_state
                and max_back >= 0.12
                and endless_spin < 25
            ),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_S1", summary)
    for line in timeline:
        print(" ", line)
    return summary


def run_scene2(out_dir: str) -> Dict[str, Any]:
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_cor_scene2_{int(t0)}.jsonl")
    setup_nav()
    injected = False
    saw_stopish = False
    saw_rev = False
    spin_ticks = 0
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", "scene": 2, "goal": "no_escape_stop"}, ensure_ascii=False) + "\n")
        end = time.time() + 14.0
        while time.time() < end:
            s = sample(t0)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            if (not injected) and s["t"] > 1.3:
                st = state()
                x, y, yaw = pose_from(st)
                seal_front_left_right(x, y, yaw)
                seal_rear_too(x, y, yaw)
                injected = True
                print(f"[S2] inject cage at t={s['t']:.2f}")
            act = (s.get("recovery") or {}).get("action")
            mode = str(s.get("mode") or "")
            if act in ("SAFE_STOP", "REPLAN") or mode in ("SAFE_STOP", "REPLAN"):
                saw_stopish = True
            if "REVERSE" in mode and act == "LOCAL_REVERSE":
                saw_rev = True
            cmd = s.get("cmd") or {}
            svx = float(cmd.get("state_vx") or 0)
            sw = float(cmd.get("state_w") or cmd.get("mppi_w") or 0)
            if abs(svx) < 0.03 and abs(sw) > 0.2:
                spin_ticks += 1
            time.sleep(0.1)
        summary = {
            "type": "summary",
            "scene": 2,
            "injected": injected,
            "stop_or_replan": saw_stopish,
            "mode_reverse": saw_rev,
            "spin_ticks": spin_ticks,
            "pass": bool(injected and saw_stopish and spin_ticks < 40),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_S2", summary)
    return summary


def run_scene6(out_dir: str) -> Dict[str, Any]:
    t0 = time.time()
    path = os.path.join(out_dir, f"step3f_cor_scene6_{int(t0)}.jsonl")
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
                st = state()
                x, y, yaw = pose_from(st)
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
