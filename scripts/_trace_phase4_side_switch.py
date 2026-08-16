#!/usr/bin/env python3
"""STEP 2 read-only side-switch tracer for http://127.0.0.1:19999.

Does NOT modify navigation logic. Only polls existing APIs and writes JSONL.
"""

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


def api(method: str, path: str, body=None, timeout: float = 8.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def dbg() -> Dict[str, Any]:
    wrap = api("GET", "/api/nav/debug")
    return wrap.get("debug") or wrap


def state() -> Dict[str, Any]:
    return api("GET", "/api/state")


def pose_from_state(st: Dict[str, Any]):
    agv = st.get("agv") or {}
    return float(agv.get("x") or 0), float(agv.get("y") or 0), float(agv.get("angle") or 0)


def plan_confirm(gx: float, gy: float):
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


def row_for(action: str, rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    for r in rows or []:
        if (r.get("action") or r.get("type")) == action:
            return r
    return {}


def sample_once(t0: float) -> Dict[str, Any]:
    now = time.time()
    d = dbg()
    st = state()
    x, y, yaw = pose_from_state(st)
    pol = d.get("nav_policy") or {}
    man = d.get("maneuver") or {}
    lm = d.get("local_maneuver") or {}
    status = d.get("status") or {}
    safety = d.get("safety") or {}
    motion = d.get("motion") or d.get("velocity") or {}
    snap = lm.get("snapshot") or {}
    rows = lm.get("rows") or []
    L = row_for("LEFT", rows) or (lm.get("left") or {})
    R = row_for("RIGHT", rows) or (lm.get("right") or {})
    F = row_for("FORWARD", rows) or (lm.get("forward") or {})
    dec = pol.get("decision") or {}
    corr = pol.get("corridor") or dec.get("corridor") or {}

    # Extract costs — fields vary by telemetry version
    def cost_of(c: Dict[str, Any]):
        return c.get("cost") if c.get("cost") is not None else c.get("total_cost")

    mppi_vx = None
    mppi_w = None
    cmd_vx = None
    safe_vx = None
    safe_w = None
    state_vx = None
    state_w = None
    vc = d.get("velocity_chain") or {}
    ctrl = d.get("controller") or {}
    saf = d.get("safety") or {}
    mot = d.get("motion") or {}
    stt = d.get("status") or {}
    mppi_vx = vc.get("mppi_vx", ctrl.get("final_vx", mot.get("vx")))
    safe_vx = vc.get("safe_vx", (saf.get("safe_cmd") or {}).get("vx"))
    cmd_vx = vc.get("cmd_vx", mppi_vx)
    state_vx = vc.get("state_vx", stt.get("vx", mot.get("vx")))
    mppi_w = ctrl.get("mppi_w", mot.get("w"))
    safe_w = (saf.get("safe_cmd") or {}).get("w", ctrl.get("final_w"))
    state_w = stt.get("w", mot.get("w"))
    # latest telemetry sample fallback
    tel = d.get("telemetry") or []
    if tel:
        last = tel[-1] if isinstance(tel, list) else {}
        mppi_vx = mppi_vx if mppi_vx is not None else last.get("mppi_vx")
        mppi_w = mppi_w if mppi_w is not None else last.get("mppi_w")
        safe_vx = safe_vx if safe_vx is not None else last.get("safe_vx")
        safe_w = safe_w if safe_w is not None else last.get("safe_w")
        state_vx = state_vx if state_vx is not None else last.get("state_vx")
        state_w = state_w if state_w is not None else last.get("state_w")
        cmd_vx = cmd_vx if cmd_vx is not None else last.get("cmd_vx")

    stop_reason = (
        stt.get("stop_reason")
        or (saf.get("block_reason") if isinstance(saf, dict) else None)
        or d.get("stop_reason")
        or "NOT_AVAILABLE"
    )
    phase = stt.get("phase") or man.get("legacy_phase") or mot.get("phase") or "NOT_AVAILABLE"
    front = None
    if isinstance(saf, dict):
        front = saf.get("front_near")
    if front is None and tel:
        front = tel[-1].get("front_near")
    if front is None:
        front = snap.get("front_near")

    Obstacles = []
    try:
        for o in (st.get("obstacles") or st.get("dyn_obstacles") or []):
            if isinstance(o, dict):
                Obstacles.append(
                    {
                        "x": o.get("x"),
                        "y": o.get("y"),
                        "r": o.get("r") or o.get("radius"),
                        "name": o.get("name") or o.get("id"),
                    }
                )
    except Exception:
        pass

    return {
        "t": round(now - t0, 4),
        "wall_ts": now,
        "pose": {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)},
        "policy_state": pol.get("state"),
        "policy_behavior": pol.get("behavior"),
        "policy_reason": pol.get("reason"),
        "avoid_side": pol.get("avoid_side"),
        "allow_side_compare": dec.get("allow_side_compare") if dec else pol.get("allow_side_compare"),
        "require_capture_hard": dec.get("require_capture_hard"),
        "path_follow_weight": pol.get("path_follow_weight") or dec.get("path_follow_weight"),
        "corridor": {
            "half_width": corr.get("half_width"),
            "lateral_error": corr.get("lateral_error") or pol.get("local_deviation"),
            "exceeded": corr.get("exceeded"),
            "mode": corr.get("mode"),
            "time_away_s": corr.get("time_away_s"),
        },
        "maneuver_mode": man.get("mode"),
        "maneuver_reason": man.get("reason"),
        "decision_label": man.get("decision") or lm.get("decision"),
        "selector_selected": lm.get("decision") or snap.get("selected"),
        "selector_reason": lm.get("reason") or snap.get("reason"),
        "left_cost": cost_of(L) if L else snap.get("left_cost"),
        "right_cost": cost_of(R) if R else snap.get("right_cost"),
        "forward_cost": cost_of(F) if F else snap.get("forward_cost"),
        "left_feasible": L.get("feasible") if L else snap.get("left_feasible"),
        "right_feasible": R.get("feasible") if R else snap.get("right_feasible"),
        "left_clr": L.get("clr") or L.get("min_clearance"),
        "right_clr": R.get("clr") or R.get("min_clearance"),
        "left_capture": L.get("capture") or L.get("path_capture_distance"),
        "right_capture": R.get("capture") or R.get("path_capture_distance"),
        "left_progress": L.get("progress") or L.get("path_progress_gain"),
        "right_progress": R.get("progress") or R.get("path_progress_gain"),
        "left_quality": L.get("quality") or L.get("route_quality"),
        "right_quality": R.get("quality") or R.get("route_quality"),
        "left_free": snap.get("left_free") or man.get("left_free"),
        "right_free": snap.get("right_free") or man.get("right_free"),
        "front_near": front,
        "phase": phase,
        "stop_reason": stop_reason,
        "mppi_vx": mppi_vx,
        "mppi_w": mppi_w,
        "cmd_vx": cmd_vx,
        "safe_vx": safe_vx,
        "safe_w": safe_w,
        "state_vx": state_vx,
        "state_w": state_w,
        "stuck_s": status.get("stuck_s") or d.get("stuck_s"),
        "nav_mode": status.get("nav_mode") or d.get("nav_mode"),
        "side_attempts": lm.get("side_attempts") or man.get("side_attempts"),
        "obstacle_passed": lm.get("obstacle_passed") or man.get("obstacle_passed"),
        "local_events": (lm.get("events") or man.get("local_events") or [])[-4:],
        "policy_events": (pol.get("events") or [])[-4:],
        "obstacles": Obstacles[:12],
        # need_side_compare is not exposed — mark explicitly
        "need_side_compare": "NOT_AVAILABLE_IN_TELEMETRY",
        "avoid_hold_active": "NOT_AVAILABLE_EXPLICIT",  # only avoid_side present
        "w_sign": (
            0
            if mppi_w is None or abs(float(mppi_w)) < 0.05
            else (1 if float(mppi_w) > 0 else -1)
        ),
        "safe_w_sign": (
            0
            if safe_w is None or abs(float(safe_w or 0)) < 0.05
            else (1 if float(safe_w) > 0 else -1)
        ),
        # Phase4 STEP3B observe block (pass-through; no decisions)
        "phase4": d.get("phase4"),
        "commitment": d.get("commitment") or (d.get("phase4") or {}).get("commitment"),
        "probe": d.get("probe") or (d.get("phase4") or {}).get("probe"),
        "side_switch": d.get("side_switch") or (d.get("phase4") or {}).get("side_switch"),
        "sides": d.get("sides") or (d.get("phase4") or {}).get("sides"),
        "ownership": d.get("ownership") or (d.get("phase4") or {}).get("ownership"),
        "phase4_events": d.get("phase4_events") or (d.get("phase4") or {}).get("events") or [],
    }


def side_of(sample: Dict[str, Any]) -> str:
    mode = (sample.get("maneuver_mode") or "").upper()
    sel = (sample.get("selector_selected") or sample.get("decision_label") or "").upper()
    if "LEFT" in mode or sel == "LEFT":
        return "LEFT"
    if "RIGHT" in mode or sel == "RIGHT":
        return "RIGHT"
    if mode:
        return mode
    return sel or "NONE"


def run_scenario(name: str, out_path: str, duration: float, setup_fn) -> Dict[str, Any]:
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    t0 = time.time()
    meta = {"scenario": name, "t0": t0, "base": BASE}
    setup_info = setup_fn()
    meta["setup"] = setup_info

    samples: List[Dict[str, Any]] = []
    switches: List[Dict[str, Any]] = []
    prev_side = None
    end = time.time() + duration
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", **meta}, ensure_ascii=False) + "\n")
        while time.time() < end:
            try:
                s = sample_once(t0)
            except Exception as e:
                s = {"t": round(time.time() - t0, 4), "error": str(e)}
            samples.append(s)
            f.write(json.dumps({"type": "sample", **s}, ensure_ascii=False) + "\n")
            f.flush()
            side = side_of(s) if "error" not in s else prev_side
            if prev_side in ("LEFT", "RIGHT") and side in ("LEFT", "RIGHT") and side != prev_side:
                ev = {
                    "type": "side_switch",
                    "t": s.get("t"),
                    "from": prev_side,
                    "to": side,
                    "sample": s,
                }
                switches.append(ev)
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
                f.flush()
                print(f"[{name}] SWITCH {prev_side}->{side} at t={s.get('t')} reason={s.get('selector_reason')} L={s.get('left_cost')} R={s.get('right_cost')}")
            if side in ("LEFT", "RIGHT", "LOCAL_LEFT", "LOCAL_RIGHT") or (isinstance(side, str) and "LEFT" in side or "RIGHT" in side):
                if "LEFT" in str(side):
                    prev_side = "LEFT"
                elif "RIGHT" in str(side):
                    prev_side = "RIGHT"
                else:
                    prev_side = side
            elif side and side not in ("NONE",):
                # keep previous LEFT/RIGHT sticky until new side decision
                pass
            time.sleep(0.05)  # 20Hz poll — observation only

    summary = {
        "scenario": name,
        "samples": len(samples),
        "switches": [{"t": s["t"], "from": s["from"], "to": s["to"]} for s in switches],
        "modes": sorted({(s.get("maneuver_mode") or "") for s in samples if s.get("maneuver_mode")}),
        "policies": sorted({(s.get("policy_state") or "") for s in samples if s.get("policy_state")}),
        "stop_reasons": sorted({(s.get("stop_reason") or "") for s in samples if s.get("stop_reason")}),
    }
    with open(out_path, "a", encoding="utf-8") as f:
        f.write(json.dumps({"type": "summary", **summary}, ensure_ascii=False) + "\n")
    print(f"[{name}] summary", json.dumps(summary, ensure_ascii=False))
    return summary


def setup_live_a_left_wide():
    """Classic: pillar ahead + right wall → expect LEFT first. Pillar farther so motion can start."""
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.6)
    except Exception as e:
        print("WARN scene", e)
    api("POST", "/api/obstacles/clear", {})
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    time.sleep(0.35)
    st = state()
    x, y, yaw = pose_from_state(st)
    # farther pillar to avoid immediate FRONT_OBSTACLE lock at ~0.18m
    fx = x + 1.45 * math.cos(yaw)
    fy = y + 1.45 * math.sin(yaw)
    rx = x + 1.10 * math.cos(yaw) + 1.05 * math.cos(yaw - math.pi / 2)
    ry = y + 1.10 * math.sin(yaw) + 1.05 * math.sin(yaw - math.pi / 2)
    api("POST", "/api/obstacles/add", {"x": fx, "y": fy, "r": 0.38, "name": "p4_pillar"})
    api("POST", "/api/obstacles/add", {"x": rx, "y": ry, "r": 0.50, "name": "p4_right_wall"})
    gx = x + 4.0 * math.cos(yaw)
    gy = y + 4.0 * math.sin(yaw)
    plan_confirm(gx, gy)
    return {
        "pose": {"x": x, "y": y, "yaw": yaw},
        "pillar": {"x": fx, "y": fy, "r": 0.38},
        "right_wall": {"x": rx, "y": ry, "r": 0.50},
        "goal": {"x": gx, "y": gy},
        "note": "LIVE farther pillar for motion; left open, right blocked",
    }


def setup_mid_avoid_extend():
    """Same as LIVE-A but longer watch to catch mid-maneuver switch; optionally nudge right wall farther mid-run is NOT done here (no mid mutation in setup)."""
    return setup_live_a_left_wide()


def run_scenario_with_optional_mutate(name: str, out_path: str, duration: float, setup_fn, mutate_at: Optional[float] = None):
    """If mutate_at set: after LOCAL_LEFT seen, at mutate_at clear right wall to make RIGHT cheaper (LIVE scene mutation)."""
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    t0 = time.time()
    meta = {"scenario": name, "t0": t0, "base": BASE, "mutate_at": mutate_at}
    setup_info = setup_fn()
    meta["setup"] = setup_info
    samples: List[Dict[str, Any]] = []
    switches: List[Dict[str, Any]] = []
    prev_side = None
    saw_left = False
    mutated = False
    end = time.time() + duration
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", **meta}, ensure_ascii=False) + "\n")
        while time.time() < end:
            try:
                s = sample_once(t0)
            except Exception as e:
                s = {"t": round(time.time() - t0, 4), "error": str(e)}
            samples.append(s)
            f.write(json.dumps({"type": "sample", **s}, ensure_ascii=False) + "\n")
            side = side_of(s) if "error" not in s else prev_side
            if side == "LEFT":
                saw_left = True
            if (
                mutate_at is not None
                and (not mutated)
                and saw_left
                and s.get("t", 0) >= mutate_at
            ):
                # Scene mutation only: remove right wall obstacle names — clear all and re-add pillar only
                try:
                    setup = setup_info or {}
                    pillar = setup.get("pillar") or {}
                    api("POST", "/api/obstacles/clear", {})
                    if pillar:
                        api(
                            "POST",
                            "/api/obstacles/add",
                            {"x": pillar["x"], "y": pillar["y"], "r": pillar.get("r", 0.38), "name": "p4_pillar_only"},
                        )
                    mutated = True
                    f.write(json.dumps({"type": "scene_mutation", "t": s.get("t"), "action": "clear_right_wall_keep_pillar"}, ensure_ascii=False) + "\n")
                    print(f"[{name}] SCENE_MUTATION clear right wall at t={s.get('t')}")
                except Exception as e:
                    print("mutate fail", e)
            if prev_side in ("LEFT", "RIGHT") and side in ("LEFT", "RIGHT") and side != prev_side:
                ev = {"type": "side_switch", "t": s.get("t"), "from": prev_side, "to": side, "sample": s}
                switches.append(ev)
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
                print(f"[{name}] SWITCH {prev_side}->{side} at t={s.get('t')} reason={s.get('selector_reason')} L={s.get('left_cost')} R={s.get('right_cost')}")
            if "LEFT" in str(side):
                prev_side = "LEFT"
            elif "RIGHT" in str(side):
                prev_side = "RIGHT"
            time.sleep(0.05)
    summary = {
        "scenario": name,
        "samples": len(samples),
        "switches": [{"t": s["t"], "from": s["from"], "to": s["to"]} for s in switches],
        "modes": sorted({(s.get("maneuver_mode") or "") for s in samples if s.get("maneuver_mode")}),
        "policies": sorted({(s.get("policy_state") or "") for s in samples if s.get("policy_state")}),
        "stop_reasons": sorted({(s.get("stop_reason") or "") for s in samples if s.get("stop_reason")}),
        "mutated": mutated,
    }
    with open(out_path, "a", encoding="utf-8") as f:
        f.write(json.dumps({"type": "summary", **summary}, ensure_ascii=False) + "\n")
    print(f"[{name}] summary", json.dumps(summary, ensure_ascii=False))
    return summary


def _body_frame(x: float, y: float, yaw: float, forward: float, left: float):
    """Robot body frame: +forward along yaw, +left = yaw+pi/2."""
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = -s, c
    return x + forward * c + left * lx, y + forward * s + left * ly


def setup_clear_then_nav():
    """Correct method: clear world → plan/confirm FIRST → no obstacles yet."""
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
    time.sleep(0.35)
    st = state()
    x, y, yaw = pose_from_state(st)
    gx = x + 6.5 * math.cos(yaw)
    gy = y + 6.5 * math.sin(yaw)
    plan_confirm(gx, gy)
    return {
        "pose0": {"x": x, "y": y, "yaw": yaw},
        "goal": {"x": gx, "y": gy},
        "note": "CLEAR_NAV_FIRST then mid-inject obstacles (user method)",
    }


def inject_stage1_force_left(x: float, y: float, yaw: float) -> Dict[str, Any]:
    """Sudden front block + heavy RIGHT seal → prefer LOCAL_LEFT (harder than pre-place)."""
    px, py = _body_frame(x, y, yaw, 1.15, -0.12)
    r1x, r1y = _body_frame(x, y, yaw, 1.05, -0.95)
    r2x, r2y = _body_frame(x, y, yaw, 1.45, -1.15)
    r3x, r3y = _body_frame(x, y, yaw, 0.75, -0.85)
    api("POST", "/api/obstacles/add", {"x": px, "y": py, "r": 0.42, "name": "p4_mid_pillar"})
    api("POST", "/api/obstacles/add", {"x": r1x, "y": r1y, "r": 0.55, "name": "p4_right_seal_a"})
    api("POST", "/api/obstacles/add", {"x": r2x, "y": r2y, "r": 0.48, "name": "p4_right_seal_b"})
    api("POST", "/api/obstacles/add", {"x": r3x, "y": r3y, "r": 0.40, "name": "p4_right_seal_c"})
    return {
        "stage": 1,
        "pillar": {"x": px, "y": py, "r": 0.42},
        "right_seals": [
            {"x": r1x, "y": r1y, "r": 0.55},
            {"x": r2x, "y": r2y, "r": 0.48},
            {"x": r3x, "y": r3y, "r": 0.40},
        ],
    }


def inject_stage2_flip_to_right(
    x: float, y: float, yaw: float, stage1: Dict[str, Any], *, soft: bool = False
) -> Dict[str, Any]:
    """After LOCAL_LEFT+hold: degrade LEFT / open RIGHT.

    soft=False (default, harsher): hard-clog LEFT corridor (often RIGHT_ONLY).
    soft=True: keep LEFT collision-feasible but raise soft cost / open RIGHT slightly
               (targets LOWER_TOTAL_COST both-feasible flip).
    """
    pillar = (stage1 or {}).get("pillar") or {}
    api("POST", "/api/obstacles/clear", {})
    if pillar:
        # soft: smaller leftward nudge; hard: larger nudge into LEFT path
        nudge = 0.05 if soft else 0.10
        ox = float(pillar["x"]) + nudge * math.cos(yaw + math.pi / 2)
        oy = float(pillar["y"]) + nudge * math.sin(yaw + math.pi / 2)
        pr = 0.40 if soft else 0.44
        api("POST", "/api/obstacles/add", {"x": ox, "y": oy, "r": pr, "name": "p4_mid_pillar_shift"})
        pillar = {"x": ox, "y": oy, "r": pr}
    left_blocks = []
    if soft:
        # Soft flank clutter — near LEFT arc but not guaranteed hard collision
        specs = [
            (1.05, 0.95, 0.28, "p4_left_soft_a"),
            (1.35, 1.15, 0.30, "p4_left_soft_b"),
            (0.80, 1.05, 0.26, "p4_left_soft_c"),
        ]
    else:
        specs = [
            (0.85, 0.55, 0.36, "p4_left_clog_a"),
            (1.15, 0.75, 0.38, "p4_left_clog_b"),
            (1.40, 0.95, 0.34, "p4_left_clog_c"),
            (0.65, 0.70, 0.30, "p4_left_clog_d"),
            (1.55, 0.55, 0.32, "p4_left_clog_e"),
        ]
    for fwd, left, r, name in specs:
        bx, by = _body_frame(x, y, yaw, fwd, left)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": name})
        left_blocks.append({"x": bx, "y": by, "r": r, "name": name})
    # Open RIGHT more in soft mode (remnant farther / smaller)
    if soft:
        rx, ry = _body_frame(x, y, yaw, 1.8, -1.55)
        rr = 0.22
    else:
        rx, ry = _body_frame(x, y, yaw, 1.6, -1.35)
        rr = 0.28
    api("POST", "/api/obstacles/add", {"x": rx, "y": ry, "r": rr, "name": "p4_right_remnant"})
    return {
        "stage": 2,
        "soft": soft,
        "pillar": pillar,
        "left_blocks": left_blocks,
        "right_remnant": {"x": rx, "y": ry, "r": rr},
    }

def run_mid_inject_adversarial(
    name: str, out_path: str, duration: float, *, soft_stage2: bool = False
) -> Dict[str, Any]:
    """LIVE: clear nav → wait motion → inject L-bias → after LEFT hold, flip geometry."""
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    t0 = time.time()
    setup_info = setup_clear_then_nav()
    meta = {
        "scenario": name,
        "t0": t0,
        "base": BASE,
        "method": "NAV_FIRST_THEN_MID_INJECT",
        "soft_stage2": soft_stage2,
        "setup": setup_info,
    }
    samples: List[Dict[str, Any]] = []
    switches: List[Dict[str, Any]] = []
    prev_side = None
    stage1_done = False
    stage2_done = False
    stage1_info: Dict[str, Any] = {}
    left_since: Optional[float] = None
    end = time.time() + duration
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "meta", **meta}, ensure_ascii=False) + "\n")
        while time.time() < end:
            try:
                s = sample_once(t0)
            except Exception as e:
                s = {"t": round(time.time() - t0, 4), "error": str(e)}
            samples.append(s)
            f.write(json.dumps({"type": "sample", **s}, ensure_ascii=False) + "\n")
            f.flush()
            if "error" in s:
                time.sleep(0.05)
                continue

            t = float(s.get("t") or 0.0)
            mode = (s.get("maneuver_mode") or "").upper()
            pol = (s.get("policy_state") or "").upper()
            side = side_of(s)
            pose = s.get("pose") or {}
            x = float(pose.get("x") or 0.0)
            y = float(pose.get("y") or 0.0)
            yaw = float(pose.get("yaw") or 0.0)
            state_vx = float(s.get("state_vx") or 0.0)
            front = float(s.get("front_near") or 99.0)

            if (not stage1_done) and t >= 1.2 and (
                mode in ("FORWARD_TRACK", "FORWARD_TURN")
                or pol in ("FOLLOW_GLOBAL", "CAUTION", "OBSTACLE_APPROACH")
                or abs(state_vx) > 0.03
                or t >= 2.5
            ):
                stage1_info = inject_stage1_force_left(x, y, yaw)
                stage1_done = True
                ev = {"type": "scene_injection", "t": t, "stage": 1, "info": stage1_info, "pose": pose}
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
                print(f"[{name}] INJECT_S1 force_LEFT at t={t:.3f} pose=({x:.2f},{y:.2f}) front={front:.2f}")

            if side == "LEFT" and left_since is None:
                left_since = t
                print(
                    f"[{name}] LOCAL_LEFT first at t={t:.3f} "
                    f"L={s.get('left_cost')} R={s.get('right_cost')} allow={s.get('allow_side_compare')}"
                )

            if (
                stage1_done
                and (not stage2_done)
                and left_since is not None
                and (t - left_since) >= 0.95
            ):
                stage2_info = inject_stage2_flip_to_right(
                    x, y, yaw, stage1_info, soft=soft_stage2
                )
                stage2_done = True
                ev = {"type": "scene_injection", "t": t, "stage": 2, "info": stage2_info, "pose": pose}
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
                print(
                    f"[{name}] INJECT_S2 flip_RIGHT soft={soft_stage2} "
                    f"at t={t:.3f} held_left={t - left_since:.3f}s"
                )

            if prev_side in ("LEFT", "RIGHT") and side in ("LEFT", "RIGHT") and side != prev_side:
                ev = {
                    "type": "side_switch",
                    "t": t,
                    "from": prev_side,
                    "to": side,
                    "sample": s,
                    "stage1_done": stage1_done,
                    "stage2_done": stage2_done,
                }
                switches.append(ev)
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
                f.flush()
                print(
                    f"[{name}] SWITCH {prev_side}->{side} at t={t:.3f} "
                    f"reason={s.get('selector_reason')} L={s.get('left_cost')} R={s.get('right_cost')} "
                    f"Lf={s.get('left_feasible')} Rf={s.get('right_feasible')} "
                    f"stop={s.get('stop_reason')} mppi_w={s.get('mppi_w')} safe_vx={s.get('safe_vx')}"
                )

            if "LEFT" in str(side):
                prev_side = "LEFT"
            elif "RIGHT" in str(side):
                prev_side = "RIGHT"
            time.sleep(0.05)

    summary = {
        "scenario": name,
        "samples": len(samples),
        "switches": [{"t": s["t"], "from": s["from"], "to": s["to"]} for s in switches],
        "modes": sorted({(s.get("maneuver_mode") or "") for s in samples if s.get("maneuver_mode")}),
        "policies": sorted({(s.get("policy_state") or "") for s in samples if s.get("policy_state")}),
        "stop_reasons": sorted({(s.get("stop_reason") or "") for s in samples if s.get("stop_reason")}),
        "stage1_done": stage1_done,
        "stage2_done": stage2_done,
        "left_since": left_since,
        "method": "NAV_FIRST_THEN_MID_INJECT",
        "soft_stage2": soft_stage2,
    }
    with open(out_path, "a", encoding="utf-8") as f:
        f.write(json.dumps({"type": "summary", **summary}, ensure_ascii=False) + "\n")
    print(f"[{name}] summary", json.dumps(summary, ensure_ascii=False))
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/_phase4_trace")
    ap.add_argument("--duration", type=float, default=14.0)
    ap.add_argument(
        "--scenario",
        default="mid_inject",
        choices=[
            "left_wide",
            "left_wide_long",
            "snapshot_only",
            "left_wide_mutate",
            "mid_inject",
            "mid_inject_soft",
        ],
    )
    args = ap.parse_args()

    st = state()
    scene = (st.get("scene") or {}).get("id") or st.get("scene")
    print("CONNECTED 19999 scene=", scene)
    snap = sample_once(time.time())
    print(
        "SNAPSHOT",
        json.dumps(
            {
                k: snap[k]
                for k in (
                    "policy_state",
                    "maneuver_mode",
                    "selector_selected",
                    "allow_side_compare",
                    "front_near",
                    "stop_reason",
                )
            },
            ensure_ascii=False,
        ),
    )
    if args.scenario == "snapshot_only":
        return 0

    out = os.path.join(args.out, f"{args.scenario}_{int(time.time())}.jsonl")
    if args.scenario == "mid_inject":
        run_mid_inject_adversarial(args.scenario, out, max(args.duration, 22.0), soft_stage2=False)
    elif args.scenario == "mid_inject_soft":
        run_mid_inject_adversarial(args.scenario, out, max(args.duration, 22.0), soft_stage2=True)
    elif args.scenario == "left_wide_mutate":
        run_scenario_with_optional_mutate(
            args.scenario, out, max(args.duration, 16.0), setup_live_a_left_wide, mutate_at=4.0
        )
    else:
        dur = args.duration if args.scenario != "left_wide_long" else max(args.duration, 18.0)
        run_scenario(args.scenario, out, dur, setup_live_a_left_wide)
    print("WROTE", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
