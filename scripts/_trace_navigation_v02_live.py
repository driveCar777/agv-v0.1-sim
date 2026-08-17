#!/usr/bin/env python3
"""V0.2 LIVE trace — deterministic M3.1 scenarios (JSONL).

Scenes LIVE-00 … LIVE-06 via nav_scenario_injector (LiDAR-visible obstacles).

Requires web sim at --base (default http://127.0.0.1:19999).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BRIDGE = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
if BRIDGE not in sys.path:
    sys.path.insert(0, BRIDGE)

from agv_bridge.nav_live_client import NavLiveClient  # noqa: E402
from agv_bridge.nav_scenario_injector import ALL_SCENES, SCENARIOS, apply_scenario  # noqa: E402
from agv_bridge.sim_world import SimWorld  # noqa: E402

_TRACK_KEYS = (
    "nav_state",
    "behavior_state",
    "avoidance_phase",
    "committed_side",
    "planner_state",
    "planner_failure_reason",
    "recovery_state",
    "safe_vx_reason",
    "maneuver_mode",
    "stop_reason",
    "nav_ui_severity",
)

_SCENE_FILE_TAG = {
    "LIVE-00": "LIVE-00-baseline",
    "LIVE-01": "LIVE-01-static-left",
    "LIVE-02": "LIVE-02-static-right",
    "LIVE-03": "LIVE-03-both-blocked",
    "LIVE-04": "LIVE-04-dynamic-cross",
    "LIVE-05": "LIVE-05-dynamic-away",
    "LIVE-06": "LIVE-06-field-p0d1",
}


def _dig(d: dict, *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


def _sample_v02(client: NavLiveClient, seq: int, scene: str, setup_meta: dict) -> dict:
    st = client.get("/api/state")
    dbg_raw = {}
    op = {}
    diag = {}
    safety = {}
    recovery = {}
    try:
        dbg_raw = client.get("/api/nav/debug")
    except Exception:
        pass
    dbg = dbg_raw.get("debug") if isinstance(dbg_raw.get("debug"), dict) else dbg_raw
    try:
        op = client.get("/api/nav/obstacle-preview")
    except Exception:
        pass
    try:
        diag = client.get("/api/logs/diagnostics?window_s=5")
    except Exception:
        pass
    try:
        safety = client.get("/api/nav/safety")
    except Exception:
        safety = st.get("safety") or _dig(dbg, "safety") or {}
    try:
        recovery = client.get("/api/nav/recovery")
    except Exception:
        recovery = _dig(dbg, "execution_recovery") or _dig(dbg, "recovery") or {}

    nav = st.get("nav") or {}
    agv = st.get("agv") or {}
    status = dbg.get("status") or {}
    vc = dbg.get("velocity_chain") or {}
    lp = dbg.get("local_planner") or nav.get("local_plan") or {}
    mppi_meta = st.get("debug", {}).get("mppi_meta") if isinstance(st.get("debug"), dict) else {}
    if not mppi_meta:
        mppi_meta = lp if isinstance(lp, dict) else {}
    nav_pol = dbg.get("nav_policy") or {}

    ec = nav_pol.get("execution_corridor") or _dig(dbg, "nav_policy", "execution_corridor") or nav.get("execution_corridor") or {}
    if not ec:
        ec = _dig(op, "obstacle_preview", "execution_corridor") or {}

    goal = nav.get("goal") or setup_meta.get("goal")
    gpath = nav.get("path") or []

    row = {
        "ts": time.time(),
        "seq": seq,
        "scene": scene,
        "schema": "navigation_v0.2_trace/2",
        "setup": setup_meta,
        "vehicle_pose": {"x": agv.get("x"), "y": agv.get("y"), "yaw": agv.get("angle")},
        "vehicle_vx": agv.get("vx"),
        "vehicle_omega": agv.get("w"),
        "state_vx": nav.get("state_vx") or status.get("vx"),
        "state_omega": nav.get("state_w") or status.get("w"),
        "nav_state": nav.get("mode") or status.get("nav_mode"),
        "behavior_state": _dig(dbg, "nav_policy", "state") or nav.get("policy_state") or status.get("policy_state"),
        "maneuver_mode": _dig(dbg, "maneuver", "mode") or nav.get("maneuver_mode") or status.get("maneuver_mode"),
        "avoidance_phase": op.get("avoidance_phase") or _dig(dbg, "nav_policy", "avoidance_phase", "phase"),
        "probe_confidence": op.get("probe_confidence"),
        "committed_side": op.get("committed_side") or _dig(dbg, "nav_policy", "committed_side"),
        "execution_corridor": ec if ec else None,
        "goal": goal,
        "global_path_len": len(gpath) if isinstance(gpath, list) else None,
        "obstacle_distance": st.get("front_near") or nav.get("safety_envelope", {}).get("front_near"),
        "front_near": nav.get("safety_envelope", {}).get("front_near") or st.get("front_near"),
        "footprint_clearance_m": nav.get("footprint_clearance_m") or safety.get("footprint_clearance_m") or status.get("footprint_clearance_m"),
        "predicted_min_clearance_m": nav.get("predicted_min_clearance_m") or safety.get("predicted_min_clearance_m") or mppi_meta.get("predicted_min_clearance_m"),
        "future_collision": _dig(op, "obstacle_preview", "future_collision") or op.get("future_collision"),
        "first_collision_m": op.get("first_collision_distance_m"),
        "candidate_count": mppi_meta.get("candidate_count") or lp.get("candidate_count") or nav.get("candidate_count"),
        "valid_candidate_count": mppi_meta.get("valid_candidate_count"),
        "collision_rejected_count": mppi_meta.get("collision_rejected_count"),
        "constraint_rejected_count": mppi_meta.get("constraint_rejected_count"),
        "clearance_rejected_count": mppi_meta.get("clearance_rejected_count"),
        "mppi_failure_reason": mppi_meta.get("failure_reason"),
        "mppi_vx": nav.get("mppi_vx") or vc.get("mppi_vx"),
        "mppi_omega": nav.get("mppi_w") or vc.get("requested_omega") or dbg.get("local_planner", {}).get("mppi_w"),
        "requested_vx": nav.get("requested_vx") or vc.get("requested_vx") or nav.get("cmd_vx_before_safety"),
        "approved_vx": nav.get("approved_vx") or vc.get("approved_vx") or nav.get("cmd_vx_after_safety"),
        "requested_omega": nav.get("requested_omega") or vc.get("requested_omega"),
        "approved_omega": nav.get("approved_omega") or vc.get("approved_omega") or nav.get("cmd_w_after_safety"),
        "safe_vx": vc.get("safe_vx") or nav.get("cmd_vx_after_safety"),
        "safe_vx_reason": nav.get("safe_vx_reason") or safety.get("safe_vx_reason") or status.get("safe_vx_reason") or vc.get("safe_vx_reason"),
        "braking_feasible": safety.get("braking_feasible"),
        "planner_state": nav.get("planner_state") or status.get("planner_state"),
        "planner_failure_reason": nav.get("planner_failure_reason") or status.get("planner_failure_reason"),
        "recovery_state": nav.get("recovery_state") or recovery.get("recovery_state") or status.get("recovery_state"),
        "recovery_attempt": nav.get("recovery_attempt") or recovery.get("recovery_attempt") or status.get("recovery_attempt"),
        "recovery_action": recovery.get("recovery_action") or recovery.get("action"),
        "recovery_result": recovery.get("recovery_result") or recovery.get("result"),
        "stop_reason": nav.get("stop_reason") or status.get("stop_reason"),
        "nav_ui_severity": nav.get("nav_ui_severity") or safety.get("nav_ui_severity") or status.get("nav_ui_severity"),
        "sensor_health": diag.get("sensor_health") or "UNKNOWN",
        "localization_health": diag.get("localization_health") or ("OK" if float(agv.get("confidence") or 0) > 0.5 else "DEGRADED"),
        "selected_candidate": lp.get("selected_candidate") or dbg.get("local_planner", {}).get("selected_candidate"),
        "trajectory_omega_sign": _omega_sign(nav.get("mppi_w") or vc.get("requested_omega")),
        "corridor_side": _corridor_side(ec),
        "heading": agv.get("angle"),
        "obstacles": st.get("obstacles"),
        "scenario_movers": st.get("scenario_movers"),
    }
    return row


def _omega_sign(w: Any) -> Optional[str]:
    if w is None:
        return None
    try:
        ww = float(w)
    except (TypeError, ValueError):
        return None
    if ww > 0.02:
        return "LEFT"
    if ww < -0.02:
        return "RIGHT"
    return "STRAIGHT"


def _corridor_side(ec: Any) -> Optional[str]:
    if not isinstance(ec, dict):
        return None
    mode = str(ec.get("mode") or "").upper()
    if mode in ("LEFT", "RIGHT"):
        return mode
    return ec.get("committed_side")


def _detect_transitions(prev: Optional[dict], cur: dict) -> List[dict]:
    if prev is None:
        return [{"event": "TRACE_START", "timestamp": cur["ts"], "scene": cur.get("scene")}]
    out = []
    for key in _TRACK_KEYS:
        old = prev.get(key)
        new = cur.get(key)
        if old != new and (old is not None or new is not None):
            out.append(
                {
                    "event": "STATE_TRANSITION",
                    "timestamp": cur["ts"],
                    "seq": cur.get("seq"),
                    "field": key,
                    "previous_state": old,
                    "new_state": new,
                    "reason": cur.get("safe_vx_reason") if key == "planner_state" else cur.get("planner_failure_reason"),
                }
            )
    return out


def run_scene(
    client: NavLiveClient,
    world: SimWorld,
    scene_id: str,
    seconds: float,
    hz: float,
    seq_start: int,
) -> Tuple[List[dict], int, dict, str]:
    spec = SCENARIOS[scene_id]
    setup = apply_scenario(client, world, scene_id)
    if not setup.get("success"):
        return (
            [{"ts": time.time(), "seq": seq_start, "scene": scene_id, "error": setup.get("failure_class", "SETUP_FAILED"), "setup": setup, "schema": "navigation_v0.2_trace/2"}],
            seq_start + 1,
            setup,
            "",
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = _SCENE_FILE_TAG.get(scene_id, scene_id)
    rows: List[dict] = []
    seq = seq_start
    prev_row: Optional[dict] = None
    t0 = time.time()
    injected = False
    n = max(1, int(seconds * hz))

    for _ in range(n):
        elapsed = time.time() - t0
        if (not injected) and spec.inject_fn and elapsed >= spec.inject_delay_s:
            pose = client.get("/api/state").get("agv") or {}
            px, py = float(pose.get("x") or 0), float(pose.get("y") or 0)
            yaw = float(pose.get("angle") or 0)
            spec.inject_fn(client, world, (px, py), yaw)
            injected = True
            rows.append(
                {
                    "ts": time.time(),
                    "seq": seq,
                    "scene": scene_id,
                    "event": "OBSTACLE_INJECT",
                    "elapsed_s": round(elapsed, 3),
                    "schema": "navigation_v0.2_trace/2",
                }
            )
            seq += 1

        try:
            sample = _sample_v02(client, seq, scene_id, setup)
            transitions = _detect_transitions(prev_row, sample)
            if transitions:
                sample["transitions"] = transitions
            rows.append(sample)
            prev_row = sample
            seq += 1
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            rows.append({"ts": time.time(), "seq": seq, "scene": scene_id, "error": str(exc), "schema": "navigation_v0.2_trace/2"})
            seq += 1
        time.sleep(1.0 / hz)

    return rows, seq, setup, f"{tag}-{stamp}.jsonl"


def main() -> int:
    ap = argparse.ArgumentParser(description="V0.2 navigation LIVE trace (M3.1 deterministic scenarios)")
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--scene", choices=ALL_SCENES + ["ALL"], default="LIVE-00")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "navigation_v02"))
    ap.add_argument("--navigate", action="store_true", help="Deprecated — scenarios always plan+confirm")
    args = ap.parse_args()

    client = NavLiveClient(base=args.base)
    if not client.ping():
        print("SIM_NOT_RUNNING")
        print(f"Cannot reach {args.base}/api/state")
        print("RESULT: NOT RUN")
        return 2

    world = SimWorld()
    os.makedirs(args.out_dir, exist_ok=True)
    scenes = ALL_SCENES if args.scene == "ALL" else [args.scene]

    written: List[str] = []
    seq = 0
    for sc in scenes:
        print(f"--- {sc}: {SCENARIOS[sc].description}")
        rows, seq, setup, fname = run_scene(client, world, sc, args.seconds, args.hz, seq)
        if not fname:
            print(f"SETUP FAIL: {setup}")
            continue
        jsonl = os.path.join(args.out_dir, fname)
        with open(jsonl, "w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")
        n_trans = sum(len(r.get("transitions") or []) for r in rows if isinstance(r.get("transitions"), list))
        n_err = sum(1 for r in rows if r.get("error"))
        print(f"  → {len(rows)} samples ({n_trans} transitions, {n_err} errors) {jsonl}")
        written.append(jsonl)

    if not written:
        print("RESULT: FAIL (no traces written)")
        return 1
    print(f"Wrote {len(written)} trace file(s)")
    print("RESULT: PASS (trace captured; analyze with _analyze_navigation_v02_trace.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
