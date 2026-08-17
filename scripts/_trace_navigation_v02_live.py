#!/usr/bin/env python3
"""V0.2 LIVE trace — full navigation timeline (JSONL).

Polls existing APIs only; does not recompute planner/safety logic.

Scenes:
  LIVE-01  static front obstacle (manual placement or navigate into obstacle)
  LIVE-02  dynamic obstacle (manual moving actor)
  LIVE-03  P0-D.1 field-failure proxy (heading vs footprint clearance)

Requires web sim at --base (default http://127.0.0.1:19999).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Fields tracked for state-transition detection
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


def _get(base: str, path: str) -> dict:
    url = base.rstrip("/") + path
    with urllib.request.urlopen(url, timeout=8.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _post(base: str, path: str, body: dict) -> dict:
    url = base.rstrip("/") + path
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=12.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _dig(d: dict, *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


def _sample_v02(base: str, seq: int, scene: str) -> dict:
    """Single timeline sample from existing API surfaces."""
    st = _get(base, "/api/state")
    dbg_raw = {}
    op = {}
    diag = {}
    safety = {}
    recovery = {}
    try:
        dbg_raw = _get(base, "/api/nav/debug")
    except Exception:
        pass
    dbg = dbg_raw.get("debug") if isinstance(dbg_raw.get("debug"), dict) else dbg_raw
    try:
        op = _get(base, "/api/nav/obstacle-preview")
    except Exception:
        pass
    try:
        diag = _get(base, "/api/logs/diagnostics?window_s=5")
    except Exception:
        pass
    try:
        safety = _get(base, "/api/nav/safety")
    except Exception:
        safety = st.get("safety") or _dig(dbg, "safety") or {}
    try:
        recovery = _get(base, "/api/nav/recovery")
    except Exception:
        recovery = _dig(dbg, "execution_recovery") or _dig(dbg, "recovery") or {}

    nav = st.get("nav") or {}
    agv = st.get("agv") or {}
    status = dbg.get("status") or {}
    vc = dbg.get("velocity_chain") or {}
    lp = dbg.get("local_planner") or nav.get("local_plan") or {}
    mppi_meta = st.get("debug", {}).get("mppi_meta") if isinstance(st.get("debug"), dict) else {}
    if not mppi_meta:
        # debug hub ADVANCED exposes batch/counters on local_planner when physics active
        mppi_meta = lp if isinstance(lp, dict) else {}
    nav_pol = dbg.get("nav_policy") or {}

    ec = nav_pol.get("execution_corridor") or _dig(dbg, "nav_policy", "execution_corridor") or nav.get("execution_corridor") or {}
    if not ec:
        ec = _dig(op, "obstacle_preview", "execution_corridor") or {}

    row = {
        "ts": time.time(),
        "seq": seq,
        "scene": scene,
        "schema": "navigation_v0.2_trace/1",
        # pose / motion
        "vehicle_pose": {"x": agv.get("x"), "y": agv.get("y"), "yaw": agv.get("angle")},
        "vehicle_vx": agv.get("vx"),
        "vehicle_omega": agv.get("w"),
        "state_vx": nav.get("state_vx") or status.get("vx"),
        "state_omega": nav.get("state_w") or status.get("w"),
        # nav / behavior
        "nav_state": nav.get("mode") or status.get("nav_mode"),
        "behavior_state": _dig(dbg, "nav_policy", "state") or nav.get("policy_state") or status.get("policy_state"),
        "maneuver_mode": _dig(dbg, "maneuver", "mode") or nav.get("maneuver_mode") or status.get("maneuver_mode"),
        "avoidance_phase": op.get("avoidance_phase") or _dig(dbg, "nav_policy", "avoidance_phase", "phase"),
        "probe_confidence": op.get("probe_confidence"),
        "committed_side": op.get("committed_side") or _dig(dbg, "nav_policy", "committed_side"),
        "execution_corridor": ec if ec else None,
        # clearance / obstacle
        "obstacle_distance": st.get("front_near") or nav.get("safety_envelope", {}).get("front_near"),
        "front_near": nav.get("safety_envelope", {}).get("front_near") or st.get("front_near"),
        "footprint_clearance_m": nav.get("footprint_clearance_m") or safety.get("footprint_clearance_m") or status.get("footprint_clearance_m"),
        "predicted_min_clearance_m": nav.get("predicted_min_clearance_m") or safety.get("predicted_min_clearance_m") or mppi_meta.get("predicted_min_clearance_m"),
        "future_collision": _dig(op, "obstacle_preview", "future_collision") or op.get("future_collision"),
        "first_collision_m": op.get("first_collision_distance_m"),
        # MPPI diagnostics
        "candidate_count": mppi_meta.get("candidate_count") or lp.get("candidate_count") or nav.get("candidate_count"),
        "valid_candidate_count": mppi_meta.get("valid_candidate_count"),
        "collision_rejected_count": mppi_meta.get("collision_rejected_count"),
        "constraint_rejected_count": mppi_meta.get("constraint_rejected_count"),
        "clearance_rejected_count": mppi_meta.get("clearance_rejected_count"),
        "mppi_failure_reason": mppi_meta.get("failure_reason"),
        # command chain
        "mppi_vx": nav.get("mppi_vx") or vc.get("mppi_vx"),
        "mppi_omega": nav.get("mppi_w") or vc.get("requested_omega") or dbg.get("local_planner", {}).get("mppi_w"),
        "requested_vx": nav.get("requested_vx") or vc.get("requested_vx") or nav.get("cmd_vx_before_safety"),
        "approved_vx": nav.get("approved_vx") or vc.get("approved_vx") or nav.get("cmd_vx_after_safety"),
        "requested_omega": nav.get("requested_omega") or vc.get("requested_omega"),
        "approved_omega": nav.get("approved_omega") or vc.get("approved_omega") or nav.get("cmd_w_after_safety"),
        "safe_vx": vc.get("safe_vx") or nav.get("cmd_vx_after_safety"),
        "safe_vx_reason": nav.get("safe_vx_reason") or safety.get("safe_vx_reason") or status.get("safe_vx_reason") or vc.get("safe_vx_reason"),
        # planner / recovery
        "planner_state": nav.get("planner_state") or status.get("planner_state"),
        "planner_failure_reason": nav.get("planner_failure_reason") or status.get("planner_failure_reason"),
        "recovery_state": nav.get("recovery_state") or recovery.get("recovery_state") or status.get("recovery_state"),
        "recovery_attempt": nav.get("recovery_attempt") or recovery.get("recovery_attempt") or status.get("recovery_attempt"),
        "stop_reason": nav.get("stop_reason") or status.get("stop_reason"),
        "nav_ui_severity": nav.get("nav_ui_severity") or safety.get("nav_ui_severity") or status.get("nav_ui_severity"),
        # health (explicit when unavailable)
        "sensor_health": diag.get("sensor_health") or "UNKNOWN",
        "localization_health": diag.get("localization_health") or ("OK" if float(agv.get("confidence") or 0) > 0.5 else "DEGRADED"),
        # trajectory selection
        "selected_candidate": lp.get("selected_candidate") or dbg.get("local_planner", {}).get("selected_candidate"),
        "selected_trajectory_mode": _dig(dbg, "local_planner", "mode"),
        "trajectory_omega_sign": _omega_sign(nav.get("mppi_w") or vc.get("requested_omega")),
        "corridor_side": _corridor_side(ec),
        "heading": agv.get("angle"),
        "diagnostics_primary": _dig(diag, "decision", "primary") or _dig(dbg, "diagnostics", "primary"),
        "braking_calibration_status": "CALIBRATION_REQUIRED",
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


def _ping(base: str) -> bool:
    try:
        _get(base, "/api/state")
        return True
    except Exception:
        return False


def run_scene(base: str, scene: str, seconds: float, hz: float, prev: Optional[dict], seq_start: int) -> Tuple[List[dict], Optional[dict], int]:
    rows: List[dict] = []
    seq = seq_start
    prev_row = prev
    n = max(1, int(seconds * hz))
    for _ in range(n):
        try:
            sample = _sample_v02(base, seq, scene)
            transitions = _detect_transitions(prev_row, sample)
            if transitions:
                sample["transitions"] = transitions
            rows.append(sample)
            prev_row = sample
            seq += 1
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            rows.append({"ts": time.time(), "seq": seq, "scene": scene, "error": str(exc), "schema": "navigation_v0.2_trace/1"})
            seq += 1
        time.sleep(1.0 / hz)
    return rows, prev_row, seq


def main() -> int:
    ap = argparse.ArgumentParser(description="V0.2 navigation LIVE trace (JSONL)")
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--scene", choices=("LIVE-01", "LIVE-02", "LIVE-03", "ALL"), default="ALL")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "navigation_v02"))
    ap.add_argument("--navigate", action="store_true", help="POST /api/nav/plan to a forward goal before trace")
    args = ap.parse_args()

    if not _ping(args.base):
        print("SIM_NOT_RUNNING")
        print(f"Cannot reach {args.base}/api/state")
        print("RESULT: NOT RUN")
        return 2

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    jsonl = os.path.join(args.out_dir, f"nav_v02_{args.scene}_{stamp}.jsonl")

    if args.navigate:
        try:
            st = _get(args.base, "/api/state")
            x = float(_dig(st, "agv", "x") or 0)
            y = float(_dig(st, "agv", "y") or 0)
            _post(args.base, "/api/nav/plan", {"x": x + 8.0, "y": y, "yaw": 0.0})
            time.sleep(1.0)
            _post(args.base, "/api/nav/confirm", {})
            print("Navigation goal set (+8m forward)")
        except Exception as exc:
            print(f"WARN: navigate failed: {exc}")

    scenes = ["LIVE-01", "LIVE-02", "LIVE-03"] if args.scene == "ALL" else [args.scene]
    hints = {
        "LIVE-01": "Place static obstacle ahead OR use --navigate into map obstacle",
        "LIVE-02": "Inject moving actor manually; observe dynamic wait/resume",
        "LIVE-03": "Observe heading vs footprint clearance during LEFT/RIGHT commit turn",
    }

    all_rows: List[dict] = []
    prev: Optional[dict] = None
    seq = 0
    for sc in scenes:
        print(f"--- {sc} ({args.seconds}s @ {args.hz}Hz) — {hints.get(sc, '')}")
        rows, prev, seq = run_scene(args.base, sc, args.seconds, args.hz, prev, seq)
        all_rows.extend(rows)

    with open(jsonl, "w", encoding="utf-8") as fh:
        for r in all_rows:
            fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")

    n_trans = sum(len(r.get("transitions") or []) for r in all_rows if isinstance(r.get("transitions"), list))
    n_err = sum(1 for r in all_rows if r.get("error"))
    print(f"Wrote {len(all_rows)} samples ({n_trans} transitions, {n_err} errors) → {jsonl}")
    if n_err == len(all_rows):
        print("RESULT: FAIL (all samples errored)")
        return 1
    if n_err > 0:
        print("RESULT: PARTIAL (some errors)")
        return 0
    print("RESULT: PASS (trace captured; analyze with _analyze_navigation_v02_trace.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
