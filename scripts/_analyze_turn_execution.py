#!/usr/bin/env python3
"""Analyze turn execution latency from navigation V0.2 JSONL traces."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_turn_execution import (  # noqa: E402
    TURN_OMEGA_EPSILON,
    TURN_START_THRESHOLD,
    detect_turn_start_frames,
    turn_required_distance_m,
)


def _load_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _samples(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [r for r in rows if r.get("vehicle_pose") and not r.get("event")]


def _events(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [r for r in rows if r.get("event")]


def _first_ts(samples: List[Dict[str, Any]], pred) -> Optional[Dict[str, Any]]:
    for r in samples:
        if pred(r):
            return r
    return None


def _turn_start_index(samples: List[Dict[str, Any]], key: str) -> Optional[int]:
    omegas = [float(r.get(key) or 0) for r in samples]
    idx = detect_turn_start_frames(omegas)
    return idx


def analyze(path: Path) -> Dict[str, Any]:
    rows = _load_rows(path)
    samples = _samples(rows)
    events = _events(rows)
    if not samples:
        return {"file": str(path), "error": "no samples"}

    t0 = _first_ts(events, lambda r: r.get("event") == "ONLINE_OBSTACLE_INJECT")
    t0 = t0 or _first_ts(samples, lambda r: float(r.get("front_near") or 99) < 5.0)
    t0_ts = t0["ts"] if t0 else samples[0]["ts"]
    t0_sample = _first_ts(samples, lambda r: float(r.get("ts") or 0) >= float(t0_ts))

    def _at(key: str) -> Optional[Dict[str, Any]]:
        idx = _turn_start_index(samples, key)
        return samples[idx] if idx is not None else None

    t_plan = _first_ts(
        samples,
        lambda r: r.get("trajectory_kind") == "FORWARD_FUTURE"
        and (r.get("trajectory_side") in ("LEFT", "RIGHT") or abs(float(r.get("direction_angle") or 0)) > 15),
    )
    t_mppi = _first_ts(samples, lambda r: abs(float(r.get("mppi_omega") or 0)) >= TURN_OMEGA_EPSILON)
    t_req = _at("requested_omega")
    t_app = _at("approved_omega")
    t_act = _at("actual_omega")
    t_safe = _first_ts(samples, lambda r: float(r.get("safe_vx") or 1) < 0.02 and float(r.get("front_near") or 99) < 1.5)

    ages = [float(r["trajectory_age_ms"]) for r in samples if r.get("trajectory_age_ms") is not None]
    compute = []
    for r in samples:
        pst, pft = r.get("planner_start_timestamp"), r.get("planner_finish_timestamp")
        if pst and pft and float(pft) > float(pst):
            compute.append((float(pft) - float(pst)) * 1000.0)

    vx = float((t_act or t_app or samples[-1]).get("actual_vx") or 0.15)
    req_d = turn_required_distance_m(vx=vx, omega_target=0.28)

    def _lat(t_start: Optional[Dict[str, Any]], t_end: Optional[Dict[str, Any]]) -> Optional[float]:
        if not t_start or not t_end:
            return None
        return (float(t_end["ts"]) - float(t_start["ts"])) * 1000.0

    planner_to_actual = _lat(t_plan or {"ts": t0_ts}, t_act)
    dist_latency = None if planner_to_actual is None else vx * planner_to_actual / 1000.0

    obs_at_turn = float(t_act.get("front_near") or 0) if t_act else None
    margin = None if obs_at_turn is None else obs_at_turn - req_d - 0.35
    late = margin is not None and margin < 0

    report = {
        "file": str(path),
        "sample_count": len(samples),
        "T_detect": t0_ts,
        "T_plan_turn": None if not t_plan else t_plan["ts"],
        "T_mppi_turn": None if not t_mppi else t_mppi["ts"],
        "T_requested_turn": None if not t_req else t_req["ts"],
        "T_approved_turn": None if not t_app else t_app["ts"],
        "T_actual_turn": None if not t_act else t_act["ts"],
        "T_safe_stop": None if not t_safe else t_safe["ts"],
        "distance_at_detect": float(t0_sample.get("front_near") or 0) if t0_sample else None,
        "obstacle_detect_distance": float(t0_sample.get("front_near") or 0) if t0_sample else None,
        "turn_decision_distance": None if not t_plan else t_plan.get("front_near"),
        "turn_command_distance": None if not t_req else t_req.get("front_near"),
        "distance_at_actual_turn": obs_at_turn,
        "actual_turn_start_distance": obs_at_turn,
        "safe_stop_distance": None if not t_safe else t_safe.get("front_near"),
        "planner_to_actual_ms": planner_to_actual,
        "distance_during_planner_to_actual_m": dist_latency,
        "vehicle_forward_distance_after_detection": dist_latency,
        "vehicle_forward_distance_before_turn": dist_latency,
        "remaining_clearance": obs_at_turn,
        "turn_required_distance_m": req_d,
        "turn_margin_m": margin,
        "formal_late_turn_margin": {
            "value_m": margin,
            "status": "INCOMPLETE",
            "note": "omega ramp only; not a safety proof",
        },
        "late_turn": late,
        "planner_age_max_ms": max(ages) if ages else None,
        "planner_compute_p50_ms": statistics.median(compute) if compute else None,
        "planner_compute_max_ms": max(compute) if compute else None,
        "control_eligible_true": sum(1 for r in samples if r.get("control_eligible") is True),
        "turn_start_threshold": TURN_START_THRESHOLD,
        "planner_to_mppi_latency_ms": _lat(t_plan or {"ts": t0_ts}, t_mppi),
        "mppi_to_requested_latency_ms": _lat(t_mppi, t_req),
        "requested_to_approved_latency_ms": _lat(t_req, t_app),
        "approved_to_actual_latency_ms": _lat(t_app, t_act),
    }
    if late:
        report["late_turn_event"] = {
            "obstacle_distance_m": obs_at_turn,
            "turn_required_distance_m": req_d,
            "actual_turn_start_distance_m": obs_at_turn,
            "margin_m": margin,
        }
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="Turn execution JSONL analyzer")
    ap.add_argument("jsonl", nargs="+", help="JSONL trace file(s)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    reports = [analyze(Path(p)) for p in args.jsonl]
    if args.json:
        print(json.dumps(reports, indent=2))
    else:
        for rep in reports:
            print("=" * 60)
            print(rep["file"])
            for k in (
                "T_plan_turn",
                "T_mppi_turn",
                "T_requested_turn",
                "T_approved_turn",
                "T_actual_turn",
                "T_safe_stop",
                "planner_to_actual_ms",
                "distance_during_planner_to_actual_m",
                "turn_required_distance_m",
                "distance_at_actual_turn",
                "turn_margin_m",
                "late_turn",
                "planner_age_max_ms",
                "planner_compute_p50_ms",
                "control_eligible_true",
            ):
                print(f"  {k}: {rep.get(k)}")
            if rep.get("late_turn_event"):
                print("  LATE TURN EVENT:", rep["late_turn_event"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
