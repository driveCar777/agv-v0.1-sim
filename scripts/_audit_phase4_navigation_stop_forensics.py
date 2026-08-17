#!/usr/bin/env python3
"""P0-B-0 — Stop forensics audit (Cases A–F). No planning behavior changes."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_log_forensics import classify_stop_case, infer_likely_owner  # noqa: E402
from agv_bridge.nav_observability import OBS, redact_mapping  # noqa: E402


def main() -> int:
    fails = []

    # Case A — no candidates
    a = infer_likely_owner(candidate_count=0, valid_candidate_count=0, selected_candidate="NONE")
    if a != "LOCAL_PLANNING":
        fails.append(f"A expected LOCAL_PLANNING got {a}")

    # Case B — all invalid
    b = infer_likely_owner(candidate_count=4, valid_candidate_count=0, selected_candidate="NONE")
    if b != "CANDIDATE":
        fails.append(f"B expected CANDIDATE got {b}")

    # Case C — selected but requested_vx=0
    c = infer_likely_owner(
        candidate_count=3,
        valid_candidate_count=2,
        selected_candidate="LEFT",
        requested_vx=0.0,
        requested_w=0.0,
        safe_vx=0.0,
        state_vx=0.0,
    )
    if c not in ("FSM", "POLICY"):
        fails.append(f"C expected FSM/POLICY got {c}")

    # Case D — safety clamp
    d = infer_likely_owner(
        candidate_count=2,
        valid_candidate_count=1,
        selected_candidate="LEFT",
        requested_vx=0.2,
        safe_vx=0.0,
        state_vx=0.0,
    )
    if d != "SAFETY":
        fails.append(f"D expected SAFETY got {d}")

    # Case E — execution/physics after safety
    e = infer_likely_owner(
        candidate_count=2,
        valid_candidate_count=1,
        selected_candidate="LEFT",
        requested_vx=0.2,
        safe_vx=0.2,
        state_vx=0.0,
    )
    if e != "EXECUTION":
        fails.append(f"E expected EXECUTION got {e}")

    # Case F — spin
    f = classify_stop_case(
        candidate_count=2,
        valid_candidate_count=1,
        selected_candidate="LEFT",
        requested_vx=0.0,
        safe_vx=0.0,
        state_vx=0.0,
        state_w=0.35,
        translation_delta=0.01,
    )
    if not f["spin_loop_suspected"]:
        fails.append("F expected spin_loop_suspected")
    if f["event"] != "SPIN_LOOP_SUSPECTED":
        fails.append(f"F event={f['event']}")

    # Explicit constructions from spec
    if (
        infer_likely_owner(
            requested_vx=0.2,
            safe_vx=0,
            state_vx=0,
            candidate_count=1,
            valid_candidate_count=1,
            selected_candidate="FWD",
        )
        != "SAFETY"
    ):
        fails.append("spec SAFETY chain")
    if (
        infer_likely_owner(
            requested_vx=0.2,
            safe_vx=0.2,
            state_vx=0,
            candidate_count=1,
            valid_candidate_count=1,
            selected_candidate="FWD",
        )
        != "EXECUTION"
    ):
        fails.append("spec EXECUTION chain")
    if (
        infer_likely_owner(selected_candidate="NONE", valid_candidate_count=0, candidate_count=3)
        != "CANDIDATE"
    ):
        fails.append("spec CANDIDATE chain")

    # API filter / since / cursor / focus
    OBS.configure(enabled=True, level="INFO")
    OBS.events.clear()
    OBS.start_cycle()
    OBS.ensure_trace("AVOID")
    e1 = OBS.emit("SAFETY_CLAMP", level="WARN", category="SAFETY", force=True)
    e2 = OBS.emit("SPIN_LOOP_SUSPECTED", level="WARN", category="DIAGNOSTIC", force=True)
    e3 = OBS.emit("CANDIDATE_SELECTED", level="INFO", category="CANDIDATE", force=True)
    if not e1 or not e2 or not e3:
        fails.append("emit failed")
    q = OBS.query_events(level="WARN", limit=50)
    if q["count"] < 2:
        fails.append(f"filter level WARN count={q['count']}")
    q2 = OBS.query_events(category="SAFETY")
    if not any(x.get("event") == "SAFETY_CLAMP" for x in q2["events"]):
        fails.append("category SAFETY filter")
    q3 = OBS.query_events(event="SPIN_LOOP_SUSPECTED")
    if q3["count"] < 1:
        fails.append("event filter")
    q4 = OBS.query_events(focus="STOP")
    if q4["count"] < 1:
        fails.append("focus STOP")
    q5 = OBS.query_events(since=e1["event_id"], limit=50)
    ids = [x["event_id"] for x in q5["events"]]
    if e1["event_id"] in ids:
        fails.append("since should be exclusive of since event_id")
    if e2["event_id"] not in ids and e3["event_id"] not in ids:
        fails.append("since did not return later events")

    # ingest decision trace
    dbg = {
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "velocity_chain": {"cmd_vx": 0.2, "safe_vx": 0.0, "state_vx": 0.0, "mppi_vx": 0.2},
        "maneuver": {"mode": "LOCAL_LEFT", "decision": "LEFT", "force_vx": 0.18},
        "local_maneuver": {
            "selected": "LEFT",
            "candidates": [
                {
                    "type": "LEFT",
                    "feasible": True,
                    "collision": False,
                    "total_cost": 1.2,
                    "vx": 0.18,
                    "w": 0.2,
                    "duration": 1.5,
                    "path": [{"x": 0, "y": 0}, {"x": 0.3, "y": 0.1}],
                    "progress_cost": 0.2,
                    "clearance_cost": 0.4,
                },
                {
                    "type": "RIGHT",
                    "feasible": False,
                    "collision": True,
                    "reason": "COLLISION",
                    "total_cost": 9.0,
                    "vx": 0.1,
                    "w": -0.2,
                    "duration": 1.5,
                },
            ],
        },
        "nav_mode": "tracking",
        "front_near": 0.5,
        "processed_path": [{"x": 0, "y": 0}, {"x": 5, "y": 0}],
        "goal_distance": 5.0,
    }
    # force stop diagnostic path: hold low vx across two ingest with time mock is hard;
    # at least decision owner should be SAFETY
    out = OBS.ingest_debug_snapshot(dbg)
    if out.get("likely_owner") != "SAFETY":
        fails.append(f"ingest owner={out.get('likely_owner')}")
    if not (out.get("decision") or {}).get("candidates", {}).get("items"):
        fails.append("candidate items missing")
    sb = ((out.get("decision") or {}).get("candidates") or {}).get("items") or []
    left = next((c for c in sb if "LEFT" in str(c.get("kind") or "").upper()), None)
    if left and "progress_cost" not in (left.get("score_breakdown") or {}):
        fails.append("score_breakdown missing")

    red = redact_mapping({"authorization": "secret", "ok": 1})
    if red["authorization"] != "***REDACTED***" or red["ok"] != 1:
        fails.append("redact failed")

    # overhead smoke
    overheads = []
    for _ in range(30):
        r = OBS.ingest_debug_snapshot(dbg)
        overheads.append(float(r.get("logging_overhead_ms") or 0))
    avg = sum(overheads) / len(overheads)
    print(f"LOGGING_OVERHEAD_AVG_MS={avg:.3f}")

    print("=== P0-B-0 Stop Forensics Audit ===")
    if fails:
        for fmsg in fails:
            print("FAIL:", fmsg)
        return 1
    print("PASS: A–F + filters + ingest + redact")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
