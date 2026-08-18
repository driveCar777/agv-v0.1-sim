#!/usr/bin/env python3
"""Field / sim collision forensic pipeline.

Does not prove safety. Reconstructs FIRST SAFETY FAILURE timeline from JSONL.

Usage:
  python scripts/_analyze_field_collision.py --jsonl path/to/field.jsonl
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


def _load(path: Path) -> List[dict]:
    return [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]


def _ts(row: dict) -> Optional[float]:
    return row.get("ts") or row.get("timestamp")


def _boolish(v: Any) -> bool:
    if isinstance(v, bool):
        return v
    if v in (None, "", 0, "0", "false", "False", "NONE"):
        return False
    return True


def analyze(rows: List[dict]) -> Dict[str, Any]:
    t_detect = t_preview = t_pred = t_invalid = t_veto = t_cmd = t_coll = None
    first_should_stop = None
    collision_rows = []
    timeline = []

    for r in rows:
        ts = _ts(r)
        ev = r.get("event")
        if ev:
            timeline.append({"ts": ts, "seq": r.get("seq"), "event": ev})
        fn = r.get("front_near")
        fc = r.get("future_collision") or r.get("first_collision_m")
        pred = r.get("predicted_min_clearance_m")
        fp = r.get("footprint_clearance_m")
        reason = str(r.get("safe_vx_reason") or "")
        approved = r.get("approved_vx")
        stop = str(r.get("stop_reason") or "")
        coll = r.get("collision")

        if t_detect is None and fn is not None and float(fn) < 5.0:
            t_detect = {"ts": ts, "seq": r.get("seq"), "front_near": fn}
        if t_preview is None and (r.get("avoidance_phase") not in (None, "OPEN", "") or r.get("readiness_signal") not in (None, "NONE", "")):
            t_preview = {"ts": ts, "seq": r.get("seq"), "phase": r.get("avoidance_phase"), "signal": r.get("readiness_signal")}
        if t_pred is None and (fc or (pred is not None and float(pred) < 0.3)):
            t_pred = {"ts": ts, "seq": r.get("seq"), "future_collision": fc, "predicted_min_clearance_m": pred}
        if t_invalid is None and str(r.get("planner_failure_reason") or "").upper() not in ("", "NONE"):
            t_invalid = {"ts": ts, "seq": r.get("seq"), "planner_failure_reason": r.get("planner_failure_reason")}
        vetoish = reason not in ("", "NORMAL") or stop not in ("", "NONE")
        if t_veto is None and vetoish:
            t_veto = {"ts": ts, "seq": r.get("seq"), "safe_vx_reason": reason, "stop_reason": stop, "approved_vx": approved}
        if t_cmd is None and approved is not None:
            t_cmd = {"ts": ts, "seq": r.get("seq"), "approved_vx": approved, "requested_vx": r.get("requested_vx")}
        if coll:
            collision_rows.append(r)
            if t_coll is None:
                t_coll = {"ts": ts, "seq": r.get("seq"), "stop_reason": stop, "front_near": fn, "footprint_clearance_m": fp, "approved_vx": approved}

        # FIRST SAFETY FAILURE: predicted/actual contact while approved_vx still positive and reason NORMAL
        if first_should_stop is None:
            danger = False
            if coll:
                danger = True
            if pred is not None and float(pred) < 0.08:
                danger = True
            if fp is not None and float(fp) < 0.08:
                danger = True
            if fn is not None and float(fn) < 0.25:
                danger = True
            if danger and (approved is None or float(approved) > 0.02) and reason in ("", "NORMAL"):
                first_should_stop = {
                    "ts": ts,
                    "seq": r.get("seq"),
                    "front_near": fn,
                    "predicted_min_clearance_m": pred,
                    "footprint_clearance_m": fp,
                    "approved_vx": approved,
                    "safe_vx_reason": reason,
                    "stop_reason": stop,
                    "note": "system should have blocked motion; approved_vx still > 0 with NORMAL",
                }

    owner = "UNKNOWN"
    missing = []
    if not any(k in (rows[0] if rows else {}) for k in ("vehicle_pose", "approved_vx", "collision")):
        missing.append("JSONL schema not a navigation_v0.2_trace")
    if t_coll is None:
        missing.append("no collision=true frame in this file")
        status = "NO_COLLISION_IN_FILE"
    else:
        status = "COLLISION_PRESENT"
        if first_should_stop:
            owner = "SAFETY"
        elif t_pred is None:
            owner = "PERCEPTION"
        elif t_invalid is None and t_veto is None:
            owner = "SAFETY"
        elif t_veto and t_coll and (t_veto.get("approved_vx") or 0) > 0.02:
            owner = "SAFETY"
        else:
            owner = "UNKNOWN"

    return {
        "status": status,
        "owner_hypothesis": owner,
        "T_obstacle_detect": t_detect,
        "T_obstacle_worldmodel": t_preview,
        "T_predicted_collision": t_pred,
        "T_trajectory_invalid": t_invalid,
        "T_safety_veto": t_veto,
        "T_command": t_cmd,
        "T_collision": t_coll,
        "FIRST_SAFETY_FAILURE": first_should_stop,
        "collision_frame_count": len(collision_rows),
        "missing": missing,
        "events": timeline[:40],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Collision forensic timeline from JSONL")
    ap.add_argument("--jsonl", required=True)
    args = ap.parse_args()
    path = Path(args.jsonl)
    if not path.exists():
        print(f"MISSING {path}")
        print("FIELD_COLLISION_ROOT_CAUSE = UNVERIFIED")
        print("Missing: field JSONL file")
        return 2
    rows = _load(path)
    if not rows:
        print("EMPTY JSONL")
        print("FIELD_COLLISION_ROOT_CAUSE = UNVERIFIED")
        print("Missing: non-empty JSONL")
        return 2
    rep = analyze(rows)
    print("=" * 60)
    print(" COLLISION FORENSICS")
    print("=" * 60)
    print(json.dumps(rep, indent=2, default=str))
    if rep["status"] != "COLLISION_PRESENT":
        print("FIELD_COLLISION_ROOT_CAUSE = UNVERIFIED")
        print("Missing:", "; ".join(rep["missing"]) or "collision frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
