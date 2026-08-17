#!/usr/bin/env python3
"""Analyze V0.2 navigation JSONL trace — T0…Tn timeline + per-scenario PASS/FAIL matrix.

Usage:
  python scripts/_analyze_navigation_v02_trace.py trace.jsonl
  python scripts/_analyze_navigation_v02_trace.py logs/navigation_v02/LIVE-*.jsonl
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

TIMELINE_KEYS = (
    "T0",
    "T_detect",
    "T_preview",
    "T_probe",
    "T_commit",
    "T_first_clearance_drop",
    "T_first_constraint_failure",
    "T_first_mppi_infeasible",
    "T_first_safe_vx_zero",
    "T_first_recovery",
    "T_reprobe",
    "T_replan",
    "T_obstacle_pass",
    "T_global_reconnect",
    "T_final",
)

FAILURE_CLASSES = (
    "BASELINE_PATH_FAILURE",
    "OBSTACLE_PLANNER_FAILURE",
    "SAFETY_STOP",
    "RECOVERY_FAILURE",
    "LOCALIZATION_FAILURE",
    "SENSOR_FAILURE",
    "NONE",
)


def _load_rows(path: Path) -> List[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _f(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _valid_samples(rows: List[dict]) -> List[dict]:
    return [r for r in rows if not r.get("error") and r.get("event") not in ("OBSTACLE_INJECT",)]


def _mark(timeline: Dict[str, Optional[dict]], key: str, row: dict) -> None:
    if timeline.get(key) is None:
        timeline[key] = {"ts": row.get("ts"), "seq": row.get("seq"), "scene": row.get("scene"), "row": _compact(row)}


def _compact(r: dict) -> dict:
    keys = (
        "ts", "seq", "scene", "vehicle_pose", "vehicle_vx", "vehicle_omega",
        "avoidance_phase", "committed_side", "planner_state", "recovery_state",
        "recovery_attempt", "safe_vx_reason", "mppi_vx", "approved_vx",
        "footprint_clearance_m", "predicted_min_clearance_m", "valid_candidate_count",
        "collision_rejected_count", "corridor_side", "trajectory_omega_sign", "heading",
        "stop_reason", "planner_failure_reason",
    )
    return {k: r.get(k) for k in keys if k in r}


def _find_timeline(rows: List[dict]) -> Dict[str, Optional[dict]]:
    tl: Dict[str, Optional[dict]] = {k: None for k in TIMELINE_KEYS}
    samples = _valid_samples(rows)
    if not samples:
        return tl
    _mark(tl, "T0", samples[0])
    prev_phase = None
    prev_side = None
    prev_recovery = None
    saw_commit = False
    saw_obstacle = False
    for r in samples:
        fn = _f(r.get("front_near") or r.get("obstacle_distance"), 99.0)
        if fn < 4.5:
            _mark(tl, "T_detect", r)
            saw_obstacle = True
        if r.get("future_collision") or (_f(r.get("first_collision_m"), 99) < 8.0):
            _mark(tl, "T_preview", r)
        ap = str(r.get("avoidance_phase") or "").upper()
        if "PROBE" in ap or r.get("probe_confidence"):
            _mark(tl, "T_probe", r)
        cs = str(r.get("committed_side") or "").upper()
        if cs in ("LEFT", "RIGHT") or "COMMIT" in ap:
            _mark(tl, "T_commit", r)
            saw_commit = True
        fp = r.get("footprint_clearance_m")
        if fp is not None and _f(fp, 99) < 0.55:
            _mark(tl, "T_first_clearance_drop", r)
        if _f(r.get("constraint_rejected_count"), 0) > 0 and _f(r.get("valid_candidate_count"), 1) == 0:
            _mark(tl, "T_first_constraint_failure", r)
        if str(r.get("mppi_failure_reason") or "") == "MPPI_NO_FEASIBLE_TRAJECTORY":
            _mark(tl, "T_first_mppi_infeasible", r)
        if _f(r.get("valid_candidate_count"), -1) == 0 and _f(r.get("candidate_count"), 0) > 0:
            _mark(tl, "T_first_mppi_infeasible", r)
        if abs(_f(r.get("approved_vx"))) < 0.02 and abs(_f(r.get("mppi_vx"))) > 0.02:
            _mark(tl, "T_first_safe_vx_zero", r)
        ps = str(r.get("planner_state") or "")
        rs = str(r.get("recovery_state") or "")
        if ps == "LOCAL_RECOVERY" or rs not in ("", "NONE", "None"):
            _mark(tl, "T_first_recovery", r)
        if rs == "REPROBE":
            _mark(tl, "T_reprobe", r)
        if str(r.get("stop_reason") or "") == "REPLAN":
            _mark(tl, "T_replan", r)
        if saw_commit and prev_phase and "PASS" in ap and "COMMIT" not in ap:
            _mark(tl, "T_obstacle_pass", r)
        if saw_commit and str(r.get("nav_state") or "") == "tracking" and cs in ("", "NONE", "None") and prev_side in ("LEFT", "RIGHT"):
            _mark(tl, "T_global_reconnect", r)
        prev_phase = ap
        prev_side = cs if cs in ("LEFT", "RIGHT") else prev_side
        prev_recovery = rs
    _mark(tl, "T_final", samples[-1])
    if not saw_obstacle:
        for k in ("T_detect", "T_preview", "T_probe", "T_commit", "T_obstacle_pass"):
            if tl[k] is not None and samples[0].get("scene") == "LIVE-00":
                pass  # baseline may still have null detect
    return tl


def _classify_failure(rows: List[dict], setup: dict) -> str:
    if setup.get("failure_class"):
        return str(setup["failure_class"])
    samples = _valid_samples(rows)
    if not samples:
        return "BASELINE_PATH_FAILURE"
    if any(str(r.get("localization_health") or "") not in ("OK", "UNKNOWN", "") for r in samples):
        return "LOCALIZATION_FAILURE"
    if any(str(r.get("sensor_health") or "") not in ("OK", "UNKNOWN", "") for r in samples):
        return "SENSOR_FAILURE"
    final = samples[-1]
    sr = str(final.get("stop_reason") or "")
    ps = str(final.get("planner_state") or "")
    if sr in ("NAVIGATION_FAILED", "FAILED") or ps == "NAVIGATION_FAILED":
        if _first(samples, lambda r: str(r.get("recovery_state") or "") not in ("", "NONE")):
            return "RECOVERY_FAILURE"
        if _first(samples, lambda r: _f(r.get("valid_candidate_count"), -1) == 0):
            return "OBSTACLE_PLANNER_FAILURE"
    if _first(samples, lambda r: str(r.get("safe_vx_reason") or "") not in ("NORMAL", "", "RECOVERY_ACTIVE")):
        return "SAFETY_STOP"
    return "NONE"


def _first(rows: List[dict], pred) -> Optional[Tuple[int, dict]]:
    for i, r in enumerate(rows):
        if pred(r):
            return i, r
    return None


def _corridor_constraint(rows: List[dict]) -> dict:
    violations = []
    for r in _valid_samples(rows):
        cs = str(r.get("committed_side") or r.get("corridor_side") or "").upper()
        ts = str(r.get("trajectory_omega_sign") or "").upper()
        if cs == "LEFT" and ts == "RIGHT":
            violations.append({"ts": r.get("ts"), "committed": cs, "trajectory": ts, "seq": r.get("seq")})
        if cs == "RIGHT" and ts == "LEFT":
            violations.append({"ts": r.get("ts"), "committed": cs, "trajectory": ts, "seq": r.get("seq")})
    return {"result": "PASS" if not violations else "FAIL", "violations": violations[:20]}


def _min_clearance_event(rows: List[dict]) -> Optional[dict]:
    best = None
    best_cl = 1e9
    for r in _valid_samples(rows):
        for key in ("predicted_min_clearance_m", "footprint_clearance_m"):
            v = r.get(key)
            if v is None:
                continue
            cl = _f(v, 99)
            if cl < best_cl:
                best_cl = cl
                best = {
                    "timestamp": r.get("ts"),
                    "seq": r.get("seq"),
                    "clearance_m": cl,
                    "source": key,
                    "vehicle_pose": r.get("vehicle_pose"),
                    "committed_side": r.get("committed_side"),
                    "planner_state": r.get("planner_state"),
                    "safe_vx_reason": r.get("safe_vx_reason"),
                }
    return best


def _stage_verdict(rows: List[dict], scene: str) -> Dict[str, str]:
    """Per-stage PASS / FAIL / NOT REACHED / N/A from JSONL evidence."""
    samples = _valid_samples(rows)
    tl = _find_timeline(rows)
    is_baseline = scene == "LIVE-00"
    is_blocked = scene == "LIVE-03"

    def reached(key: str) -> bool:
        return tl.get(key) is not None

    def moving(samples: List[dict]) -> bool:
        return any(abs(_f(r.get("vehicle_vx"))) > 0.04 for r in samples)

    v: Dict[str, str] = {}
    v["Baseline"] = "PASS" if (is_baseline and moving(samples) and not _nav_failed(samples)) else (
        "N/A" if not is_baseline else "FAIL"
    )
    v["Detection"] = "N/A" if is_baseline else ("PASS" if reached("T_detect") else "NOT REACHED")
    v["Preview"] = "N/A" if is_baseline else ("PASS" if reached("T_preview") else "NOT REACHED")
    v["Probe"] = "N/A" if is_baseline else ("PASS" if reached("T_probe") else "NOT REACHED")
    v["Commit"] = "N/A" if is_baseline else ("PASS" if reached("T_commit") else "NOT REACHED")
    ec = _corridor_constraint(rows)
    v["Corridor"] = "N/A" if is_baseline else ("PASS" if ec["result"] == "PASS" and reached("T_commit") else ("FAIL" if ec["result"] == "FAIL" else "NOT REACHED"))
    mc = _min_clearance_event(rows)
    v["Footprint"] = "N/A" if is_baseline else (
        "PASS" if mc and mc["clearance_m"] > 0 else ("FAIL" if mc and mc["clearance_m"] <= 0 else "NOT REACHED")
    )
    mppi_ok = _first(samples, lambda r: _f(r.get("valid_candidate_count"), 0) > 0)
    v["MPPI"] = "PASS" if mppi_ok else ("N/A" if is_blocked and reached("T_first_mppi_infeasible") else "NOT REACHED")
    safety_veto = _first(samples, lambda r: str(r.get("safe_vx_reason") or "") not in ("NORMAL", "", "RECOVERY_ACTIVE"))
    v["Safety"] = "PASS" if not safety_veto or is_blocked else "NOT REACHED"
    v["Recovery"] = "N/A" if is_baseline else ("PASS" if reached("T_first_recovery") else "NOT REACHED")
    v["Reconnect"] = "N/A" if is_baseline else ("PASS" if reached("T_global_reconnect") else "NOT REACHED")

    if is_blocked:
        exhausted = _first(samples, lambda r: str(r.get("stop_reason") or "") in ("NAVIGATION_FAILED", "FAILED") and _f(r.get("recovery_attempt"), 0) >= 1)
        v["Overall"] = "PASS" if exhausted and reached("T_first_recovery") else "FAIL"
    elif is_baseline:
        v["Overall"] = "PASS" if v["Baseline"] == "PASS" else "FAIL"
    else:
        need = ["Detection", "Preview", "Probe", "Commit"]
        if all(v.get(k) == "PASS" for k in need):
            v["Overall"] = "PASS"
        elif any(v.get(k) == "FAIL" for k in v):
            v["Overall"] = "FAIL"
        else:
            v["Overall"] = "PARTIAL"
    return v


def _nav_failed(samples: List[dict]) -> bool:
    if not samples:
        return True
    last = samples[-1]
    return str(last.get("stop_reason") or "") in ("NAVIGATION_FAILED", "FAILED") or str(last.get("planner_state") or "") == "NAVIGATION_FAILED"


def analyze(path: Path) -> dict:
    rows = _load_rows(path)
    samples = _valid_samples(rows)
    setup = {}
    for r in rows:
        if isinstance(r.get("setup"), dict):
            setup = r["setup"]
            break
    scene = samples[0].get("scene") if samples else rows[0].get("scene") if rows else "UNKNOWN"
    timeline = _find_timeline(rows)
    failure_class = _classify_failure(rows, setup)
    stages = _stage_verdict(rows, str(scene))
    corridor = _corridor_constraint(rows)
    min_ev = _min_clearance_event(rows)
    return {
        "file": str(path),
        "scene": scene,
        "sample_count": len(samples),
        "timeline": timeline,
        "failure_class": failure_class,
        "stages": stages,
        "execution_corridor": corridor,
        "minimum_clearance_event": min_ev,
        "setup": setup,
    }


def _print_report(report: dict) -> None:
    print(f"\n=== Analyze: {report['file']} scene={report.get('scene')} ({report['sample_count']} samples) ===")
    print(f"Failure class: {report.get('failure_class')}")
    print("\nTimeline (T0…Tn):")
    for k in TIMELINE_KEYS:
        v = report["timeline"].get(k)
        if v:
            print(f"  {k}: ts={v.get('ts')} seq={v.get('seq')}")
        else:
            print(f"  {k}: N/A")
    print("\nStage verdicts:")
    for k, val in report.get("stages", {}).items():
        print(f"  {k}: {val}")
    ec = report.get("execution_corridor") or {}
    print(f"\nExecutionCorridor: {ec.get('result')} violations={len(ec.get('violations') or [])}")
    mce = report.get("minimum_clearance_event")
    if mce:
        print(f"Minimum clearance: {mce.get('clearance_m')} m @ seq={mce.get('seq')}")


def _print_matrix(reports: List[dict]) -> None:
    cols = ["Detection", "Preview", "Probe", "Commit", "Corridor", "Footprint", "MPPI", "Safety", "Recovery", "Reconnect", "Overall"]
    print("\n=== Evidence Matrix (from JSONL) ===")
    print(f"{'Scenario':<14} " + " ".join(f"{c[:6]:>6}" for c in cols))
    for r in reports:
        st = r.get("stages") or {}
        row = f"{str(r.get('scene')):<14} "
        row += " ".join(f"{str(st.get(c, 'N/A'))[:6]:>6}" for c in cols)
        print(row)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="JSONL file(s) or glob")
    ap.add_argument("--json", action="store_true", help="Print JSON report")
    ap.add_argument("--matrix", action="store_true", help="Print evidence matrix")
    args = ap.parse_args()

    files: List[Path] = []
    for p in args.paths:
        if "*" in p:
            files.extend(Path(x) for x in glob.glob(p))
        else:
            files.append(Path(p))
    files = sorted(set(files))
    if not files:
        print("No trace files found", file=sys.stderr)
        return 1

    reports = [analyze(f) for f in files]
    if args.json:
        print(json.dumps(reports, indent=2, ensure_ascii=False, default=str))
    else:
        for r in reports:
            _print_report(r)
        if args.matrix or len(reports) > 1:
            _print_matrix(reports)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
