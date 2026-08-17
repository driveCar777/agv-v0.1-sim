#!/usr/bin/env python3
"""Analyze V0.2 navigation JSONL trace — timeline milestones + root-cause classification.

Usage:
  python scripts/_analyze_navigation_v02_trace.py trace.jsonl
  python scripts/_analyze_navigation_v02_trace.py logs/navigation_v02/*.jsonl
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

MILESTONES = (
    "FIRST_OBSTACLE_DETECTION",
    "FIRST_PREVIEW",
    "FIRST_PROBE",
    "FIRST_COMMIT",
    "FIRST_CLEARANCE_DROP",
    "FIRST_MPPI_FAILURE",
    "FIRST_SAFE_VX_ZERO",
    "FIRST_SAFETY_VETO",
    "FIRST_RECOVERY",
    "FIRST_REPROBE",
    "FIRST_FAILED",
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


def _first(rows: List[dict], pred) -> Optional[Tuple[int, dict]]:
    for i, r in enumerate(rows):
        if r.get("error"):
            continue
        if pred(r):
            return i, r
    return None


def _find_milestones(rows: List[dict]) -> Dict[str, Optional[dict]]:
    ms: Dict[str, Optional[dict]] = {k: None for k in MILESTONES}

    def set_m(name: str, row: dict) -> None:
        if ms[name] is None:
            ms[name] = {"ts": row.get("ts"), "seq": row.get("seq"), "scene": row.get("scene"), "row": _compact(row)}

    for r in rows:
        if r.get("error"):
            continue
        fn = _f(r.get("front_near") or r.get("obstacle_distance"), 99.0)
        if fn < 5.0:
            set_m("FIRST_OBSTACLE_DETECTION", r)
        if r.get("future_collision") or (_f(r.get("first_collision_m"), 99) < 8.0):
            set_m("FIRST_PREVIEW", r)
        ap = str(r.get("avoidance_phase") or "").upper()
        if "PROBE" in ap or r.get("probe_confidence"):
            set_m("FIRST_PROBE", r)
        cs = str(r.get("committed_side") or "").upper()
        if cs in ("LEFT", "RIGHT") or "COMMIT" in ap:
            set_m("FIRST_COMMIT", r)
        fp = r.get("footprint_clearance_m")
        if fp is not None and _f(fp, 99) < 0.55:
            set_m("FIRST_CLEARANCE_DROP", r)
        if str(r.get("mppi_failure_reason") or "") == "MPPI_NO_FEASIBLE_TRAJECTORY":
            set_m("FIRST_MPPI_FAILURE", r)
        if _f(r.get("valid_candidate_count"), -1) == 0 and _f(r.get("candidate_count"), 0) > 0:
            set_m("FIRST_MPPI_FAILURE", r)
        if abs(_f(r.get("safe_vx"))) < 0.02 and abs(_f(r.get("mppi_vx"))) > 0.02:
            set_m("FIRST_SAFE_VX_ZERO", r)
        svr = str(r.get("safe_vx_reason") or "NORMAL")
        if svr not in ("NORMAL", "", "RECOVERY_ACTIVE"):
            set_m("FIRST_SAFETY_VETO", r)
        ps = str(r.get("planner_state") or "")
        if ps == "LOCAL_RECOVERY" or str(r.get("recovery_state") or "") not in ("", "NONE"):
            set_m("FIRST_RECOVERY", r)
        if str(r.get("recovery_state") or "") == "REPROBE":
            set_m("FIRST_REPROBE", r)
        if ps == "NAVIGATION_FAILED" or str(r.get("stop_reason") or "") == "NAVIGATION_FAILED":
            set_m("FIRST_FAILED", r)

    return ms


def _compact(r: dict) -> dict:
    keys = (
        "ts", "seq", "scene", "vehicle_pose", "vehicle_vx", "vehicle_omega",
        "avoidance_phase", "committed_side", "planner_state", "recovery_state",
        "safe_vx_reason", "mppi_vx", "approved_vx", "footprint_clearance_m",
        "predicted_min_clearance_m", "valid_candidate_count", "collision_rejected_count",
        "corridor_side", "trajectory_omega_sign", "heading",
    )
    return {k: r.get(k) for k in keys if k in r}


def _classify_case(rows: List[dict]) -> List[str]:
    cases = []
    for r in rows:
        if r.get("error"):
            continue
        vc = int(_f(r.get("valid_candidate_count"), -1))
        cc = int(_f(r.get("collision_rejected_count"), 0))
        mppi_vx = _f(r.get("mppi_vx"))
        approved = _f(r.get("approved_vx"))
        cand = int(_f(r.get("candidate_count"), 0))
        if vc == 0 and cc > 0:
            cases.append("CASE_A_MPPI_FEASIBILITY_FAILURE")
        if mppi_vx > 0.02 and abs(approved) < 0.02:
            cases.append(f"CASE_B_SAFETY_VETO({r.get('safe_vx_reason')})")
        if cand > 0 and vc > 0 and r.get("selected_candidate") is None:
            cases.append("CASE_C_SELECTION_FAILURE")
        ps = str(r.get("planner_state") or "")
        if ps in ("LOCAL_PLAN_INFEASIBLE", "LOCAL_RECOVERY", "SAFE_STOP") and abs(_f(r.get("mppi_vx"))) < 0.01:
            cases.append("CASE_D_EARLY_FSM_STOP")
    return sorted(set(cases))


def _heading_footprint_divergence(rows: List[dict]) -> dict:
    prev_h = None
    prev_c = None
    series: List[dict] = []
    diverged = False
    for r in rows:
        if r.get("error"):
            continue
        h = r.get("heading")
        c = r.get("footprint_clearance_m")
        if h is None or c is None:
            continue
        hf = _f(h)
        cf = _f(c)
        if prev_h is not None and prev_c is not None:
            dh = abs(hf - prev_h)
            dc = cf - prev_c
            series.append({"ts": r.get("ts"), "heading": hf, "clearance": cf, "d_heading": dh, "d_clearance": dc})
            if dh > 0.005 and dc < -0.01:
                diverged = True
        prev_h, prev_c = hf, cf
    return {
        "HEADING_FOOTPRINT_DIVERGENCE": diverged,
        "samples": series[-12:],
    }


def _corridor_constraint(rows: List[dict]) -> dict:
    violations = []
    for r in rows:
        if r.get("error"):
            continue
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
    for r in rows:
        if r.get("error"):
            continue
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
                    "behavior_state": r.get("behavior_state"),
                    "committed_side": r.get("committed_side"),
                    "planner_state": r.get("planner_state"),
                    "vx": r.get("vehicle_vx"),
                    "omega": r.get("vehicle_omega"),
                    "safe_vx_reason": r.get("safe_vx_reason"),
                }
    return best


def _braking_analysis(rows: List[dict]) -> dict:
    status = rows[-1].get("braking_calibration_status") if rows else "CALIBRATION_REQUIRED"
    if status == "CALIBRATION_REQUIRED" or all(r.get("braking_calibration_status") == "CALIBRATION_REQUIRED" for r in rows if not r.get("error")):
        return {"status": "BRAKING_CALIBRATION_REQUIRED", "verdict": None}
    return {"status": "BRAKING_DATA_PRESENT", "verdict": "ANALYZE_MANUALLY"}


def _transitions_summary(rows: List[dict]) -> List[dict]:
    out = []
    for r in rows:
        for t in r.get("transitions") or []:
            out.append(t)
    return out


def _root_cause_verdict(rows: List[dict], cases: List[str], corridor: dict, divergence: dict) -> str:
    if not rows or all(r.get("error") for r in rows):
        return "UNPROVEN"
    if corridor.get("violations"):
        return "LIKELY"
    if divergence.get("HEADING_FOOTPRINT_DIVERGENCE"):
        return "LIKELY"
    if "CASE_A_MPPI_FEASIBILITY_FAILURE" in cases:
        return "LIKELY"
    if any("CASE_B_SAFETY_VETO" in c for c in cases):
        return "LIKELY"
    if _first(rows, lambda r: str(r.get("planner_state")) == "NAVIGATION_FAILED"):
        return "CONFIRMED" if _first(rows, lambda r: str(r.get("recovery_state")) not in ("", "NONE")) else "LIKELY"
    if len(rows) > 5:
        return "PARTIAL"
    return "UNPROVEN"


def analyze(path: Path) -> dict:
    rows = _load_rows(path)
    milestones = _find_milestones(rows)
    cases = _classify_case(rows)
    divergence = _heading_footprint_divergence(rows)
    corridor = _corridor_constraint(rows)
    min_ev = _min_clearance_event(rows)
    braking = _braking_analysis(rows)
    transitions = _transitions_summary(rows)
    verdict = _root_cause_verdict(rows, cases, corridor, divergence)
    return {
        "file": str(path),
        "sample_count": len(rows),
        "milestones": milestones,
        "cases": cases,
        "heading_footprint": divergence,
        "execution_corridor": corridor,
        "minimum_clearance_event": min_ev,
        "braking": braking,
        "transitions_count": len(transitions),
        "transitions": transitions[:40],
        "ROOT_CAUSE": verdict,
    }


def _print_report(report: dict) -> None:
    print(f"\n=== Analyze: {report['file']} ({report['sample_count']} samples) ===")
    print("\nMilestones:")
    for k in MILESTONES:
        v = report["milestones"].get(k)
        if v:
            print(f"  {k}: ts={v.get('ts')} seq={v.get('seq')} scene={v.get('scene')}")
        else:
            print(f"  {k}: —")
    print(f"\nCases: {', '.join(report['cases']) or 'NONE'}")
    print(f"HEADING_FOOTPRINT_DIVERGENCE: {report['heading_footprint'].get('HEADING_FOOTPRINT_DIVERGENCE')}")
    ec = report["execution_corridor"]
    print(f"ExecutionCorridor constraint: {ec.get('result')} violations={len(ec.get('violations') or [])}")
    mce = report.get("minimum_clearance_event")
    if mce:
        print(f"\nMinimum Clearance Event:")
        for k, v in mce.items():
            print(f"  {k}: {v}")
    print(f"\nBraking: {report['braking']}")
    print(f"Transitions recorded: {report['transitions_count']}")
    print(f"\nROOT CAUSE: {report['ROOT_CAUSE']}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="JSONL file(s) or glob")
    ap.add_argument("--json", action="store_true", help="Print JSON report")
    args = ap.parse_args()

    files: List[Path] = []
    for p in args.paths:
        if "*" in p:
            files.extend(Path(x) for x in glob.glob(p))
        else:
            files.append(Path(p))
    if not files:
        print("No trace files found", file=sys.stderr)
        return 1

    reports = [analyze(f) for f in files]
    if args.json:
        print(json.dumps(reports, indent=2, ensure_ascii=False, default=str))
    else:
        for r in reports:
            _print_report(r)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
