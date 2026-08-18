#!/usr/bin/env python3
"""M3.8 trajectory integrity audit — live API + optional JSONL."""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws" / "src" / "agv_bridge"
sys.path.insert(0, str(BRIDGE))

from agv_bridge.nav_trajectory_integrity import (  # noqa: E402
    anchor_error_m,
    audit_snapshot_trajectory,
    enrich_trajectory_metadata,
    global_path_fingerprint,
)


def _get(base: str, path: str) -> dict:
    with urllib.request.urlopen(f"{base}{path}", timeout=8) as resp:
        return json.loads(resp.read().decode("utf-8"))


def audit_live(base: str, samples: int, interval_s: float) -> Dict[str, Any]:
    rows: List[dict] = []
    worst = "PASS"
    rank = {"PASS": 0, "WARN": 1, "UNVERIFIED": 2, "FAIL": 3}
    for i in range(max(1, samples)):
        snap = _get(base, "/api/state?lite=1")
        rep = audit_snapshot_trajectory(snap)
        rep["sample"] = i
        rows.append(rep)
        if rank.get(rep["status"], 0) > rank.get(worst, 0):
            worst = rep["status"]
        if i + 1 < samples:
            time.sleep(max(0.1, interval_s))
    return {"overall": worst, "samples": rows}


def audit_jsonl(path: Path) -> Dict[str, Any]:
    rows_in = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    worst = "PASS"
    rank = {"PASS": 0, "WARN": 1, "UNVERIFIED": 2, "FAIL": 3}
    first_behind = None
    for r in rows_in:
        agv = r.get("vehicle_pose") or {}
        pt = r.get("physical_trajectory") or {}
        if not pt:
            continue
        ae = r.get("anchor_error_m")
        if ae is None:
            ae = anchor_error_m({"x": agv.get("x"), "y": agv.get("y"), "angle": agv.get("yaw")}, pt)
        if ae is not None and r.get("max_anchor_error_m") and ae > float(r["max_anchor_error_m"]):
            if rank["FAIL"] > rank.get(worst, 0):
                worst = "FAIL"
            if first_behind is None and r.get("trajectory_behind_vehicle"):
                first_behind = r.get("seq")
        if r.get("collision"):
            worst = "FAIL"
    return {"overall": worst, "first_trajectory_behind_seq": first_behind, "rows": len(rows_in)}


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.8 trajectory integrity audit")
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--jsonl", default="")
    args = ap.parse_args()

    print("=" * 60)
    print(" M3.8 TRAJECTORY INTEGRITY AUDIT")
    print("=" * 60)

    if args.jsonl:
        rep = audit_jsonl(Path(args.jsonl))
        print(f" JSONL: {args.jsonl}")
        print(f" overall: {rep['overall']}")
        print(f" first_behind_seq: {rep.get('first_trajectory_behind_seq')}")
        return 0 if rep["overall"] in ("PASS", "WARN") else 1

    try:
        rep = audit_live(args.base, args.samples, args.interval)
    except Exception as e:
        print(f" FAIL: cannot reach sim: {e}")
        return 2

    print(f" overall: {rep['overall']}")
    for s in rep["samples"]:
        print(
            f"  [{s['sample']}] {s['status']} display={s['display_source']} "
            f"issues={s.get('issues')} rev={s.get('global_path_revision')}"
        )
        pt = s.get("physical_trajectory") or {}
        integ = pt.get("integrity") or {}
        if integ:
            print(
                f"       anchor={integ.get('anchor_error_m')}m "
                f"max={integ.get('max_anchor_error_m')}m age={integ.get('trajectory_age_ms')}ms "
                f"behind={integ.get('behind_vehicle')}"
            )
    return 0 if rep["overall"] in ("PASS", "WARN") else 1


if __name__ == "__main__":
    sys.exit(main())
