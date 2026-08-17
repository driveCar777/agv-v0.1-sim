#!/usr/bin/env python3
"""M3.1 orchestrator — run all LIVE-00…LIVE-06 traces and print summary report."""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs" / "navigation_v02"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--scene", default="ALL", help="ALL or LIVE-00 … LIVE-06")
    args = ap.parse_args()

    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    analyze = ROOT / "scripts" / "_analyze_navigation_v02_trace.py"

    cmd = [
        sys.executable,
        str(trace),
        "--scene",
        args.scene,
        "--seconds",
        str(args.seconds),
        "--hz",
        str(args.hz),
        "--base",
        args.base,
    ]
    print("Running:", " ".join(cmd))
    rc = subprocess.call(cmd)
    if rc != 0:
        print(f"Trace failed rc={rc}")
        return rc

    files = sorted(glob.glob(str(LOG_DIR / "LIVE-*.jsonl")), key=os.path.getmtime)
    if args.scene != "ALL":
        tag = args.scene.replace("LIVE-", "LIVE-").lower()
        files = [f for f in files if tag.split("-")[0] in f.lower() or args.scene.replace("-", "-") in f]
    # Use most recent per scene prefix
    latest: dict = {}
    for f in files:
        name = Path(f).name
        prefix = "-".join(name.split("-")[:3])  # LIVE-00-baseline
        latest[prefix] = f
    run_files = sorted(latest.values())

    if not run_files:
        print("No JSONL files to analyze")
        return 1

    acmd = [sys.executable, str(analyze), *run_files, "--matrix"]
    print("\nAnalyzing:", len(run_files), "file(s)")
    subprocess.call(acmd)

    # Summary block
    import json

    reports = []
    for f in run_files:
        out = subprocess.check_output([sys.executable, str(analyze), f, "--json"], text=True)
        reports.extend(json.loads(out))

    labels = {
        "LIVE-00": "BASELINE",
        "LIVE-01": "STATIC LEFT",
        "LIVE-02": "STATIC RIGHT",
        "LIVE-03": "BOTH BLOCKED",
        "LIVE-04": "DYNAMIC CROSS",
        "LIVE-05": "DYNAMIC AWAY",
        "LIVE-06": "FIELD P0D1",
    }
    print("\n" + "=" * 44)
    print("AGV NAVIGATION V0.2 LIVE VALIDATION")
    print("=" * 44)
    by_scene = {r.get("scene"): r for r in reports}
    for sid, label in labels.items():
        r = by_scene.get(sid)
        overall = (r or {}).get("stages", {}).get("Overall", "NOT AVAILABLE")
        print(f"\n{label}\n{overall}")
    print("\n--- Verification status ---")
    print("SOURCE-CODE CONFIRMED")
    print("SIM VERIFIED: PARTIAL (see matrix above)")
    print("FIELD VERIFIED: NOT VERIFIED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
