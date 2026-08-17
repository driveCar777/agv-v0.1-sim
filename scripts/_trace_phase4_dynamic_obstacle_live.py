#!/usr/bin/env python3
"""P0-D.1 LIVE — Dynamic obstacle wait/resume forensics (Scene DYN-1)."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def api(base: str, path: str) -> dict:
    with urllib.request.urlopen(base.rstrip("/") + path, timeout=8.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def sample(base: str, tag: str) -> dict:
    st = api(base, "/api/state")
    op = api(base, "/api/nav/obstacle-preview")
    diag = {}
    try:
        diag = api(base, "/api/logs/diagnostics?window_s=10")
    except Exception:
        pass
    nav = st.get("nav") or {}
    dbg = st.get("debug") or {}
    dr = dbg.get("dynamic_resume") or op.get("dynamic_resume") or {}
    return {
        "ts": time.time(),
        "tag": tag,
        "dynamic_short": st.get("dynamic_short"),
        "dynamic_long": st.get("dynamic_long"),
        "policy_state": nav.get("policy_state") or (dbg.get("policy") or {}).get("state"),
        "policy_reason": (dbg.get("policy") or {}).get("reason"),
        "maneuver_mode": nav.get("maneuver_mode") or (dbg.get("maneuver") or {}).get("mode"),
        "requested_vx": st.get("cmd_vx"),
        "safe_vx": st.get("safe_vx"),
        "state_vx": st.get("vx"),
        "front_near": st.get("front_near"),
        "dynamic_state": dr.get("dynamic_state") or op.get("dynamic_state"),
        "resume_allowed": dr.get("resume_allowed"),
        "resume_block_reason": dr.get("resume_block_reason") or op.get("resume_block_reason"),
        "avoidance_phase": op.get("avoidance_phase") or dbg.get("avoidance_phase"),
        "summary_resume_block": (diag.get("summary") or {}).get("resume_block_reason"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "dynamic_obstacle"))
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    jsonl = os.path.join(args.out_dir, f"dynamic_obstacle_{stamp}.jsonl")
    rows = []
    n = max(1, int(args.seconds * args.hz))
    print("Scene DYN-1: drive → static obstacle → reorient → moving obstacle → stop → clear → observe resume")
    for i in range(n):
        try:
            rows.append(sample(args.base, f"DYN1_{i}"))
        except (urllib.error.URLError, json.JSONDecodeError) as e:
            rows.append({"ts": time.time(), "tag": f"DYN1_{i}", "error": str(e)})
        time.sleep(1.0 / args.hz)
    with open(jsonl, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    resume = sum(1 for r in rows if r.get("resume_allowed"))
    blocked = sum(1 for r in rows if r.get("dynamic_state") == "BLOCKED")
    print(f"Wrote {len(rows)} samples → {jsonl}")
    print(f"dynamic_blocked={blocked} resume_allowed={resume}")
    print("RESULT: PARTIAL (manual moving obstacle required)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
