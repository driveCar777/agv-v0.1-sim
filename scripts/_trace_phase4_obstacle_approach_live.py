#!/usr/bin/env python3
"""P0-D LIVE trace — obstacle approach preview (L1–L7).

Requires web sim at --base (default http://127.0.0.1:19999).
"""

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


def api(base: str, method: str, path: str, body: dict | None = None) -> dict:
    url = base.rstrip("/") + path
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method)
    if body is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=8.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def sample(base: str, tag: str) -> dict:
    st = api(base, "GET", "/api/state")
    op = api(base, "GET", "/api/nav/obstacle-preview")
    nav = st.get("nav") or {}
    dbg = st.get("debug") or {}
    row = {
        "ts": time.time(),
        "tag": tag,
        "front_near": st.get("front_near"),
        "state_vx": st.get("vx"),
        "maneuver": nav.get("maneuver_mode") or dbg.get("maneuver", {}).get("mode"),
        "policy_state": nav.get("policy_state"),
        "future_collision": op.get("obstacle_preview", {}).get("future_collision"),
        "first_collision_m": op.get("first_collision_distance_m"),
        "required_avoidance_m": op.get("required_avoidance_distance_m"),
        "approach_active": op.get("obstacle_preview", {}).get("approach_active"),
        "left_valid": op.get("left_valid"),
        "right_valid": op.get("right_valid"),
        "obstacle_pass_state": op.get("obstacle_pass_state"),
        "local_plan_id": op.get("local_plan_id"),
        "selected_kind": (dbg.get("local_plan") or {}).get("selected_kind"),
    }
    return row


def run_scene(base: str, tag: str, seconds: float, hz: float) -> list[dict]:
    rows = []
    n = max(1, int(seconds * hz))
    for i in range(n):
        try:
            rows.append(sample(base, f"{tag}_{i}"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            rows.append({"ts": time.time(), "tag": tag, "error": str(e)})
        time.sleep(1.0 / hz)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "obstacle_approach"))
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    jsonl = os.path.join(args.out_dir, f"obstacle_approach_{stamp}.jsonl")
    csv_path = os.path.join(args.out_dir, f"obstacle_approach_{stamp}.csv")

    scenes = ["L1_open", "L2_obs4m", "L3_obs3m", "L4_left_blk", "L5_right_blk", "L6_corridor", "L7_multi"]
    all_rows: list[dict] = []
    for sc in scenes:
        print(f"--- {sc} ({args.seconds}s) — place obstacles manually if needed ---")
        all_rows.extend(run_scene(args.base, sc, args.seconds, args.hz))

    with open(jsonl, "w", encoding="utf-8") as f:
        for r in all_rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    if all_rows:
        keys = sorted({k for r in all_rows for k in r.keys()})
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(all_rows)

    fc = sum(1 for r in all_rows if r.get("future_collision"))
    aa = sum(1 for r in all_rows if r.get("approach_active"))
    print(f"Wrote {len(all_rows)} samples → {jsonl}")
    print(f"future_collision ticks={fc} approach_active ticks={aa}")
    print("RESULT: PARTIAL (LIVE requires manual obstacle placement)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
