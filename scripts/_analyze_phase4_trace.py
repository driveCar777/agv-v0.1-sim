#!/usr/bin/env python3
"""Analyze Phase4 side-switch JSONL traces (read-only)."""
from __future__ import annotations

import glob
import json
import sys


def main() -> int:
    paths = sorted(glob.glob(r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace\left_wide*.jsonl"))
    if not paths:
        print("no traces")
        return 1
    path = paths[-1]
    if len(sys.argv) > 1:
        path = sys.argv[1]
    print("FILE", path)
    samples = []
    switches = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            if o.get("type") == "sample":
                samples.append(o)
            elif o.get("type") == "side_switch":
                switches.append(o)
            elif o.get("type") == "summary":
                print("SUMMARY", {k: o[k] for k in o if k != "type"})
    print("N", len(samples), "switches", len(switches))
    prev = None
    print("--- compressed timeline ---")
    for s in samples:
        key = (
            s.get("maneuver_mode"),
            s.get("policy_state"),
            s.get("selector_selected"),
            s.get("stop_reason"),
            s.get("allow_side_compare"),
            s.get("left_feasible"),
            s.get("right_feasible"),
        )
        if key != prev:
            print(
                f"t={s['t']:.3f} mode={s.get('maneuver_mode')} pol={s.get('policy_state')} "
                f"sel={s.get('selector_selected')} allow={s.get('allow_side_compare')} "
                f"L={s.get('left_cost')} R={s.get('right_cost')} Lf={s.get('left_feasible')} Rf={s.get('right_feasible')} "
                f"reason={s.get('selector_reason')} stop={s.get('stop_reason')} front={s.get('front_near')} "
                f"mppi_w={s.get('mppi_w')} safe_vx={s.get('safe_vx')} state_vx={s.get('state_vx')} avoid={s.get('avoid_side')}"
            )
            prev = key
    left_t = None
    for s in samples:
        if s.get("maneuver_mode") == "LOCAL_LEFT" and left_t is None:
            left_t = s["t"]
            print("LEFT_DECISION_TIME", left_t)
            print("LEFT_DETAIL", {k: s.get(k) for k in [
                "left_cost", "right_cost", "left_feasible", "right_feasible", "selector_reason",
                "allow_side_compare", "mppi_w", "safe_vx", "safe_w", "stop_reason", "front_near", "avoid_side",
                "left_clr", "right_clr", "left_capture", "right_capture", "maneuver_reason",
            ]})
        if s.get("maneuver_mode") == "LOCAL_RIGHT":
            print("RIGHT_SEEN", s["t"], s.get("selector_reason"), s.get("left_cost"), s.get("right_cost"))
    print("--- LOCAL_LEFT cost stream (every ~0.2s) ---")
    last = -1.0
    for s in samples:
        if s.get("maneuver_mode") != "LOCAL_LEFT":
            continue
        if s["t"] - last < 0.18:
            continue
        last = s["t"]
        print(
            f"t={s['t']:.3f} L={s.get('left_cost')} R={s.get('right_cost')} "
            f"Lf={s.get('left_feasible')} Rf={s.get('right_feasible')} reason={s.get('selector_reason')} "
            f"allow={s.get('allow_side_compare')} w={s.get('mppi_w')} svx={s.get('safe_vx')} stop={s.get('stop_reason')}"
        )
    print("--- post-LEFT modes ---")
    after = False
    for s in samples:
        if s.get("maneuver_mode") == "LOCAL_LEFT":
            after = True
        if after and s.get("maneuver_mode") not in (None, "LOCAL_LEFT"):
            print(f"t={s['t']:.3f} -> {s.get('maneuver_mode')} pol={s.get('policy_state')} stop={s.get('stop_reason')} reason={s.get('maneuver_reason') or s.get('selector_reason')}")
            if s.get("maneuver_mode") not in ("LOCAL_LEFT",):
                # print a few transitions only
                pass
    # unique transitions after first left
    seen = set()
    after = False
    for s in samples:
        if s.get("maneuver_mode") == "LOCAL_LEFT":
            after = True
        if not after:
            continue
        m = s.get("maneuver_mode")
        if m and m not in seen:
            seen.add(m)
            print("FIRST_SEEN_MODE", m, "at", s["t"], "stop", s.get("stop_reason"), "svx", s.get("safe_vx"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
