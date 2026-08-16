#!/usr/bin/env python3
"""Analyze STEP3C mid_inject JSONL traces."""
from __future__ import annotations

import glob
import json
import os
import sys


def analyze(path: str) -> None:
    print("====", os.path.basename(path))
    samples = []
    switches = []
    summary = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            t = o.get("type")
            if t == "sample":
                samples.append(o)
            elif t == "side_switch":
                switches.append(o)
            elif t == "summary":
                summary = o
    if summary:
        print("summary", {k: summary[k] for k in summary if k != "type"})
    print("switches", len(switches))
    prev = None
    for s in samples:
        key = (
            s.get("maneuver_mode"),
            s.get("selector_selected"),
            s.get("selector_reason"),
            s.get("allow_side_compare"),
        )
        if key != prev:
            print(
                f"  t={s.get('t'):.3f} mode={key[0]} sel={key[1]} reason={key[2]} allow={key[3]}"
            )
            prev = key
    rights = [
        s
        for s in samples
        if "RIGHT" in str(s.get("maneuver_mode") or "")
        or s.get("selector_selected") == "RIGHT"
    ]
    print("RIGHT samples", len(rights))
    # commitment from nested debug if tracer stored it
    shown = 0
    for s in samples:
        c = s.get("commitment") or {}
        if isinstance(c, dict) and (c.get("active") or c.get("implemented")):
            print(
                "commit",
                {
                    "t": s.get("t"),
                    "active": c.get("active"),
                    "side": c.get("side"),
                    "phase": c.get("phase"),
                    "hard_fail": c.get("hard_fail"),
                    "fail": c.get("failure_reason"),
                    "auth": c.get("authorization_status"),
                    "sel_reason": s.get("selector_reason"),
                    "mode": s.get("maneuver_mode"),
                },
            )
            shown += 1
            if shown >= 6:
                break
    if shown == 0 and samples:
        print("sample top keys", sorted(samples[min(20, len(samples) - 1)].keys())[:40])


def main() -> int:
    base = r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace"
    paths = sorted(glob.glob(os.path.join(base, "mid_inject*.jsonl")))
    if len(sys.argv) > 1:
        paths = [sys.argv[1]]
    else:
        paths = paths[-4:]
    for p in paths:
        analyze(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
