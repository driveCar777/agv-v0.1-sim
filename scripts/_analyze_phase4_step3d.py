#!/usr/bin/env python3
"""Analyze STEP3D mid_inject / probe LIVE traces."""
from __future__ import annotations

import glob
import json
import os
import sys


def analyze(path: str) -> None:
    print("====", os.path.basename(path))
    samples = []
    summary = None
    switches = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            o = json.loads(line)
            t = o.get("type")
            if t == "sample":
                samples.append(o)
            elif t == "summary":
                summary = o
            elif t == "side_switch":
                switches.append(o)
    if summary:
        print("summary", {k: summary[k] for k in summary if k != "type"})
    print("switches", len(switches))
    rights = [
        s
        for s in samples
        if "RIGHT" in str(s.get("maneuver_mode") or "")
        or s.get("selector_selected") == "RIGHT"
    ]
    print("RIGHT samples", len(rights))
    # probe status timeline
    prev = None
    n_impl = 0
    split = 0
    for s in samples:
        pr = s.get("probe") or {}
        if not isinstance(pr, dict):
            continue
        if pr.get("implemented"):
            n_impl += 1
        L = (pr.get("left") or {}) if isinstance(pr.get("left"), dict) else {}
        R = (pr.get("right") or {}) if isinstance(pr.get("right"), dict) else {}
        F = (pr.get("forward") or {}) if isinstance(pr.get("forward"), dict) else {}
        B = (pr.get("backward") or {}) if isinstance(pr.get("backward"), dict) else {}
        key = (
            s.get("maneuver_mode"),
            s.get("selector_reason"),
            L.get("status"),
            R.get("status"),
            F.get("status"),
            B.get("status"),
            (s.get("commitment") or {}).get("phase") if isinstance(s.get("commitment"), dict) else None,
        )
        if L.get("status") == "INVALID" and R.get("status") == "VALID":
            split += 1
        if key != prev:
            print(
                f"  t={s.get('t'):.3f} mode={key[0]} sel={key[1]} "
                f"P[L={key[2]} R={key[3]} F={key[4]} B={key[5]}] commit={key[6]}"
            )
            prev = key
    print("probe_implemented_samples", n_impl, "/", len(samples), "split_Linv_Rval", split)
    auth = sum(
        1
        for s in samples
        if isinstance(s.get("side_switch"), dict) and s["side_switch"].get("authorized") is True
    )
    print("authorized_true", auth)


def main() -> int:
    base = r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace"
    paths = sys.argv[1:]
    if not paths:
        paths = sorted(glob.glob(os.path.join(base, "mid_inject_178689*.jsonl")))[-2:]
        paths += sorted(glob.glob(os.path.join(base, "step3d_*.jsonl")))[-3:]
    for p in paths:
        if os.path.isfile(p):
            analyze(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
