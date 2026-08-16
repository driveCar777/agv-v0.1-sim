#!/usr/bin/env python3
"""STEP 3D LIVE probe observer for http://127.0.0.1:19999 (read-only).

Does not change navigation. Records probe + commitment + FSM each tick.
Optionally drives mid_inject via existing side-switch tracer scenarios by
delegating scene setup, or observe-only if --observe-only.
"""

from __future__ import annotations

import argparse
import json
import os
import time
import urllib.request
from typing import Any, Dict, List, Optional

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 8.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def debug_root() -> Dict[str, Any]:
    wrap = api("GET", "/api/nav/debug")
    return wrap.get("debug") if isinstance(wrap.get("debug"), dict) else wrap


def sample_row(t0: float, d: Dict[str, Any]) -> Dict[str, Any]:
    pr = d.get("probe") or (d.get("phase4") or {}).get("probe") or {}
    c = d.get("commitment") or (d.get("phase4") or {}).get("commitment") or {}
    sw = d.get("side_switch") or (d.get("phase4") or {}).get("side_switch") or {}
    man = d.get("maneuver") or {}
    pol = d.get("nav_policy") or {}
    perf = d.get("performance") or (d.get("phase4") or {}).get("performance") or {}
    return {
        "type": "sample",
        "t": round(time.time() - t0, 4),
        "policy_state": pol.get("state"),
        "maneuver_mode": man.get("mode"),
        "selector": (d.get("local_maneuver") or {}).get("decision"),
        "selector_reason": (d.get("local_maneuver") or {}).get("reason"),
        "allow_side_compare": (pol.get("decision") or {}).get("allow_side_compare")
        if isinstance(pol.get("decision"), dict)
        else pol.get("allow_side_compare"),
        "commitment": c,
        "side_switch": {
            "authorized": sw.get("authorized"),
            "authorization_status": sw.get("authorization_status"),
        },
        "probe": {
            "implemented": pr.get("implemented"),
            "status": pr.get("status"),
            "bundle_ms": pr.get("bundle_ms"),
            "forward": pr.get("forward"),
            "backward": pr.get("backward"),
            "left": pr.get("left"),
            "right": pr.get("right"),
            "turn_in_place": pr.get("turn_in_place"),
            "no_valid_escape": pr.get("no_valid_escape"),
        },
        "performance": {
            "probe_bundle_ms": perf.get("probe_bundle_ms") or pr.get("bundle_ms"),
            "probe_forward_ms": perf.get("probe_forward_ms") or pr.get("forward_ms"),
            "probe_backward_ms": perf.get("probe_backward_ms") or pr.get("backward_ms"),
            "probe_left_ms": perf.get("probe_left_ms") or pr.get("left_ms"),
            "probe_right_ms": perf.get("probe_right_ms") or pr.get("right_ms"),
            "probe_turn_ms": perf.get("probe_turn_ms") or pr.get("turn_ms"),
        },
        "stop_reason": (d.get("diagnostics") or {}).get("stop_reason")
        or (d.get("safety") or {}).get("stop_reason"),
    }


def percentile(vals: List[float], p: float) -> Optional[float]:
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(s) - 1)
    if f == c:
        return s[f]
    return s[f] + (s[c] - s[f]) * (k - f)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=8.0)
    ap.add_argument("--out", default=r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace")
    ap.add_argument("--tag", default="observe")
    ap.add_argument(
        "--via-mid-inject",
        action="store_true",
        help="Also run mid_inject scenario via side-switch tracer first",
    )
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    if args.via_mid_inject:
        # Reuse existing scenario driver (writes its own jsonl); we observe after inject starts
        import subprocess
        import sys

        script = os.path.join(os.path.dirname(__file__), "_trace_phase4_side_switch.py")
        subprocess.Popen(
            [
                sys.executable,
                script,
                "--scenario",
                "mid_inject",
                "--duration",
                str(max(16.0, args.duration)),
                "--out",
                args.out,
            ],
            cwd=os.path.dirname(os.path.dirname(__file__)),
        )
        time.sleep(2.5)

    t0 = time.time()
    rows: List[Dict[str, Any]] = []
    dt = 1.0 / max(1.0, args.hz)
    print("CONNECTED probe-live")
    while time.time() - t0 < args.duration:
        try:
            d = debug_root()
            rows.append(sample_row(t0, d))
        except Exception as exc:
            rows.append({"type": "error", "t": round(time.time() - t0, 4), "error": str(exc)})
        time.sleep(dt)

    bundle_ms = [
        float(r["performance"]["probe_bundle_ms"])
        for r in rows
        if r.get("type") == "sample" and r.get("performance", {}).get("probe_bundle_ms") is not None
    ]
    # Evidence-only checks
    switches = 0
    left_invalid_right_valid = 0
    authorized_true = 0
    for r in rows:
        if r.get("type") != "sample":
            continue
        pr = r.get("probe") or {}
        L = (pr.get("left") or {}).get("status")
        R = (pr.get("right") or {}).get("status")
        if L == "INVALID" and R == "VALID":
            left_invalid_right_valid += 1
            mode = r.get("maneuver_mode") or ""
            if "RIGHT" in mode:
                switches += 1
        if (r.get("side_switch") or {}).get("authorized") is True:
            authorized_true += 1

    summary = {
        "type": "summary",
        "tag": args.tag,
        "samples": len(rows),
        "probe_bundle_ms_p50": percentile(bundle_ms, 50),
        "probe_bundle_ms_p95": percentile(bundle_ms, 95),
        "left_invalid_right_valid_ticks": left_invalid_right_valid,
        "fsm_right_while_probe_split": switches,
        "side_switch_authorized_true": authorized_true,
        "evidence_only_ok": switches == 0 and authorized_true == 0,
    }
    ts = int(time.time())
    path = os.path.join(args.out, f"step3d_{args.tag}_{ts}.jsonl")
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY", summary)
    print("WROTE", path)
    return 0 if summary["evidence_only_ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
