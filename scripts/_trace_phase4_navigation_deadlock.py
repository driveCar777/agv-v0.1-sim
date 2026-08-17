#!/usr/bin/env python3
"""P0-B-0 — Capture last N seconds of nav diagnostics when vehicle stalls.

Does NOT change planning. Requires web sim on :19999.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from typing import Any, Dict, List

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 10.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-s", type=float, default=10.0)
    ap.add_argument("--watch-s", type=float, default=25.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out_dir = args.out or os.path.join("logs", "p0b0_deadlock")
    os.makedirs(out_dir, exist_ok=True)

    # Start a simple outdoor goal so motion can occur
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.4)
        api("POST", "/api/obstacles/clear", {})
        st = api("GET", "/api/state")
        a = st.get("agv") or {}
        x, y, yaw = float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)
        import math

        gx = x + 12.0 * math.cos(yaw)
        gy = y + 12.0 * math.sin(yaw)
        plan = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
        api("POST", "/api/nav/confirm", {})
        print("nav started", plan.get("success") or plan.get("api"))
    except Exception as exc:
        print("WARN setup:", exc)

    t0 = time.time()
    samples: List[Dict[str, Any]] = []
    saw_move = False
    stalled = False
    while time.time() - t0 < args.watch_s:
        try:
            summary = api("GET", "/api/logs/summary")
            st = api("GET", "/api/state")
            nav = st.get("nav") or {}
            agv = st.get("agv") or {}
            vx = float(nav.get("state_vx") if nav.get("state_vx") is not None else agv.get("vx") or 0)
            row = {
                "t": round(time.time() - t0, 3),
                "vx": vx,
                "mode": summary.get("current_mode"),
                "selected": summary.get("current_candidate"),
                "cand": summary.get("candidate_count"),
                "valid": summary.get("valid_candidate_count"),
                "req": summary.get("requested_vx"),
                "safe": summary.get("safe_vx"),
                "owner": summary.get("likely_owner"),
                "deadlock": summary.get("deadlock_suspected"),
                "spin": summary.get("spin_suspected"),
                "starve": summary.get("planning_starvation_suspected"),
            }
            samples.append(row)
            if abs(vx) > 0.05:
                saw_move = True
            if saw_move and abs(vx) < 0.03 and row["t"] > 3.0:
                stalled = True
                print(f"[stall] t={row['t']} owner={row['owner']} selected={row['selected']}")
                break
        except Exception as exc:
            samples.append({"t": round(time.time() - t0, 3), "error": str(exc)})
        time.sleep(0.25)

    diag = {}
    try:
        diag = api("GET", f"/api/logs/diagnostics?window_s={args.window_s}")
    except Exception as exc:
        diag = {"success": False, "error": str(exc)}

    payload = {
        "schema": "p0b0_deadlock_trace_v1",
        "saw_move": saw_move,
        "stalled": stalled,
        "samples": samples,
        "diagnostics": diag,
        "summary": api("GET", "/api/logs/summary") if True else {},
    }
    path = os.path.join(out_dir, f"deadlock_{int(time.time())}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print("WROTE", path)
    print(
        "RESULT",
        "PASS" if (diag.get("success") and (stalled or saw_move)) else "PARTIAL",
        f"events={diag.get('event_count')} cycles={diag.get('cycle_count')}",
    )
    return 0 if diag.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
