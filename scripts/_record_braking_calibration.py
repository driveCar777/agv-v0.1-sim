#!/usr/bin/env python3
"""Record braking calibration trials to JSONL (no invented decel values).

Usage (vehicle or sim):
  python scripts/_record_braking_calibration.py --base http://127.0.0.1:19999 --speed 0.20

Operator marks stop command; script records pose/speed until rest.
"""

from __future__ import annotations

import argparse
import json
import os
import time
import urllib.request
from datetime import datetime

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _get(base: str, path: str) -> dict:
    with urllib.request.urlopen(base.rstrip("/") + path, timeout=5.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--speed", type=float, required=True, help="Target cruise speed before stop (m/s)")
    ap.add_argument("--payload", default="empty", choices=("empty", "loaded"))
    ap.add_argument("--surface", default="unknown")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "logs", "braking_calibration"))
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(args.out_dir, f"braking_{args.payload}_{args.speed:.2f}mps_{stamp}.jsonl")

    meta = {
        "schema": "braking_calibration/1",
        "target_speed_mps": args.speed,
        "payload": args.payload,
        "surface": args.surface,
        "calibration_status": "CALIBRATION_REQUIRED",
        "note": "Post-process: effective_decel = v0^2 / (2 * stop_distance); latency from cmd→vx drop",
    }
    rows = [meta]
    n = max(1, int(args.seconds * args.hz))
    t_cmd = time.time()
    for i in range(n):
        try:
            st = _get(args.base, "/api/state")
            agv = st.get("agv") or {}
            nav = st.get("nav") or {}
            rows.append(
                {
                    "ts": time.time(),
                    "seq": i,
                    "t_since_stop_cmd_s": time.time() - t_cmd,
                    "x": agv.get("x"),
                    "y": agv.get("y"),
                    "vx": agv.get("vx"),
                    "w": agv.get("w"),
                    "state_vx": nav.get("state_vx"),
                    "safe_vx": nav.get("cmd_vx_after_safety"),
                    "stop_reason": nav.get("stop_reason"),
                }
            )
        except Exception as exc:
            rows.append({"ts": time.time(), "seq": i, "error": str(exc)})
        time.sleep(1.0 / args.hz)

    with open(out, "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r, ensure_ascii=False) + "\n")
    print(f"Wrote {len(rows)-1} samples → {out}")
    print("NEXT: measure stop distance manually; compute max_decel_mps2 from multiple trials")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
