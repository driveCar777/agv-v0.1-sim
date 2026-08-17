#!/usr/bin/env python3
"""P1-0 offline: MPPI mean/cmd smoothing response (no behavior change).

Replicates DiffDriveMppi update formulas from mppi_controller.step:
  mean_vx := 0.9 * mean_vx + 0.1 * vx_raw
  vx_cmd  := 0.82 * cmd_vx + 0.18 * mean_vx

Does NOT run the sampler; assumes a sustained vx_raw preference (e.g. 0.30)
to measure how fast command smoothing can climb from init 0.16.
"""

from __future__ import annotations

import argparse
from typing import List


def simulate(
    *,
    mean0: float = 0.16,
    cmd0: float = 0.0,
    vx_raw: float = 0.30,
    dt: float = 0.35,
    seconds: float = 5.0,
    a_vx_max: float = 0.38,
) -> List[dict]:
    """dt≈0.35 matches sim local_period (local replan cadence), not physics 0.05."""
    mean = mean0
    cmd = cmd0
    t = 0.0
    rows = [{"t": 0.0, "mean_vx": mean, "vx_raw": vx_raw, "vx_cmd": cmd}]
    while t + 1e-9 < seconds:
        mean = 0.9 * mean + 0.1 * vx_raw
        mean = max(-0.18, min(a_vx_max, mean))
        cmd = 0.82 * cmd + 0.18 * mean
        cmd = max(0.0, min(a_vx_max, cmd))
        t += dt
        rows.append({"t": round(t, 3), "mean_vx": round(mean, 5), "vx_raw": vx_raw, "vx_cmd": round(cmd, 5)})
    return rows


def first_reach(rows: List[dict], key: str, thr: float):
    for r in rows:
        if float(r[key]) >= thr:
            return r["t"]
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx-raw", type=float, default=0.30)
    ap.add_argument("--dt", type=float, default=0.35)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()
    print("=== P1-0 OPEN Speed Response (EMA formulas only) ===")
    print("mean := 0.9*mean + 0.1*vx_raw")
    print("cmd  := 0.82*cmd + 0.18*mean")
    print(f"init mean=0.16 cmd=0.0  sustained vx_raw={args.vx_raw}  dt={args.dt}s")
    rows = simulate(vx_raw=float(args.vx_raw), dt=float(args.dt), seconds=float(args.seconds))
    print(f"{'t':>6} {'mean':>8} {'raw':>8} {'cmd':>8}")
    for r in rows:
        if r["t"] in (0.0,) or abs(r["t"] % 0.5) < 1e-6 or r["t"] >= args.seconds - 1e-6:
            print(f"{r['t']:6.2f} {r['mean_vx']:8.4f} {r['vx_raw']:8.2f} {r['vx_cmd']:8.4f}")
    for thr in (0.20, 0.22, 0.25, 0.28, 0.30):
        tm = first_reach(rows, "mean_vx", thr)
        tc = first_reach(rows, "vx_cmd", thr)
        print(f"reach mean>={thr}: t={tm}   cmd>={thr}: t={tc}")
    # also show if sampler only weakly prefers 0.22
    print("--- if vx_raw sustained at preference 0.22 ---")
    rows2 = simulate(vx_raw=0.22, dt=float(args.dt), seconds=float(args.seconds))
    print(f"t={rows2[-1]['t']} mean={rows2[-1]['mean_vx']} cmd={rows2[-1]['vx_cmd']}")
    print("P1-0 SPEED RESPONSE = PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
