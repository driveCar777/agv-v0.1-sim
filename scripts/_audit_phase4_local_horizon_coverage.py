#!/usr/bin/env python3
"""P1-0 offline: Local/MPPI horizon × speed coverage matrix (no behavior change)."""

from __future__ import annotations

import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.local_maneuver import NOMINAL_VX, ROLLOUT_DT, ROLLOUT_STEPS, SIDE_VX, selector_horizon_s  # noqa: E402
from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_global_preview import GLOBAL_PREVIEW_NORMAL_M  # noqa: E402


def main() -> int:
    mppi = DiffDriveMppi()
    local_h = selector_horizon_s()
    mppi_h = mppi.time_steps * mppi.model_dt
    print("=== P1-0 Local / MPPI Horizon Coverage Matrix ===")
    print(f"Local:  ROLLOUT_STEPS={ROLLOUT_STEPS} DT={ROLLOUT_DT} → horizon_s={local_h}")
    print(f"MPPI:   time_steps={mppi.time_steps} model_dt={mppi.model_dt} → horizon_s={mppi_h}")
    print(f"NOMINAL_VX={NOMINAL_VX} SIDE_VX={SIDE_VX} max_vx={DEFAULT_GEOM.max_vx}")
    print(f"Global preview normal≈{GLOBAL_PREVIEW_NORMAL_M}m")
    horizons = (1.0, 1.5, 1.6, 2.0, 2.5, 3.0, 4.0)
    speeds = (0.10, 0.16, 0.18, 0.22, 0.30, 0.40)
    header = "vx\\h " + " ".join(f"{h:6.1f}s" for h in horizons)
    print(header)
    for vx in speeds:
        cells = " ".join(f"{vx * h:6.2f}m" for h in horizons)
        mark = ""
        if abs(vx - NOMINAL_VX) < 1e-9:
            mark = "  <- NOMINAL_VX"
        elif abs(vx - SIDE_VX) < 1e-9:
            mark = "  <- SIDE_VX"
        elif abs(vx - 0.16) < 1e-9:
            mark = "  <- MPPI mean init"
        print(f"{vx:4.2f} {cells}{mark}")
    print("--- current design points ---")
    print(f"Local @ NOMINAL: {NOMINAL_VX}*{local_h} = {NOMINAL_VX * local_h:.3f}m")
    print(f"Local @ SIDE:    {SIDE_VX}*{local_h} = {SIDE_VX * local_h:.3f}m")
    print(f"MPPI  @ 0.16:    {0.16 * mppi_h:.3f}m")
    print(f"MPPI  @ 0.22:    {0.22 * mppi_h:.3f}m")
    print(f"coverage vs Global {GLOBAL_PREVIEW_NORMAL_M}m @ Local NOMINAL: {(NOMINAL_VX * local_h) / GLOBAL_PREVIEW_NORMAL_M:.3f}")
    print(f"coverage if Local 3.0s @ 0.22: {(0.22 * 3.0) / GLOBAL_PREVIEW_NORMAL_M:.3f}")
    print(f"coverage if Local 3.0s @ 0.30: {(0.30 * 3.0) / GLOBAL_PREVIEW_NORMAL_M:.3f}")
    print("P1-0 HORIZON COVERAGE = PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
