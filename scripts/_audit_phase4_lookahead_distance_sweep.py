#!/usr/bin/env python3
"""P1-2-LIVE offline — Pure Pursuit lookahead distance sweep (diagnostic only).

Does NOT change default lookahead_m in production code.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, List, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
sys.path.insert(0, PKG)

from agv_bridge.mppi_controller import pure_pursuit_target, pure_pursuit_w  # noqa: E402
from agv_bridge.nav_geometry import DEFAULT_GEOM

Pt = Tuple[float, float]
LOOKAHEADS = [0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0]
VEHICLE_L = DEFAULT_GEOM.overall_length_m if hasattr(DEFAULT_GEOM, "overall_length_m") else 1.05


def straight_path(length: float = 8.0, step: float = 0.2) -> List[Pt]:
    return [(i * step, 0.0) for i in range(int(length / step) + 1)]


def gentle_turn() -> List[Pt]:
    pts: List[Pt] = []
    for i in range(40):
        s = i * 0.15
        pts.append((s, 0.8 * math.sin(s * 0.35)))
    return pts


def turn_90() -> List[Pt]:
    pts: List[Pt] = [(0.0, 0.0), (2.0, 0.0)]
    for i in range(1, 16):
        a = i * (math.pi / 2) / 15
        pts.append((2.0 + 1.5 * math.sin(a), 1.5 * (1 - math.cos(a))))
    return pts


def bypass_arc() -> List[Pt]:
    pts: List[Pt] = [(0.0, 0.0), (1.0, 0.0), (1.5, 0.5), (2.5, 0.8), (3.5, 0.4), (4.5, 0.0)]
    return pts


def narrow_corridor() -> List[Pt]:
    return [(i * 0.2, 0.15 * math.sin(i * 0.25)) for i in range(35)]


SCENARIOS = {
    "open_straight": straight_path(),
    "gentle_turn": gentle_turn(),
    "turn_90": turn_90(),
    "obstacle_bypass": bypass_arc(),
    "narrow_corridor": narrow_corridor(),
}


def sweep_scenario(name: str, path: List[Pt], x: float, y: float, yaw: float, vx: float) -> List[Dict[str, Any]]:
    rows = []
    for la in LOOKAHEADS:
        dbg = pure_pursuit_target(x, y, yaw, path, vx, lookahead_m=la)
        w = pure_pursuit_w(x, y, yaw, path, vx, lookahead_m=la)
        alpha = _f(dbg.get("alpha")) or 0.0
        rows.append(
            {
                "scenario": name,
                "lookahead_m": la,
                "vx": vx,
                "target_x": dbg.get("x"),
                "target_y": dbg.get("y"),
                "distance_m": dbg.get("distance_m"),
                "heading_error_deg": round(math.degrees(alpha), 3),
                "kappa": dbg.get("kappa"),
                "pp_w": round(w, 4),
                "lookahead_over_vehicle_L": round(la / VEHICLE_L, 3),
            }
        )
    return rows


def _f(v):
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def stopping_distance(v: float, decel: float = 0.35) -> float:
    return (v * v) / (2 * max(decel, 0.05))


def main() -> int:
    all_rows: List[Dict[str, Any]] = []
    for sname, path in SCENARIOS.items():
        for vx in (0.10, 0.15, 0.20, 0.25, 0.30):
            all_rows.extend(sweep_scenario(sname, path, 0.5, 0.0, 0.0, vx))

    print("=== P1-2 Lookahead Distance Sweep (offline) ===")
    print(f"vehicle_length_m={VEHICLE_L:.2f}")
    for vx in (0.10, 0.15, 0.20, 0.25, 0.30):
        sd = stopping_distance(vx)
        ratio_14 = 1.4 / sd if sd > 0 else 0
        print(f"  v={vx:.2f} stopping_dist={sd:.3f}m  1.4m/stopping={ratio_14:.2f}  1.4m/L={1.4/VEHICLE_L:.2f}")

    # Best lookahead by min |heading_error| on gentle_turn @ v=0.20
    gt = [r for r in all_rows if r["scenario"] == "gentle_turn" and r["vx"] == 0.20]
    if gt:
        best = min(gt, key=lambda r: abs(r["heading_error_deg"]))
        print(f"  gentle_turn@v=0.20 min|heading_err| lookahead={best['lookahead_m']}m err={best['heading_error_deg']}°")

    # Corner cutting proxy: turn_90 overshoot in w
    t90 = [r for r in all_rows if r["scenario"] == "turn_90" and r["vx"] == 0.20]
    if t90:
        max_w_la = max(t90, key=lambda r: abs(r["pp_w"]))
        print(f"  turn_90@v=0.20 max|pp_w| lookahead={max_w_la['lookahead_m']}m w={max_w_la['pp_w']}")

    ok = len(all_rows) == len(SCENARIOS) * 5 * len(LOOKAHEADS)
    print(f"RESULT {'PASS' if ok else 'FAIL'} ({len(all_rows)} rows)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
