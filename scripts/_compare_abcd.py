"""Quick A/B/C/D comparison after path-quality changes."""
from __future__ import annotations

import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.sim_world import SimWorld


def main():
    w = SimWorld()
    w.load_indoor()
    s = (6.26, -0.66)
    cases = [
        ("G1", (s[0] + 2, s[1])),
        ("G2", (s[0], s[1] - 4)),
        ("hug", (16.0, -2.0)),
        ("far", (10.57, -12.0)),
    ]
    for name, g in cases:
        cmp = w.compare_path_variants(s, g, robot_r=DEFAULT_GEOM.planner_radius)
        print("==", name, "direct", cmp["A"]["direct_safe"])
        for k in "ABCD":
            v = cmp[k]
            print(
                f"  {k}: L={v['length']:.2f} ratio={v['ratio']:.3f} "
                f"minc={v['min_clearance']:.3f} meanc={v['mean_clearance']:.3f} "
                f"turns={v['turns']} pts={v['n_points']} rm={v['removed']} "
                f"sm={v['smoothing_applied']}/{v['smoothing_valid']}"
            )


if __name__ == "__main__":
    main()
