"""Find wall-hugging A* cases: place circle obstacle, compare paths."""
from __future__ import annotations

import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "agv_bridge"))

from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.sim_world import SimWorld
from _audit_path_quality import (  # noqa: E402
    audit_case,
    clearance_at,
    is_direct_path_safe,
    path_length,
    turn_angles,
)

# Allow importing sibling script
sys.path.insert(0, os.path.dirname(__file__))


def main():
    world = SimWorld()
    world.load_indoor()
    r = DEFAULT_GEOM.planner_radius

    # Live-like spawn
    start = (6.26, -0.66)
    # Place a fat obstacle slightly south of start to force two corridors
    # Open space around spawn; put obstacle at ~ (8.5, -2.5) blocking diagonal
    world.dyn_obstacles = [{"x": 8.5, "y": -2.0, "r": 1.2, "id": "audit_wall"}]

    # Force avoid_dynamic for this audit by temporarily using collides with actors
    # plan_path avoid_dynamic=False ignores dyn — so bake into occupied via forbid or patch.
    # Use forbid_circles on plan instead:
    forbid = [(8.5, -2.0, 1.2)]

    goals = [
        ("around_obs_+6x_-4y", (12.0, -4.5)),
        ("past_obs_+8x", (14.5, -0.7)),
        ("south_past", (8.5, -6.0)),
    ]

    for name, goal in goals:
        direct = is_direct_path_safe(world, start, goal, r)
        # Direct with forbid: check samples vs circles + map
        path = world.plan_path(start, goal, robot_r=r, avoid_dynamic=False, forbid_circles=forbid)
        L = path_length(path)
        euc = math.hypot(goal[0] - start[0], goal[1] - start[1])
        angs = turn_angles(path)
        clears = [clearance_at(world, p[0], p[1]) for p in path] if path else []
        # Also clearance to forbid circle
        circ_clr = [
            max(0.0, math.hypot(p[0] - 8.5, p[1] + 2.0) - 1.2) for p in path
        ] if path else []
        print(f"\n{name}: direct_map={direct} pts={len(path)} L={L:.2f} euc={euc:.2f} ratio={L/max(euc,1e-9):.2f}")
        print(f"  turns>30={sum(a>30 for a in angs)} max_turn={max(angs) if angs else 0:.0f}")
        print(f"  map_min_clr={min(clears) if clears else None:.3f} circ_min_clr={min(circ_clr) if circ_clr else None:.3f}")
        print(f"  preview={[(round(p[0],2),round(p[1],2)) for p in path[:6]]}...{[(round(p[0],2),round(p[1],2)) for p in path[-2:]]}")

    # Corridor squeeze: find two free cells with wall between wider path
    # Scan map for a cell with clearance ~0.35 and a parallel route with higher clearance
    m = world.map
    assert m is not None
    # Plan far POI-like without forbid
    world.dyn_obstacles = []
    row = audit_case(world, "baseline_far", start, (10.57, -12.0), r)
    print("\nbaseline far without forbid:", {k: row[k] for k in (
        "DIRECT_PATH_SAFE","A_STAR_LENGTH","DIRECT_LENGTH","RATIO","min_clearance","mean_clearance",
        "turns_gt30","turns_90ish","los_removed"
    )})

    # Find worst min-clearance among random free→free goals
    worst = None
    rng_goals = []
    for gx in range(-5, 20, 3):
        for gy in range(-20, 2, 3):
            g = (float(gx), float(gy))
            if world.collides(g[0], g[1], robot_r=r, include_actors=False):
                continue
            if math.hypot(g[0]-start[0], g[1]-start[1]) < 4:
                continue
            path = world.plan_path(start, g, robot_r=r)
            if not path:
                continue
            clears = [clearance_at(world, p[0], p[1]) for p in path[1:-1] or path]
            mc = min(clears) if clears else 99
            L = path_length(path)
            euc = math.hypot(g[0]-start[0], g[1]-start[1])
            ratio = L / max(euc, 1e-9)
            item = (mc, ratio, L, euc, g, len(path), path)
            rng_goals.append(item)
            if worst is None or mc < worst[0] or (abs(mc-worst[0])<1e-6 and ratio > worst[1]):
                worst = item
    rng_goals.sort(key=lambda t: (t[0], -t[1]))
    print("\n=== Worst hugging candidates (lowest min clearance) ===")
    for item in rng_goals[:8]:
        mc, ratio, L, euc, g, n, path = item
        angs = turn_angles(path)
        print(f"  goal={g} min_clr={mc:.3f} ratio={ratio:.2f} L={L:.1f} euc={euc:.1f} pts={n} t>45={sum(a>45 for a in angs)}")

    print("\n=== Highest ratio candidates ===")
    by_ratio = sorted(rng_goals, key=lambda t: -t[1])[:8]
    for item in by_ratio:
        mc, ratio, L, euc, g, n, path = item
        print(f"  goal={g} ratio={ratio:.2f} L={L:.1f} euc={euc:.1f} min_clr={mc:.3f} pts={n}")


if __name__ == "__main__":
    main()
