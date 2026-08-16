"""Phase-3 STEP0-2: Path Quality Audit + direct LOS (footprint-aware).

No planner changes — diagnosis only.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
import urllib.request
from typing import Dict, List, Optional, Tuple

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BRIDGE = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
if BRIDGE not in sys.path:
    sys.path.insert(0, BRIDGE)

from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.sim_world import SimWorld  # noqa: E402

BASE = "http://127.0.0.1:19999"
Pt = Tuple[float, float]


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.loads(r.read().decode())


def post(path: str, obj=None):
    data = json.dumps({} if obj is None else obj).encode()
    req = urllib.request.Request(
        BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())


def path_length(path: List[Pt]) -> float:
    return sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]) for i in range(1, len(path)))


def turn_angles(path: List[Pt]) -> List[float]:
    angs: List[float] = []
    for i in range(1, len(path) - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        angs.append(math.degrees(math.acos(cos)))
    return angs


def sample_segment(a: Pt, b: Pt, step: float) -> List[Pt]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return [a]
    n = max(1, int(math.ceil(L / step)))
    return [(a[0] + dx * t / n, a[1] + dy * t / n) for t in range(n + 1)]


def is_direct_path_safe(
    world: SimWorld,
    start: Pt,
    goal: Pt,
    robot_r: float,
    step: Optional[float] = None,
) -> bool:
    """Footprint-aware swept check (disk samples along segment). Not center-only."""
    m = world.map
    assert m is not None
    step = step or max(0.05, m.plan_res * 0.4)
    for p in sample_segment(start, goal, step):
        if world.collides(p[0], p[1], robot_r=robot_r, include_actors=False):
            return False
    return True


def clearance_at(world: SimWorld, x: float, y: float, search_m: float = 2.5) -> float:
    """Approx distance to nearest occupied (inflated) cell center."""
    m = world.map
    assert m is not None
    res = m.plan_res
    cx = int(math.floor((x - m.min_x) / res))
    cy = int(math.floor((y - m.min_y) / res))
    rmax = max(1, int(math.ceil(search_m / res)))
    best = search_m
    for r in range(0, rmax + 1):
        found = False
        for dx in range(-r, r + 1):
            for dy in (-r, r) if r > 0 else (0,):
                c = (cx + dx, cy + dy)
                if c in m.occupied:
                    wx = m.min_x + (c[0] + 0.5) * res
                    wy = m.min_y + (c[1] + 0.5) * res
                    best = min(best, math.hypot(x - wx, y - wy))
                    found = True
            if r == 0:
                continue
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    c = (cx + dx, cy + dy)
                    if c in m.occupied:
                        wx = m.min_x + (c[0] + 0.5) * res
                        wy = m.min_y + (c[1] + 0.5) * res
                        best = min(best, math.hypot(x - wx, y - wy))
                        found = True
        if found and best <= r * res:
            break
    return best


def path_clearance_stats(world: SimWorld, path: List[Pt]) -> Dict[str, float]:
    if not path:
        return {"min": 0.0, "mean": 0.0, "pct_05": 0.0, "pct_10": 0.0, "pct_20": 0.0, "pct_30": 0.0}
    clears = [clearance_at(world, p[0], p[1]) for p in path]
    n = len(clears)
    return {
        "min": min(clears),
        "mean": sum(clears) / n,
        "pct_05": 100.0 * sum(c < 0.05 for c in clears) / n,
        "pct_10": 100.0 * sum(c < 0.10 for c in clears) / n,
        "pct_20": 100.0 * sum(c < 0.20 for c in clears) / n,
        "pct_30": 100.0 * sum(c < 0.30 for c in clears) / n,
    }


def colinear_mergeable(path: List[Pt], ang_tol_deg: float = 8.0) -> int:
    """Count interior points that are nearly colinear with neighbors."""
    angs = turn_angles(path)
    return sum(1 for a in angs if a < ang_tol_deg)


def los_shortcut_count(world: SimWorld, path: List[Pt], robot_r: float) -> Tuple[int, int]:
    """Greedy farthest-LOS prune; return (kept_points, removed)."""
    if len(path) < 3:
        return len(path), 0
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        best_j = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if is_direct_path_safe(world, path[i], path[j], robot_r):
                best_j = j
                break
        out.append(path[best_j])
        i = best_j
    return len(out), len(path) - len(out)


def audit_case(world: SimWorld, name: str, start: Pt, goal: Pt, robot_r: float) -> Dict:
    m = world.map
    assert m is not None
    res = m.plan_res

    def cell(p: Pt) -> Tuple[int, int]:
        return (
            int(math.floor((p[0] - m.min_x) / res)),
            int(math.floor((p[1] - m.min_y) / res)),
        )

    def world_of(c: Tuple[int, int]) -> Pt:
        return (m.min_x + (c[0] + 0.5) * res, m.min_y + (c[1] + 0.5) * res)

    start_c, goal_c = cell(start), cell(goal)
    start_snap, goal_snap = world_of(start_c), world_of(goal_c)

    t0 = time.perf_counter()
    raw = world.plan_path(start, goal, robot_r=robot_r, avoid_dynamic=False)
    plan_ms = (time.perf_counter() - t0) * 1000.0

    euc = math.hypot(goal[0] - start[0], goal[1] - start[1])
    L = path_length(raw)
    angs = turn_angles(raw)
    clr = path_clearance_stats(world, raw)
    direct = is_direct_path_safe(world, start, goal, robot_r)
    # Also check snapped cell centers (what A* actually searches)
    direct_snap = is_direct_path_safe(world, start_snap, goal_snap, robot_r)
    kept, removed = los_shortcut_count(world, raw, robot_r) if raw else (0, 0)

    return {
        "name": name,
        "start_world": start,
        "goal_world": goal,
        "start_grid": start_c,
        "goal_grid": goal_c,
        "start_snap": start_snap,
        "goal_snap": goal_snap,
        "goal_snap_err": math.hypot(goal_snap[0] - goal[0], goal_snap[1] - goal[1]),
        "DIRECT_PATH_SAFE": direct,
        "DIRECT_PATH_SAFE_SNAP": direct_snap,
        "A_STAR_LENGTH": round(L, 3),
        "DIRECT_LENGTH": round(euc, 3),
        "RATIO": round(L / max(euc, 1e-9), 3),
        "n_points": len(raw),
        "n_turns": len(angs),
        "max_turn": round(max(angs), 1) if angs else 0.0,
        "avg_turn": round(sum(angs) / len(angs), 1) if angs else 0.0,
        "turns_gt30": sum(a > 30 for a in angs),
        "turns_gt60": sum(a > 60 for a in angs),
        "turns_90ish": sum(a > 80 for a in angs),
        "min_clearance": round(clr["min"], 3),
        "mean_clearance": round(clr["mean"], 3),
        "pct_within_0.05": round(clr["pct_05"], 1),
        "pct_within_0.10": round(clr["pct_10"], 1),
        "pct_within_0.20": round(clr["pct_20"], 1),
        "pct_within_0.30": round(clr["pct_30"], 1),
        "colinear_mergeable": colinear_mergeable(raw),
        "los_kept": kept,
        "los_removed": removed,
        "plan_ms": round(plan_ms, 1),
        "path_preview": [(round(p[0], 2), round(p[1], 2)) for p in raw[:8]]
        + (["..."] if len(raw) > 8 else [])
        + ([(round(p[0], 2), round(p[1], 2)) for p in raw[-2:]] if len(raw) > 8 else []),
    }


def live_api_cases(start: Pt) -> List[Tuple[str, Pt]]:
    cases = [
        ("G1_open_+2x", (start[0] + 2.0, start[1])),
        ("G2_south_-4y", (start[0], start[1] - 4.0)),
        ("G_diag_+5_-5", (start[0] + 5.0, start[1] - 5.0)),
        ("G_far_+8x", (start[0] + 8.0, start[1])),
    ]
    try:
        s = get("/api/state")
        pois = s.get("pois") or []
        if pois:
            far = max(pois, key=lambda p: math.hypot(p["x"] - start[0], p["y"] - start[1]))
            cases.append((f"far_poi_{far.get('id') or far.get('name')}", (far["x"], far["y"])))
            # pick one with large |dy|
            south = [p for p in pois if p["y"] < start[1] - 2.0]
            if south:
                g = min(south, key=lambda p: math.hypot(p["x"] - start[0], p["y"] - start[1]))
                cases.append((f"south_poi_{g.get('id') or g.get('name')}", (g["x"], g["y"])))
    except Exception as e:
        print("live API optional fail:", e)
    return cases


def main():
    print("=== Path Quality Audit (offline SimWorld, same map loader) ===")
    world = SimWorld()
    world.load_indoor()
    m = world.map
    assert m is not None
    print(
        f"map={m.name} plan_res={m.plan_res} inflate_req={m.requested_inflate_m} "
        f"inflate_act={m.actual_inflate_m} bounds=({m.min_x:.1f},{m.min_y:.1f})-({m.max_x:.1f},{m.max_y:.1f})"
    )
    print(f"planner_radius={DEFAULT_GEOM.planner_radius} safety_radius={DEFAULT_GEOM.safety_radius}")

    # Prefer live spawn if sim is up
    start = (0.0, 0.0)
    try:
        post("/api/cancel")
        post("/api/scene", {"id": "indoor_office"})
        time.sleep(0.4)
        s = get("/api/state")
        start = (float(s["agv"]["x"]), float(s["agv"]["y"]))
        print("live spawn", start, "scene_inflate", s.get("scene"))
    except Exception as e:
        # fallback: first free near origin / map center
        cx = 0.5 * (m.min_x + m.max_x)
        cy = 0.5 * (m.min_y + m.max_y)
        for r in range(0, 40):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    x, y = cx + dx * m.plan_res, cy + dy * m.plan_res
                    if not world.collides(x, y, robot_r=DEFAULT_GEOM.planner_radius, include_actors=False):
                        start = (x, y)
                        break
                else:
                    continue
                break
            else:
                continue
            break
        print("offline spawn", start, "live fail:", e)

    robot_r = DEFAULT_GEOM.planner_radius
    rows = []
    for name, goal in live_api_cases(start):
        row = audit_case(world, name, start, goal, robot_r)
        rows.append(row)
        print("\n---", name, "---")
        for k in (
            "DIRECT_PATH_SAFE",
            "DIRECT_PATH_SAFE_SNAP",
            "A_STAR_LENGTH",
            "DIRECT_LENGTH",
            "RATIO",
            "n_points",
            "n_turns",
            "max_turn",
            "avg_turn",
            "turns_gt30",
            "turns_gt60",
            "turns_90ish",
            "min_clearance",
            "mean_clearance",
            "pct_within_0.10",
            "pct_within_0.20",
            "pct_within_0.30",
            "colinear_mergeable",
            "los_removed",
            "goal_snap_err",
            "plan_ms",
        ):
            print(f"  {k}: {row[k]}")
        print("  path_preview:", row["path_preview"])

        # Cross-check live API path length if available
        try:
            post("/api/cancel")
            time.sleep(0.05)
            r = post("/api/nav/plan", {"x": goal[0], "y": goal[1]})
            st = get("/api/state")
            api_path = [(p["x"], p["y"]) for p in (st["nav"].get("path") or [])]
            api_L = path_length(api_path)
            print(
                f"  LIVE_API: success={r.get('success')} pts={len(api_path)} "
                f"L={api_L:.3f} ratio={api_L/max(row['DIRECT_LENGTH'],1e-9):.3f}"
            )
        except Exception as e:
            print("  LIVE_API skip:", e)

    # Repeatability G8: same start/goal 3 times
    if rows:
        g = rows[0]
        print("\n=== G8 repeatability (3x same plan) ===")
        lengths = []
        for i in range(3):
            p = world.plan_path(start, g["goal_world"], robot_r=robot_r)
            lengths.append(round(path_length(p), 4))
        print("lengths", lengths, "stable", len(set(lengths)) == 1)

    out = os.path.join(os.path.dirname(__file__), "_audit_path_quality_out.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"start": start, "robot_r": robot_r, "cases": rows}, f, indent=2)
    print("\nWrote", out)


if __name__ == "__main__":
    main()
