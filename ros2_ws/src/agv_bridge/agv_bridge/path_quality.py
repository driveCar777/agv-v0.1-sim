"""Global path quality: clearance DT, metrics, LOS prune, collision-checked smooth."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

Pt = Tuple[float, float]
Cell = Tuple[int, int]

# Soft preferences (cell-step units ≈ plan_res meters for cardinal steps)
CLEARANCE_PREF_M = 0.70
CLEARANCE_WEIGHT = 0.55  # soft; not absolute objective
TURN_WEIGHT = 0.35
# Turn penalties by 45° steps: 0,45,90,135,180
TURN_PENALTY = (0.0, 0.12, 0.42, 0.85, 1.35)

DIRS8: Tuple[Tuple[int, int], ...] = (
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
)


@dataclass
class PathQualityMetrics:
    raw_path_length: float = 0.0
    final_path_length: float = 0.0
    euclidean_distance: float = 0.0
    path_ratio: float = 0.0
    raw_turn_count: int = 0
    final_turn_count: int = 0
    max_turn_angle: float = 0.0
    min_clearance: float = 0.0
    mean_clearance: float = 0.0
    direct_path_safe: bool = False
    simplification_removed_points: int = 0
    smoothing_applied: bool = False
    smoothing_valid: bool = False
    planner_cost: float = 0.0
    clearance_cost: float = 0.0
    turn_cost: float = 0.0
    n_raw_points: int = 0
    n_final_points: int = 0
    turns_gt30: int = 0
    turns_gt60: int = 0
    turns_90ish: int = 0
    start_world: Pt = (0.0, 0.0)
    goal_world: Pt = (0.0, 0.0)
    start_grid: Cell = (0, 0)
    goal_grid: Cell = (0, 0)
    goal_snap_error: float = 0.0
    variant: str = "C"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["start_world"] = list(self.start_world)
        d["goal_world"] = list(self.goal_world)
        d["start_grid"] = list(self.start_grid)
        d["goal_grid"] = list(self.goal_grid)
        return d


def path_length(path: Sequence[Pt]) -> float:
    return sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )


def turn_angles(path: Sequence[Pt]) -> List[float]:
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
    n = max(1, int(math.ceil(L / max(step, 1e-6))))
    return [(a[0] + dx * t / n, a[1] + dy * t / n) for t in range(n + 1)]


def is_direct_path_safe(
    free_xy: Callable[[float, float], bool],
    start: Pt,
    goal: Pt,
    step: float,
) -> bool:
    """Footprint-aware: free_xy already encodes robot radius / bans."""
    for p in sample_segment(start, goal, step):
        if not free_xy(p[0], p[1]):
            return False
    return True


def clearance_m_at_cell(
    cell: Cell,
    occupied: set,
    plan_res: float,
    min_x: float,
    min_y: float,
    search_m: float = CLEARANCE_PREF_M + 0.15,
) -> float:
    """Spiral search to nearest occupied; returns meters to cell-center obstacle."""
    if cell in occupied:
        return 0.0
    res = plan_res
    rmax = max(1, int(math.ceil(search_m / res)))
    best = search_m
    cx, cy = cell
    for r in range(0, rmax + 1):
        found = False
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if r > 0 and abs(dx) != r and abs(dy) != r:
                    continue
                c2 = (cx + dx, cy + dy)
                if c2 not in occupied:
                    continue
                wx = min_x + (c2[0] + 0.5) * res
                wy = min_y + (c2[1] + 0.5) * res
                px = min_x + (cx + 0.5) * res
                py = min_y + (cy + 0.5) * res
                best = min(best, math.hypot(px - wx, py - wy))
                found = True
        if found and best <= (r + 0.5) * res:
            break
    return best


def clearance_cost_at(dist_m: float, pref_m: float = CLEARANCE_PREF_M, weight: float = CLEARANCE_WEIGHT) -> float:
    """Bounded soft cost: 0 at >= pref, weight at 0 clearance. No 1/d blow-up."""
    if dist_m >= pref_m:
        return 0.0
    t = 1.0 - max(0.0, dist_m) / max(pref_m, 1e-6)
    return weight * (t * t)


def turn_cost_dirs(prev_dir: int, new_dir: int, weight: float = TURN_WEIGHT) -> float:
    if prev_dir < 0:
        return 0.0
    steps = min((new_dir - prev_dir) % 8, (prev_dir - new_dir) % 8)
    return weight * TURN_PENALTY[steps]


def dir_index(dx: int, dy: int) -> int:
    for i, (ax, ay) in enumerate(DIRS8):
        if ax == dx and ay == dy:
            return i
    return 0


def compute_path_clearance_stats(
    path: Sequence[Pt],
    clearance_at: Callable[[float, float], float],
) -> Tuple[float, float]:
    if not path:
        return 0.0, 0.0
    vals = [clearance_at(p[0], p[1]) for p in path]
    return min(vals), sum(vals) / len(vals)


def los_simplify(
    path: Sequence[Pt],
    segment_safe: Callable[[Pt, Pt], bool],
) -> Tuple[List[Pt], int]:
    """Greedy farthest LOS shortcut. Returns (simplified, removed_count)."""
    if len(path) < 3:
        return list(path), 0
    out: List[Pt] = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        best_j = i + 1
        # Search farthest j that is LOS-safe from i
        for j in range(n - 1, i + 1, -1):
            if segment_safe(path[i], path[j]):
                best_j = j
                break
        out.append(path[best_j])
        i = best_j
    removed = n - len(out)
    return out, removed


def densify_path(path: Sequence[Pt], spacing_m: float = 0.55) -> List[Pt]:
    """Re-sample along simplified segments for PP tracking (keeps LOS geometry)."""
    if len(path) < 2 or spacing_m <= 1e-6:
        return list(path)
    out: List[Pt] = [path[0]]
    for i in range(1, len(path)):
        a, b = path[i - 1], path[i]
        for p in sample_segment(a, b, spacing_m)[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < 0.08:
                out[-1] = p
            else:
                out.append(p)
    return out


def smooth_path_collision_checked(
    path: Sequence[Pt],
    segment_safe: Callable[[Pt, Pt], bool],
    point_free: Callable[[float, float], bool],
    corner_deg: float = 28.0,
) -> Tuple[List[Pt], bool, bool]:
    """Light corner chamfer. Returns (path, applied, valid). Rollback on failure."""
    if len(path) < 3:
        return list(path), False, True
    angs = turn_angles(path)
    if not angs or max(angs) < corner_deg:
        return list(path), False, True

    cand: List[Pt] = [path[0]]
    for i in range(1, len(path) - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 < 1e-6 or n2 < 1e-6:
            cand.append(b)
            continue
        cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        ang = math.degrees(math.acos(cos))
        if ang < corner_deg:
            cand.append(b)
            continue
        # Chamfer: keep points 30% back along both edges
        t = 0.30
        p1 = (b[0] - v1[0] / n1 * min(0.35, n1 * t), b[1] - v1[1] / n1 * min(0.35, n1 * t))
        p2 = (b[0] + v2[0] / n2 * min(0.35, n2 * t), b[1] + v2[1] / n2 * min(0.35, n2 * t))
        if point_free(*p1) and point_free(*p2):
            if cand[-1] != p1:
                cand.append(p1)
            cand.append(p2)
        else:
            cand.append(b)
    cand.append(path[-1])

    # Validate all consecutive segments
    for i in range(1, len(cand)):
        if not segment_safe(cand[i - 1], cand[i]):
            return list(path), True, False
    return cand, True, True


def metrics_for_paths(
    raw: Sequence[Pt],
    final: Sequence[Pt],
    start: Pt,
    goal: Pt,
    *,
    direct_safe: bool,
    removed: int,
    smoothing_applied: bool,
    smoothing_valid: bool,
    clearance_at: Callable[[float, float], float],
    planner_cost: float = 0.0,
    clearance_cost: float = 0.0,
    turn_cost: float = 0.0,
    start_grid: Cell = (0, 0),
    goal_grid: Cell = (0, 0),
    goal_snap_error: float = 0.0,
    variant: str = "C",
) -> PathQualityMetrics:
    raw_angs = turn_angles(raw)
    fin_angs = turn_angles(final)
    euc = math.hypot(goal[0] - start[0], goal[1] - start[1])
    raw_L = path_length(raw)
    fin_L = path_length(final)
    mn, mean = compute_path_clearance_stats(final if final else raw, clearance_at)
    return PathQualityMetrics(
        raw_path_length=raw_L,
        final_path_length=fin_L,
        euclidean_distance=euc,
        path_ratio=fin_L / max(euc, 1e-9),
        raw_turn_count=len(raw_angs),
        final_turn_count=len(fin_angs),
        max_turn_angle=max(fin_angs) if fin_angs else (max(raw_angs) if raw_angs else 0.0),
        min_clearance=mn,
        mean_clearance=mean,
        direct_path_safe=direct_safe,
        simplification_removed_points=removed,
        smoothing_applied=smoothing_applied,
        smoothing_valid=smoothing_valid,
        planner_cost=planner_cost,
        clearance_cost=clearance_cost,
        turn_cost=turn_cost,
        n_raw_points=len(raw),
        n_final_points=len(final),
        turns_gt30=sum(a > 30 for a in fin_angs),
        turns_gt60=sum(a > 60 for a in fin_angs),
        turns_90ish=sum(a > 80 for a in fin_angs),
        start_world=start,
        goal_world=goal,
        start_grid=start_grid,
        goal_grid=goal_grid,
        goal_snap_error=goal_snap_error,
        variant=variant,
    )


@dataclass
class PlanResult:
    raw_path: List[Pt] = field(default_factory=list)
    path: List[Pt] = field(default_factory=list)
    metrics: PathQualityMetrics = field(default_factory=PathQualityMetrics)
