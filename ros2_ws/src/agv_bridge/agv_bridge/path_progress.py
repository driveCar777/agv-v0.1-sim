"""沿全局路径的进度投影（stuck / telemetry 用）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Pt = Tuple[float, float]


@dataclass
class PathProgress:
    index: int
    s: float
    lateral_m: float
    heading_err: float
    path_len: float


def _cumlen(path: Sequence[Pt]) -> List[float]:
    out = [0.0]
    for i in range(1, len(path)):
        out.append(out[-1] + math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]))
    return out


def project_pose_to_path(
    x: float,
    y: float,
    yaw: float,
    path: Sequence[Pt],
    hint_index: int = 0,
    search_window: int = 40,
) -> PathProgress:
    """将位姿投影到路径：最近点邻域搜索 + 弧长 s。"""
    if not path:
        return PathProgress(0, 0.0, 0.0, 0.0, 0.0)
    n = len(path)
    cum = _cumlen(path)
    path_len = cum[-1]
    i0 = max(0, min(n - 1, hint_index))
    lo = max(0, i0 - search_window)
    hi = min(n, i0 + search_window + 1)
    best_i, best_d = i0, 1e18
    for i in range(lo, hi):
        d = math.hypot(path[i][0] - x, path[i][1] - y)
        if d < best_d:
            best_d, best_i = d, i
    # 若邻域没命中更好点，退回全路径（稀疏路径可接受）
    if best_d > 2.5 and n > search_window * 2:
        for i in range(0, n, max(1, n // 80)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d < best_d:
                best_d, best_i = d, i
    # 段内投影
    s = cum[best_i]
    if best_i + 1 < n:
        ax, ay = path[best_i]
        bx, by = path[best_i + 1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        if L2 > 1e-9:
            t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
            s = cum[best_i] + t * math.sqrt(L2)
            px, py = ax + t * vx, ay + t * vy
            lat = math.hypot(x - px, y - py)
            path_yaw = math.atan2(vy, vx)
        else:
            lat = best_d
            path_yaw = yaw
    else:
        lat = best_d
        if best_i > 0:
            path_yaw = math.atan2(path[best_i][1] - path[best_i - 1][1], path[best_i][0] - path[best_i - 1][0])
        else:
            path_yaw = yaw
    herr = (path_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
    return PathProgress(best_i, float(s), float(lat), float(herr), float(path_len))


@dataclass
class ProgressTracker:
    """沿路径进步跟踪。仅 NORMAL_FORWARD 累计 no-progress。"""

    path_index: int = 0
    path_progress_s: float = 0.0
    best_progress_s: float = 0.0
    last_progress_time: float = 0.0
    no_progress_s: float = 0.0
    goal_distance: float = 0.0
    initialized: bool = False

    def reset(self) -> None:
        self.path_index = 0
        self.path_progress_s = 0.0
        self.best_progress_s = 0.0
        self.last_progress_time = 0.0
        self.no_progress_s = 0.0
        self.goal_distance = 0.0
        self.initialized = False

    def update(
        self,
        *,
        now: float,
        x: float,
        y: float,
        yaw: float,
        path: Sequence[Pt],
        goal: Optional[Tuple[float, float]],
        accumulate_stuck: bool,
        progress_min_m: float,
    ) -> PathProgress:
        prog = project_pose_to_path(x, y, yaw, path, hint_index=self.path_index)
        self.path_index = prog.index
        # 倒车时允许 s 略降，但不回退 best（防抖）
        if not self.initialized:
            self.path_progress_s = prog.s
            self.best_progress_s = prog.s
            self.last_progress_time = now
            self.no_progress_s = 0.0
            self.initialized = True
        else:
            # 单调前进：s 相对上次增加才算
            if prog.s > self.best_progress_s + 1e-3:
                gain = prog.s - self.best_progress_s
                self.best_progress_s = prog.s
                if gain >= progress_min_m * 0.25:  # 小步进也刷新时间
                    self.last_progress_time = now
                    self.no_progress_s = 0.0
            self.path_progress_s = prog.s

        if goal is not None:
            self.goal_distance = math.hypot(goal[0] - x, goal[1] - y)
        else:
            self.goal_distance = 0.0

        if accumulate_stuck:
            self.no_progress_s = max(0.0, now - self.last_progress_time)
        else:
            # 恢复阶段：冻结计时，不累加
            self.last_progress_time = now
            self.no_progress_s = 0.0
        return prog
