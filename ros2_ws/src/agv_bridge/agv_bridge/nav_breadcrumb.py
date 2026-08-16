"""Trajectory Breadcrumb — STEP 3F executed-pose history for retreat.

Trusted samples only. History is NOT inherently safe — must be re-probed.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple

from agv_bridge.nav_trajectory import TrajectorySample

Pt = Tuple[float, float]


@dataclass
class BreadcrumbConfig:
    sample_dist_m: float = 0.08
    yaw_merge_rad: float = 0.12
    max_points: int = 400
    max_distance_m: float = 25.0
    max_age_s: float = 180.0
    min_trusted_dist_m: float = 0.6


DEFAULT_BREADCRUMB_CFG = BreadcrumbConfig()


@dataclass
class BreadcrumbPoint:
    ts: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    w: float = 0.0
    mode: str = ""
    trusted: bool = True
    s_along: float = 0.0  # cumulative distance from oldest→newest
    commitment_side: str = "NONE"

    def to_sample(self) -> TrajectorySample:
        return TrajectorySample(t=self.ts, x=self.x, y=self.y, yaw=self.yaw, vx=self.vx, w=self.w)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ts": self.ts,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 4),
            "vx": round(self.vx, 3),
            "w": round(self.w, 3),
            "mode": self.mode,
            "trusted": self.trusted,
            "s_along": round(self.s_along, 3),
            "commitment_side": self.commitment_side,
        }


class TrajectoryBreadcrumb:
    def __init__(self, cfg: BreadcrumbConfig = DEFAULT_BREADCRUMB_CFG) -> None:
        self.cfg = cfg
        self._pts: Deque[BreadcrumbPoint] = deque()
        self._total_s = 0.0

    def reset(self) -> None:
        self._pts.clear()
        self._total_s = 0.0

    def __len__(self) -> int:
        return len(self._pts)

    def points(self, *, trusted_only: bool = False) -> List[BreadcrumbPoint]:
        if trusted_only:
            return [p for p in self._pts if p.trusted]
        return list(self._pts)

    def record(
        self,
        *,
        now: float,
        x: float,
        y: float,
        yaw: float,
        vx: float = 0.0,
        w: float = 0.0,
        mode: str = "",
        emergency: bool = False,
        collision: bool = False,
        commitment_side: str = "NONE",
    ) -> bool:
        """Append if moved enough. Returns True if a point was stored."""
        trusted = (not emergency) and (not collision) and math.isfinite(x) and math.isfinite(y)
        if not self._pts:
            self._pts.append(
                BreadcrumbPoint(
                    ts=now, x=x, y=y, yaw=yaw, vx=vx, w=w, mode=mode, trusted=trusted, s_along=0.0,
                    commitment_side=commitment_side,
                )
            )
            return True
        last = self._pts[-1]
        dist = math.hypot(x - last.x, y - last.y)
        dyaw = abs(_wrap(yaw - last.yaw))
        if dist < self.cfg.sample_dist_m and dyaw < self.cfg.yaw_merge_rad:
            # Refresh last point metadata only
            last.ts = now
            last.vx = vx
            last.w = w
            last.mode = mode
            if not trusted:
                last.trusted = False
            return False
        self._total_s += dist
        self._pts.append(
            BreadcrumbPoint(
                ts=now,
                x=x,
                y=y,
                yaw=yaw,
                vx=vx,
                w=w,
                mode=mode,
                trusted=trusted,
                s_along=self._total_s,
                commitment_side=commitment_side,
            )
        )
        self._trim(now)
        return True

    def _trim(self, now: float) -> None:
        cfg = self.cfg
        while len(self._pts) > cfg.max_points:
            self._pts.popleft()
        while self._pts and (now - self._pts[0].ts) > cfg.max_age_s:
            self._pts.popleft()
        # distance window from newest
        if self._pts:
            newest_s = self._pts[-1].s_along
            while self._pts and (newest_s - self._pts[0].s_along) > cfg.max_distance_m:
                self._pts.popleft()

    def retreat_polyline(self, *, trusted_only: bool = True, max_m: float = 8.0) -> List[BreadcrumbPoint]:
        """Return recent history from current (newest) backwards, capped by distance."""
        pts = self.points(trusted_only=trusted_only)
        if not pts:
            return []
        out: List[BreadcrumbPoint] = []
        newest = pts[-1]
        for p in reversed(pts):
            if (newest.s_along - p.s_along) > max_m:
                break
            out.append(p)
        return out  # newest-first

    def to_dict(self, *, max_points: int = 80) -> Dict[str, Any]:
        pts = list(self._pts)
        if len(pts) > max_points:
            step = max(1, len(pts) // max_points)
            pts = pts[::step]
            if pts[-1] is not self._pts[-1]:
                pts = list(pts) + [self._pts[-1]]
        return {
            "implemented": True,
            "count": len(self._pts),
            "length_m": round(
                (self._pts[-1].s_along - self._pts[0].s_along) if len(self._pts) >= 2 else 0.0, 3
            ),
            "points": [p.to_dict() for p in pts],
            "cfg": {
                "sample_dist_m": self.cfg.sample_dist_m,
                "max_distance_m": self.cfg.max_distance_m,
                "max_age_s": self.cfg.max_age_s,
                "max_points": self.cfg.max_points,
            },
        }


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
