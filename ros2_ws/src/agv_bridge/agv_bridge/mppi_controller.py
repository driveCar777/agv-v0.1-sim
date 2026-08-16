"""局部控制：DiffDrive MPPI 或 PP-only。

职责边界：
- 本模块输出「推荐」vx/w 与 planned_band
- 最终停车由 Safety Supervisor（sim_api_ext）裁决
- MPPI 不对 front_near 做硬停车（仅代价软提示）
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry

Pt = Tuple[float, float]
CollideFn = Callable[[float, float], bool]


@dataclass
class MppiResult:
    vx: float
    w: float
    best_path: List[Pt]  # planned band（安全门前）
    candidates: List[Dict[str, Any]]
    confidence: float
    mode: str
    control_mode: str = "mppi"


def bumper_pose(x: float, y: float, yaw: float, reverse: bool, bumper_l: float) -> Pt:
    s = -1.0 if reverse else 1.0
    return (x + s * bumper_l * math.cos(yaw), y + s * bumper_l * math.sin(yaw))


def _nearest_index(path: List[Pt], x: float, y: float) -> int:
    best_i, best_d = 0, 1e18
    for i, p in enumerate(path):
        d = math.hypot(p[0] - x, p[1] - y)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def pure_pursuit_w(
    x: float,
    y: float,
    yaw: float,
    global_path: List[Pt],
    vx: float,
    lookahead_m: float = 1.35,
    w_max: float = 0.45,
) -> float:
    if not global_path or len(global_path) < 2:
        return 0.0
    i0 = _nearest_index(global_path, x, y)
    target = global_path[min(i0 + 1, len(global_path) - 1)]
    acc = 0.0
    prev = global_path[i0]
    for p in global_path[i0:]:
        acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        if acc >= lookahead_m:
            target = p
            break
    dx = target[0] - x
    dy = target[1] - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    L = max(0.35, math.hypot(local_x, local_y))
    alpha = math.atan2(local_y, local_x)
    speed = vx if abs(vx) > 0.05 else 0.18
    if abs(speed) < 0.04:
        return max(-0.12, min(0.12, 0.35 * alpha))
    kappa = 2.0 * math.sin(alpha) / L
    return max(-w_max, min(w_max, speed * kappa))


def kinematic_band(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    w: float,
    *,
    reverse: bool,
    bumper_l: float,
    steps: int = 12,
    dt: float = 0.1,
) -> List[Pt]:
    """与 physics 一致的差速积分蓝带；vx≈0 时不画假尖端。"""
    if abs(vx) < 0.02 and abs(w) < 0.02:
        bx, by = bumper_pose(x, y, yaw, reverse, bumper_l)
        return [(bx, by)]
    bx, by = bumper_pose(x, y, yaw, reverse, bumper_l)
    pts: List[Pt] = [(bx, by)]
    cx, cy, cyaw = x, y, yaw
    s = -1.0 if reverse else 1.0
    for _ in range(steps):
        cx += vx * math.cos(cyaw) * dt
        cy += vx * math.sin(cyaw) * dt
        cyaw += w * dt
        pts.append((cx + s * bumper_l * math.cos(cyaw), cy + s * bumper_l * math.sin(cyaw)))
    cleaned: List[Pt] = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) >= 0.03:
            cleaned.append(p)
    return cleaned if len(cleaned) >= 2 else pts[:2]


def get_control_mode() -> str:
    mode = (os.environ.get("AGV_LOCAL_CONTROL") or "mppi").strip().lower()
    return "pp_only" if mode in ("pp", "pp_only", "pure_pursuit") else "mppi"


class DiffDriveMppi:
    def __init__(
        self,
        batch_size: int = 90,
        time_steps: int = 16,
        model_dt: float = 0.1,
        geom: Optional[VehicleGeometry] = None,
        temperature: float = 0.22,
        top_k: int = 8,
    ) -> None:
        self.geom = geom or DEFAULT_GEOM
        self.batch_size = batch_size
        self.time_steps = time_steps
        self.model_dt = model_dt
        self.vx_max = self.geom.max_vx * 0.95
        self.vx_min = -0.18
        self.wz_max = min(0.42, self.geom.max_w)
        self.temperature = max(1e-3, temperature)
        self.top_k = top_k
        self._mean_vx = 0.16
        self._mean_dw = 0.0
        self._cmd_vx = 0.0
        self._cmd_w = 0.0
        # Observability only
        self._last_pp_w = 0.0
        self._last_best_cost = 0.0
        self._last_selected_id: Optional[int] = None
        self._last_first_collision: Optional[Dict[str, Any]] = None
        self._last_meta: Dict[str, Any] = {}

    def reset(self) -> None:
        self._mean_vx = 0.16
        self._mean_dw = 0.0
        self._cmd_vx = 0.0
        self._cmd_w = 0.0
        self._last_pp_w = 0.0
        self._last_best_cost = 0.0
        self._last_selected_id = None
        self._last_first_collision = None
        self._last_meta = {}

    def _rollout_body(
        self, x: float, y: float, yaw: float, vx_seq: Sequence[float], w_seq: Sequence[float]
    ) -> List[Pt]:
        pts: List[Pt] = [(x, y)]
        cx, cy, cyaw = x, y, yaw
        for i in range(len(vx_seq)):
            cx += vx_seq[i] * math.cos(cyaw) * self.model_dt
            cy += vx_seq[i] * math.sin(cyaw) * self.model_dt
            cyaw += w_seq[i] * self.model_dt
            pts.append((cx, cy))
        return pts

    def _guide_band(
        self,
        x: float,
        y: float,
        yaw: float,
        vx_seq: Sequence[float],
        w_seq: Sequence[float],
        reverse: bool,
    ) -> List[Pt]:
        return kinematic_band(
            x,
            y,
            yaw,
            vx_seq[0] if vx_seq else 0.0,
            w_seq[0] if w_seq else 0.0,
            reverse=reverse,
            bumper_l=self.geom.bumper_l,
            steps=len(vx_seq) or 8,
            dt=self.model_dt,
        )

    def _score(
        self,
        path: List[Pt],
        vx_seq: Sequence[float],
        w_seq: Sequence[float],
        global_path: List[Pt],
        goal: Pt,
        collide: CollideFn,
        front_near: float,
        front_cost_m: float,
    ) -> Tuple[float, str, Dict[str, Any]]:
        """Return (cost, mode, debug_meta). Cost formula unchanged; meta is observability only."""
        if len(path) < 2:
            return 1e6, "invalid", {"cost_breakdown": {}, "collision": True, "first_collision": None}
        cost = 0.0
        collision_cost = 0.0
        first_collision = None
        for ti, p in enumerate(path[1::2]):
            if collide(p[0], p[1]):
                collision_cost = 700.0
                cost += collision_cost
                first_collision = {
                    "t": round((ti * 2 + 1) * self.model_dt, 3),
                    "x": round(p[0], 3),
                    "y": round(p[1], 3),
                    "step": ti * 2 + 1,
                }
                break
        gdev = 0.0
        if global_path:
            step = max(1, len(global_path) // 24)
            gpts = global_path[::step]
            for p in path[::3]:
                gdev += min(math.hypot(p[0] - g[0], p[1] - g[1]) for g in gpts)
            gdev /= max(1, len(path[::3]))
        global_path_cost = 5.0 * gdev
        cost += global_path_cost
        sx, sy = path[0]
        ex, ey = path[-1]
        d0 = math.hypot(goal[0] - sx, goal[1] - sy) + 1e-3
        d1 = math.hypot(goal[0] - ex, goal[1] - ey)
        goal_cost = 3.5 * d1 - 2.0 * max(0.0, d0 - d1)
        cost += 3.5 * d1
        cost -= 2.0 * max(0.0, d0 - d1)
        mean_vx = sum(vx_seq) / max(1, len(vx_seq))
        mean_w = sum(w_seq) / max(1, len(w_seq))
        reverse_cost = 0.0
        speed_track_cost = 0.0
        if mean_vx < -0.05:
            reverse_cost = (1.0 if front_near < front_cost_m else 4.5) * abs(mean_vx)
            cost += reverse_cost
            mode = "reverse"
        else:
            speed_track_cost = 0.25 * abs(mean_vx - 0.22)
            cost += speed_track_cost
            mode = "forward"
        w_cost = 2.4 * abs(mean_w)
        cost += w_cost
        jerk_w = sum(abs(w_seq[i] - w_seq[i - 1]) for i in range(1, len(w_seq)))
        jerk_cost = 1.2 * jerk_w
        cost += jerk_cost
        front_obstacle_cost = 0.0
        # 软提示：靠近前障时惩罚前进，最终停车交给 Safety
        if front_near < front_cost_m and mean_vx > 0.04:
            front_obstacle_cost = 40.0 * (1.0 - front_near / max(front_cost_m, 1e-3))
            cost += front_obstacle_cost
        # heading proxy: mean_w already; expose alpha-like via end displacement
        heading_cost = 0.0
        meta = {
            "collision": first_collision is not None,
            "first_collision": first_collision,
            "global_path_error": round(gdev, 4),
            "goal_error": round(d1, 4),
            "jerk_w": round(jerk_w, 4),
            "clearance_m": None,
            "cost_breakdown": {
                "collision_cost": round(collision_cost, 3),
                "front_obstacle_cost": round(front_obstacle_cost, 3),
                "global_path_cost": round(global_path_cost, 3),
                "goal_cost": round(goal_cost, 3),
                "heading_cost": round(heading_cost, 3),
                "jerk_cost": round(jerk_cost, 3),
                "w_cost": round(w_cost, 3),
                "reverse_cost": round(reverse_cost, 3),
                "speed_track_cost": round(speed_track_cost, 3),
                "total": round(cost, 3),
            },
        }
        return cost, mode, meta

    def step_pp_only(
        self,
        x: float,
        y: float,
        yaw: float,
        global_path: List[Pt],
        goal: Pt,
        front_near: float = 30.0,
        force_reverse: bool = False,
    ) -> MppiResult:
        """可解释 PP 旁路：仍输出推荐 cmd，安全门在外层。"""
        if force_reverse:
            vx = -0.12
            w = pure_pursuit_w(x, y, yaw, global_path, vx, lookahead_m=1.0, w_max=self.wz_max)
            w = -w * 0.5
        else:
            vx = 0.22
            if front_near < self.geom.front_cost_m:
                vx = max(0.06, 0.22 * (front_near / self.geom.front_cost_m))
            w = pure_pursuit_w(x, y, yaw, global_path, vx, lookahead_m=1.4, w_max=self.wz_max)
        self._cmd_vx, self._cmd_w = vx, w
        reverse = vx < -0.05
        band = kinematic_band(
            x, y, yaw, vx, w, reverse=reverse, bumper_l=self.geom.bumper_l, steps=12, dt=0.1
        )
        self._last_pp_w = float(w)
        self._last_best_cost = 0.0
        self._last_selected_id = 1
        self._last_first_collision = None
        self._last_meta = {"batch": 1, "time_steps": 12, "model_dt": 0.1, "horizon_s": 1.2}
        return MppiResult(
            vx=float(vx),
            w=float(w),
            best_path=band,
            candidates=[
                {
                    "id": 1,
                    "label": "PP-only",
                    "confidence": 0.9,
                    "mode": "reverse" if reverse else "forward",
                    "horizon_m": round(abs(vx) * 1.2, 2),
                    "path": [{"x": p[0], "y": p[1]} for p in band],
                    "cost": 0.0,
                    "vx": round(float(vx), 4),
                    "w": round(float(w), 4),
                    "collision": False,
                    "cost_breakdown": {"total": 0.0},
                    "confidence_note": "PP-only (not cost-normalized safety)",
                }
            ],
            confidence=0.9,
            mode="reverse" if reverse else "forward",
            control_mode="pp_only",
        )

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        global_path: List[Pt],
        goal: Pt,
        collide: CollideFn,
        front_near: float = 30.0,
        stuck_s: float = 0.0,
        force_reverse: bool = False,
        control_mode: Optional[str] = None,
        maneuver_mode: Optional[str] = None,
        vx_min: Optional[float] = None,
        vx_max: Optional[float] = None,
        force_vx: Optional[float] = None,
        force_w: Optional[float] = None,
    ) -> MppiResult:
        mode = control_mode or get_control_mode()
        # Maneuver-constrained action space (defaults to controller limits)
        a_vx_min = float(self.vx_min if vx_min is None else vx_min)
        a_vx_max = float(self.vx_max if vx_max is None else vx_max)
        if a_vx_min > a_vx_max:
            a_vx_min, a_vx_max = a_vx_max, a_vx_min
        # Clamp within absolute hardware-ish limits
        a_vx_min = max(self.vx_min, a_vx_min)
        a_vx_max = min(self.vx_max, a_vx_max)

        # ALIGN / TURN_IN_PLACE / LOCAL_LEFT / LOCAL_RIGHT: directed commands
        mmode = (maneuver_mode or "").upper()
        if force_vx is not None and force_w is not None and mmode in (
            "ALIGN",
            "TURN_IN_PLACE",
            "WAIT_FOR_CLEARANCE",
            "SAFE_STOP",
            "LOCAL_LEFT",
            "LOCAL_RIGHT",
        ):
            vx_cmd = float(force_vx)
            w_cmd = float(force_w)
            vx_cmd = max(a_vx_min, min(a_vx_max, vx_cmd))
            w_cmd = max(-self.wz_max, min(self.wz_max, w_cmd))
            if mmode == "LOCAL_LEFT":
                vx_cmd = max(0.04, vx_cmd)
                w_cmd = abs(w_cmd)
            elif mmode == "LOCAL_RIGHT":
                vx_cmd = max(0.04, vx_cmd)
                w_cmd = -abs(w_cmd)
            self._cmd_vx, self._cmd_w = vx_cmd, w_cmd
            reverse = vx_cmd < -0.05
            band = kinematic_band(
                x, y, yaw, vx_cmd, w_cmd, reverse=reverse, bumper_l=self.geom.bumper_l, steps=12, dt=0.1
            )
            self._last_pp_w = float(w_cmd)
            self._last_best_cost = 0.0
            self._last_selected_id = 1
            self._last_first_collision = None
            self._last_meta = {
                "batch": 0,
                "time_steps": 12,
                "model_dt": 0.1,
                "horizon_s": 1.2,
                "maneuver_mode": mmode,
            }
            return MppiResult(
                vx=float(vx_cmd),
                w=float(w_cmd),
                best_path=band,
                candidates=[
                    {
                        "id": 1,
                        "label": f"MANEUVER-{mmode}",
                        "confidence": 0.95,
                        "mode": "reverse" if reverse else ("left" if mmode == "LOCAL_LEFT" else "right" if mmode == "LOCAL_RIGHT" else "turn"),
                        "horizon_m": round(abs(vx_cmd) * 1.2, 2),
                        "path": [{"x": p[0], "y": p[1]} for p in band],
                        "cost": 0.0,
                        "vx": round(float(vx_cmd), 4),
                        "w": round(float(w_cmd), 4),
                        "collision": False,
                        "cost_breakdown": {"total": 0.0},
                        "confidence_note": f"maneuver {mmode}",
                    }
                ],
                confidence=0.95,
                mode="reverse" if reverse else "forward",
                control_mode=mode,
            )

        if mode == "pp_only":
            res = self.step_pp_only(
                x, y, yaw, global_path, goal, front_near=front_near, force_reverse=force_reverse
            )
            # Enforce maneuver vx bounds on PP-only too
            vx = max(a_vx_min, min(a_vx_max, float(res.vx)))
            if force_vx is not None and mmode == "REPOSITION":
                vx = max(a_vx_min, min(a_vx_max, float(force_vx)))
            w = float(res.w)
            if force_w is not None and mmode == "REPOSITION":
                w = max(-self.wz_max, min(self.wz_max, float(force_w)))
            if not force_reverse and a_vx_min >= 0.0 and vx < 0.0:
                vx = 0.0
            self._cmd_vx, self._cmd_w = vx, w
            return MppiResult(
                vx=vx,
                w=w,
                best_path=res.best_path,
                candidates=res.candidates,
                confidence=res.confidence,
                mode=("reverse" if vx < -0.05 else res.mode),
                control_mode=res.control_mode,
            )

        n = self.time_steps
        batch = self.batch_size
        # Normal navigation: do not bias toward reverse unless recovery mode
        allow_rev = force_reverse or mmode == "REVERSE_ESCAPE" or mmode == "REPOSITION"
        vx_std = 0.08 + (0.08 if force_reverse else 0.0)
        dw_std = 0.06 + (0.05 if force_reverse else 0.0)
        if force_reverse:
            self._mean_vx = min(self._mean_vx, -0.08)
        elif not allow_rev and self._mean_vx < 0.0:
            self._mean_vx = max(0.0, self._mean_vx)

        pp_w = pure_pursuit_w(
            x, y, yaw, global_path, self._cmd_vx or 0.16, lookahead_m=1.4, w_max=self.wz_max
        )
        if force_w is not None and mmode in ("FORWARD_TURN", "REPOSITION"):
            pp_w = 0.55 * pp_w + 0.45 * float(force_w)
        samples: List[Dict[str, Any]] = []
        for _ in range(batch):
            vx_seq: List[float] = []
            w_seq: List[float] = []
            vx = self._mean_vx
            dw = self._mean_dw
            for _t in range(n):
                vx = max(a_vx_min, min(a_vx_max, vx + random.gauss(0.0, vx_std)))
                dw = max(-0.18, min(0.18, dw + random.gauss(0.0, dw_std)))
                ww = max(-self.wz_max, min(self.wz_max, pp_w + dw))
                if force_reverse and random.random() < 0.35:
                    vx = random.uniform(a_vx_min, min(a_vx_max, -0.04))
                vx_seq.append(vx)
                w_seq.append(ww)
            body = self._rollout_body(x, y, yaw, vx_seq, w_seq)
            cost, smode, meta = self._score(
                body,
                vx_seq,
                w_seq,
                global_path,
                goal,
                collide,
                front_near,
                self.geom.front_cost_m,
            )
            samples.append(
                {
                    "cost": cost,
                    "mode": smode,
                    "vx0": vx_seq[0],
                    "w0": w_seq[0],
                    "dw0": w_seq[0] - pp_w,
                    "vx_seq": vx_seq,
                    "w_seq": w_seq,
                    "path": body,
                    "meta": meta,
                }
            )

        costs = [s["cost"] for s in samples]
        cmin = min(costs)
        weights = [math.exp(-(c - cmin) / self.temperature) for c in costs]
        wsum = sum(weights) + 1e-12
        weights = [w / wsum for w in weights]

        vx_raw = sum(weights[i] * samples[i]["vx0"] for i in range(batch))
        dw_raw = sum(weights[i] * samples[i]["dw0"] for i in range(batch))
        vx_raw = max(a_vx_min, min(a_vx_max, vx_raw))
        dw_raw = max(-0.15, min(0.15, dw_raw))

        self._mean_vx = 0.9 * self._mean_vx + 0.1 * vx_raw
        self._mean_dw = 0.92 * self._mean_dw + 0.08 * dw_raw

        vx_cmd = 0.82 * self._cmd_vx + 0.18 * self._mean_vx
        vx_cmd = max(a_vx_min, min(a_vx_max, vx_cmd))
        w_des = pp_w + 0.35 * self._mean_dw
        # 大航向误差时加快响应，避免 0.12 混合 + 0.03 死区把转向永久掐死
        w_blend = 0.35 if abs(pp_w) > 0.12 else 0.12
        w_cmd = (1.0 - w_blend) * self._cmd_w + w_blend * w_des
        max_dw = 0.18 if abs(pp_w) > 0.12 else 0.12
        w_cmd = max(self._cmd_w - max_dw, min(self._cmd_w + max_dw, w_cmd))
        w_cmd = max(-self.wz_max, min(self.wz_max, w_cmd))
        if abs(vx_cmd) < 0.06 and mmode not in ("FORWARD_TURN",):
            w_cmd *= 0.5  # 低速仍允许对准，不再 *0.25 过死
        if abs(w_cmd) < 0.02 and abs(pp_w) < 0.08:
            w_cmd = 0.0
        # 不再在此硬停车；仅轻微建议减速
        if front_near < self.geom.front_cost_m and vx_cmd > 0:
            vx_cmd *= max(0.35, front_near / self.geom.front_cost_m)

        if force_reverse and vx_cmd > -0.04:
            vx_cmd = max(a_vx_min, min(a_vx_max, -0.10))
        # Forbid sneaking reverse when maneuver forbids it
        if not allow_rev and vx_cmd < 0.0:
            vx_cmd = 0.0
        if force_vx is not None and mmode == "REPOSITION":
            vx_cmd = max(a_vx_min, min(a_vx_max, float(force_vx)))
        if force_w is not None and mmode == "REPOSITION":
            w_cmd = max(-self.wz_max, min(self.wz_max, float(force_w)))

        self._cmd_vx, self._cmd_w = vx_cmd, w_cmd
        reverse = vx_cmd < -0.05
        cmd_mode = "reverse" if reverse else "forward"

        ranked = sorted(samples, key=lambda s: s["cost"])
        best = next((s for s in ranked if s["mode"] == cmd_mode), ranked[0])
        mix = 0.35
        vx_seq_band = [(1.0 - mix) * vx_cmd + mix * best["vx_seq"][i] for i in range(n)]
        w_seq_band = [(1.0 - mix) * w_cmd + mix * best["w_seq"][i] for i in range(n)]
        band = kinematic_band(
            x,
            y,
            yaw,
            sum(vx_seq_band) / n,
            sum(w_seq_band) / n,
            reverse=reverse,
            bumper_l=self.geom.bumper_l,
            steps=n,
            dt=self.model_dt,
        )

        same = [s for s in ranked if s["mode"] == cmd_mode]
        other = [s for s in ranked if s["mode"] != cmd_mode]
        ordered = (same + other)[: self.top_k] or ranked[: self.top_k]
        cmin_show = ordered[0]["cost"]
        cmax_show = ordered[-1]["cost"] if len(ordered) > 1 else cmin_show + 1.0
        span = max(1.0, cmax_show - cmin_show)
        candidates: List[Dict[str, Any]] = []
        for i, s in enumerate(ordered):
            conf = max(0.05, min(0.99, 1.0 - 0.85 * (s["cost"] - cmin_show) / span))
            guide = kinematic_band(
                x,
                y,
                yaw,
                s["vx0"],
                s["w0"],
                reverse=s["mode"] == "reverse",
                bumper_l=self.geom.bumper_l,
                steps=10,
                dt=self.model_dt,
            )
            meta = s.get("meta") or {}
            candidates.append(
                {
                    "id": i + 1,
                    "label": f"MPPI-{i+1}·{s['mode']}",
                    "confidence": round(conf, 4),
                    "mode": s["mode"],
                    "horizon_m": round(abs(s["vx0"]) * n * self.model_dt, 2),
                    "path": [{"x": p[0], "y": p[1]} for p in guide],
                    "cost": round(float(s["cost"]), 2),
                    "vx": round(float(s["vx0"]), 4),
                    "w": round(float(s["w0"]), 4),
                    "collision": bool(meta.get("collision")),
                    "first_collision": meta.get("first_collision"),
                    "cost_breakdown": meta.get("cost_breakdown"),
                    "global_path_error": meta.get("global_path_error"),
                    "goal_error": meta.get("goal_error"),
                    "jerk_w": meta.get("jerk_w"),
                    "clearance_m": meta.get("clearance_m"),
                }
            )

        # Attach observability on controller (does not affect cmd)
        self._last_pp_w = float(pp_w)
        self._last_best_cost = float(best["cost"])
        self._last_selected_id = int(candidates[0]["id"]) if candidates else None
        self._last_first_collision = (best.get("meta") or {}).get("first_collision")
        self._last_meta = {
            "batch": batch,
            "time_steps": n,
            "model_dt": self.model_dt,
            "horizon_s": round(n * self.model_dt, 3),
            "maneuver_mode": mmode or "NONE",
            "vx_min": a_vx_min,
            "vx_max": a_vx_max,
        }

        return MppiResult(
            vx=float(vx_cmd),
            w=float(w_cmd),
            best_path=band,
            candidates=candidates,
            confidence=float(candidates[0]["confidence"]) if candidates else 0.0,
            mode=cmd_mode,
            control_mode="mppi",
        )
