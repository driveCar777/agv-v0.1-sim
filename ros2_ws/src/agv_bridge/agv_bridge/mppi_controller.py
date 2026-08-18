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

from agv_bridge.nav_execution_corridor import ExecutionCorridor, corridor_allows_omega_sign
from agv_bridge.nav_geometry import DEFAULT_GEOM, VehicleGeometry, get_vehicle_model
from agv_bridge.nav_trajectory_validator import REASON_COLLISION, REASON_CORRIDOR, validate_trajectory

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


def pure_pursuit_target(
    x: float,
    y: float,
    yaw: float,
    path: List[Pt],
    vx: float,
    lookahead_m: float = 1.35,
) -> Dict[str, Any]:
    """Diagnostic-only: same target selection as pure_pursuit_w, returns metadata (no cmd change)."""
    out: Dict[str, Any] = {
        "exists": False,
        "x": None,
        "y": None,
        "lookahead_m": float(lookahead_m),
        "local_x": None,
        "local_y": None,
        "alpha": None,
        "kappa": None,
        "distance_m": None,
        "heading_error": None,
        "path_index": None,
        "s_along_m": None,
    }
    if not path or len(path) < 2:
        return out
    i0 = _nearest_index(path, x, y)
    target = path[min(i0 + 1, len(path) - 1)]
    acc = 0.0
    prev = path[i0]
    ti = i0
    for j, p in enumerate(path[i0:], start=i0):
        acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        ti = j
        if acc >= lookahead_m:
            target = p
            break
    tx, ty = float(target[0]), float(target[1])
    dx = tx - x
    dy = ty - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    L = max(0.35, math.hypot(local_x, local_y))
    alpha = math.atan2(local_y, local_x)
    speed = vx if abs(vx) > 0.05 else 0.18
    kappa = 2.0 * math.sin(alpha) / L if abs(speed) >= 0.04 else 0.0
    out.update(
        {
            "exists": True,
            "x": round(tx, 4),
            "y": round(ty, 4),
            "local_x": round(local_x, 4),
            "local_y": round(local_y, 4),
            "alpha": round(alpha, 5),
            "kappa": round(kappa, 5),
            "distance_m": round(L, 4),
            "heading_error": round(alpha, 5),
            "path_index": int(ti),
            "s_along_m": round(acc, 4),
        }
    )
    return out


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

    def _rollout_pose_sequence(
        self, x: float, y: float, yaw: float, vx_seq: Sequence[float], w_seq: Sequence[float]
    ) -> List[Dict[str, float]]:
        poses: List[Dict[str, float]] = [{"x": x, "y": y, "yaw": yaw}]
        cx, cy, cyaw = x, y, yaw
        for i in range(len(vx_seq)):
            cx += vx_seq[i] * math.cos(cyaw) * self.model_dt
            cy += vx_seq[i] * math.sin(cyaw) * self.model_dt
            cyaw += w_seq[i] * self.model_dt
            poses.append({"x": cx, "y": cy, "yaw": cyaw})
        return poses

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
        body_poses: Sequence[Dict[str, float]],
        vx_seq: Sequence[float],
        w_seq: Sequence[float],
        global_path: List[Pt],
        goal: Pt,
        collide: CollideFn,
        front_near: float,
        front_cost_m: float,
        path_follow_weight: float = 5.0,
        target_vx: Optional[float] = None,
        clearance_at: Optional[Callable[[float, float], float]] = None,
    ) -> Tuple[float, str, Dict[str, Any]]:
        """Return (cost, mode, debug_meta). Footprint collision via TrajectoryValidator."""
        if len(body_poses) < 2:
            return 1e6, "invalid", {"cost_breakdown": {}, "collision": True, "first_collision": None}
        validation = validate_trajectory(
            body_poses,
            collide=collide,
            clearance_at=clearance_at,
            geom=self.geom,
            margin_m=float(self.geom.safety_margin_m),
            dt=self.model_dt,
            enforce_limits=False,
        )
        cost = 0.0
        collision_cost = 0.0
        first_collision = None
        if not validation.valid and validation.reason in (REASON_COLLISION, "CLEARANCE_TOO_LOW"):
            collision_cost = 700.0
            cost += collision_cost
            fc = validation.collision
            if fc.collision and fc.first_pose is not None:
                first_collision = {
                    "t": round(float(fc.collision_time_s or 0.0), 3),
                    "x": round(float(fc.first_pose.get("x", 0.0)), 3),
                    "y": round(float(fc.first_pose.get("y", 0.0)), 3),
                    "step": fc.first_index,
                    "region": fc.collision_region,
                }
        path = [(float(p["x"]), float(p["y"])) for p in body_poses]
        gdev = 0.0
        if global_path:
            step = max(1, len(global_path) // 24)
            gpts = global_path[::step]
            for p in path[::3]:
                gdev += min(math.hypot(p[0] - g[0], p[1] - g[1]) for g in gpts)
            gdev /= max(1, len(path[::3]))
        w_path = max(0.5, float(path_follow_weight))
        global_path_cost = w_path * gdev
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
            spd_tgt = 0.30 if target_vx is None else float(target_vx)
            speed_track_cost = 0.25 * abs(mean_vx - spd_tgt)
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
            "collision": first_collision is not None or validation.collision.collision,
            "first_collision": first_collision,
            "global_path_error": round(gdev, 4),
            "goal_error": round(d1, 4),
            "jerk_w": round(jerk_w, 4),
            "clearance_m": validation.minimum_clearance.minimum_clearance_m,
            "footprint_validated": True,
            "validator_reason": validation.reason,
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
        path_follow_weight: float = 5.0,
        vx_scale: float = 1.0,
        target_vx: Optional[float] = None,
        local_plan_path: Optional[List[Pt]] = None,
        local_plan_id: Optional[str] = None,
        local_plan_horizon_m: Optional[float] = None,
        execution_corridor: Optional[ExecutionCorridor] = None,
        clearance_at: Optional[Callable[[float, float], float]] = None,
    ) -> MppiResult:
        mode = control_mode or get_control_mode()
        self._path_follow_weight = float(path_follow_weight)
        self._vx_scale = max(0.2, min(1.0, float(vx_scale)))
        self._target_vx = None if target_vx is None else float(target_vx)
        self._local_plan_id = local_plan_id
        mean_vx_before = float(self._mean_vx)
        mean_dw_before = float(self._mean_dw)
        # Maneuver-constrained action space (defaults to controller limits)
        a_vx_min = float(self.vx_min if vx_min is None else vx_min)
        a_vx_max = float(self.vx_max if vx_max is None else vx_max)
        if a_vx_min > a_vx_max:
            a_vx_min, a_vx_max = a_vx_max, a_vx_min
        # Clamp within absolute hardware-ish limits
        a_vx_min = max(self.vx_min, a_vx_min)
        a_vx_max = min(self.vx_max, a_vx_max)

        # ALIGN / TURN_IN_PLACE / LOCAL_* / REVERSE_ESCAPE: directed commands
        mmode = (maneuver_mode or "").upper()
        if force_vx is not None and force_w is not None and mmode in (
            "ALIGN",
            "TURN_IN_PLACE",
            "WAIT_FOR_CLEARANCE",
            "SAFE_STOP",
            "LOCAL_LEFT",
            "LOCAL_RIGHT",
            "REVERSE_ESCAPE",
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
            elif mmode == "REVERSE_ESCAPE":
                # Straight reverse tracker — never invent large yaw
                vx_cmd = min(-0.04, vx_cmd)
                if abs(w_cmd) > 0.12:
                    w_cmd = max(-0.12, min(0.12, w_cmd))
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
                "tracker": "TEMPORARY_REVERSE_TRACKER" if mmode == "REVERSE_ESCAPE" else None,
                "mean_vx_before": round(mean_vx_before, 4),
                "mean_vx_after": round(float(self._mean_vx), 4),
                "vx_raw": round(float(vx_cmd), 4),
                "vx_cmd": round(float(vx_cmd), 4),
                "vx_scale": round(float(getattr(self, "_vx_scale", 1.0)), 4),
                "mean_dw": round(float(self._mean_dw), 4),
                "pp_w": round(float(w_cmd), 4),
                "w_cmd": round(float(w_cmd), 4),
                "follow_path_source": "MANEUVER",
                "tracking_local_plan": False,
                "a_vx_min": round(a_vx_min, 4),
                "a_vx_max": round(a_vx_max, 4),
                "wz_max": round(float(self.wz_max), 4),
                "path_follow_weight": round(float(getattr(self, "_path_follow_weight", 5.0)), 3),
                "temperature": round(float(self.temperature), 4),
                "top_k": int(self.top_k),
                "batch_size": int(self.batch_size),
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
            self._last_meta = {
                **dict(self._last_meta or {}),
                "control_mode": "pp_only",
                "horizon_s": round(self.time_steps * self.model_dt, 3),
                "time_steps": self.time_steps,
                "model_dt": self.model_dt,
                "batch_size": self.batch_size,
                "mean_vx_before": round(mean_vx_before, 4),
                "mean_vx_after": round(float(self._mean_vx), 4),
                "vx_raw": round(float(vx), 4),
                "vx_cmd": round(float(vx), 4),
                "vx_scale": round(float(getattr(self, "_vx_scale", 1.0)), 4),
                "mean_dw": round(mean_dw_before, 4),
                "pp_w": round(float(w), 4),
                "w_cmd": round(float(w), 4),
                "a_vx_min": round(a_vx_min, 4),
                "a_vx_max": round(a_vx_max, 4),
                "wz_max": round(float(self.wz_max), 4),
                "path_follow_weight": round(float(getattr(self, "_path_follow_weight", 5.0)), 3),
            }
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

        # P1-1: sample around SpeedPolicy target, not a frozen 0.16 prior
        tgt = self._target_vx
        if tgt is not None and not force_reverse and mmode not in ("REVERSE_ESCAPE",):
            self._mean_vx = 0.55 * self._mean_vx + 0.45 * float(tgt)
            self._mean_vx = max(a_vx_min, min(a_vx_max, self._mean_vx))
            vx_std = max(0.06, min(0.12, 0.07 + 0.12 * abs(float(tgt) - float(self._mean_vx))))
        if execution_corridor is not None and execution_corridor.active:
            a_vx_max = min(a_vx_max, float(execution_corridor.max_vx_mps))
            if execution_corridor.mode == "WAIT":
                a_vx_min = min(a_vx_min, 0.0)
                a_vx_max = 0.0

        follow_path = global_path
        if local_plan_path and len(local_plan_path) >= 2:
            follow_path = local_plan_path
        follow_len = None
        if len(follow_path) >= 2:
            follow_len = sum(
                math.hypot(follow_path[i][0] - follow_path[i - 1][0], follow_path[i][1] - follow_path[i - 1][1])
                for i in range(1, len(follow_path))
            )
        try:
            from agv_bridge.nav_obstacle_preview import compute_dynamic_lookahead_m

            la_m, _la_src = compute_dynamic_lookahead_m(
                vx=self._cmd_vx or (tgt if tgt is not None else 0.16),
                follow_path_length_m=follow_len,
                local_plan_horizon_m=local_plan_horizon_m,
            )
        except Exception:
            la_m = 1.4
            if local_plan_horizon_m is not None and float(local_plan_horizon_m) > 0.4:
                spd_la = abs(self._cmd_vx) if abs(self._cmd_vx) > 0.05 else abs(self._mean_vx)
                la_m = max(0.55, min(2.2, max(0.7, spd_la * 1.25)))
                la_m = min(la_m, max(0.6, float(local_plan_horizon_m) * 0.70))
        pp_w = pure_pursuit_w(
            x, y, yaw, follow_path, self._cmd_vx or (tgt if tgt is not None else 0.16), lookahead_m=la_m, w_max=self.wz_max
        )
        pp_dbg = pure_pursuit_target(
            x, y, yaw, follow_path, self._cmd_vx or (tgt if tgt is not None else 0.16), lookahead_m=la_m
        )
        follow_src = "LOCAL_PLAN" if (local_plan_path and len(local_plan_path) >= 2) else "GLOBAL_PATH"
        if force_w is not None and mmode in ("FORWARD_TURN", "REPOSITION"):
            pp_w = 0.55 * pp_w + 0.45 * float(force_w)
        samples: List[Dict[str, Any]] = []
        candidate_count = 0
        valid_candidate_count = 0
        collision_rejected_count = 0
        clearance_rejected_count = 0
        constraint_rejected_count = 0
        for _ in range(batch):
            vx_seq: List[float] = []
            w_seq: List[float] = []
            vx = self._mean_vx
            dw = self._mean_dw
            for _t in range(n):
                vx = max(a_vx_min, min(a_vx_max, vx + random.gauss(0.0, vx_std)))
                dw = max(-0.18, min(0.18, dw + random.gauss(0.0, dw_std)))
                ww = max(-self.wz_max, min(self.wz_max, pp_w + dw))
                if execution_corridor is not None and execution_corridor.active and not corridor_allows_omega_sign(execution_corridor, ww):
                    ww = abs(ww) if execution_corridor.mode == "LEFT" else -abs(ww)
                if force_reverse and random.random() < 0.35:
                    vx = random.uniform(a_vx_min, min(a_vx_max, -0.04))
                vx_seq.append(vx)
                w_seq.append(ww)
            body = self._rollout_body(x, y, yaw, vx_seq, w_seq)
            body_poses = self._rollout_pose_sequence(x, y, yaw, vx_seq, w_seq)
            candidate_count += 1
            validation = validate_trajectory(
                body_poses,
                collide=collide,
                clearance_at=clearance_at,
                geom=self.geom,
                margin_m=float(self.geom.safety_margin_m),
                dt=self.model_dt,
                enforce_limits=False,
                execution_corridor=execution_corridor,
            )
            if not validation.valid:
                reason = validation.reason
                if reason == "FOOTPRINT_COLLISION":
                    collision_rejected_count += 1
                elif reason == "CLEARANCE_TOO_LOW":
                    clearance_rejected_count += 1
                else:
                    constraint_rejected_count += 1
                samples.append(
                    {
                        "cost": 1e6,
                        "mode": "invalid",
                        "vx0": vx_seq[0] if vx_seq else 0.0,
                        "w0": w_seq[0] if w_seq else 0.0,
                        "dw0": (w_seq[0] - pp_w) if w_seq else 0.0,
                        "vx_seq": vx_seq,
                        "w_seq": w_seq,
                        "path": body,
                        "meta": {
                            "collision": validation.collision.collision,
                            "first_collision": validation.collision.first_pose,
                            "clearance_m": validation.minimum_clearance.minimum_clearance_m,
                            "validator_reason": reason,
                            "cost_breakdown": {"validator_reject": 1e6, "total": 1e6},
                        },
                        "valid": False,
                    }
                )
                continue
            cost, smode, meta = self._score(
                body_poses,
                vx_seq,
                w_seq,
                follow_path,
                goal,
                collide,
                front_near,
                self.geom.front_cost_m,
                path_follow_weight=getattr(self, "_path_follow_weight", 5.0),
                target_vx=tgt,
                clearance_at=clearance_at,
            )
            valid_candidate_count += 1
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
                    "valid": True,
                }
            )

        valid_samples = [s for s in samples if s.get("valid", True)]
        if not valid_samples:
            self._cmd_vx = 0.0
            self._cmd_w = 0.0
            self._last_pp_w = float(pp_w)
            self._last_best_cost = 1e6
            self._last_selected_id = None
            self._last_first_collision = None
            self._last_meta = {
                "batch": batch,
                "time_steps": n,
                "model_dt": self.model_dt,
                "horizon_s": round(n * self.model_dt, 3),
                "maneuver_mode": mmode or "NONE",
                "candidate_count": candidate_count,
                "valid_candidate_count": 0,
                "collision_rejected_count": collision_rejected_count,
                "clearance_rejected_count": clearance_rejected_count,
                "constraint_rejected_count": constraint_rejected_count,
                "failure_reason": "MPPI_NO_FEASIBLE_TRAJECTORY",
                "execution_corridor": None if execution_corridor is None else execution_corridor.to_dict(),
            }
            return MppiResult(
                vx=0.0,
                w=0.0,
                best_path=[(x, y)],
                candidates=[],
                confidence=0.0,
                mode="invalid",
                control_mode="mppi",
            )

        costs = [s["cost"] for s in valid_samples]
        cmin = min(costs)
        weights = [math.exp(-(c - cmin) / self.temperature) for c in costs]
        wsum = sum(weights) + 1e-12
        weights = [w / wsum for w in weights]

        vx_raw = sum(weights[i] * valid_samples[i]["vx0"] for i in range(len(valid_samples)))
        dw_raw = sum(weights[i] * valid_samples[i]["dw0"] for i in range(len(valid_samples)))
        vx_raw = max(a_vx_min, min(a_vx_max, vx_raw))
        dw_raw = max(-0.15, min(0.15, dw_raw))

        self._mean_vx = 0.85 * self._mean_vx + 0.15 * vx_raw
        self._mean_dw = 0.90 * self._mean_dw + 0.10 * dw_raw

        vx_cmd = 0.50 * self._cmd_vx + 0.50 * self._mean_vx
        vx_cmd = max(a_vx_min, min(a_vx_max, vx_cmd))
        w_des = pp_w + 0.35 * self._mean_dw
        # 大航向误差时加快响应，避免 0.12 混合 + 0.03 死区把转向永久掐死
        w_blend = 0.35 if abs(pp_w) > 0.12 else 0.12
        max_dw = 0.18 if abs(pp_w) > 0.12 else 0.12
        if mmode == "POST_TURN":
            # Recapture must decay leftover LOCAL_* omega toward PP, not hold +0.35.
            w_blend = 0.50
            max_dw = min(float(self.geom.acc_w) * max(n * self.model_dt, 0.05), 0.12)
        w_cmd = (1.0 - w_blend) * self._cmd_w + w_blend * w_des
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

        # Policy CAUTION/AVOID: scale forward speed (never invent reverse)
        if vx_cmd > 0.0:
            vx_cmd *= getattr(self, "_vx_scale", 1.0)
            vx_cmd = max(a_vx_min, min(a_vx_max, vx_cmd))

        self._cmd_vx, self._cmd_w = vx_cmd, w_cmd
        reverse = vx_cmd < -0.05
        cmd_mode = "reverse" if reverse else "forward"

        ranked = sorted(valid_samples, key=lambda s: s["cost"])
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

        # M2-C: final selected trajectory gate (MPPI output ≠ safety truth)
        final_poses = self._rollout_pose_sequence(x, y, yaw, vx_seq_band, w_seq_band)
        braking = get_vehicle_model().braking
        final_validation = validate_trajectory(
            final_poses,
            collide=collide,
            clearance_at=clearance_at,
            geom=self.geom,
            margin_m=float(self.geom.safety_margin_m),
            dt=self.model_dt,
            enforce_limits=True,
            execution_corridor=execution_corridor,
            braking=braking,
        )
        if not final_validation.valid:
            self._cmd_vx = 0.0
            self._cmd_w = 0.0
            self._last_meta = {
                "batch": batch,
                "time_steps": n,
                "model_dt": self.model_dt,
                "candidate_count": candidate_count,
                "valid_candidate_count": valid_candidate_count,
                "collision_rejected_count": collision_rejected_count,
                "clearance_rejected_count": clearance_rejected_count,
                "constraint_rejected_count": constraint_rejected_count + 1,
                "failure_reason": "MPPI_NO_FEASIBLE_TRAJECTORY",
                "final_validator_reason": final_validation.reason,
                "execution_corridor": None if execution_corridor is None else execution_corridor.to_dict(),
            }
            return MppiResult(
                vx=0.0,
                w=0.0,
                best_path=[(x, y)],
                candidates=[],
                confidence=0.0,
                mode="invalid",
                control_mode="mppi",
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
            "mean_vx_before": round(mean_vx_before, 4),
            "mean_vx_after": round(float(self._mean_vx), 4),
            "vx_raw": round(float(vx_raw), 4),
            "vx_cmd": round(float(vx_cmd), 4),
            "vx_scale": round(float(getattr(self, "_vx_scale", 1.0)), 4),
            "mean_dw": round(float(self._mean_dw), 4),
            "mean_dw_before": round(mean_dw_before, 4),
            "pp_w": round(float(pp_w), 4),
            "w_cmd": round(float(w_cmd), 4),
            "a_vx_min": round(a_vx_min, 4),
            "a_vx_max": round(a_vx_max, 4),
            "wz_max": round(float(self.wz_max), 4),
            "path_follow_weight": round(float(getattr(self, "_path_follow_weight", 5.0)), 3),
            "temperature": round(float(self.temperature), 4),
            "top_k": int(self.top_k),
            "batch_size": int(batch),
            "candidate_count": int(candidate_count),
            "valid_candidate_count": int(valid_candidate_count),
            "collision_rejected_count": int(collision_rejected_count),
            "clearance_rejected_count": int(clearance_rejected_count),
            "constraint_rejected_count": int(constraint_rejected_count),
            "final_validator_valid": True,
            "predicted_min_clearance_m": final_validation.minimum_clearance.minimum_clearance_m,
            "selected_candidate_index": int(candidates[0]["id"]) if candidates else None,
            "selected_vx": round(float(vx_cmd), 4),
            "selected_omega": round(float(w_cmd), 4),
            "failure_reason": None,
            "control_mode": "mppi",
            "target_vx": None if tgt is None else round(float(tgt), 4),
            "local_plan_id": local_plan_id,
            "execution_corridor": None if execution_corridor is None else execution_corridor.to_dict(),
            "pp_lookahead_m": round(float(la_m), 3),
            "tracking_local_plan": bool(local_plan_path and len(local_plan_path) >= 2),
            "follow_path_source": follow_src,
            "follow_path_length_m": round(
                sum(
                    math.hypot(follow_path[i][0] - follow_path[i - 1][0], follow_path[i][1] - follow_path[i - 1][1])
                    for i in range(1, len(follow_path))
                )
                if len(follow_path) >= 2
                else 0.0,
                3,
            ),
            "pp_follow_path_source": follow_src,
            "pp_lookahead_point": {"x": pp_dbg.get("x"), "y": pp_dbg.get("y")} if pp_dbg.get("exists") else None,
            "pp_alpha": pp_dbg.get("alpha"),
            "pp_kappa": pp_dbg.get("kappa"),
            "pp_local_x": pp_dbg.get("local_x"),
            "pp_local_y": pp_dbg.get("local_y"),
            "pp_heading_error": pp_dbg.get("heading_error"),
            "w_des": round(float(w_des), 4),
            "w_blend": round(float(w_blend), 4),
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
