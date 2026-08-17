"""仿真世界：真实 smap / 室外园区 + 双对角雷达 + 文言编年。"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

from agv_bridge.smap_loader import (
    LoadedSmap,
    MapPOI,
    build_m32_open_straight,
    build_outdoor_campus,
    default_smap_candidates,
    load_smap,
)
from agv_bridge.path_quality import (
    CLEARANCE_PREF_M,
    CLEARANCE_WEIGHT,
    DIRS8,
    PlanResult,
    PathQualityMetrics,
    TURN_WEIGHT,
    clearance_cost_at,
    clearance_m_at_cell,
    is_direct_path_safe,
    los_simplify,
    densify_path,
    metrics_for_paths,
    smooth_path_collision_checked,
    turn_cost_dirs,
)


@dataclass
class ChronicleEvent:
    ts: float
    code: str
    classical: str
    reason: str
    level: str = "info"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ts": self.ts,
            "code": self.code,
            "classical": self.classical,
            "reason": self.reason,
            "level": self.level,
        }


CHRONICLE_TEMPLATES = {
    "idle": ("车止于途，静观四野", "待命，无运动指令"),
    "cmd_vel": ("轮毂既转，气机初动", "收到速度指令，经 API 3055 下发"),
    "api_3055": ("令出平动，车体应之", "调用实车同款 API 3055 平动控制"),
    "api_3056": ("令出回旋，方位更易", "调用实车同款 API 3056 转动控制"),
    "api_stop": ("急勒其轮，止于当前", "调用停止/急停接口"),
    "goal_set": ("指点江山，以此为的", "地图设点，导航目标已确立"),
    "planning": ("揆度路径，择其通达", "全局规划中，绕开已知障碍"),
    "planned": ("路径既定，候令而行", "规划完成，待确认后启程"),
    "confirm_go": ("奉令启程，循线以进", "车上确认开始，执行导航"),
    "tracking": ("循线而行，不逾矩度", "局部跟踪规划路径"),
    "lidar_see": ("双雷达交映，物形毕现", "对角激光感知到周围障碍"),
    "avoid": ("障在目前，故迂回以进", "雷达见障，局部避障减速/绕行"),
    "slow": ("前途逼仄，徐行勿躁", "近距障碍，限速缓行"),
    "blocked": ("无路可通，暂且驻足", "前方阻塞，无法通行"),
    "arrived": ("既至其地，敛辔而息", "到达目标点"),
    "cancel": ("中道而废，奉令而止", "导航任务取消"),
    "control_lock": ("权柄在握，方可行车", "已获取控制权"),
    "control_lost": ("权既旁落，不可妄动", "控制权丢失，安全停车"),
    "scene_indoor": ("入室寻径，点云为鉴", "切换室内办公室点云地图"),
    "scene_outdoor": ("出户临衢，广域而行", "切换室外公开园区场景"),
}


class SimWorld:
    """占用栅格世界 + 双对角雷达。"""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.chronicle: Deque[ChronicleEvent] = deque(maxlen=40)
        self._last_codes: Dict[str, float] = {}
        self.map: Optional[LoadedSmap] = None
        self.scene_id = "indoor_office"
        # 用户动态障碍（圆形），影响雷达/规划
        self.dyn_obstacles: List[Dict[str, Any]] = []
        # 用户自定义点位（按场景保留）
        self._extra_pois: Dict[str, List[MapPOI]] = {}
        # 动态参与物（行人/小车等），参与雷达，不永久改地图
        self.actors: List[Dict[str, Any]] = []
        self._actors_enabled = True
        # M3.1 deterministic scenario movers (LiDAR-visible, stepped each tick)
        self.scenario_movers: List[Dict[str, Any]] = []
        self._actor_last_t = time.time()
        self._last_plan_metrics: Optional[Dict[str, Any]] = None
        self._last_raw_path: List[Tuple[float, float]] = []
        self._last_plan_result: Optional[PlanResult] = None
        self._load_default_scene()

    # ---- scene ----
    def _load_default_scene(self) -> None:
        cands = default_smap_candidates()
        if cands:
            # 优先点数更多的办公室图
            best = max(cands, key=lambda p: p.stat().st_size)
            self.load_indoor(best)
        else:
            self.load_outdoor()

    def load_indoor(self, path: Optional[Path] = None) -> LoadedSmap:
        if path is None:
            cands = default_smap_candidates()
            if not cands:
                raise FileNotFoundError("no office smap found under agv_downloaded/maps")
            path = max(cands, key=lambda p: p.stat().st_size)
        loaded = load_smap(path, plan_res=0.25, inflate_m=0.28, scene_kind="indoor")
        self._clear_poi_cells(loaded)
        with self.lock:
            self.map = loaded
            self.scene_id = "indoor_office"
            self._reset_dynamic_actors()
        self.emit("scene_indoor", f"地图 {loaded.name} · 点云 {len(loaded.cloud)}")
        return loaded

    def load_outdoor(self) -> LoadedSmap:
        loaded = build_outdoor_campus(plan_res=0.4)
        self._clear_poi_cells(loaded)
        with self.lock:
            self.map = loaded
            self.scene_id = "outdoor_campus"
            self._reset_dynamic_actors()
        self.emit("scene_outdoor", f"公开园区 · 点云 {len(loaded.cloud)}")
        return loaded

    def load_m32_open_straight(self) -> LoadedSmap:
        loaded = build_m32_open_straight(plan_res=0.4, inflate_m=0.28)
        self._clear_poi_cells(loaded, clear_r=1.0)
        with self.lock:
            self.map = loaded
            self.scene_id = "m32_open_straight"
            self.dyn_obstacles = []
            self.scenario_movers = []
            self._actors_enabled = False
            self.actors = []
        self.emit("scene_indoor", f"M3.2 open straight · runway 50m")
        return loaded

    @staticmethod
    def _clear_poi_cells(loaded: LoadedSmap, clear_r: float = 0.7) -> None:
        """仅清除规划层 occupied，保留 occupied_raw。

        语义：POI 落在膨胀墙里时允许 A* 起终点；雷达仍感知真实墙体。
        """
        rad = max(1, int(math.ceil(clear_r / loaded.plan_res)))
        for p in loaded.pois:
            ix0, iy0 = loaded.world_to_cell(p.x, p.y)
            for dx in range(-rad, rad + 1):
                for dy in range(-rad, rad + 1):
                    if dx * dx + dy * dy <= rad * rad:
                        loaded.occupied.discard((ix0 + dx, iy0 + dy))

    def set_scene(self, scene_id: str) -> Dict[str, Any]:
        if scene_id in ("indoor", "indoor_office", "office"):
            m = self.load_indoor()
        elif scene_id in ("outdoor", "outdoor_campus", "campus"):
            m = self.load_outdoor()
        elif scene_id in ("m32_open_straight", "M32-OPEN-STRAIGHT", "open_straight"):
            m = self.load_m32_open_straight()
        else:
            return {"success": False, "message": f"unknown scene {scene_id}"}
        with self.lock:
            self.dyn_obstacles = []
        return {"success": True, "scene": self.scene_info()}

    def scene_info(self) -> Dict[str, Any]:
        with self.lock:
            m = self.map
            assert m is not None
            return {
                "id": self.scene_id,
                "kind": m.scene_kind,
                "name": m.name,
                "path_color": m.path_color,
                "bounds": {
                    "min_x": m.min_x,
                    "min_y": m.min_y,
                    "max_x": m.max_x,
                    "max_y": m.max_y,
                },
                "cloud_count": len(m.cloud),
                "poi_count": len(m.pois),
                "source": m.path,
                "plan_res": m.plan_res,
                "requested_inflate_m": float(getattr(m, "requested_inflate_m", 0.0) or 0.0),
                "actual_inflate_m": float(getattr(m, "actual_inflate_m", 0.0) or 0.0),
            }

    def list_scenes(self) -> List[Dict[str, Any]]:
        return [
            {
                "id": "indoor_office",
                "kind": "indoor",
                "label": "室内 · 办公室点云",
                "path_color": "#E8B84A",
                "description": "agv_downloaded 真实 .smap",
            },
            {
                "id": "outdoor_campus",
                "kind": "outdoor",
                "label": "室外 · 公开园区",
                "path_color": "#3B82F6",
                "description": "离线公开园区/停车场场景",
            },
            {
                "id": "m32_open_straight",
                "kind": "open",
                "label": "M3.2 · Open Straight Baseline",
                "path_color": "#22C55E",
                "description": "50m obstacle-free runway for baseline validity",
            },
        ]

    def spawn_pose(self) -> Tuple[float, float, float]:
        """找一个自由出生点。"""
        with self.lock:
            m = self.map
            assert m is not None
            if m.pois:
                for p in m.pois:
                    if not self._collides_unlocked(m, p.x, p.y, 0.45):
                        return p.x, p.y, 0.0
            # 扫中心附近
            cx = (m.min_x + m.max_x) * 0.5
            cy = (m.min_y + m.max_y) * 0.5
            for r in range(0, 40):
                for a in range(0, 360, 30):
                    x = cx + r * 0.5 * math.cos(math.radians(a))
                    y = cy + r * 0.5 * math.sin(math.radians(a))
                    if m.in_bounds(x, y) and not self._collides_unlocked(m, x, y, 0.45):
                        return x, y, 0.0
            return cx, cy, 0.0

    def pois(self) -> List[Dict[str, Any]]:
        with self.lock:
            assert self.map is not None
            base = [p.to_dict() for p in self.map.pois]
            extra = [p.to_dict() for p in self._extra_pois.get(self.scene_id, [])]
            return base + extra

    def find_poi(self, query: str) -> Optional[MapPOI]:
        q = (query or "").strip().lower()
        with self.lock:
            assert self.map is not None
            pool = list(self.map.pois) + list(self._extra_pois.get(self.scene_id, []))
            for p in pool:
                if p.id.lower() == q:
                    return p
            for p in pool:
                if q and q in p.id.lower():
                    return p
        return None

    def add_poi(self, poi_id: str, x: float, y: float, kind: str = "UserMark") -> Dict[str, Any]:
        pid = (poi_id or "").strip()
        if not pid:
            return {"success": False, "message": "点位名不能为空"}
        with self.lock:
            assert self.map is not None
            if not self.map.in_bounds(x, y):
                return {"success": False, "message": "点位超出地图范围"}
            for p in self.map.pois:
                if p.id.lower() == pid.lower():
                    return {"success": False, "message": "与系统点位重名，请换名"}
            bucket = self._extra_pois.setdefault(self.scene_id, [])
            for p in bucket:
                if p.id.lower() == pid.lower():
                    p.x, p.y = float(x), float(y)
                    return {"success": True, "poi": p.to_dict(), "updated": True}
            poi = MapPOI(id=pid, x=float(x), y=float(y), kind=kind)
            bucket.append(poi)
            return {"success": True, "poi": poi.to_dict(), "updated": False}

    def remove_poi(self, poi_id: str) -> Dict[str, Any]:
        pid = (poi_id or "").strip().lower()
        with self.lock:
            bucket = self._extra_pois.get(self.scene_id, [])
            before = len(bucket)
            self._extra_pois[self.scene_id] = [p for p in bucket if p.id.lower() != pid]
            ok = len(self._extra_pois[self.scene_id]) < before
            return {"success": ok, "message": "ok" if ok else "只能删除用户添加的点位"}

    def add_dyn_obstacle(
        self, x: float, y: float, r: float = 0.4, name: str = "", kind: str = "box"
    ) -> Dict[str, Any]:
        with self.lock:
            nm = name or f"obs_{len(self.dyn_obstacles) + 1}"
            self.dyn_obstacles = [o for o in self.dyn_obstacles if o.get("name") != nm]
            obs = {"name": nm, "x": float(x), "y": float(y), "r": float(max(0.15, r)), "kind": kind}
            self.dyn_obstacles.append(obs)
            return {"success": True, "obstacle": obs}

    def remove_dyn_obstacle(self, name: str) -> Dict[str, Any]:
        with self.lock:
            before = len(self.dyn_obstacles)
            self.dyn_obstacles = [o for o in self.dyn_obstacles if o.get("name") != name]
            return {"success": len(self.dyn_obstacles) < before, "name": name}

    def clear_dyn_obstacles(self) -> Dict[str, Any]:
        with self.lock:
            n = len(self.dyn_obstacles)
            self.dyn_obstacles = []
            return {"success": True, "removed": n}

    def clear_scenario(self) -> Dict[str, Any]:
        """Remove injected static + scripted movers; does not restore default actors."""
        with self.lock:
            nd = len(self.dyn_obstacles)
            nm = len(self.scenario_movers)
            self.dyn_obstacles = []
            self.scenario_movers = []
        return {"success": True, "dyn_removed": nd, "movers_removed": nm}

    def set_actors_enabled(self, enabled: bool) -> Dict[str, Any]:
        with self.lock:
            self._actors_enabled = bool(enabled)
            if not enabled:
                self.actors = []
        return {"success": True, "actors_enabled": bool(enabled)}

    def add_scenario_mover(
        self,
        name: str,
        x: float,
        y: float,
        r: float = 0.35,
        vx: float = 0.0,
        vy: float = 0.0,
        kind: str = "dynamic",
    ) -> Dict[str, Any]:
        with self.lock:
            nm = name or f"mover_{len(self.scenario_movers) + 1}"
            self.scenario_movers = [m for m in self.scenario_movers if m.get("name") != nm]
            mover = {
                "name": nm,
                "kind": kind,
                "x": float(x),
                "y": float(y),
                "r": float(max(0.15, r)),
                "vx": float(vx),
                "vy": float(vy),
            }
            self.scenario_movers.append(mover)
            return {"success": True, "mover": dict(mover)}

    def scenario_mover_list(self) -> List[Dict[str, Any]]:
        with self.lock:
            return [dict(m) for m in self.scenario_movers]

    def step_scenario_movers(self, dt: float) -> None:
        with self.lock:
            m = self.map
            if m is None or not self.scenario_movers:
                return
            for mv in self.scenario_movers:
                nx = float(mv["x"]) + float(mv["vx"]) * dt
                ny = float(mv["y"]) + float(mv["vy"]) * dt
                if not m.in_bounds(nx, ny) or self._collides_unlocked(m, nx, ny, float(mv["r"]) + 0.12):
                    mv["vx"] = -float(mv["vx"])
                    mv["vy"] = -float(mv["vy"])
                    nx = float(mv["x"]) + float(mv["vx"]) * dt
                    ny = float(mv["y"]) + float(mv["vy"]) * dt
                mv["x"], mv["y"] = nx, ny

    def obstacle_list(self) -> List[Dict[str, Any]]:
        with self.lock:
            return [dict(o) for o in self.dyn_obstacles]

    def clearance_at_xy(self, x: float, y: float, search_m: float = 3.5) -> float:
        """Actual vehicle clearance to nearest inflated occupancy (meters)."""
        with self.lock:
            m = self.map
            if m is None:
                return search_m
            occupied = m.occupied
            plan_res = float(m.plan_res)
            min_x, min_y = float(m.min_x), float(m.min_y)
        cell = (
            int(math.floor((x - min_x) / plan_res)),
            int(math.floor((y - min_y) / plan_res)),
        )
        return float(clearance_m_at_cell(cell, occupied, plan_res, min_x, min_y, search_m=search_m))

    def nearest_obstacle_info(
        self, x: float, y: float, yaw: float = 0.0, search_m: float = 4.0
    ) -> Dict[str, Any]:
        """Nearest occupied cell + bearing relative to vehicle yaw (diagnostics only)."""
        with self.lock:
            m = self.map
            if m is None:
                return {
                    "nearest_obstacle_distance": search_m,
                    "nearest_obstacle_direction": 0.0,
                    "nearest_obstacle_point": None,
                    "vehicle_to_obstacle_clearance": search_m,
                }
            occupied = m.occupied
            plan_res = float(m.plan_res)
            min_x, min_y = float(m.min_x), float(m.min_y)
            dyn = list(self.dyn_obstacles)

        best_d = search_m
        best_pt = None
        cell = (
            int(math.floor((x - min_x) / plan_res)),
            int(math.floor((y - min_y) / plan_res)),
        )
        rmax = max(1, int(math.ceil(search_m / plan_res)))
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
                    wx = min_x + (c2[0] + 0.5) * plan_res
                    wy = min_y + (c2[1] + 0.5) * plan_res
                    d = math.hypot(wx - x, wy - y)
                    if d < best_d:
                        best_d = d
                        best_pt = (wx, wy)
                        found = True
            if found and best_d <= (r + 0.5) * plan_res:
                break
        for o in dyn:
            ox, oy, rr = float(o["x"]), float(o["y"]), float(o["r"])
            d = max(0.0, math.hypot(ox - x, oy - y) - rr)
            if d < best_d:
                best_d = d
                best_pt = (ox, oy)
        bearing = 0.0
        if best_pt is not None:
            bearing = math.atan2(best_pt[1] - y, best_pt[0] - x) - yaw
            while bearing > math.pi:
                bearing -= 2 * math.pi
            while bearing < -math.pi:
                bearing += 2 * math.pi
        return {
            "nearest_obstacle_distance": round(best_d, 3),
            "nearest_obstacle_direction": round(bearing, 4),
            "nearest_obstacle_point": {"x": best_pt[0], "y": best_pt[1]} if best_pt else None,
            "vehicle_to_obstacle_clearance": round(best_d, 3),
        }

    def _reset_dynamic_actors(self) -> None:
        """按当前场景生成可移动参与物（行人/巡游小车）。"""
        assert self.map is not None
        m = self.map
        cx = (m.min_x + m.max_x) * 0.5
        cy = (m.min_y + m.max_y) * 0.5
        actors: List[Dict[str, Any]] = []
        if m.scene_kind == "indoor":
            seeds = [
                ("ped_a", cx - 2.0, cy, 0.28, 0.35, 0.55),
                ("ped_b", cx + 3.0, cy - 1.5, 0.28, 0.45, -0.4),
                ("cart_1", cx + 1.0, cy + 2.0, 0.45, 0.25, 0.9),
            ]
        else:
            seeds = [
                ("ped_a", cx - 8.0, cy + 4.0, 0.3, 0.8, 0.3),
                ("ped_b", cx + 6.0, cy - 5.0, 0.3, 0.7, -0.5),
                ("vru_car", cx - 12.0, cy, 0.9, 1.2, 0.2),
                ("vru_bike", cx + 10.0, cy + 8.0, 0.4, 1.5, -0.8),
            ]
        for name, x, y, r, spd, ang in seeds:
            # 找附近自由点
            sx, sy = x, y
            for k in range(20):
                if not self._collides_unlocked(m, sx, sy, r + 0.2):
                    break
                sx = x + (k % 5) * 0.5
                sy = y + (k // 5) * 0.5
            actors.append(
                {
                    "id": name,
                    "kind": "pedestrian" if name.startswith("ped") else "vehicle",
                    "x": sx,
                    "y": sy,
                    "r": r,
                    "yaw": ang,
                    "speed": spd,
                    "phase": hash(name) % 100 / 10.0,
                }
            )
        self.actors = actors
        self._actor_last_t = time.time()

    def step_actors(self, dt: float | None = None) -> None:
        use_dt = float(dt if dt is not None else 0.05)
        self.step_scenario_movers(use_dt)
        with self.lock:
            m = self.map
            if m is None or not self.actors or not self._actors_enabled:
                return
            now = time.time()
            use_dt = float(dt if dt is not None else max(0.02, min(0.2, now - self._actor_last_t)))
            self._actor_last_t = now
            for a in self.actors:
                a["phase"] = float(a.get("phase", 0.0)) + use_dt
                # 缓慢转向 + 前进，撞墙则掉头
                yaw = float(a["yaw"]) + 0.15 * math.sin(float(a["phase"]) * 0.7) * use_dt
                spd = float(a["speed"])
                nx = float(a["x"]) + math.cos(yaw) * spd * use_dt
                ny = float(a["y"]) + math.sin(yaw) * spd * use_dt
                if self._collides_unlocked(m, nx, ny, float(a["r"]) + 0.15) or not m.in_bounds(nx, ny):
                    yaw += math.pi * 0.6
                    nx = float(a["x"]) + math.cos(yaw) * spd * use_dt * 0.5
                    ny = float(a["y"]) + math.sin(yaw) * spd * use_dt * 0.5
                a["x"], a["y"], a["yaw"] = nx, ny, yaw

    def actor_list(self) -> List[Dict[str, Any]]:
        with self.lock:
            self.step_actors()
            return [
                {
                    "id": a["id"],
                    "kind": a["kind"],
                    "x": float(a["x"]),
                    "y": float(a["y"]),
                    "r": float(a["r"]),
                    "yaw": float(a["yaw"]),
                }
                for a in self.actors
            ]

    def _moving_circles(self) -> List[Tuple[float, float, float]]:
        """动态障碍 + 参与物 + 场景脚本 mover，供碰撞/雷达。"""
        circles = [(float(o["x"]), float(o["y"]), float(o["r"])) for o in self.dyn_obstacles]
        for a in self.actors:
            circles.append((float(a["x"]), float(a["y"]), float(a["r"])))
        for mv in self.scenario_movers:
            circles.append((float(mv["x"]), float(mv["y"]), float(mv["r"])))
        return circles

    def map_cloud(self, max_n: int = 8000) -> List[Dict[str, float]]:
        with self.lock:
            assert self.map is not None
            cloud = self.map.cloud
            if len(cloud) > max_n:
                step = max(1, len(cloud) // max_n)
                cloud = cloud[::step]
            return [{"x": x, "y": y, "z": 0.05} for x, y in cloud]

    def local_cloud(
        self, x: float, y: float, radius: float = 25.0, max_n: int = 6000
    ) -> List[Dict[str, float]]:
        """车周围静态地图点云（未导航时的全景底景用）。"""
        with self.lock:
            assert self.map is not None
            r2 = radius * radius
            out: List[Dict[str, float]] = []
            for px, py in self.map.cloud:
                dx, dy = px - x, py - y
                if dx * dx + dy * dy <= r2:
                    out.append({"x": px, "y": py, "z": 0.05})
                    if len(out) >= max_n:
                        break
            return out

    # ---- chronicle ----
    def emit(self, code: str, extra_reason: str = "", level: str = "info", min_interval_s: float = 1.2) -> None:
        now = time.time()
        with self.lock:
            last = self._last_codes.get(code, 0.0)
            if now - last < min_interval_s and code not in (
                "goal_set",
                "arrived",
                "blocked",
                "nav_stop",
                "nav_decision",
                "cancel",
                "planned",
                "confirm_go",
                "scene_indoor",
                "scene_outdoor",
            ):
                return
            self._last_codes[code] = now
            classical, reason = CHRONICLE_TEMPLATES.get(code, ("事出非常", code))
            # 停车/决策类：extra 为真实主因，避免模板误导成「前方阻塞」
            if extra_reason and code in ("blocked", "nav_stop", "nav_decision", "api_stop"):
                reason = extra_reason
            elif extra_reason:
                reason = f"{reason}；{extra_reason}"
            self.chronicle.appendleft(
                ChronicleEvent(ts=now, code=code, classical=classical, reason=reason, level=level)
            )

    def recent_chronicle(self, n: int = 12) -> List[Dict[str, Any]]:
        with self.lock:
            return [e.to_dict() for e in list(self.chronicle)[:n]]

    def current_banner(self) -> Dict[str, Any]:
        with self.lock:
            if not self.chronicle:
                classical, reason = CHRONICLE_TEMPLATES["idle"]
                return {"classical": classical, "reason": reason, "code": "idle", "level": "info"}
            return self.chronicle[0].to_dict()

    # ---- occupancy helpers ----
    @staticmethod
    def _collides_unlocked(m: LoadedSmap, x: float, y: float, robot_r: float) -> bool:
        cells = max(1, int(math.ceil(robot_r / m.plan_res)))
        ix0, iy0 = m.world_to_cell(x, y)
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:
                    continue
                if (ix0 + dx, iy0 + dy) in m.occupied:
                    return True
        if not m.in_bounds(x, y):
            return True
        return False

    def collides(self, x: float, y: float, robot_r: float = 0.35, include_actors: bool = True) -> bool:
        with self.lock:
            assert self.map is not None
            if self._collides_unlocked(self.map, x, y, robot_r):
                return True
            for o in self.dyn_obstacles:
                if math.hypot(x - float(o["x"]), y - float(o["y"])) < robot_r + float(o["r"]):
                    return True
            if include_actors:
                for a in self.actors:
                    if math.hypot(x - float(a["x"]), y - float(a["y"])) < robot_r + float(a["r"]):
                        return True
            return False

    def cast_ray(
        self, ox: float, oy: float, theta: float, max_range: float = 30.0, use_inflated: bool = False
    ) -> float:
        """射线测距。雷达默认打未膨胀栅格（贴近可视点云）；规划碰撞仍用膨胀层。"""
        with self.lock:
            m = self.map
            assert m is not None
            occupied = m.occupied if use_inflated else (m.occupied_raw or m.occupied)
            plan_res = m.plan_res
            min_x, min_y, max_x, max_y = m.min_x, m.min_y, m.max_x, m.max_y
            circles = self._moving_circles()

        dx, dy = math.cos(theta), math.sin(theta)
        step = max(0.04, plan_res * 0.4)
        t = 0.05  # 跳过机身近距离自击
        best = max_range
        while t < best:
            t += step
            x = ox + dx * t
            y = oy + dy * t
            if x < min_x or x > max_x or y < min_y or y > max_y:
                best = min(best, t)
                break
            ix = int(math.floor((x - min_x) / plan_res))
            iy = int(math.floor((y - min_y) / plan_res))
            if (ix, iy) in occupied:
                best = t
                break
        # 动态圆障必须与栅格一起参与最近点，不能在栅格 early-return 后被跳过
        for cx, cy, r in circles:
            fx, fy = ox - cx, oy - cy
            a = dx * dx + dy * dy
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4 * a * c
            if disc < 0:
                continue
            sq = math.sqrt(disc)
            for tt in ((-b - sq) / (2 * a), (-b + sq) / (2 * a)):
                if 0.08 < tt < best:
                    best = tt
        return best

    def dual_lidar(
        self,
        x: float,
        y: float,
        yaw: float,
        angle_min_deg: float = -135.0,
        angle_max_deg: float = 135.0,
        step_deg: float = 1.0,
        max_range: float = 30.0,
    ) -> Dict[str, Any]:
        front_off = (0.35, 0.0, 0.0)
        rear_off = (-0.35, 0.0, math.pi)

        def one(ox: float, oy: float, oyaw: float, name: str) -> Dict[str, Any]:
            sx = x + ox * math.cos(yaw) - oy * math.sin(yaw)
            sy = y + ox * math.sin(yaw) + oy * math.cos(yaw)
            base = yaw + oyaw
            beams = []
            deg = angle_min_deg
            while deg <= angle_max_deg + 1e-6:
                th = base + math.radians(deg)
                d = self.cast_ray(sx, sy, th, max_range=max_range, use_inflated=False)
                beams.append({"angle": float(deg), "dist": float(d), "valid": d < max_range * 0.99})
                deg += step_deg
            return {
                "device_name": name,
                "min_angle": angle_min_deg,
                "max_angle": angle_max_deg,
                "max_range": max_range,
                "install_info": {"x": ox, "y": oy, "yaw": oyaw},
                "beams": beams,
            }

        front = one(*front_off, "lidar_front")
        rear = one(*rear_off, "lidar_rear")

        def _near(beams, half_deg: float = 22.0) -> float:
            # 取前方扇区第2近，抑制单点噪声
            dists = sorted(b["dist"] for b in beams if abs(b["angle"]) <= half_deg)
            if not dists:
                return max_range
            if len(dists) == 1:
                return dists[0]
            return dists[min(1, len(dists) - 1)]

        return {
            "front": front,
            "rear": rear,
            "front_near": _near(front["beams"]),
            "rear_near": _near(rear["beams"]),
        }

    def lidar_world_points(
        self, x: float, y: float, yaw: float, stride: int = 1, step_deg: float = 1.0
    ) -> List[Dict[str, float]]:
        dual = self.dual_lidar(x, y, yaw, step_deg=step_deg)
        pts: List[Dict[str, float]] = []
        for laser in (dual["front"], dual["rear"]):
            info = laser["install_info"]
            ox, oy, oyaw = float(info["x"]), float(info["y"]), float(info["yaw"])
            sx = x + ox * math.cos(yaw) - oy * math.sin(yaw)
            sy = y + ox * math.sin(yaw) + oy * math.cos(yaw)
            base = yaw + oyaw
            for i, b in enumerate(laser["beams"]):
                if stride > 1 and i % stride:
                    continue
                if not b.get("valid", True):
                    continue
                th = base + math.radians(float(b["angle"]))
                d = float(b["dist"])
                pts.append({"x": sx + d * math.cos(th), "y": sy + d * math.sin(th), "z": 0.35})
        return pts

    def plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        robot_r: float = 0.25,
        avoid_dynamic: bool = False,
        forbid_circles: Optional[List[Tuple[float, float, float]]] = None,
        *,
        use_clearance: bool = True,
        use_turn: bool = True,
        use_los: bool = True,
        use_smooth: bool = True,
        variant: str = "C",
    ) -> List[Tuple[float, float]]:
        """网格 A* + 路径质量后处理。forbid_circles=(x,y,r) 封禁走廊。

        默认 variant C: clearance + turn + LOS (+ light smooth)。
        兼容旧调用：仍返回 List[path]；metrics 在 self._last_plan_metrics。
        """
        result = self.plan_path_quality(
            start,
            goal,
            robot_r=robot_r,
            avoid_dynamic=avoid_dynamic,
            forbid_circles=forbid_circles,
            use_clearance=use_clearance,
            use_turn=use_turn,
            use_los=use_los,
            use_smooth=use_smooth,
            variant=variant,
        )
        return result.path

    def plan_path_quality(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        robot_r: float = 0.25,
        avoid_dynamic: bool = False,
        forbid_circles: Optional[List[Tuple[float, float, float]]] = None,
        *,
        use_clearance: bool = True,
        use_turn: bool = True,
        use_los: bool = True,
        use_smooth: bool = True,
        variant: str = "C",
    ) -> PlanResult:
        radii = [robot_r] + [rr for rr in (0.22, 0.18, 0.14) if rr < robot_r]
        last = PlanResult()
        for rr in radii:
            raw, cost_info = self._plan_path_once(
                start,
                goal,
                robot_r=rr,
                avoid_dynamic=avoid_dynamic,
                forbid_circles=forbid_circles,
                use_clearance=use_clearance,
                use_turn=use_turn,
            )
            if not raw:
                continue
            processed, metrics = self._postprocess_path(
                raw,
                start,
                goal,
                robot_r=rr,
                avoid_dynamic=avoid_dynamic,
                forbid_circles=forbid_circles,
                use_los=use_los,
                use_smooth=use_smooth,
                cost_info=cost_info,
                variant=variant,
            )
            last = PlanResult(raw_path=raw, path=processed, metrics=metrics)
            break
        with self.lock:
            self._last_raw_path = list(last.raw_path)
            self._last_plan_result = last
            self._last_plan_metrics = last.metrics.to_dict() if last.path else None
        return last

    def _make_free_xy(
        self,
        robot_r: float,
        avoid_dynamic: bool,
        forbid_circles: Optional[List[Tuple[float, float, float]]],
    ):
        with self.lock:
            m = self.map
            assert m is not None
            min_x, min_y, max_x, max_y = m.min_x, m.min_y, m.max_x, m.max_y
        bans = list(forbid_circles or [])

        def free_xy(x: float, y: float) -> bool:
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return False
            for bx, by, br in bans:
                if math.hypot(x - bx, y - by) < br + robot_r * 0.35:
                    return False
            return not self.collides(x, y, robot_r=robot_r, include_actors=avoid_dynamic)

        return free_xy

    def _postprocess_path(
        self,
        raw: List[Tuple[float, float]],
        start: Tuple[float, float],
        goal: Tuple[float, float],
        *,
        robot_r: float,
        avoid_dynamic: bool,
        forbid_circles: Optional[List[Tuple[float, float, float]]],
        use_los: bool,
        use_smooth: bool,
        cost_info: Dict[str, float],
        variant: str,
    ) -> Tuple[List[Tuple[float, float]], PathQualityMetrics]:
        with self.lock:
            m = self.map
            assert m is not None
            plan_res = m.plan_res
            occupied = m.occupied
            min_x, min_y = m.min_x, m.min_y

        free_xy = self._make_free_xy(robot_r, avoid_dynamic, forbid_circles)
        step = max(0.05, plan_res * 0.4)

        def segment_safe(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
            return is_direct_path_safe(free_xy, a, b, step)

        direct = is_direct_path_safe(free_xy, start, goal, step)
        removed = 0
        path = list(raw)
        if use_los and len(path) >= 3:
            path, removed = los_simplify(path, segment_safe)
            # densify along LOS segments so PP/progress have enough samples
            path = densify_path(path, spacing_m=0.55)
        # 不要把 path[0]/path[-1] 硬改成精确 start/goal：
        # 会把走廊中心线拉到车辆瞬时位姿所在的贴边直线。
        if path and math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) > 0.12:
            if segment_safe(path[-1], (float(goal[0]), float(goal[1]))):
                path.append((float(goal[0]), float(goal[1])))

        sm_applied = False
        sm_valid = True
        if use_smooth and len(path) >= 3:
            path, sm_applied, sm_valid = smooth_path_collision_checked(
                path, segment_safe, free_xy
            )

        def clr_at(x: float, y: float) -> float:
            c = (
                int(math.floor((x - min_x) / plan_res)),
                int(math.floor((y - min_y) / plan_res)),
            )
            return clearance_m_at_cell(c, occupied, plan_res, min_x, min_y)

        start_c = (
            int(math.floor((start[0] - min_x) / plan_res)),
            int(math.floor((start[1] - min_y) / plan_res)),
        )
        goal_c = (
            int(math.floor((goal[0] - min_x) / plan_res)),
            int(math.floor((goal[1] - min_y) / plan_res)),
        )
        goal_snap = (min_x + (goal_c[0] + 0.5) * plan_res, min_y + (goal_c[1] + 0.5) * plan_res)
        metrics = metrics_for_paths(
            raw,
            path,
            start,
            goal,
            direct_safe=direct,
            removed=removed,
            smoothing_applied=sm_applied,
            smoothing_valid=sm_valid,
            clearance_at=clr_at,
            planner_cost=float(cost_info.get("planner_cost", 0.0)),
            clearance_cost=float(cost_info.get("clearance_cost", 0.0)),
            turn_cost=float(cost_info.get("turn_cost", 0.0)),
            start_grid=start_c,
            goal_grid=goal_c,
            goal_snap_error=math.hypot(goal_snap[0] - goal[0], goal_snap[1] - goal[1]),
            variant=variant,
        )
        return path, metrics

    def compare_path_variants(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        robot_r: float = 0.25,
    ) -> Dict[str, Any]:
        """A/B/C/D path quality comparison for same start/goal."""
        specs = [
            ("A", False, False, False, False),
            ("B", True, False, False, False),
            ("C", True, True, True, False),
            ("D", True, True, True, True),
        ]
        out: Dict[str, Any] = {}
        for name, clr, turn, los, sm in specs:
            r = self.plan_path_quality(
                start,
                goal,
                robot_r=robot_r,
                use_clearance=clr,
                use_turn=turn,
                use_los=los,
                use_smooth=sm,
                variant=name,
            )
            m = r.metrics
            out[name] = {
                "length": round(m.final_path_length, 3),
                "raw_length": round(m.raw_path_length, 3),
                "ratio": round(m.path_ratio, 3),
                "turns": m.final_turn_count,
                "max_turn": round(m.max_turn_angle, 1),
                "min_clearance": round(m.min_clearance, 3),
                "mean_clearance": round(m.mean_clearance, 3),
                "direct_safe": m.direct_path_safe,
                "n_points": m.n_final_points,
                "removed": m.simplification_removed_points,
                "smoothing_applied": m.smoothing_applied,
                "smoothing_valid": m.smoothing_valid,
                "path": [{"x": p[0], "y": p[1]} for p in r.path],
                "raw_path": [{"x": p[0], "y": p[1]} for p in r.raw_path],
            }
        return out

    @staticmethod
    def path_overlap_ratio(
        a: List[Tuple[float, float]], b: List[Tuple[float, float]], tol: float = 0.55
    ) -> float:
        """估计两条路径重合比例（0~1）。用于拒绝「换汤不换药」的全局重规划。"""
        if not a or not b:
            return 0.0
        step = max(1, len(a) // 24)
        hits = 0
        n = 0
        for p in a[::step]:
            n += 1
            if min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in b) <= tol:
                hits += 1
        return hits / max(1, n)

    def _nearest_path_index(self, path: List[Tuple[float, float]], p: Tuple[float, float]) -> int:
        best_i, best_d = 0, 1e18
        for i, q in enumerate(path):
            d = math.hypot(q[0] - p[0], q[1] - p[1])
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _point_along_path(
        self, path: List[Tuple[float, float]], from_i: int, dist_m: float
    ) -> Tuple[Tuple[float, float], int]:
        if not path:
            return (0.0, 0.0), 0
        acc = 0.0
        prev = path[from_i]
        for j in range(from_i, len(path)):
            q = path[j]
            acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
            if acc >= dist_m:
                return q, j
        return path[-1], len(path) - 1

    def _plan_path_once(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        robot_r: float = 0.25,
        avoid_dynamic: bool = False,
        forbid_circles: Optional[List[Tuple[float, float, float]]] = None,
        *,
        use_clearance: bool = True,
        use_turn: bool = True,
    ) -> Tuple[List[Tuple[float, float]], Dict[str, float]]:
        """8-邻域 A*。可选 clearance / turn soft cost。状态可带 previous direction。"""
        import heapq

        with self.lock:
            m = self.map
            assert m is not None
            occupied = m.occupied
            plan_res = m.plan_res
            min_x, min_y, max_x, max_y = m.min_x, m.min_y, m.max_x, m.max_y

        free_xy = self._make_free_xy(robot_r, avoid_dynamic, forbid_circles)

        def cell(p: Tuple[float, float]) -> Tuple[int, int]:
            return (
                int(math.floor((p[0] - min_x) / plan_res)),
                int(math.floor((p[1] - min_y) / plan_res)),
            )

        def world(c: Tuple[int, int]) -> Tuple[float, float]:
            return (min_x + (c[0] + 0.5) * plan_res, min_y + (c[1] + 0.5) * plan_res)

        def free_c(c: Tuple[int, int]) -> bool:
            return free_xy(*world(c))

        start_c, goal_c = cell(start), cell(goal)
        if not free_c(goal_c):
            found = None
            for r in range(1, 48):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        c2 = (goal_c[0] + dx, goal_c[1] + dy)
                        if free_c(c2):
                            found = c2
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return [], {}
            goal_c = found

        if not free_c(start_c):
            found = None
            for r in range(1, 48):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        c2 = (start_c[0] + dx, start_c[1] + dy)
                        if free_c(c2):
                            found = c2
                            break
                    if found:
                        break
                if found:
                    break
            if found:
                start_c = found

        clr_cache: Dict[Tuple[int, int], float] = {}

        def cell_clearance(c: Tuple[int, int]) -> float:
            if c not in clr_cache:
                clr_cache[c] = clearance_m_at_cell(c, occupied, plan_res, min_x, min_y)
            return clr_cache[c]

        # state: (cx, cy, dir) dir=-1 at start
        start_state = (start_c[0], start_c[1], -1)
        gscore: Dict[Tuple[int, int, int], float] = {start_state: 0.0}
        came: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
        g_clear: Dict[Tuple[int, int, int], float] = {start_state: 0.0}
        g_turn: Dict[Tuple[int, int, int], float] = {start_state: 0.0}
        counter = 0
        h0 = math.hypot(goal_c[0] - start_c[0], goal_c[1] - start_c[1])
        # heap: (f, -clearance, counter, g, state) — tie-break prefers higher clearance
        open_heap: List[Tuple[float, float, int, float, Tuple[int, int, int]]] = []
        heapq.heappush(open_heap, (h0, -cell_clearance(start_c), counter, 0.0, start_state))
        closed: set = set()
        steps = 0
        max_steps = 120000
        best_goal: Optional[Tuple[int, int, int]] = None

        while open_heap and steps < max_steps:
            steps += 1
            _f, _nc, _ctr, gcur, state = heapq.heappop(open_heap)
            if state in closed:
                continue
            if gcur > gscore.get(state, 1e18) + 1e-9:
                continue
            closed.add(state)
            cx, cy, pdir = state
            if (cx, cy) == goal_c:
                best_goal = state
                break
            for di, (dx, dy) in enumerate(DIRS8):
                nb = (cx + dx, cy + dy)
                if not free_c(nb):
                    continue
                ndir = di if use_turn else -1
                nstate = (nb[0], nb[1], ndir if use_turn else -1)
                # When not using turn, collapse direction
                if not use_turn:
                    nstate = (nb[0], nb[1], -1)
                step_len = math.hypot(dx, dy)
                c_cost = (
                    clearance_cost_at(cell_clearance(nb), CLEARANCE_PREF_M, CLEARANCE_WEIGHT)
                    if use_clearance
                    else 0.0
                )
                t_cost = turn_cost_dirs(pdir, di, TURN_WEIGHT) if use_turn else 0.0
                tent = gscore[state] + step_len + c_cost + t_cost
                if tent + 1e-12 < gscore.get(nstate, 1e18):
                    came[nstate] = state
                    gscore[nstate] = tent
                    g_clear[nstate] = g_clear[state] + c_cost
                    g_turn[nstate] = g_turn[state] + t_cost
                    f = tent + math.hypot(goal_c[0] - nb[0], goal_c[1] - nb[1])
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (f, -cell_clearance(nb), counter, tent, nstate),
                    )

        if best_goal is None:
            return [], {}

        # reconstruct cells
        cur = best_goal
        path_states = [cur]
        while cur in came:
            cur = came[cur]
            path_states.append(cur)
        path_states.reverse()
        path_c = [(s[0], s[1]) for s in path_states]
        # dedupe consecutive identical cells (dir-only changes shouldn't happen)
        dedup: List[Tuple[int, int]] = []
        for c in path_c:
            if not dedup or dedup[-1] != c:
                dedup.append(c)
        raw = [world(c) for c in dedup]
        info = {
            "planner_cost": float(gscore[best_goal]),
            "clearance_cost": float(g_clear.get(best_goal, 0.0)),
            "turn_cost": float(g_turn.get(best_goal, 0.0)),
        }
        return raw, info

    def replan_global_repair(
        self,
        start: Tuple[float, float],
        old_path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        skip_front_m: float = 3.5,
        reconnect_ms: Optional[List[float]] = None,
    ) -> Dict[str, Any]:
        """局部修补全局：只重规划到旧路上更远的汇合点，保留后半段。

        适用于「前面几米不通，后面走廊仍可用」。
        """
        if not old_path:
            p = self.plan_path(start, goal)
            return {"path": p, "mode": "full", "kept_tail": False}
        reconnect_ms = reconnect_ms or [4.0, 7.0, 11.0, 16.0]
        i0 = self._nearest_path_index(old_path, start)
        for dist in reconnect_ms:
            # 跳过紧贴前方 skip_front_m，避免汇合点仍在堵死段
            join_pt, join_i = self._point_along_path(old_path, i0, max(dist, skip_front_m))
            if join_i <= i0:
                continue
            prefix = self.plan_path(start, join_pt, avoid_dynamic=False)
            if not prefix or len(prefix) < 2:
                continue
            tail = old_path[join_i:]
            # 拼接：去掉重复接点
            merged = prefix[:-1] + tail if tail else prefix
            if math.hypot(merged[-1][0] - goal[0], merged[-1][1] - goal[1]) > 0.6:
                # 尾段未到目标则补一段到目标（尽量短）
                fin = self.plan_path(merged[-1], goal, avoid_dynamic=False)
                if fin:
                    merged = merged[:-1] + fin
            return {
                "path": merged,
                "mode": "splice",
                "kept_tail": True,
                "reconnect_m": dist,
                "join_i": join_i,
            }
        # 修补失败 → 完整重规划（无封禁）
        full = self.plan_path(start, goal)
        return {"path": full, "mode": "full_fallback", "kept_tail": False}

    def replan_global_alternative(
        self,
        start: Tuple[float, float],
        old_path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        ban_front_m: float = 5.0,
        ban_radius: float = 0.7,
        max_overlap: float = 0.72,
        prefer_splice: bool = True,
    ) -> Dict[str, Any]:
        """卡住久了：封禁旧路前方走廊，强制走出不同全局路线；拒绝高度重复。"""
        # 1) 可选：先试拼接修补（仅「前方几米」场景；强制换拓扑时关掉）
        if prefer_splice:
            repair = self.replan_global_repair(start, old_path, goal)
            if repair.get("path") and repair.get("mode") == "splice":
                ov = self.path_overlap_ratio(repair["path"], old_path) if old_path else 0.0
                repair["overlap"] = ov
                return repair

        forbid: List[Tuple[float, float, float]] = []
        if old_path:
            i0 = self._nearest_path_index(old_path, start)
            acc = 0.0
            prev = old_path[i0]
            for q in old_path[i0:]:
                acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
                prev = q
                if acc > 0.4:
                    forbid.append((q[0], q[1], ban_radius))
                if acc >= ban_front_m:
                    break

        # 2) 封禁旧前方后全图重规划
        alt = self.plan_path(start, goal, forbid_circles=forbid or None)
        if alt:
            ov = self.path_overlap_ratio(alt, old_path) if old_path else 0.0
            if ov <= max_overlap:
                return {"path": alt, "mode": "alt_ban", "kept_tail": False, "overlap": ov}

        # 3) 经侧向 via 点逼出另一拓扑
        if old_path:
            i0 = self._nearest_path_index(old_path, start)
            mid, _ = self._point_along_path(old_path, i0, 8.0)
            i1 = min(i0 + 1, len(old_path) - 1)
            tx = old_path[i1][0] - old_path[i0][0]
            ty = old_path[i1][1] - old_path[i0][1]
            L = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / L, tx / L
            for side in (1.8, -1.8, 3.0, -3.0, 4.5, -4.5):
                via = (mid[0] + nx * side, mid[1] + ny * side)
                p1 = self.plan_path(start, via, forbid_circles=forbid or None)
                p2 = self.plan_path(via, goal, forbid_circles=forbid or None) if p1 else []
                if p1 and p2:
                    alt2 = p1[:-1] + p2
                    ov = self.path_overlap_ratio(alt2, old_path)
                    if ov <= max_overlap:
                        return {
                            "path": alt2,
                            "mode": "alt_via",
                            "kept_tail": False,
                            "overlap": ov,
                            "via": via,
                        }

        # 4) 仍失败：返回封禁规划结果（即使重合偏高）或普通全规
        if alt:
            return {
                "path": alt,
                "mode": "alt_soft",
                "kept_tail": False,
                "overlap": self.path_overlap_ratio(alt, old_path) if old_path else 0.0,
            }
        full = self.plan_path(start, goal)
        return {
            "path": full,
            "mode": "full",
            "kept_tail": False,
            "overlap": self.path_overlap_ratio(full, old_path) if (full and old_path) else 0.0,
        }

    def local_replan(
        self,
        start: Tuple[float, float],
        global_path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        horizon_m: float = 5.0,
        robot_r: float = 0.22,
        yaw: float = 0.0,
    ) -> List[Tuple[float, float]]:
        ranked = self.local_replan_ranked(
            start, global_path, goal, horizon_m=horizon_m, robot_r=robot_r, yaw=yaw, top_n=10, stuck_s=0.0
        )
        return list(ranked[0]["path"]) if ranked else []

    def local_replan_ranked(
        self,
        start: Tuple[float, float],
        global_path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        horizon_m: float = 5.0,
        robot_r: float = 0.22,
        yaw: float = 0.0,
        top_n: int = 10,
        stuck_s: float = 0.0,
    ) -> List[Dict[str, Any]]:
        """一次产出最多 top_n 条局部假设，按置信度降序。

        含前瞻/侧移/几何倒车。stuck_s≥10 时倒车置信大幅提权。
        """
        sx, sy = start
        if not global_path:
            p = self.plan_path(start, goal, robot_r=robot_r, avoid_dynamic=True)
            if not p:
                p = self.plan_path(start, goal, robot_r=robot_r, avoid_dynamic=False)
            if not p:
                return []
            return [
                {
                    "id": 1,
                    "label": "直达目标",
                    "confidence": 0.55,
                    "path": p,
                    "mode": "forward",
                    "horizon_m": horizon_m,
                }
            ]

        best_i, best_d = 0, 1e18
        for i, (px, py) in enumerate(global_path):
            d = math.hypot(px - sx, py - sy)
            if d < best_d:
                best_d, best_i = d, i

        def point_along(from_i: int, dist_m: float, backward: bool = False) -> Tuple[float, float]:
            if backward:
                acc = 0.0
                prev = global_path[from_i]
                for j in range(from_i, -1, -1):
                    p = global_path[j]
                    acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
                    prev = p
                    if acc >= abs(dist_m):
                        return p
                return global_path[0]
            acc = 0.0
            prev = global_path[from_i]
            for p in global_path[from_i:]:
                acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
                prev = p
                if acc >= dist_m:
                    return p
            return goal

        def tangent_normal_at(i: int) -> Tuple[float, float, float, float]:
            i0 = max(0, min(i, len(global_path) - 2))
            a, b = global_path[i0], global_path[min(i0 + 1, len(global_path) - 1)]
            tx_, ty_ = b[0] - a[0], b[1] - a[1]
            L = math.hypot(tx_, ty_) or 1.0
            tx_, ty_ = tx_ / L, ty_ / L
            return tx_, ty_, -ty_, tx_  # tangent, normal

        # 构建 ~10 个子目标种子
        seeds: List[Dict[str, Any]] = []
        for hz, name in ((3.0, "近前瞻3m"), (horizon_m, "前瞻5m"), (8.0, "远前瞻8m")):
            seeds.append({"label": name, "sub": point_along(best_i, hz), "mode": "forward", "hz": hz})

        mid = point_along(best_i, horizon_m)
        mi = best_i
        # 用 mid 附近索引估切向
        acc = 0.0
        prev = global_path[best_i]
        for j in range(best_i, len(global_path)):
            p = global_path[j]
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
            mi = j
            if acc >= horizon_m:
                break
        _, _, nx, ny = tangent_normal_at(mi)
        for lat, name in ((0.6, "右偏0.6"), (-0.6, "左偏0.6"), (1.2, "右偏1.2"), (-1.2, "左偏1.2")):
            seeds.append(
                {
                    "label": name,
                    "sub": (mid[0] + nx * lat, mid[1] + ny * lat),
                    "mode": "forward",
                    "hz": horizon_m,
                }
            )

        back_pt = point_along(best_i, 2.0, backward=True)
        seeds.append(
            {"label": "倒退2m再接回", "sub": mid, "mode": "reverse_then_forward", "via": back_pt, "hz": horizon_m}
        )
        seeds.append({"label": "直趋目标", "sub": goal, "mode": "forward", "hz": 99.0})
        for dist, name in ((0.9, "倒车0.9m"), (1.6, "倒车1.6m"), (2.4, "倒车2.4m")):
            escape = (sx - dist * math.cos(yaw), sy - dist * math.sin(yaw))
            seeds.append({"label": name, "sub": escape, "mode": "reverse", "hz": dist, "geom": True})

        candidates: List[Dict[str, Any]] = []
        seen: set = set()

        def _key(path: List[Tuple[float, float]]) -> Tuple:
            if not path:
                return ()
            return tuple((round(p[0], 1), round(p[1], 1)) for p in path[:: max(1, len(path) // 6)])

        for seed in seeds:
            path: List[Tuple[float, float]] = []
            dyn_used = True
            if seed.get("geom") and seed["mode"] == "reverse":
                sub = seed["sub"]
                # 始终保留几何倒车（后向格子常被膨胀封死，A* 出不来）
                if not self.collides(sub[0], sub[1], robot_r=0.14, include_actors=False):
                    path = [start, sub]
                    dyn_used = not self.collides(sub[0], sub[1], robot_r=0.18, include_actors=True)
                else:
                    soft = (
                        sx - float(seed.get("hz", 0.9)) * 0.5 * math.cos(yaw),
                        sy - float(seed.get("hz", 0.9)) * 0.5 * math.sin(yaw),
                    )
                    path = [start, soft]
                    dyn_used = False
                    seed = {**seed, "label": str(seed["label"]) + "·短"}
            else:
                for dyn in (True, False):
                    if seed["mode"] == "reverse_then_forward":
                        via = seed["via"]
                        p1 = self.plan_path(start, via, robot_r=robot_r, avoid_dynamic=dyn)
                        p2 = (
                            self.plan_path(via, seed["sub"], robot_r=robot_r, avoid_dynamic=dyn)
                            if p1
                            else []
                        )
                        path = (p1[:-1] + p2) if (p1 and p2) else (p1 or [])
                    else:
                        path = self.plan_path(start, seed["sub"], robot_r=robot_r, avoid_dynamic=dyn)
                    dyn_used = dyn
                    if path and len(path) >= 2:
                        break
            if not path or len(path) < 2:
                continue
            k = _key(path)
            if k in seen:
                continue
            seen.add(k)
            conf = self._score_local_path(
                start, yaw, path, goal, global_path, seed["mode"], dyn_used
            )
            candidates.append(
                {
                    "label": seed["label"] + ("" if dyn_used else "·静"),
                    "confidence": conf,
                    "path": path,
                    "mode": seed["mode"],
                    "horizon_m": float(seed.get("hz", horizon_m)),
                    "dynamic": dyn_used,
                }
            )

        if stuck_s >= 10.0:
            boost = min(0.55, 0.18 + (stuck_s - 10.0) * 0.03)
            for c in candidates:
                if "reverse" in str(c.get("mode", "")):
                    c["confidence"] = min(0.99, float(c["confidence"]) + boost)
                    c["label"] = f"{c['label']}·卡{stuck_s:.0f}s提权"
        elif stuck_s >= 5.0:
            boost = 0.06 + (stuck_s - 5.0) * 0.02
            for c in candidates:
                if "reverse" in str(c.get("mode", "")):
                    c["confidence"] = min(0.99, float(c["confidence"]) + boost)

        candidates.sort(key=lambda c: c["confidence"], reverse=True)
        out = candidates[:top_n]
        for i, c in enumerate(out):
            c["id"] = i + 1
            c["confidence"] = round(float(c["confidence"]), 4)
        return out

    def _score_local_path(
        self,
        start: Tuple[float, float],
        yaw: float,
        path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        global_path: List[Tuple[float, float]],
        mode: str,
        dynamic: bool,
    ) -> float:
        """置信度 0~1：短、顺车头、清障好、贴近全局者更高；倒车略罚但不禁用。"""
        if len(path) < 2:
            return 0.0
        sx, sy = start
        length = 0.0
        for i in range(1, len(path)):
            length += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        dx0, dy0 = path[1][0] - path[0][0], path[1][1] - path[0][1]
        head = math.atan2(dy0, dx0)
        err = abs((head - yaw + math.pi) % (2 * math.pi) - math.pi)
        ahead = math.cos(head - yaw)

        # 抽样与动态物距离
        clear = 2.0
        with self.lock:
            movers = self._moving_circles()
        for p in path[:: max(1, len(path) // 8)]:
            for mx, my, mr in movers:
                clear = min(clear, max(0.0, math.hypot(p[0] - mx, p[1] - my) - mr))

        # 相对全局的偏离
        gdev = 0.0
        if global_path:
            for p in path[:: max(1, len(path) // 5)]:
                gdev += min(math.hypot(p[0] - g[0], p[1] - g[1]) for g in global_path[:: max(1, len(global_path) // 20)])
            gdev /= max(1, len(path[:: max(1, len(path) // 5)]))

        goal_d0 = math.hypot(goal[0] - sx, goal[1] - sy) + 1e-3
        goal_d1 = math.hypot(goal[0] - path[-1][0], goal[1] - path[-1][1])
        progress = max(0.0, 1.0 - goal_d1 / goal_d0)

        score = 0.42
        score += 0.18 * max(0.0, 1.0 - length / 18.0)
        score += 0.14 * max(0.0, 1.0 - err / math.pi)
        score += 0.10 * max(0.0, ahead)  # 朝前加分
        score += 0.12 * min(1.0, clear / 1.2)
        score += 0.08 * max(0.0, 1.0 - gdev / 3.0)
        score += 0.08 * progress
        if not dynamic:
            score -= 0.06
        if mode == "reverse":
            score -= 0.04
        elif mode == "reverse_then_forward":
            score -= 0.02
        if ahead < -0.2:
            # 首段在身后：标成可倒车执行，置信略降但仍可用
            score -= 0.03
        return max(0.05, min(0.99, score))


_WORLD: Optional[SimWorld] = None


def get_world() -> SimWorld:
    global _WORLD
    if _WORLD is None:
        _WORLD = SimWorld()
    return _WORLD
