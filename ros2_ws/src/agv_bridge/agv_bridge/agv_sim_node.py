"""AGV simulation node — fully offline, uses stations from agv_downloaded smap."""

# DEPRECATION NOTICE (V0.1仿真版, 2026-08-16)
# ---------------------------------------------------------------------------
# agv_sim_node 是基于「站点导航 LM1-LM10」的旧仿真节点。
# 新底盘仿真请用: agv_control/agv_control_bridge (cmd_vel + MockBackend)
# 新导航请用: agv_navigation/nav2_bringup
# 本节点保留仅用于旧 Web dashboard / Robokit 3051 回归测试。
# ---------------------------------------------------------------------------



from __future__ import annotations

import json
import math
import time
import uuid
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from delivery_interfaces.msg import AgvPose, AgvStatus
from delivery_interfaces.srv import AgvCancel, AgvLock, AgvNavigate


# Fallback if no map file is found (still offline)
DEFAULT_STATIONS: Dict[str, Tuple[float, float, float]] = {
    "LM1": (6.26, -0.659, 0.0),
    "LM2": (10.135, -0.659, -1.5708),
    "LM3": (10.135, -6.268, -3.1416),
    "LM4": (6.26, -6.268, 1.5708),
    "LM5": (4.506, 0.265, 0.0),
    "LM6": (1.617, 0.186, 0.0),
    "LM7": (-0.212, -0.875, 0.0),
    "LM10": (1.724, -7.071, 0.0),
}


def load_stations(path: str) -> Tuple[Dict[str, Tuple[float, float, float]], dict]:
    """Load stations JSON exported from .smap. Returns (stations, meta)."""
    meta = {"map_name": "offline_fallback", "vehicle_model": "AMB-150", "source": "builtin"}
    p = Path(path)
    if not path or not p.is_file():
        return dict(DEFAULT_STATIONS), meta
    data = json.loads(p.read_text(encoding="utf-8"))
    stations: Dict[str, Tuple[float, float, float]] = {}
    raw = data.get("stations", data)
    for name, v in raw.items():
        if not isinstance(v, dict):
            continue
        stations[name] = (
            float(v.get("x", 0.0)),
            float(v.get("y", 0.0)),
            float(v.get("yaw", v.get("angle", 0.0))),
        )
    if not stations:
        return dict(DEFAULT_STATIONS), meta
    meta = {
        "map_name": str(data.get("map_name", p.stem)),
        "map_file": str(data.get("map_file", p.name)),
        "vehicle_model": str(data.get("vehicle_model", "AMB-150")),
        "source": str(p),
    }
    return stations, meta


def parse_smap_stations(smap_path: str) -> Dict[str, Tuple[float, float, float]]:
    """Lightweight parse of Robokit .smap advancedPointList (no external deps)."""
    text = Path(smap_path).read_text(encoding="utf-8")
    data = json.loads(text)
    stations: Dict[str, Tuple[float, float, float]] = {}
    for p in data.get("advancedPointList", []) or []:
        name = p.get("instanceName") or ""
        if not name:
            continue
        pos = p.get("pos") or {}
        yaw = float(p.get("dir", p.get("angle", 0.0)) or 0.0)
        stations[name] = (float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
    return stations


class AgvSimNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_sim_node")
        self.declare_parameter("sim_speed_mps", 0.6)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("vehicle_id", "SIM-AMB-150")
        self.declare_parameter("start_station", "LM1")
        # Prefer exported JSON; can also point at raw .smap
        self.declare_parameter(
            "stations_file",
            "/data/agv_downloaded/maps/stations_from_smap.json",
        )
        self.declare_parameter(
            "smap_file",
            "/data/agv_downloaded/maps/20260723112931750.smap",
        )

        stations_file = self.get_parameter("stations_file").get_parameter_value().string_value
        smap_file = self.get_parameter("smap_file").get_parameter_value().string_value

        self._stations, meta = load_stations(stations_file)
        if meta.get("source") == "builtin" and Path(smap_file).is_file():
            try:
                parsed = parse_smap_stations(smap_file)
                if parsed:
                    self._stations = parsed
                    meta = {
                        "map_name": Path(smap_file).stem,
                        "map_file": Path(smap_file).name,
                        "vehicle_model": "AMB-150",
                        "source": smap_file,
                    }
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"smap parse failed, using fallback: {exc}")

        self._map_name = str(meta.get("map_name", "offline"))
        self._vehicle_model = str(meta.get("vehicle_model", "AMB-150"))

        self._cg = ReentrantCallbackGroup()
        start = self.get_parameter("start_station").get_parameter_value().string_value
        if start not in self._stations:
            start = next(iter(self._stations.keys()))
        x, y, a = self._stations[start]
        self._x, self._y, self._angle = x, y, a
        self._station = start
        self._last_station = ""
        self._task_status = 0
        self._target_id = ""
        self._task_id = ""
        self._locked = False
        self._nick = ""
        self._goal: Optional[Tuple[float, float, float]] = None
        self._unfinished = []
        self._finished = []

        self._pose_pub = self.create_publisher(AgvPose, "agv/pose", 10)
        self._status_pub = self.create_publisher(AgvStatus, "agv/status", 10)
        self.create_service(AgvNavigate, "agv/navigate", self._on_navigate, callback_group=self._cg)
        self.create_service(AgvCancel, "agv/cancel", self._on_cancel, callback_group=self._cg)
        self.create_service(AgvLock, "agv/lock", self._on_lock, callback_group=self._cg)

        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_timer)
        self.get_logger().info(
            f"[OFFLINE SIM] model={self._vehicle_model} map={self._map_name} "
            f"start={start} ({x:.3f},{y:.3f}) stations={sorted(self._stations)} "
            f"source={meta.get('source')}"
        )

    def _on_lock(self, req: AgvLock.Request, res: AgvLock.Response) -> AgvLock.Response:
        if req.lock:
            self._locked = True
            self._nick = req.nick_name or "sim"
            res.success = True
            res.message = f"locked by {self._nick}"
        else:
            self._locked = False
            self._nick = ""
            res.success = True
            res.message = "unlocked"
        return res

    def _on_cancel(self, _req: AgvCancel.Request, res: AgvCancel.Response) -> AgvCancel.Response:
        if self._task_status == 2:
            self._task_status = 6
            self._goal = None
            res.success = True
            res.message = "canceled"
        else:
            res.success = True
            res.message = "nothing to cancel"
        return res

    def _on_navigate(self, req: AgvNavigate.Request, res: AgvNavigate.Response) -> AgvNavigate.Response:
        target = req.target_id.strip()
        if target not in self._stations:
            res.success = False
            res.final_status = 5
            res.message = f"unknown station: {target}; known={sorted(self._stations)}"
            return res
        if not self._locked:
            self._locked = True
            self._nick = "sim_auto"

        self._task_id = req.task_id.strip() or str(uuid.uuid4())[:8]
        self._target_id = target
        self._goal = self._stations[target]
        self._task_status = 2
        self._finished = [self._station] if self._station else []
        self._unfinished = [target]
        self.get_logger().info(f"SIM navigate {self._station} -> {target} task_id={self._task_id}")

        if req.wait_until_done:
            timeout = req.timeout_sec if req.timeout_sec > 0 else 60.0
            deadline = time.time() + timeout
            while time.time() < deadline and self._task_status == 2:
                time.sleep(0.05)
            res.success = self._task_status == 4
            res.final_status = self._task_status
            res.message = "arrived" if res.success else f"status={self._task_status}"
            res.task_id = self._task_id
            return res

        res.success = True
        res.final_status = self._task_status
        res.message = "accepted"
        res.task_id = self._task_id
        return res

    def _on_timer(self) -> None:
        speed = self.get_parameter("sim_speed_mps").get_parameter_value().double_value
        dt = 1.0 / max(self.get_parameter("publish_hz").get_parameter_value().double_value, 1.0)

        if self._goal is not None and self._task_status == 2:
            gx, gy, ga = self._goal
            dx, dy = gx - self._x, gy - self._y
            dist = math.hypot(dx, dy)
            step = speed * dt
            if dist <= step:
                self._x, self._y, self._angle = gx, gy, ga
                self._last_station = self._station
                self._station = self._target_id
                self._task_status = 4
                self._goal = None
                self._unfinished = []
                self._finished = [self._station]
                self.get_logger().info(f"SIM arrived {self._station}")
            else:
                self._x += dx / dist * step
                self._y += dy / dist * step
                self._angle = math.atan2(dy, dx)

        now = self.get_clock().now().to_msg()
        pose = AgvPose()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.x = self._x
        pose.y = self._y
        pose.angle = self._angle
        pose.confidence = 0.99
        pose.current_station = self._station if self._task_status != 2 else ""
        pose.last_station = self._last_station
        self._pose_pub.publish(pose)

        st = AgvStatus()
        st.header.stamp = now
        st.header.frame_id = "map"
        st.task_status = self._task_status
        st.task_type = 3 if self._task_status in (2, 4) else 0
        st.target_id = self._target_id
        st.finished_path = list(self._finished)
        st.unfinished_path = list(self._unfinished)
        st.blocked = False
        st.emergency = False
        st.soft_emc = False
        st.battery_level = 0.95
        st.charging = False
        st.current_map = self._map_name
        st.vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        st.x = self._x
        st.y = self._y
        st.angle = self._angle
        st.confidence = 0.99
        st.current_station = pose.current_station
        st.mode = "sim_offline"
        self._status_pub.publish(st)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgvSimNode()
    # Multi-threaded: navigate(wait=True) must not block the motion timer
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
