"""Keyence floor QR scanner bridge — ROS /scanner/barcode + trigger service."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

from rclpy.node import Node
from std_msgs.msg import String

from delivery_web.device_config import DeviceConfig
from delivery_web.localization import LocalizationResult, verify_localization
from delivery_web.vision.models import FloorQrScannerStatus
from delivery_web.vision.ros_probe import RosTopicProbe, ping_host


class FloorQrScannerBridge:
    """Subscribes to Keyence SR wrapper topic; no direct TCP from dashboard."""

    def __init__(
        self,
        node: Node,
        lock: threading.Lock,
        cfg: Optional[DeviceConfig] = None,
        get_robot_pose: Optional[Callable[[], Optional[Dict[str, float]]]] = None,
        get_stations: Optional[Callable[[], Dict[str, Dict[str, float]]]] = None,
    ) -> None:
        self._node = node
        self._lock = lock
        self._cfg = cfg or DeviceConfig.load()
        self._get_robot_pose = get_robot_pose
        self._get_stations = get_stations
        self._last_qr = ""
        self._last_scan_at = 0.0
        self._last_heard = 0.0
        self._net_ping_ok = False
        self._network_checked = 0.0
        self._topic_probe = RosTopicProbe(
            self._cfg.floor_qr_ros_topic,
            domain_id=self._cfg.ros_domain_id,
        )
        cg = getattr(node, "_cg", None)
        node.create_subscription(
            String,
            self._cfg.floor_qr_ros_topic,
            self._on_barcode,
            10,
            callback_group=cg,
        )
        from std_srvs.srv import Trigger

        self._trigger_client = node.create_client(
            Trigger,
            self._cfg.floor_qr_trigger_service,
            callback_group=cg,
        )

    def _on_barcode(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text or text.startswith("ER"):
            return
        now = time.time()
        with self._lock:
            self._last_qr = text
            self._last_scan_at = now
            self._last_heard = now

    def _network_online(self) -> bool:
        now = time.time()
        if (now - self._network_checked) > 10.0:
            self._net_ping_ok = ping_host(self._cfg.floor_qr_scanner_host)
            self._network_checked = now
        return self._net_ping_ok

    def _scanner_status(self) -> str:
        now = time.time()
        with self._lock:
            heard = self._last_heard
        ros_pub = self._topic_probe.publisher_count() >= 1 or heard > 0
        net = self._network_online()
        if heard > 0 and (now - heard) <= 30.0:
            return "ONLINE"
        if ros_pub:
            return "SCANNER READY"
        if net:
            return "SCANNER READY"
        return "OFFLINE"

    def status(self) -> Dict[str, Any]:
        st = self._scanner_status()
        ros_pub = self._topic_probe.publisher_count() >= 1
        net = self._network_online()
        with self._lock:
            out = FloorQrScannerStatus(
                host=self._cfg.floor_qr_scanner_host,
                protocol="keyence_tcp",
                status=st if st != "SCANNER READY" else "ONLINE",
                last_qr_id=self._last_qr,
                last_scan_at=self._last_scan_at,
                ros_topic=self._cfg.floor_qr_ros_topic,
            ).to_dict()
        out["network_online"] = net
        out["tcp_port_open"] = ros_pub
        out["ros_node_online"] = ros_pub
        out["ros_domain_id"] = self._cfg.ros_domain_id
        if st == "SCANNER READY" and not out.get("last_qr_id"):
            out["status"] = "SCANNER READY"
        return out

    def localization(self) -> Dict[str, Any]:
        st = self._scanner_status()
        with self._lock:
            qr_id = self._last_qr
        pose = (self._get_robot_pose() or {}) if self._get_robot_pose else {}
        rx = float(pose.get("x", 0.0) or 0.0)
        ry = float(pose.get("y", 0.0) or 0.0)
        ra = float(pose.get("angle", 0.0) or 0.0)
        result: LocalizationResult = verify_localization(
            qr_id,
            rx,
            ry,
            ra,
            scanner_status=st,
            thresholds=self._cfg.localization,
            stations=(self._get_stations() or {}) if self._get_stations else None,
        )
        return result.to_dict()

    def trigger_scan(self, timeout_sec: float = 8.0) -> Dict[str, Any]:
        try:
            from std_srvs.srv import Trigger

            cli = self._trigger_client
            if not cli.wait_for_service(timeout_sec=3.0):
                return {
                    "success": False,
                    "message": (
                        "scanner trigger service unavailable — start keyence_sr_node "
                        f"({self._cfg.floor_qr_scanner_host}:{self._cfg.floor_qr_scanner_port})"
                    ),
                }
            future = cli.call_async(Trigger.Request())
            deadline = time.time() + max(3.0, float(timeout_sec))
            try:
                import rclpy
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=max(3.0, float(timeout_sec)))
            except Exception:  # noqa: BLE001
                while not future.done() and time.time() < deadline:
                    time.sleep(0.03)
            if not future.done():
                return {
                    "success": False,
                    "message": "trigger timeout — scanner TCP no response (check 172.31.0.91:9004)",
                }
            res = future.result()
            msg = str(res.message or "")
            if not res.success and "not connected" in msg.lower():
                return {
                    "success": False,
                    "message": (
                        f"scanner offline at {self._cfg.floor_qr_scanner_host}:"
                        f"{self._cfg.floor_qr_scanner_port} — {msg}"
                    ),
                }
            return {
                "success": bool(res.success),
                "message": msg,
                "qr_id": msg if res.success else "",
            }
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": str(exc)}
