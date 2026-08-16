"""Jason Basler real camera + QR web bridge for delivery_web dashboard."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

JASON_IMAGE_TOPIC = "/my_camera/pylon_ros2_camera_node/image_raw"
JASON_QR_TOPIC = "/wechat_qr_node/decoded_info"
JASON_OFFLINE_SEC = 8.0


class JasonCameraWebBridge:
    """ROS2 Jason camera stream + decoupled QR control for HTTP dashboard."""

    def __init__(
        self,
        node: Node,
        lock: threading.Lock,
        encode_jpeg: Callable[[Image], Optional[bytes]],
        logger: Callable[..., None],
    ) -> None:
        self._node = node
        self._lock = lock
        self._encode_jpeg = encode_jpeg
        self._log = logger
        self._qr_lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._last_frame = 0.0
        self._width = 0
        self._height = 0
        self._encoding = ""
        self._fps = 0.0
        self._frame_times: deque = deque(maxlen=60)
        self._qr_running = False
        self._qr_result = ""
        self._qr_updated = 0.0
        self._qr_proc: Optional[subprocess.Popen] = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        cg = getattr(node, "_cg", None)
        node.create_subscription(Image, JASON_IMAGE_TOPIC, self._on_img, qos, callback_group=cg)
        node.create_subscription(String, JASON_QR_TOPIC, self._on_qr_decoded, 10, callback_group=cg)
        threading.Thread(target=self._ensure_qr_off_at_boot, daemon=True).start()

    def _ensure_qr_off_at_boot(self) -> None:
        time.sleep(2.0)
        self.stop_qr(silent=True)

    def _on_img(self, msg: Image) -> None:
        now = time.time()
        jpg = self._encode_jpeg(msg)
        with self._lock:
            self._last_frame = now
            self._width = int(msg.width)
            self._height = int(msg.height)
            self._encoding = str(msg.encoding or "")
            self._frame_times.append(now)
            if len(self._frame_times) >= 2:
                span = self._frame_times[-1] - self._frame_times[0]
                if span > 0:
                    self._fps = (len(self._frame_times) - 1) / span
            if jpg:
                self._jpeg = jpg

    def _on_qr_decoded(self, msg: String) -> None:
        if not self._qr_running:
            return
        text = (msg.data or "").strip()
        with self._lock:
            self._qr_result = text or self._qr_result
            self._qr_updated = time.time()

    def latest_jpeg(self) -> Optional[bytes]:
        now = time.time()
        with self._lock:
            jpg = self._jpeg
            last = self._last_frame
        if not jpg or last <= 0 or (now - last) > 8.0:
            return None
        return jpg

    def status(self) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            last = self._last_frame
            width = self._width
            height = self._height
            encoding = self._encoding
            fps = self._fps
            qr_running = self._qr_running
            qr_result = self._qr_result
            qr_updated = self._qr_updated
            has_jpeg = self._jpeg is not None

        age_ms = int((now - last) * 1000) if last > 0 else None
        if last <= 0 or (now - last) > JASON_OFFLINE_SEC:
            cam_status = "OFFLINE"
        elif age_ms is not None and age_ms > 1500:
            cam_status = "CONNECTING"
        else:
            cam_status = "ONLINE"

        return {
            "camera_id": "my_camera",
            "source": "REAL",
            "model": "acA2500-14gc",
            "device_user_id": "106611-18",
            "topic": JASON_IMAGE_TOPIC,
            "status": cam_status,
            "has_frame": has_jpeg,
            "fps": round(float(fps), 2),
            "width": width,
            "height": height,
            "encoding": encoding,
            "last_frame_ms_ago": age_ms,
            "qr_recognition": "RUNNING" if qr_running else "OFF",
            "last_qr_result": qr_result if qr_running else "--",
            "qr_result_updated_at": qr_updated if qr_running else 0.0,
        }

    def _ros_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        env.setdefault("ROS_DOMAIN_ID", "30")
        return env

    def start_qr(self) -> Dict[str, Any]:
        with self._qr_lock:
            if self._qr_running:
                return {"success": True, "message": "already running", "qr_recognition": "RUNNING"}
            cmd = [
                "bash",
                "-lc",
                "source /opt/ros/humble/setup.bash && "
                "source /opt/delivery_ws/install/setup.bash && "
                f"ros2 launch qrcode_detector qrcode_detector.launch.py image_topic:={JASON_IMAGE_TOPIC}",
            ]
            try:
                self._qr_proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=self._ros_env(),
                )
            except Exception as exc:  # noqa: BLE001
                self._log(f"Jason QR start failed: {exc}")
                return {"success": False, "message": str(exc)}
            with self._lock:
                self._qr_running = True
                self._qr_result = ""
                self._qr_updated = 0.0
            self._log("Jason QR recognition started")
            return {"success": True, "message": "started", "qr_recognition": "RUNNING"}

    def stop_qr(self, silent: bool = False) -> Dict[str, Any]:
        with self._qr_lock:
            subprocess.run(["pkill", "-f", "qrcode_node"], check=False)
            subprocess.run(["pkill", "-f", "qrcode_detector.launch"], check=False)
            if self._qr_proc is not None:
                try:
                    self._qr_proc.terminate()
                except Exception:  # noqa: BLE001
                    pass
                self._qr_proc = None
            with self._lock:
                self._qr_running = False
                self._qr_result = ""
                self._qr_updated = 0.0
            if not silent:
                self._log("Jason QR recognition stopped")
            return {"success": True, "message": "stopped", "qr_recognition": "OFF"}
