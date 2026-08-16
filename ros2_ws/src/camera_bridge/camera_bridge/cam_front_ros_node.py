#!/usr/bin/env python3
"""Xavier-side industrial cam_front ROS2 publisher skeleton (docs/25).

Run inside ROS2 Humble (Docker recommended on Jetson Ubuntu 20.04):

  export ROS_DOMAIN_ID=30
  export ROS_LOCALHOST_ONLY=0
  ros2 run camera_bridge cam_front_ros_node --ros-args \\
    -p camera_id:=192.168.1.100 \\
    -p jpeg_quality:=70 \\
    -p target_fps:=10.0

Publishes:
  /camera/cam_front/image_raw/compressed
  /camera/cam_front/driver_info   (optional driver self-check; NOT authoritative)
  /camera/cam_front/heartbeat
  /camera/cam_front/status
Service:
  /camera/capture

Authoritative /camera/cam_front/info is published only by NUC camera_ingress_node.
Without Basler/Aravis the node stays alive and reports link_ok=false (industrial-safe).
"""

from __future__ import annotations

import json
import time
from threading import Lock
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from delivery_interfaces.msg import CameraHeartbeat, CameraInfoLite
from delivery_interfaces.srv import CameraCapture


def _qos_reliable_keep1() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class CamFrontRosNode(Node):
    def __init__(self) -> None:
        super().__init__("cam_front_ros_node")
        self.declare_parameter("camera_name", "cam_front")
        self.declare_parameter("camera_id", "192.168.1.100")
        self.declare_parameter("frame_id", "cam_front_optical")
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("target_fps", 10.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("status_hz", 1.0)

        self._name = str(self.get_parameter("camera_name").value)
        self._device_id = str(self.get_parameter("camera_id").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._quality = int(self.get_parameter("jpeg_quality").value)
        self._fps = float(self.get_parameter("target_fps").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)

        self._cg = ReentrantCallbackGroup()
        self._lock = Lock()
        self._seq = 0
        self._frames_total = 0
        self._frames_dropped = 0
        self._link_ok = False
        self._error_code = 1
        self._error_msg = "camera_not_opened"
        self._last_jpeg: Optional[bytes] = None
        self._t0 = time.time()
        self._cap = None

        reliable = _qos_reliable_keep1()
        self._pub_jpg = self.create_publisher(
            CompressedImage, f"/camera/{self._name}/image_raw/compressed", qos_profile_sensor_data
        )
        # 权威 /camera/<name>/info 仅由 NUC camera_ingress 发布（看门狗）。
        # Xavier 只发 driver_info（可选自检），避免双发布者冲突。
        self._pub_info = self.create_publisher(
            CameraInfoLite, f"/camera/{self._name}/driver_info", reliable
        )
        self._pub_hb = self.create_publisher(
            CameraHeartbeat, f"/camera/{self._name}/heartbeat", reliable
        )
        self._pub_st = self.create_publisher(String, f"/camera/{self._name}/status", reliable)
        self.create_service(
            CameraCapture, "/camera/capture", self._on_capture, callback_group=self._cg
        )

        self._open_camera()
        period = 1.0 / max(self._fps, 1.0)
        self.create_timer(period, self._grab_tick)
        self.create_timer(1.0 / max(float(self.get_parameter("heartbeat_hz").value), 0.5), self._hb_tick)
        self.create_timer(1.0 / max(float(self.get_parameter("status_hz").value), 0.2), self._status_tick)
        self.get_logger().warn(
            f"cam_front_ros_node device_id={self._device_id} link_ok={self._link_ok} err={self._error_msg}"
        )

    def _open_camera(self) -> None:
        """Try Aravis/Basler; on failure keep node alive with link_ok=false."""
        try:
            from app.capture.gige_capture import GigeCapture  # type: ignore

            cap = GigeCapture(camera_id=self._device_id, buffer_count=20, timeout_ms=800)
            cap.start()
            self._cap = cap
            self._link_ok = True
            self._error_code = 0
            self._error_msg = ""
            self.get_logger().info(f"Basler opened {self._device_id}")
        except Exception as exc:  # noqa: BLE001
            self._cap = None
            self._link_ok = False
            self._error_code = 2
            self._error_msg = f"open_fail:{exc}"
            self.get_logger().error(self._error_msg)

    def _encode_jpeg(self, frame) -> Optional[bytes]:
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore

            if frame is None:
                return None
            if len(getattr(frame, "shape", ())) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            if frame.shape[1] != self._w or frame.shape[0] != self._h:
                frame = cv2.resize(frame, (self._w, self._h))
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
            if not ok:
                return None
            return bytes(buf)
        except Exception as exc:  # noqa: BLE001
            self._error_msg = f"encode_fail:{exc}"
            return None

    def _grab_tick(self) -> None:
        jpeg = None
        if self._cap is not None:
            try:
                frame = self._cap.read()
                jpeg = self._encode_jpeg(frame)
                if jpeg is None:
                    self._frames_dropped += 1
            except Exception as exc:  # noqa: BLE001
                self._link_ok = False
                self._error_code = 3
                self._error_msg = f"read_fail:{exc}"
                self._frames_dropped += 1
        if jpeg is None:
            return
        self._seq += 1
        self._frames_total += 1
        with self._lock:
            self._last_jpeg = jpeg
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        self._pub_jpg.publish(msg)

    def _hb_tick(self) -> None:
        self._seq += 0  # seq advances on frames; heartbeat carries last seq
        hb = CameraHeartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.header.frame_id = self._frame_id
        hb.camera_name = self._name
        hb.seq = int(self._seq)
        hb.uptime_sec = float(time.time() - self._t0)
        hb.fps_actual = float(self._fps if self._link_ok else 0.0)
        hb.frames_total = int(self._frames_total)
        hb.frames_dropped = int(self._frames_dropped)
        hb.link_ok = bool(self._link_ok)
        hb.device_id = self._device_id
        hb.error_code = int(self._error_code)
        hb.error_msg = self._error_msg
        self._pub_hb.publish(hb)

        info = CameraInfoLite()
        info.header = hb.header
        info.camera_name = self._name
        info.width = self._w
        info.height = self._h
        info.fps = hb.fps_actual
        info.connected = bool(self._link_ok and self._last_jpeg is not None)
        info.transport = "xavier_ros2"
        self._pub_info.publish(info)

    def _status_tick(self) -> None:
        payload = {
            "ok": bool(self._link_ok),
            "camera_name": self._name,
            "transport": "xavier_ros2" if self._link_ok else "xavier_ros2_waiting",
            "peer": "xavier",
            "domain_id": int(__import__("os").environ.get("ROS_DOMAIN_ID", "30")),
            "width": self._w,
            "height": self._h,
            "fps": self._fps if self._link_ok else 0.0,
            "link_ok": bool(self._link_ok),
            "last_error": self._error_msg,
            "seq": int(self._seq),
            "device_id": self._device_id,
            "frames_total": int(self._frames_total),
            "frames_dropped": int(self._frames_dropped),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_st.publish(msg)

    def _on_capture(self, req: CameraCapture.Request, res: CameraCapture.Response) -> CameraCapture.Response:
        name = (req.camera_name or self._name).strip() or self._name
        if name != self._name:
            res.success = False
            res.message = f"unknown camera {name}"
            return res
        with self._lock:
            jpeg = self._last_jpeg
        if not jpeg:
            res.success = False
            res.message = "no frame"
            return res
        res.success = True
        res.message = "ok"
        res.encoding = "jpeg"
        res.width = self._w
        res.height = self._h
        res.data = list(jpeg)
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamFrontRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._cap is not None:
            try:
                node._cap.stop()
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
