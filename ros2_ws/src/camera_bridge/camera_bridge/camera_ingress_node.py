"""NUC-side ROS2 ingress for industrial cam_front (docs/25).

Subscribes Xavier:
  /camera/<name>/image_raw/compressed   sensor_msgs/CompressedImage  BEST_EFFORT
  /camera/<name>/info                   CameraInfoLite               RELIABLE
  /camera/<name>/heartbeat              CameraHeartbeat              RELIABLE
  /camera/<name>/status                 std_msgs/String JSON         RELIABLE

Publishes for local consumers (dashboard / face_bridge):
  /camera/<name>/image_raw              sensor_msgs/Image rgb8
  /camera/<name>/info                   CameraInfoLite (watchdog may force connected=false)
"""

from __future__ import annotations

import json
import os
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from delivery_interfaces.msg import CameraHeartbeat, CameraInfoLite


def _qos_reliable_keep1() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


def _decode_jpeg_to_rgb(blob: bytes) -> Optional[tuple[int, int, bytes]]:
    try:
        from io import BytesIO

        from PIL import Image as PILImage  # type: ignore

        im = PILImage.open(BytesIO(blob)).convert("RGB")
        w, h = im.size
        return w, h, im.tobytes()
    except Exception:  # noqa: BLE001
        pass
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        arr = np.frombuffer(blob, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        h, w = bgr.shape[:2]
        return w, h, bgr[:, :, ::-1].tobytes()
    except Exception:  # noqa: BLE001
        return None


class CameraIngressNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_ingress_node")
        self.declare_parameter("camera_name", "cam_front")
        self.declare_parameter("heartbeat_timeout_sec", 1.5)
        self.declare_parameter("frame_id", "cam_front_optical")
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("log_dir", os.environ.get("SYSTEM_LOG_DIR", "/var/log/delivery/system"))

        self._name = str(self.get_parameter("camera_name").value).strip() or "cam_front"
        self._timeout = float(self.get_parameter("heartbeat_timeout_sec").value)
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or f"{self._name}_optical"
        self._publish_raw = bool(self.get_parameter("publish_raw").value)

        self._last_hb_wall = 0.0
        self._last_frame_wall = 0.0
        self._last_seq: Optional[int] = None
        self._seq_gaps = 0
        self._frames = 0
        self._last_wh = (0, 0)
        self._last_status: Dict = {}
        self._link_ok = False
        self._last_error = "waiting_heartbeat"
        self._warned_timeout = False

        reliable = _qos_reliable_keep1()
        self._raw_pub = self.create_publisher(Image, f"/camera/{self._name}/image_raw", 5)
        self._info_pub = self.create_publisher(CameraInfoLite, f"/camera/{self._name}/info", 10)

        self.create_subscription(
            CompressedImage,
            f"/camera/{self._name}/image_raw/compressed",
            self._on_compressed,
            qos_profile_sensor_data,
        )
        # Peer may also publish /camera/<name>/driver_info; authoritative info is ours only.
        self.create_subscription(
            CameraHeartbeat,
            f"/camera/{self._name}/heartbeat",
            self._on_heartbeat,
            reliable,
        )
        self.create_subscription(
            String,
            f"/camera/{self._name}/status",
            self._on_status,
            reliable,
        )

        self.create_timer(0.5, self._watchdog_tick)
        self.get_logger().warn(
            f"camera_ingress: waiting ROS2 cam={self._name} "
            f"compressed+heartbeat domain hints ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '?')}"
        )

    def _on_compressed(self, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        if not data:
            return
        parsed = _decode_jpeg_to_rgb(data)
        if not parsed:
            self._last_error = "jpeg_decode_fail"
            self.get_logger().warn("compressed jpeg decode failed", throttle_duration_sec=2.0)
            return
        w, h, rgb = parsed
        self._last_wh = (w, h)
        self._last_frame_wall = time.time()
        self._frames += 1
        if self._publish_raw:
            out = Image()
            out.header = msg.header
            if not out.header.frame_id:
                out.header.frame_id = self._frame_id
            out.height = h
            out.width = w
            out.encoding = "rgb8"
            out.is_bigendian = 0
            out.step = w * 3
            out.data = rgb
            self._raw_pub.publish(out)
        if self._frames == 1:
            self.get_logger().info(f"first compressed frame {w}x{h} bytes={len(data)}")

    def _on_heartbeat(self, msg: CameraHeartbeat) -> None:
        now = time.time()
        self._last_hb_wall = now
        self._link_ok = bool(msg.link_ok)
        self._last_error = str(msg.error_msg or "")
        seq = int(msg.seq)
        if self._last_seq is not None and seq > self._last_seq + 1:
            self._seq_gaps += int(seq - self._last_seq - 1)
        self._last_seq = seq
        if self._warned_timeout:
            self.get_logger().info("heartbeat restored")
            self._warned_timeout = False

    def _on_status(self, msg: String) -> None:
        try:
            self._last_status = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self._last_status = {"raw": msg.data}

    def _watchdog_tick(self) -> None:
        now = time.time()
        hb_age = (now - self._last_hb_wall) if self._last_hb_wall else 1e9
        alive = hb_age <= self._timeout
        connected = alive and self._link_ok and (now - self._last_frame_wall) < max(3.0, self._timeout * 3)
        if not alive and not self._warned_timeout:
            self.get_logger().error(
                f"cam_front heartbeat timeout age={hb_age:.1f}s err={self._last_error}"
            )
            self._warned_timeout = True
            self._last_error = self._last_error or "heartbeat_timeout"

        info = CameraInfoLite()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self._frame_id
        info.camera_name = self._name
        info.width = int(self._last_wh[0])
        info.height = int(self._last_wh[1])
        info.fps = float(self._last_status.get("fps") or (10.0 if connected else 0.0))
        info.connected = bool(connected)
        info.transport = "xavier_ros2" if connected else "xavier_ros2_waiting"
        self._info_pub.publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraIngressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
