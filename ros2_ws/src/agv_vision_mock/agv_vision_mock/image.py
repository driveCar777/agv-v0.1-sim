"""Mock 相机图像 — 完全离线。

周期性发布 /camera/{front,rear,wrist}/image_raw (sensor_msgs/Image)。
"""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from agv_vision_mock.presets import CAMERA_LAYOUT


class MockImageNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_mock_image")
        self.declare_parameter("cameras", "front,rear,wrist")
        self.declare_parameter("hz", 10.0)
        self.declare_parameter("noise_seed", 99)

        cams_csv = self.get_parameter("cameras").get_parameter_value().string_value
        self._cams = [c.strip() for c in cams_csv.split(",") if c.strip()]
        self._pubs = {}
        random.seed(int(self.get_parameter("noise_seed").get_parameter_value().integer_value))

        for cam in self._cams:
            cfg = CAMERA_LAYOUT.get(cam)
            if cfg is None:
                continue
            topic, frame, w, h = cfg
            self._pubs[cam] = (
                self.create_publisher(Image, topic, 10),
                frame, w, h,
            )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(f"mock_image ready, cameras={list(self._pubs.keys())}")

    def _on_tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for cam, (pub, frame, w, h) in self._pubs.items():
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame
            msg.width = w
            msg.height = h
            msg.encoding = "rgb8"
            msg.step = w * 3
            msg.is_bigendian = False
            # 纯随机噪声（避免 import numpy，依赖更轻）
            n = w * h * 3
            msg.data = bytearray(random.randint(0, 255) for _ in range(n))
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()