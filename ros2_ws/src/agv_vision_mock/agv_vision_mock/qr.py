"""Mock 二维码识别 — 完全离线。

周期性发布 /qr/detections (QrDetectionArray)。
"""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node

from delivery_interfaces.msg import QrDetection, QrDetectionArray
from agv_vision_mock.presets import PRESET_QRS


class MockQrNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_mock_qr")
        self.declare_parameter("codes", "")
        self.declare_parameter("topic", "/qr/detections")
        self.declare_parameter("hz", 1.0)
        self.declare_parameter("mock_seed", 7)

        codes_csv = self.get_parameter("codes").get_parameter_value().string_value
        self._codes = [c.strip() for c in codes_csv.split(",") if c.strip()] or PRESET_QRS
        random.seed(int(self.get_parameter("mock_seed").get_parameter_value().integer_value))

        self._pub = self.create_publisher(
            QrDetectionArray,
            self.get_parameter("topic").get_parameter_value().string_value, 10,
        )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 0.2), self._on_tick)
        self.get_logger().info(f"mock_qr ready, codes={self._codes}")

    def _on_tick(self) -> None:
        msg = QrDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        if random.random() > 0.3:
            d = QrDetection()
            d.code = random.choice(self._codes)
            d.confidence = random.uniform(0.7, 0.99)
            msg.detections.append(d)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockQrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()