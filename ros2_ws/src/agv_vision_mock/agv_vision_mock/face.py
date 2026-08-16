"""Mock 人脸识别 — 完全离线。

输出 /face/detection + 提供识别服务。
默认每 1s 报一次随机识别结果（来自预置用户表）。
"""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from delivery_interfaces.msg import FaceDetection, FaceDetectionArray
from agv_vision_mock.presets import PRESET_USERS


class MockFaceNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_mock_face")
        self.declare_parameter("users", "")
        self.declare_parameter("topic_detections", "/face/detections")
        self.declare_parameter("topic_result", "/face/recognition_result")
        self.declare_parameter("hz", 1.0)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("mock_seed", 42)

        users_csv = self.get_parameter("users").get_parameter_value().string_value
        self._users = [u.strip() for u in users_csv.split(",") if u.strip()] or PRESET_USERS
        random.seed(int(self.get_parameter("mock_seed").get_parameter_value().integer_value))

        self._pub_det = self.create_publisher(
            FaceDetectionArray,
            self.get_parameter("topic_detections").get_parameter_value().string_value, 10,
        )
        self._pub_res = self.create_publisher(
            String,
            self.get_parameter("topic_result").get_parameter_value().string_value, 10,
        )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 0.2), self._on_tick)
        self.get_logger().info(f"mock_face ready, users={self._users}")

    def _on_tick(self) -> None:
        min_c = float(self.get_parameter("min_confidence").get_parameter_value().double_value)
        user = random.choice(self._users)
        conf = random.uniform(min_c, 0.99)

        det_arr = FaceDetectionArray()
        det = FaceDetection()
        det.name = user
        det.confidence = float(conf)
        det_arr.detections.append(det)
        stamp = self.get_clock().now().to_msg()
        det_arr.header.stamp = stamp
        self._pub_det.publish(det_arr)

        result = String()
        result.data = f"{user}:{conf:.3f}"
        self._pub_res.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = MockFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()