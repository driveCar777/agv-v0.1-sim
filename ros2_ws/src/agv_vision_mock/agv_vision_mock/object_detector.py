"""Mock 物体检测 — 完全离线。

周期性发布 /vision/objects (PoseArray of detected object class)。
"""

from __future__ import annotations

import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from agv_vision_mock.presets import PRESET_OBJECTS


class MockObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_mock_object")
        self.declare_parameter("classes", "")
        self.declare_parameter("topic", "/vision/objects")
        self.declare_parameter("hz", 2.0)
        self.declare_parameter("frame_id", "camera_front")
        self.declare_parameter("mock_seed", 100)

        cls_csv = self.get_parameter("classes").get_parameter_value().string_value
        self._classes = [c.strip() for c in cls_csv.split(",") if c.strip()] or PRESET_OBJECTS
        random.seed(int(self.get_parameter("mock_seed").get_parameter_value().integer_value))

        self._pub = self.create_publisher(
            PoseArray,
            self.get_parameter("topic").get_parameter_value().string_value, 10,
        )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 0.5), self._on_tick)
        self.get_logger().info(f"mock_object ready, classes={self._classes}")

    def _on_tick(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        n = random.randint(0, 3)
        for _ in range(n):
            p = Pose()
            p.position.x = random.uniform(-1.0, 1.0)
            p.position.y = random.uniform(-1.0, 1.0)
            p.position.z = random.uniform(0.5, 1.5)
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()