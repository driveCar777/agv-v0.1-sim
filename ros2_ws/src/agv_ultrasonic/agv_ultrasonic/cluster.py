"""12 超声 → PointCloud 融合。

输入：/ultrasonic/01..12 (sensor_msgs/Range)
输出：/ultrasonic_cloud (sensor_msgs/PointCloud)

完全离线。
"""

from __future__ import annotations

import math
from typing import Dict

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, Range

from agv_ultrasonic.driver import SENSOR_LAYOUT


class UltrasonicClusterNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_ultrasonic_cluster")
        self.declare_parameter("topic_prefix", "/ultrasonic/")
        self.declare_parameter("output", "/ultrasonic_cloud")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("hz", 20.0)
        self.declare_parameter("max_obstacle_distance", 2.0)

        self._ranges: Dict[int, float] = {}
        for idx in range(1, 13):
            topic = f"{self.get_parameter('topic_prefix').get_parameter_value().string_value}{idx:02d}"
            self.create_subscription(
                Range,
                topic,
                lambda msg, i=idx: self._on_range(i, msg),
                10,
            )

        self._pub = self.create_publisher(
            PointCloud,
            self.get_parameter("output").get_parameter_value().string_value, 10,
        )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info("ultrasonic cluster ready")

    def _on_range(self, idx: int, msg: Range) -> None:
        if msg.min_range <= msg.range <= msg.max_range:
            self._ranges[idx] = float(msg.range)

    def _on_tick(self) -> None:
        max_d = float(self.get_parameter("max_obstacle_distance").get_parameter_value().double_value)
        frame = self.get_parameter("frame_id").get_parameter_value().string_value
        stamp = self.get_clock().now().to_msg()

        msg = PointCloud()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.channels = []
        msg.points = []
        for idx, (ox, oy, base_yaw) in enumerate(SENSOR_LAYOUT, start=1):
            d = self._ranges.get(idx)
            if d is None or d > max_d:
                continue
            x = ox + d * math.cos(base_yaw)
            y = oy + d * math.sin(base_yaw)
            msg.points.append(Point32(x=float(x), y=float(y), z=0.05))
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()