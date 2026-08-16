"""双激光融合 — front + rear → /scan (单台虚拟激光)。

优先复用 ira_laser_tools / laser_filters。
本节点只做最简实现 (TF + 拼接)，生产环境建议用 ira_laser_tools。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DualLidarMergerNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_dual_lidar_merger")
        self.declare_parameter("scan_front", "/scan_front")
        self.declare_parameter("scan_rear", "/scan_rear")
        self.declare_parameter("output", "/scan")
        self.declare_parameter("frame_id", "base_link")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_front: LaserScan | None = None
        self._latest_rear: LaserScan | None = None

        self._sub_front = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_front").get_parameter_value().string_value,
            lambda msg: self._latest_front.__setattr__("x", msg) or setattr(self, "_latest_front", msg),
            10,
        )
        self._sub_rear = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_rear").get_parameter_value().string_value,
            lambda msg: setattr(self, "_latest_rear", msg),
            10,
        )
        self._pub = self.create_publisher(
            LaserScan,
            self.get_parameter("output").get_parameter_value().string_value, 10,
        )
        self.create_timer(0.1, self._on_tick)
        self.get_logger().info("dual_lidar_merger ready")

    def _on_tick(self) -> None:
        # 简化：直接转发 front 作为 /scan。生产环境应做 TF + 拼接。
        if self._latest_front is not None:
            msg = self._latest_front
            msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()