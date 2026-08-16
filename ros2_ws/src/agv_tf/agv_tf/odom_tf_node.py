"""AGV 动态 odom→base_link TF 节点。

完全离线。从 AGV Backend 读取 odom，发布 TF。
"""

from __future__ import annotations

import math
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from agv_control.backend.factory import create_backend


class OdomTfNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_odom_tf")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self._backend = create_backend(
            backend_name=self.get_parameter("backend").get_parameter_value().string_value,
        )
        self._backend.connect()

        self._tf = TransformBroadcaster(self)
        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.create_timer(1.0 / hz, self._on_tick)
        self.get_logger().info(
            f"odom_tf ready, backend={type(self._backend).__name__}, hz={hz}"
        )

    def _on_tick(self) -> None:
        odom = self._backend.get_odom()
        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        t.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        t.transform.translation.x = odom.x
        t.transform.translation.y = odom.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(odom.yaw / 2.0)
        t.transform.rotation.w = math.cos(odom.yaw / 2.0)
        self._tf.sendTransform(t)

        if hasattr(self._backend, "step"):
            self._backend.step()


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
