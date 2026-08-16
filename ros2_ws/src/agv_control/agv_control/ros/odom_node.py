"""Backend → /odom + /tf (odom→base_link)。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from agv_control.backend.factory import create_backend
from agv_control.state.pose import yaw_to_quat


class OdomNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_odom_node")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self._backend = create_backend(
            backend_name=self.get_parameter("backend").get_parameter_value().string_value,
        )
        self._backend.connect()

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf = TransformBroadcaster(self)

        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)

    def _on_tick(self) -> None:
        odom = self._backend.get_odom()
        stamp = self.get_clock().now().to_msg()

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        o.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        o.pose.pose.position.x = odom.x
        o.pose.pose.position.y = odom.y
        o.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(odom.yaw)[2], yaw_to_quat(odom.yaw)[3]
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = odom.vx
        o.twist.twist.angular.z = odom.w
        self._odom_pub.publish(o)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = o.header.frame_id
        t.child_frame_id = o.child_frame_id
        t.transform.translation.x = odom.x
        t.transform.translation.y = odom.y
        t.transform.translation.z = 0.0
        t.transform.rotation = o.pose.pose.orientation
        self._tf.sendTransform(t)

        if hasattr(self._backend, "step"):
            self._backend.step()


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()