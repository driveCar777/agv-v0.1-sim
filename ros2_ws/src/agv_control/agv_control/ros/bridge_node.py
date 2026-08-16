"""agv_control_bridge — 合并 cmd_vel_node + odom_node，单节点模式。

默认 Backend=mock，零硬件依赖。
"""

from __future__ import annotations

import math
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from agv_control.backend.base import IAGVControlBackend, Twist2D
from agv_control.backend.factory import create_backend
from agv_control.safety.watchdog import Watchdog, WatchdogConfig


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_control_bridge")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("agv_host", "")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("auto_request_control", True)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("cmd_vel_timeout_ms", 500)
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("odom_out", "/odom")
        self.declare_parameter("tf_odom_to_base", True)

        backend_name = self.get_parameter("backend").get_parameter_value().string_value
        agv_host = self.get_parameter("agv_host").get_parameter_value().string_value
        kw = {}
        if agv_host:
            kw["agv_host"] = agv_host
        self._backend: IAGVControlBackend = create_backend(
            backend_name=backend_name,
            **kw,
        )
        self._backend.connect()
        if self.get_parameter("auto_request_control").get_parameter_value().bool_value:
            self._backend.request_control("ros2_bridge")

        self._last_cmd = Twist2D()
        self._last_cmd_stamp = self.get_clock().now()
        self._sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_in").get_parameter_value().string_value,
            self._on_cmd,
            10,
        )

        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_out").get_parameter_value().string_value, 10
        )
        self._tf = TransformBroadcaster(self)

        self._watchdog = Watchdog(
            cfg=WatchdogConfig(
                cmd_stale_ms=self.get_parameter("cmd_vel_timeout_ms").get_parameter_value().integer_value,
            ),
            has_control=lambda: self._backend.has_control,
            on_safe_stop=lambda: self._backend.stop(),
            clock_s=lambda: self.get_clock().now().nanoseconds / 1e9,
        )

        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(
            f"bridge ready, backend={type(self._backend).__name__}, hz={hz}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = Twist2D(
            vx=float(msg.linear.x), vy=float(msg.linear.y), w=float(msg.angular.z)
        )
        self._watchdog.feed_cmd()

    def _on_tick(self) -> None:
        self._watchdog.tick()

        self._backend.set_velocity(self._last_cmd)

        odom = self._backend.get_odom()
        stamp = self.get_clock().now().to_msg()

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        o.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        o.pose.pose.position.x = odom.x
        o.pose.pose.position.y = odom.y
        o.pose.pose.position.z = 0.0
        half = odom.yaw / 2.0
        o.pose.pose.orientation.z = math.sin(half)
        o.pose.pose.orientation.w = math.cos(half)
        o.twist.twist.linear.x = odom.vx
        o.twist.twist.angular.z = odom.w
        self._odom_pub.publish(o)

        if self.get_parameter("tf_odom_to_base").get_parameter_value().bool_value:
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

    def destroy_node(self):
        try:
            self._backend.stop()
            self._backend.release_control()
            self._backend.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()