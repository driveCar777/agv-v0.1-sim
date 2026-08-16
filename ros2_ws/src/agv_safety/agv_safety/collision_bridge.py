"""Collision Bridge — 根据 SafetyLevel 缩放 /cmd_vel_safe。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from delivery_interfaces.msg import SafetyState as SafetyStateMsg


class CollisionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_collision_bridge")
        self.declare_parameter("slow_factor", 0.3)
        self.declare_parameter("cmd_vel_in", "/cmd_vel_raw")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_safe")
        self.declare_parameter("state_topic", "/agv/safety_state")
        self.declare_parameter("hz", 50.0)

        self._level = 0
        self._last_cmd = Twist()

        self._state_sub = self.create_subscription(
            SafetyStateMsg,
            self.get_parameter("state_topic").get_parameter_value().string_value,
            self._on_state, 10,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_in").get_parameter_value().string_value,
            self._on_cmd, 10,
        )
        self._pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_out").get_parameter_value().string_value, 10,
        )
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info("collision bridge ready")

    def _on_state(self, msg: SafetyStateMsg) -> None:
        self._level = int(msg.level)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg

    def _on_tick(self) -> None:
        out = Twist()
        if self._level >= 3:
            out.linear.x = 0.0
            out.angular.z = 0.0
        elif self._level == 2:
            f = float(self.get_parameter("slow_factor").get_parameter_value().double_value)
            out.linear.x = self._last_cmd.linear.x * f
            out.angular.z = self._last_cmd.angular.z * f
        else:
            out = self._last_cmd
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()