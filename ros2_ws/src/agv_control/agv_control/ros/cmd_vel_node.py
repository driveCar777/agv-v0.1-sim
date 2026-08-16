"""/cmd_vel → Backend（默认 Mock）。

完全离线。订阅 /cmd_vel，写入 Backend。
50 Hz 内部 loop 把 vx/vy/w 持续推给 backend（防止硬件 watchdog 触发）。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from agv_control.backend.base import IAGVControlBackend, Twist2D
from agv_control.backend.factory import create_backend


class CmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_cmd_vel_node")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("auto_request_control", True)

        self._backend: IAGVControlBackend = create_backend(
            backend_name=self.get_parameter("backend").get_parameter_value().string_value,
        )
        if not self._backend.connect():
            self.get_logger().error("backend connect failed")
        if self.get_parameter("auto_request_control").get_parameter_value().bool_value:
            ok = self._backend.request_control("ros2_cmd_vel")
            self.get_logger().info(f"request_control -> {ok}")

        self._last_cmd = Twist2D(0.0, 0.0, 0.0)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(
            f"cmd_vel_node ready, backend={type(self._backend).__name__}, hz={hz}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = Twist2D(
            vx=float(msg.linear.x), vy=float(msg.linear.y), w=float(msg.angular.z)
        )

    def _on_tick(self) -> None:
        self._backend.set_velocity(self._last_cmd)

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
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()