"""AGV Safety Monitor — 订阅 cmd_vel + lidar + odom，发布安全状态。

完全离线。默认订阅 mock 后端发出的 topic。
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from delivery_interfaces.msg import SafetyState as SafetyStateMsg
from agv_safety.levels import SafetyLevel


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_safety_monitor")
        self.declare_parameter("cmd_vel_timeout_ms", 500)
        self.declare_parameter("scan_timeout_ms", 1000)
        self.declare_parameter("min_obstacle_distance_m", 0.6)
        self.declare_parameter("slow_distance_m", 1.5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("scan_topic", "/scan_front")
        self.declare_parameter("state_topic", "/agv/safety_state")
        self.declare_parameter("hz", 20.0)

        self._last_cmd_stamp = self.get_clock().now()
        self._last_scan_stamp = self.get_clock().now()
        self._level = SafetyLevel.OK
        self._reason = "init"
        self._min_front = math.inf
        self._min_rear = math.inf

        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._state_pub = self.create_publisher(SafetyStateMsg, state_topic, 10)
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(
            f"safety monitor ready (cmd={cmd_topic} scan={scan_topic} state={state_topic})"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd_stamp = self.get_clock().now()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_stamp = self.get_clock().now()
        if len(msg.ranges) == 0:
            return
        n = len(msg.ranges)
        front = msg.ranges[max(0, n // 2 - n // 12): n // 2 + n // 12]
        rear = msg.ranges[max(0, n - n // 12):] + msg.ranges[: n // 12]
        valid_front = [r for r in front if msg.range_min < r < msg.range_max]
        valid_rear = [r for r in rear if msg.range_min < r < msg.range_max]
        self._min_front = min(valid_front) if valid_front else math.inf
        self._min_rear = min(valid_rear) if valid_rear else math.inf

    def _on_tick(self) -> None:
        now = self.get_clock().now()
        cmd_age = (now - self._last_cmd_stamp).nanoseconds / 1e6
        scan_age = (now - self._last_scan_stamp).nanoseconds / 1e6

        old_level, old_reason = self._level, self._reason

        if scan_age > self.get_parameter("scan_timeout_ms").get_parameter_value().integer_value:
            self._level = SafetyLevel.STOP
            self._reason = f"scan_timeout({scan_age:.0f}ms)"
        elif cmd_age > self.get_parameter("cmd_vel_timeout_ms").get_parameter_value().integer_value:
            self._level = SafetyLevel.SLOW
            self._reason = f"cmd_stale({cmd_age:.0f}ms)"
        else:
            slow_d = self.get_parameter("slow_distance_m").get_parameter_value().double_value
            stop_d = self.get_parameter("min_obstacle_distance_m").get_parameter_value().double_value
            closest = min(self._min_front, self._min_rear)
            if closest < stop_d:
                self._level = SafetyLevel.STOP
                self._reason = f"too_close({closest:.2f}m)"
            elif closest < slow_d:
                self._level = SafetyLevel.SLOW
                self._reason = f"approaching({closest:.2f}m)"
            else:
                self._level = SafetyLevel.OK
                self._reason = "clear"

        if self._level != old_level or self._reason != old_reason:
            self.get_logger().info(f"safety -> {self._level.name} ({self._reason})")

        msg = SafetyStateMsg()
        msg.level = int(self._level)
        msg.reason = self._reason
        msg.min_front_distance = float(self._min_front)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()