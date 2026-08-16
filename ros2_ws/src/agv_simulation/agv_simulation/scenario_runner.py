"""Scenario Runner — 加载 YAML 场景并启动仿真。

完全离线。把场景障碍物通过参数推给 lidar / ultrasonic mock。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from agv_simulation.scenario_loader import (
    load_scenario,
    scenario_to_obstacle_list,
)


class ScenarioRunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_scenario_runner")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("scenario_name", "empty_room")
        self.declare_parameter("publish_topic", "/agv/scenario")

        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        self._scenario = None
        if scenario_file and Path(scenario_file).is_file():
            try:
                self._scenario = load_scenario(scenario_file)
                self.get_logger().info(
                    f"loaded scenario '{self._scenario.name}': "
                    f"{len(self._scenario.obstacles)} obstacles"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"scenario load fail: {exc}")
        self._pub = self.create_publisher(
            String,
            self.get_parameter("publish_topic").get_parameter_value().string_value, 10,
        )
        self.create_timer(2.0, self._publish)
        self.get_logger().info("scenario runner ready")

    def _publish(self) -> None:
        if self._scenario is None:
            return
        msg = String()
        msg.data = self._scenario.name
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()