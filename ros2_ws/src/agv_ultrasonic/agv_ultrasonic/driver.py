"""12 个超声波仿真驱动 — 每个输出 sensor_msgs/Range。

完全离线。无 ROS2 时也可单测（closest_distance / make_sensor_list）。
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from agv_lidar.scan_sim import DEFAULT_SCENE
from agv_ultrasonic.layout import SENSOR_LAYOUT, closest_distance



class UltrasonicDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_ultrasonic_driver")
        self.declare_parameter("hz", 20.0)
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_yaw", 0.0)
        self.declare_parameter("frame_prefix", "ultrasonic_")
        self.declare_parameter("topic_prefix", "/ultrasonic/")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 4.0)
        self.declare_parameter("field_of_view", 0.3)
        self.declare_parameter("radiation_type", Range.ULTRASOUND)
        self.declare_parameter("scene_json", "")

        self._x = float(self.get_parameter("init_x").get_parameter_value().double_value)
        self._y = float(self.get_parameter("init_y").get_parameter_value().double_value)
        self._yaw = float(self.get_parameter("init_yaw").get_parameter_value().double_value)
        self._scene = self._load_scene()
        self._pubs: Dict[str, object] = {}

        for idx, (ox, oy, base_yaw) in enumerate(SENSOR_LAYOUT, start=1):
            name = f"{self.get_parameter('frame_prefix').get_parameter_value().string_value}{idx:02d}"
            topic = f"{self.get_parameter('topic_prefix').get_parameter_value().string_value}{idx:02d}"
            self._pubs[topic] = self.create_publisher(Range, topic, 10)
            self._pubs[topic + "_frame"] = name

        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(f"ultrasonic driver ready, {len(self._pubs) // 2} sensors")

    def _load_scene(self):
        path = self.get_parameter("scene_json").get_parameter_value().string_value
        if path:
            import json
            from pathlib import Path
            try:
                data = json.loads(Path(path).read_text(encoding="utf-8"))
                if "obstacles" in data:
                    return [(o["x"], o["y"], o.get("r", 0.05)) for o in data["obstacles"]]
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"scene load fail: {exc}")
        return DEFAULT_SCENE

    def _on_tick(self) -> None:
        range_min = float(self.get_parameter("range_min").get_parameter_value().double_value)
        range_max = float(self.get_parameter("range_max").get_parameter_value().double_value)
        fov = float(self.get_parameter("field_of_view").get_parameter_value().double_value)
        rad_type = int(self.get_parameter("radiation_type").get_parameter_value().integer_value)
        stamp = self.get_clock().now().to_msg()

        for idx, (ox, oy, base_yaw) in enumerate(SENSOR_LAYOUT, start=1):
            sx_world = self._x + ox * math.cos(self._yaw) - oy * math.sin(self._yaw)
            sy_world = self._y + ox * math.sin(self._yaw) + oy * math.cos(self._yaw)
            sensor_yaw_world = self._yaw + base_yaw
            d = closest_distance(sx_world, sy_world, sensor_yaw_world, self._scene, range_max)
            d = max(range_min, min(range_max, d))

            topic = f"{self.get_parameter('topic_prefix').get_parameter_value().string_value}{idx:02d}"
            frame = self._pubs.get(topic + "_frame", "ultrasonic_unknown")
            pub = self._pubs.get(topic)
            if pub is None:
                continue
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = frame if isinstance(frame, str) else "ultrasonic_unknown"
            msg.radiation_type = rad_type
            msg.field_of_view = fov
            msg.min_range = range_min
            msg.max_range = range_max
            msg.range = float(d)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()