"""AGV 双激光仿真节点 — 纯几何仿真。

front 在车头、rear 在车尾。两台 LiDAR 对角安装。
输出 /scan_front (lidar_front) + /scan_rear (lidar_rear)。
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from agv_lidar.scan_sim import DEFAULT_SCENE, make_scan as make_scan_result


def make_scan(*args, **kwargs) -> LaserScan:
    """ROS LaserScan 包装；单测请直接用 scan_sim.make_scan。"""
    result = make_scan_result(*args, **kwargs)
    msg = LaserScan()
    msg.header.stamp = args[0] if args else kwargs.get('stamp')
    msg.header.frame_id = result.frame_id
    msg.angle_min = result.angle_min
    msg.angle_max = result.angle_max
    msg.angle_increment = result.angle_increment
    msg.time_increment = 0.0
    msg.scan_time = 0.05
    msg.range_min = result.range_min
    msg.range_max = result.range_max
    msg.ranges = list(result.ranges)
    msg.intensities = []
    return msg


class MockLidarNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_mock_lidar")
        self.declare_parameter("frame_front", "lidar_front")
        self.declare_parameter("frame_rear", "lidar_rear")
        self.declare_parameter("topic_front", "/scan_front")
        self.declare_parameter("topic_rear", "/scan_rear")
        self.declare_parameter("hz", 10.0)
        self.declare_parameter("front_offset_x", 0.30)
        self.declare_parameter("front_offset_y", 0.0)
        self.declare_parameter("rear_offset_x", -0.30)
        self.declare_parameter("rear_offset_y", 0.0)
        self.declare_parameter("scene_json", "")
        self.declare_parameter("odom_topic", "/odom")

        from std_msgs.msg import String  # noqa: F401
        self._sub_odom = self.create_subscription(
            # 这里用 String 占位，因为 Odometry 需要 nav_msgs 依赖，简化用外发 topic
            # 实际部署应订阅 /odom 并用当前位姿生成 scan
            String,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            lambda msg: None,
            10,
        )

        # 当前位置（订阅 /odom 后的状态，第一版用参数注入）
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_yaw", 0.0)
        self._x = float(self.get_parameter("init_x").get_parameter_value().double_value)
        self._y = float(self.get_parameter("init_y").get_parameter_value().double_value)
        self._yaw = float(self.get_parameter("init_yaw").get_parameter_value().double_value)

        self._pub_front = self.create_publisher(
            LaserScan,
            self.get_parameter("topic_front").get_parameter_value().string_value, 10,
        )
        self._pub_rear = self.create_publisher(
            LaserScan,
            self.get_parameter("topic_rear").get_parameter_value().string_value, 10,
        )
        self._scene = self._load_scene()
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.get_logger().info(
            f"mock_lidar ready, scene obstacles={len(self._scene)}"
        )

    def _load_scene(self):
        path = self.get_parameter("scene_json").get_parameter_value().string_value
        if path:
            import json
            from pathlib import Path
            try:
                data = json.loads(Path(path).read_text(encoding="utf-8"))
                if "obstacles" in data:
                    obs = []
                    for o in data["obstacles"]:
                        obs.append((o["x"], o["y"], o.get("r", 0.05)))
                    return obs
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"scene load fail: {exc}")
        return DEFAULT_SCENE

    def _on_tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        fx = float(self.get_parameter("front_offset_x").get_parameter_value().double_value)
        fy = float(self.get_parameter("front_offset_y").get_parameter_value().double_value)
        rx = float(self.get_parameter("rear_offset_x").get_parameter_value().double_value)
        ry = float(self.get_parameter("rear_offset_y").get_parameter_value().double_value)

        front_msg = make_scan(
            stamp,
            self.get_parameter("frame_front").get_parameter_value().string_value,
            self._scene,
            (self._x + fx * math.cos(self._yaw) - fy * math.sin(self._yaw),
             self._y + fx * math.sin(self._yaw) + fy * math.cos(self._yaw)),
            self._yaw,
        )
        rear_msg = make_scan(
            stamp,
            self.get_parameter("frame_rear").get_parameter_value().string_value,
            self._scene,
            (self._x + rx * math.cos(self._yaw) - ry * math.sin(self._yaw),
             self._y + rx * math.sin(self._yaw) + ry * math.cos(self._yaw)),
            self._yaw + math.pi,
        )
        self._pub_front.publish(front_msg)
        self._pub_rear.publish(rear_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()