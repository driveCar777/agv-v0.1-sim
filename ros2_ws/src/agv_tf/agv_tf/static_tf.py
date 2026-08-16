"""AGV 静态 TF — base_link → 全部传感器 / 机械臂。

完全离线。RPY 正确转换为四元数。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from agv_tf.geometry import quat_from_rpy

# name -> (x, y, z, roll, pitch, yaw)
TF_CONFIG = {
    "lidar_front": (0.30, 0.00, 0.20, 0.0, 0.0, 0.0),
    "lidar_rear": (-0.30, 0.00, 0.20, 0.0, 0.0, 3.1416),
    "camera_front": (0.35, 0.00, 0.30, 0.0, 0.0, 0.0),
    "camera_rear": (-0.35, 0.00, 0.30, 0.0, 0.0, 3.1416),
    "camera_down": (0.00, 0.00, 0.05, 0.0, 1.5708, 0.0),
    "camera_wrist": (0.10, 0.20, 1.10, 0.0, 0.0, 0.0),
    "left_arm_base": (0.10, 0.20, 0.40, 0.0, 0.0, 0.0),
    "right_arm_base": (0.10, -0.20, 0.40, 0.0, 0.0, 0.0),
}

ULTRASONIC_OFFSETS = [
    ("ultrasonic_01", 0.35, 0.30),
    ("ultrasonic_02", 0.35, 0.20),
    ("ultrasonic_03", 0.35, 0.10),
    ("ultrasonic_04", 0.35, -0.10),
    ("ultrasonic_05", 0.35, -0.20),
    ("ultrasonic_06", 0.35, -0.30),
    ("ultrasonic_07", -0.35, 0.30),
    ("ultrasonic_08", -0.35, 0.20),
    ("ultrasonic_09", -0.35, 0.10),
    ("ultrasonic_10", -0.35, -0.10),
    ("ultrasonic_11", -0.35, -0.20),
    ("ultrasonic_12", -0.35, -0.30),
]


def _make_tf(name: str, xyzrpy, parent: str = "base_link") -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = name
    t.transform.translation.x = float(xyzrpy[0])
    t.transform.translation.y = float(xyzrpy[1])
    t.transform.translation.z = float(xyzrpy[2])
    qx, qy, qz, qw = quat_from_rpy(float(xyzrpy[3]), float(xyzrpy[4]), float(xyzrpy[5]))
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


class StaticTfNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_static_tf")
        self._pub = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for name, cfg in TF_CONFIG.items():
            t = _make_tf(name, cfg)
            t.header.stamp = stamp
            tfs.append(t)
        for name, x, y in ULTRASONIC_OFFSETS:
            yaw = 0.0 if x > 0 else 3.1416
            t = _make_tf(name, (x, y, 0.10, 0.0, 0.0, yaw))
            t.header.stamp = stamp
            tfs.append(t)
        self._pub.sendTransform(tfs)
        self.get_logger().info(f"static tf published: {len(tfs)} frames")


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
