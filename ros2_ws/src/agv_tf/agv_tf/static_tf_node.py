"""agv_static_tf — 一次性发布静态 TF：lidar_optical, camera_optical 等。

完全离线。默认从 URDF 加载，可独立覆盖。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

# 光学坐标系修正：camera 通常需要额外 -90° 旋转
# 简化：lidar 用 front_link 即可，camera 用 front_link + optical
EXTRA_TFS = [
    # (parent, child, x, y, z, roll, pitch, yaw)
    ("lidar_front",       "lidar_front_optical",       0, 0, 0, 0, 0, 0),
    ("lidar_rear",        "lidar_rear_optical",        0, 0, 0, 0, 0, 0),
    ("camera_front",      "camera_front_optical",      0, 0, 0, -1.5708, 0, -1.5708),
    ("camera_rear",       "camera_rear_optical",       0, 0, 0, -1.5708, 0, -1.5708),
    ("camera_down",       "camera_down_optical",       0, 0, 0, -1.5708, 0, -1.5708),
]


class StaticTfNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_static_tf")
        self._pub = StaticTransformBroadcaster(self)

        tfs = []
        for parent, child, x, y, z, roll, pitch, yaw in EXTRA_TFS:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            cy = __import__("math").cos(yaw * 0.5)
            sy = __import__("math").sin(yaw * 0.5)
            cp = __import__("math").cos(pitch * 0.5)
            sp = __import__("math").sin(pitch * 0.5)
            cr = __import__("math").cos(roll * 0.5)
            sr = __import__("math").sin(roll * 0.5)
            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy
            tfs.append(t)
        self._pub.sendTransform(tfs)
        self.get_logger().info(f"published {len(tfs)} static TFs")


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
