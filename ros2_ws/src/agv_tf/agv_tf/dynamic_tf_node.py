"""动态 TF 节点 — 把 mock 后端的传感器位姿 / xArm joint_states 转成 TF。

完全离线。订阅 /joint_states 和后端位姿，发布 map→odom 校正、base_link→arm 等动态变换。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DynamicTfNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_dynamic_tf")
        self._tf = TransformBroadcaster(self)
        self._last_odom: Odometry | None = None
        self._last_joints: JointState | None = None
        self._map_to_odom = (0.0, 0.0, 0.0)
        self._map_yaw = 0.0

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Odometry, "/agv/pose", self._on_agv_pose, 10)
        self.create_timer(0.02, self._publish_tf)
        self.get_logger().info("dynamic tf node ready")

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_joints(self, msg: JointState) -> None:
        self._last_joints = msg

    def _on_agv_pose(self, msg: Odometry) -> None:
        # 用 AGV 在 map 中的全局位姿推算 map→odom
        if self._last_odom is None:
            return
        agv_x = msg.pose.pose.position.x
        agv_y = msg.pose.pose.position.y
        odom_x = self._last_odom.pose.pose.position.x
        odom_y = self._last_odom.pose.pose.position.y
        # map→odom = map→base - odom→base
        self._map_to_odom = (agv_x - odom_x, agv_y - odom_y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        agv_yaw = math.atan2(siny_cosp, cosy_cosp)
        oq = self._last_odom.pose.pose.orientation
        osiny = 2.0 * (oq.w * oq.z + oq.x * oq.y)
        ocosy = 1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z)
        odom_yaw = math.atan2(osiny, ocosy)
        self._map_yaw = agv_yaw - odom_yaw

    def _publish_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()

        # map → odom
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = self._map_to_odom[0]
        t.transform.translation.y = self._map_to_odom[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.w = math.cos(self._map_yaw / 2.0)
        t.transform.rotation.z = math.sin(self._map_yaw / 2.0)
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
