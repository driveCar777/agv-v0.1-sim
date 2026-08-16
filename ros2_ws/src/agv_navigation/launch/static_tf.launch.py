"""AGV 静态 TF 发布 — base_link → 全部传感器 / 机械臂。

完全离线。ros2 launch agv_navigation static_tf.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    base_link_to_lidar_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_lidar_front",
        arguments=["0.30", "0.0", "0.20", "0.0", "0.0", "0.0", "base_link", "lidar_front"],
    )
    base_link_to_lidar_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_lidar_rear",
        arguments=["-0.30", "0.0", "0.20", "0.0", "0.0", "3.1416", "base_link", "lidar_rear"],
    )
    base_link_to_camera_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera_front",
        arguments=["0.35", "0.0", "0.30", "0.0", "0.0", "0.0", "base_link", "camera_front"],
    )
    base_link_to_camera_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera_rear",
        arguments=["-0.35", "0.0", "0.30", "0.0", "0.0", "3.1416", "base_link", "camera_rear"],
    )
    base_link_to_camera_down = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera_down",
        arguments=["0.0", "0.0", "0.05", "0.0", "1.5708", "0.0", "base_link", "camera_down"],
    )
    base_link_to_arm_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_arm_left",
        arguments=["0.10", "0.20", "0.40", "0.0", "0.0", "0.0", "base_link", "left_arm_base"],
    )
    base_link_to_arm_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_arm_right",
        arguments=["0.10", "-0.20", "0.40", "0.0", "0.0", "0.0", "base_link", "right_arm_base"],
    )

    # 12 个超声波（围车一周）
    ultrasonic_tfs = []
    ultrasonic_positions = [
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
    for name, x, y in ultrasonic_positions:
        yaw = 0.0 if x > 0 else 3.1416
        ultrasonic_tfs.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"tf_base_to_{name}",
                arguments=[str(x), str(y), "0.10", "0.0", "0.0", str(yaw), "base_link", name],
            )
        )

    return LaunchDescription([
        base_link_to_lidar_front,
        base_link_to_lidar_rear,
        base_link_to_camera_front,
        base_link_to_camera_rear,
        base_link_to_camera_down,
        base_link_to_arm_left,
        base_link_to_arm_right,
        *ultrasonic_tfs,
    ])