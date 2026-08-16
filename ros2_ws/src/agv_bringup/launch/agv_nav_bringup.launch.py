"""AGV 仿真 + Nav2 bringup (P2) — 底盘 + 传感器 + Nav2 + Safety。完全离线。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("agv_bringup")
    nav_share = get_package_share_directory("agv_navigation")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "agv_sim_bringup.launch.py")
            )
        ),
        Node(
            package="agv_lidar",
            executable="agv_mock_lidar",
            name="agv_mock_lidar",
            output="screen",
        ),
        Node(
            package="agv_lidar",
            executable="agv_dual_lidar_merger",
            name="agv_dual_lidar_merger",
            output="screen",
        ),
        Node(
            package="agv_ultrasonic",
            executable="agv_ultrasonic_driver",
            name="agv_ultrasonic_driver",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, "launch", "nav2_bringup.launch.py")
            )
        ),
        Node(
            package="agv_safety",
            executable="agv_collision_bridge",
            name="agv_collision_bridge",
            output="screen",
            parameters=[{"slow_factor": 0.3, "hz": 50.0}],
        ),
        Node(
            package="agv_safety",
            executable="agv_safety_monitor",
            name="agv_safety_monitor",
            output="screen",
        ),
    ])
