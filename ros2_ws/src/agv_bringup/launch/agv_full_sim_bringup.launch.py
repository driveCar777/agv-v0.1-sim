"""AGV 全量仿真 — 底盘 + 传感器 + 视觉 Mock + Safety + Mission + Web。

完全离线。不连真车 / Leo / Zack / Jetson。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("agv_bringup")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "agv_nav_bringup.launch.py")
            )
        ),
        Node(
            package="agv_vision_mock",
            executable="agv_mock_face",
            name="agv_mock_face",
            output="screen",
        ),
        Node(
            package="agv_vision_mock",
            executable="agv_mock_qr",
            name="agv_mock_qr",
            output="screen",
        ),
        Node(
            package="agv_vision_mock",
            executable="agv_mock_object",
            name="agv_mock_object",
            output="screen",
        ),
        Node(
            package="agv_vision_mock",
            executable="agv_mock_image",
            name="agv_mock_image",
            output="screen",
        ),
        Node(
            package="agv_mission",
            executable="agv_mission_manager",
            name="agv_mission_manager",
            output="screen",
            parameters=[{
                "auto_offline_skip": True,
                "auto_start": True,
                "pickup_x": 2.0,
                "pickup_y": 1.5,
                "user_x": -2.0,
                "user_y": 3.0,
            }],
        ),
    ])
