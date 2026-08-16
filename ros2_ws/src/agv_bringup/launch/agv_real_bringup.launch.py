"""AGV 真车 bringup — 仅 REAL_ROBOT_ENABLED=true 时启动。默认禁用。"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_and_launch(context, *args, **kwargs):
    if os.environ.get("REAL_ROBOT_ENABLED", "false").lower() != "true":
        raise RuntimeError(
            "agv_real_bringup blocked: set REAL_ROBOT_ENABLED=true "
            "and AGV_BACKEND=real explicitly. Default is offline mock."
        )
    if os.environ.get("AGV_BACKEND", "mock").lower() != "real":
        raise RuntimeError("agv_real_bringup requires AGV_BACKEND=real")

    return [
        Node(
            package="agv_control",
            executable="agv_control_bridge",
            name="agv_control_bridge",
            output="screen",
            parameters=[{
                "backend": "real",
                "publish_hz": 50.0,
                "auto_request_control": True,
                "agv_host": LaunchConfiguration("agv_host").perform(context),
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("agv_host", default_value="192.168.192.5"),
        DeclareLaunchArgument("agv_port", default_value="502"),
        OpaqueFunction(function=_check_and_launch),
    ])
