"""Launch agv_control_bridge → Robokit TCP 3055（Mock 或真车）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="tcp"),
            DeclareLaunchArgument("agv_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("cmd_vel_in", default_value="/cmd_vel"),
            Node(
                package="agv_control",
                executable="agv_control_bridge",
                name="agv_control_bridge",
                output="screen",
                parameters=[
                    {
                        "backend": LaunchConfiguration("backend"),
                        "agv_host": LaunchConfiguration("agv_host"),
                        "cmd_vel_in": LaunchConfiguration("cmd_vel_in"),
                        "auto_request_control": True,
                        "publish_hz": 50.0,
                        "cmd_vel_timeout_ms": 500,
                    }
                ],
            ),
        ]
    )
