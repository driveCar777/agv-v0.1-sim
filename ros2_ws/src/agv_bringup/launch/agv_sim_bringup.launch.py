"""AGV 仿真 bringup (P1) — 底盘 + TF + Web + 相机。完全离线。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="agv_control",
            executable="agv_control_bridge",
            name="agv_control_bridge",
            output="screen",
            parameters=[{
                "backend": "mock",
                "publish_hz": 50.0,
                "auto_request_control": True,
                "cmd_vel_in": "/cmd_vel",
                "odom_out": "/odom",
            }],
        ),
        Node(
            package="agv_tf",
            executable="agv_static_tf",
            name="agv_static_tf",
            output="screen",
        ),
        Node(
            package="agv_bridge",
            executable="robokit_mock_server",
            name="robokit_mock_server",
            output="screen",
            arguments=["--host", "127.0.0.1"],
        ),
        Node(
            package="camera_bridge",
            executable="camera_sim_node",
            name="camera_sim_node",
            output="screen",
        ),
        Node(
            package="delivery_web",
            executable="dashboard_node",
            name="dashboard_node",
            output="screen",
            parameters=[{
                "http_host": "0.0.0.0",
                "demo_port": 19999,
                "debug_port": 1999,
            }],
        ),
    ])
