"""AGV TF bringup — URDF → robot_state_publisher."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    urdf_path = os.path.join(
        get_package_share_directory("agv_tf"), "urdf", "agv.urdf.xacro"
    )
    robot_description = Command([FindExecutable("xacro"), " ", urdf_path])
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
        ),
        Node(
            package="agv_tf",
            executable="agv_static_tf",
            output="screen",
        ),
    ])
