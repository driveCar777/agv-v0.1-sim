"""AGV Nav2 bringup — mock odom + mock lidar。完全离线。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nav_share = get_package_share_directory("agv_navigation")
    default_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    default_map = os.environ.get(
        "AGV_MAP_YAML",
        os.path.join(nav_share, "maps", "empty_room.yaml"),
    )

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml, "use_sim_time": False}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": ["map_server", "amcl"]},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": [
                    "controller_server", "planner_server",
                    "behavior_server", "bt_navigator",
                    "waypoint_follower", "velocity_smoother",
                    "collision_monitor",
                ]},
            ],
        ),
    ])
