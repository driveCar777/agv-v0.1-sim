"""Launch Jason Basler pylon camera with AGV-agreed namespace and config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_pylon = get_package_share_directory('pylon_ros2_camera_wrapper')
    tuned_config = os.path.join(
        pkg_pylon, 'config', 'aca2500_106611_18.tuned_v3.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_pylon, 'launch', 'pylon_ros2_camera.launch.py')
            ),
            launch_arguments={
                'camera_id': 'my_camera',
                'config_file': tuned_config,
            }.items(),
        ),
    ])
