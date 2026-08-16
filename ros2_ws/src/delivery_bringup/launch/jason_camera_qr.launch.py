"""Launch Jason Basler camera + qrcode_detector (verify single wechat_qr_node)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_pylon = get_package_share_directory('pylon_ros2_camera_wrapper')
    pkg_qr = get_package_share_directory('qrcode_detector')
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_qr, 'launch', 'qrcode_detector.launch.py')
            ),
            launch_arguments={
                'image_topic': '/my_camera/pylon_ros2_camera_node/image_raw',
                'prefer_wechat_qr': 'true',
            }.items(),
        ),
    ])
