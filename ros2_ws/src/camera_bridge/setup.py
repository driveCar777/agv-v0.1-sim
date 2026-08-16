from setuptools import setup
import os
from glob import glob

package_name = 'camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delivery',
    maintainer_email='dev@example.com',
    description='Three industrial camera bridge + QR/AprilTag sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_sim_node = camera_bridge.camera_sim_node:main',
            'camera_bridge_node = camera_bridge.camera_bridge_node:main',
            'xavier_cam_front_bridge = camera_bridge.xavier_cam_front_bridge:main',
            'camera_ingress_node = camera_bridge.camera_ingress_node:main',
            'cam_front_ros_node = camera_bridge.cam_front_ros_node:main',
        ],
    },
)
