from setuptools import setup
import os
from glob import glob

package_name = 'agv_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f"{package_name}.agv_adapter", f"{package_name}.arm"],
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
    description='Robokit TCP API bridge + simulation for AGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agv_bridge_node = agv_bridge.agv_bridge_node:main',
            'agv_sim_node = agv_bridge.agv_sim_node:main',
            'agv_demo_nav = agv_bridge.agv_demo_nav:main',
            'robokit_mock_server = agv_bridge.robokit_mock_server:main',
        ],
    },
)
