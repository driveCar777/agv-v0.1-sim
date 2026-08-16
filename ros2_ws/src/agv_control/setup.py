from setuptools import find_packages, setup
import os
from glob import glob


package_name = "agv_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_control"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    description="AGV Direct Control Bridge.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_cmd_vel_node = agv_control.ros.cmd_vel_node:main",
            "agv_odom_node = agv_control.ros.odom_node:main",
            "agv_control_bridge = agv_control.ros.bridge_node:main",
        ],
    },
)
