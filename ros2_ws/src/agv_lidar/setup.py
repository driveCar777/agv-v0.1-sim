from setuptools import find_packages, setup


setup(
    name="agv_lidar",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_lidar"]),
        ("share/" + "agv_lidar", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_mock_lidar = agv_lidar.mock_lidar:main",
            "agv_dual_lidar_merger = agv_lidar.dual_lidar_merger:main",
        ],
    },
)