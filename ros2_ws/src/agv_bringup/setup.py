from setuptools import find_packages, setup

setup(
    name="agv_bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_bringup"]),
        ("share/agv_bringup", ["package.xml"]),
        ("share/agv_bringup/launch", [
            "launch/agv_sim_bringup.launch.py",
            "launch/agv_nav_bringup.launch.py",
            "launch/agv_full_sim_bringup.launch.py",
            "launch/agv_real_bringup.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
)
