from setuptools import find_packages, setup


setup(
    name="agv_navigation",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_navigation"]),
        ("share/" + "agv_navigation", ["package.xml"]),
        ("share/" + "agv_navigation/config", [
            "config/nav2_params.yaml",
            "config/local_costmap.yaml",
            "config/global_costmap.yaml",
        ]),
        ("share/" + "agv_navigation/launch", [
            "launch/nav2_bringup.launch.py",
            "launch/static_tf.launch.py",
        ]),
        ("share/" + "agv_navigation/maps", [
            "maps/empty_room.yaml",
            "maps/empty_room.pgm",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    description="Nav2 bringup & config for AGV.",
    license="Apache-2.0",
)