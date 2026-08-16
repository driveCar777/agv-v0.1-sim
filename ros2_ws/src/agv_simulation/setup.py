from setuptools import find_packages, setup


setup(
    name="agv_simulation",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_simulation"]),
        ("share/" + "agv_simulation", ["package.xml"]),
        ("share/" + "agv_simulation/scenarios", [
            "scenarios/empty_room.yaml",
            "scenarios/obstacle_corridor.yaml",
            "scenarios/dynamic_obstacle.yaml",
            "scenarios/narrow_passage.yaml",
            "scenarios/emergency_stop.yaml",
        ]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_scenario_runner = agv_simulation.scenario_runner:main",
        ],
    },
)