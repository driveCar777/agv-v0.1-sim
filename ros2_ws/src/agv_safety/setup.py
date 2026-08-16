from setuptools import find_packages, setup


setup(
    name="agv_safety",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_safety"]),
        ("share/" + "agv_safety", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_safety_monitor = agv_safety.safety_monitor:main",
            "agv_collision_bridge = agv_safety.collision_bridge:main",
        ],
    },
)