from setuptools import find_packages, setup


setup(
    name="agv_ultrasonic",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_ultrasonic"]),
        ("share/" + "agv_ultrasonic", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_ultrasonic_driver = agv_ultrasonic.driver:main",
            "agv_ultrasonic_cluster = agv_ultrasonic.cluster:main",
        ],
    },
)