from setuptools import find_packages, setup


setup(
    name="agv_vision_mock",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_vision_mock"]),
        ("share/" + "agv_vision_mock", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agv_mock_face = agv_vision_mock.face:main",
            "agv_mock_qr = agv_vision_mock.qr:main",
            "agv_mock_object = agv_vision_mock.object_detector:main",
            "agv_mock_image = agv_vision_mock.image:main",
        ],
    },
)