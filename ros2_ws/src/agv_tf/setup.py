from setuptools import find_packages, setup


setup(
    name="agv_tf",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agv_tf"]),
        ("share/" + "agv_tf", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Team",
    maintainer_email="agv-team@example.com",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agv_static_tf = agv_tf.static_tf:main",
        ],
    },
)