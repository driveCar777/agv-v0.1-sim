from setuptools import setup
import os
from glob import glob

package_name = 'delivery_web'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name, f"{package_name}.vision", f"{package_name}.arm"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'www'), [p for p in glob('www/*') if not __import__('os').path.isdir(p)]),
        (os.path.join('share', package_name, 'www', 'debug'), glob('www/debug/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delivery',
    maintainer_email='dev@example.com',
    description='Web dashboard for offline delivery robot sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard_node = delivery_web.dashboard_node:main',
        ],
    },
)
