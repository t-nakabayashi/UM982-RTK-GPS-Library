from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'um982_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 driver for UM982 dual-antenna RTK GNSS module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_node = um982_ros.um982_node:main',
        ],
    },
)
