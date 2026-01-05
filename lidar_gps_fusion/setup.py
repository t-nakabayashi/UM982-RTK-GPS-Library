from setuptools import setup, find_packages

package_name = 'lidar_gps_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fusion.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='LiDAR-GPS Fusion for drift-corrected ENU positioning',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fusion_node = lidar_gps_fusion.fusion_node:main',
        ],
    },
)
