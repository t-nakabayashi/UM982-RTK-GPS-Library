#!/usr/bin/env python3
"""
UM982 ROS2 Launch File

Usage:
  ros2 launch um982_ros um982.launch.py
  ros2 launch um982_ros um982.launch.py port:=/dev/ttyUSB1 baud:=115200
  ros2 launch um982_ros um982.launch.py params_file:=/path/to/params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for UM982'
    )

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    output_rate_arg = DeclareLaunchArgument(
        'output_rate',
        default_value='10',
        description='GPS output rate in Hz'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='Frame ID for GPS messages'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to parameter file (optional)'
    )

    # UM982 node
    um982_node = Node(
        package='um982_ros',
        executable='um982_node',
        name='um982_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'output_rate': LaunchConfiguration('output_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        remappings=[
            ('gps/fix', 'gps/fix'),
            ('gps/vel', 'gps/vel'),
            ('gps/status', 'gps/status'),
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        output_rate_arg,
        frame_id_arg,
        params_file_arg,
        um982_node,
    ])
