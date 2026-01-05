#!/usr/bin/env python3
"""
UM982 ROS2 Launch File with NTRIP Support

Usage:
  ros2 launch um982_ros um982_ntrip.launch.py ntrip_host:=rtk.example.com ntrip_mountpoint:=RTCM3 ntrip_user:=user ntrip_password:=pass
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Serial port arguments
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

    # NTRIP arguments
    ntrip_host_arg = DeclareLaunchArgument(
        'ntrip_host',
        default_value='',
        description='NTRIP caster hostname'
    )

    ntrip_port_arg = DeclareLaunchArgument(
        'ntrip_port',
        default_value='2101',
        description='NTRIP caster port'
    )

    ntrip_mountpoint_arg = DeclareLaunchArgument(
        'ntrip_mountpoint',
        default_value='',
        description='NTRIP mountpoint'
    )

    ntrip_user_arg = DeclareLaunchArgument(
        'ntrip_user',
        default_value='',
        description='NTRIP username'
    )

    ntrip_password_arg = DeclareLaunchArgument(
        'ntrip_password',
        default_value='',
        description='NTRIP password'
    )

    ntrip_gga_interval_arg = DeclareLaunchArgument(
        'ntrip_gga_interval',
        default_value='5.0',
        description='GGA send interval in seconds'
    )

    # UM982 node with NTRIP
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
            'ntrip.enabled': True,
            'ntrip.host': LaunchConfiguration('ntrip_host'),
            'ntrip.port': LaunchConfiguration('ntrip_port'),
            'ntrip.mountpoint': LaunchConfiguration('ntrip_mountpoint'),
            'ntrip.user': LaunchConfiguration('ntrip_user'),
            'ntrip.password': LaunchConfiguration('ntrip_password'),
            'ntrip.gga_interval': LaunchConfiguration('ntrip_gga_interval'),
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        output_rate_arg,
        frame_id_arg,
        ntrip_host_arg,
        ntrip_port_arg,
        ntrip_mountpoint_arg,
        ntrip_user_arg,
        ntrip_password_arg,
        ntrip_gga_interval_arg,
        um982_node,
    ])
