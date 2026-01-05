#!/usr/bin/env python3
"""
LiDAR-GPS Fusion Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('lidar_gps_fusion')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    # Declare launch arguments
    return LaunchDescription([
        # Input topics
        DeclareLaunchArgument(
            'lidar_odom_topic',
            default_value='/odom',
            description='LiDAR odometry topic'
        ),
        DeclareLaunchArgument(
            'gps_status_topic',
            default_value='/gps/status',
            description='GPS status topic'
        ),

        # Output settings
        DeclareLaunchArgument(
            'output_odom_topic',
            default_value='/fused_odom',
            description='Output odometry topic'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish TF (map -> odom)'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame name'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odom frame name'
        ),

        # GPS usage
        DeclareLaunchArgument(
            'gps_use_condition',
            default_value='rtk_fix',
            description='GPS usage condition: rtk_fix or hdop'
        ),
        DeclareLaunchArgument(
            'hdop_threshold',
            default_value='1.0',
            description='HDOP threshold for hdop mode'
        ),

        # Correction settings
        DeclareLaunchArgument(
            'correction_rate',
            default_value='0.05',
            description='Correction rate (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'correct_xy',
            default_value='true',
            description='Correct XY position'
        ),
        DeclareLaunchArgument(
            'correct_z',
            default_value='true',
            description='Correct Z position'
        ),
        DeclareLaunchArgument(
            'correct_yaw',
            default_value='true',
            description='Correct yaw angle'
        ),

        # Initialization
        DeclareLaunchArgument(
            'wait_for_gps_init',
            default_value='true',
            description='Wait for RTK Fix at startup'
        ),
        DeclareLaunchArgument(
            'init_timeout',
            default_value='30.0',
            description='Initialization timeout (seconds)'
        ),

        # Node
        Node(
            package='lidar_gps_fusion',
            executable='fusion_node',
            name='lidar_gps_fusion',
            parameters=[{
                'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
                'gps_status_topic': LaunchConfiguration('gps_status_topic'),
                'output_odom_topic': LaunchConfiguration('output_odom_topic'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'gps_use_condition': LaunchConfiguration('gps_use_condition'),
                'hdop_threshold': LaunchConfiguration('hdop_threshold'),
                'correction_rate': LaunchConfiguration('correction_rate'),
                'correct_xy': LaunchConfiguration('correct_xy'),
                'correct_z': LaunchConfiguration('correct_z'),
                'correct_yaw': LaunchConfiguration('correct_yaw'),
                'wait_for_gps_init': LaunchConfiguration('wait_for_gps_init'),
                'init_timeout': LaunchConfiguration('init_timeout'),
                'publish_diagnostics': True,
                'diagnostics_rate': 1.0,
            }],
            output='screen'
        ),
    ])
