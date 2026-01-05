#!/usr/bin/env python3
"""
LiDAR-GPS Fusion Node

Fuses LiDAR odometry with UM982 RTK-GPS for drift-corrected ENU positioning.
"""

import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from um982_ros.msg import UM982Status
from lidar_gps_fusion.msg import FusionStatus
from .math_utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    normalize_angle,
    heading_to_enu_yaw,
    rotate_2d
)


class State(IntEnum):
    WAITING_INIT = 0
    RUNNING = 1


class RTKStatus(IntEnum):
    UNKNOWN = 0
    STANDALONE = 1
    DGPS = 2
    RTK_FIX = 4
    RTK_FLOAT = 5


class LidarGpsFusionNode(Node):
    """LiDAR-GPS Fusion ROS2 Node"""

    def __init__(self):
        super().__init__('lidar_gps_fusion_node')

        # Declare parameters
        self._declare_parameters()

        # Load parameters
        self._load_parameters()

        # State initialization
        self.state = State.WAITING_INIT
        self.initialized = False
        self.init_timestamp = 0.0

        # Transformation offsets (map -> odom)
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.yaw_offset = 0.0

        # Latest data
        self.latest_lidar_odom = None
        self.latest_gps_status = None

        # Diagnostics
        self.last_error = (0.0, 0.0, 0.0, 0.0)
        self.gps_used = False

        # Subscribers
        self.lidar_sub = self.create_subscription(
            Odometry,
            self.lidar_odom_topic,
            self._lidar_callback,
            10
        )
        self.gps_sub = self.create_subscription(
            UM982Status,
            self.gps_status_topic,
            self._gps_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_odom_topic,
            10
        )

        if self.publish_diagnostics:
            self.status_pub = self.create_publisher(
                FusionStatus,
                '/fusion/status',
                10
            )
            self.create_timer(
                1.0 / self.diagnostics_rate,
                self._publish_status
            )

        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Initialization timeout timer
        if self.init_timeout > 0:
            self.init_timer = self.create_timer(
                self.init_timeout,
                self._init_timeout_callback
            )

        self.get_logger().info('LiDAR-GPS Fusion Node started')
        self.get_logger().info(f'  LiDAR topic: {self.lidar_odom_topic}')
        self.get_logger().info(f'  GPS topic: {self.gps_status_topic}')
        self.get_logger().info(f'  Output topic: {self.output_odom_topic}')
        self.get_logger().info(f'  Correction rate: {self.correction_rate}')
        self.get_logger().info('Waiting for RTK Fix to initialize...')

    def _declare_parameters(self):
        """Declare all ROS2 parameters"""
        # Input topics
        self.declare_parameter('lidar_odom_topic', '/odom')
        self.declare_parameter('gps_status_topic', '/gps/status')

        # Output settings
        self.declare_parameter('output_odom_topic', '/fused_odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        # GPS usage condition
        self.declare_parameter('gps_use_condition', 'rtk_fix')
        self.declare_parameter('hdop_threshold', 1.0)

        # Correction settings
        self.declare_parameter('correction_rate', 0.05)
        self.declare_parameter('correct_xy', True)
        self.declare_parameter('correct_z', True)
        self.declare_parameter('correct_yaw', True)

        # Initialization
        self.declare_parameter('wait_for_gps_init', True)
        self.declare_parameter('init_timeout', 30.0)

        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_rate', 1.0)

    def _load_parameters(self):
        """Load parameters from ROS2 parameter server"""
        self.lidar_odom_topic = self.get_parameter('lidar_odom_topic').value
        self.gps_status_topic = self.get_parameter('gps_status_topic').value

        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

        self.gps_use_condition = self.get_parameter('gps_use_condition').value
        self.hdop_threshold = self.get_parameter('hdop_threshold').value

        self.correction_rate = self.get_parameter('correction_rate').value
        self.correct_xy = self.get_parameter('correct_xy').value
        self.correct_z = self.get_parameter('correct_z').value
        self.correct_yaw = self.get_parameter('correct_yaw').value

        self.wait_for_gps_init = self.get_parameter('wait_for_gps_init').value
        self.init_timeout = self.get_parameter('init_timeout').value

        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.diagnostics_rate = self.get_parameter('diagnostics_rate').value

    def _is_gps_usable(self, gps: UM982Status) -> bool:
        """Check if GPS data should be used for correction"""
        if not gps.enu_valid or not gps.heading_valid:
            return False

        if self.gps_use_condition == 'rtk_fix':
            return gps.rtk_status == RTKStatus.RTK_FIX
        elif self.gps_use_condition == 'hdop':
            return (gps.rtk_status == RTKStatus.RTK_FIX and
                    gps.hdop < self.hdop_threshold)
        return False

    def _initialize(self, lidar: Odometry, gps: UM982Status):
        """Initialize fusion with first RTK fix"""
        # Extract LiDAR pose
        lidar_x = lidar.pose.pose.position.x
        lidar_y = lidar.pose.pose.position.y
        lidar_z = lidar.pose.pose.position.z
        lidar_yaw = quaternion_to_yaw(
            lidar.pose.pose.orientation.x,
            lidar.pose.pose.orientation.y,
            lidar.pose.pose.orientation.z,
            lidar.pose.pose.orientation.w
        )

        # Extract GPS pose (ENU)
        gps_x = gps.enu_east
        gps_y = gps.enu_north
        gps_z = gps.enu_up
        gps_yaw = heading_to_enu_yaw(gps.heading)

        # Calculate yaw offset
        self.yaw_offset = gps_yaw - lidar_yaw

        # Calculate position offset (with rotation)
        rotated_lidar_x, rotated_lidar_y = rotate_2d(lidar_x, lidar_y, self.yaw_offset)

        self.x_offset = gps_x - rotated_lidar_x
        self.y_offset = gps_y - rotated_lidar_y
        self.z_offset = gps_z - lidar_z

        self.initialized = True
        self.init_timestamp = self.get_clock().now().nanoseconds / 1e9
        self.state = State.RUNNING

        # Cancel init timeout timer
        if hasattr(self, 'init_timer'):
            self.init_timer.cancel()

        self.get_logger().info(
            f'Fusion initialized: offsets=({self.x_offset:.3f}, {self.y_offset:.3f}, '
            f'{self.z_offset:.3f}, {math.degrees(self.yaw_offset):.1f}deg)'
        )

    def _transform_to_map(self, lidar: Odometry) -> tuple:
        """
        Transform LiDAR odom to map frame

        Returns:
            Tuple of (x, y, z, yaw)
        """
        lx = lidar.pose.pose.position.x
        ly = lidar.pose.pose.position.y
        lz = lidar.pose.pose.position.z
        l_yaw = quaternion_to_yaw(
            lidar.pose.pose.orientation.x,
            lidar.pose.pose.orientation.y,
            lidar.pose.pose.orientation.z,
            lidar.pose.pose.orientation.w
        )

        # Apply rotation and offset
        rotated_x, rotated_y = rotate_2d(lx, ly, self.yaw_offset)

        map_x = rotated_x + self.x_offset
        map_y = rotated_y + self.y_offset
        map_z = lz + self.z_offset
        map_yaw = normalize_angle(l_yaw + self.yaw_offset)

        return (map_x, map_y, map_z, map_yaw)

    def _update_correction(self, gps: UM982Status, current_pose: tuple):
        """Update offsets based on GPS data"""
        current_x, current_y, current_z, current_yaw = current_pose
        gps_yaw = heading_to_enu_yaw(gps.heading)

        # Calculate errors
        error_x = gps.enu_east - current_x
        error_y = gps.enu_north - current_y
        error_z = gps.enu_up - current_z
        error_yaw = normalize_angle(gps_yaw - current_yaw)

        # Store for diagnostics
        self.last_error = (error_x, error_y, error_z, math.degrees(error_yaw))

        # Apply correction
        alpha = self.correction_rate

        if self.correct_xy:
            self.x_offset += alpha * error_x
            self.y_offset += alpha * error_y

        if self.correct_z:
            self.z_offset += alpha * error_z

        if self.correct_yaw:
            self.yaw_offset += alpha * error_yaw

    def _lidar_callback(self, msg: Odometry):
        """LiDAR odometry callback"""
        self.latest_lidar_odom = msg

        # Not initialized yet - don't output
        if self.state == State.WAITING_INIT:
            return

        # Transform to map frame
        map_pose = self._transform_to_map(msg)

        # Create output odometry
        out_odom = Odometry()
        out_odom.header.stamp = msg.header.stamp
        out_odom.header.frame_id = self.map_frame
        out_odom.child_frame_id = msg.child_frame_id

        # Set position
        out_odom.pose.pose.position.x = map_pose[0]
        out_odom.pose.pose.position.y = map_pose[1]
        out_odom.pose.pose.position.z = map_pose[2]

        # Set orientation
        qx, qy, qz, qw = yaw_to_quaternion(map_pose[3])
        out_odom.pose.pose.orientation.x = qx
        out_odom.pose.pose.orientation.y = qy
        out_odom.pose.pose.orientation.z = qz
        out_odom.pose.pose.orientation.w = qw

        # Transform twist (rotate velocity direction)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        rotated_vx, rotated_vy = rotate_2d(vx, vy, self.yaw_offset)
        out_odom.twist.twist.linear.x = rotated_vx
        out_odom.twist.twist.linear.y = rotated_vy
        out_odom.twist.twist.linear.z = msg.twist.twist.linear.z
        out_odom.twist.twist.angular = msg.twist.twist.angular

        # Publish
        self.odom_pub.publish(out_odom)

        # Publish TF
        if self.publish_tf:
            self._publish_tf(msg.header.stamp)

    def _gps_callback(self, msg: UM982Status):
        """GPS status callback"""
        self.latest_gps_status = msg

        # Check if GPS is usable
        gps_usable = self._is_gps_usable(msg)
        self.gps_used = gps_usable

        # Initialization
        if self.state == State.WAITING_INIT:
            if gps_usable and self.latest_lidar_odom is not None:
                self._initialize(self.latest_lidar_odom, msg)
            return

        # Update correction if GPS available
        if gps_usable and self.latest_lidar_odom is not None:
            current_pose = self._transform_to_map(self.latest_lidar_odom)
            self._update_correction(msg, current_pose)

    def _publish_tf(self, stamp):
        """Publish map -> odom transform"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = self.x_offset
        t.transform.translation.y = self.y_offset
        t.transform.translation.z = self.z_offset

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw_offset)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _publish_status(self):
        """Publish fusion status for diagnostics"""
        status = FusionStatus()
        status.state = self.state
        status.initialized = self.initialized
        status.init_timestamp = self.init_timestamp

        # GPS info
        if self.latest_gps_status:
            status.gps_available = True
            status.rtk_status = self.latest_gps_status.rtk_status
            status.hdop = self.latest_gps_status.hdop
        else:
            status.gps_available = False
            status.rtk_status = 0
            status.hdop = 99.0

        status.gps_used = self.gps_used

        # Error
        status.error_x = self.last_error[0]
        status.error_y = self.last_error[1]
        status.error_z = self.last_error[2]
        status.error_yaw = self.last_error[3]
        status.error_distance = math.sqrt(
            self.last_error[0]**2 + self.last_error[1]**2
        )

        # Corrections
        status.correction_x = self.x_offset
        status.correction_y = self.y_offset
        status.correction_z = self.z_offset
        status.correction_yaw = math.degrees(self.yaw_offset)

        self.status_pub.publish(status)

    def _init_timeout_callback(self):
        """Called when initialization timeout expires"""
        if self.state == State.WAITING_INIT:
            self.get_logger().warn(
                f'Initialization timeout ({self.init_timeout}s) - '
                'Starting without GPS. Large corrections may occur when RTK becomes available.'
            )
            # Initialize with zero offsets
            self.initialized = True
            self.init_timestamp = self.get_clock().now().nanoseconds / 1e9
            self.state = State.RUNNING

        self.init_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = LidarGpsFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
