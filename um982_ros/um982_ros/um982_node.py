#!/usr/bin/env python3
"""
UM982 ROS2 Node

Publishes GPS data from UM982 dual-antenna GNSS module.
Outputs:
  - sensor_msgs/NavSatFix (standard GPS message)
  - geometry_msgs/TwistStamped (velocity)
  - um982_ros/UM982Status (full status with heading/pitch)
"""

import sys
import os

# Add parent directory to path to import um982 library
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

from um982_ros.msg import UM982Status
from um982 import UM982Client
from um982.types import PositionData


class UM982Node(Node):
    """ROS2 Node for UM982 RTK GPS"""

    RTK_STATUS_MAP = {
        'unknown': UM982Status.STATUS_UNKNOWN,
        'standalone': UM982Status.STATUS_STANDALONE,
        'dgps': UM982Status.STATUS_DGPS,
        'rtk_fix': UM982Status.STATUS_RTK_FIX,
        'rtk_float': UM982Status.STATUS_RTK_FLOAT,
    }

    NAVSAT_STATUS_MAP = {
        'unknown': NavSatStatus.STATUS_NO_FIX,
        'standalone': NavSatStatus.STATUS_FIX,
        'dgps': NavSatStatus.STATUS_SBAS_FIX,
        'rtk_fix': NavSatStatus.STATUS_GBAS_FIX,
        'rtk_float': NavSatStatus.STATUS_GBAS_FIX,
    }

    def __init__(self):
        super().__init__('um982_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('output_rate', 10)
        self.declare_parameter('frame_id', 'gps')

        # NTRIP parameters
        self.declare_parameter('ntrip.enabled', False)
        self.declare_parameter('ntrip.host', '')
        self.declare_parameter('ntrip.port', 2101)
        self.declare_parameter('ntrip.mountpoint', '')
        self.declare_parameter('ntrip.user', '')
        self.declare_parameter('ntrip.password', '')
        self.declare_parameter('ntrip.gga_interval', 5.0)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.output_rate = self.get_parameter('output_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.navsat_pub = self.create_publisher(NavSatFix, 'gps/fix', sensor_qos)
        self.vel_pub = self.create_publisher(TwistStamped, 'gps/vel', sensor_qos)
        self.status_pub = self.create_publisher(UM982Status, 'gps/status', sensor_qos)

        # Initialize UM982 client
        self.client = None
        self._init_client()

    def _init_client(self):
        """Initialize UM982 client and start reading"""
        try:
            self.client = UM982Client(
                port=self.port,
                baud=self.baud,
                output_rate=self.output_rate
            )
            self.client.set_position_callback(self._on_position)
            self.client.start()
            self.get_logger().info(f'UM982 client started on {self.port} @ {self.baud} baud')

            # Start NTRIP if enabled
            if self.get_parameter('ntrip.enabled').value:
                self._start_ntrip()

        except Exception as e:
            self.get_logger().error(f'Failed to initialize UM982 client: {e}')
            raise

    def _start_ntrip(self):
        """Start NTRIP client for RTK corrections"""
        try:
            host = self.get_parameter('ntrip.host').value
            port = self.get_parameter('ntrip.port').value
            mountpoint = self.get_parameter('ntrip.mountpoint').value
            user = self.get_parameter('ntrip.user').value
            password = self.get_parameter('ntrip.password').value
            gga_interval = self.get_parameter('ntrip.gga_interval').value

            if not host or not mountpoint:
                self.get_logger().warn('NTRIP enabled but host/mountpoint not configured')
                return

            self.client.start_ntrip(
                host=host,
                port=port,
                mountpoint=mountpoint,
                user=user,
                password=password,
                gga_interval=gga_interval
            )
            self.get_logger().info(f'NTRIP client started: {host}:{port}/{mountpoint}')

        except Exception as e:
            self.get_logger().error(f'Failed to start NTRIP: {e}')

    def _on_position(self, pos: PositionData):
        """Callback when new position data is available"""
        if not pos.is_valid:
            return

        stamp = self.get_clock().now().to_msg()

        # Publish all messages
        self._publish_navsat(pos, stamp)
        self._publish_velocity(pos, stamp)
        self._publish_status(pos, stamp)

    def _publish_navsat(self, pos: PositionData, stamp):
        """Publish sensor_msgs/NavSatFix"""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.latitude = pos.lat
        msg.longitude = pos.lon
        msg.altitude = pos.alt if pos.alt is not None else 0.0

        # Status
        msg.status.status = self.NAVSAT_STATUS_MAP.get(
            pos.rtk_state, NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariance (approximate based on RTK status)
        if pos.rtk_state == 'rtk_fix':
            # cm-level accuracy
            cov = 0.01 ** 2
        elif pos.rtk_state == 'rtk_float':
            # dm-level accuracy
            cov = 0.1 ** 2
        elif pos.rtk_state == 'dgps':
            # m-level accuracy
            cov = 1.0 ** 2
        else:
            # 10m-level accuracy
            cov = 10.0 ** 2

        msg.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov * 4  # vertical typically worse
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.navsat_pub.publish(msg)

    def _publish_velocity(self, pos: PositionData, stamp):
        """Publish geometry_msgs/TwistStamped"""
        if pos.speed_mps is None:
            return

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Convert speed and course to velocity components
        import math
        speed = pos.speed_mps
        course_rad = math.radians(pos.course) if pos.course is not None else 0.0

        # Velocity in ENU frame (East-North-Up)
        msg.twist.linear.x = speed * math.sin(course_rad)  # East
        msg.twist.linear.y = speed * math.cos(course_rad)  # North
        msg.twist.linear.z = 0.0

        self.vel_pub.publish(msg)

    def _publish_status(self, pos: PositionData, stamp):
        """Publish um982_ros/UM982Status (custom message with full data)"""
        msg = UM982Status()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Position
        msg.latitude = pos.lat
        msg.longitude = pos.lon
        msg.altitude = pos.alt if pos.alt is not None else 0.0

        # RTK Status
        msg.rtk_status = self.RTK_STATUS_MAP.get(
            pos.rtk_state, UM982Status.STATUS_UNKNOWN
        )
        msg.rtk_status_string = pos.rtk_state
        msg.num_satellites = pos.num_sats if pos.num_sats is not None else 0
        msg.hdop = float(pos.hdop) if pos.hdop is not None else 99.9
        msg.diff_age = float(pos.diff_age) if pos.diff_age is not None else 0.0

        # Dual Antenna Heading
        msg.heading_valid = pos.heading is not None
        if pos.heading is not None:
            msg.heading = float(pos.heading)
            msg.pitch = float(pos.pitch) if pos.pitch is not None else 0.0
            msg.heading_stddev = float(pos.heading_stddev) if pos.heading_stddev is not None else 0.0
            msg.pitch_stddev = float(pos.pitch_stddev) if pos.pitch_stddev is not None else 0.0
            msg.baseline = float(pos.baseline_m) if pos.baseline_m is not None else 0.0

        # Velocity
        msg.velocity_valid = pos.speed_mps is not None
        if pos.speed_mps is not None:
            msg.speed_mps = float(pos.speed_mps)
            msg.speed_knots = float(pos.speed_knots) if pos.speed_knots is not None else 0.0
            msg.course = float(pos.course) if pos.course is not None else 0.0

        # RTCM statistics
        msg.rtcm_bytes_received = self.client.get_rtcm_bytes() if self.client else 0

        self.status_pub.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.client:
            self.get_logger().info('Stopping UM982 client...')
            self.client.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = UM982Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
