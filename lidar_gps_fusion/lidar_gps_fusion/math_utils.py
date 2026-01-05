"""
Mathematical utility functions for LiDAR-GPS Fusion
"""

import math
from typing import Tuple


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Extract yaw angle from quaternion

    Args:
        x, y, z, w: Quaternion components

    Returns:
        Yaw angle in radians
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw angle to quaternion (roll=0, pitch=0)

    Args:
        yaw: Yaw angle in radians

    Returns:
        Tuple of (x, y, z, w)
    """
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi]

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def heading_to_enu_yaw(heading_deg: float) -> float:
    """
    Convert GPS heading to ENU yaw

    GPS heading: North=0, clockwise positive
    ENU yaw: East=0 (X-axis), counterclockwise positive

    Args:
        heading_deg: GPS heading in degrees

    Returns:
        ENU yaw in radians
    """
    # heading=0 (North) -> yaw=pi/2
    # heading=90 (East) -> yaw=0
    # heading=180 (South) -> yaw=-pi/2
    # heading=270 (West) -> yaw=pi
    return math.radians(90.0 - heading_deg)


def enu_yaw_to_heading(yaw_rad: float) -> float:
    """
    Convert ENU yaw to GPS heading

    Args:
        yaw_rad: ENU yaw in radians

    Returns:
        GPS heading in degrees [0, 360)
    """
    heading = 90.0 - math.degrees(yaw_rad)
    while heading < 0:
        heading += 360.0
    while heading >= 360.0:
        heading -= 360.0
    return heading


def rotate_2d(x: float, y: float, angle: float) -> Tuple[float, float]:
    """
    Rotate 2D point by angle

    Args:
        x, y: Point coordinates
        angle: Rotation angle in radians (counterclockwise positive)

    Returns:
        Rotated (x, y) coordinates
    """
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        cos_a * x - sin_a * y,
        sin_a * x + cos_a * y
    )
