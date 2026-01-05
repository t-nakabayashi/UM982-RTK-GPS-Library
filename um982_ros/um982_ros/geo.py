"""
Geodetic coordinate conversion utilities.

WGS84 (latitude, longitude, altitude) to ENU (East, North, Up) conversion
via ECEF (Earth-Centered, Earth-Fixed) intermediate coordinates.
"""

import math
from typing import Tuple, Optional
from dataclasses import dataclass


# WGS84 ellipsoid parameters
WGS84_A = 6378137.0  # Semi-major axis (equatorial radius) in meters
WGS84_F = 1.0 / 298.257223563  # Flattening
WGS84_B = WGS84_A * (1 - WGS84_F)  # Semi-minor axis (polar radius)
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # First eccentricity squared


@dataclass
class ENUOrigin:
    """ENU coordinate system origin."""
    lat: float  # Latitude in degrees
    lon: float  # Longitude in degrees
    alt: float  # Altitude in meters
    # Pre-computed ECEF coordinates
    ecef_x: float = 0.0
    ecef_y: float = 0.0
    ecef_z: float = 0.0
    # Pre-computed trigonometric values
    sin_lat: float = 0.0
    cos_lat: float = 0.0
    sin_lon: float = 0.0
    cos_lon: float = 0.0

    def __post_init__(self):
        """Pre-compute values for efficient repeated conversions."""
        lat_rad = math.radians(self.lat)
        lon_rad = math.radians(self.lon)

        self.sin_lat = math.sin(lat_rad)
        self.cos_lat = math.cos(lat_rad)
        self.sin_lon = math.sin(lon_rad)
        self.cos_lon = math.cos(lon_rad)

        self.ecef_x, self.ecef_y, self.ecef_z = geodetic_to_ecef(
            self.lat, self.lon, self.alt
        )


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> Tuple[float, float, float]:
    """
    Convert WGS84 geodetic coordinates to ECEF (Earth-Centered, Earth-Fixed).

    Args:
        lat_deg: Latitude in decimal degrees
        lon_deg: Longitude in decimal degrees
        alt: Altitude above ellipsoid in meters

    Returns:
        Tuple of (X, Y, Z) in meters (ECEF coordinates)
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Radius of curvature in the prime vertical
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt) * sin_lat

    return x, y, z


def ecef_to_enu(
    x: float, y: float, z: float,
    origin: ENUOrigin
) -> Tuple[float, float, float]:
    """
    Convert ECEF coordinates to ENU relative to origin.

    Args:
        x, y, z: ECEF coordinates in meters
        origin: ENU origin with pre-computed values

    Returns:
        Tuple of (East, North, Up) in meters
    """
    dx = x - origin.ecef_x
    dy = y - origin.ecef_y
    dz = z - origin.ecef_z

    # Rotation matrix from ECEF to ENU
    e = -origin.sin_lon * dx + origin.cos_lon * dy
    n = (-origin.sin_lat * origin.cos_lon * dx
         - origin.sin_lat * origin.sin_lon * dy
         + origin.cos_lat * dz)
    u = (origin.cos_lat * origin.cos_lon * dx
         + origin.cos_lat * origin.sin_lon * dy
         + origin.sin_lat * dz)

    return e, n, u


def geodetic_to_enu(
    lat_deg: float, lon_deg: float, alt: float,
    origin: ENUOrigin
) -> Tuple[float, float, float]:
    """
    Convert WGS84 geodetic coordinates to ENU relative to origin.

    Args:
        lat_deg: Latitude in decimal degrees
        lon_deg: Longitude in decimal degrees
        alt: Altitude above ellipsoid in meters
        origin: ENU origin

    Returns:
        Tuple of (East, North, Up) in meters
    """
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    return ecef_to_enu(x, y, z, origin)


def heading_to_quaternion(heading_deg: float) -> Tuple[float, float, float, float]:
    """
    Convert heading (yaw) angle to quaternion.

    Assumes ENU frame where:
    - heading 0 = North (+Y)
    - heading 90 = East (+X)
    - rotation is clockwise when viewed from above

    Args:
        heading_deg: Heading in degrees (0-360, clockwise from North)

    Returns:
        Tuple of (x, y, z, w) quaternion components
    """
    # Convert heading to yaw in ENU frame
    # In ENU: yaw 0 = East (+X), counter-clockwise positive
    # Heading: 0 = North (+Y), clockwise positive
    # yaw = 90 - heading (and negate for counter-clockwise)
    yaw_rad = math.radians(90.0 - heading_deg)

    # Quaternion from yaw only (roll=0, pitch=0)
    half_yaw = yaw_rad / 2.0

    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    return qx, qy, qz, qw


def identity_quaternion() -> Tuple[float, float, float, float]:
    """Return identity quaternion (no rotation)."""
    return 0.0, 0.0, 0.0, 1.0
