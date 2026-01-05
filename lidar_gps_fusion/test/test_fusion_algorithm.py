#!/usr/bin/env python3
"""
LiDAR-GPS Fusion Algorithm Simulation and Visualization

This script simulates:
1. LiDAR odometry with drift
2. GPS data with varying RTK status
3. Fusion algorithm behavior

Usage:
    python3 test_fusion_algorithm.py
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import IntEnum


class RTKStatus(IntEnum):
    UNKNOWN = 0
    STANDALONE = 1
    DGPS = 2
    RTK_FIX = 4
    RTK_FLOAT = 5


@dataclass
class LidarOdom:
    """Simulated LiDAR odometry data"""
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float  # radians


@dataclass
class GPSStatus:
    """Simulated GPS status data"""
    timestamp: float
    enu_east: float
    enu_north: float
    enu_up: float
    heading: float  # degrees (north=0, clockwise positive)
    rtk_status: RTKStatus
    hdop: float
    enu_valid: bool
    heading_valid: bool


@dataclass
class FusedPose:
    """Fused pose output"""
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float  # radians


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def heading_to_enu_yaw(heading_deg: float) -> float:
    """
    Convert GPS heading (north=0, clockwise) to ENU yaw (east=0, counterclockwise)
    """
    return math.radians(90.0 - heading_deg)


def enu_yaw_to_heading(yaw_rad: float) -> float:
    """
    Convert ENU yaw to GPS heading
    """
    heading = 90.0 - math.degrees(yaw_rad)
    while heading < 0:
        heading += 360
    while heading >= 360:
        heading -= 360
    return heading


class SimulationDataGenerator:
    """Generate simulated sensor data"""

    def __init__(self, duration: float = 60.0, dt: float = 0.1):
        self.duration = duration
        self.dt = dt
        self.time_steps = np.arange(0, duration, dt)

    def generate_ground_truth(self) -> List[Tuple[float, float, float, float, float]]:
        """
        Generate ground truth trajectory (figure-8 pattern)
        Returns: [(t, x, y, z, yaw), ...]
        """
        trajectory = []
        for t in self.time_steps:
            # Figure-8 pattern
            x = 20 * math.sin(2 * math.pi * t / 30)
            y = 10 * math.sin(4 * math.pi * t / 30)
            z = 0.5 * math.sin(2 * math.pi * t / 60)  # slight Z variation

            # Yaw follows velocity direction
            if t > 0:
                dx = x - trajectory[-1][1]
                dy = y - trajectory[-1][2]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            trajectory.append((t, x, y, z, yaw))
        return trajectory

    def generate_lidar_odom(self, ground_truth: List[Tuple],
                           drift_rate: Tuple[float, float, float, float] = (0.01, 0.01, 0.005, 0.001),
                           noise_std: Tuple[float, float, float, float] = (0.02, 0.02, 0.01, 0.005)
                           ) -> List[LidarOdom]:
        """
        Generate LiDAR odometry with drift and noise
        drift_rate: (dx, dy, dz, dyaw) per second
        noise_std: standard deviation for (x, y, z, yaw)
        """
        lidar_data = []
        cumulative_drift = [0.0, 0.0, 0.0, 0.0]

        for i, (t, x, y, z, yaw) in enumerate(ground_truth):
            # Add cumulative drift
            cumulative_drift[0] += drift_rate[0] * self.dt
            cumulative_drift[1] += drift_rate[1] * self.dt
            cumulative_drift[2] += drift_rate[2] * self.dt
            cumulative_drift[3] += drift_rate[3] * self.dt

            # Add noise
            noise = [
                np.random.normal(0, noise_std[0]),
                np.random.normal(0, noise_std[1]),
                np.random.normal(0, noise_std[2]),
                np.random.normal(0, noise_std[3])
            ]

            lidar_data.append(LidarOdom(
                timestamp=t,
                x=x + cumulative_drift[0] + noise[0],
                y=y + cumulative_drift[1] + noise[1],
                z=z + cumulative_drift[2] + noise[2],
                yaw=yaw + cumulative_drift[3] + noise[3]
            ))

        return lidar_data

    def generate_gps_data(self, ground_truth: List[Tuple],
                         rtk_available_ranges: List[Tuple[float, float]] = None,
                         noise_std: Tuple[float, float, float, float] = (0.02, 0.02, 0.05, 1.0)
                         ) -> List[GPSStatus]:
        """
        Generate GPS data with RTK status changes
        rtk_available_ranges: list of (start_time, end_time) when RTK fix is available
        """
        if rtk_available_ranges is None:
            # Default: RTK available 0-20s, unavailable 20-40s, available 40-60s
            rtk_available_ranges = [(0, 20), (40, 60)]

        gps_data = []

        for t, x, y, z, yaw in ground_truth:
            # Determine RTK status
            rtk_fix = any(start <= t <= end for start, end in rtk_available_ranges)

            if rtk_fix:
                rtk_status = RTKStatus.RTK_FIX
                hdop = 0.8
                pos_noise = noise_std
            else:
                rtk_status = RTKStatus.STANDALONE
                hdop = 3.0
                pos_noise = (2.0, 2.0, 5.0, 10.0)  # Much larger noise when no RTK

            # Convert yaw to heading
            heading = enu_yaw_to_heading(yaw)

            gps_data.append(GPSStatus(
                timestamp=t,
                enu_east=x + np.random.normal(0, pos_noise[0]),
                enu_north=y + np.random.normal(0, pos_noise[1]),
                enu_up=z + np.random.normal(0, pos_noise[2]),
                heading=heading + np.random.normal(0, pos_noise[3]),
                rtk_status=rtk_status,
                hdop=hdop,
                enu_valid=True,
                heading_valid=True
            ))

        return gps_data


class LidarGpsFusion:
    """LiDAR-GPS Fusion Algorithm"""

    def __init__(self,
                 correction_rate: float = 0.05,
                 gps_use_condition: str = "rtk_fix",
                 hdop_threshold: float = 1.0,
                 correct_xy: bool = True,
                 correct_z: bool = True,
                 correct_yaw: bool = True):

        self.correction_rate = correction_rate
        self.gps_use_condition = gps_use_condition
        self.hdop_threshold = hdop_threshold
        self.correct_xy = correct_xy
        self.correct_z = correct_z
        self.correct_yaw = correct_yaw

        # State
        self.initialized = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.yaw_offset = 0.0

        # Diagnostics
        self.errors_history = []
        self.corrections_history = []
        self.gps_used_history = []

    def is_gps_usable(self, gps: GPSStatus) -> bool:
        """Check if GPS data should be used for correction"""
        if not gps.enu_valid or not gps.heading_valid:
            return False

        if self.gps_use_condition == "rtk_fix":
            return gps.rtk_status == RTKStatus.RTK_FIX
        elif self.gps_use_condition == "hdop":
            return (gps.rtk_status == RTKStatus.RTK_FIX and
                    gps.hdop < self.hdop_threshold)
        return False

    def initialize(self, lidar: LidarOdom, gps: GPSStatus):
        """Initialize fusion with first RTK fix"""
        gps_yaw = heading_to_enu_yaw(gps.heading)

        # Calculate yaw offset
        self.yaw_offset = gps_yaw - lidar.yaw

        # Calculate position offset (with rotation)
        cos_y = math.cos(self.yaw_offset)
        sin_y = math.sin(self.yaw_offset)

        rotated_lidar_x = cos_y * lidar.x - sin_y * lidar.y
        rotated_lidar_y = sin_y * lidar.x + cos_y * lidar.y

        self.x_offset = gps.enu_east - rotated_lidar_x
        self.y_offset = gps.enu_north - rotated_lidar_y
        self.z_offset = gps.enu_up - lidar.z

        self.initialized = True
        print(f"[INIT] t={lidar.timestamp:.1f}s: offsets=({self.x_offset:.3f}, {self.y_offset:.3f}, {self.z_offset:.3f}, {math.degrees(self.yaw_offset):.1f}deg)")

    def transform_to_map(self, lidar: LidarOdom) -> FusedPose:
        """Transform LiDAR odom to map frame"""
        cos_y = math.cos(self.yaw_offset)
        sin_y = math.sin(self.yaw_offset)

        map_x = cos_y * lidar.x - sin_y * lidar.y + self.x_offset
        map_y = sin_y * lidar.x + cos_y * lidar.y + self.y_offset
        map_z = lidar.z + self.z_offset
        map_yaw = lidar.yaw + self.yaw_offset

        return FusedPose(
            timestamp=lidar.timestamp,
            x=map_x,
            y=map_y,
            z=map_z,
            yaw=normalize_angle(map_yaw)
        )

    def update_correction(self, gps: GPSStatus, current_fused: FusedPose):
        """Update offsets based on GPS data"""
        gps_yaw = heading_to_enu_yaw(gps.heading)

        # Calculate errors
        error_x = gps.enu_east - current_fused.x
        error_y = gps.enu_north - current_fused.y
        error_z = gps.enu_up - current_fused.z
        error_yaw = normalize_angle(gps_yaw - current_fused.yaw)

        # Store for diagnostics
        self.errors_history.append({
            't': gps.timestamp,
            'error_x': error_x,
            'error_y': error_y,
            'error_z': error_z,
            'error_yaw': math.degrees(error_yaw),
            'error_dist': math.sqrt(error_x**2 + error_y**2)
        })

        # Apply correction
        alpha = self.correction_rate

        if self.correct_xy:
            self.x_offset += alpha * error_x
            self.y_offset += alpha * error_y

        if self.correct_z:
            self.z_offset += alpha * error_z

        if self.correct_yaw:
            self.yaw_offset += alpha * error_yaw

        # Store correction history
        self.corrections_history.append({
            't': gps.timestamp,
            'x_offset': self.x_offset,
            'y_offset': self.y_offset,
            'z_offset': self.z_offset,
            'yaw_offset': math.degrees(self.yaw_offset)
        })

    def process(self, lidar: LidarOdom, gps: GPSStatus) -> Optional[FusedPose]:
        """Process one step of fusion"""
        gps_usable = self.is_gps_usable(gps)
        self.gps_used_history.append({'t': lidar.timestamp, 'used': gps_usable})

        # Initialization
        if not self.initialized:
            if gps_usable:
                self.initialize(lidar, gps)
            else:
                return None  # Wait for RTK fix

        # Transform to map frame
        fused = self.transform_to_map(lidar)

        # Update correction if GPS available
        if gps_usable:
            self.update_correction(gps, fused)
            # Re-transform with updated offsets
            fused = self.transform_to_map(lidar)

        return fused


def run_simulation():
    """Run simulation and generate visualization"""
    print("=" * 60)
    print("LiDAR-GPS Fusion Algorithm Simulation")
    print("=" * 60)

    # Generate data
    generator = SimulationDataGenerator(duration=60.0, dt=0.1)
    ground_truth = generator.generate_ground_truth()

    # LiDAR with drift
    lidar_data = generator.generate_lidar_odom(
        ground_truth,
        drift_rate=(0.02, 0.015, 0.005, 0.002),  # drift per second
        noise_std=(0.02, 0.02, 0.01, 0.005)
    )

    # GPS with RTK availability windows
    gps_data = generator.generate_gps_data(
        ground_truth,
        rtk_available_ranges=[(0, 15), (35, 60)],  # RTK unavailable 15-35s
        noise_std=(0.02, 0.02, 0.05, 1.0)
    )

    # Run fusion
    fusion = LidarGpsFusion(
        correction_rate=0.05,
        gps_use_condition="rtk_fix",
        correct_xy=True,
        correct_z=True,
        correct_yaw=True
    )

    fused_poses = []
    for lidar, gps in zip(lidar_data, gps_data):
        fused = fusion.process(lidar, gps)
        if fused:
            fused_poses.append(fused)

    print(f"\nProcessed {len(fused_poses)} fused poses")

    # Visualization
    create_visualization(ground_truth, lidar_data, gps_data, fused_poses, fusion)


def create_visualization(ground_truth, lidar_data, gps_data, fused_poses, fusion):
    """Create comprehensive visualization"""

    fig = plt.figure(figsize=(16, 12))

    # Extract data for plotting
    gt_t = [p[0] for p in ground_truth]
    gt_x = [p[1] for p in ground_truth]
    gt_y = [p[2] for p in ground_truth]
    gt_z = [p[3] for p in ground_truth]

    lidar_t = [p.timestamp for p in lidar_data]
    lidar_x = [p.x for p in lidar_data]
    lidar_y = [p.y for p in lidar_data]

    gps_t = [p.timestamp for p in gps_data]
    gps_x = [p.enu_east for p in gps_data]
    gps_y = [p.enu_north for p in gps_data]
    gps_rtk = [p.rtk_status == RTKStatus.RTK_FIX for p in gps_data]

    fused_t = [p.timestamp for p in fused_poses]
    fused_x = [p.x for p in fused_poses]
    fused_y = [p.y for p in fused_poses]

    # 1. XY Trajectory
    ax1 = fig.add_subplot(2, 3, 1)
    ax1.plot(gt_x, gt_y, 'g-', linewidth=2, label='Ground Truth', alpha=0.8)
    ax1.plot(lidar_x, lidar_y, 'b--', linewidth=1, label='LiDAR (with drift)', alpha=0.6)
    ax1.plot(fused_x, fused_y, 'r-', linewidth=1.5, label='Fused', alpha=0.8)

    # Mark RTK fix/unavailable regions
    rtk_fix_x = [gps_data[i].enu_east for i in range(len(gps_data)) if gps_rtk[i]]
    rtk_fix_y = [gps_data[i].enu_north for i in range(len(gps_data)) if gps_rtk[i]]
    ax1.scatter(rtk_fix_x[::10], rtk_fix_y[::10], c='green', s=10, alpha=0.3, label='GPS (RTK Fix)')

    ax1.set_xlabel('East [m]')
    ax1.set_ylabel('North [m]')
    ax1.set_title('XY Trajectory')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. X position over time
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(gt_t, gt_x, 'g-', linewidth=2, label='Ground Truth')
    ax2.plot(lidar_t, lidar_x, 'b--', linewidth=1, label='LiDAR', alpha=0.6)
    ax2.plot(fused_t, fused_x, 'r-', linewidth=1.5, label='Fused')

    # Shade RTK unavailable regions
    for i in range(len(gps_t)-1):
        if not gps_rtk[i]:
            ax2.axvspan(gps_t[i], gps_t[i+1], alpha=0.2, color='red')

    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('East [m]')
    ax2.set_title('X Position (red shade = no RTK)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Y position over time
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(gt_t, gt_y, 'g-', linewidth=2, label='Ground Truth')
    ax3.plot(lidar_t, lidar_y, 'b--', linewidth=1, label='LiDAR', alpha=0.6)
    ax3.plot(fused_t, fused_y, 'r-', linewidth=1.5, label='Fused')

    for i in range(len(gps_t)-1):
        if not gps_rtk[i]:
            ax3.axvspan(gps_t[i], gps_t[i+1], alpha=0.2, color='red')

    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('North [m]')
    ax3.set_title('Y Position')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Position Error
    ax4 = fig.add_subplot(2, 3, 4)
    if fusion.errors_history:
        err_t = [e['t'] for e in fusion.errors_history]
        err_dist = [e['error_dist'] for e in fusion.errors_history]
        ax4.plot(err_t, err_dist, 'purple', linewidth=1)
        ax4.fill_between(err_t, 0, err_dist, alpha=0.3, color='purple')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Error [m]')
    ax4.set_title('Position Error (GPS - Fused)')
    ax4.grid(True, alpha=0.3)

    # 5. Correction offsets over time
    ax5 = fig.add_subplot(2, 3, 5)
    if fusion.corrections_history:
        corr_t = [c['t'] for c in fusion.corrections_history]
        corr_x = [c['x_offset'] for c in fusion.corrections_history]
        corr_y = [c['y_offset'] for c in fusion.corrections_history]
        ax5.plot(corr_t, corr_x, 'r-', label='X offset')
        ax5.plot(corr_t, corr_y, 'b-', label='Y offset')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Offset [m]')
    ax5.set_title('Accumulated Correction Offsets')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 6. GPS usage status
    ax6 = fig.add_subplot(2, 3, 6)
    if fusion.gps_used_history:
        gps_used_t = [g['t'] for g in fusion.gps_used_history]
        gps_used = [1 if g['used'] else 0 for g in fusion.gps_used_history]
        ax6.fill_between(gps_used_t, 0, gps_used, alpha=0.5, color='green', step='post')
        ax6.set_ylim(-0.1, 1.1)
        ax6.set_yticks([0, 1])
        ax6.set_yticklabels(['Not Used', 'Used'])
    ax6.set_xlabel('Time [s]')
    ax6.set_title('GPS Used for Correction')
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save figure
    output_path = '/home/user/UM982-RTK-GPS-Library/lidar_gps_fusion/test/fusion_simulation_result.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nVisualization saved to: {output_path}")

    # Also display statistics
    print("\n" + "=" * 60)
    print("Simulation Statistics")
    print("=" * 60)

    if fusion.errors_history:
        errors = [e['error_dist'] for e in fusion.errors_history]
        print(f"Position Error (when GPS available):")
        print(f"  Mean: {np.mean(errors):.3f} m")
        print(f"  Max:  {np.max(errors):.3f} m")
        print(f"  Std:  {np.std(errors):.3f} m")

    # Calculate final drift comparison
    final_gt = ground_truth[-1]
    final_lidar = lidar_data[-1]
    final_fused = fused_poses[-1] if fused_poses else None

    lidar_drift = math.sqrt((final_lidar.x - final_gt[1])**2 + (final_lidar.y - final_gt[2])**2)
    print(f"\nFinal LiDAR drift from ground truth: {lidar_drift:.3f} m")

    if final_fused:
        fused_error = math.sqrt((final_fused.x - final_gt[1])**2 + (final_fused.y - final_gt[2])**2)
        print(f"Final Fused error from ground truth: {fused_error:.3f} m")
        print(f"Improvement: {(1 - fused_error/lidar_drift)*100:.1f}%")

    plt.show()


if __name__ == "__main__":
    run_simulation()
