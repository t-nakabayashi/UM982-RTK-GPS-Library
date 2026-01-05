# UM982 ROS2 Package

ROS2 driver for UM982 dual-antenna RTK GNSS module.

## Features

- Standard ROS2 messages (`sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped`)
- Custom message with full UM982 data including dual-antenna heading/pitch
- NTRIP client support for RTK corrections
- Configurable output rate and serial parameters

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `gps/fix` | `sensor_msgs/NavSatFix` | Standard GPS fix message |
| `gps/vel` | `geometry_msgs/TwistStamped` | Velocity in ENU frame |
| `gps/status` | `um982_ros/UM982Status` | Full status with heading/pitch |

## Installation

```bash
# Clone to your ROS2 workspace
cd ~/ros2_ws/src
git clone <repository_url>

# Build
cd ~/ros2_ws
colcon build --packages-select um982_ros

# Source
source install/setup.bash
```

## Usage

### Basic Usage

```bash
ros2 launch um982_ros um982.launch.py port:=/dev/ttyUSB0
```

### With NTRIP (RTK Corrections)

```bash
ros2 launch um982_ros um982_ntrip.launch.py \
  port:=/dev/ttyUSB0 \
  ntrip_host:=rtk.example.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=RTCM3 \
  ntrip_user:=username \
  ntrip_password:=password
```

### Using Parameter File

```bash
ros2 launch um982_ros um982.launch.py params_file:=/path/to/params.yaml
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial port |
| `baud` | int | `115200` | Baud rate |
| `output_rate` | int | `10` | Output rate (Hz) |
| `frame_id` | string | `gps` | TF frame ID |
| `ntrip.enabled` | bool | `false` | Enable NTRIP |
| `ntrip.host` | string | `""` | NTRIP caster host |
| `ntrip.port` | int | `2101` | NTRIP caster port |
| `ntrip.mountpoint` | string | `""` | NTRIP mountpoint |
| `ntrip.user` | string | `""` | NTRIP username |
| `ntrip.password` | string | `""` | NTRIP password |
| `ntrip.gga_interval` | float | `5.0` | GGA send interval (s) |

## Custom Message: UM982Status

The `um982_ros/UM982Status` message contains all data from the UM982 module:

```
# Position
float64 latitude
float64 longitude
float64 altitude

# RTK Status
uint8 rtk_status          # 0=unknown, 1=standalone, 2=dgps, 4=rtk_fix, 5=rtk_float
string rtk_status_string
uint8 num_satellites
float32 hdop
float32 diff_age

# Dual Antenna Heading
bool heading_valid
float32 heading           # degrees (true north)
float32 pitch             # degrees
float32 heading_stddev
float32 pitch_stddev
float32 baseline          # meters

# Velocity
bool velocity_valid
float32 speed_mps
float32 speed_knots
float32 course            # degrees

# Statistics
uint32 rtcm_bytes_received
```

## RTK Status Values

| Value | Constant | Description | Typical Accuracy |
|-------|----------|-------------|------------------|
| 0 | `STATUS_UNKNOWN` | Unknown | - |
| 1 | `STATUS_STANDALONE` | Single point | ~10m |
| 2 | `STATUS_DGPS` | Differential GPS | ~1m |
| 4 | `STATUS_RTK_FIX` | RTK Fixed | ~1cm |
| 5 | `STATUS_RTK_FLOAT` | RTK Float | ~10cm |

## Example Subscriber

```python
import rclpy
from rclpy.node import Node
from um982_ros.msg import UM982Status

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            UM982Status,
            'gps/status',
            self.callback,
            10)

    def callback(self, msg):
        if msg.rtk_status == UM982Status.STATUS_RTK_FIX:
            self.get_logger().info(
                f'RTK Fix: {msg.latitude:.8f}, {msg.longitude:.8f}, '
                f'Heading: {msg.heading:.1f} deg'
            )

def main():
    rclpy.init()
    node = GPSSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## License

MIT
