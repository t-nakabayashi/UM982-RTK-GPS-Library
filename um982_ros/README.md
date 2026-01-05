# UM982 ROS2 Package

[English](#english) | [日本語](#japanese)

---

<a name="english"></a>
## English

ROS2 driver for UM982 dual-antenna RTK GNSS module. Publishes both standard ROS2 messages and custom messages with full UM982 data including dual-antenna heading/pitch.

### Features

- **Standard Messages**: `sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped`, `geometry_msgs/PoseStamped`
- **Custom Message**: `um982_ros/UM982Status` with full data including heading/pitch/ENU
- **ENU Coordinates**: WGS84 to ENU conversion with configurable origin
- **TF Broadcast**: Optional TF transform publishing
- **NTRIP Support**: Built-in NTRIP client for RTK corrections

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `gps/fix` | `sensor_msgs/NavSatFix` | Standard GPS fix message (WGS84) |
| `gps/vel` | `geometry_msgs/TwistStamped` | Velocity in ENU frame |
| `gps/pose` | `geometry_msgs/PoseStamped` | ENU position with heading (requires enu.enabled) |
| `gps/status` | `um982_ros/UM982Status` | Full status with heading/pitch/ENU |

### Installation

```bash
# Clone to your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/t-nakabayashi/UM982-RTK-GPS-Library.git

# Build
cd ~/ros2_ws
colcon build --packages-select um982_ros

# Source
source install/setup.bash
```

### Usage

#### Basic Usage

```bash
ros2 launch um982_ros um982.launch.py port:=/dev/ttyUSB0
```

#### With ENU Coordinates

```bash
ros2 run um982_ros um982_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p enu.enabled:=true \
  -p enu.origin_lat:=35.0 \
  -p enu.origin_lon:=139.0 \
  -p enu.origin_alt:=0.0
```

#### With ENU (First Fix as Origin)

```bash
ros2 run um982_ros um982_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p enu.enabled:=true \
  -p enu.use_first_fix:=true
```

#### With NTRIP (RTK Corrections)

```bash
ros2 launch um982_ros um982_ntrip.launch.py \
  port:=/dev/ttyUSB0 \
  ntrip_host:=rtk.example.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=RTCM3 \
  ntrip_user:=username \
  ntrip_password:=password
```

### Parameters

#### Basic Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial port |
| `baud` | int | `115200` | Baud rate |
| `output_rate` | int | `10` | Output rate (Hz) |
| `frame_id` | string | `gps` | Frame ID for messages |

#### ENU Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enu.enabled` | bool | `false` | Enable ENU output |
| `enu.origin_lat` | float | `0.0` | Origin latitude (degrees) |
| `enu.origin_lon` | float | `0.0` | Origin longitude (degrees) |
| `enu.origin_alt` | float | `0.0` | Origin altitude (meters) |
| `enu.use_first_fix` | bool | `false` | Use first valid fix as origin |

#### TF Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tf.publish_tf` | bool | `false` | Enable TF broadcast |
| `tf.parent_frame` | string | `map` | TF parent frame |
| `tf.child_frame` | string | `gps` | TF child frame |

#### NTRIP Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ntrip.enabled` | bool | `false` | Enable NTRIP |
| `ntrip.host` | string | `""` | NTRIP caster host |
| `ntrip.port` | int | `2101` | NTRIP caster port |
| `ntrip.mountpoint` | string | `""` | NTRIP mountpoint |
| `ntrip.user` | string | `""` | NTRIP username |
| `ntrip.password` | string | `""` | NTRIP password |
| `ntrip.gga_interval` | float | `5.0` | GGA send interval (s) |

### Custom Message: UM982Status

```
# Position (WGS84)
float64 latitude
float64 longitude
float64 altitude

# ENU Position (meters, relative to origin)
bool enu_valid
float64 enu_east
float64 enu_north
float64 enu_up

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

### Coordinate Conversion

WGS84 to ENU conversion uses ECEF (Earth-Centered, Earth-Fixed) as intermediate:

```
WGS84 (lat, lon, alt) → ECEF (X, Y, Z) → ENU (East, North, Up)
```

The conversion is based on WGS84 ellipsoid parameters and provides cm-level accuracy.

### RTK Status Values

| Value | Constant | Description | Typical Accuracy |
|-------|----------|-------------|------------------|
| 0 | `STATUS_UNKNOWN` | Unknown | - |
| 1 | `STATUS_STANDALONE` | Single point | ~10m |
| 2 | `STATUS_DGPS` | Differential GPS | ~1m |
| 4 | `STATUS_RTK_FIX` | RTK Fixed | ~1cm |
| 5 | `STATUS_RTK_FLOAT` | RTK Float | ~10cm |

### Example: Using ENU Position

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
        if msg.enu_valid:
            self.get_logger().info(
                f'ENU: E={msg.enu_east:.3f}m, N={msg.enu_north:.3f}m, U={msg.enu_up:.3f}m'
            )

def main():
    rclpy.init()
    node = GPSSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

<a name="japanese"></a>
## 日本語

UM982デュアルアンテナRTK GNSSモジュール用のROS2ドライバー。標準ROS2メッセージとカスタムメッセージ（方位/ピッチ/ENU座標含む）を配信します。

### 特徴

- **標準メッセージ**: `sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped`, `geometry_msgs/PoseStamped`
- **カスタムメッセージ**: `um982_ros/UM982Status`（方位/ピッチ/ENU座標含む）
- **ENU座標**: WGS84からENU座標への変換（原点設定可能）
- **TFブロードキャスト**: オプションでTF変換を配信
- **NTRIP対応**: RTK補正データ受信用NTRIPクライアント内蔵

### 配信トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `gps/fix` | `sensor_msgs/NavSatFix` | 標準GPS位置メッセージ（WGS84） |
| `gps/vel` | `geometry_msgs/TwistStamped` | ENU座標系での速度 |
| `gps/pose` | `geometry_msgs/PoseStamped` | ENU位置と方位（enu.enabled時） |
| `gps/status` | `um982_ros/UM982Status` | 方位/ピッチ/ENU含む全データ |

### インストール

```bash
# ROS2ワークスペースにクローン
cd ~/ros2_ws/src
git clone https://github.com/t-nakabayashi/UM982-RTK-GPS-Library.git

# ビルド
cd ~/ros2_ws
colcon build --packages-select um982_ros

# 環境設定
source install/setup.bash
```

### 使用方法

#### 基本的な使用方法

```bash
ros2 launch um982_ros um982.launch.py port:=/dev/ttyUSB0
```

#### ENU座標を使用

```bash
ros2 run um982_ros um982_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p enu.enabled:=true \
  -p enu.origin_lat:=35.0 \
  -p enu.origin_lon:=139.0 \
  -p enu.origin_alt:=0.0
```

#### ENU座標（最初の測位を原点に）

```bash
ros2 run um982_ros um982_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p enu.enabled:=true \
  -p enu.use_first_fix:=true
```

#### NTRIP使用（RTK補正）

```bash
ros2 launch um982_ros um982_ntrip.launch.py \
  port:=/dev/ttyUSB0 \
  ntrip_host:=rtk.example.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=RTCM3 \
  ntrip_user:=username \
  ntrip_password:=password
```

### パラメータ

#### 基本パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `port` | string | `/dev/ttyUSB0` | シリアルポート |
| `baud` | int | `115200` | ボーレート |
| `output_rate` | int | `10` | 出力レート（Hz） |
| `frame_id` | string | `gps` | メッセージのフレームID |

#### ENUパラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `enu.enabled` | bool | `false` | ENU出力を有効化 |
| `enu.origin_lat` | float | `0.0` | 原点緯度（度） |
| `enu.origin_lon` | float | `0.0` | 原点経度（度） |
| `enu.origin_alt` | float | `0.0` | 原点高度（メートル） |
| `enu.use_first_fix` | bool | `false` | 最初の測位を原点に使用 |

#### TFパラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `tf.publish_tf` | bool | `false` | TFブロードキャストを有効化 |
| `tf.parent_frame` | string | `map` | TF親フレーム |
| `tf.child_frame` | string | `gps` | TF子フレーム |

#### NTRIPパラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `ntrip.enabled` | bool | `false` | NTRIP有効化 |
| `ntrip.host` | string | `""` | NTRIPキャスターホスト |
| `ntrip.port` | int | `2101` | NTRIPキャスターポート |
| `ntrip.mountpoint` | string | `""` | NTRIPマウントポイント |
| `ntrip.user` | string | `""` | NTRIPユーザー名 |
| `ntrip.password` | string | `""` | NTRIPパスワード |
| `ntrip.gga_interval` | float | `5.0` | GGA送信間隔（秒） |

### カスタムメッセージ: UM982Status

```
# 位置（WGS84）
float64 latitude          # 緯度（10進数度）
float64 longitude         # 経度（10進数度）
float64 altitude          # 高度（メートル）

# ENU位置（メートル、原点からの相対位置）
bool enu_valid            # ENU座標が有効か
float64 enu_east          # 東方向（メートル）
float64 enu_north         # 北方向（メートル）
float64 enu_up            # 上方向（メートル）

# RTK状態
uint8 rtk_status          # 0=不明, 1=単独, 2=DGPS, 4=RTK Fix, 5=RTK Float
string rtk_status_string  # RTK状態（文字列）
uint8 num_satellites      # 使用衛星数
float32 hdop              # 水平精度低下率
float32 diff_age          # 補正データ経過時間（秒）

# デュアルアンテナ方位
bool heading_valid        # 方位データが有効か
float32 heading           # 方位（度、真北基準）
float32 pitch             # ピッチ角（度）
float32 heading_stddev    # 方位標準偏差
float32 pitch_stddev      # ピッチ標準偏差
float32 baseline          # ベースライン長（メートル）

# 速度
bool velocity_valid       # 速度データが有効か
float32 speed_mps         # 対地速度（m/s）
float32 speed_knots       # 対地速度（ノット）
float32 course            # 進行方位（度）

# 統計情報
uint32 rtcm_bytes_received  # 受信RTCMバイト数
```

### 座標変換

WGS84からENU座標への変換はECEF（地心地固座標系）を中間座標として使用：

```
WGS84 (lat, lon, alt) → ECEF (X, Y, Z) → ENU (East, North, Up)
```

WGS84楕円体パラメータに基づき、cm級の精度で変換します。

### RTK状態値

| 値 | 定数 | 説明 | 代表的な精度 |
|----|------|------|-------------|
| 0 | `STATUS_UNKNOWN` | 不明 | - |
| 1 | `STATUS_STANDALONE` | 単独測位 | 約10m |
| 2 | `STATUS_DGPS` | DGPS | 約1m |
| 4 | `STATUS_RTK_FIX` | RTK Fix | 約1cm |
| 5 | `STATUS_RTK_FLOAT` | RTK Float | 約10cm |

### 使用例: ENU位置を取得

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
        if msg.enu_valid:
            self.get_logger().info(
                f'ENU: E={msg.enu_east:.3f}m, N={msg.enu_north:.3f}m, U={msg.enu_up:.3f}m'
            )

def main():
    rclpy.init()
    node = GPSSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### ライセンス

MIT License
