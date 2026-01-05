# UM982 ROS2 Package

[English](#english) | [日本語](#japanese)

---

<a name="english"></a>
## English

ROS2 driver for UM982 dual-antenna RTK GNSS module. Publishes both standard ROS2 messages and custom messages with full UM982 data including dual-antenna heading/pitch.

### Features

- **Standard Messages**: `sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped`
- **Custom Message**: `um982_ros/UM982Status` with full data including heading/pitch
- **NTRIP Support**: Built-in NTRIP client for RTK corrections
- **Configurable**: Output rate, serial parameters via ROS2 parameters

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `gps/fix` | `sensor_msgs/NavSatFix` | Standard GPS fix message |
| `gps/vel` | `geometry_msgs/TwistStamped` | Velocity in ENU frame |
| `gps/status` | `um982_ros/UM982Status` | Full status with heading/pitch |

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

#### Using Parameter File

```bash
ros2 launch um982_ros um982.launch.py params_file:=/path/to/params.yaml
```

### Parameters

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

### Custom Message: UM982Status

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

### RTK Status Values

| Value | Constant | Description | Typical Accuracy |
|-------|----------|-------------|------------------|
| 0 | `STATUS_UNKNOWN` | Unknown | - |
| 1 | `STATUS_STANDALONE` | Single point | ~10m |
| 2 | `STATUS_DGPS` | Differential GPS | ~1m |
| 4 | `STATUS_RTK_FIX` | RTK Fixed | ~1cm |
| 5 | `STATUS_RTK_FLOAT` | RTK Float | ~10cm |

### Example Subscriber

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

---

<a name="japanese"></a>
## 日本語

UM982デュアルアンテナRTK GNSSモジュール用のROS2ドライバー。標準ROS2メッセージとカスタムメッセージ（方位/ピッチ含む）の両方を配信します。

### 特徴

- **標準メッセージ**: `sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped`
- **カスタムメッセージ**: `um982_ros/UM982Status`（方位/ピッチ等全データ含む）
- **NTRIP対応**: RTK補正データ受信用NTRIPクライアント内蔵
- **設定可能**: 出力レート、シリアル設定をROS2パラメータで設定

### 配信トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `gps/fix` | `sensor_msgs/NavSatFix` | 標準GPS位置メッセージ |
| `gps/vel` | `geometry_msgs/TwistStamped` | ENU座標系での速度 |
| `gps/status` | `um982_ros/UM982Status` | 方位/ピッチ含む全データ |

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

#### パラメータファイル使用

```bash
ros2 launch um982_ros um982.launch.py params_file:=/path/to/params.yaml
```

### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `port` | string | `/dev/ttyUSB0` | シリアルポート |
| `baud` | int | `115200` | ボーレート |
| `output_rate` | int | `10` | 出力レート（Hz） |
| `frame_id` | string | `gps` | TFフレームID |
| `ntrip.enabled` | bool | `false` | NTRIP有効化 |
| `ntrip.host` | string | `""` | NTRIPキャスターホスト |
| `ntrip.port` | int | `2101` | NTRIPキャスターポート |
| `ntrip.mountpoint` | string | `""` | NTRIPマウントポイント |
| `ntrip.user` | string | `""` | NTRIPユーザー名 |
| `ntrip.password` | string | `""` | NTRIPパスワード |
| `ntrip.gga_interval` | float | `5.0` | GGA送信間隔（秒） |

### カスタムメッセージ: UM982Status

`um982_ros/UM982Status`メッセージにはUM982モジュールの全データが含まれます：

```
# 位置
float64 latitude          # 緯度（10進数度）
float64 longitude         # 経度（10進数度）
float64 altitude          # 高度（メートル）

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

### RTK状態値

| 値 | 定数 | 説明 | 代表的な精度 |
|----|------|------|-------------|
| 0 | `STATUS_UNKNOWN` | 不明 | - |
| 1 | `STATUS_STANDALONE` | 単独測位 | 約10m |
| 2 | `STATUS_DGPS` | DGPS | 約1m |
| 4 | `STATUS_RTK_FIX` | RTK Fix | 約1cm |
| 5 | `STATUS_RTK_FLOAT` | RTK Float | 約10cm |

### サブスクライバーの例

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
                f'方位: {msg.heading:.1f}度'
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
