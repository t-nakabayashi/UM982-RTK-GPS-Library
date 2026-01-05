# LiDAR-GPS Fusion Node

[English](#english) | [日本語](#japanese)

---

<a name="english"></a>
## English

ROS2 node that fuses LiDAR odometry (e.g., FastLIO) with UM982 RTK-GPS for drift-corrected ENU positioning.

### Features

- **LiDAR-based positioning**: High-frequency output (10Hz) based on LiDAR odometry
- **GPS drift correction**: Corrects accumulated LiDAR drift using RTK-GPS
- **Smooth output**: Gradual correction prevents position jumps
- **RTK-only correction**: Only uses GPS when RTK Fix is available
- **TF broadcast**: Optional map→odom transform publishing

### How It Works

```
       map (ENU, absolute position)
        │
        │  ← T_map_odom: Computed by this node
        │     (Set at initialization, gradually corrected with RTK)
        ↓
       odom (FastLIO origin)
        │
        │  ← Published by FastLIO
        ↓
     base_link (Robot frame)
```

1. **Initialization**: Waits for first RTK Fix, then aligns LiDAR frame with GPS ENU frame
2. **Normal operation**: Transforms LiDAR odom to map frame, publishes fused odometry
3. **Drift correction**: When RTK Fix available, gradually adjusts transformation to correct drift

### Topics

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` (configurable) | `nav_msgs/Odometry` | LiDAR odometry |
| `/gps/status` (configurable) | `um982_ros/UM982Status` | GPS status |

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/fused_odom` (configurable) | `nav_msgs/Odometry` | Fused odometry (ENU) |
| `/fusion/status` | `lidar_gps_fusion/FusionStatus` | Diagnostics |
| TF: `map` → `odom` | - | Transform (optional) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_odom_topic` | string | `/odom` | LiDAR odometry topic |
| `gps_status_topic` | string | `/gps/status` | GPS status topic |
| `output_odom_topic` | string | `/fused_odom` | Output topic |
| `publish_tf` | bool | `true` | Publish TF |
| `map_frame` | string | `map` | Map frame name |
| `odom_frame` | string | `odom` | Odom frame name |
| `gps_use_condition` | string | `rtk_fix` | `rtk_fix` or `hdop` |
| `hdop_threshold` | float | `1.0` | HDOP threshold |
| `correction_rate` | float | `0.05` | Correction speed (0.0-1.0) |
| `correct_xy` | bool | `true` | Correct XY |
| `correct_z` | bool | `true` | Correct Z |
| `correct_yaw` | bool | `true` | Correct yaw |
| `init_timeout` | float | `30.0` | Init timeout (seconds) |

### Correction Rate Guide

| Rate | Behavior | Use Case |
|------|----------|----------|
| 0.01 | Very smooth, slow convergence | High-precision LiDAR |
| 0.05 | Balanced (recommended) | General use |
| 0.1 | Faster convergence | High-drift environment |
| 0.5 | Fast, slightly discontinuous | Testing |

### Usage

```bash
# Basic usage
ros2 launch lidar_gps_fusion fusion.launch.py

# With custom topics
ros2 launch lidar_gps_fusion fusion.launch.py \
    lidar_odom_topic:=/fastlio/odom \
    gps_status_topic:=/gps/status \
    correction_rate:=0.05
```

### Dependencies

- um982_ros (for UM982Status message)
- nav_msgs
- geometry_msgs
- tf2_ros

---

<a name="japanese"></a>
## 日本語

LiDARオドメトリ（FastLIO等）とUM982 RTK-GPSを融合し、ドリフト補正されたENU位置を出力するROS2ノード。

### 特徴

- **LiDARベース測位**: LiDARオドメトリに基づく高頻度出力（10Hz）
- **GPSドリフト補正**: RTK-GPSを使用してLiDARの累積ドリフトを補正
- **滑らかな出力**: 緩やかな補正により位置ジャンプを防止
- **RTK時のみ補正**: RTK Fix時のみGPSを使用
- **TFブロードキャスト**: オプションでmap→odom変換を配信

### 動作原理

```
       map (ENU座標系、絶対位置)
        │
        │  ← T_map_odom: このノードが計算
        │     (初期化時に設定、RTK時に徐々に補正)
        ↓
       odom (FastLIO原点)
        │
        │  ← FastLIOが配信
        ↓
     base_link (ロボット座標系)
```

1. **初期化**: 最初のRTK Fixを待ち、LiDARフレームをGPS ENUフレームに合わせる
2. **通常動作**: LiDAR odomをmapフレームに変換し、融合オドメトリを配信
3. **ドリフト補正**: RTK Fix時に変換を徐々に調整してドリフトを補正

### トピック

#### 購読

| トピック | 型 | 説明 |
|---------|-----|------|
| `/odom` (設定可) | `nav_msgs/Odometry` | LiDARオドメトリ |
| `/gps/status` (設定可) | `um982_ros/UM982Status` | GPS状態 |

#### 配信

| トピック | 型 | 説明 |
|---------|-----|------|
| `/fused_odom` (設定可) | `nav_msgs/Odometry` | 融合オドメトリ（ENU） |
| `/fusion/status` | `lidar_gps_fusion/FusionStatus` | 診断情報 |
| TF: `map` → `odom` | - | 変換（オプション） |

### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|---------|------|
| `lidar_odom_topic` | string | `/odom` | LiDARオドメトリトピック |
| `gps_status_topic` | string | `/gps/status` | GPSステータストピック |
| `output_odom_topic` | string | `/fused_odom` | 出力トピック |
| `publish_tf` | bool | `true` | TF配信 |
| `map_frame` | string | `map` | Mapフレーム名 |
| `odom_frame` | string | `odom` | Odomフレーム名 |
| `gps_use_condition` | string | `rtk_fix` | `rtk_fix` または `hdop` |
| `hdop_threshold` | float | `1.0` | HDOP閾値 |
| `correction_rate` | float | `0.05` | 補正速度 (0.0-1.0) |
| `correct_xy` | bool | `true` | XY補正 |
| `correct_z` | bool | `true` | Z補正 |
| `correct_yaw` | bool | `true` | Yaw補正 |
| `init_timeout` | float | `30.0` | 初期化タイムアウト（秒） |

### 補正レート目安

| レート | 動作 | 用途 |
|--------|------|------|
| 0.01 | 非常に滑らか、収束遅い | 高精度LiDAR |
| 0.05 | バランス型（推奨） | 一般用途 |
| 0.1 | やや速い収束 | ドリフト大きい環境 |
| 0.5 | 速い収束、やや不連続 | テスト用 |

### 使用方法

```bash
# 基本的な使用方法
ros2 launch lidar_gps_fusion fusion.launch.py

# カスタムトピック指定
ros2 launch lidar_gps_fusion fusion.launch.py \
    lidar_odom_topic:=/fastlio/odom \
    gps_status_topic:=/gps/status \
    correction_rate:=0.05
```

### 依存パッケージ

- um982_ros (UM982Statusメッセージ用)
- nav_msgs
- geometry_msgs
- tf2_ros

### ライセンス

MIT License
