# UM982 RTK GPS Library

[English](#english) | [日本語](#japanese)

---

<a name="english"></a>
## English

Python library for Unicore UM982 dual-antenna GNSS module. Provides high-precision RTK positioning and heading measurement via NTRIP RTCM correction data.

### Features

- **RTK Positioning**: Supports RTK Fix (cm-level) / RTK Float (dm-level) accuracy
- **Heading Measurement**: High-precision heading from dual-antenna configuration
- **NTRIP Support**: Receives RTCM3.x correction data from base stations or VRS
- **Easy-to-use Library**: Simple Python API for position and heading data
- **JSON Lines Output**: Ideal for logging and data analysis pipelines
- **Lightweight**: Only requires Python standard library + pyserial

### Supported Modules

- **Unicore UM982** (dual-antenna, recommended)
- Other NMEA 0183 compatible GNSS modules (with GGA/RMC output)

### Installation

```bash
git clone https://github.com/t-nakabayashi/UM982-RTK-GPS-Library.git
cd UM982-RTK-GPS-Library
pip install -r requirements.txt
```

### Directory Structure

```
.
├── um982/                      # Library package
│   ├── __init__.py
│   ├── client.py               # UM982Client class
│   ├── nmea.py                 # NMEA/Unicore parser
│   ├── ntrip.py                # NTRIP client
│   └── types.py                # Data type definitions
├── examples/                   # Example programs
│   ├── simple_position.py      # Simple position retrieval
│   ├── callback_example.py     # Callback usage example
│   └── log_position.py         # Position logging
├── tools/                      # Utilities
│   └── log_and_analyze.py      # Log analysis tool
├── requirements.txt
└── README.md
```

### Library Usage

#### Basic Usage

```python
from um982 import UM982Client
import time

# Create and start client (default: 10Hz output)
client = UM982Client("/dev/ttyUSB0", baud=115200)
client.start()

# Position data loop
while True:
    pos = client.get_position()
    if pos and pos.is_valid:
        print(f"Lat: {pos.lat:.8f}")
        print(f"Lon: {pos.lon:.8f}")
        print(f"Alt: {pos.alt:.2f}m")
        print(f"Heading: {pos.heading:.2f}°")  # Dual-antenna
        print(f"RTK State: {pos.rtk_state}")
        print(f"Satellites: {pos.num_sats}")
    time.sleep(0.1)

client.stop()
```

#### Changing Output Rate

```python
# Set output rate at initialization (20Hz)
client = UM982Client("/dev/ttyUSB0", output_rate=20)
client.start()

# Or change rate after start
client.set_output_rate(5)  # Change to 5Hz
```

#### RTK Positioning with NTRIP

```python
from um982 import UM982Client
import time

client = UM982Client("/dev/ttyUSB0")
client.start()

# Start NTRIP client for RTK corrections
client.start_ntrip(
    host="rtk.example.com",
    port=2101,
    mountpoint="RTCM3",
    user="guest",
    password="guest",
)

while True:
    pos = client.get_position()
    if pos and pos.is_rtk_fix:  # Process only RTK Fix
        print(f"[RTK FIX] Lat: {pos.lat:.8f}, Lon: {pos.lon:.8f}")
    time.sleep(0.1)

client.stop()
```

#### Using Callbacks

```python
from um982 import UM982Client, PositionData

def on_position(pos: PositionData):
    """Called on position update"""
    if pos.is_valid:
        print(f"Lat: {pos.lat:.8f}, Lon: {pos.lon:.8f}, Heading: {pos.heading}")

client = UM982Client("/dev/ttyUSB0")
client.set_position_callback(on_position)
client.start()

# Main thread can do other work
import time
while True:
    time.sleep(1)
```

### PositionData Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `lat` | float | Latitude (decimal degrees) |
| `lon` | float | Longitude (decimal degrees) |
| `alt` | float | Altitude (meters) |
| `heading` | float | Heading (degrees, dual-antenna) |
| `pitch` | float | Pitch angle (degrees) |
| `speed_knots` | float | Ground speed (knots) |
| `speed_mps` | float | Ground speed (m/s) - property |
| `speed_kmh` | float | Ground speed (km/h) - property |
| `course` | float | Course over ground (degrees) |
| `rtk_state` | str | RTK state |
| `num_sats` | int | Number of satellites |
| `hdop` | float | HDOP |
| `baseline_m` | float | Baseline length (meters) |
| `timestamp` | float | Timestamp |
| `diff_age` | float | Correction data age (seconds) |
| `heading_stddev` | float | Heading standard deviation (degrees) |
| `pitch_stddev` | float | Pitch standard deviation (degrees) |
| `is_valid` | bool | Valid position data - property |
| `is_rtk_fix` | bool | RTK Fix status - property |

### Example Programs

#### Simple Position

```bash
# Basic (without NTRIP)
python examples/simple_position.py --port /dev/ttyUSB0

# With NTRIP
python examples/simple_position.py --port /dev/ttyUSB0 \
    --ntrip-host rtk.example.com --ntrip-port 2101 \
    --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass
```

#### Logging

```bash
# Log for 1 hour
python examples/log_position.py --port /dev/ttyUSB0 --duration 3600

# Log for 10 minutes with NTRIP and console output
python examples/log_position.py --port /dev/ttyUSB0 --duration 600 --pretty \
    --ntrip-host rtk.example.com --ntrip-port 2101 \
    --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass

# Auto-analyze after logging
python examples/log_position.py --port /dev/ttyUSB0 --duration 3600 --analyze
```

#### Log Analysis

```bash
# Analyze log file
python tools/log_and_analyze.py rtk_log.jsonl

# Analyze with graph
python tools/log_and_analyze.py rtk_log.jsonl --plot

# Save graph as image
python tools/log_and_analyze.py rtk_log.jsonl --save-plot analysis.png
```

### Output Format

#### JSON Lines (default)

```json
{
  "timestamp": 1704067200.123,
  "rtk_state": "rtk_fix",
  "lat": 35.XXXXXXXX,
  "lon": 139.XXXXXXXX,
  "alt_m": 40.5,
  "quality": 4,
  "num_sats": 28,
  "hdop": 0.7,
  "diff_age_s": 0.5,
  "ref_station_id": "0",
  "rtcm_bytes": 12345,
  "heading_deg": 45.2,
  "pitch_deg": 1.5,
  "baseline_m": 0.612
}
```

#### Key Fields

| Field | Description |
|-------|-------------|
| `rtk_state` | RTK state: `rtk_fix`, `rtk_float`, `dgps`, `standalone`, `unknown` |
| `lat`, `lon` | Latitude/Longitude (decimal degrees) |
| `alt_m` | Altitude (meters) |
| `quality` | GPS quality (1=Standalone, 2=DGPS, 4=RTK Fix, 5=RTK Float) |
| `num_sats` | Number of satellites |
| `hdop` | Horizontal dilution of precision |
| `heading_deg` | Heading (degrees, true north, dual-antenna) |
| `baseline_m` | Baseline length (meters) |
| `rtcm_bytes` | Cumulative RTCM bytes received |

### Hardware Connection

```
[PC/Raspberry Pi]          [UM982 Module]
USB ─────────────────────── UART (COM1/COM2/COM3)
                            │
                            ├── ANT1 (Master antenna)
                            └── ANT2 (Slave antenna)
```

#### Recommended Baud Rates

| Baud Rate | Use Case |
|-----------|----------|
| 115200 | General use (default) |
| 460800 | High-rate output |
| 921600 | Maximum speed |

### Troubleshooting

#### Cannot open serial port

```bash
# Check permissions
ls -la /dev/ttyUSB0

# Add to dialout group (Ubuntu)
sudo usermod -a -G dialout $USER
# Logout and login required
```

#### Garbled data

Ensure baud rate matches the module configuration.

```bash
# Auto-detect baud rate
for baud in 115200 230400 460800 921600; do
  echo "Testing $baud..."
  timeout 2 python examples/simple_position.py --port /dev/ttyUSB0 --baud $baud
done
```

#### NTRIP connection fails

- Verify hostname, port, and mountpoint
- Check username and password
- Check firewall settings
- Confirm base station is operational

### License

MIT License - See [LICENSE](LICENSE) for details.

### References

- [UM982 User Manual](https://www.unicorecomm.com/products/um982)
- [NTRIP Protocol](https://igs.bkg.bund.de/ntrip/about)
- [NMEA 0183](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)

---

<a name="japanese"></a>
## 日本語

Unicore UM982デュアルアンテナGNSSモジュール用のPythonライブラリ。NTRIPを通じてRTCM補正データを取得し、高精度RTK測位とマルチアンテナによる方位取得を行います。

### 特徴

- **RTK測位**: RTK Fix（cm級精度）/ RTK Float（dm級精度）に対応
- **方位取得**: デュアルアンテナによる高精度方位（heading）測定
- **NTRIP対応**: 基準局やVRS等からRTCM3.x補正データを受信
- **ライブラリ提供**: Pythonから簡単に位置・方位を取得
- **JSON Lines出力**: パイプライン処理やログ解析に最適な出力形式
- **軽量**: Python標準ライブラリ + pyserialのみで動作

### 対応モジュール

- **Unicore UM982**（デュアルアンテナ対応、推奨）
- その他NMEA 0183互換のGNSSモジュール（GGA/RMC出力対応のもの）

### インストール

```bash
git clone https://github.com/t-nakabayashi/UM982-RTK-GPS-Library.git
cd UM982-RTK-GPS-Library
pip install -r requirements.txt
```

### ディレクトリ構成

```
.
├── um982/                      # ライブラリパッケージ
│   ├── __init__.py
│   ├── client.py               # UM982Clientクラス
│   ├── nmea.py                 # NMEA/Unicoreパーサー
│   ├── ntrip.py                # NTRIPクライアント
│   └── types.py                # データ型定義
├── examples/                   # サンプルプログラム
│   ├── simple_position.py      # シンプルな位置取得
│   ├── callback_example.py     # コールバック使用例
│   └── log_position.py         # ログ記録（分析ツール互換）
├── tools/                      # ユーティリティ
│   └── log_and_analyze.py      # ログ分析ツール
├── requirements.txt
└── README.md
```

### ライブラリとしての使い方

#### 基本的な使い方

```python
from um982 import UM982Client
import time

# クライアント作成・開始（デフォルト: 10Hz出力）
client = UM982Client("/dev/ttyUSB0", baud=115200)
client.start()

# 位置データ取得ループ
while True:
    pos = client.get_position()
    if pos and pos.is_valid:
        print(f"緯度: {pos.lat:.8f}")
        print(f"経度: {pos.lon:.8f}")
        print(f"高度: {pos.alt:.2f}m")
        print(f"方位: {pos.heading:.2f}°")  # デュアルアンテナ時
        print(f"RTK状態: {pos.rtk_state}")
        print(f"衛星数: {pos.num_sats}")
    time.sleep(0.1)

client.stop()
```

#### 出力レートの変更

```python
# 初期化時に出力レートを指定（20Hz）
client = UM982Client("/dev/ttyUSB0", output_rate=20)
client.start()

# または開始後に変更
client.set_output_rate(5)  # 5Hzに変更
```

#### NTRIP接続でRTK測位

```python
from um982 import UM982Client
import time

client = UM982Client("/dev/ttyUSB0")
client.start()

# NTRIPクライアントを開始（RTK補正データ受信）
client.start_ntrip(
    host="rtk.example.com",
    port=2101,
    mountpoint="RTCM3",
    user="guest",
    password="guest",
)

while True:
    pos = client.get_position()
    if pos and pos.is_rtk_fix:  # RTK Fixの時だけ処理
        print(f"[RTK FIX] Lat: {pos.lat:.8f}, Lon: {pos.lon:.8f}")
    time.sleep(0.1)

client.stop()
```

#### コールバックを使用

```python
from um982 import UM982Client, PositionData

def on_position(pos: PositionData):
    """位置更新時に呼ばれる"""
    if pos.is_valid:
        print(f"Lat: {pos.lat:.8f}, Lon: {pos.lon:.8f}, Heading: {pos.heading}")

client = UM982Client("/dev/ttyUSB0")
client.set_position_callback(on_position)
client.start()

# メインスレッドは他の処理を実行可能
import time
while True:
    time.sleep(1)
```

### PositionDataの属性

| 属性 | 型 | 説明 |
|-----|-----|------|
| `lat` | float | 緯度（10進数度） |
| `lon` | float | 経度（10進数度） |
| `alt` | float | 高度（メートル） |
| `heading` | float | 方位（度、デュアルアンテナ） |
| `pitch` | float | ピッチ角（度） |
| `speed_knots` | float | 対地速度（ノット） |
| `speed_mps` | float | 対地速度（m/s）- プロパティ |
| `speed_kmh` | float | 対地速度（km/h）- プロパティ |
| `course` | float | 進行方位（度） |
| `rtk_state` | str | RTK状態 |
| `num_sats` | int | 使用衛星数 |
| `hdop` | float | HDOP |
| `baseline_m` | float | ベースライン長（メートル） |
| `timestamp` | float | タイムスタンプ |
| `diff_age` | float | 補正データ経過時間（秒） |
| `heading_stddev` | float | 方位標準偏差（度） |
| `pitch_stddev` | float | ピッチ標準偏差（度） |
| `is_valid` | bool | 有効な位置データか - プロパティ |
| `is_rtk_fix` | bool | RTK Fixか - プロパティ |

### サンプルプログラム

#### シンプルな位置取得

```bash
# 基本（NTRIPなし）
python examples/simple_position.py --port /dev/ttyUSB0

# NTRIP使用
python examples/simple_position.py --port /dev/ttyUSB0 \
    --ntrip-host rtk.example.com --ntrip-port 2101 \
    --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass
```

#### ログ記録

```bash
# 1時間のログを取得
python examples/log_position.py --port /dev/ttyUSB0 --duration 3600

# NTRIP使用で10分間ログ（コンソール表示付き）
python examples/log_position.py --port /dev/ttyUSB0 --duration 600 --pretty \
    --ntrip-host rtk.example.com --ntrip-port 2101 \
    --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass

# ログ取得後に自動分析
python examples/log_position.py --port /dev/ttyUSB0 --duration 3600 --analyze
```

#### ログ分析

```bash
# 既存ログを分析
python tools/log_and_analyze.py rtk_log.jsonl

# グラフ付きで分析
python tools/log_and_analyze.py rtk_log.jsonl --plot

# グラフを画像として保存
python tools/log_and_analyze.py rtk_log.jsonl --save-plot analysis.png
```

### 出力フォーマット

#### JSON Lines形式（デフォルト）

```json
{
  "timestamp": 1704067200.123,
  "rtk_state": "rtk_fix",
  "lat": 35.XXXXXXXX,
  "lon": 139.XXXXXXXX,
  "alt_m": 40.5,
  "quality": 4,
  "num_sats": 28,
  "hdop": 0.7,
  "diff_age_s": 0.5,
  "ref_station_id": "0",
  "rtcm_bytes": 12345,
  "heading_deg": 45.2,
  "pitch_deg": 1.5,
  "baseline_m": 0.612
}
```

#### 主要フィールド一覧

| フィールド | 説明 |
|-----------|------|
| `rtk_state` | RTK状態: `rtk_fix`, `rtk_float`, `dgps`, `standalone`, `unknown` |
| `lat`, `lon` | 緯度・経度（10進数度） |
| `alt_m` | 高度（メートル） |
| `quality` | GPS品質（1=単独, 2=DGPS, 4=RTK Fix, 5=RTK Float） |
| `num_sats` | 使用衛星数 |
| `hdop` | 水平精度低下率 |
| `heading_deg` | 方位（度、真北基準、デュアルアンテナ使用時） |
| `baseline_m` | ベースライン長（メートル） |
| `rtcm_bytes` | 受信したRTCMデータの累積バイト数 |

### ハードウェア接続

```
[PC/Raspberry Pi]          [UM982モジュール]
USB ─────────────────────── UART (COM1/COM2/COM3)
                            │
                            ├── ANT1 (マスターアンテナ)
                            └── ANT2 (スレーブアンテナ)
```

#### 推奨ボーレート

| ボーレート | 用途 |
|-----------|------|
| 115200 | 一般的な使用（デフォルト） |

### トラブルシューティング

#### シリアルポートが開けない

```bash
# 権限を確認
ls -la /dev/ttyUSB0

# dialoutグループに追加（Ubuntuの場合）
sudo usermod -a -G dialout $USER
# ログアウト・ログインが必要
```

#### データが文字化けする

ボーレートがモジュールの設定と一致しているか確認してください。

```bash
# 自動でボーレートを検出するスクリプト例
for baud in 115200 230400 460800 921600; do
  echo "Testing $baud..."
  timeout 2 python examples/simple_position.py --port /dev/ttyUSB0 --baud $baud
done
```

#### NTRIP接続が失敗する

- ホスト名・ポート番号・マウントポイント名を確認
- ユーザ名・パスワードを確認
- ファイアウォールの設定を確認
- 基準局が稼働しているか確認

### ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照してください。

### 関連ドキュメント

- [NTRIP Protocol](https://igs.bkg.bund.de/ntrip/about)
- [NMEA 0183](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
