# UM982 RTK GPS Client

Unicore UM982デュアルアンテナGNSSモジュール用のRTKクライアント。NTRIPを通じてRTCM補正データを取得し、高精度RTK測位とマルチアンテナによる方位取得を行います。

## 特徴

- **RTK測位**: RTK Fix（cm級精度）/ RTK Float（dm級精度）に対応
- **方位取得**: デュアルアンテナによる高精度方位（heading）測定
- **NTRIP対応**: 善意の基準局やVRS等からRTCM3.x補正データを受信
- **ライブラリ提供**: Pythonから簡単に位置・方位を取得
- **JSON Lines出力**: パイプライン処理やログ解析に最適な出力形式
- **軽量**: Python標準ライブラリ + pyserialのみで動作

## 対応モジュール

- **Unicore UM982**（デュアルアンテナ対応、推奨）
- その他NMEA 0183互換のGNSSモジュール（GGA/RMC出力対応のもの）

## インストール

```bash
# リポジトリをクローン
git clone https://github.com/yourname/um982-rtk-client.git
cd um982-rtk-client

# 依存パッケージをインストール
pip install -r requirements.txt
```

## ディレクトリ構成

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

## ライブラリとしての使い方

### 基本的な使い方

```python
from um982 import UM982Client
import time

# クライアント作成・開始
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

# 終了
client.stop()
```

### NTRIP接続でRTK測位

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

### コールバックを使用

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
| `is_valid` | bool | 有効な位置データか - プロパティ |
| `is_rtk_fix` | bool | RTK Fixか - プロパティ |

## サンプルプログラム

### シンプルな位置取得

```bash
# 基本（NTRIPなし）
python examples/simple_position.py --port /dev/ttyUSB0

# NTRIP使用
python examples/simple_position.py --port /dev/ttyUSB0 \
    --ntrip-host rtk.example.com --ntrip-port 2101 \
    --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass
```

### ログ記録

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

### ログ分析

```bash
# 既存ログを分析
python tools/log_and_analyze.py rtk_log.jsonl

# グラフ付きで分析
python tools/log_and_analyze.py rtk_log.jsonl --plot

# グラフを画像として保存
python tools/log_and_analyze.py rtk_log.jsonl --save-plot analysis.png
```

## 出力フォーマット

### JSON Lines形式（デフォルト）

各行がJSON形式で出力されます。

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

### 主要フィールド一覧

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

### 人間可読形式（--pretty）

```
[12:34:56] rtk_fix      lat=35.XXXXXXXX lon=139.XXXXXXXX alt=40.5m hdop=0.7 sats=28 heading=45.2° baseline=0.612m
```

## ハードウェア接続

### UM982の接続例

```
[PC/Raspberry Pi]          [UM982モジュール]
USB ─────────────────────── UART (COM1/COM2/COM3)
                            │
                            ├── ANT1 (マスターアンテナ)
                            └── ANT2 (スレーブアンテナ)
```

### 推奨ボーレート

| ボーレート | 用途 |
|-----------|------|
| 115200 | 一般的な使用（デフォルト） |
| 460800 | 高レート出力時 |
| 921600 | 最高速度 |

## トラブルシューティング

### シリアルポートが開けない

```bash
# 権限を確認
ls -la /dev/ttyUSB0

# dialoutグループに追加（Ubuntuの場合）
sudo usermod -a -G dialout $USER
# ログアウト・ログインが必要
```

### データが文字化けする

ボーレートがモジュールの設定と一致しているか確認してください。

```bash
# 自動でボーレートを検出するスクリプト例
for baud in 115200 230400 460800 921600; do
  echo "Testing $baud..."
  timeout 2 python examples/simple_position.py --port /dev/ttyUSB0 --baud $baud
done
```

### NTRIP接続が失敗する

- ホスト名・ポート番号・マウントポイント名を確認
- ユーザ名・パスワードを確認
- ファイアウォールの設定を確認
- 基準局が稼働しているか確認

## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照してください。

## 関連ドキュメント

- [UM982 User Manual](https://www.unicorecomm.com/products/um982)
- [NTRIP Protocol](https://igs.bkg.bund.de/ntrip/about)
- [NMEA 0183](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
