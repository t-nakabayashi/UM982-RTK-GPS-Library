#!/usr/bin/env python3
"""
シンプルな位置取得サンプル

緯度、経度、高度、方位を取得して表示する。

使用方法:
    # 基本（NTRIPなし）
    python simple_position.py --port /dev/ttyUSB0

    # NTRIP使用（RTK測位）
    python simple_position.py --port /dev/ttyUSB0 \
        --ntrip-host rtk.example.com --ntrip-port 2101 \
        --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass
"""

import argparse
import sys
import time

# ライブラリのパスを追加（開発時用）
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from um982 import UM982Client


def main():
    parser = argparse.ArgumentParser(description="UM982 シンプル位置取得サンプル")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="シリアルポート")
    parser.add_argument("--baud", type=int, default=115200, help="ボーレート")
    parser.add_argument("--rate", type=int, default=10, help="出力レート（Hz、デフォルト: 10）")
    parser.add_argument("--ntrip-host", help="NTRIPキャスターホスト")
    parser.add_argument("--ntrip-port", type=int, help="NTRIPポート")
    parser.add_argument("--ntrip-mount", help="NTRIPマウントポイント")
    parser.add_argument("--ntrip-user", help="NTRIPユーザ名")
    parser.add_argument("--ntrip-pass", help="NTRIPパスワード")
    args = parser.parse_args()

    # クライアント作成
    print(f"Connecting to {args.port}...", file=sys.stderr)
    client = UM982Client(args.port, args.baud, output_rate=args.rate)

    try:
        client.start()
        print(f"Connected.", file=sys.stderr)

        # NTRIP設定
        if all([args.ntrip_host, args.ntrip_port, args.ntrip_mount, args.ntrip_user, args.ntrip_pass]):
            print(f"Starting NTRIP client...", file=sys.stderr)
            client.start_ntrip(
                host=args.ntrip_host,
                port=args.ntrip_port,
                mountpoint=args.ntrip_mount,
                user=args.ntrip_user,
                password=args.ntrip_pass,
            )

        print("Waiting for position data... (Ctrl+C to stop)", file=sys.stderr)
        print("-" * 60, file=sys.stderr)

        # メインループ
        while True:
            pos = client.get_position()

            if pos and pos.is_valid:
                # 緯度・経度・高度
                lat_str = f"{pos.lat:.8f}" if pos.lat else "---"
                lon_str = f"{pos.lon:.8f}" if pos.lon else "---"
                alt_str = f"{pos.alt:.2f}m" if pos.alt else "---"

                # 方位（デュアルアンテナ）
                heading_str = f"{pos.heading:.2f}°" if pos.heading else "---"
                heading_stddev_str = f"±{pos.heading_stddev:.2f}°" if pos.heading_stddev else ""
                baseline_str = f"{pos.baseline_m:.3f}m" if pos.baseline_m else "---"

                # 速度（RMC）
                speed_str = f"{pos.speed_kmh:.1f}km/h" if pos.speed_kmh else "---"

                # 補正データ経過時間
                diff_age_str = f"{pos.diff_age:.1f}s" if pos.diff_age else "---"

                # RTK状態
                rtk_state = pos.rtk_state.upper().replace("_", " ")

                print(
                    f"[{rtk_state:10s}] "
                    f"Lat: {lat_str}  Lon: {lon_str}  Alt: {alt_str}  "
                    f"Heading: {heading_str}{heading_stddev_str}  Baseline: {baseline_str}  "
                    f"Speed: {speed_str}  Sats: {pos.num_sats}  DiffAge: {diff_age_str}"
                )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping...", file=sys.stderr)
    finally:
        client.stop()


if __name__ == "__main__":
    main()
