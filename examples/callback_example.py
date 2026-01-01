#!/usr/bin/env python3
"""
コールバックを使った位置取得サンプル

位置データが更新されるたびにコールバック関数が呼ばれる。

使用方法:
    python callback_example.py --port /dev/ttyUSB0
"""

import argparse
import sys
import time

from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from um982 import UM982Client, PositionData


def on_position_update(pos: PositionData):
    """位置データ更新時のコールバック"""
    if not pos.is_valid:
        return

    # RTK Fixの時だけ詳細表示
    if pos.is_rtk_fix:
        print(
            f"[RTK FIX] "
            f"Lat: {pos.lat:.8f}  "
            f"Lon: {pos.lon:.8f}  "
            f"Alt: {pos.alt:.2f}m  "
            f"Heading: {pos.heading:.2f}°" if pos.heading else ""
        )
    else:
        print(f"[{pos.rtk_state}] Waiting for RTK Fix...")


def main():
    parser = argparse.ArgumentParser(description="コールバックサンプル")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="シリアルポート")
    parser.add_argument("--baud", type=int, default=115200, help="ボーレート")
    args = parser.parse_args()

    client = UM982Client(args.port, args.baud)

    # コールバック設定
    client.set_position_callback(on_position_update)

    try:
        client.start()
        print("Listening for position updates... (Ctrl+C to stop)", file=sys.stderr)

        # メインスレッドは待機するだけ
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...", file=sys.stderr)
    finally:
        client.stop()


if __name__ == "__main__":
    main()
