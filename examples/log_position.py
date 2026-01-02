#!/usr/bin/env python3
"""
位置データをJSON Lines形式でログ記録するサンプル

tools/log_and_analyze.py で分析可能な形式で出力する。

使用方法:
    # 1時間のログを取得
    python log_position.py --port /dev/ttyUSB0 --duration 3600

    # NTRIP使用で10分間ログ
    python log_position.py --port /dev/ttyUSB0 --duration 600 \
        --ntrip-host rtk.example.com --ntrip-port 2101 \
        --ntrip-mount RTCM3 --ntrip-user user --ntrip-pass pass

    # 出力ファイル名を指定
    python log_position.py --port /dev/ttyUSB0 --duration 3600 -o mylog.jsonl

    # ログ後に自動分析
    python log_position.py --port /dev/ttyUSB0 --duration 3600 --analyze
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from um982 import UM982Client


def main():
    parser = argparse.ArgumentParser(
        description="UM982 位置データログ記録",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="シリアルポート")
    parser.add_argument("--baud", type=int, default=115200, help="ボーレート")
    parser.add_argument("--rate", type=int, default=10, help="出力レート（Hz、デフォルト: 10）")
    parser.add_argument(
        "--duration", "-d", type=int, default=3600,
        help="記録時間（秒）。デフォルト: 3600秒（1時間）"
    )
    parser.add_argument(
        "--output", "-o", type=str,
        help="出力ファイル名（省略時は自動生成）"
    )
    parser.add_argument(
        "--analyze", "-a", action="store_true",
        help="ログ取得後に自動で分析を実行"
    )
    parser.add_argument(
        "--pretty", "-p", action="store_true",
        help="コンソールに人間可読形式でも表示"
    )
    parser.add_argument("--ntrip-host", help="NTRIPキャスターホスト")
    parser.add_argument("--ntrip-port", type=int, help="NTRIPポート")
    parser.add_argument("--ntrip-mount", help="NTRIPマウントポイント")
    parser.add_argument("--ntrip-user", help="NTRIPユーザ名")
    parser.add_argument("--ntrip-pass", help="NTRIPパスワード")
    args = parser.parse_args()

    # 出力ファイル名を生成
    if args.output:
        output_file = args.output
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"rtk_log_{timestamp}.jsonl"

    # クライアント作成
    print(f"[LOG] Connecting to {args.port}...", file=sys.stderr)
    client = UM982Client(args.port, args.baud, output_rate=args.rate)

    try:
        client.start()
        print(f"[LOG] Connected.", file=sys.stderr)

        # NTRIP設定
        if all([args.ntrip_host, args.ntrip_port, args.ntrip_mount, args.ntrip_user, args.ntrip_pass]):
            print(f"[LOG] Starting NTRIP client...", file=sys.stderr)
            client.start_ntrip(
                host=args.ntrip_host,
                port=args.ntrip_port,
                mountpoint=args.ntrip_mount,
                user=args.ntrip_user,
                password=args.ntrip_pass,
            )

        print(f"[LOG] Output file: {output_file}", file=sys.stderr)
        print(f"[LOG] Duration: {args.duration}秒 ({args.duration/60:.1f}分)", file=sys.stderr)
        print(f"[LOG] Press Ctrl+C to stop early", file=sys.stderr)
        print("-" * 60, file=sys.stderr)

        start_time = time.time()
        record_count = 0
        last_progress = 0

        with open(output_file, "w") as f:
            while True:
                elapsed = time.time() - start_time
                if elapsed >= args.duration:
                    print(f"\n[LOG] Duration reached: {args.duration}秒", file=sys.stderr)
                    break

                pos = client.get_position()
                if pos and pos.is_valid:
                    # log_and_analyze.py互換のJSON形式で出力
                    record = {
                        "timestamp": pos.timestamp,
                        "rtk_state": pos.rtk_state,
                        "lat": pos.lat,
                        "lon": pos.lon,
                        "alt_m": pos.alt,
                        "quality": None,  # GGAのquality
                        "num_sats": pos.num_sats,
                        "hdop": pos.hdop,
                        "diff_age_s": None,
                        "ref_station_id": None,
                        "rtcm_bytes": client.get_rtcm_bytes(),
                    }

                    # UNIHEADING データ
                    uni = client.get_uniheading()
                    if uni:
                        record.update({
                            "heading_deg": uni.heading_deg,
                            "pitch_deg": uni.pitch_deg,
                            "baseline_m": uni.length_m,
                            "heading_stddev": uni.heading_stddev,
                            "pitch_stddev": uni.pitch_stddev,
                            "uniheading_age_s": round(time.time() - uni.timestamp, 3),
                            "uniheading_stn_id": uni.stn_id,
                            "uniheading_num_svs": uni.num_svs,
                            "uniheading_num_soln_svs": uni.num_soln_svs,
                        })

                    # RMC データ
                    rmc = client.get_rmc()
                    if rmc:
                        record.update({
                            "rmc_speed_knots": rmc.speed_knots,
                            "rmc_course_deg": rmc.course_deg,
                            "rmc_mode": rmc.mode,
                            "rmc_age_s": round(time.time() - rmc.timestamp, 3),
                        })

                    # GGA データからquality等を取得
                    gga = client.get_gga()
                    if gga:
                        record["quality"] = gga.quality
                        record["diff_age_s"] = gga.diff_age
                        record["ref_station_id"] = gga.ref_station_id

                    # ファイルに書き込み
                    f.write(json.dumps(record, default=str) + "\n")
                    f.flush()
                    record_count += 1

                    # コンソール表示
                    if args.pretty:
                        lat_str = f"{pos.lat:.8f}" if pos.lat else "---"
                        lon_str = f"{pos.lon:.8f}" if pos.lon else "---"
                        alt_str = f"{pos.alt:.1f}" if pos.alt else "---"
                        heading_str = f"{pos.heading:.1f}" if pos.heading else "---"
                        baseline_str = f"{pos.baseline_m:.3f}" if pos.baseline_m else "---"
                        time_str = time.strftime('%H:%M:%S')
                        print(
                            f"[{time_str}] {pos.rtk_state:12s} "
                            f"lat={lat_str} lon={lon_str} alt={alt_str}m "
                            f"hdop={pos.hdop} sats={pos.num_sats} "
                            f"heading={heading_str}° baseline={baseline_str}m"
                        )

                # 進捗表示（10秒ごと）
                progress_interval = 10
                if int(elapsed) // progress_interval > last_progress:
                    last_progress = int(elapsed) // progress_interval
                    remaining = args.duration - elapsed
                    print(
                        f"\r[LOG] Elapsed: {elapsed/60:.1f}min | "
                        f"Remaining: {remaining/60:.1f}min | "
                        f"Records: {record_count}",
                        end="", file=sys.stderr
                    )

                time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"\n[LOG] Interrupted by user", file=sys.stderr)
    finally:
        client.stop()

    print(f"[LOG] Completed: {record_count} records saved to {output_file}", file=sys.stderr)

    # 自動分析
    if args.analyze and record_count > 0:
        print(f"\n[LOG] Running analysis...", file=sys.stderr)
        import subprocess
        tools_dir = Path(__file__).parent.parent / "tools"
        analyzer = tools_dir / "log_and_analyze.py"
        if analyzer.exists():
            subprocess.run([sys.executable, str(analyzer), output_file, "--plot"])
        else:
            print(f"[LOG] Analyzer not found: {analyzer}", file=sys.stderr)


if __name__ == "__main__":
    main()
