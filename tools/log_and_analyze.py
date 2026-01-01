#!/usr/bin/env python3
"""
RTK GPSログ分析ツール

JSON Lines形式のログファイルを分析し、方位・ベースライン長・位置精度を評価する。

使用方法:
    # 既存ログの分析
    python log_and_analyze.py rtk_log_20240101_120000.jsonl

    # グラフ付きで分析
    python log_and_analyze.py rtk_log_20240101_120000.jsonl --plot

    # グラフを画像として保存
    python log_and_analyze.py rtk_log_20240101_120000.jsonl --save-plot analysis.png

ログの取得は examples/log_position.py を使用してください。
"""

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# matplotlibは遅延インポート（グラフ表示時のみ）
plt = None
np = None


def import_plotting_libs():
    """グラフ描画ライブラリを遅延インポート"""
    global plt, np
    if plt is None:
        try:
            import matplotlib
            matplotlib.use('Agg')  # 非対話モード（警告抑制）
            import matplotlib.pyplot as _plt
            import numpy as _np
            import warnings
            warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
            plt = _plt
            np = _np
            # 英語ラベルを使用（日本語フォントの問題を回避）
            plt.rcParams['font.family'] = 'DejaVu Sans'
            return True
        except ImportError:
            print("Warning: matplotlib/numpy がインストールされていません。", file=sys.stderr)
            print("  pip install matplotlib numpy", file=sys.stderr)
            return False
    return True


@dataclass
class LogStats:
    """ログ統計データ"""
    count: int
    mean: float
    std: float
    min_val: float
    max_val: float
    median: float


def calc_stats(values: List[float]) -> Optional[LogStats]:
    """統計値を計算"""
    if not values:
        return None

    n = len(values)
    mean = sum(values) / n

    if n > 1:
        variance = sum((x - mean) ** 2 for x in values) / (n - 1)
        std = math.sqrt(variance)
    else:
        std = 0.0

    sorted_vals = sorted(values)
    if n % 2 == 0:
        median = (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2
    else:
        median = sorted_vals[n // 2]

    return LogStats(
        count=n,
        mean=mean,
        std=std,
        min_val=min(values),
        max_val=max(values),
        median=median,
    )


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2点間の距離をメートルで計算（Haversine公式）"""
    R = 6371000  # 地球の半径（メートル）
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def latlon_to_local_meters(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """緯度経度をローカル座標（メートル）に変換"""
    # 緯度1度あたりのメートル数（概算）
    lat_m_per_deg = 111320.0
    # 経度1度あたりのメートル数（緯度による補正）
    lon_m_per_deg = 111320.0 * math.cos(math.radians(ref_lat))

    x = (lon - ref_lon) * lon_m_per_deg  # 東方向
    y = (lat - ref_lat) * lat_m_per_deg  # 北方向
    return x, y


def load_records(log_file: str, exclude_initial_non_fix: bool = True, skip_seconds: float = 0) -> Tuple[List[dict], List[dict], List[dict], List[dict], List[dict]]:
    """ログファイルを読み込み、RTK状態別に分類

    Args:
        log_file: ログファイルパス
        exclude_initial_non_fix: Trueの場合、最初のRTK Fix以前のデータを除外
        skip_seconds: 最初からスキップする秒数（安定化待ち）
    """
    all_records = []

    with open(log_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
                all_records.append(record)
            except json.JSONDecodeError:
                continue

    if not all_records:
        return [], [], [], [], []

    # 最初のRTK Fixを見つける
    first_fix_idx = 0
    if exclude_initial_non_fix:
        for i, record in enumerate(all_records):
            if record.get("rtk_state") == "rtk_fix":
                first_fix_idx = i
                break

        if first_fix_idx > 0:
            excluded_count = first_fix_idx
            print(f"[INFO] 初期の非Fixデータを除外: {excluded_count}レコード", file=sys.stderr)

    # 最初のFix以降のレコードのみを使用
    records = all_records[first_fix_idx:]

    # 指定秒数をスキップ
    if skip_seconds > 0 and records:
        start_time = records[0].get("timestamp", 0)
        skip_idx = 0
        for i, record in enumerate(records):
            if record.get("timestamp", 0) - start_time >= skip_seconds:
                skip_idx = i
                break
        if skip_idx > 0:
            print(f"[INFO] 最初の{skip_seconds:.0f}秒をスキップ: {skip_idx}レコード", file=sys.stderr)
            records = records[skip_idx:]

    rtk_fix_records = []
    rtk_float_records = []
    dgps_records = []
    standalone_records = []

    for record in records:
        state = record.get("rtk_state", "unknown")
        if state == "rtk_fix":
            rtk_fix_records.append(record)
        elif state == "rtk_float":
            rtk_float_records.append(record)
        elif state == "dgps":
            dgps_records.append(record)
        elif state == "standalone":
            standalone_records.append(record)

    return records, rtk_fix_records, rtk_float_records, dgps_records, standalone_records

def analyze_log(log_file: str, skip_seconds: float = 0) -> dict:
    """ログファイルを分析

    Args:
        log_file: ログファイルパス
        skip_seconds: 最初からスキップする秒数（安定化待ち）
    """
    print(f"\n[ANALYZE] ログ分析開始: {log_file}", file=sys.stderr)

    records, rtk_fix_records, rtk_float_records, dgps_records, standalone_records = load_records(log_file, skip_seconds=skip_seconds)

    if not records:
        print("[ANALYZE] エラー: レコードが見つかりません", file=sys.stderr)
        return {}

    total = len(records)
    duration = records[-1]["timestamp"] - records[0]["timestamp"] if len(records) > 1 else 0

    # RTK Fix時のデータを分析
    analysis_records = rtk_fix_records if rtk_fix_records else records

    # 方位（heading）
    headings = [r["heading_deg"] for r in analysis_records if r.get("heading_deg") is not None]
    heading_stats = calc_stats(headings)

    # ベースライン長
    baselines = [r["baseline_m"] for r in analysis_records if r.get("baseline_m") is not None]
    baseline_stats = calc_stats(baselines)

    # 緯度経度
    lats = [r["lat"] for r in analysis_records if r.get("lat") is not None]
    lons = [r["lon"] for r in analysis_records if r.get("lon") is not None]
    alts = [r["alt_m"] for r in analysis_records if r.get("alt_m") is not None]

    lat_stats = calc_stats(lats)
    lon_stats = calc_stats(lons)
    alt_stats = calc_stats(alts)

    # 位置のばらつき（平均位置からの距離）
    if lat_stats and lon_stats:
        mean_lat, mean_lon = lat_stats.mean, lon_stats.mean
        position_errors = []
        for r in analysis_records:
            if r.get("lat") is not None and r.get("lon") is not None:
                dist = haversine_distance(mean_lat, mean_lon, r["lat"], r["lon"])
                position_errors.append(dist)
        position_stats = calc_stats(position_errors)

        # CEP (Circular Error Probable) - 50%の点が含まれる円の半径
        if position_errors:
            sorted_errors = sorted(position_errors)
            cep50 = sorted_errors[int(len(sorted_errors) * 0.50)] if len(sorted_errors) > 0 else 0
            cep95 = sorted_errors[int(len(sorted_errors) * 0.95)] if len(sorted_errors) > 0 else 0
            cep99 = sorted_errors[int(len(sorted_errors) * 0.99)] if len(sorted_errors) > 0 else 0
        else:
            cep50 = cep95 = cep99 = 0
    else:
        position_stats = None
        cep50 = cep95 = cep99 = 0

    # HDOP
    hdops = [r["hdop"] for r in analysis_records if r.get("hdop") is not None]
    hdop_stats = calc_stats(hdops)

    # 衛星数
    sats = [r["num_sats"] for r in analysis_records if r.get("num_sats") is not None]
    sats_stats = calc_stats(sats)

    result = {
        "file": log_file,
        "total_records": total,
        "duration_sec": duration,
        "duration_min": duration / 60,
        "rtk_fix_count": len(rtk_fix_records),
        "rtk_float_count": len(rtk_float_records),
        "dgps_count": len(dgps_records),
        "standalone_count": len(standalone_records),
        "rtk_fix_rate": len(rtk_fix_records) / total * 100 if total > 0 else 0,
        "analysis_state": "rtk_fix" if rtk_fix_records else "all",
        "analysis_count": len(analysis_records),
        "heading": heading_stats.__dict__ if heading_stats else None,
        "baseline_m": baseline_stats.__dict__ if baseline_stats else None,
        "latitude": lat_stats.__dict__ if lat_stats else None,
        "longitude": lon_stats.__dict__ if lon_stats else None,
        "altitude_m": alt_stats.__dict__ if alt_stats else None,
        "position_error_m": position_stats.__dict__ if position_stats else None,
        "cep50_m": cep50,
        "cep95_m": cep95,
        "cep99_m": cep99,
        "hdop": hdop_stats.__dict__ if hdop_stats else None,
        "num_sats": sats_stats.__dict__ if sats_stats else None,
        "mean_position": {
            "lat": lat_stats.mean if lat_stats else None,
            "lon": lon_stats.mean if lon_stats else None,
            "alt_m": alt_stats.mean if alt_stats else None,
        },
    }

    return result


def plot_analysis(log_file: str, analysis: dict, output_image: Optional[str] = None, skip_seconds: float = 0):
    """分析結果をグラフで表示"""
    if not import_plotting_libs():
        return

    records, rtk_fix_records, rtk_float_records, dgps_records, standalone_records = load_records(log_file, skip_seconds=skip_seconds)

    if not records:
        return

    # 分析対象のレコード
    analysis_records = rtk_fix_records if rtk_fix_records else records

    # 平均位置を基準点として使用
    mean_lat = analysis["mean_position"]["lat"]
    mean_lon = analysis["mean_position"]["lon"]

    if mean_lat is None or mean_lon is None:
        print("Error: 位置データがありません", file=sys.stderr)
        return

    # ローカル座標に変換（cm単位）- RTK Fixのみを対象
    positions_fix = []

    for r in rtk_fix_records:
        if r.get("lat") and r.get("lon"):
            x, y = latlon_to_local_meters(r["lat"], r["lon"], mean_lat, mean_lon)
            positions_fix.append((x * 100, y * 100))  # cm単位

    # Figure作成（2x2のサブプロット）
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f'RTK GPS Analysis: {os.path.basename(log_file)}', fontsize=14, fontweight='bold')

    # ========== 1. 位置散布図（左上） ==========
    ax1 = axes[0, 0]

    # CEP円を描画
    cep50_cm = analysis["cep50_m"] * 100
    cep95_cm = analysis["cep95_m"] * 100
    cep99_cm = analysis["cep99_m"] * 100

    circle50 = plt.Circle((0, 0), cep50_cm, fill=False, color='green', linestyle='--', linewidth=1.5, label=f'CEP50: {cep50_cm:.1f}cm')
    circle95 = plt.Circle((0, 0), cep95_cm, fill=False, color='orange', linestyle='--', linewidth=1.5, label=f'CEP95: {cep95_cm:.1f}cm')
    circle99 = plt.Circle((0, 0), cep99_cm, fill=False, color='red', linestyle='--', linewidth=1.5, label=f'CEP99: {cep99_cm:.1f}cm')

    ax1.add_patch(circle50)
    ax1.add_patch(circle95)
    ax1.add_patch(circle99)

    # RTK Fixの点をプロット
    if positions_fix:
        xs, ys = zip(*positions_fix)
        ax1.scatter(xs, ys, c='blue', s=1, alpha=0.5, label=f'RTK Fix ({len(positions_fix)})')

    # 原点（平均位置）
    ax1.scatter([0], [0], c='red', s=100, marker='+', linewidths=2, label='Mean Position')

    ax1.set_xlabel('East (cm)')
    ax1.set_ylabel('North (cm)')
    ax1.set_title('Position Scatter Plot')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)

    # 軸範囲を調整（CEP99の1.5倍程度）
    max_range = max(cep99_cm * 1.5, 10)
    ax1.set_xlim(-max_range, max_range)
    ax1.set_ylim(-max_range, max_range)

    # ========== 2. 方位（Heading）のヒストグラム（右上） ==========
    ax2 = axes[0, 1]
    headings = [r["heading_deg"] for r in analysis_records if r.get("heading_deg") is not None]

    if headings:
        ax2.hist(headings, bins=50, color='steelblue', edgecolor='white', alpha=0.7)
        h = analysis["heading"]
        ax2.axvline(h["mean"], color='red', linestyle='-', linewidth=2, label=f'Mean: {h["mean"]:.2f}°')
        ax2.axvline(h["mean"] - h["std"], color='orange', linestyle='--', linewidth=1, label=f'Std: ±{h["std"]:.2f}°')
        ax2.axvline(h["mean"] + h["std"], color='orange', linestyle='--', linewidth=1)
        ax2.set_xlabel('Heading (degrees)')
        ax2.set_ylabel('Count')
        ax2.set_title('Heading Distribution')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

    # ========== 3. ベースライン長のヒストグラム（左下） ==========
    ax3 = axes[1, 0]
    baselines = [r["baseline_m"] * 100 for r in analysis_records if r.get("baseline_m") is not None]  # cm

    if baselines:
        ax3.hist(baselines, bins=50, color='forestgreen', edgecolor='white', alpha=0.7)
        b = analysis["baseline_m"]
        mean_cm = b["mean"] * 100
        std_cm = b["std"] * 100
        ax3.axvline(mean_cm, color='red', linestyle='-', linewidth=2, label=f'Mean: {mean_cm:.2f}cm')
        ax3.axvline(mean_cm - std_cm, color='orange', linestyle='--', linewidth=1, label=f'Std: ±{std_cm:.2f}cm')
        ax3.axvline(mean_cm + std_cm, color='orange', linestyle='--', linewidth=1)
        ax3.set_xlabel('Baseline Length (cm)')
        ax3.set_ylabel('Count')
        ax3.set_title('Baseline Length Distribution (Antenna Distance)')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)

    # ========== 4. 時系列プロット（右下） ==========
    ax4 = axes[1, 1]

    # タイムスタンプを相対時間（分）に変換
    if len(analysis_records) > 1:
        start_time = analysis_records[0]["timestamp"]
        times = [(r["timestamp"] - start_time) / 60 for r in analysis_records]

        # 位置誤差の時系列
        errors = []
        for r in analysis_records:
            if r.get("lat") and r.get("lon"):
                dist = haversine_distance(mean_lat, mean_lon, r["lat"], r["lon"]) * 100  # cm
                errors.append(dist)
            else:
                errors.append(None)

        valid_times = [t for t, e in zip(times, errors) if e is not None]
        valid_errors = [e for e in errors if e is not None]

        ax4.plot(valid_times, valid_errors, 'b-', alpha=0.5, linewidth=0.5, label='Position Error')

        # 移動平均（100点）
        if len(valid_errors) > 100:
            window = 100
            moving_avg = []
            for i in range(len(valid_errors)):
                start_idx = max(0, i - window // 2)
                end_idx = min(len(valid_errors), i + window // 2)
                moving_avg.append(sum(valid_errors[start_idx:end_idx]) / (end_idx - start_idx))
            ax4.plot(valid_times, moving_avg, 'r-', linewidth=2, label=f'Moving Avg ({window}pts)')

        ax4.set_xlabel('Time (minutes)')
        ax4.set_ylabel('Position Error (cm)')
        ax4.set_title('Position Error over Time')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)

        # CEP線
        ax4.axhline(cep50_cm, color='green', linestyle='--', alpha=0.5, label='CEP50')
        ax4.axhline(cep95_cm, color='orange', linestyle='--', alpha=0.5, label='CEP95')

    plt.tight_layout()

    # 保存または表示
    if output_image:
        plt.savefig(output_image, dpi=150, bbox_inches='tight')
        print(f"[PLOT] グラフを保存しました: {output_image}", file=sys.stderr)
    else:
        plt.show()

    plt.close()


def print_report(analysis: dict):
    """分析レポートを出力"""
    print("\n" + "=" * 70)
    print("RTK GPS ログ分析レポート")
    print("=" * 70)

    print(f"\n【基本情報】")
    print(f"  ファイル: {analysis['file']}")
    print(f"  総レコード数: {analysis['total_records']:,}")
    print(f"  記録時間: {analysis['duration_min']:.1f}分 ({analysis['duration_sec']:.0f}秒)")
    print(f"  分析対象: {analysis['analysis_state']} ({analysis['analysis_count']:,}レコード)")

    print(f"\n【RTK状態の内訳】")
    print(f"  RTK Fix:     {analysis['rtk_fix_count']:>7,} ({analysis['rtk_fix_rate']:.1f}%)")
    print(f"  RTK Float:   {analysis['rtk_float_count']:>7,}")
    print(f"  DGPS:        {analysis['dgps_count']:>7,}")
    print(f"  Standalone:  {analysis['standalone_count']:>7,}")

    if analysis.get("heading"):
        h = analysis["heading"]
        print(f"\n【方位 (Heading)】")
        print(f"  平均:    {h['mean']:.4f}°")
        print(f"  標準偏差: {h['std']:.4f}°")
        print(f"  最小:    {h['min_val']:.4f}°")
        print(f"  最大:    {h['max_val']:.4f}°")
        print(f"  中央値:  {h['median']:.4f}°")

    if analysis.get("baseline_m"):
        b = analysis["baseline_m"]
        print(f"\n【ベースライン長 (アンテナ間距離)】")
        print(f"  平均:    {b['mean']*100:.2f} cm ({b['mean']:.4f} m)")
        print(f"  標準偏差: {b['std']*100:.2f} cm ({b['std']:.4f} m)")
        print(f"  最小:    {b['min_val']*100:.2f} cm")
        print(f"  最大:    {b['max_val']*100:.2f} cm")
        print(f"  中央値:  {b['median']*100:.2f} cm")

    if analysis.get("position_error_m"):
        p = analysis["position_error_m"]
        print(f"\n【位置精度 (平均位置からのばらつき)】")
        print(f"  平均誤差:  {p['mean']*100:.2f} cm ({p['mean']:.4f} m)")
        print(f"  標準偏差:  {p['std']*100:.2f} cm")
        print(f"  最大誤差:  {p['max_val']*100:.2f} cm")
        print(f"  CEP 50%:   {analysis['cep50_m']*100:.2f} cm (50%の点がこの半径内)")
        print(f"  CEP 95%:   {analysis['cep95_m']*100:.2f} cm (95%の点がこの半径内)")
        print(f"  CEP 99%:   {analysis['cep99_m']*100:.2f} cm (99%の点がこの半径内)")

    if analysis.get("altitude_m"):
        a = analysis["altitude_m"]
        print(f"\n【高度】")
        print(f"  平均:    {a['mean']:.2f} m")
        print(f"  標準偏差: {a['std']:.2f} m")
        print(f"  最小:    {a['min_val']:.2f} m")
        print(f"  最大:    {a['max_val']:.2f} m")

    if analysis.get("hdop"):
        hd = analysis["hdop"]
        print(f"\n【HDOP (精度低下率)】")
        print(f"  平均:    {hd['mean']:.2f}")
        print(f"  最小:    {hd['min_val']:.2f}")
        print(f"  最大:    {hd['max_val']:.2f}")

    if analysis.get("num_sats"):
        s = analysis["num_sats"]
        print(f"\n【使用衛星数】")
        print(f"  平均:    {s['mean']:.1f}")
        print(f"  最小:    {s['min_val']:.0f}")
        print(f"  最大:    {s['max_val']:.0f}")

    if analysis.get("mean_position") and analysis["mean_position"]["lat"]:
        mp = analysis["mean_position"]
        print(f"\n【平均位置】")
        print(f"  緯度:  {mp['lat']:.8f}")
        print(f"  経度:  {mp['lon']:.8f}")
        print(f"  高度:  {mp['alt_m']:.2f} m")
        print(f"  Google Maps: https://www.google.com/maps?q={mp['lat']},{mp['lon']}")

    print("\n" + "=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="RTK GPSログ分析ツール",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  %(prog)s rtk_log.jsonl              # 基本的な分析
  %(prog)s rtk_log.jsonl --plot       # グラフ付きで分析
  %(prog)s rtk_log.jsonl -s out.png   # グラフを画像保存

ログの取得は examples/log_position.py を使用してください。
"""
    )
    parser.add_argument("logfile", help="分析するログファイル（JSON Lines形式）")
    parser.add_argument(
        "--json", action="store_true",
        help="結果をJSON形式で出力"
    )
    parser.add_argument(
        "--plot", "-p", action="store_true",
        help="グラフを表示"
    )
    parser.add_argument(
        "--save-plot", "-s", type=str, metavar="FILE",
        help="グラフを画像ファイルに保存（例: analysis.png）"
    )
    parser.add_argument(
        "--skip", type=float, default=120,
        help="最初からスキップする秒数（安定化待ち）。デフォルト: 120秒"
    )

    args = parser.parse_args()

    if not os.path.exists(args.logfile):
        print(f"Error: ファイルが見つかりません: {args.logfile}", file=sys.stderr)
        sys.exit(1)

    analysis = analyze_log(args.logfile, skip_seconds=args.skip)
    if analysis:
        if args.json:
            print(json.dumps(analysis, indent=2, ensure_ascii=False))
        else:
            print_report(analysis)

        # グラフ表示
        if args.plot or args.save_plot:
            plot_analysis(args.logfile, analysis, args.save_plot, skip_seconds=args.skip)


if __name__ == "__main__":
    main()
