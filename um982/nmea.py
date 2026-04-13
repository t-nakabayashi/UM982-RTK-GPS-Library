"""
NMEA/Unicoreメッセージパーサー
"""

import calendar
import logging
import time
from typing import Optional

from .types import GGAData, RMCData, UniheadingData

logger = logging.getLogger(__name__)

# GPS エポック (1980-01-06 00:00:00 UTC) の Unix 時刻
GPS_EPOCH_UNIX = 315964800
# GPS - UTC の閏秒オフセット（2017-01-01 以降 18 秒）
GPS_LEAP_SECONDS = 18


def _parse_nmea_hhmmss(time_field: str) -> Optional[tuple]:
    """NMEA の hhmmss[.sss] を (hh, mm, ss_float) に分解"""
    if not time_field or len(time_field) < 6:
        return None
    try:
        hh = int(time_field[0:2])
        mm = int(time_field[2:4])
        ss = float(time_field[4:])
    except ValueError:
        return None
    if not (0 <= hh < 24 and 0 <= mm < 60 and 0 <= ss < 61):
        return None
    return hh, mm, ss


def nmea_utc_to_unix(time_field: str, date_field: Optional[str] = None,
                     reference_unix: Optional[float] = None) -> Optional[float]:
    """
    NMEA UTC 時刻文字列から Unix 時刻 (UTC) を構築。

    Args:
        time_field: NMEA の hhmmss[.sss]（例: "123456.789"）
        date_field: NMEA の ddmmyy（例: "130424"）。None の場合は
                    reference_unix の UTC 日付から ±1 日の候補のうち最も近い
                    ものを採用（GGA など日付を含まないセンテンス用）。
        reference_unix: 参照 Unix 時刻（デフォルトは time.time()）

    Returns:
        Unix 時刻（float）、解析失敗時は None
    """
    parsed = _parse_nmea_hhmmss(time_field)
    if parsed is None:
        return None
    hh, mm, ss = parsed
    time_of_day = hh * 3600 + mm * 60 + ss

    if date_field:
        if len(date_field) != 6:
            return None
        try:
            dd = int(date_field[0:2])
            mo = int(date_field[2:4])
            yy = int(date_field[4:6])
        except ValueError:
            return None
        year = 2000 + yy  # NMEA は 2 桁年。2000 年代を仮定。
        try:
            day_unix = calendar.timegm((year, mo, dd, 0, 0, 0, 0, 0, 0))
        except (OverflowError, ValueError):
            return None
        return day_unix + time_of_day

    # 日付がない場合: PC 時計から最も近い UTC 日を採用
    ref = reference_unix if reference_unix is not None else time.time()
    ref_day = int(ref // 86400) * 86400
    candidates = (
        ref_day - 86400 + time_of_day,
        ref_day + time_of_day,
        ref_day + 86400 + time_of_day,
    )
    return min(candidates, key=lambda t: abs(t - ref))


def gps_week_seconds_to_unix(week: int, seconds: float,
                             leap_seconds: int = GPS_LEAP_SECONDS) -> float:
    """
    GPS 週番号と週内秒から Unix 時刻（UTC）を構築。

    注: leap_seconds は GPS-UTC オフセット（2017 年以降 18 秒）。将来
    追加される閏秒に追従する必要がある場合は呼び出し側で上書きする。
    """
    return GPS_EPOCH_UNIX + week * 7 * 86400 + seconds - leap_seconds


def nmea_deg_to_decimal(value: str, direction: str) -> Optional[float]:
    """
    NMEA形式の緯度経度を10進数度に変換。

    Args:
        value: NMEA形式の値（例: "3538.92590151"）
        direction: 方向（N/S/E/W）

    Returns:
        10進数度、または変換失敗時はNone
    """
    if not value or not direction:
        return None
    try:
        if "." not in value:
            return None
        deg_len = 2 if direction in ("N", "S") else 3
        deg = float(value[:deg_len])
        minutes = float(value[deg_len:])
        sign = -1 if direction in ("S", "W") else 1
        return sign * (deg + minutes / 60.0)
    except ValueError:
        return None


def parse_gga(sentence: str) -> Optional[GGAData]:
    """
    NMEA GGA文をパース。

    対応フォーマット: $GPGGA, $GNGGA, $GLGGA等
    """
    if not sentence.startswith("$") or "GGA" not in sentence:
        return None
    parts = sentence.split(",")
    if len(parts) < 15:
        return None

    lat = nmea_deg_to_decimal(parts[2], parts[3])
    lon = nmea_deg_to_decimal(parts[4], parts[5])

    try:
        quality = int(parts[6]) if parts[6] else None
    except ValueError:
        quality = None
    try:
        num_sats = int(parts[7]) if parts[7] else None
    except ValueError:
        num_sats = None
    try:
        hdop = float(parts[8]) if parts[8] else None
    except ValueError:
        hdop = None
    try:
        alt = float(parts[9]) if parts[9] else None
    except ValueError:
        alt = None
    try:
        diff_age = float(parts[13]) if len(parts) > 13 and parts[13] else None
    except ValueError:
        diff_age = None

    ref_station_id = parts[14].split("*")[0] if len(parts) > 14 else None

    # GGA の UTC 時刻フィールド (hhmmss.sss) から Unix 時刻を構築。
    # GGA には日付が含まれないため PC 時計の UTC 日付を使って補完する。
    # 時刻フィールドが欠落・不正な場合は異常データとして破棄する。
    timestamp = nmea_utc_to_unix(parts[1]) if parts[1] else None
    if timestamp is None:
        logger.error("GGA: invalid UTC time field %r, discarding sentence: %s",
                     parts[1], sentence)
        return None

    return GGAData(
        lat=lat,
        lon=lon,
        alt=alt,
        quality=quality,
        num_sats=num_sats,
        hdop=hdop,
        diff_age=diff_age,
        ref_station_id=ref_station_id or None,
        timestamp=timestamp,
        raw=sentence,
    )


def parse_rmc(sentence: str) -> Optional[RMCData]:
    """
    NMEA RMC文をパース。

    対応フォーマット: $GPRMC, $GNRMC等
    """
    if not sentence.startswith("$") or "RMC" not in sentence:
        return None
    parts = sentence.split(",")
    if len(parts) < 12:
        return None

    try:
        speed_knots = float(parts[7]) if parts[7] else None
    except ValueError:
        speed_knots = None
    try:
        course_deg = float(parts[8]) if parts[8] else None
    except ValueError:
        course_deg = None

    mode_field = None
    if len(parts) >= 13:
        mode_field = parts[12].split("*")[0] if parts[12] else None

    # RMC は UTC の fix time (parts[1]) と日付 (parts[9]) を持つので
    # これを直接 Unix 時刻に変換して GPS 由来のタイムスタンプにする。
    # どちらかが欠落・不正な場合は異常データとして破棄する。
    timestamp = nmea_utc_to_unix(parts[1], parts[9]) if parts[1] and parts[9] else None
    if timestamp is None:
        logger.error("RMC: invalid UTC time/date fields (time=%r, date=%r), "
                     "discarding sentence: %s", parts[1], parts[9], sentence)
        return None

    return RMCData(
        speed_knots=speed_knots,
        course_deg=course_deg,
        mode=mode_field,
        timestamp=timestamp,
    )


def parse_uniheading(sentence: str) -> Optional[UniheadingData]:
    """
    Unicore HEADING文をパース。

    フォーマット: #HEADINGA,<header>;<data>*<checksum>
    または: #UNIHEADINGA,<header>;<data>*<checksum>
    """
    if not sentence.startswith("#") or "HEADING" not in sentence:
        return None
    if ";" not in sentence:
        return None

    def safe_float(val: Optional[str]) -> Optional[float]:
        if val is None or val == "":
            return None
        try:
            return float(val)
        except ValueError:
            return None

    def safe_int(val: Optional[str]) -> Optional[int]:
        if val is None or val == "":
            return None
        try:
            return int(val)
        except ValueError:
            return None

    try:
        header_part, data_part = sentence.split(";", 1)
        if "*" in data_part:
            data_part = data_part.split("*", 1)[0]
        fields = data_part.split(",")

        if len(fields) < 8:
            return None

        # Unicore / NovAtel スタイルのレシーバヘッダから GPS 週/週内秒を抽出:
        #   #HEADINGA,<port>,<seq>,<idle>,<time_status>,<week>,<seconds>,...
        # week, seconds から Unix 時刻 (UTC) を構築する。
        # ヘッダの時刻情報が欠落・不正な場合は異常データとして破棄する。
        timestamp: Optional[float] = None
        header_fields = header_part.split(",")
        if len(header_fields) > 6:
            week = safe_int(header_fields[5])
            seconds = safe_float(header_fields[6])
            if week is not None and seconds is not None and week > 0:
                timestamp = gps_week_seconds_to_unix(week, seconds)
        if timestamp is None:
            logger.error("HEADINGA: invalid GPS week/seconds in header %r, "
                         "discarding sentence: %s", header_part, sentence)
            return None

        return UniheadingData(
            length_m=safe_float(fields[2]),
            heading_deg=safe_float(fields[3]),
            pitch_deg=safe_float(fields[4]),
            heading_stddev=safe_float(fields[6]) if len(fields) > 6 else None,
            pitch_stddev=safe_float(fields[7]) if len(fields) > 7 else None,
            stn_id=fields[8].strip('"') if len(fields) > 8 and fields[8] else None,
            num_svs=safe_int(fields[9]) if len(fields) > 9 else None,
            num_soln_svs=safe_int(fields[10]) if len(fields) > 10 else None,
            num_obs=safe_int(fields[11]) if len(fields) > 11 else None,
            num_multi=safe_int(fields[12]) if len(fields) > 12 else None,
            timestamp=timestamp,
        )
    except (ValueError, IndexError):
        return None


def determine_rtk_state(gga: Optional[GGAData], rmc: Optional[RMCData]) -> str:
    """
    RTK状態を判定。

    Returns:
        "rtk_fix": RTK固定解（cm級精度）
        "rtk_float": RTKフロート解（dm級精度）
        "dgps": 差分GPS
        "standalone": 単独測位
        "unknown": 不明
    """
    if gga and gga.quality is not None:
        if gga.quality == 4:
            return "rtk_fix"
        if gga.quality == 5:
            return "rtk_float"
        if gga.quality == 2:
            return "dgps"
        if gga.quality == 1:
            return "standalone"

    if rmc and rmc.mode:
        mode = rmc.mode.upper()
        if mode.startswith("R"):
            return "rtk_fix"
        if mode.startswith("F"):
            return "rtk_float"
        if mode.startswith("D"):
            return "dgps"
        if mode.startswith("A"):
            return "standalone"

    return "unknown"
