"""
NMEA/Unicoreメッセージパーサー
"""

import time
from typing import Optional

from .types import GGAData, RMCData, UniheadingData


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

    return GGAData(
        lat=lat,
        lon=lon,
        alt=alt,
        quality=quality,
        num_sats=num_sats,
        hdop=hdop,
        diff_age=diff_age,
        ref_station_id=ref_station_id or None,
        timestamp=time.time(),
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

    return RMCData(
        speed_knots=speed_knots,
        course_deg=course_deg,
        mode=mode_field,
        timestamp=time.time(),
    )


def parse_uniheading(sentence: str) -> Optional[UniheadingData]:
    """
    Unicore UNIHEADING文をパース。

    フォーマット: #UNIHEADINGA,<header>;<data>*<checksum>
    """
    if not sentence.startswith("#") or "UNIHEADING" not in sentence:
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
        data_part = sentence.split(";", 1)[1]
        if "*" in data_part:
            data_part = data_part.split("*", 1)[0]
        fields = data_part.split(",")

        if len(fields) < 8:
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
            timestamp=time.time(),
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
