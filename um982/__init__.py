"""
UM982 RTK GPS ライブラリ

Unicore UM982デュアルアンテナGNSSモジュール用Pythonライブラリ。
NTRIPを通じてRTCM補正データを取得し、高精度RTK測位と方位取得を行う。

使用例:
    from um982 import UM982Client

    # クライアント作成・開始
    client = UM982Client("/dev/ttyUSB0")
    client.start()

    # NTRIPで補正データを受信
    client.start_ntrip(
        host="rtk.example.com",
        port=2101,
        mountpoint="RTCM3",
        user="user",
        password="pass"
    )

    # 位置データ取得
    while True:
        pos = client.get_position()
        if pos and pos.is_valid:
            print(f"Lat: {pos.lat:.8f}")
            print(f"Lon: {pos.lon:.8f}")
            print(f"Alt: {pos.alt:.2f}m")
            print(f"Heading: {pos.heading:.2f}°")
            print(f"RTK: {pos.rtk_state}")
        time.sleep(0.1)
"""

from .client import UM982Client
from .types import PositionData, GGAData, RMCData, UniheadingData
from .nmea import parse_gga, parse_rmc, parse_uniheading, determine_rtk_state
from .ntrip import NtripClient

__version__ = "1.0.0"
__all__ = [
    "UM982Client",
    "PositionData",
    "GGAData",
    "RMCData",
    "UniheadingData",
    "NtripClient",
    "parse_gga",
    "parse_rmc",
    "parse_uniheading",
    "determine_rtk_state",
]
