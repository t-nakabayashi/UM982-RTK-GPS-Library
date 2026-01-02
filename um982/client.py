"""
UM982 RTK GPSクライアント

使用例:
    from um982 import UM982Client

    client = UM982Client("/dev/ttyUSB0")
    client.start()

    # NTRIPで補正データを受信する場合
    client.start_ntrip(
        host="rtk.example.com",
        port=2101,
        mountpoint="RTCM3",
        user="user",
        password="pass"
    )

    while True:
        pos = client.get_position()
        if pos and pos.is_valid:
            print(f"Lat: {pos.lat}, Lon: {pos.lon}, Heading: {pos.heading}")
        time.sleep(0.1)
"""

import sys
import threading
import time
from typing import Callable, Optional

try:
    import serial
except ImportError:
    serial = None

from .nmea import parse_gga, parse_rmc, parse_uniheading, determine_rtk_state
from .ntrip import NtripClient
from .types import GGAData, RMCData, UniheadingData, PositionData

__version__ = "1.0.0"

DEFAULT_BAUD = 115200
DEFAULT_SERIAL_TIMEOUT = 0.5
DEFAULT_OUTPUT_RATE = 10  # Hz


class UM982Client:
    """UM982 RTK GPSクライアント"""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        timeout: float = DEFAULT_SERIAL_TIMEOUT,
        output_rate: int = DEFAULT_OUTPUT_RATE,
    ):
        """
        Args:
            port: シリアルポート（例: "/dev/ttyUSB0"）
            baud: ボーレート（デフォルト: 115200）
            timeout: シリアルタイムアウト（秒）
            output_rate: 出力レート（Hz、デフォルト: 10）
        """
        if serial is None:
            raise ImportError("pyserial is required. Install with: pip install pyserial")

        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.output_rate = output_rate

        self._ser: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._data_lock = threading.Lock()

        self._gga: Optional[GGAData] = None
        self._rmc: Optional[RMCData] = None
        self._uniheading: Optional[UniheadingData] = None
        self._rtcm_bytes = 0

        self._reader_thread: Optional[threading.Thread] = None
        self._ntrip_client: Optional[NtripClient] = None
        self._stop = threading.Event()

        self._on_position: Optional[Callable[[PositionData], None]] = None

    def open(self):
        """シリアルポートを開く"""
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self):
        """シリアルポートを閉じる"""
        if self._ser:
            self._ser.close()
            self._ser = None

    def start(self, configure: bool = True):
        """
        クライアントを開始（シリアル読み取りスレッド起動）

        Args:
            configure: 起動時に出力レートを設定するか（デフォルト: True）
        """
        if self._ser is None:
            self.open()
        self._stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        if configure:
            self.set_output_rate(self.output_rate)

    def stop(self):
        """クライアントを停止"""
        self._stop.set()
        if self._ntrip_client:
            self._ntrip_client.stop()
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        self.close()

    def start_ntrip(
        self,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        gga_interval: float = 5.0,
    ):
        """
        NTRIPクライアントを開始

        Args:
            host: NTRIPキャスターのホスト名/IP
            port: ポート番号
            mountpoint: マウントポイント名
            user: ユーザ名
            password: パスワード
            gga_interval: GGA送信間隔（秒）
        """
        self._ntrip_client = NtripClient(
            host=host,
            port=port,
            mountpoint=mountpoint,
            user=user,
            password=password,
            on_rtcm_data=self._write_bytes,
            get_gga=self._get_latest_gga_raw,
            gga_interval=gga_interval,
            on_rtcm_count=self._update_rtcm_count,
        )
        self._ntrip_client.start()

    def stop_ntrip(self):
        """NTRIPクライアントを停止"""
        if self._ntrip_client:
            self._ntrip_client.stop()
            self._ntrip_client = None

    def set_output_rate(self, rate: int, enable_rmc: bool = False):
        """
        出力レートを設定

        Args:
            rate: 出力レート（Hz、1-20推奨）
            enable_rmc: RMC出力を有効にするか
        """
        self.output_rate = rate
        # UM982のLOGコマンド形式: LOG <メッセージ> ONTIME <秒>
        interval = 1.0 / rate if rate > 0 else 1.0
        cmds = [
            f"LOG GPGGA ONTIME {interval}",
            f"LOG UNIHEADINGA ONTIME {interval}",
        ]
        if enable_rmc:
            cmds.append(f"LOG GPRMC ONTIME {interval}")

        for cmd in cmds:
            self._write_line(cmd)
            time.sleep(0.1)

    def get_position(self) -> Optional[PositionData]:
        """
        現在の位置データを取得

        Returns:
            PositionData: 位置データ（データがない場合はNone）
        """
        with self._data_lock:
            gga = self._gga
            rmc = self._rmc
            uni = self._uniheading

        if gga is None:
            return None

        rtk_state = determine_rtk_state(gga, rmc)

        return PositionData(
            lat=gga.lat,
            lon=gga.lon,
            alt=gga.alt,
            heading=uni.heading_deg if uni else None,
            pitch=uni.pitch_deg if uni else None,
            speed_knots=rmc.speed_knots if rmc else None,
            course=rmc.course_deg if rmc else None,
            rtk_state=rtk_state,
            num_sats=gga.num_sats,
            hdop=gga.hdop,
            baseline_m=uni.length_m if uni else None,
            timestamp=gga.timestamp,
            diff_age=gga.diff_age,
            heading_stddev=uni.heading_stddev if uni else None,
            pitch_stddev=uni.pitch_stddev if uni else None,
        )

    def get_gga(self) -> Optional[GGAData]:
        """最新のGGAデータを取得"""
        with self._data_lock:
            return self._gga

    def get_rmc(self) -> Optional[RMCData]:
        """最新のRMCデータを取得"""
        with self._data_lock:
            return self._rmc

    def get_uniheading(self) -> Optional[UniheadingData]:
        """最新のUNIHEADINGデータを取得"""
        with self._data_lock:
            return self._uniheading

    def get_rtcm_bytes(self) -> int:
        """受信したRTCMバイト数を取得"""
        with self._data_lock:
            return self._rtcm_bytes

    def set_position_callback(self, callback: Optional[Callable[[PositionData], None]]):
        """
        位置データ更新時のコールバックを設定

        Args:
            callback: コールバック関数（PositionData -> None）
        """
        self._on_position = callback

    def _reader_loop(self):
        """シリアル受信ループ"""
        while not self._stop.is_set():
            line = self._readline()
            if not line:
                continue

            gga = parse_gga(line)
            if gga:
                with self._data_lock:
                    self._gga = gga
                if self._on_position:
                    pos = self.get_position()
                    if pos:
                        self._on_position(pos)
                continue

            rmc = parse_rmc(line)
            if rmc:
                with self._data_lock:
                    self._rmc = rmc
                continue

            uni = parse_uniheading(line)
            if uni:
                with self._data_lock:
                    self._uniheading = uni

    def _readline(self) -> Optional[str]:
        """1行読み取り"""
        if not self._ser:
            return None
        line = self._ser.readline()
        if not line:
            return None
        try:
            return line.decode("ascii", errors="ignore").strip()
        except UnicodeDecodeError:
            return None

    def _write_line(self, line: str):
        """ASCII行を送信（CRLF付加）"""
        data = (line.strip() + "\r\n").encode("ascii", errors="ignore")
        self._write_bytes(data)

    def _write_bytes(self, data: bytes):
        """バイト列を送信（排他制御付き）"""
        if not self._ser:
            return
        with self._write_lock:
            self._ser.write(data)

    def _get_latest_gga_raw(self) -> Optional[str]:
        """最新のGGA生データを取得（NTRIP送信用にGPGGAに変換）"""
        with self._data_lock:
            if not self._gga:
                return None
            raw = self._gga.raw
            # $GNGGA → $GPGGA に変換（古いNTRIPサーバー互換性のため）
            if raw.startswith("$GNGGA"):
                raw = "$GPGGA" + raw[6:]
            return raw

    def _update_rtcm_count(self, delta: int):
        """RTCMバイト数を更新"""
        with self._data_lock:
            self._rtcm_bytes += delta

    @property
    def is_running(self) -> bool:
        """クライアントが動作中かどうか"""
        return self._reader_thread is not None and self._reader_thread.is_alive()

    @property
    def is_ntrip_connected(self) -> bool:
        """NTRIPが接続中かどうか"""
        return self._ntrip_client is not None and self._ntrip_client.is_connected
