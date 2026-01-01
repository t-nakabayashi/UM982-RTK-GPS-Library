"""
NTRIPクライアント
"""

import base64
import socket
import sys
import threading
import time
from typing import Callable, Optional

__version__ = "1.0.0"

DEFAULT_GGA_INTERVAL = 5.0


class NtripClient(threading.Thread):
    """NTRIPクライアント（RTCM補正データ受信）"""

    def __init__(
        self,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
        on_rtcm_data: Callable[[bytes], None],
        get_gga: Optional[Callable[[], Optional[str]]] = None,
        gga_interval: float = DEFAULT_GGA_INTERVAL,
        on_rtcm_count: Optional[Callable[[int], None]] = None,
    ):
        """
        Args:
            host: NTRIPキャスターのホスト名/IP
            port: ポート番号
            mountpoint: マウントポイント名
            user: ユーザ名
            password: パスワード
            on_rtcm_data: RTCMデータ受信時のコールバック（bytes -> None）
            get_gga: 最新のGGA文を取得するコールバック（省略可）
            gga_interval: GGA送信間隔（秒）
            on_rtcm_count: RTCMバイト数更新時のコールバック（省略可）
        """
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        self.on_rtcm_data = on_rtcm_data
        self.get_gga = get_gga
        self.gga_interval = gga_interval
        self.on_rtcm_count = on_rtcm_count
        self._stop = threading.Event()
        self._connected = False
        self._total_bytes = 0

    @property
    def is_connected(self) -> bool:
        """接続中かどうか"""
        return self._connected

    @property
    def total_bytes(self) -> int:
        """受信した総RTCMバイト数"""
        return self._total_bytes

    def stop(self):
        """クライアントを停止"""
        self._stop.set()

    def run(self):
        backoff = 1
        while not self._stop.is_set():
            try:
                self._run_once()
                backoff = 1
            except Exception as exc:
                self._connected = False
                print(f"[NTRIP] Error: {exc}, reconnecting in {backoff}s...", file=sys.stderr)
                time.sleep(backoff)
                backoff = min(backoff * 2, 60)

    def _run_once(self):
        """NTRIP接続とRTCMデータ転送"""
        auth = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()

        # NTRIP 1.0形式（広い互換性）
        request = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP um982-rtk-client/{__version__}\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        ).encode()

        sock = socket.create_connection((self.host, self.port), timeout=10)
        try:
            sock.sendall(request)
            response = sock.recv(1024)

            if b"200" not in response and b"ICY" not in response:
                raise RuntimeError(f"NTRIP caster rejected: {response[:100].decode(errors='ignore')}")

            print(f"[NTRIP] Connected to {self.host}:{self.port}/{self.mountpoint}", file=sys.stderr)
            self._connected = True
            sock.settimeout(2.0)
            last_gga = 0.0

            while not self._stop.is_set():
                # 定期的にGGAを送信（位置報告）
                now = time.time()
                if self.get_gga and self.gga_interval > 0 and now - last_gga >= self.gga_interval:
                    gga = self.get_gga()
                    if gga:
                        try:
                            sock.sendall((gga + "\r\n").encode("ascii", errors="ignore"))
                            last_gga = now
                        except Exception:
                            pass

                # RTCMデータ受信・転送
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue

                if not data:
                    raise RuntimeError("Connection closed by server")

                self._total_bytes += len(data)
                if self.on_rtcm_count:
                    self.on_rtcm_count(len(data))
                self.on_rtcm_data(data)
        finally:
            self._connected = False
            sock.close()
