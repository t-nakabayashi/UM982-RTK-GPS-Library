"""
UM982 RTK GPS データ型定義
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class GGAData:
    """NMEA GGA文のパース結果"""
    lat: Optional[float]          # 緯度（10進数度）
    lon: Optional[float]          # 経度（10進数度）
    alt: Optional[float]          # 高度（メートル）
    quality: Optional[int]        # GPS品質（1=単独, 2=DGPS, 4=RTK Fix, 5=RTK Float）
    num_sats: Optional[int]       # 使用衛星数
    hdop: Optional[float]         # 水平精度低下率
    diff_age: Optional[float]     # 差分補正データの経過時間（秒）
    ref_station_id: Optional[str] # 基準局ID
    timestamp: float              # 受信時刻（Unix時間）
    raw: str                      # 生のNMEA文


@dataclass
class RMCData:
    """NMEA RMC文のパース結果"""
    speed_knots: Optional[float]  # 対地速度（ノット）
    course_deg: Optional[float]   # 進行方位（度）
    mode: Optional[str]           # モード指標（R=RTK Fix, F=RTK Float, D=DGPS, A=単独）
    timestamp: float              # 受信時刻（Unix時間）


@dataclass
class UniheadingData:
    """Unicore UNIHEADING文のパース結果（デュアルアンテナ方位）"""
    length_m: Optional[float]       # ベースライン長（メートル）
    heading_deg: Optional[float]    # 方位（度、真北基準）
    pitch_deg: Optional[float]      # ピッチ角（度）
    heading_stddev: Optional[float] # 方位標準偏差
    pitch_stddev: Optional[float]   # ピッチ標準偏差
    stn_id: Optional[str]           # ステーションID
    num_svs: Optional[int]          # 追跡衛星数
    num_soln_svs: Optional[int]     # 解に使用した衛星数
    num_obs: Optional[int]          # 観測数
    num_multi: Optional[int]        # マルチ周波数観測数
    timestamp: float                # 受信時刻（Unix時間）


@dataclass
class PositionData:
    """統合された位置データ"""
    lat: Optional[float]            # 緯度（10進数度）
    lon: Optional[float]            # 経度（10進数度）
    alt: Optional[float]            # 高度（メートル）
    heading: Optional[float]        # 方位（度、デュアルアンテナ）
    pitch: Optional[float]          # ピッチ角（度）
    speed_knots: Optional[float]    # 対地速度（ノット）
    course: Optional[float]         # 進行方位（度）
    rtk_state: str                  # RTK状態
    num_sats: Optional[int]         # 使用衛星数
    hdop: Optional[float]           # HDOP
    baseline_m: Optional[float]     # ベースライン長（メートル）
    timestamp: float                # タイムスタンプ

    @property
    def speed_mps(self) -> Optional[float]:
        """対地速度をm/sで取得"""
        if self.speed_knots is None:
            return None
        return self.speed_knots * 0.514444

    @property
    def speed_kmh(self) -> Optional[float]:
        """対地速度をkm/hで取得"""
        if self.speed_knots is None:
            return None
        return self.speed_knots * 1.852

    @property
    def is_rtk_fix(self) -> bool:
        """RTK Fixかどうか"""
        return self.rtk_state == "rtk_fix"

    @property
    def is_valid(self) -> bool:
        """有効な位置データかどうか"""
        return self.lat is not None and self.lon is not None
