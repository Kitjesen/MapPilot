"""lingtu.runtime.msgs.gnss — GNSS message types.

Classes
-------
GnssFixType    — RTK fix quality enum (matches NMEA GGA field 6 + Unicore extensions)
GnssFix        — WGS84 position + covariance + fix quality (like sensor_msgs/NavSatFix)
GnssStatus     — constellation / satellite count / HDOP / link quality (diagnostic)
GnssOdom       — ENU-frame position + velocity for native global constraints

Coordinate convention
---------------------
GnssFix uses WGS84 (lat/lon/alt — degrees / metres).
GnssOdom uses local ENU (East-North-Up) from a configured map origin.

Fix quality mapping (WTRTK-980 / UM980 / NMEA GGA):
    0  NO_FIX         — no signal / invalid
    1  SINGLE         — standalone pseudorange (~3 m accuracy)
    2  DGPS           — differential GPS
    4  RTK_FIXED      — RTK fix (0.8 cm + 1 ppm)
    5  RTK_FLOAT      — RTK float (~10 cm)
    6  ESTIMATED      — dead reckoning
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, ClassVar

from runtime.runtime_interface import map_frame_id

from .protocol import json_message_decode, json_message_encode
from .sensor import Header

GNSS_MAP_FRAME_ID = map_frame_id()
_NAVSAT_STATUS_NO_FIX = -1
_NAVSAT_STATUS_FIX = 0
_NAVSAT_STATUS_GBAS_FIX = 2
_NAVSAT_SERVICE_GPS = 1
_NAVSAT_COVARIANCE_TYPE_UNKNOWN = 0
_NAVSAT_COVARIANCE_TYPE_KNOWN = 3

# ---------------------------------------------------------------------------
# GnssFixType
# ---------------------------------------------------------------------------


class GnssFixType(IntEnum):
    """Fix quality — matches NMEA GGA field 6 and UM980 Unicore extension."""

    NO_FIX = 0
    SINGLE = 1
    DGPS = 2
    PPS = 3
    RTK_FIXED = 4
    RTK_FLOAT = 5
    ESTIMATED = 6
    MANUAL = 7
    SIMULATION = 8

    @property
    def is_rtk(self) -> bool:
        """True for RTK fixed or float solutions."""
        return self in (GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT)

    @property
    def is_usable(self) -> bool:
        """True if the fix has any meaningful position information."""
        return self not in (GnssFixType.NO_FIX,)

    @property
    def typical_accuracy_m(self) -> float:
        """Typical horizontal accuracy for this fix type (metres)."""
        return {
            GnssFixType.NO_FIX: float("inf"),
            GnssFixType.SINGLE: 3.0,
            GnssFixType.DGPS: 1.0,
            GnssFixType.PPS: 2.0,
            GnssFixType.RTK_FIXED: 0.02,
            GnssFixType.RTK_FLOAT: 0.2,
            GnssFixType.ESTIMATED: 10.0,
            GnssFixType.MANUAL: 0.0,
            GnssFixType.SIMULATION: 0.01,
        }.get(self, 10.0)


# ---------------------------------------------------------------------------
# GnssFix
# ---------------------------------------------------------------------------

# 4 × float64 (lat, lon, alt, fix_type) + 9 × float64 (3×3 covariance) + ts(f64)
# + 4×u32 (fix_type as int, num_sat, num_sat_used, seq) + frame_id string
_FIX_FMT = struct.Struct("<3d9dd4I")  # 3+9+1+4 = 17 values, 3*8+9*8+8+4*4 = 112 bytes


@dataclass
class GnssFix:
    """WGS84 GNSS position fix with covariance.

    Similar to ROS2 ``sensor_msgs/NavSatFix`` but with extra Unicore-specific
    fields (fix_type, num_sat) packed directly.

    Parameters
    ----------
    lat, lon : float
        Latitude / longitude (degrees, WGS84).
    alt : float
        Altitude above WGS84 ellipsoid (metres).
    fix_type : GnssFixType
        Solution quality.
    covariance : tuple[float, ...]
        3x3 position covariance in ENU (m^2), row-major 9 floats.
        Order: [E-E, E-N, E-U, N-E, N-N, N-U, U-E, U-N, U-U]
    num_sat : int
        Total satellites tracked.
    num_sat_used : int
        Satellites used in the fix.
    seq : int
        Sequence number (monotonic counter, useful for dedup).
    ts : float
        Timestamp (seconds since epoch).
    frame_id : str
        TF frame — typically ``gnss_antenna``.
    """

    msg_name: ClassVar[str] = "lingtu_msgs.GnssFix"
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    fix_type: GnssFixType = GnssFixType.NO_FIX
    covariance: tuple[float, ...] = field(
        default_factory=lambda: (0.0,) * 9
    )
    num_sat: int = 0
    num_sat_used: int = 0
    seq: int = 0
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default="gnss_antenna")

    def __post_init__(self) -> None:
        if isinstance(self.fix_type, int) and not isinstance(self.fix_type, GnssFixType):
            self.fix_type = GnssFixType(self.fix_type)
        if len(self.covariance) != 9:
            raise ValueError(
                f"covariance must have 9 elements, got {len(self.covariance)}"
            )

    # -- convenience properties ---------------------------------------------

    @property
    def hdop_estimate(self) -> float:
        """Rough HDOP estimate from horizontal covariance (sqrt of diag sum)."""
        ee, _, _, _, nn, _, _, _, _ = self.covariance
        return float((ee + nn) ** 0.5)

    @property
    def horizontal_std_m(self) -> float:
        """1-sigma horizontal accuracy (metres)."""
        ee, _, _, _, nn, *_ = self.covariance
        return float(((ee + nn) / 2) ** 0.5)

    @property
    def vertical_std_m(self) -> float:
        """1-sigma vertical accuracy (metres)."""
        return float(self.covariance[8] ** 0.5)

    @property
    def is_rtk(self) -> bool:
        return self.fix_type.is_rtk

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    @property
    def navsat_status(self) -> int:
        if self.fix_type is GnssFixType.NO_FIX:
            return _NAVSAT_STATUS_NO_FIX
        if self.fix_type in (GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT):
            return _NAVSAT_STATUS_GBAS_FIX
        return _NAVSAT_STATUS_FIX

    # -- serialisation ------------------------------------------------------

    def encode(self) -> bytes:
        """Binary encode: fixed header + frame string."""
        frame_bytes = self.frame_id.encode()
        buf = bytearray()
        buf += _FIX_FMT.pack(
            self.lat, self.lon, self.alt,
            *self.covariance,
            self.ts,
            int(self.fix_type), self.num_sat, self.num_sat_used, self.seq,
        )
        buf += struct.pack("<I", len(frame_bytes)) + frame_bytes
        return bytes(buf)

    @classmethod
    def decode(cls, raw: bytes) -> GnssFix:
        vals = _FIX_FMT.unpack_from(raw, 0)
        lat, lon, alt = vals[0], vals[1], vals[2]
        cov = tuple(vals[3:12])
        ts = vals[12]
        fix_type_int, num_sat, num_sat_used, seq = vals[13], vals[14], vals[15], vals[16]
        off = _FIX_FMT.size
        frame_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        frame_id = raw[off : off + frame_len].decode()
        return cls(
            lat=lat, lon=lon, alt=alt,
            fix_type=GnssFixType(fix_type_int),
            covariance=cov,
            num_sat=num_sat, num_sat_used=num_sat_used, seq=seq,
            ts=ts, frame_id=frame_id,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "fix_type": self.fix_type.name,
            "fix_type_int": int(self.fix_type),
            "num_sat": self.num_sat,
            "num_sat_used": self.num_sat_used,
            "horizontal_std_m": self.horizontal_std_m,
            "vertical_std_m": self.vertical_std_m,
            "is_rtk": self.is_rtk,
            "seq": self.seq,
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "status": {
                "status": self.navsat_status,
                "service": _NAVSAT_SERVICE_GPS,
            },
            "latitude": float(self.lat),
            "longitude": float(self.lon),
            "altitude": float(self.alt),
            "position_covariance": list(self.covariance),
            "position_covariance_type": (
                _NAVSAT_COVARIANCE_TYPE_UNKNOWN
                if all(float(v) == 0.0 for v in self.covariance)
                else _NAVSAT_COVARIANCE_TYPE_KNOWN
            ),
        }

    def __repr__(self) -> str:
        return (
            f"GnssFix({self.fix_type.name}, "
            f"({self.lat:.7f}°, {self.lon:.7f}°, {self.alt:.2f}m), "
            f"sat={self.num_sat_used}/{self.num_sat}, "
            f"±{self.horizontal_std_m:.2f}m)"
        )


# ---------------------------------------------------------------------------
# GnssStatus — diagnostic / health
# ---------------------------------------------------------------------------


@dataclass
class GnssStatus:
    """GNSS receiver health and diagnostic state.

    Published periodically so Dashboard and watchdogs can surface link quality.
    """

    msg_name: ClassVar[str] = "lingtu_msgs.GnssStatus"
    fix_type: GnssFixType = GnssFixType.NO_FIX
    num_sat: int = 0
    num_sat_used: int = 0
    hdop: float = 99.9
    age_s: float = 0.0                  # seconds since last fix
    rtcm_age_s: float = 99.9            # seconds since last RTCM differential correction
    receiver: str = "WTRTK-980"         # hardware model string
    link_ok: bool = False               # UART / USB link alive
    ts: float = field(default_factory=time.time)

    @property
    def is_healthy(self) -> bool:
        """True if the receiver is producing a usable fix."""
        return (
            self.link_ok
            and self.fix_type.is_usable
            and self.num_sat_used >= 4
            and self.age_s < 2.0
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "fix_type": self.fix_type.name,
            "num_sat": self.num_sat,
            "num_sat_used": self.num_sat_used,
            "hdop": self.hdop,
            "age_s": self.age_s,
            "rtcm_age_s": self.rtcm_age_s,
            "receiver": self.receiver,
            "link_ok": self.link_ok,
            "is_healthy": self.is_healthy,
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> GnssStatus:
        return cls(
            fix_type=GnssFixType[d.get("fix_type", "NO_FIX")],
            num_sat=int(d.get("num_sat", 0)),
            num_sat_used=int(d.get("num_sat_used", 0)),
            hdop=float(d.get("hdop", 99.9)),
            age_s=float(d.get("age_s", 0)),
            rtcm_age_s=float(d.get("rtcm_age_s", 99.9)),
            receiver=str(d.get("receiver", "WTRTK-980")),
            link_ok=bool(d.get("link_ok", False)),
            ts=float(d.get("ts", 0)),
        )

    def __repr__(self) -> str:
        return (
            f"GnssStatus({self.fix_type.name}, "
            f"sat={self.num_sat_used}/{self.num_sat}, "
            f"hdop={self.hdop:.2f}, age={self.age_s:.2f}s, "
            f"link={'OK' if self.link_ok else 'DOWN'})"
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> GnssStatus:
        return json_message_decode(data, cls)


# ---------------------------------------------------------------------------
# GnssOdom — ENU-frame odometry ready for fusion
# ---------------------------------------------------------------------------

_ODOM_FMT = struct.Struct("<9d")  # 3 pos + 3 vel + 3 pos_cov_diag (ENU), 72 bytes


@dataclass
class GnssOdom:
    """GNSS position + velocity in local ENU frame for native global correction.

    Produced by the native GNSS service after LLA→ENU conversion against a
    configured map origin. Downstream SLAM backends ingest this as a global
    constraint.

    Parameters
    ----------
    east, north, up : float
        Position in local ENU frame (metres from map origin).
    ve, vn, vu : float
        Velocity in ENU frame (m/s).
    cov_e, cov_n, cov_u : float
        Position variance (m^2), diagonal only (3 floats).
    fix_type : GnssFixType
        Inherited from underlying fix.
    ts : float
        Timestamp (seconds since epoch).
    frame_id : str
        ENU map frame id.
    """

    msg_name: ClassVar[str] = "lingtu_msgs.GnssOdom"
    east: float = 0.0
    north: float = 0.0
    up: float = 0.0
    ve: float = 0.0
    vn: float = 0.0
    vu: float = 0.0
    cov_e: float = 99.0
    cov_n: float = 99.0
    cov_u: float = 99.0
    fix_type: GnssFixType = GnssFixType.NO_FIX
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=GNSS_MAP_FRAME_ID)

    def __post_init__(self) -> None:
        if isinstance(self.fix_type, int) and not isinstance(self.fix_type, GnssFixType):
            self.fix_type = GnssFixType(self.fix_type)

    @property
    def horizontal_std_m(self) -> float:
        return float(((self.cov_e + self.cov_n) / 2) ** 0.5)

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "east": self.east,
            "north": self.north,
            "up": self.up,
            "ve": self.ve,
            "vn": self.vn,
            "vu": self.vu,
            "horizontal_std_m": self.horizontal_std_m,
            "vertical_std_m": float(self.cov_u ** 0.5),
            "fix_type": self.fix_type.name,
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        pose_covariance = [0.0] * 36
        pose_covariance[0] = float(self.cov_e)
        pose_covariance[7] = float(self.cov_n)
        pose_covariance[14] = float(self.cov_u)
        return {
            "header": self.header.to_dict(),
            "child_frame_id": "gnss",
            "pose": {
                "pose": {
                    "position": {"x": self.east, "y": self.north, "z": self.up},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "covariance": pose_covariance,
            },
            "twist": {
                "twist": {
                    "linear": {"x": self.ve, "y": self.vn, "z": self.vu},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                "covariance": [0.0] * 36,
            },
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> GnssOdom:
        return cls(
            east=float(d.get("east", 0)),
            north=float(d.get("north", 0)),
            up=float(d.get("up", 0)),
            ve=float(d.get("ve", 0)),
            vn=float(d.get("vn", 0)),
            vu=float(d.get("vu", 0)),
            cov_e=float(d.get("cov_e", 99)),
            cov_n=float(d.get("cov_n", 99)),
            cov_u=float(d.get("cov_u", 99)),
            fix_type=GnssFixType[d.get("fix_type", "NO_FIX")],
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", GNSS_MAP_FRAME_ID)),
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> GnssOdom:
        return json_message_decode(data, cls)

    def __repr__(self) -> str:
        return (
            f"GnssOdom(E={self.east:.2f} N={self.north:.2f} U={self.up:.2f}, "
            f"v=({self.ve:.2f},{self.vn:.2f},{self.vu:.2f}), "
            f"{self.fix_type.name}, ±{self.horizontal_std_m:.3f}m)"
        )
