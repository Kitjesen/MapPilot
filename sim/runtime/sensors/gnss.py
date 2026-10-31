"""Experimental WGS84 GNSS measurements, separate from physics truth.

The local world is ENU at an explicitly declared ellipsoidal origin. This is
not a satellite constellation, RTK, multipath, or indoor-reception simulator.
"""

from __future__ import annotations

import math
import random
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields, replace
from typing import Any

from runtime.msgs.gnss import GnssFix, GnssFixType

from .contracts import ScheduledSensorSample
from .extractors import SensorSampleError, _validate_snapshot_identity, _vector
from .samples import SensorSampleStamp


@dataclass(frozen=True, slots=True)
class GnssConfig:
    """Map georeference; altitude is above the WGS84 ellipsoid, not MSL."""

    origin_latitude_deg: float
    origin_longitude_deg: float
    origin_altitude_m: float
    body_frame_id: str
    antenna_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_origin_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_to_enu_yaw_deg: float = 0.0
    noise_enabled: bool = False
    horizontal_std_m: float = 0.0
    vertical_std_m: float = 0.0
    seed: int = 0
    fix_available: bool = False

    def __post_init__(self) -> None:
        for name in (
            "origin_latitude_deg",
            "origin_longitude_deg",
            "origin_altitude_m",
            "world_to_enu_yaw_deg",
            "horizontal_std_m",
            "vertical_std_m",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"gnss.{name} must be finite")
        if not -90 <= self.origin_latitude_deg <= 90:
            raise ValueError("gnss.origin_latitude_deg must be in [-90, 90]")
        if not -180 <= self.origin_longitude_deg <= 180:
            raise ValueError("gnss.origin_longitude_deg must be in [-180, 180]")
        if (
            not isinstance(self.body_frame_id, str)
            or not self.body_frame_id
            or self.body_frame_id != self.body_frame_id.strip()
        ):
            raise ValueError("gnss.body_frame_id must be a non-empty trimmed string")
        for name in ("antenna_offset_m", "world_origin_m"):
            object.__setattr__(self, name, _vector(getattr(self, name), 3, f"gnss.{name}"))
        if type(self.noise_enabled) is not bool:
            raise ValueError("gnss.noise_enabled must be a boolean")
        if type(self.fix_available) is not bool:
            raise ValueError("gnss.fix_available must be a boolean")
        if self.horizontal_std_m < 0 or self.vertical_std_m < 0:
            raise ValueError("gnss noise standard deviations must be non-negative")
        if type(self.seed) is not int or not 0 <= self.seed < 1 << 64:
            raise ValueError("gnss.seed must be a uint64 integer")

    @classmethod
    def from_mapping(cls, value: Any) -> GnssConfig:
        """Require a declared map origin; reject unsupported or misspelled knobs."""
        allowed = {item.name for item in fields(cls)}
        required = {"origin_latitude_deg", "origin_longitude_deg", "origin_altitude_m", "body_frame_id"}
        if not isinstance(value, Mapping) or set(value) - allowed or required - set(value):
            raise ValueError("gnss requires explicit origin/body_frame_id and only supported configuration fields")
        return cls(**value)


@dataclass(frozen=True, slots=True)
class GnssSample:
    """Same-generation envelope around LingTu's existing GNSS message."""

    stamp: SensorSampleStamp
    fix: GnssFix
    configuration: GnssConfig

    def fidelity(self) -> dict[str, Any]:
        """Disclose the measurement model rather than claiming a calibrated receiver."""
        return {
            "qualification": "EXPERIMENTAL",
            "measurement_model": "wgs84_enu_gaussian",
            "fix_type": int(self.fix.fix_type),
            "configuration": asdict(self.configuration),
        }


def _enu_to_lla(enu: tuple[float, ...], config: GnssConfig) -> tuple[float, float, float]:
    """Transform ENU via ECEF, using the WGS84 ellipsoid (metres/degrees)."""

    a = 6378137.0
    e2 = (1 / 298.257223563) * (2 - 1 / 298.257223563)
    lat0, lon0 = map(math.radians, (config.origin_latitude_deg, config.origin_longitude_deg))
    slat, clat, slon, clon = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    n0 = a / math.sqrt(1 - e2 * slat * slat)
    east, north, up = enu
    x = (n0 + config.origin_altitude_m) * clat * clon - slon * east - slat * clon * north + clat * clon * up
    y = (n0 + config.origin_altitude_m) * clat * slon + clon * east - slat * slon * north + clat * slon * up
    z = (n0 * (1 - e2) + config.origin_altitude_m) * slat + clat * north + slat * up
    p = math.hypot(x, y)
    if p < 1e-8:
        return math.copysign(90.0, z), config.origin_longitude_deg, abs(z) - a * math.sqrt(1 - e2)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(12):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        lat = math.atan2(z + e2 * n * math.sin(lat), p)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    altitude = p * math.cos(lat) + z * math.sin(lat) - n * (1 - e2 * math.sin(lat) ** 2)
    return math.degrees(lat), math.degrees(math.atan2(y, x)), altitude


def _fix_covariance(lat_deg: float, lon_deg: float, config: GnssConfig) -> tuple[float, ...]:
    """Rotate origin-ENU noise covariance into ENU at the reported fix."""
    if not config.noise_enabled:
        return (0.0,) * 9
    lat, lat0 = math.radians(lat_deg), math.radians(config.origin_latitude_deg)
    delta_lon = math.radians(lon_deg - config.origin_longitude_deg)
    origin_up = (
        -math.cos(lat0) * math.sin(delta_lon),
        math.sin(lat0) * math.cos(lat) - math.cos(lat0) * math.sin(lat) * math.cos(delta_lon),
        math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * math.cos(delta_lon),
    )
    horizontal_var = config.horizontal_std_m**2
    extra_vertical_var = config.vertical_std_m**2 - horizontal_var
    return tuple(
        (horizontal_var if row == col else 0.0) + extra_vertical_var * origin_up[row] * origin_up[col]
        for row in range(3)
        for col in range(3)
    )


class GnssExtractor:
    """Read identity-matched physics snapshots without modifying them."""

    def __init__(self, config: GnssConfig) -> None:
        self.config = config

    def set_noise_enabled(self, enabled: bool) -> None:
        """Change only subsequent measurements; call on the sensor owner thread."""
        self.config = replace(self.config, noise_enabled=enabled)

    def set_fix_available(self, available: bool) -> None:
        """Apply an explicit scenario outage/recovery on the sensor owner thread."""
        self.config = replace(self.config, fix_available=available)

    def __call__(self, scheduled: ScheduledSensorSample, snapshot: Mapping[str, Any]) -> GnssSample:
        if scheduled.stream.stream_kind != "gnss":
            raise SensorSampleError("scheduled stream is not gnss")
        _validate_snapshot_identity(scheduled, snapshot)
        bodies = snapshot.get("bodies")
        if not isinstance(bodies, list):
            raise SensorSampleError("snapshot.bodies must be a list")
        matches = [
            body for body in bodies if isinstance(body, Mapping) and body.get("stable_id") == self.config.body_frame_id
        ]
        if len(matches) != 1:
            raise SensorSampleError("GNSS requires exactly one configured source body")
        stamp = SensorSampleStamp.from_scheduled(scheduled)
        if not self.config.fix_available:
            return GnssSample(
                stamp,
                GnssFix(
                    lat=math.nan,
                    lon=math.nan,
                    alt=math.nan,
                    fix_type=GnssFixType.NO_FIX,
                    covariance=(math.nan,) * 9,
                    num_sat=0,
                    num_sat_used=0,
                    seq=stamp.sequence,
                    ts=stamp.sim_time_ns / 1_000_000_000,
                    frame_id=stamp.frame_id,
                ),
                self.config,
            )
        position = _vector(matches[0].get("position_m"), 3, "body.position_m")
        w, x, y, z = _vector(matches[0].get("quaternion_wxyz"), 4, "body.quaternion_wxyz")
        norm = math.hypot(w, x, y, z)
        if norm < 1e-12:
            raise SensorSampleError("GNSS source body requires a non-zero quaternion")
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        vx, vy, vz = self.config.antenna_offset_m
        tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
        offset = (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)
        world = tuple(p + arm - origin for p, arm, origin in zip(position, offset, self.config.world_origin_m))
        yaw = math.radians(self.config.world_to_enu_yaw_deg)
        enu: tuple[float, ...] = (
            math.cos(yaw) * world[0] - math.sin(yaw) * world[1],
            math.sin(yaw) * world[0] + math.cos(yaw) * world[1],
            world[2],
        )
        horizontal = self.config.horizontal_std_m if self.config.noise_enabled else 0.0
        vertical = self.config.vertical_std_m if self.config.noise_enabled else 0.0
        if self.config.noise_enabled:
            # Stamp-keyed draws replay across drops, resets and endpoint restarts.
            rng = random.Random(f"{self.config.seed}:{stamp!r}")
            enu = tuple(value + rng.gauss(0.0, std) for value, std in zip(enu, (horizontal, horizontal, vertical)))
        lat, lon, alt = _enu_to_lla(enu, self.config)
        return GnssSample(
            stamp,
            GnssFix(
                lat=lat,
                lon=lon,
                alt=alt,
                fix_type=GnssFixType.SIMULATION,
                covariance=_fix_covariance(lat, lon, self.config),
                num_sat=0,
                num_sat_used=0,
                seq=stamp.sequence,
                ts=stamp.sim_time_ns / 1_000_000_000,
                frame_id=stamp.frame_id,
            ),
            self.config,
        )
