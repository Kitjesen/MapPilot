"""GnssModule â€?bridges WTRTK-980 GNSS into the LingTu Module pipeline.

Architecture
------------
    WTRTK-980 (UART)
         â†?
    GnssSerialDriver (pyserial + NMEA, default product path)
         â†?
    direct NMEA fixes, or /gps/fix DDS compatibility samples
         â†?
    GnssModule (this file)
      â”œâ”€ serial or DDS fix ingest
      â”œâ”€ LLA â†?local ENU conversion (against map origin)
      â”œâ”€ Quality filter (fix_type, sat count, age)
      â””â”€ publish {gnss_fix, gnss_status, gnss_odom}
         â†?
    SLAM backend (fastlio2_gnss) â€?uses gnss_odom as global PGO factor

Runs gracefully when pyserial/cyclonedds is absent (stub mode for Windows dev
and unit tests). Real robot products default to direct UART; DDS ``/gps/fix``
remains a compatibility fallback.

Usage::

    from runtime.blueprint import autoconnect
    from localization.gnss_module import GnssModule

    bp = Blueprint()
    bp.add(GnssModule, device_model="WTRTK-980")
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

from runtime.module import Module
from runtime.msgs.gnss import GnssFix, GnssFixType, GnssOdom, GnssStatus
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# WGS84 ellipsoid constants â€?used for LLAâ†’ENU conversion
_WGS84_A = 6378137.0          # semi-major axis (m)
_WGS84_F = 1.0 / 298.257223563  # flattening
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)  # first eccentricity squared


# ---------------------------------------------------------------------------
# WGS84 â†?ENU math (self-contained, no pyproj dependency)
# ---------------------------------------------------------------------------


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """Convert WGS84 (lat, lon, alt) to ECEF (x, y, z) metres."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    x = (N + alt_m) * cos_lat * math.cos(lon)
    y = (N + alt_m) * cos_lat * math.sin(lon)
    z = (N * (1.0 - _WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float, y: float, z: float,
    ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float,
) -> tuple[float, float, float]:
    """Convert ECEF point to local ENU tangent plane at reference origin."""
    ref_x, ref_y, ref_z = lla_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def lla_to_enu(
    lat: float, lon: float, alt: float,
    ref_lat: float, ref_lon: float, ref_alt: float,
) -> tuple[float, float, float]:
    """Direct LLA â†?ENU conversion against reference origin."""
    x, y, z = lla_to_ecef(lat, lon, alt)
    return ecef_to_enu(x, y, z, ref_lat, ref_lon, ref_alt)


# ---------------------------------------------------------------------------
# Quality filter
# ---------------------------------------------------------------------------


@dataclass
class QualityConfig:
    """GNSS quality gate parameters â€?all fixes below these thresholds
    are down-weighted or dropped before reaching the fusion backend."""

    min_sat_used: int = 8
    max_hdop: float = 2.5
    max_age_s: float = 2.0
    require_fix_type: int = 1     # minimum accepted fix type (1=SINGLE, 4=RTK_FIX)
    allow_float: bool = True


def fix_weight(fix: GnssFix, cfg: QualityConfig) -> float:
    """Return a weight in [0, 1] for how much to trust this fix.

    Used by downstream PGO backend to scale the GNSS factor covariance.
    0.0 means "drop this fix".
    """
    if int(fix.fix_type) < cfg.require_fix_type:
        return 0.0
    if fix.num_sat_used < cfg.min_sat_used:
        return 0.0
    if fix.hdop_estimate > cfg.max_hdop and fix.hdop_estimate > 0:
        return 0.0

    # Fix-type-based weighting
    if fix.fix_type == GnssFixType.RTK_FIXED:
        return 1.0
    if fix.fix_type == GnssFixType.RTK_FLOAT:
        return 0.3 if cfg.allow_float else 0.0
    if fix.fix_type == GnssFixType.DGPS:
        return 0.1
    if fix.fix_type == GnssFixType.SINGLE:
        return 0.05
    return 0.0


# ---------------------------------------------------------------------------
# Origin management
# ---------------------------------------------------------------------------


@dataclass
class MapOrigin:
    """Reference origin for LLAâ†’ENU conversion.

    If ``auto_init`` is True, the first valid fix becomes the origin
    and is logged for future sessions. Should then be pinned into
    ``config/robot_config.yaml`` for map consistency.
    """

    lat: float | None = None
    lon: float | None = None
    alt: float | None = None
    auto_init: bool = True

    @property
    def is_initialised(self) -> bool:
        return (
            self.lat is not None
            and self.lon is not None
            and self.alt is not None
        )

    def initialise(self, lat: float, lon: float, alt: float) -> None:
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def as_tuple(self) -> tuple[float, float, float]:
        if not self.is_initialised:
            raise RuntimeError("MapOrigin not initialised yet")
        return (self.lat, self.lon, self.alt)  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# DDS IDL types (lazy load â€?only if cyclonedds is available)
# ---------------------------------------------------------------------------


def _try_load_dds_types():
    """Return (NavSatFix, Odometry) DDS IDL structs if available, else (None, None)."""
    try:
        from dataclasses import dataclass as dds_dataclass

        from cyclonedds.idl import IdlStruct, types

        @dds_dataclass
        class DDS_Time(IdlStruct):
            sec: types.int32
            nanosec: types.uint32

        @dds_dataclass
        class DDS_Header(IdlStruct):
            stamp: DDS_Time
            frame_id: str

        @dds_dataclass
        class DDS_NavSatStatus(IdlStruct):
            status: types.int8
            service: types.uint16

        @dds_dataclass
        class DDS_NavSatFix(IdlStruct, typename="sensor_msgs::msg::dds_::NavSatFix_"):
            header: DDS_Header
            status: DDS_NavSatStatus
            latitude: types.float64
            longitude: types.float64
            altitude: types.float64
            position_covariance: types.array[types.float64, 9]
            position_covariance_type: types.uint8

        return DDS_NavSatFix
    except ImportError:
        return None


# ---------------------------------------------------------------------------
# GnssModule
# ---------------------------------------------------------------------------


@register("gnss", "wtrtk980", description="WTRTK-980 GNSS bridge (UM980 chipset)")
class GnssModule(Module, layer=1):
    """Bridges GNSS fixes into the Module pipeline.

    Real robot products read the configured UART through ``GnssSerialDriver``.
    DDS ``/gps/fix`` subscription is retained as a ROS2 compatibility fallback.

    Windows / offline: starts in stub mode (no DDS, no data published).
    Unit tests inject fixes directly via :meth:`inject_fix`.
    """

    # â”€â”€ outputs â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    gnss_fix:    Out[GnssFix]
    gnss_status: Out[GnssStatus]
    gnss_odom:   Out[GnssOdom]
    alive:       Out[bool]

    # â”€â”€ inputs â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    # RTCM corrections from NtripClientModule â€?auto-wired, forwarded to
    # the serial driver so the receiver can reach RTK_FIXED.
    rtcm_bytes:  In[bytes]

    def __init__(
        self,
        device_model: str = "WTRTK-980",
        fix_topic: str = "/gps/fix",
        antenna_frame: str = "gnss_antenna",
        map_frame: str = "map",
        origin_lat: float | None = None,
        origin_lon: float | None = None,
        origin_alt: float | None = None,
        auto_init_origin: bool = True,
        min_sat_used: int = 8,
        max_hdop: float = 2.5,
        max_age_s: float = 2.0,
        require_fix_type: int = 1,
        allow_float: bool = True,
        status_rate_hz: float = 2.0,
        serial_port: str | None = None,
        serial_baud: int = 115200,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._device_model = device_model
        self._fix_topic = fix_topic
        self._antenna_frame = antenna_frame
        self._map_frame = map_frame
        self._serial_port = serial_port
        self._serial_baud = serial_baud

        self._origin = MapOrigin(
            lat=origin_lat, lon=origin_lon, alt=origin_alt,
            auto_init=auto_init_origin,
        )
        self._quality = QualityConfig(
            min_sat_used=min_sat_used,
            max_hdop=max_hdop,
            max_age_s=max_age_s,
            require_fix_type=require_fix_type,
            allow_float=allow_float,
        )

        # runtime state
        self._seq = 0
        self._last_fix_ts = 0.0
        self._link_ok = False
        self._running = False
        self._dds_reader = None
        self._serial_driver = None
        self._status_thread: threading.Thread | None = None
        self._status_rate_hz = status_rate_hz
        self._shutdown_event = threading.Event()

        # most recent fix (for status publishing)
        self._last_fix: GnssFix | None = None

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        """Try serial driver first (if configured), then DDS, then stub.

        Serial is preferred because it doesn't need a separate ROS2
        GNSS driver service â€?just pyserial reading the UART directly.
        """
        # RTCM forwarding: subscribe regardless of transport â€?when NTRIP
        # module publishes, we forward bytes to the serial driver.
        self.rtcm_bytes.subscribe(self._on_rtcm_bytes)

        if self._serial_port:
            if self._try_start_serial():
                return
        if self._try_start_dds():
            return
        logger.info("GnssModule: no serial and no DDS â€?stub mode")

    def _on_rtcm_bytes(self, data: bytes) -> None:
        """Forward RTCM corrections to the receiver via serial.

        If there is no serial driver (DDS mode), the ironoa driver is
        expected to handle RTCM injection via its own NTRIP client; we
        silently no-op here.
        """
        if not data or self._serial_driver is None:
            return
        try:
            self._serial_driver.write_rtcm(data)
        except Exception as e:
            logger.debug("GnssModule: RTCM forward failed: %s", e)

    def _try_start_serial(self) -> bool:
        """Start direct serial NMEA reader as DDS fallback."""
        try:
            from localization.gnss_serial_driver import GnssSerialDriver
            driver = GnssSerialDriver(
                port=self._serial_port,
                baud=self._serial_baud,
                callback=self.inject_fix,
            )
            if driver.start():
                self._serial_driver = driver
                logger.info(
                    "GnssModule: serial driver started on %s @ %d baud",
                    self._serial_port, self._serial_baud,
                )
                return True
            return False
        except Exception as e:
            logger.warning("GnssModule: serial driver failed: %s", e)
            return False

    def _try_start_dds(self) -> bool:
        """Start DDS subscription if cyclonedds available."""
        fix_type = _try_load_dds_types()
        if fix_type is None:
            logger.info(
                "GnssModule: cyclonedds not available â€?running in stub mode "
                "(inject fixes via inject_fix for testing)"
            )
            return False
        try:
            from runtime.dds import DDSReader  # reuse existing reader

            reader = DDSReader(domain_id=0)
            reader.subscribe(self._fix_topic, fix_type, self._on_dds_fix)
            if not reader.start():
                logger.warning("GnssModule: DDSReader failed to start")
                return False
            reader.spin_background()
            self._dds_reader = reader
            logger.info(
                "GnssModule: subscribed to %s via DDS (receiver=%s)",
                self._fix_topic, self._device_model,
            )
            return True
        except Exception as e:
            logger.warning("GnssModule: DDS setup failed: %s", e)
            return False

    def start(self) -> None:
        super().start()
        self._running = True
        self.alive.publish(True)
        # periodic status publisher â€?even without DDS, reports link_ok=False
        self._shutdown_event.clear()
        self._status_thread = threading.Thread(
            target=self._status_loop, daemon=True, name="gnss-status",
        )
        self._status_thread.start()

    def stop(self) -> None:
        self._running = False
        self._shutdown_event.set()
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=2.0)
        self._status_thread = None
        if self._dds_reader is not None:
            try:
                self._dds_reader.stop()
            except Exception:
                pass
            self._dds_reader = None
        if self._serial_driver is not None:
            try:
                self._serial_driver.stop()
            except Exception:
                pass
            self._serial_driver = None
        super().stop()

    # -- DDS callback -------------------------------------------------------

    def _on_dds_fix(self, msg: Any) -> None:
        """Parse a NavSatFix DDS sample and forward to the pipeline."""
        try:
            # Convert NavSatStatus.status (-1=no fix, 0=SINGLE, 1=DGPS, 2=RTK)
            # into our GnssFixType. The ironoa driver maps Unicore PVTSLN into
            # these values â€?see its README for exact mapping.
            status_int = int(getattr(msg.status, "status", -1))
            if status_int < 0:
                fix_type = GnssFixType.NO_FIX
            elif status_int == 0:
                fix_type = GnssFixType.SINGLE
            elif status_int == 1:
                fix_type = GnssFixType.DGPS
            elif status_int == 2:
                fix_type = GnssFixType.RTK_FIXED
            else:
                fix_type = GnssFixType.RTK_FLOAT

            ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if ts <= 0:
                ts = time.time()

            cov_tuple = tuple(float(c) for c in msg.position_covariance)
            if len(cov_tuple) != 9:
                cov_tuple = (0.0,) * 9

            fix = GnssFix(
                lat=float(msg.latitude),
                lon=float(msg.longitude),
                alt=float(msg.altitude),
                fix_type=fix_type,
                covariance=cov_tuple,
                num_sat=0,             # not in standard NavSatFix
                num_sat_used=0,
                seq=self._seq,
                ts=ts,
                frame_id=str(msg.header.frame_id) or self._antenna_frame,
            )
            self.inject_fix(fix)
        except Exception as e:
            logger.debug("GnssModule: DDS parse error: %s", e)

    # -- Public injection API (for tests + simulator) ----------------------

    def inject_fix(self, fix: GnssFix) -> None:
        """Inject a fix from any source (DDS, simulator, unit test).

        Performs quality filtering, publishes gnss_fix, and if the fix
        passes quality gate, computes ENU odometry and publishes gnss_odom.
        """
        self._seq += 1
        self._last_fix_ts = fix.ts
        self._link_ok = True
        self._last_fix = fix

        weight = fix_weight(fix, self._quality)
        self.gnss_fix.publish(fix)

        if weight <= 0.0:
            # Fix too poor â€?publish status but no odom
            return

        # Auto-initialise origin from first good fix
        if not self._origin.is_initialised:
            if not self._origin.auto_init:
                logger.warning(
                    "GnssModule: fix received but no origin set and auto_init=False"
                )
                return
            self._origin.initialise(fix.lat, fix.lon, fix.alt)
            logger.info(
                "GnssModule: map origin auto-initialised to "
                "(%.7f, %.7f, %.2f). Pin this in robot_config.yaml.",
                fix.lat, fix.lon, fix.alt,
            )

        # LLA â†?ENU
        e, n, u = lla_to_enu(
            fix.lat, fix.lon, fix.alt, *self._origin.as_tuple()
        )

        # Covariance: extract diagonal, scale by inverse of weight
        cov_e = fix.covariance[0] / max(weight, 1e-3)
        cov_n = fix.covariance[4] / max(weight, 1e-3)
        cov_u = fix.covariance[8] / max(weight, 1e-3)

        odom = GnssOdom(
            east=e, north=n, up=u,
            ve=0.0, vn=0.0, vu=0.0,  # WTRTK-980 velocity comes from separate topic
            cov_e=cov_e, cov_n=cov_n, cov_u=cov_u,
            fix_type=fix.fix_type,
            ts=fix.ts,
            frame_id=self._map_frame,
        )
        self.gnss_odom.publish(odom)

    # -- Status loop --------------------------------------------------------

    def _status_loop(self) -> None:
        """Periodically publish GnssStatus for dashboard / watchdog."""
        interval = 1.0 / max(self._status_rate_hz, 0.1)
        while not self._shutdown_event.is_set():
            self._shutdown_event.wait(timeout=interval)
            if not self._running:
                continue
            now = time.time()
            age = now - self._last_fix_ts if self._last_fix_ts > 0 else 99.0
            if age > self._quality.max_age_s:
                self._link_ok = False

            last = self._last_fix
            status = GnssStatus(
                fix_type=last.fix_type if last else GnssFixType.NO_FIX,
                num_sat=last.num_sat if last else 0,
                num_sat_used=last.num_sat_used if last else 0,
                hdop=last.hdop_estimate if last else 99.9,
                age_s=age,
                rtcm_age_s=99.9,  # TODO: pipe from driver when available
                receiver=self._device_model,
                link_ok=self._link_ok,
                ts=now,
            )
            self.gnss_status.publish(status)

    # -- health reporting ---------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["device_model"] = self._device_model
        info["link_ok"] = self._link_ok
        info["origin_initialised"] = self._origin.is_initialised
        if self._origin.is_initialised:
            info["origin"] = {
                "lat": self._origin.lat,
                "lon": self._origin.lon,
                "alt": self._origin.alt,
            }
        if self._last_fix:
            info["last_fix"] = self._last_fix.to_dict()
        return info
