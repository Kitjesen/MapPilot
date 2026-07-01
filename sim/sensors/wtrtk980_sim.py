"""WTRTK-980 GNSS simulator for MuJoCo-based simulation.

Injects synthetic GnssFix messages into a GnssModule as if the robot
were outside with a real WTRTK-980 receiver. Supports:

- Clean RTK_FIXED output (0.8 cm noise)
- RTK_FLOAT fallback (10 cm noise)
- SINGLE fallback (3 m noise)
- Loss-of-signal windows (indoor / tunnel simulation)
- Configurable update rate

Typical wiring in a sim scenario::

    from sim.sensors.wtrtk980_sim import WtrtkSim
    from localization.gnss_module import GnssModule

    gnss = GnssModule(origin_lat=31.2304, origin_lon=121.4737, origin_alt=10.0)
    sim = WtrtkSim(
        gnss=gnss,
        origin_lla=(31.2304, 121.4737, 10.0),
        get_body_xy=lambda: (physics.robot_x, physics.robot_y),
    )
    sim.start()
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from runtime.msgs.gnss import GnssFix, GnssFixType


@dataclass
class NoiseProfile:
    """1-sigma noise (metres) per fix type."""

    rtk_fixed: float = 0.008
    rtk_float: float = 0.1
    dgps: float = 1.0
    single: float = 3.0


class WtrtkSim:
    """Synthetic WTRTK-980 GNSS publisher.

    Converts MuJoCo-world-frame (x, y) into WGS84 (lat, lon) using a
    local ENU linearisation around a configured origin, adds Gaussian
    noise matching the configured fix type, and calls ``gnss.inject_fix``.
    """

    def __init__(
        self,
        gnss: Any,                                  # GnssModule (kept Any to avoid cycle)
        origin_lla: tuple[float, float, float],
        get_body_xy: Callable[[], tuple[float, float]],
        get_body_z: Optional[Callable[[], float]] = None,
        rate_hz: float = 50.0,
        fix_type: GnssFixType = GnssFixType.RTK_FIXED,
        noise: Optional[NoiseProfile] = None,
        num_sat_used: int = 15,
        num_sat_total: int = 22,
        loss_windows: Optional[list[tuple[float, float]]] = None,
        seed: Optional[int] = None,
    ) -> None:
        """
        Parameters
        ----------
        gnss
            ``GnssModule`` instance â€?its ``inject_fix`` will be called.
        origin_lla
            Reference WGS84 origin ``(lat_deg, lon_deg, alt_m)``.
        get_body_xy
            Callable returning the robot's current (x, y) in map metres.
        get_body_z
            Optional callable for altitude; defaults to origin altitude.
        rate_hz
            Publish rate (WTRTK-980 hardware max = 50 Hz).
        fix_type
            Baseline fix quality. Changes during ``loss_windows``.
        noise
            1-sigma noise per fix type.
        num_sat_used, num_sat_total
            Reported satellite count in GnssFix.
        loss_windows
            Time windows (since simulator start) during which GNSS
            degrades to NO_FIX â€?simulates tunnels, trees, underpasses.
        seed
            RNG seed for reproducible noise.
        """
        self._gnss = gnss
        self._origin_lat, self._origin_lon, self._origin_alt = origin_lla
        self._get_xy = get_body_xy
        self._get_z = get_body_z
        self._rate_hz = rate_hz
        self._fix_type = fix_type
        self._noise = noise or NoiseProfile()
        self._num_sat_used = num_sat_used
        self._num_sat_total = num_sat_total
        self._loss_windows = loss_windows or []
        self._rng = random.Random(seed)

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._start_ts = time.time()   # set here so publish_once() works w/o start()
        self._seq = 0

    # -- public API ---------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._start_ts = time.time()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="wtrtk-sim"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def publish_once(self) -> GnssFix:
        """Generate and publish a single fix (useful for tests)."""
        return self._publish_once()

    # -- internals ---------------------------------------------------------

    def _run(self) -> None:
        period = 1.0 / max(self._rate_hz, 0.1)
        while self._running:
            try:
                self._publish_once()
            except Exception:  # noqa: BLE001 â€?sim loop must not crash
                pass
            time.sleep(period)

    def _publish_once(self) -> GnssFix:
        now = time.time()
        elapsed = now - self._start_ts

        # Loss-of-signal window?
        in_loss = any(start <= elapsed <= end for start, end in self._loss_windows)
        if in_loss:
            fix = GnssFix(
                lat=0.0, lon=0.0, alt=0.0,
                fix_type=GnssFixType.NO_FIX,
                num_sat=0, num_sat_used=0,
                seq=self._seq, ts=now, frame_id="gnss_antenna",
            )
            self._seq += 1
            self._gnss.inject_fix(fix)
            return fix

        # Read body pose from physics
        x, y = self._get_xy()
        z = self._get_z() if self._get_z else 0.0

        # Add noise matching fix type
        sigma = self._sigma_for(self._fix_type)
        x_n = x + self._rng.gauss(0, sigma)
        y_n = y + self._rng.gauss(0, sigma)
        z_n = z + self._rng.gauss(0, sigma * 1.5)  # vertical typically worse

        # Local ENU â†?WGS84 (inverse of GnssModule's forward transform)
        lat, lon = self._enu_to_lla(x_n, y_n)
        alt = self._origin_alt + z_n

        var = sigma * sigma
        covariance = (
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var * 2.25,  # 1.5 sigma vertical â†?2.25 variance
        )

        fix = GnssFix(
            lat=lat, lon=lon, alt=alt,
            fix_type=self._fix_type,
            covariance=covariance,
            num_sat=self._num_sat_total,
            num_sat_used=self._num_sat_used,
            seq=self._seq,
            ts=now,
            frame_id="gnss_antenna",
        )
        self._seq += 1
        self._gnss.inject_fix(fix)
        return fix

    def _sigma_for(self, fix_type: GnssFixType) -> float:
        if fix_type == GnssFixType.RTK_FIXED:
            return self._noise.rtk_fixed
        if fix_type == GnssFixType.RTK_FLOAT:
            return self._noise.rtk_float
        if fix_type == GnssFixType.DGPS:
            return self._noise.dgps
        return self._noise.single

    def _enu_to_lla(self, east: float, north: float) -> tuple[float, float]:
        """Small-region linearised inverse of lla_to_enu.

        Valid within a few km of origin (locally flat-Earth approximation).
        """
        # Metres per degree at reference latitude
        lat_rad = math.radians(self._origin_lat)
        m_per_deg_lat = 111_320.0
        m_per_deg_lon = 111_320.0 * math.cos(lat_rad)

        d_lat = north / m_per_deg_lat
        d_lon = east / m_per_deg_lon if abs(m_per_deg_lon) > 1e-6 else 0.0
        return self._origin_lat + d_lat, self._origin_lon + d_lon
