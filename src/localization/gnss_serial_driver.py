#!/usr/bin/env python3
"""Minimal GNSS serial driver â€?reads NMEA from WTRTK-980 and publishes
GnssFix messages into the LingTu module pipeline.

No ROS2 dependency. No rclpy. No colcon build.
Just pyserial + NMEA parsing â†?GnssModule.inject_fix().

Usage as standalone (diagnostics):
    python3 src/localization/gnss_serial_driver.py --port /dev/ttyUSB0 --baud 115200

Usage from GnssModule (integrated):
    from localization.gnss_serial_driver import GnssSerialDriver
    driver = GnssSerialDriver("/dev/ttyUSB0", 115200, callback=gnss_module.inject_fix)
    driver.start()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

logger = logging.getLogger(__name__)

# â”€â”€ NMEA checksum â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€


def _nmea_checksum(sentence: str) -> bool:
    """Verify NMEA 0183 XOR checksum. Returns True if valid or no checksum."""
    if '*' not in sentence:
        return True
    body, cksum_hex = sentence.rsplit('*', 1)
    body = body.lstrip('$')
    computed = 0
    for ch in body:
        computed ^= ord(ch)
    try:
        expected = int(cksum_hex[:2], 16)
    except ValueError:
        return False
    return computed == expected


# â”€â”€ NMEA GGA parser â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

# GGA fix quality â†?GnssFixType int mapping
_GGA_QUALITY = {
    '0': 0,   # NO_FIX
    '1': 1,   # SINGLE (GPS)
    '2': 2,   # DGPS
    '3': 3,   # PPS
    '4': 4,   # RTK_FIXED
    '5': 5,   # RTK_FLOAT
    '6': 6,   # ESTIMATED (dead reckoning)
    '7': 7,   # MANUAL
    '8': 8,   # SIMULATION
}


def _parse_latlon(raw: str, hemi: str) -> float | None:
    """Parse NMEA ddmm.mmmmm + N/S/E/W to decimal degrees."""
    if not raw or not hemi:
        return None
    try:
        # GGA latitude: ddmm.mmmmm (2-digit degree)
        # GGA longitude: dddmm.mmmmm (3-digit degree)
        dot = raw.index('.')
        deg_len = dot - 2  # 2 for lat, 3 for lon
        degrees = int(raw[:deg_len])
        minutes = float(raw[deg_len:])
        dec = degrees + minutes / 60.0
        if hemi in ('S', 'W'):
            dec = -dec
        return dec
    except (ValueError, IndexError):
        return None


def parse_gga(fields: list[str]) -> dict[str, Any] | None:
    """Parse $GNGGA / $GPGGA sentence fields into a dict.

    Returns None if sentence has no valid position.
    """
    # $GNGGA,hhmmss.ss,lat,N,lon,E,quality,numSV,HDOP,alt,M,sep,M,age,refID*CS
    if len(fields) < 15:
        return None

    quality_str = fields[6]
    fix_type = _GGA_QUALITY.get(quality_str, 0)

    lat = _parse_latlon(fields[2], fields[3])
    lon = _parse_latlon(fields[4], fields[5])

    if lat is None or lon is None:
        # Still return with NO_FIX so GnssModule sees link is alive
        return {
            'fix_type': 0,
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'num_sat_used': 0, 'hdop': 99.9,
        }

    try:
        alt = float(fields[9]) if fields[9] else 0.0
    except ValueError:
        alt = 0.0

    try:
        num_sat = int(fields[7]) if fields[7] else 0
    except ValueError:
        num_sat = 0

    try:
        hdop = float(fields[8]) if fields[8] else 99.9
    except ValueError:
        hdop = 99.9

    # GGA field 14 (index 13) = age of differential corrections (seconds).
    # Empty when no RTCM corrections are being received.
    rtcm_age: float | None
    try:
        rtcm_age = float(fields[13]) if fields[13] else None
    except (ValueError, IndexError):
        rtcm_age = None

    return {
        'fix_type': fix_type,
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'num_sat_used': num_sat,
        'hdop': hdop,
        'rtcm_age_s': rtcm_age,
    }


# â”€â”€ Driver class â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€


class GnssSerialDriver:
    """Read NMEA from a serial port and invoke a callback with GnssFix.

    Designed to be lightweight â€?no ROS2, no DDS, just pyserial.
    GnssModule can use this internally when DDS is not available
    (no ironoa driver installed).
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        callback: Callable | None = None,
        timeout: float = 1.0,
    ) -> None:
        self._port = port
        self._baud = baud
        self._callback = callback
        self._timeout = timeout
        self._serial = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._seq = 0
        # Latest RTCM differential age from GGA field 14 â€?None if not received.
        self.last_rtcm_age: float | None = None

    def start(self) -> bool:
        """Open serial port and start read thread. Returns True on success."""
        try:
            import serial
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=self._timeout,
            )
            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop, daemon=True, name="gnss-serial",
            )
            self._thread.start()
            logger.info(
                "GnssSerialDriver: started on %s @ %d baud",
                self._port, self._baud,
            )
            return True
        except ImportError:
            logger.error("GnssSerialDriver: pyserial not installed (pip install pyserial)")
            return False
        except Exception as e:
            logger.error("GnssSerialDriver: cannot open %s: %s", self._port, e)
            return False

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def write_rtcm(self, data: bytes) -> bool:
        """Forward RTCM differential corrections to the receiver UART.

        Called by NtripClientModule callbacks â€?receivers that accept RTCM
        on the same serial line (UM980 / UBX) consume them and upgrade the
        fix type to RTK_FIXED / RTK_FLOAT. Safe to call concurrently with
        the read loop; pyserial's Serial.write is thread-safe on POSIX.
        """
        if self._serial is None or not self._running:
            return False
        try:
            self._serial.write(data)
            return True
        except Exception as e:
            logger.debug("GnssSerialDriver: RTCM write failed: %s", e)
            return False

    def _read_loop(self) -> None:
        """Continuously read NMEA lines and dispatch GGA fixes."""
        from runtime.msgs.gnss import GnssFix, GnssFixType

        buf = b''
        while self._running and self._serial:
            try:
                chunk = self._serial.read(self._serial.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk

                # Process complete lines
                while b'\n' in buf:
                    line_raw, buf = buf.split(b'\n', 1)
                    line = line_raw.decode('ascii', errors='ignore').strip()

                    if not line.startswith('$'):
                        continue

                    # Only parse GGA (position fix)
                    if 'GGA' not in line:
                        continue

                    if not _nmea_checksum(line):
                        continue

                    fields = line.split(',')
                    parsed = parse_gga(fields)
                    if parsed is None:
                        continue

                    self._seq += 1
                    hdop = parsed['hdop']
                    self.last_rtcm_age = parsed.get('rtcm_age_s')
                    # Estimate covariance from HDOP (rough: cov â‰?(hdop * base_sigma)Â²)
                    base_sigma = 0.01 if parsed['fix_type'] == 4 else 0.1 if parsed['fix_type'] == 5 else 3.0
                    h_var = (hdop * base_sigma) ** 2
                    v_var = h_var * 2.25  # vertical typically 1.5x worse

                    fix = GnssFix(
                        lat=parsed['lat'],
                        lon=parsed['lon'],
                        alt=parsed['alt'],
                        fix_type=GnssFixType(parsed['fix_type']),
                        covariance=(h_var, 0, 0, 0, h_var, 0, 0, 0, v_var),
                        num_sat=parsed['num_sat_used'],
                        num_sat_used=parsed['num_sat_used'],
                        seq=self._seq,
                        ts=time.time(),
                        frame_id="gnss_antenna",
                    )

                    if self._callback:
                        try:
                            self._callback(fix)
                        except Exception as e:
                            logger.debug("GnssSerialDriver: callback error: %s", e)

            except OSError as e:
                if self._running:
                    logger.warning("GnssSerialDriver: read error: %s", e)
                    time.sleep(0.05)  # brief backoff; serial read already uses timeout


# â”€â”€ Standalone mode â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

if __name__ == "__main__":
    import argparse
    import sys

    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent.parent))

    parser = argparse.ArgumentParser(description="WTRTK-980 GNSS diagnostic reader")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    from runtime.msgs.gnss import GnssFix

    def _print_fix(fix: GnssFix) -> None:
        print(fix)

    driver = GnssSerialDriver(args.port, args.baud, callback=_print_fix)
    if not driver.start():
        sys.exit(1)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        driver.stop()
        print("\nStopped.")
