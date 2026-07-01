"""NMEA 0183 decoder — parses GGA/RMC/GSA sentences into GnssFix.

Reusable across any NMEA-speaking device (WTRTK-980, ZED-F9P, BD-980, ...).
"""

from __future__ import annotations

import time
from typing import Iterator

from runtime.devices.decoder import Decoder, register_decoder
from runtime.msgs.gnss import GnssFix, GnssFixType


def _checksum(sentence: str) -> bool:
    if "*" not in sentence:
        return True
    body, cks = sentence.rsplit("*", 1)
    body = body.lstrip("$")
    val = 0
    for ch in body:
        val ^= ord(ch)
    try:
        return val == int(cks[:2], 16)
    except ValueError:
        return False


def _parse_latlon(raw: str, hemi: str) -> float | None:
    if not raw or not hemi:
        return None
    try:
        dot = raw.index(".")
        deg_len = dot - 2
        deg = int(raw[:deg_len])
        minutes = float(raw[deg_len:])
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except (ValueError, IndexError):
        return None


_GGA_QUALITY = {
    "0": 0, "1": 1, "2": 2, "3": 3,
    "4": 4, "5": 5, "6": 6, "7": 7, "8": 8,
}


@register_decoder("nmea0183")
class NmeaDecoder(Decoder):
    """Parses NMEA 0183 GGA sentences. Buffers partial lines across calls."""

    def __init__(self) -> None:
        self._buf = b""
        self._seq = 0

    def decode(self, raw: bytes) -> Iterator[GnssFix]:
        self._buf += raw
        while b"\n" in self._buf:
            line_raw, self._buf = self._buf.split(b"\n", 1)
            line = line_raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("$") or "GGA" not in line:
                continue
            if not _checksum(line):
                continue
            fix = self._parse_gga(line.split(","))
            if fix is not None:
                yield fix

    def _parse_gga(self, fields: list[str]) -> GnssFix | None:
        if len(fields) < 15:
            return None

        fix_type_int = _GGA_QUALITY.get(fields[6], 0)
        lat = _parse_latlon(fields[2], fields[3])
        lon = _parse_latlon(fields[4], fields[5])

        try:
            num_sat = int(fields[7]) if fields[7] else 0
        except ValueError:
            num_sat = 0
        try:
            hdop = float(fields[8]) if fields[8] else 99.9
        except ValueError:
            hdop = 99.9
        try:
            alt = float(fields[9]) if fields[9] else 0.0
        except ValueError:
            alt = 0.0

        self._seq += 1

        if lat is None or lon is None:
            # Still emit a NO_FIX so downstream knows link is alive
            return GnssFix(
                lat=0.0, lon=0.0, alt=0.0,
                fix_type=GnssFixType.NO_FIX,
                covariance=(99.0, 0, 0, 0, 99.0, 0, 0, 0, 99.0),
                num_sat=num_sat, num_sat_used=num_sat, seq=self._seq,
                ts=time.time(), frame_id="gnss_antenna",
            )

        # Estimate covariance from HDOP * fix-type base sigma
        base = 0.01 if fix_type_int == 4 else 0.1 if fix_type_int == 5 else 3.0
        h_var = (hdop * base) ** 2
        v_var = h_var * 2.25
        return GnssFix(
            lat=lat, lon=lon, alt=alt,
            fix_type=GnssFixType(fix_type_int),
            covariance=(h_var, 0, 0, 0, h_var, 0, 0, 0, v_var),
            num_sat=num_sat, num_sat_used=num_sat, seq=self._seq,
            ts=time.time(), frame_id="gnss_antenna",
        )
