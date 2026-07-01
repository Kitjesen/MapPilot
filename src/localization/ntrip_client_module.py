"""NtripClientModule — pull RTCM differential corrections from a NTRIP caster
and forward them to the GNSS receiver, unlocking RTK_FIXED solutions.

Protocol
--------
NTRIP 1.0 (HTTP-like): open TCP, send
    GET /<MOUNT> HTTP/1.0
    User-Agent: NTRIP LingTu/1.0
    Authorization: Basic <b64(user:pass)>
    Ntrip-Version: Ntrip/1.0
    \r\n

Caster answers ``ICY 200 OK`` (or ``HTTP/1.1 200``), then streams raw RTCM.
We forward bytes to an `rtcm_bytes: Out[bytes]` port; downstream the GNSS
serial driver writes them to the receiver's UART.

Some VRS / network RTK services require us to *upload* our current position
as NMEA GGA every 10s. We take the latest ``GnssFix`` from GnssModule and
format GGA locally — no extra driver needed.

Testing notes
-------------
You MUST have a valid NTRIP account (Qianxun, local CORS, etc.) and an
antenna with clear sky view. With ``rtcm.enabled=false`` this module is
inert — no connection attempts, no threads spun up.
"""

from __future__ import annotations

import base64
import logging
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.gnss import GnssFix
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@dataclass
class _Stats:
    connected: bool = False
    connect_attempts: int = 0
    connect_failures: int = 0
    rtcm_bytes_rx: int = 0
    gga_sent: int = 0
    last_rtcm_ts: float = 0.0
    last_error: str = ""


def _format_gga(fix: GnssFix) -> bytes:
    """Build a minimal NMEA GGA sentence from a GnssFix (enough for VRS)."""
    lat = abs(fix.lat)
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60.0
    lat_hem = "N" if fix.lat >= 0 else "S"

    lon = abs(fix.lon)
    lon_deg = int(lon)
    lon_min = (lon - lon_deg) * 60.0
    lon_hem = "E" if fix.lon >= 0 else "W"

    utc = time.gmtime(fix.ts)
    hhmmss = f"{utc.tm_hour:02d}{utc.tm_min:02d}{utc.tm_sec:02d}.00"

    # Compact the fields
    body = (
        f"GPGGA,{hhmmss},"
        f"{lat_deg:02d}{lat_min:09.6f},{lat_hem},"
        f"{lon_deg:03d}{lon_min:09.6f},{lon_hem},"
        f"{int(fix.fix_type)},"
        f"{max(fix.num_sat_used, 0):02d},"
        f"{fix.hdop_estimate if fix.hdop_estimate > 0 else 1.0:.1f},"
        f"{fix.alt:.2f},M,0.0,M,,"
    )
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    return f"${body}*{csum:02X}\r\n".encode("ascii")


@register("ntrip", "tcp", description="NTRIP 1.0 TCP client → RTCM bytes")
class NtripClientModule(Module, layer=1):
    """Pulls RTCM 3.x from a NTRIP caster, feeds GGA upstream, publishes bytes.

    Outputs
    -------
    rtcm_bytes:   raw RTCM frames, forwarded to GNSS serial driver
    ntrip_status: periodic diagnostic snapshot
    alive:        True when connected

    Inputs
    ------
    gnss_fix: latest fix from GnssModule, used to build NMEA GGA upload
    """

    rtcm_bytes:    Out[bytes]
    ntrip_status:  Out[dict]
    alive:         Out[bool]
    gnss_fix:      In[GnssFix]

    def __init__(
        self,
        enabled: bool = False,
        host: str = "",
        port: int = 2101,
        mount: str = "",
        user: str = "",
        password: str = "",
        gga_interval_s: float = 10.0,
        reconnect_backoff_s: float = 5.0,
        reconnect_backoff_max_s: float = 60.0,
        read_chunk_bytes: int = 4096,
        socket_timeout_s: float = 10.0,
        status_rate_hz: float = 0.5,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._enabled = bool(enabled and host and mount)
        self._host = host
        self._port = int(port)
        self._mount = mount
        self._user = user
        self._password = password
        self._gga_interval = gga_interval_s
        self._backoff = reconnect_backoff_s
        self._backoff_max = reconnect_backoff_max_s
        self._read_chunk = read_chunk_bytes
        self._socket_timeout = socket_timeout_s
        self._status_rate_hz = status_rate_hz

        self._last_fix: GnssFix | None = None
        self._last_gga_sent: float = 0.0
        self._stats = _Stats()

        self._thread: threading.Thread | None = None
        self._status_thread: threading.Thread | None = None
        self._shutdown = threading.Event()
        self._sock: socket.socket | None = None

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.gnss_fix.subscribe(self._on_fix)

    def start(self) -> None:
        super().start()
        if not self._enabled:
            logger.info("NtripClientModule disabled (gnss.rtcm.enabled=false or host/mount missing)")
            self.alive.publish(False)
            return
        self._shutdown.clear()
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="ntrip-client")
        self._thread.start()
        self._status_thread = threading.Thread(
            target=self._status_loop, daemon=True, name="ntrip-status")
        self._status_thread.start()
        logger.info(
            "NtripClientModule started: %s:%d/%s (gga every %.1fs)",
            self._host, self._port, self._mount, self._gga_interval,
        )

    def stop(self) -> None:
        self._shutdown.set()
        try:
            if self._sock is not None:
                self._sock.close()
        except Exception:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=2.0)
        super().stop()

    # -- callbacks ----------------------------------------------------------

    def _on_fix(self, fix: GnssFix) -> None:
        self._last_fix = fix

    # -- main loop ----------------------------------------------------------

    def _run_loop(self) -> None:
        backoff = self._backoff
        while not self._shutdown.is_set():
            try:
                self._connect_and_stream()
                backoff = self._backoff  # reset after successful session
            except Exception as e:
                self._stats.connect_failures += 1
                self._stats.last_error = str(e)
                self._stats.connected = False
                self.alive.publish(False)
                logger.warning(
                    "NTRIP session error: %s — retrying in %.1fs", e, backoff)
                # Truncated exponential backoff
                if self._shutdown.wait(backoff):
                    return
                backoff = min(backoff * 2.0, self._backoff_max)

    def _connect_and_stream(self) -> None:
        self._stats.connect_attempts += 1
        auth = base64.b64encode(
            f"{self._user}:{self._password}".encode()
        ).decode("ascii")
        req = (
            f"GET /{self._mount} HTTP/1.0\r\n"
            f"User-Agent: NTRIP LingTu/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Ntrip-Version: Ntrip/1.0\r\n"
            f"\r\n"
        ).encode("ascii")

        sock = socket.create_connection(
            (self._host, self._port), timeout=self._socket_timeout)
        self._sock = sock
        try:
            sock.sendall(req)
            # Parse handshake header (ends with \r\n\r\n or ICY response)
            header = b""
            while b"\r\n\r\n" not in header and b"ICY 200 OK" not in header:
                chunk = sock.recv(512)
                if not chunk:
                    raise OSError("NTRIP caster closed during handshake")
                header += chunk
                if len(header) > 8192:
                    raise OSError("NTRIP handshake too long — likely not a caster")

            if b"ICY 200 OK" not in header and b" 200 " not in header:
                # Surface 401 / 404 to logs for diagnosis
                snippet = header[:200].decode("ascii", errors="replace").strip()
                raise OSError(f"NTRIP handshake rejected: {snippet}")

            # Trim any body-start bytes that came after the header
            sep = header.find(b"\r\n\r\n")
            leftover = header[sep + 4:] if sep >= 0 else b""

            self._stats.connected = True
            self._stats.last_error = ""
            self.alive.publish(True)
            logger.info("NTRIP connected: %s:%d/%s",
                         self._host, self._port, self._mount)

            sock.settimeout(self._socket_timeout)
            if leftover:
                self._handle_rtcm(leftover)

            while not self._shutdown.is_set():
                # Send GGA at configured interval so VRS services can
                # compute our position-dependent corrections.
                self._maybe_send_gga(sock)

                try:
                    chunk = sock.recv(self._read_chunk)
                except TimeoutError:
                    continue
                if not chunk:
                    raise OSError("NTRIP caster closed the stream")
                self._handle_rtcm(chunk)
        finally:
            try:
                sock.close()
            except Exception:
                pass
            self._sock = None
            self._stats.connected = False
            self.alive.publish(False)

    def _handle_rtcm(self, data: bytes) -> None:
        if not data:
            return
        self._stats.rtcm_bytes_rx += len(data)
        self._stats.last_rtcm_ts = time.time()
        self.rtcm_bytes.publish(data)

    def _maybe_send_gga(self, sock: socket.socket) -> None:
        now = time.time()
        if now - self._last_gga_sent < self._gga_interval:
            return
        if self._last_fix is None:
            return
        try:
            sock.sendall(_format_gga(self._last_fix))
            self._last_gga_sent = now
            self._stats.gga_sent += 1
        except OSError as e:
            logger.debug("NTRIP GGA upload failed: %s", e)

    # -- status -------------------------------------------------------------

    def _status_loop(self) -> None:
        interval = 1.0 / max(self._status_rate_hz, 0.1)
        while not self._shutdown.wait(interval):
            if not self._enabled:
                continue
            now = time.time()
            age = (now - self._stats.last_rtcm_ts
                   if self._stats.last_rtcm_ts else float("inf"))
            self.ntrip_status.publish({
                "enabled": self._enabled,
                "connected": self._stats.connected,
                "host": self._host,
                "mount": self._mount,
                "attempts": self._stats.connect_attempts,
                "failures": self._stats.connect_failures,
                "rtcm_bytes_rx": self._stats.rtcm_bytes_rx,
                "rtcm_age_s": age,
                "gga_sent": self._stats.gga_sent,
                "last_error": self._stats.last_error,
            })

    # -- skills -------------------------------------------------------------

    @skill
    def get_ntrip_status(self) -> str:
        """Return NTRIP caster connection state: host, mount, bytes received,
        GGA uploads, last error. Use this to diagnose why RTK_FIXED is not
        being achieved."""
        import json
        now = time.time()
        age = (now - self._stats.last_rtcm_ts
               if self._stats.last_rtcm_ts else float("inf"))
        return json.dumps({
            "enabled": self._enabled,
            "connected": self._stats.connected,
            "host": self._host,
            "mount": self._mount,
            "attempts": self._stats.connect_attempts,
            "failures": self._stats.connect_failures,
            "rtcm_bytes_rx": self._stats.rtcm_bytes_rx,
            "rtcm_age_s": None if age == float("inf") else round(age, 2),
            "gga_sent": self._stats.gga_sent,
            "last_error": self._stats.last_error,
        })
