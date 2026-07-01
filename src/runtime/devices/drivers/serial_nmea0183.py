"""Serial NMEA 0183 device driver — generic for any UART NMEA receiver.

Replaces the device-specific GnssSerialDriver. Spec.driver = "serial_nmea0183"
matches this class via the driver registry.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable

from runtime.devices.base import Device, DeviceStatus
from runtime.devices.decoders.nmea import NmeaDecoder

logger = logging.getLogger(__name__)


# ── Driver registry (small — drivers are typed not by string lookup ──
# but by spec.driver string. The DeviceManager maps name → class here.)

_driver_registry: dict[str, type[Device]] = {}


def register_driver(name: str):
    def wrap(cls):
        _driver_registry[name] = cls
        return cls
    return wrap


def get_driver(name: str) -> type[Device] | None:
    return _driver_registry.get(name)


def driver_registry() -> dict[str, type[Device]]:
    return dict(_driver_registry)


# ──────────────────────────────────────────────────────────────────────


@register_driver("serial_nmea0183")
class SerialNmea0183Device(Device):
    """Generic NMEA 0183 over serial — works for any GNSS that speaks NMEA.

    spec.serial:
        device: /dev/wtrtk980     # or actual /dev/ttyUSB*
        baud:   115200
    """

    def __init__(self, spec):
        super().__init__(spec)
        self._port = spec.serial.get("device", "/dev/ttyUSB0")
        self._baud = spec.serial.get("baud", 115200)
        self._serial = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._decoder = NmeaDecoder()
        self._on_message: Callable | None = None

    def set_callback(self, cb: Callable) -> None:
        """Register receiver for decoded messages."""
        self._on_message = cb

    def detect(self) -> bool:
        """Hardware present iff device file exists."""
        present = Path(self._port).exists()
        if present and self._status == DeviceStatus.OFFLINE:
            self._status = DeviceStatus.DETECTED
        elif not present:
            self._status = DeviceStatus.OFFLINE
        return present

    def open(self) -> bool:
        if not self.detect():
            return False
        try:
            import serial
            self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop, daemon=True,
                name=f"dev-{self.id}",
            )
            self._thread.start()
            self._status = DeviceStatus.READY
            logger.info(
                "Device %s: opened %s @ %d baud", self.id, self._port, self._baud,
            )
            return True
        except ImportError:
            self.mark_error("pyserial not installed")
            return False
        except Exception as e:
            self.mark_error(str(e))
            return False

    def close(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._status = DeviceStatus.OFFLINE

    def _read_loop(self) -> None:
        while self._running and self._serial:
            try:
                chunk = self._serial.read(self._serial.in_waiting or 1)
                if not chunk:
                    continue
                for msg in self._decoder.decode(chunk):
                    self.mark_data()
                    if self._on_message:
                        try:
                            self._on_message(msg)
                        except Exception as e:
                            logger.debug("device callback error: %s", e)
            except Exception as e:
                if self._running:
                    self.mark_error(str(e))
                    time.sleep(0.5)

    def _extra_metadata(self) -> dict[str, Any]:
        return {
            "port": self._port,
            "baud": self._baud,
        }
