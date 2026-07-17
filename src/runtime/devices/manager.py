"""hw orchestrates hardware inventory and low-rate device status.

Loads `config/devices.yaml`, instantiates a Device per spec, opens them,
publishes `device_status` snapshot to Dashboard. Optional udev watcher
auto-reopens devices on hot-plug.

Phase 3: hot-plug detection via pyudev (graceful no-op if not installed).
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any, Callable

from runtime.contracts import HW_ROLE
from runtime.devices.base import Device, DeviceStatus
from runtime.devices.spec import DeviceSpec, load_device_specs
from runtime.module import Module
from runtime.registry import register
from runtime.stream import Out

logger = logging.getLogger(__name__)


@register("hw", "default", description="Hardware inventory/status module")
@register("device_manager", "default", description="Compatibility alias for hw")
class Hw(Module, layer=0):
    """Loads devices.yaml and manages all hardware lifecycle.

    Outputs:
        device_status — list of dicts (one per device), updated 2 Hz
        device_event  — string events (open/close/error/hotplug)
    """

    device_status: Out[list]
    device_event: Out[str]
    alive: Out[bool]
    runtime_id = HW_ROLE

    def __init__(
        self,
        config_path: str = "config/devices.yaml",
        status_rate_hz: float = 0.5,  # 2Hz → 0.5Hz, GIL friendly
        enable_hotplug: bool = False,  # default off — saves a thread
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._config_path = Path(config_path)
        self._status_rate_hz = status_rate_hz
        self._enable_hotplug = enable_hotplug

        self._specs: list[DeviceSpec] = []
        self._devices: dict[str, Device] = {}
        self._running = False
        self._status_thread: threading.Thread | None = None
        self._hotplug_thread: threading.Thread | None = None
        self._shutdown = threading.Event()

        # subscribers: callbacks for decoded messages, keyed by device id
        self._subscribers: dict[str, list[Callable]] = {}

    # ── Module lifecycle ──────────────────────────────────────────────

    def setup(self) -> None:
        """Load specs + import drivers + open enabled devices."""
        self._load_specs()
        self._import_drivers()
        self._open_all()

    def start(self) -> None:
        super().start()
        self._running = True
        self.alive.publish(True)
        self._shutdown.clear()
        self._status_thread = threading.Thread(
            target=self._status_loop,
            daemon=True,
            name="dev-status",
        )
        self._status_thread.start()
        if self._enable_hotplug:
            self._start_hotplug()

    def stop(self) -> None:
        self._running = False
        self._shutdown.set()
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=2.0)
        if self._hotplug_thread and self._hotplug_thread.is_alive():
            self._hotplug_thread.join(timeout=2.0)
        for d in self._devices.values():
            try:
                d.close()
            except Exception:
                logger.debug("hw: error closing device %s", d.id, exc_info=True)
        super().stop()

    # ── Spec loading ──────────────────────────────────────────────────

    def _load_specs(self) -> None:
        if not self._config_path.exists():
            logger.warning("hw: %s not found", self._config_path)
            return
        self._specs = load_device_specs(self._config_path)
        logger.info(
            "hw: loaded %d device specs from %s",
            len(self._specs),
            self._config_path,
        )

    def _import_drivers(self) -> None:
        """Trigger driver registration by importing the package."""
        try:
            pass
        except Exception as e:
            logger.warning("hw: driver import failed: %s", e)

    def _open_all(self) -> None:
        from runtime.devices.drivers.serial_nmea0183 import driver_registry

        registry = driver_registry()
        for spec in self._specs:
            if not spec.enabled:
                logger.info("hw: skip disabled device %s", spec.id)
                continue
            cls = registry.get(spec.driver)
            if cls is None:
                logger.warning(
                    "hw: driver %r for %s not registered",
                    spec.driver,
                    spec.id,
                )
                continue
            try:
                dev = cls(spec)
                self._devices[spec.id] = dev
                if hasattr(dev, "set_callback"):
                    dev.set_callback(self._make_dispatch(spec.id))
                if dev.open():
                    self.device_event.publish(f"open:{spec.id}")
                    logger.info("hw: opened %s (%s)", spec.id, spec.driver)
                else:
                    logger.warning(
                        "hw: failed to open %s — %s",
                        spec.id,
                        dev.health().last_error or "no hardware",
                    )
            except Exception as e:
                logger.exception("hw: error creating %s: %s", spec.id, e)

    # ── Subscription API ───────────────────────────────────────────────

    def subscribe(self, device_id: str, callback: Callable) -> None:
        """Register a callback to receive decoded messages from a device."""
        self._subscribers.setdefault(device_id, []).append(callback)

    def _make_dispatch(self, device_id: str) -> Callable:
        def _dispatch(msg: Any) -> None:
            for cb in self._subscribers.get(device_id, []):
                try:
                    cb(msg)
                except Exception as e:
                    logger.debug("hw dispatch %s: %s", device_id, e)

        return _dispatch

    def get_device(self, device_id: str) -> Device | None:
        return self._devices.get(device_id)

    def all_devices(self) -> list[Device]:
        return list(self._devices.values())

    # ── Status publishing ─────────────────────────────────────────────

    def _status_loop(self) -> None:
        interval = 1.0 / max(self._status_rate_hz, 0.1)
        while not self._shutdown.is_set():
            self._shutdown.wait(timeout=interval)
            if not self._running:
                continue
            snapshot = []
            for d in self._devices.values():
                snapshot.append(d.health().to_dict())
            # Include "missing" devices (spec exists, no Device created)
            opened_ids = {d.id for d in self._devices.values()}
            for spec in self._specs:
                if spec.id in opened_ids:
                    continue
                snapshot.append(
                    {
                        "id": spec.id,
                        "type": spec.type,
                        "status": "disabled" if not spec.enabled else "missing",
                        "description": spec.description,
                        "is_healthy": False,
                    }
                )
            self.device_status.publish(snapshot)

    # ── Hot-plug (optional, pyudev) ───────────────────────────────────

    def _start_hotplug(self) -> None:
        try:
            import pyudev

            ctx = pyudev.Context()
            monitor = pyudev.Monitor.from_netlink(ctx)
            monitor.filter_by(subsystem="usb")
        except ImportError:
            logger.info("hw: pyudev not installed, hot-plug disabled")
            return
        except Exception as e:
            logger.warning("hw: hot-plug init failed: %s", e)
            return

        def _watch():
            for action, device in monitor:  # blocking
                if not self._running:
                    break
                if action in ("add", "remove"):
                    self.device_event.publish(f"hotplug:{action}:{device.device_node}")
                    self._on_hotplug(action, device)

        self._hotplug_thread = threading.Thread(
            target=_watch,
            daemon=True,
            name="dev-hotplug",
        )
        self._hotplug_thread.start()
        logger.info("hw: hot-plug watcher started")

    def _on_hotplug(self, action: str, device: Any) -> None:
        """Re-detect/open devices that match the hot-plug event."""
        # Simple strategy: on add, try opening any OFFLINE device
        if action == "add":
            for d in self._devices.values():
                if d.status == DeviceStatus.OFFLINE:
                    if d.open():
                        logger.info("hw: hot-plug reopened %s", d.id)
                        self.device_event.publish(f"reopen:{d.id}")
        elif action == "remove":
            for d in self._devices.values():
                if d.status in (DeviceStatus.READY, DeviceStatus.STREAMING):
                    if not d.detect():
                        d.close()
                        logger.warning("hw: device %s removed", d.id)
                        self.device_event.publish(f"unplug:{d.id}")

    # ── Health for Dashboard ──────────────────────────────────────────

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["role"] = HW_ROLE
        info["backend"] = "inventory"
        info["status"] = "running" if self._running else "stopped"
        info["sensor_streams"] = False
        info["scope"] = "hardware inventory/status only"
        info["devices"] = [d.health().to_dict() for d in self._devices.values()]
        info["spec_count"] = len(self._specs)
        info["opened_count"] = len(self._devices)
        return info


DeviceManager = Hw
