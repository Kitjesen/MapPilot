"""Host measurement ports backed by an explicitly bound native DDS reader."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from collections.abc import Callable, Collection
from copy import deepcopy
from math import isfinite
from typing import Any

from runtime.contracts.sensor_control import SensorControlResult
from runtime.module import Module
from runtime.msgs.measurement import AcousticMeasurement, SemanticMeasurement, ThermalMeasurement
from runtime.registry import register
from runtime.stream import In, Out

from .native_bridge import NativeMeasurementBridge

logger = logging.getLogger(__name__)


@register("measurements", "native_dds", description="Native typed DDS measurement endpoint")
class DdsMeasurementModule(Module, layer=1):
    """Adapt native measurements to Host ports; never own a source process.

    Polls take at most 1..1024 records (including rejected records). The owned
    thread waits at least 1 ms between batches. Readiness describes receipt
    freshness, not sensor quality or suitability for epoch-aligned inspection.
    Host-local ports deliver typed objects without a serialization protocol.
    """

    thermal: Out[ThermalMeasurement]
    acoustic: Out[AcousticMeasurement]
    semantic: Out[SemanticMeasurement]
    alive: Out[bool]
    control_state: In[SensorControlResult]
    runtime_id = "measurements"

    def __init__(
        self,
        *,
        product_session_id: str,
        session_id: str,
        domain_id: int,
        required_sensor_ids: Collection[str],
        bridge_factory: Callable[..., NativeMeasurementBridge] | None = None,
        start_thread: bool = True,
        stale_timeout_s: float = 1.0,
        max_frames_per_poll: int = 32,
        poll_interval_s: float = 0.01,
        **kw,
    ) -> None:
        if type(domain_id) is not int or not 0 <= domain_id <= 232:
            raise ValueError("domain_id must be an integer in 0..232")
        if isinstance(required_sensor_ids, str) or not required_sensor_ids:
            raise ValueError("required_sensor_ids must be a nonempty collection of sensor IDs")
        for value in (product_session_id, session_id, *required_sensor_ids):
            if not isinstance(value, str) or not value or value != value.strip() or "\0" in value:
                raise ValueError("measurement identities must be explicit nonempty trimmed strings")
        for name, duration in (("poll_interval_s", poll_interval_s), ("stale_timeout_s", stale_timeout_s)):
            if (
                isinstance(duration, bool)
                or not isinstance(duration, (int, float))
                or not isfinite(duration)
                or duration <= 0
            ):
                raise ValueError(f"{name} must be finite positive seconds")
        if type(max_frames_per_poll) is not int or not 1 <= max_frames_per_poll <= 1024:
            raise ValueError("max_frames_per_poll must be an integer in 1..1024")
        super().__init__(**kw)
        self._domain_id = domain_id
        self._product_session_id = product_session_id
        self._session_id = session_id
        self._bridge_factory = NativeMeasurementBridge if bridge_factory is None else bridge_factory
        self._bridge: NativeMeasurementBridge | None = None
        self._required_sensor_ids = tuple(required_sensor_ids)
        self._stale_timeout_s = stale_timeout_s
        self._sensors = {sensor_id: self._empty_sensor() for sensor_id in self._required_sensor_ids}
        self._max_frames_per_poll = max_frames_per_poll
        self._rejected_frames = 0
        self._errors: dict[str, str] = {}
        self._start_thread = start_thread
        self._poll_interval_s = max(0.001, poll_interval_s)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._state_lock = threading.RLock()
        self._controls: dict[str, SensorControlResult] = {}
        self._control_seen: deque[str] = deque(maxlen=256)

    @staticmethod
    def _empty_sensor() -> dict[str, Any]:
        return dict(
            frames=0, received_at_s=None, captured_at_s=None, clock_mapped=False, receipt_monotonic=None, quality=None
        )

    def setup(self) -> None:
        """Open only the explicitly configured reader, without consuming data."""
        if self._closed:
            raise RuntimeError("measurement endpoint is closed")
        if self._bridge is not None:
            return
        self.control_state.subscribe(self._on_control_state)
        try:
            self._bridge = self._bridge_factory(
                domain_id=self._domain_id, product_session_id=self._product_session_id, session_id=self._session_id
            )
        except Exception as exc:
            self._fail("setup", exc)
            raise

    def start(self) -> None:
        """Start after Host has wired consumer subscriptions."""
        if self._closed:
            raise RuntimeError("measurement endpoint is closed")
        if self._bridge is None:
            raise RuntimeError("measurement setup must succeed before start")
        if self._stop_event.is_set():
            raise RuntimeError("measurement endpoint failed or is stopping")
        if self._running:
            return
        super().start()
        self.alive.publish(False)
        if self._start_thread:
            try:
                self._thread = threading.Thread(target=self._read_loop, name="measurement-dds-reader", daemon=True)
                self._thread.start()
            except Exception as exc:
                self._thread = None
                self._fail("start", exc)
                raise

    def _read_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                self.poll_once()
                self._stop_event.wait(self._poll_interval_s)
        except Exception as exc:
            self._fail("reader", exc)
            logger.exception("Native measurement reader stopped")
        finally:
            self._running = False
            self.alive.publish(False)

    def poll_once(self) -> int:
        """Dispatch one bounded batch; malformed records consume the budget too."""
        if not self._running or self._bridge is None:
            return 0
        published = 0
        for _ in range(self._max_frames_per_poll):
            if self._stop_event.is_set():
                break
            try:
                sample = self._bridge.take(received_at_s=time.time())
                if sample is None:
                    break
                self._publish_frame(sample)
            except (ValueError, TypeError) as exc:
                with self._state_lock:
                    self._rejected_frames += 1
                    self._errors["frame"] = f"{type(exc).__name__}: {exc}"
                continue
            except Exception as exc:
                self._fail("take", exc)
                raise
            with self._state_lock:
                self._errors.pop("frame", None)
            published += 1
        self.alive.publish(self.startup_readiness() is None)
        return published

    def _fail(self, stage: str, exc: Exception) -> None:
        with self._state_lock:
            self._errors[stage] = f"{type(exc).__name__}: {exc}"
            self._running = False
        self._stop_event.set()
        self.alive.publish(False)

    def _publish_frame(self, sample: ThermalMeasurement | AcousticMeasurement | SemanticMeasurement) -> None:
        ports: dict[type, Out[Any]] = {
            ThermalMeasurement: self.thermal,
            AcousticMeasurement: self.acoustic,
            SemanticMeasurement: self.semantic,
        }
        if type(sample) not in ports:
            raise TypeError(f"unsupported measurement type: {type(sample).__name__}")
        total = int(sample.valid_mask.size)
        valid = int(sample.valid_mask.sum())
        with self._state_lock:
            sensor = self._sensors.setdefault(sample.identity.sensor_id, self._empty_sensor())
            sensor.update(
                frames=sensor["frames"] + 1,
                received_at_s=sample.received_at_s,
                captured_at_s=sample.captured_at_s,
                clock_mapped=sample.captured_at_s is not None,
                receipt_monotonic=time.monotonic(),
                quality=dict(
                    valid_elements=valid, total_elements=total, valid_fraction=valid / total if total else 0.0
                ),
            )
        port = ports[type(sample)]
        errors_before = port.publish_errors
        port.publish(sample)
        if port.publish_errors != errors_before:
            raise RuntimeError(f"measurement {port.name} publication failed; inspect port publish_errors")

    def _sensor_health(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        sensors = {}
        with self._state_lock:
            snapshot = {sensor_id: dict(state) for sensor_id, state in self._sensors.items()}
            controls = dict(self._controls)
        for sensor_id, state in snapshot.items():
            info = dict(state)
            if info["quality"] is not None:
                info["quality"] = dict(info["quality"])
            received = info.pop("receipt_monotonic")
            age = None if received is None else max(0.0, now - received)
            info.update(receipt_age_s=age, fresh=age is not None and age <= self._stale_timeout_s)
            control = controls.get(sensor_id)
            enabled = control.settings.get("enabled", True) if control is not None else True
            info.update(enabled=enabled, state="DISABLED" if not enabled else "ACTIVE" if info["fresh"] else "WAITING",
                        controls=deepcopy(control.settings) if control is not None else None)
            sensors[sensor_id] = info
        return sensors

    def _on_control_state(self, result: SensorControlResult) -> None:
        if (not result.applied or result.product_session_id != self._product_session_id
                or result.session_id != self._session_id or result.sensor_id not in self._sensors
                or type(result.settings.get("enabled")) is not bool):
            return
        with self._state_lock:
            previous = self._controls.get(result.sensor_id)
            if (result.request_id in self._control_seen
                    or (previous is not None and result.applied_sim_time_ns < previous.applied_sim_time_ns)):
                return
            self._control_seen.append(result.request_id)
            if previous is not None and previous.settings.get("enabled") is False and result.settings["enabled"]:
                self._sensors[result.sensor_id]["receipt_monotonic"] = None
            self._controls[result.sensor_id] = deepcopy(result)

    def startup_readiness(self) -> str | None:
        """Require a fresh receipt for every configured sensor, not just a type."""
        if not self._running:
            return "not_running"
        if self._errors:
            return "measurement_error"
        sensors = self._sensor_health()
        expected = [name for name in self._required_sensor_ids if sensors[name]["enabled"]]
        missing = [name for name in expected if not sensors[name]["frames"]]
        if missing:
            return f"measurement_sensors_missing:{','.join(missing)}"
        stale = [name for name in expected if not sensors[name]["fresh"]]
        if stale:
            return f"measurement_sensors_stale:{','.join(stale)}"
        return None

    def health(self) -> dict[str, Any]:
        """Expose transport errors, receipt clocks, and quality independently."""
        info = super().port_summary()
        ready = self.startup_readiness() is None
        info.update(
            role="measurements",
            backend="native_dds",
            ready=ready,
            status="error" if self._errors else "ready" if ready else "running" if self._running else "stopped",
            sensors=self._sensor_health(),
            rejected_frames=self._rejected_frames,
            errors=dict(self._errors),
            error="; ".join(self._errors.copy().values()) or None,
            product_session_id=self._product_session_id,
            session_id=self._session_id,
            domain_id=self._domain_id,
            required_sensor_ids=list(self._required_sensor_ids),
            poll_interval_s=self._poll_interval_s,
            max_frames_per_poll=self._max_frames_per_poll,
            stale_timeout_s=self._stale_timeout_s,
            transport="native_dds",
        )
        return info

    def stop(self) -> None:
        """Join the owned reader before closing its bridge exactly once."""
        if self._closed:
            return
        self._stop_event.set()
        self._running = False
        if self._thread is not None:
            if self._thread is threading.current_thread():
                exc = RuntimeError("measurement stop must join from outside the poll thread")
                self._fail("shutdown", exc)
                raise exc
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                exc = RuntimeError("measurement poll thread did not stop")
                self._fail("shutdown", exc)
                raise exc
            self._thread = None
        self._running = False
        with self._state_lock:
            bridge, self._bridge = self._bridge, None
        try:
            if bridge is not None:
                bridge.close()
        except Exception as exc:
            self._fail("close", exc)
            raise
        finally:
            self.alive.publish(False)
            super().stop()
