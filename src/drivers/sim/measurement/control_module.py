"""Bounded Host sensor controls backed by source-owner acknowledgements."""

from __future__ import annotations

import time
from collections import OrderedDict
from collections.abc import Callable, Collection, Mapping
from dataclasses import replace
from threading import Event, RLock
from types import MappingProxyType
from typing import Protocol

from runtime.contracts.sensor_control import SensorControlConflict, SensorControlRequest, SensorControlResult
from runtime.module import Module
from runtime.registry import register
from runtime.stream import Out


class _ControlBridge(Protocol):
    def write_request(self, request: SensorControlRequest) -> None: ...
    def take_result(self) -> SensorControlResult | None: ...
    def close(self) -> None: ...


@register("sensor_controls", "native_dds", description="Acknowledged native DDS sensor controls")
class SensorControlModule(Module, layer=1):
    """Request controls without owning sensors, data endpoints, or processes."""

    state: Out[SensorControlResult]
    runtime_id = "sensor.controls"

    def __init__(
        self,
        *,
        product_session_id: str,
        session_id: str,
        domain_id: int,
        sensor_ids: Collection[str],
        model_generation: int,
        reset_generation: int,
        bridge_factory: Callable[..., _ControlBridge] | None = None,
        **kw,
    ) -> None:
        if type(domain_id) is not int or not 0 <= domain_id <= 232:
            raise ValueError("domain_id must be an integer in 0..232")
        if isinstance(sensor_ids, (str, bytes)) or not isinstance(sensor_ids, Collection) or not sensor_ids:
            raise ValueError("sensor_ids must be a nonempty collection of sensor IDs")
        for value in (product_session_id, session_id, *sensor_ids):
            if (
                not isinstance(value, str)
                or not value
                or value != value.strip()
                or "\0" in value
                or len(value.encode("utf-8")) > 255
            ):
                raise ValueError("sensor control identities must be nonempty bounded trimmed text")
        for name, generation in (("model_generation", model_generation), ("reset_generation", reset_generation)):
            if type(generation) is not int or not 0 <= generation < 2**64:
                raise ValueError(f"{name} must be uint64")
        super().__init__(**kw)
        self._product_session_id = product_session_id
        self._session_id = session_id
        self._domain_id = domain_id
        self._sensor_ids = frozenset(sensor_ids)
        self._model_generation = model_generation
        self._reset_generation = reset_generation
        self._bridge_factory = bridge_factory
        self._bridge: _ControlBridge | None = None
        self._operation_lock = RLock()
        self._closing = Event()
        self._published: OrderedDict[str, SensorControlResult] = OrderedDict()
        self._submitted: OrderedDict[str, SensorControlRequest] = OrderedDict()

    def setup(self) -> None:
        """Open a client endpoint, without asserting source/sensor readiness."""
        with self._operation_lock:
            if self._closing.is_set():
                raise RuntimeError("sensor control endpoint is closed")
            try:
                self._open_bridge()
            except Exception as exc:
                raise RuntimeError(f"sensor control bridge setup failed: {exc}") from exc

    def _open_bridge(self) -> None:
        if self._bridge is not None:
            return
        factory = self._bridge_factory
        if factory is None:
            from .control_bridge import NativeSensorControlBridge

            factory = NativeSensorControlBridge
        self._bridge = factory(
            domain_id=self._domain_id,
            product_session_id=self._product_session_id,
            session_id=self._session_id,
            owner_mode=False,
        )

    def start(self) -> None:
        with self._operation_lock:
            if self._closing.is_set():
                raise RuntimeError("sensor control endpoint is closed")
            if self._bridge is None:
                raise RuntimeError("sensor control setup must succeed before start")
            super().start()

    def startup_readiness(self) -> str | None:
        """Describe bridge setup only; this is not sensor readiness evidence."""
        if self._closing.is_set():
            return "sensor_control_closed"
        return None if self._bridge is not None else "sensor_control_bridge_not_setup"

    def binding(self, sensor_id: str) -> Mapping[str, str | int]:
        """Return an immutable identity snapshot for a declared sensor."""
        if sensor_id not in self._sensor_ids:
            raise ValueError(f"undeclared sensor: {sensor_id}")
        return MappingProxyType(
            {
                "product_session_id": self._product_session_id,
                "session_id": self._session_id,
                "sensor_id": sensor_id,
                "model_generation": self._model_generation,
                "reset_generation": self._reset_generation,
            }
        )

    def execute(self, request: SensorControlRequest) -> SensorControlResult:
        """Write once and wait at most two seconds, including serialization wait.

        ``deadline_s`` is an absolute wall-clock deadline. Only the matching
        source ACK supplies effective settings; timeout does not prove that
        the source did not apply a command. Expiry raises ``TimeoutError``.
        """
        if not isinstance(request, SensorControlRequest):
            raise TypeError("request must be SensorControlRequest")
        if request.sensor_id not in self._sensor_ids or any(
            getattr(request, field) != getattr(self, f"_{field}")
            for field in ("product_session_id", "session_id", "model_generation", "reset_generation")
        ):
            raise ValueError("sensor control request does not match the declared sensor binding")
        deadline = time.monotonic() + min(2.0, request.deadline_s - time.time())
        if self._closing.is_set():
            raise RuntimeError("sensor control endpoint is closed")
        if not self._operation_lock.acquire(timeout=max(0.0, deadline - time.monotonic())):
            raise TimeoutError(f"sensor control request {request.request_id} timed out waiting for endpoint")
        try:
            if self._closing.is_set():
                raise RuntimeError("sensor control endpoint is closed")
            if self._bridge is None:
                raise RuntimeError("sensor control setup must succeed before execute")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"sensor control request {request.request_id} timed out before write")
            signature = replace(request, deadline_s=1.)
            previous = self._submitted.get(request.request_id)
            if previous is not None and previous != signature:
                raise SensorControlConflict("request_id_conflict")
            self._submitted[request.request_id] = signature
            self._submitted.move_to_end(request.request_id)
            if len(self._submitted) > 256:
                self._submitted.popitem(last=False)
            try:
                return self._execute(request, deadline, self._bridge)
            except (RuntimeError, TimeoutError):
                raise
            except Exception as exc:
                raise RuntimeError(f"sensor control request {request.request_id} failed: {exc}") from exc
        finally:
            self._operation_lock.release()

    def _execute(self, request: SensorControlRequest, deadline: float, bridge: _ControlBridge) -> SensorControlResult:
        bridge.write_request(request)
        while time.monotonic() < deadline:
            result = bridge.take_result()
            if time.monotonic() >= deadline:
                break
            if result is not None and all(
                getattr(result, field) == getattr(request, field)
                for field in (
                    "request_id",
                    "product_session_id",
                    "session_id",
                    "sensor_id",
                    "model_generation",
                    "reset_generation",
                )
            ):
                if self._published.get(result.request_id) != result:
                    self._published[result.request_id] = result
                    self._published.move_to_end(result.request_id)
                    if len(self._published) > 256:
                        self._published.popitem(last=False)
                    self.state.publish(result)
                return result
            time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(f"sensor control request {request.request_id} timed out")

    def close(self) -> None:
        """Reject new work, wait for the bounded active operation, close once."""
        self._closing.set()
        if not self._operation_lock.acquire(timeout=2.0):
            raise RuntimeError("sensor control close timed out waiting for active operation")
        try:
            bridge, self._bridge = self._bridge, None
            try:
                if bridge is not None:
                    bridge.close()
            except Exception as exc:
                raise RuntimeError(f"sensor control bridge close failed: {exc}") from exc
            finally:
                super().stop()
        finally:
            self._operation_lock.release()

    def stop(self) -> None:
        self.close()
