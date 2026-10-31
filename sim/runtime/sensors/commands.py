"""Apply typed commands at the existing sensor owner's sampling boundary."""

from __future__ import annotations

import json
from collections import OrderedDict
from copy import deepcopy
from dataclasses import replace

from message.generated.enums import SensorControlKind as Kind
from runtime.contracts.sensor_control import SensorControlRequest, SensorControlResult

from .measurement_control import MeasurementNoise
from .session import SessionSensorRuntime


class SensorCommandProcessor:
    """Synchronous adapter, not a scheduler, process owner or motion controller."""

    def __init__(
        self,
        runtime: SessionSensorRuntime,
        *,
        product_session_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
    ):
        self.runtime = runtime
        self.product_session_id, self.session_id = product_session_id, session_id
        self.model_generation, self.reset_generation = model_generation, reset_generation
        self._results: OrderedDict[str, tuple[SensorControlRequest, SensorControlResult]] = OrderedDict()

    def apply(self, request: SensorControlRequest, *, now_s: float, sim_time_ns: int) -> SensorControlResult:
        def result(applied: bool, reason: str, settings=None) -> SensorControlResult:
            # Effective settings cross DDS as JSON; tuples are declared vectors,
            # so canonicalize here rather than weakening native roundtrip checks.
            wire_settings = json.loads(json.dumps(settings or {}, allow_nan=False))
            return SensorControlResult(
                request.request_id,
                self.product_session_id,
                self.session_id,
                request.sensor_id,
                self.model_generation,
                self.reset_generation,
                sim_time_ns,
                applied,
                reason,
                wire_settings,
            )

        if (request.product_session_id, request.session_id) != (self.product_session_id, self.session_id):
            return result(False, "source_identity_mismatch")
        if (request.model_generation, request.reset_generation) != (self.model_generation, self.reset_generation):
            return result(False, "generation_mismatch")
        signature = replace(request, deadline_s=1.0)
        if previous := self._results.get(request.request_id):
            return deepcopy(previous[1]) if previous[0] == signature else result(False, "request_id_conflict")
        if now_s > request.deadline_s:
            return result(False, "deadline_expired")
        try:
            self.runtime.status(request.sensor_id)
            if request.operation == Kind.QUERY:
                pass
            elif request.operation == Kind.SET_ENABLED:
                self.runtime.set_enabled(request.sensor_id, request.enabled)
            elif request.operation == Kind.SET_RATE:
                self.runtime.set_rate_hz(request.sensor_id, request.rate_hz)
            elif request.operation == Kind.CONFIGURE_NOISE:
                self.runtime.configure_noise(
                    request.sensor_id,
                    MeasurementNoise(
                        unit=request.noise_unit,
                        enabled=request.enabled,
                        stddev=request.noise_stddev,
                        seed=request.noise_seed,
                    ),
                )
            elif request.operation == Kind.SET_NOISE_ENABLED:
                self.runtime.set_noise_enabled(request.sensor_id, request.enabled)
            elif request.operation == Kind.SET_RECORDING:
                self.runtime.set_recording(request.sensor_id, request.enabled)
            elif request.operation == Kind.SET_GIMBAL:
                self.runtime.set_gimbal(request.sensor_id, request.pan_deg, request.tilt_deg)
            else:
                return result(False, "unsupported_operation")
            outcome = result(True, "applied", self.runtime.status(request.sensor_id))
        except (ValueError, RuntimeError) as exc:
            outcome = result(False, "invalid_configuration", {"error": str(exc)})
        self._results[request.request_id] = (signature, deepcopy(outcome))
        while len(self._results) > 256:
            self._results.popitem(last=False)
        return outcome
