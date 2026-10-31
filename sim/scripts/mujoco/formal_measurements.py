"""Optional measurements inside the existing Product physics/publisher owners."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass, replace
from typing import Any

from sim.runtime.recording.measurement_sink import MeasurementSink
from sim.runtime.recording.timeline import SensorPayloadSample
from sim.runtime.sensors.commands import SensorCommandProcessor
from sim.runtime.sensors.contracts import SensorRoute
from sim.runtime.sensors.measurement_configuration import make_measurement_extractor
from sim.runtime.sensors.measurement_extractors import PhysicsRead
from sim.runtime.sensors.runtime import SensorRuntime
from sim.runtime.sensors.semantic_camera import SemanticCameraSample
from sim.runtime.sensors.session import SensorEndpoint, SessionSensorRuntime

from .measurement_wire import encode_measurement

_TYPES = {
    "thermal": "lingtu.dds.ThermalImage",
    "acoustic": "lingtu.dds.AcousticBlock",
    "semantic": "lingtu.dds.SemanticImage",
}


@dataclass(frozen=True)
class PlannedMeasurements:
    sensor_plan: dict[str, Any]
    configurations: dict[str, dict[str, Any]]
    scene: dict[str, Any]
    max_hz: float


def planned_measurements(
    plan: Mapping[str, Any], *, instance_id: str, body_names: set[str], max_hz: float, camera_endpoint_available: bool
) -> PlannedMeasurements | None:
    """Validate opt-in data before creating engine, connection or publisher."""
    selected = {kind: declarations for kind in _TYPES if (declarations := plan.get("streams", {}).get(kind))}
    if not selected:
        return None
    if not camera_endpoint_available:
        raise ValueError("measurements require the Product-owned camera endpoint")
    if isinstance(max_hz, bool) or not math.isfinite(max_hz) or max_hz <= 0:
        raise ValueError("measurement physics sampling rate is invalid")
    scene = plan.get("scene_measurements")
    if not isinstance(scene, Mapping):
        raise ValueError("measurements require declared world scene_measurements")
    filtered = deepcopy({**plan, "streams": selected})
    runtime = SensorRuntime.from_plan(filtered)
    raw = {item["sensor_id"]: item for items in selected.values() for item in items}
    configs = {}

    def no_state():
        raise AssertionError("plan validation must not acquire physical state")

    for stream in runtime.streams:
        if stream.instance_id != instance_id or not stream.frame_id.startswith(instance_id + "/"):
            raise ValueError("measurement must belong to the physical robot instance")
        if (
            stream.route != SensorRoute("physics", "mujoco_measurement", "typed_dds")
            or stream.message_type != _TYPES[stream.stream_kind]
        ):
            raise ValueError("measurement requires the declared typed DDS route and message type")
        if stream.rate_hz > max_hz:
            raise ValueError("measurement rate exceeds physical sampling rate")
        config = raw[stream.sensor_id].get("measurement")
        if not isinstance(config, Mapping):
            raise ValueError("measurement configuration is missing")
        body = config.get("body_frame_id", "")
        prefix = instance_id + "/"
        if not isinstance(body, str) or not body.startswith(prefix) or body[len(prefix) :] not in body_names:
            raise ValueError("measurement body is not a declared robot body")
        make_measurement_extractor(stream, config, scene, no_state)
        if stream.stream_kind == "acoustic":
            samples_per_tick = config["microphone"]["sample_rate_hz"] / max_hz
            if not math.isclose(samples_per_tick, round(samples_per_tick), rel_tol=0, abs_tol=1e-9):
                raise ValueError("acoustic sample grid must align with the physics owner tick; no implicit resampling")
        configs[stream.sensor_id] = deepcopy(dict(config))
    return PlannedMeasurements(filtered, configs, deepcopy(dict(scene)), float(max_hz))


class _WireSink(MeasurementSink):
    def __init__(self, owner: FormalMeasurements, sensor_id: str, modality: str, on_record):
        super().__init__(sensor_id, modality, on_record=on_record)
        self.owner = owner

    def publish(self, sample):
        clock = {
            "source": "product_physics_tick",
            "sim_time_ns": sample.stamp.sim_time_ns,
            "mapped_epoch_s": self.owner._mapped_epoch_s,
        }
        if isinstance(sample, SemanticCameraSample):
            sample = replace(sample, calibration_metadata={**sample.calibration_metadata, "source_clock": clock})
        else:
            sample = replace(sample, metadata={**sample.metadata, "source_clock": clock})
        wire = encode_measurement(sample, product_session_id=self.owner.product_session_id).wire
        super().publish(sample)
        self.owner._pending.append(wire)


class FormalMeasurements:
    """Source-time sampling and one batched enqueue, never a new process owner.

    ``enqueue`` is the existing owned publisher queue, not a receipt ACK.
    ``on_record`` is optional existing-run recording staging. No recorder is
    silently created. The physics owner calls sample once per accepted tick.
    """

    def __init__(
        self,
        planned: PlannedMeasurements,
        *,
        product_session_id: str,
        read_state: PhysicsRead,
        enqueue: Callable[[bytes], None],
        model_generation: int,
        reset_generation: int,
        on_record: Callable[[SensorPayloadSample], None] | None = None,
    ):
        self.product_session_id = product_session_id
        self._planned, self._enqueue = planned, enqueue
        self._model_generation, self._reset_generation = model_generation, reset_generation
        self._pending: list[bytes] = []
        self._mapped_epoch_s = 0.0
        self._closed = False
        self._last_status: dict[str, Any] = {}
        self._sampled = False
        self._failure: str | None = None

        def endpoint(stream, allocation):
            extractor = make_measurement_extractor(
                stream, planned.configurations[stream.sensor_id], planned.scene, read_state
            )
            if hasattr(extractor, "max_rate_hz"):
                extractor.max_rate_hz = planned.max_hz
            return SensorEndpoint(
                "formal-mujoco-measurements",
                _WireSink(self, stream.sensor_id, stream.stream_kind, on_record),
                extractor,
            )

        self.controls = SessionSensorRuntime(
            plan=SensorRuntime.from_plan(planned.sensor_plan), allocation=None, endpoint_factory=endpoint
        )
        self.controls.prepare()
        self.commands = SensorCommandProcessor(self.controls, product_session_id=product_session_id,
            session_id=planned.sensor_plan["session_id"], model_generation=model_generation, reset_generation=reset_generation)
        self._last_status = self.status()

    def apply_commands(self, bridge: Any, *, now_s: float, sim_time_ns: int) -> None:
        """Apply at most eight requests on the physics/sampling owner's thread."""
        if self._closed:
            raise RuntimeError("formal measurements are closed")
        for _ in range(8):
            request = bridge.take_request()
            if request is None:
                break
            bridge.write_result(self.commands.apply(request, now_s=now_s, sim_time_ns=sim_time_ns))

    def sample(self, *, sim_time_s: float, mapped_epoch_s: float, physics_sequence: int) -> None:
        if self._closed:
            raise RuntimeError("formal measurements are closed")
        if not math.isfinite(sim_time_s) or sim_time_s < 0 or not math.isfinite(mapped_epoch_s) or mapped_epoch_s <= 0:
            raise ValueError("measurement source clock requires finite simulation and epoch times")
        if type(physics_sequence) is not int or physics_sequence < 0:
            raise ValueError("measurement physics sequence must be nonnegative")
        self._mapped_epoch_s = mapped_epoch_s
        self._pending.clear()
        snapshot = {
            "session_id": self._planned.sensor_plan["session_id"],
            "model_generation": self._model_generation,
            "reset_generation": self._reset_generation,
            "sim_time_ns": round(sim_time_s * 1e9),
            "sequence": physics_sequence,
        }
        try:
            self._last_status = self.status()
            self.controls.process_snapshot(snapshot)
            self._sampled = True
            self._last_status = self.status()
            if self._pending:
                self._enqueue(b"".join(self._pending))
        except Exception as exc:
            self._failure = f"{type(exc).__name__}: {exc}"
            self.close()
            raise
        finally:
            self._pending.clear()

    def status(self) -> dict[str, Any]:
        if self._closed:
            return {**deepcopy(self._last_status), "state": "failed" if self._failure else "closed", "error": self._failure}
        return {"state": "sampling", "streams": self.controls.status(),
                "observations": list(self.controls.evidence_observations()) if self._sampled else [],
                "publication_semantics": "sampled_and_queued_not_dds_receipt_ack", "error": None}

    def close(self) -> None:
        if not self._closed:
            try:
                if self._failure is None:
                    self._last_status = self.status()
                self.controls.close()
            finally:
                self._closed = True
