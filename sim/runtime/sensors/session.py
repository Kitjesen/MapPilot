"""Lifecycle and dispatch for plan-declared sensor endpoints.

The session runtime schedules only streams that have a concrete endpoint.  It
never upgrades an unimplemented SensorPlan declaration into a working stream.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from .contracts import ScheduledSensorSample, SensorStreamPlan
from .evidence import sensor_stream_binding_identity
from .extractors import imu_from_snapshot, truth_odometry_from_snapshot
from .runtime import SensorRuntime
from .samples import Mid360FrameSample, SensorSampleStamp


class SensorSessionError(RuntimeError):
    """Raised when a bound sensor endpoint cannot operate safely."""

    def __init__(self, message: str, *, sensor_id: str | None = None) -> None:
        self.sensor_id = sensor_id
        super().__init__(message)


class SensorSink(Protocol):
    """Transport endpoint owned by one plan-declared sensor stream."""

    def start(self) -> Mapping[str, Any]:
        """Create the transport endpoint and block until it is ready."""

    def publish(self, sample: Any) -> None:
        """Publish one typed sample."""

    def close(self) -> None:
        """Quiesce and release the endpoint."""


SensorExtractor = Callable[[ScheduledSensorSample, Mapping[str, Any]], Any]


@dataclass(frozen=True)
class SensorEndpoint:
    """One concrete extractor and transport sink for a SensorPlan stream."""

    source_id: str
    sink: SensorSink
    extractor: SensorExtractor

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id or self.source_id != self.source_id.strip():
            raise ValueError("sensor endpoint source_id must be a non-empty trimmed string")
        for method in ("start", "publish", "close"):
            if not callable(getattr(self.sink, method, None)):
                raise ValueError(f"sensor endpoint sink must provide {method}()")
        if not callable(self.extractor):
            raise ValueError("sensor endpoint extractor must be callable")

    @classmethod
    def truth_odometry(cls, *, source_id: str, sink: SensorSink) -> SensorEndpoint:
        """Create the canonical MuJoCo-truth odometry endpoint."""

        return cls(
            source_id=source_id,
            sink=sink,
            extractor=truth_odometry_from_snapshot,
        )

    @classmethod
    def imu(cls, *, source_id: str, sink: SensorSink) -> SensorEndpoint:
        """Create the canonical MuJoCo sensor-value IMU endpoint."""

        return cls(source_id=source_id, sink=sink, extractor=imu_from_snapshot)


SensorEndpointFactory = Callable[
    [SensorStreamPlan, Any],
    SensorEndpoint | None,
]


@dataclass(frozen=True)
class SensorPreparation:
    """Concrete stream evidence produced by endpoint preparation."""

    prepared_sensor_ids: tuple[str, ...]
    active_sensor_ids: tuple[str, ...]
    sources: Mapping[str, str]


class SessionSensorRuntime:
    """Own concrete sensor endpoints for one resolved simulation session."""

    def __init__(
        self,
        *,
        plan: SensorRuntime,
        allocation: Any,
        endpoint_factory: SensorEndpointFactory,
    ) -> None:
        if not callable(endpoint_factory):
            raise ValueError("endpoint_factory must be callable")
        self._plan = plan
        self._allocation = allocation
        self._factory = endpoint_factory
        self._endpoints: dict[str, SensorEndpoint] = {}
        self._streams = {stream.sensor_id: stream for stream in plan.streams}
        self._published_counts: dict[str, int] = {}
        self._last_samples: dict[str, Any] = {}
        self._failure_reasons: dict[str, str] = {}
        self._evidence_generation: tuple[int, int] | None = None
        self._scheduler: SensorRuntime | None = None
        self._prepared = False
        self._closed = False

    @property
    def active_sensor_ids(self) -> tuple[str, ...]:
        """Return streams backed by started endpoints."""

        return tuple(sorted(self._endpoints)) if self._prepared and not self._closed else ()

    @property
    def unbound_sensor_ids(self) -> tuple[str, ...]:
        """Return plan streams for which no endpoint was created."""

        active = set(self._endpoints)
        return tuple(stream.sensor_id for stream in self._plan.streams if stream.sensor_id not in active)

    @property
    def sources(self) -> Mapping[str, str]:
        """Return runtime source IDs keyed by stable sensor ID."""

        return {sensor_id: endpoint.source_id for sensor_id, endpoint in sorted(self._endpoints.items())}

    def binding_identity(self, sensor_id: str) -> str:
        """Return the canonical identity of one bound stream generation."""

        try:
            stream = self._streams[sensor_id]
            endpoint = self._endpoints[sensor_id]
        except KeyError as exc:
            raise SensorSessionError(
                f"sensor stream {sensor_id!r} is not bound",
                sensor_id=sensor_id,
            ) from exc
        generation = self._evidence_generation
        if generation is None:
            raise SensorSessionError(
                f"sensor stream {sensor_id!r} has no published generation",
                sensor_id=sensor_id,
            )
        return sensor_stream_binding_identity(
            stream,
            session_id=self._plan.session_id,
            model_generation=generation[0],
            reset_generation=generation[1],
            runtime_source_id=endpoint.source_id,
        )

    def evidence_observations(self) -> tuple[dict[str, Any], ...]:
        """Return strict observations for the most recently published generation."""

        if not self._prepared:
            raise SensorSessionError("sensor session is not prepared")
        generation = self._evidence_generation
        if generation is None:
            raise SensorSessionError("sensor session has not published a snapshot")
        observations: list[dict[str, Any]] = []
        for sensor_id, endpoint in sorted(self._endpoints.items()):
            stream = self._streams[sensor_id]
            failure_reason = self._failure_reasons.get(sensor_id)
            observation: dict[str, Any] = {
                "sensor_id": sensor_id,
                "state": "FAILED" if failure_reason is not None else "ACTIVE",
                "session_id": self._plan.session_id,
                "model_generation": generation[0],
                "reset_generation": generation[1],
                "owner": stream.route.owner,
                "source": stream.route.source,
                "transport": stream.route.transport,
                "message_type": stream.message_type,
                "runtime_source_id": endpoint.source_id,
                "binding_identity": self.binding_identity(sensor_id),
                "sample_count": self._published_counts.get(sensor_id, 0),
                "last_sample_truth_sequence": None,
                "last_sample_sim_time_ns": None,
            }
            if stream.route.transport == "camera_shm":
                observation["shm_name"] = self._allocation.shm.get(sensor_id)
            sample = self._last_samples.get(sensor_id)
            sample_count = observation["sample_count"]
            if sample_count:
                stamp = getattr(sample, "stamp", None)
                if not isinstance(stamp, SensorSampleStamp):
                    raise SensorSessionError(
                        f"typed sensor stream {sensor_id!r} has no stamped last sample evidence",
                        sensor_id=sensor_id,
                    )
                observation["last_sample_sim_time_ns"] = stamp.sim_time_ns
                observation["last_sample_truth_sequence"] = stamp.sequence
            if failure_reason is not None:
                observation["failure_reason"] = failure_reason
            if stream.stream_kind == "mid360":
                if isinstance(sample, Mid360FrameSample):
                    observation["fidelity"] = {
                        "reflectivity_semantics": sample.reflectivity_semantics,
                        "scan_time_profile": sample.scan_time_profile,
                        "unknown_line_representation": sample.unknown_line_representation,
                    }
                elif observation["sample_count"] > 0 or failure_reason is None:
                    raise SensorSessionError(
                        f"sensor stream {sensor_id!r} has no typed Mid360 frame evidence",
                        sensor_id=sensor_id,
                    )
            observations.append(observation)
        return tuple(observations)

    def prepare(self) -> SensorPreparation:
        """Create and start every endpoint supported by the supplied factory."""

        if self._prepared or self._endpoints:
            raise SensorSessionError("sensor session is already prepared")
        if self._closed:
            raise SensorSessionError("sensor session is closed")

        bound_streams: list[SensorStreamPlan] = []
        for stream in self._plan.streams:
            try:
                endpoint = self._factory(stream, self._allocation)
            except Exception as exc:
                message = self._close_after_failure(f"cannot create sensor endpoint {stream.sensor_id!r}: {exc}")
                raise SensorSessionError(
                    message,
                    sensor_id=stream.sensor_id,
                ) from exc
            if endpoint is None:
                continue
            if not isinstance(endpoint, SensorEndpoint):
                message = self._close_after_failure(
                    f"sensor endpoint factory returned an invalid value for {stream.sensor_id!r}"
                )
                raise SensorSessionError(
                    message,
                    sensor_id=stream.sensor_id,
                ) from None
            self._endpoints[stream.sensor_id] = endpoint
            bound_streams.append(stream)
            try:
                readiness = endpoint.sink.start()
                if not isinstance(readiness, Mapping):
                    raise TypeError("endpoint readiness must be an object")
            except Exception as exc:
                message = self._close_after_failure(f"cannot start sensor endpoint {stream.sensor_id!r}: {exc}")
                raise SensorSessionError(
                    message,
                    sensor_id=stream.sensor_id,
                ) from exc

        self._scheduler = SensorRuntime(
            self._plan.session_id,
            tuple(bound_streams),
        )
        self._prepared = True
        sensor_ids = tuple(stream.sensor_id for stream in bound_streams)
        return SensorPreparation(
            prepared_sensor_ids=sensor_ids,
            active_sensor_ids=sensor_ids,
            sources=self.sources,
        )

    def process_snapshot(self, snapshot: Mapping[str, Any]) -> None:
        """Extract and publish every bound sample due at this physics snapshot."""

        if not self._prepared or self._scheduler is None:
            raise SensorSessionError("sensor session is not prepared")
        if self._closed:
            raise SensorSessionError("sensor session is closed")
        try:
            batch = self._scheduler.advance(
                sim_time_ns=_snapshot_generation(snapshot, "sim_time_ns"),
                model_generation=_snapshot_generation(snapshot, "model_generation"),
                reset_generation=_snapshot_generation(snapshot, "reset_generation"),
            )
            generation = (batch.model_generation, batch.reset_generation)
            if generation != self._evidence_generation:
                self._published_counts = {
                    sensor_id: 0 for sensor_id in self._endpoints
                }
                self._last_samples.clear()
                self._evidence_generation = generation
            for scheduled in batch.samples:
                endpoint = self._endpoints[scheduled.sensor_id]
                sample = endpoint.extractor(scheduled, snapshot)
                _validate_sample_identity(sample, scheduled)
                endpoint.sink.publish(sample)
                self._published_counts[scheduled.sensor_id] += 1
                self._last_samples[scheduled.sensor_id] = sample
        except Exception as exc:
            sensor_id = scheduled.sensor_id if "scheduled" in locals() else "sensor-runtime"
            if sensor_id in self._endpoints:
                detail = str(exc).strip() or type(exc).__name__
                self._failure_reasons[sensor_id] = detail
            message = self._close_after_failure(f"sensor stream {sensor_id!r} failed while publishing: {exc}")
            raise SensorSessionError(
                message,
                sensor_id=(sensor_id if sensor_id != "sensor-runtime" else None),
            ) from exc

    def close(self) -> None:
        """Quiesce every endpoint; repeated calls are harmless."""

        self._close_all()

    def _close_after_failure(self, primary_message: str) -> str:
        try:
            self._close_all()
        except SensorSessionError as cleanup_error:
            return f"{primary_message}; secondary cleanup failure: {cleanup_error}"
        return primary_message

    def _close_all(self) -> None:
        if self._closed:
            return
        first_error: Exception | None = None
        for endpoint in reversed(tuple(self._endpoints.values())):
            try:
                endpoint.sink.close()
            except Exception as exc:  # pragma: no cover - covered through aggregate behavior
                if first_error is None:
                    first_error = exc
        self._closed = True
        if first_error is not None:
            raise SensorSessionError(f"cannot close sensor endpoint: {first_error}") from first_error


def _snapshot_generation(snapshot: Mapping[str, Any], field: str) -> int:
    value = snapshot.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SensorSessionError(f"snapshot.{field} must be a non-negative integer")
    return value


def _validate_sample_identity(
    sample: Any,
    scheduled: ScheduledSensorSample,
) -> None:
    if scheduled.stream.route.transport != "typed_dds":
        return
    stamp = getattr(sample, "stamp", None)
    if not isinstance(stamp, SensorSampleStamp):
        raise SensorSessionError(
            f"typed sensor {scheduled.sensor_id!r} sample has no SensorSampleStamp",
            sensor_id=scheduled.sensor_id,
        )
    expected = {
        "session_id": scheduled.session_id,
        "instance_id": scheduled.stream.instance_id,
        "sensor_id": scheduled.sensor_id,
        "frame_id": scheduled.stream.frame_id,
        "model_generation": scheduled.model_generation,
        "reset_generation": scheduled.reset_generation,
        "sequence": scheduled.sequence,
        "sim_time_ns": scheduled.deadline_ns,
    }
    for field, expected_value in expected.items():
        if getattr(stamp, field) != expected_value:
            raise SensorSessionError(
                f"sensor {scheduled.sensor_id!r} sample {field} does not match schedule",
                sensor_id=scheduled.sensor_id,
            )


__all__ = [
    "SensorEndpoint",
    "SensorEndpointFactory",
    "SensorPreparation",
    "SensorSessionError",
    "SensorSink",
    "SessionSensorRuntime",
]
