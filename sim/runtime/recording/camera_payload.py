"""Bridge validated RobotSimUE camera SHM frames into recording payloads."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from sim.adapters.shm import (
    SHM_SCHEMA,
    CameraShmAllocation,
    CameraShmChanged,
    CameraShmCorrupt,
    CameraShmNotReady,
    CameraShmReader,
    CameraShmStale,
    FrameSnapshot,
    StreamKind,
    WindowsNamedMappingBackend,
)

from .timeline import SensorPayloadSample


class SensorPayloadCaptureError(RuntimeError):
    """A runtime sensor payload violated its compiled recording contract."""


class SensorPayloadSource(Protocol):
    """Ordered source sampled at one accepted truth-snapshot boundary."""

    def capture(
        self,
        snapshot: Mapping[str, Any],
    ) -> Sequence[SensorPayloadSample]:
        """Return payload samples aligned to one accepted snapshot."""

        ...

    def close(self) -> None:
        """Release source-owned transport handles."""

        ...


class CameraMappingBackend(Protocol):
    """Backend capable of opening one explicitly allocated camera mapping."""

    def open(self, allocation: CameraShmAllocation) -> Any: ...


@dataclass(frozen=True, slots=True)
class _CameraStreamSpec:
    sensor_id: str
    stream_kind: str
    frame_id: str
    encoding: str
    width: int
    height: int


class CameraShmPayloadSource:
    """Read each new validated Camera SHM frame once for recording."""

    def __init__(
        self,
        *,
        sensor_plan: Mapping[str, Any],
        allocation_provider: Callable[[], Any],
        mapping_backend: CameraMappingBackend | None = None,
        slot_capacity: int = 8 * 1024 * 1024,
        max_age_s: float | None = 1.0,
    ) -> None:
        if not callable(allocation_provider):
            raise TypeError("allocation_provider must be callable")
        if isinstance(slot_capacity, bool) or not isinstance(slot_capacity, int):
            raise ValueError("slot_capacity must be a positive integer")
        if slot_capacity <= 0:
            raise ValueError("slot_capacity must be a positive integer")
        if max_age_s is not None and (
            isinstance(max_age_s, bool)
            or not isinstance(max_age_s, (int, float))
            or float(max_age_s) < 0.0
        ):
            raise ValueError("max_age_s must be non-negative or None")
        self._session_id, self._streams = _camera_stream_specs(sensor_plan)
        self._allocation_provider = allocation_provider
        self._backend = mapping_backend or WindowsNamedMappingBackend()
        self._slot_capacity = slot_capacity
        self._max_age_s = None if max_age_s is None else float(max_age_s)
        self._mappings: dict[str, Any] = {}
        self._readers: dict[str, CameraShmReader] = {}
        self._closed = False

    @property
    def stream_ids(self) -> tuple[str, ...]:
        """Return the deterministic camera stream identities."""

        return tuple(spec.sensor_id for spec in self._streams)

    def capture(
        self,
        snapshot: Mapping[str, Any],
    ) -> tuple[SensorPayloadSample, ...]:
        """Return unseen valid camera frames at one truth-snapshot boundary."""

        if self._closed:
            raise SensorPayloadCaptureError("camera payload source is closed")
        _validate_snapshot_identity(snapshot, session_id=self._session_id)
        if not self._streams:
            return ()
        shm = _allocation_shm(self._allocation_provider())
        samples: list[SensorPayloadSample] = []
        for spec in self._streams:
            mapping_name = shm.get(spec.sensor_id)
            if not isinstance(mapping_name, str) or not mapping_name:
                raise SensorPayloadCaptureError(
                    f"camera stream {spec.sensor_id!r} has no RunAllocation SHM name"
                )
            reader = self._reader(spec.sensor_id, mapping_name)
            if reader is None:
                continue
            try:
                frame = reader.read_latest()
            except (CameraShmNotReady, CameraShmChanged, CameraShmStale):
                continue
            except CameraShmCorrupt as exc:
                raise SensorPayloadCaptureError(
                    f"camera stream {spec.sensor_id!r} is corrupt: {exc}"
                ) from exc
            if frame is None:
                continue
            _validate_frame(spec, frame)
            samples.append(_payload_sample(spec, frame))
        return tuple(samples)

    def close(self) -> None:
        """Release all read-side mapping handles; repeated calls are harmless."""

        if self._closed:
            return
        self._closed = True
        self._readers.clear()
        for mapping in self._mappings.values():
            close = getattr(mapping, "close", None)
            if callable(close):
                try:
                    close()
                except OSError:
                    pass
        self._mappings.clear()

    def _reader(self, sensor_id: str, mapping_name: str) -> CameraShmReader | None:
        existing = self._readers.get(sensor_id)
        if existing is not None:
            return existing
        allocation = CameraShmAllocation(
            mapping_name,
            slot_capacity=self._slot_capacity,
        )
        try:
            mapping = self._backend.open(allocation)
        except (CameraShmNotReady, FileNotFoundError, OSError):
            return None
        reader = CameraShmReader(mapping, max_age_s=self._max_age_s)
        self._mappings[sensor_id] = mapping
        self._readers[sensor_id] = reader
        return reader


def _camera_stream_specs(
    sensor_plan: Mapping[str, Any],
) -> tuple[str, tuple[_CameraStreamSpec, ...]]:
    if not isinstance(sensor_plan, Mapping):
        raise SensorPayloadCaptureError("sensor_plan must be an object")
    if sensor_plan.get("schema") != "lingtu.sim.sensor-plan.v1":
        raise SensorPayloadCaptureError("sensor_plan schema is invalid")
    session_id = sensor_plan.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        raise SensorPayloadCaptureError("sensor_plan session_id is invalid")
    streams = sensor_plan.get("streams")
    if not isinstance(streams, Mapping):
        raise SensorPayloadCaptureError("sensor_plan streams must be an object")
    result: list[_CameraStreamSpec] = []
    seen: set[str] = set()
    for stream_kind, declarations in streams.items():
        if not isinstance(declarations, Sequence) or isinstance(
            declarations, (str, bytes)
        ):
            raise SensorPayloadCaptureError(
                f"sensor_plan streams.{stream_kind} must be an array"
            )
        for declaration in declarations:
            if not isinstance(declaration, Mapping):
                raise SensorPayloadCaptureError(
                    f"sensor_plan streams.{stream_kind} entry must be an object"
                )
            if declaration.get("transport") != "camera_shm":
                continue
            if stream_kind not in {"rgb", "depth"}:
                raise SensorPayloadCaptureError(
                    f"camera_shm stream kind {stream_kind!r} is unsupported"
                )
            if declaration.get("source") != "unreal_camera":
                raise SensorPayloadCaptureError(
                    f"camera_shm stream {stream_kind!r} must use unreal_camera"
                )
            sensor_id = _text(declaration.get("sensor_id"), "sensor_id")
            if sensor_id in seen:
                raise SensorPayloadCaptureError(
                    f"camera sensor_id {sensor_id!r} is duplicated"
                )
            seen.add(sensor_id)
            result.append(
                _CameraStreamSpec(
                    sensor_id=sensor_id,
                    stream_kind=str(stream_kind),
                    frame_id=_text(declaration.get("frame_id"), "frame_id"),
                    encoding=_text(declaration.get("encoding"), "encoding"),
                    width=_positive_integer(declaration.get("width"), "width"),
                    height=_positive_integer(declaration.get("height"), "height"),
                )
            )
    return session_id, tuple(sorted(result, key=lambda item: item.sensor_id))


def _allocation_shm(allocation: Any) -> Mapping[str, Any]:
    shm = (
        allocation.get("shm")
        if isinstance(allocation, Mapping)
        else getattr(allocation, "shm", None)
    )
    if not isinstance(shm, Mapping):
        raise SensorPayloadCaptureError("RunAllocation shm registry is invalid")
    return shm


def _validate_snapshot_identity(
    snapshot: Mapping[str, Any],
    *,
    session_id: str,
) -> None:
    if not isinstance(snapshot, Mapping) or snapshot.get("event") != "snapshot":
        raise SensorPayloadCaptureError("sensor payload capture requires a snapshot event")
    if snapshot.get("session_id") != session_id:
        raise SensorPayloadCaptureError(
            "sensor payload snapshot session_id does not match SensorPlan"
        )
    for field in ("model_generation", "reset_generation", "sequence", "sim_time_ns"):
        value = snapshot.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise SensorPayloadCaptureError(
                f"sensor payload snapshot {field} must be a non-negative integer"
            )


def _validate_frame(spec: _CameraStreamSpec, frame: FrameSnapshot) -> None:
    expected_kind = StreamKind.COLOR if spec.stream_kind == "rgb" else StreamKind.DEPTH
    if frame.stream_kind is not expected_kind:
        raise SensorPayloadCaptureError(
            f"camera stream {spec.sensor_id!r} kind does not match SensorPlan"
        )
    if frame.encoding != spec.encoding:
        raise SensorPayloadCaptureError(
            f"camera stream {spec.sensor_id!r} encoding does not match SensorPlan"
        )
    if frame.frame_id != spec.frame_id:
        raise SensorPayloadCaptureError(
            f"camera stream {spec.sensor_id!r} frame_id does not match SensorPlan"
        )
    if frame.width != spec.width or frame.height != spec.height:
        raise SensorPayloadCaptureError(
            f"camera stream {spec.sensor_id!r} dimensions do not match SensorPlan"
        )


def _payload_sample(
    spec: _CameraStreamSpec,
    frame: FrameSnapshot,
) -> SensorPayloadSample:
    media_types = {
        "rgb8": "application/vnd.lingtu.rgb8",
        "bgr8": "application/vnd.lingtu.bgr8",
        "rgba8": "application/vnd.lingtu.rgba8",
        "mono8": "application/vnd.lingtu.mono8",
        "8UC1": "application/vnd.lingtu.u8",
        "16UC1": "application/vnd.lingtu.depth-u16",
        "32FC1": "application/vnd.lingtu.depth-f32",
    }
    return SensorPayloadSample(
        sensor_id=spec.sensor_id,
        stream_kind=spec.stream_kind,
        encoding=frame.encoding,
        media_type=media_types.get(frame.encoding, "application/octet-stream"),
        sample_sequence=frame.sequence,
        sample_time_ns=frame.timestamp_ns,
        payload=frame.payload,
        metadata={
            "transport_schema": SHM_SCHEMA,
            "clock_domain": "unix_realtime",
            "frame_id": frame.frame_id,
            "width": frame.width,
            "height": frame.height,
            "stride_bytes": frame.stride,
            "fx": frame.fx,
            "fy": frame.fy,
            "cx": frame.cx,
            "cy": frame.cy,
            "depth_scale": frame.depth_scale,
            "distortion": [
                frame.dist_k1,
                frame.dist_k2,
                frame.dist_p1,
                frame.dist_p2,
                frame.dist_k3,
            ],
        },
    )


def _text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SensorPayloadCaptureError(f"camera stream {field} is invalid")
    return value


def _positive_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise SensorPayloadCaptureError(
            f"camera stream {field} must be a positive integer"
        )
    return value


__all__ = [
    "CameraShmPayloadSource",
    "SensorPayloadCaptureError",
    "SensorPayloadSource",
]
