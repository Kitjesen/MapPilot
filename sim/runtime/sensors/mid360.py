"""MID-360 scan-pattern timing owned by the new Sensor Runtime."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol

from runtime.msgs.numpy_compat import np

from .contracts import DeadlinePolicy, ScheduledSensorSample
from .dds_adapter import Mid360AdapterError
from .samples import LivoxPointSample, Mid360FrameSample, SensorSampleStamp

DEFAULT_PATTERN_PATH = Path(__file__).resolve().parents[2] / "assets" / "livox" / "mid360.npy"
DEFAULT_POINTS_PER_FRAME = 20_000
DEFAULT_SCAN_PERIOD_NS = 100_000_000
DEFAULT_REFLECTIVITY_PROXY = 15


class Mid360PatternError(ValueError):
    """Raised when the MID-360 pattern cannot be parsed."""


@dataclass(slots=True)
class Mid360PatternCursor:
    """Deterministic cursor over a parsed Livox MID-360 pattern."""

    path: Path = DEFAULT_PATTERN_PATH
    points_per_frame: int = DEFAULT_POINTS_PER_FRAME
    scan_period_ns: int = DEFAULT_SCAN_PERIOD_NS
    cursor: int = 0
    _angles: Any = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.path = Path(self.path)
        if (
            isinstance(self.points_per_frame, bool)
            or self.points_per_frame <= 0
            or self.points_per_frame > 800_000
        ):
            raise Mid360PatternError("points_per_frame must be in the range 1..800000")
        if (
            isinstance(self.scan_period_ns, bool)
            or self.scan_period_ns <= 0
            or self.scan_period_ns > (1 << 32)
        ):
            raise Mid360PatternError("scan_period_ns must fit the uint32 offset field")
        angles: Any = np.load(self.path, mmap_mode="r")
        if angles.shape != (800_000, 2) or angles.dtype != np.float32:
            raise Mid360PatternError("MID-360 pattern shape or dtype mismatch")
        self._angles = angles

    def next_rays(self) -> tuple[np.ndarray, np.ndarray]:
        """Return sensor-frame unit vectors and uint32 offsets for one 10 Hz scan."""

        indexes = (np.arange(self.points_per_frame, dtype=np.int64) + self.cursor) % int(
            self._angles.shape[0]
        )
        self.cursor = int((self.cursor + self.points_per_frame) % int(self._angles.shape[0]))
        theta = np.asarray(self._angles[indexes, 0], dtype=np.float64)
        phi = np.asarray(self._angles[indexes, 1], dtype=np.float64)
        cos_phi = np.cos(phi)
        directions = np.empty((self.points_per_frame, 3), dtype=np.float64)
        directions[:, 0] = cos_phi * np.cos(theta)
        directions[:, 1] = cos_phi * np.sin(theta)
        directions[:, 2] = np.sin(phi)
        offsets = (
            np.arange(self.points_per_frame, dtype=np.uint64)
            * int(self.scan_period_ns)
            // int(self.points_per_frame)
        ).astype(np.uint32)
        return directions, offsets


def _raycast_value(frame: object, field: str, default: Any = None) -> Any:
    if isinstance(frame, dict):
        return frame.get(field, default)
    return getattr(frame, field, default)


def _snapshot_uint_or_default(snapshot: object, field: str, default: int) -> int:
    if not isinstance(snapshot, Mapping) or field not in snapshot:
        return default
    value = snapshot[field]
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise Mid360AdapterError(
            f"MID-360 physics snapshot {field} must be a non-negative integer"
        )
    return value


def _reset_cursor(cursor: object) -> None:
    if hasattr(cursor, "cursor"):
        cursor.cursor = 0


def _array_rows(value: object, *, field: str) -> tuple[tuple[float, float, float], ...]:
    rows = np.asarray(value, dtype=np.float64)
    if len(rows.shape) != 2 or rows.shape[1] != 3 or rows.shape[0] == 0:
        raise Mid360PatternError(f"{field} must contain one or more xyz rows")
    result: list[tuple[float, float, float]] = []
    for index, row in enumerate(rows):
        triple = (float(row[0]), float(row[1]), float(row[2]))
        if not all(math.isfinite(component) for component in triple):
            raise Mid360PatternError(f"{field}[{index}] must be finite")
        result.append(triple)
    return tuple(result)


def _uint32_tuple(value: object, *, field: str) -> tuple[int, ...]:
    values = tuple(int(item) for item in np.asarray(value).reshape(-1).tolist())
    if not values:
        raise Mid360PatternError(f"{field} must contain one or more offsets")
    for index, item in enumerate(values):
        if item < 0 or item > 0xFFFFFFFF:
            raise Mid360PatternError(f"{field}[{index}] must fit uint32")
    return values


class _Mid360Publisher(Protocol):
    def start(self) -> dict[str, Any]: ...

    def publish(self, sample: Mid360FrameSample) -> None: ...

    def close(self) -> None: ...


class _Mid360RaycastProcess(Protocol):
    def raycast(self, **kwargs: Any) -> object: ...


class _Mid360Cursor(Protocol):
    def next_rays(self) -> tuple[Any, Any]: ...


@dataclass(slots=True)
class Mid360RaycastPipeline:
    """Compose MID-360 pattern timing, MuJoCo raycast, frame building, and publishing."""

    process: _Mid360RaycastProcess
    publisher: _Mid360Publisher
    sensor_frame_id: str
    cursor: _Mid360Cursor = field(default_factory=Mid360PatternCursor)
    physics_state: Callable[[], str] | None = None
    range_min_m: float = 0.1
    range_max_m: float = 40.0
    reflectivity_proxy: int = DEFAULT_REFLECTIVITY_PROXY
    unknown_line: int = 0
    scan_time_profile: str = "instantaneous_geometry/scheduled_offsets"
    _started: bool = field(default=False, init=False, repr=False)
    _last_generation: tuple[int, int] | None = field(default=None, init=False, repr=False)
    _last_sequence: int | None = field(default=None, init=False, repr=False)
    _last_sim_time_ns: int | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if not callable(getattr(self.process, "raycast", None)):
            raise ValueError("process must provide raycast()")
        for method in ("start", "publish", "close"):
            if not callable(getattr(self.publisher, method, None)):
                raise ValueError(f"publisher must provide {method}()")
        if not callable(getattr(self.cursor, "next_rays", None)):
            raise ValueError("cursor must provide next_rays()")
        if (
            not isinstance(self.sensor_frame_id, str)
            or not self.sensor_frame_id
            or self.sensor_frame_id != self.sensor_frame_id.strip()
            or any(character.isspace() for character in self.sensor_frame_id)
        ):
            raise ValueError("sensor_frame_id must be one non-empty stable frame token")

    def start(self) -> dict[str, Any]:
        """Start only the injected publisher; the Physics process is owned elsewhere."""

        if self._started:
            raise Mid360AdapterError("MID-360 raycast pipeline is already started")
        readiness = self.publisher.start()
        if not isinstance(readiness, dict):
            raise Mid360AdapterError("MID-360 publisher readiness must be an object")
        self._started = True
        return readiness

    def publish_scheduled(self, scheduled: ScheduledSensorSample) -> Mid360FrameSample:
        """Raycast and publish one scheduler-owned MID-360 sample."""

        if not self._started:
            raise Mid360AdapterError("MID-360 raycast pipeline is not started")
        self._require_physics_ready()
        self._validate_scheduled(scheduled)
        generation = (scheduled.model_generation, scheduled.reset_generation)
        generation_changed = generation != self._last_generation
        if generation_changed:
            _reset_cursor(self.cursor)

        directions, offsets = self.cursor.next_rays()
        direction_rows = _array_rows(directions, field="directions_sensor")
        offset_values = _uint32_tuple(offsets, field="offsets_time_ns")
        if len(direction_rows) != len(offset_values):
            raise Mid360PatternError("MID-360 directions and offsets length mismatch")

        raycast_frame = self.process.raycast(
            sensor_frame_id=self.sensor_frame_id,
            directions_sensor=direction_rows,
            offsets_time_ns=offset_values,
            session_id=scheduled.session_id,
            model_generation=scheduled.model_generation,
            reset_generation=scheduled.reset_generation,
            sequence=scheduled.sequence,
            sim_time_ns=scheduled.deadline_ns,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            reflectivity_proxy=self.reflectivity_proxy,
            unknown_line=self.unknown_line,
        )
        sample = mid360_frame_from_raycast(
            scheduled,
            raycast_frame,
            scan_time_profile=self.scan_time_profile,
        )
        self.publisher.publish(sample)
        self._last_generation = generation
        self._last_sequence = scheduled.sequence
        self._last_sim_time_ns = scheduled.deadline_ns
        return sample

    def close(self) -> None:
        """Close only the injected publisher."""

        if self._started:
            self.publisher.close()
            self._started = False

    def _require_physics_ready(self) -> None:
        if self.physics_state is None:
            return
        state = self.physics_state()
        if state not in {"READY", "RUNNING"}:
            raise Mid360AdapterError("MID-360 raycast requires Physics READY/RUNNING")

    def _validate_scheduled(self, scheduled: ScheduledSensorSample) -> None:
        if not isinstance(scheduled, ScheduledSensorSample):
            raise Mid360PatternError("scheduled must be a ScheduledSensorSample")
        if scheduled.stream.stream_kind != "mid360":
            raise Mid360PatternError("scheduled stream is not mid360")
        generation = (scheduled.model_generation, scheduled.reset_generation)
        if self._last_generation is None:
            return
        if generation < self._last_generation:
            raise Mid360AdapterError("MID-360 scheduled generation moved backward")
        if generation != self._last_generation:
            if scheduled.sequence != 0:
                raise Mid360AdapterError("MID-360 sequence must restart at 0 after reset/model generation")
            return
        if self._last_sequence is None or scheduled.sequence <= self._last_sequence:
            raise Mid360AdapterError("MID-360 scheduled sequence is not increasing")
        if (
            scheduled.policy is DeadlinePolicy.CATCH_UP
            and scheduled.sequence != self._last_sequence + 1
        ):
            raise Mid360AdapterError("MID-360 scheduled sequence is not contiguous")
        if self._last_sim_time_ns is None or scheduled.deadline_ns <= self._last_sim_time_ns:
            raise Mid360AdapterError("MID-360 scheduled sim_time is not increasing")


def mid360_frame_from_raycast(
    scheduled: ScheduledSensorSample,
    raycast_frame: object,
    scan_time_profile: str = "instantaneous_geometry/scheduled_offsets",
) -> Mid360FrameSample:
    """Validate a Physics Runtime raycast result and produce a typed frame sample."""

    if scheduled.stream.stream_kind != "mid360":
        raise Mid360PatternError("scheduled stream is not mid360")
    if isinstance(raycast_frame, dict):
        scan_time_profile = str(
            raycast_frame.get("scan_time_profile", scan_time_profile) or scan_time_profile
        )
        raycast_frame = raycast_frame.get("mid360_raycast_frame", raycast_frame)
    frame_time = _raycast_value(raycast_frame, "sim_time_ns")
    if frame_time != scheduled.deadline_ns:
        raise Mid360PatternError("MID-360 raycast frame time does not match schedule")
    if _raycast_value(raycast_frame, "model_generation") != scheduled.model_generation:
        raise Mid360PatternError("MID-360 raycast frame model generation mismatch")
    if _raycast_value(raycast_frame, "reset_generation") != scheduled.reset_generation:
        raise Mid360PatternError("MID-360 raycast frame reset generation mismatch")
    points: list[LivoxPointSample] = []
    previous_offset = -1
    for raw in _raycast_value(raycast_frame, "hits", ()):
        x, y, z = tuple(_raycast_value(raw, "xyz_sensor"))
        if not all(math.isfinite(float(value)) for value in (x, y, z)):
            raise Mid360PatternError("MID-360 raycast produced non-finite xyz")
        offset = int(_raycast_value(raw, "offset_time_ns"))
        if offset < previous_offset:
            raise Mid360PatternError("MID-360 offset_time_ns must be non-decreasing")
        previous_offset = offset
        points.append(
            LivoxPointSample(
                x=float(x),
                y=float(y),
                z=float(z),
                reflectivity=int(_raycast_value(raw, "reflectivity", DEFAULT_REFLECTIVITY_PROXY)),
                offset_time_ns=offset,
                tag=int(_raycast_value(raw, "tag", 0)),
                line=int(_raycast_value(raw, "line", 0)),
            )
        )
    return Mid360FrameSample(
        stamp=SensorSampleStamp.from_scheduled(scheduled),
        points=tuple(points),
        scan_time_profile=scan_time_profile,
    )


@dataclass(slots=True)
class Mid360RaycastExtractor:
    """Build one scheduled MID-360 frame through the injected Physics Runtime."""

    process: _Mid360RaycastProcess
    sensor_frame_id: str
    cursor: _Mid360Cursor = field(default_factory=Mid360PatternCursor)
    physics_state: Callable[[], str] | None = None
    range_min_m: float = 0.1
    range_max_m: float = 40.0
    reflectivity_proxy: int = DEFAULT_REFLECTIVITY_PROXY
    unknown_line: int = 0
    scan_time_profile: str = "instantaneous_geometry/scheduled_offsets"
    _last_generation: tuple[int, int] | None = field(default=None, init=False, repr=False)
    _last_sequence: int | None = field(default=None, init=False, repr=False)
    _last_sim_time_ns: int | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if not callable(getattr(self.process, "raycast", None)):
            raise ValueError("process must provide raycast()")
        if not callable(getattr(self.cursor, "next_rays", None)):
            raise ValueError("cursor must provide next_rays()")
        if (
            not isinstance(self.sensor_frame_id, str)
            or not self.sensor_frame_id
            or self.sensor_frame_id != self.sensor_frame_id.strip()
            or any(character.isspace() for character in self.sensor_frame_id)
        ):
            raise ValueError("sensor_frame_id must be one non-empty stable frame token")

    def __call__(
        self,
        scheduled: ScheduledSensorSample,
        snapshot: object,
    ) -> Mid360FrameSample:
        """Extract from Physics raycast; the ordinary snapshot is only the clock trigger."""

        physics_sequence = _snapshot_uint_or_default(
            snapshot,
            "sequence",
            scheduled.sequence,
        )
        physics_sim_time_ns = _snapshot_uint_or_default(
            snapshot,
            "sim_time_ns",
            scheduled.deadline_ns,
        )
        return self.extract(
            scheduled,
            physics_sequence=physics_sequence,
            physics_sim_time_ns=physics_sim_time_ns,
        )

    def extract(
        self,
        scheduled: ScheduledSensorSample,
        *,
        physics_sequence: int | None = None,
        physics_sim_time_ns: int | None = None,
    ) -> Mid360FrameSample:
        """Raycast and return one scheduler-owned MID-360 sample."""

        self._require_physics_ready()
        self._validate_scheduled(scheduled)
        generation = (scheduled.model_generation, scheduled.reset_generation)
        if generation != self._last_generation:
            _reset_cursor(self.cursor)

        directions, offsets = self.cursor.next_rays()
        direction_rows = _array_rows(directions, field="directions_sensor")
        offset_values = _uint32_tuple(offsets, field="offsets_time_ns")
        if len(direction_rows) != len(offset_values):
            raise Mid360PatternError("MID-360 directions and offsets length mismatch")

        raycast_frame = self.process.raycast(
            sensor_frame_id=self.sensor_frame_id,
            directions_sensor=direction_rows,
            offsets_time_ns=offset_values,
            session_id=scheduled.session_id,
            model_generation=scheduled.model_generation,
            reset_generation=scheduled.reset_generation,
            sequence=scheduled.sequence if physics_sequence is None else physics_sequence,
            sim_time_ns=(
                scheduled.deadline_ns
                if physics_sim_time_ns is None
                else physics_sim_time_ns
            ),
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            reflectivity_proxy=self.reflectivity_proxy,
            unknown_line=self.unknown_line,
        )
        sample = mid360_frame_from_raycast(
            scheduled,
            raycast_frame,
            scan_time_profile=self.scan_time_profile,
        )
        self._last_generation = generation
        self._last_sequence = scheduled.sequence
        self._last_sim_time_ns = scheduled.deadline_ns
        return sample

    def _require_physics_ready(self) -> None:
        if self.physics_state is None:
            return
        state = self.physics_state()
        if state not in {"READY", "RUNNING"}:
            raise Mid360AdapterError("MID-360 raycast requires Physics READY/RUNNING")

    def _validate_scheduled(self, scheduled: ScheduledSensorSample) -> None:
        if not isinstance(scheduled, ScheduledSensorSample):
            raise Mid360PatternError("scheduled must be a ScheduledSensorSample")
        if scheduled.stream.stream_kind != "mid360":
            raise Mid360PatternError("scheduled stream is not mid360")
        generation = (scheduled.model_generation, scheduled.reset_generation)
        if self._last_generation is None:
            return
        if generation < self._last_generation:
            raise Mid360AdapterError("MID-360 scheduled generation moved backward")
        if generation != self._last_generation:
            if scheduled.sequence != 0:
                raise Mid360AdapterError(
                    "MID-360 sequence must restart at 0 after reset/model generation"
                )
            return
        if self._last_sequence is None or scheduled.sequence <= self._last_sequence:
            raise Mid360AdapterError("MID-360 scheduled sequence is not increasing")
        if (
            scheduled.policy is DeadlinePolicy.CATCH_UP
            and scheduled.sequence != self._last_sequence + 1
        ):
            raise Mid360AdapterError("MID-360 scheduled sequence is not contiguous")
        if self._last_sim_time_ns is None or scheduled.deadline_ns <= self._last_sim_time_ns:
            raise Mid360AdapterError("MID-360 scheduled sim_time is not increasing")
