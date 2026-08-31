"""Strict ingestion of evidence emitted by externally hosted runtimes.

External runtimes never mutate coordinator readiness directly.  They write a
generation-stamped projection into the run log directory; this module validates
that projection and applies only monotonic, per-binding transitions through the
coordinator's public evidence API.
"""

from __future__ import annotations

import json
import math
import stat
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Protocol

from .coordinator import CoordinatorError

_SCHEMA = "lingtu.sim.sensor-readiness-evidence.v1"
_VISUAL_ACTIVE_BASIS = "truth_snapshot_applied_to_visual_bindings"
_FILE_ATTRIBUTE_REPARSE_POINT = 0x400


class _EvidenceTemporarilyUnavailable(RuntimeError):
    """Evidence exists but is not yet safely readable by the coordinator."""


class _EvidenceFileMissing(RuntimeError):
    """Evidence disappeared between path inspection and the file read."""


class _EvidenceFileReplaced(RuntimeError):
    """The atomic producer replaced the named candidate during inspection."""


class EvidenceState(str, Enum):
    """Monotonic external qualification states."""

    PREPARING = "PREPARING"
    PREPARED = "PREPARED"
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"


class ExternalEvidenceTarget(Protocol):
    """Coordinator surface used by the external evidence watcher."""

    def report_binding_prepared(
        self,
        facet: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
    ) -> None: ...

    def report_binding_active(
        self,
        facet: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
    ) -> None: ...

    def report_binding_failed(
        self,
        facet: str,
        *,
        source_id: str,
        reason: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
    ) -> None: ...

    def report_sensor_stream_prepared(
        self,
        sensor_id: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        published_frames: int = 0,
        last_sample_truth_sequence: int = 0,
        last_sample_sim_time_ns: int = 0,
    ) -> None: ...

    def report_sensor_stream_active(
        self,
        sensor_id: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        published_frames: int = 0,
        last_sample_truth_sequence: int = 0,
        last_sample_sim_time_ns: int = 0,
    ) -> None: ...

    def report_sensor_stream_retracted(
        self,
        sensor_id: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        published_frames: int = 0,
        last_sample_truth_sequence: int = 0,
        last_sample_sim_time_ns: int = 0,
    ) -> None: ...

    def report_sensor_stream_failed(
        self,
        sensor_id: str,
        *,
        source_id: str,
        reason: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        published_frames: int = 0,
        last_sample_truth_sequence: int = 0,
        last_sample_sim_time_ns: int = 0,
    ) -> None: ...


def _strict_object(path: Path) -> dict[str, Any]:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise CoordinatorError(f"external evidence contains duplicate key {key!r}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=object_from_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(
                CoordinatorError(f"external evidence contains non-finite value {value}")
            ),
        )
    except CoordinatorError:
        raise
    except FileNotFoundError as exc:
        raise _EvidenceFileMissing(str(exc)) from exc
    except PermissionError as exc:
        raise _EvidenceTemporarilyUnavailable(str(exc)) from exc
    except (OSError, UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise CoordinatorError(f"cannot read external evidence {path}: {exc}") from exc
    if type(value) is not dict:
        raise CoordinatorError("external evidence must contain a JSON object")
    return value


def _text(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise CoordinatorError(f"external evidence {field_name} must be non-empty text")
    return value


def _generation(value: Any, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"external evidence {field_name} must be a non-negative integer")
    return value


def _state(value: Any, field_name: str) -> EvidenceState:
    try:
        return EvidenceState(value)
    except (TypeError, ValueError) as exc:
        raise CoordinatorError(f"external evidence {field_name} has unsupported state {value!r}") from exc


@dataclass(frozen=True)
class StreamEvidence:
    """One external SensorPlan stream qualification."""

    sensor_id: str
    state: EvidenceState
    published_frames: int = 0
    last_sample_truth_sequence: int = 0
    last_sample_sim_time_ns: int = 0
    reason: str | None = None


@dataclass(frozen=True)
class ExternalRuntimeEvidence:
    """Validated evidence projection from one external runtime."""

    session_id: str
    model_generation: int
    reset_generation: int
    source_id: str
    basis: str
    visual_state: EvidenceState
    visual_reason: str | None
    streams: tuple[StreamEvidence, ...]

    @classmethod
    def from_path(cls, path: Path) -> ExternalRuntimeEvidence:
        """Load one strict, generation-stamped evidence file."""

        raw = _strict_object(Path(path))
        allowed = {
            "schema",
            "session_id",
            "model_generation",
            "reset_generation",
            "source_id",
            "basis",
            "visual",
            "sensors",
            "streams",
        }
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise CoordinatorError("external evidence has unknown field(s): " + ", ".join(unknown))
        if raw.get("schema") != _SCHEMA:
            raise CoordinatorError("external evidence schema is unsupported")
        sensors = raw.get("sensors")
        if type(sensors) is not dict:
            raise CoordinatorError("external evidence sensors must be an object")
        sensor_summary_unknown = sorted(set(sensors) - {"camera_streams", "overall"})
        if sensor_summary_unknown:
            raise CoordinatorError(
                "external evidence sensors has unknown field(s): " + ", ".join(sensor_summary_unknown)
            )
        for field_name in ("camera_streams", "overall"):
            _state(sensors.get(field_name), f"sensors.{field_name}")
        visual = raw.get("visual")
        if type(visual) is not dict:
            raise CoordinatorError("external evidence visual must be an object")
        visual_unknown = sorted(set(visual) - {"state", "reason"})
        if visual_unknown:
            raise CoordinatorError("external evidence visual has unknown field(s): " + ", ".join(visual_unknown))
        visual_state = _state(visual.get("state"), "visual.state")
        visual_reason = visual.get("reason")
        if visual_state is EvidenceState.FAILED:
            visual_reason = _text(visual_reason, "visual.reason")
        elif visual_reason is not None:
            raise CoordinatorError("external evidence visual.reason is valid only for FAILED")

        declarations = raw.get("streams")
        if not isinstance(declarations, list):
            raise CoordinatorError("external evidence streams must be a list")
        streams: list[StreamEvidence] = []
        seen: set[str] = set()
        for index, item in enumerate(declarations):
            if type(item) is not dict:
                raise CoordinatorError(f"external evidence streams[{index}] must be an object")
            unknown_item = sorted(
                set(item)
                - {
                    "sensor_id",
                    "state",
                    "published_frames",
                    "last_sample_truth_sequence",
                    "last_sample_sim_time_ns",
                    "reason",
                }
            )
            if unknown_item:
                raise CoordinatorError(
                    f"external evidence streams[{index}] has unknown field(s): " + ", ".join(unknown_item)
                )
            sensor_id = _text(item.get("sensor_id"), f"streams[{index}].sensor_id")
            if sensor_id in seen:
                raise CoordinatorError(f"external evidence contains duplicate sensor_id {sensor_id!r}")
            seen.add(sensor_id)
            stream_state = _state(item.get("state"), f"streams[{index}].state")
            frames = _generation(
                item.get("published_frames", 0),
                f"streams[{index}].published_frames",
            )
            last_truth_sequence = _generation(
                item.get("last_sample_truth_sequence"),
                f"streams[{index}].last_sample_truth_sequence",
            )
            last_sim_time_ns = _generation(
                item.get("last_sample_sim_time_ns"),
                f"streams[{index}].last_sample_sim_time_ns",
            )
            reason = item.get("reason")
            if stream_state is EvidenceState.FAILED:
                reason = _text(reason, f"streams[{index}].reason")
            elif reason is not None:
                raise CoordinatorError(f"external evidence streams[{index}].reason is valid only for FAILED")
            if stream_state is EvidenceState.ACTIVE and frames == 0:
                raise CoordinatorError(f"external ACTIVE stream {sensor_id!r} has no produced frame")
            if frames == 0 and (last_truth_sequence != 0 or last_sim_time_ns != 0):
                raise CoordinatorError(
                    f"external stream {sensor_id!r} has a sample stamp with no produced frame"
                )
            streams.append(
                StreamEvidence(
                    sensor_id,
                    stream_state,
                    frames,
                    last_truth_sequence,
                    last_sim_time_ns,
                    reason,
                )
            )

        return cls(
            session_id=_text(raw.get("session_id"), "session_id"),
            model_generation=_generation(raw.get("model_generation"), "model_generation"),
            reset_generation=_generation(raw.get("reset_generation"), "reset_generation"),
            source_id=_text(raw.get("source_id"), "source_id"),
            basis=_text(raw.get("basis"), "basis"),
            visual_state=visual_state,
            visual_reason=visual_reason,
            streams=tuple(streams),
        )


@dataclass
class ExternalEvidenceWatcher:
    """Apply only newly observed monotonic transitions from one evidence file."""

    path: Path
    session_id: str
    model_generation: int
    reset_generation: int
    expected_source_id: str
    sharing_lock_retry_window_s: float = 0.5
    clock: Callable[[], float] = field(
        default=time.monotonic,
        repr=False,
        compare=False,
    )
    _visual_state: EvidenceState = field(default=EvidenceState.PREPARING, init=False)
    _stream_states: dict[str, EvidenceState] = field(default_factory=dict, init=False)
    _stream_published_frames: dict[str, int] = field(default_factory=dict, init=False)
    _stream_last_truth_sequences: dict[str, int] = field(default_factory=dict, init=False)
    _stream_last_sim_times_ns: dict[str, int] = field(default_factory=dict, init=False)
    _sharing_lock_first_denied_at: float | None = field(default=None, init=False)
    _sharing_lock_identity: tuple[int, int] | None = field(default=None, init=False)
    _unstable_first_observed_at: float | None = field(default=None, init=False)
    _last_file_version: tuple[int, int, int, int, int] | None = field(
        default=None,
        init=False,
    )

    def __post_init__(self) -> None:
        window = self.sharing_lock_retry_window_s
        if isinstance(window, bool) or not isinstance(window, (int, float)) or not math.isfinite(window) or window < 0:
            raise ValueError("sharing_lock_retry_window_s must be a finite non-negative number")
        self.sharing_lock_retry_window_s = float(window)
        if not callable(self.clock):
            raise ValueError("clock must be callable")

    def _clear_sharing_lock_retry(self) -> None:
        self._sharing_lock_first_denied_at = None
        self._sharing_lock_identity = None

    def _clear_unstable_retry(self) -> None:
        self._unstable_first_observed_at = None

    def _defer_unstable_replacement(self, path: Path) -> bool:
        now = self._clock_now()
        first_observed_at = self._unstable_first_observed_at
        if first_observed_at is None:
            self._unstable_first_observed_at = now
            first_observed_at = now
        elif now < first_observed_at:
            raise CoordinatorError("external evidence retry clock moved backwards")
        if now - first_observed_at < self.sharing_lock_retry_window_s:
            return False
        raise CoordinatorError(
            f"external evidence unstable replacement retry window expired: {path}"
        )

    def _clock_now(self) -> float:
        try:
            value = self.clock()
        except Exception as exc:
            raise CoordinatorError(f"external evidence retry clock failed: {exc}") from exc
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise CoordinatorError("external evidence retry clock returned a non-finite value")
        return float(value)

    def _named_file_identity(self, path: Path) -> tuple[int, int] | None:
        try:
            metadata = path.lstat()
        except FileNotFoundError:
            self._clear_sharing_lock_retry()
            self._clear_unstable_retry()
            return None
        except OSError as exc:
            raise CoordinatorError(f"cannot inspect external evidence {path}: {exc}") from exc
        if not stat.S_ISREG(metadata.st_mode) or (
            getattr(metadata, "st_file_attributes", 0) & _FILE_ATTRIBUTE_REPARSE_POINT
        ):
            raise CoordinatorError(f"external evidence path is not a regular file: {path}")
        identity = (metadata.st_dev, metadata.st_ino)
        if self._sharing_lock_identity is not None and identity != self._sharing_lock_identity:
            raise CoordinatorError(f"external evidence identity changed during sharing lock retry: {path}")
        return identity

    def advance_generation(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        """Move the accepted evidence generation after a coordinator reset."""

        _generation(model_generation, "model_generation")
        _generation(reset_generation, "reset_generation")
        generation_changed = model_generation != self.model_generation or reset_generation != self.reset_generation
        if generation_changed:
            self._clear_sharing_lock_retry()
            self._clear_unstable_retry()
            self._last_file_version = None
            self._stream_states.clear()
            self._stream_published_frames.clear()
            self._stream_last_truth_sequences.clear()
            self._stream_last_sim_times_ns.clear()
        if model_generation != self.model_generation:
            self._visual_state = EvidenceState.PREPARING
        self.model_generation = model_generation
        self.reset_generation = reset_generation

    def apply(self, target: ExternalEvidenceTarget) -> bool:
        """Apply new evidence; return false when the producer has not written yet."""

        path = Path(self.path)
        identity = self._named_file_identity(path)
        if identity is None:
            self._last_file_version = None
            return False
        try:
            version = self._named_file_version(path, identity)
        except _EvidenceFileReplaced:
            return self._defer_unstable_replacement(path)
        if version is None:
            self._last_file_version = None
            self._clear_unstable_retry()
            return False
        if version == self._last_file_version:
            self._clear_unstable_retry()
            return True
        try:
            evidence = ExternalRuntimeEvidence.from_path(path)
        except _EvidenceFileMissing:
            self._clear_sharing_lock_retry()
            self._clear_unstable_retry()
            return False
        except _EvidenceTemporarilyUnavailable as exc:
            self._clear_unstable_retry()
            now = self._clock_now()
            first_denied_at = self._sharing_lock_first_denied_at
            if first_denied_at is None:
                self._sharing_lock_first_denied_at = now
                self._sharing_lock_identity = identity
                first_denied_at = now
            elif now < first_denied_at:
                raise CoordinatorError("external evidence retry clock moved backwards") from exc
            if now - first_denied_at < self.sharing_lock_retry_window_s:
                return False
            raise CoordinatorError(f"external evidence sharing lock retry window expired: {path}") from exc
        identity_after_read = self._named_file_identity(path)
        if identity_after_read is None:
            return False
        if identity_after_read != identity:
            return self._defer_unstable_replacement(path)
        try:
            version_after_read = self._named_file_version(path, identity_after_read)
        except _EvidenceFileReplaced:
            return self._defer_unstable_replacement(path)
        if version_after_read is None:
            return False
        if version_after_read != version:
            raise CoordinatorError(f"external evidence changed while reading: {path}")
        self._clear_unstable_retry()
        if evidence.session_id != self.session_id:
            raise CoordinatorError("external evidence session_id mismatch")
        if evidence.source_id != self.expected_source_id:
            raise CoordinatorError("external evidence source_id mismatch")
        if evidence.model_generation > self.model_generation:
            raise CoordinatorError("external evidence model_generation mismatch")
        if evidence.model_generation < self.model_generation:
            self._clear_sharing_lock_retry()
            return False
        if evidence.reset_generation > self.reset_generation:
            raise CoordinatorError("external evidence reset_generation mismatch")
        self._clear_sharing_lock_retry()
        if evidence.reset_generation < self.reset_generation:
            return False
        for stream in evidence.streams:
            current = self._stream_states.get(stream.sensor_id, EvidenceState.PREPARING)
            previous_frames = self._stream_published_frames.get(stream.sensor_id)
            previous_truth_sequence = self._stream_last_truth_sequences.get(
                stream.sensor_id
            )
            previous_sim_time_ns = self._stream_last_sim_times_ns.get(
                stream.sensor_id
            )
            if (
                previous_frames is not None
                and stream.published_frames < previous_frames
            ):
                raise CoordinatorError(
                    f"external stream {stream.sensor_id!r} published_frames moved backward"
                )
            if previous_frames is not None:
                if (
                    previous_truth_sequence is None
                    or previous_sim_time_ns is None
                ):
                    raise CoordinatorError(
                        f"external stream {stream.sensor_id!r} lost its prior sample stamp"
                    )
                if (
                    stream.last_sample_truth_sequence < previous_truth_sequence
                    or stream.last_sample_sim_time_ns < previous_sim_time_ns
                ):
                    raise CoordinatorError(
                        f"external stream {stream.sensor_id!r} sample stamp moved backward"
                    )
                if stream.published_frames == previous_frames and (
                    stream.last_sample_truth_sequence != previous_truth_sequence
                    or stream.last_sample_sim_time_ns != previous_sim_time_ns
                ):
                    raise CoordinatorError(
                        f"external stream {stream.sensor_id!r} changed its sample stamp without a new frame"
                    )
                if stream.published_frames > previous_frames and (
                    stream.last_sample_truth_sequence <= previous_truth_sequence
                    or stream.last_sample_sim_time_ns <= previous_sim_time_ns
                ):
                    raise CoordinatorError(
                        f"external stream {stream.sensor_id!r} new frame lacks a newer sample stamp"
                    )
            if stream.state is EvidenceState.FAILED and current is not EvidenceState.FAILED:
                if stream.reason is None:
                    raise CoordinatorError(
                        f"external stream {stream.sensor_id!r} missing failure reason"
                    )
                target.report_sensor_stream_failed(
                    stream.sensor_id,
                    reason=stream.reason,
                    published_frames=stream.published_frames,
                    last_sample_truth_sequence=stream.last_sample_truth_sequence,
                    last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._stream_states[stream.sensor_id] = stream.state
                self._stream_published_frames[stream.sensor_id] = stream.published_frames
                self._stream_last_truth_sequences[stream.sensor_id] = (
                    stream.last_sample_truth_sequence
                )
                self._stream_last_sim_times_ns[stream.sensor_id] = (
                    stream.last_sample_sim_time_ns
                )
            elif (
                stream.state in {EvidenceState.PREPARING, EvidenceState.PREPARED}
                and current is EvidenceState.ACTIVE
            ):
                target.report_sensor_stream_retracted(
                    stream.sensor_id,
                    published_frames=stream.published_frames,
                    last_sample_truth_sequence=stream.last_sample_truth_sequence,
                    last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._stream_states[stream.sensor_id] = stream.state
                self._stream_published_frames[stream.sensor_id] = stream.published_frames
                self._stream_last_truth_sequences[stream.sensor_id] = (
                    stream.last_sample_truth_sequence
                )
                self._stream_last_sim_times_ns[stream.sensor_id] = (
                    stream.last_sample_sim_time_ns
                )
            elif stream.state is EvidenceState.PREPARED and current is EvidenceState.PREPARING:
                target.report_sensor_stream_prepared(
                    stream.sensor_id,
                    published_frames=stream.published_frames,
                    last_sample_truth_sequence=stream.last_sample_truth_sequence,
                    last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._stream_states[stream.sensor_id] = stream.state
                self._stream_published_frames[stream.sensor_id] = stream.published_frames
                self._stream_last_truth_sequences[stream.sensor_id] = (
                    stream.last_sample_truth_sequence
                )
                self._stream_last_sim_times_ns[stream.sensor_id] = (
                    stream.last_sample_sim_time_ns
                )
            elif stream.state is EvidenceState.ACTIVE and current is not EvidenceState.ACTIVE:
                if current is EvidenceState.PREPARING:
                    target.report_sensor_stream_prepared(
                        stream.sensor_id,
                        published_frames=stream.published_frames,
                        last_sample_truth_sequence=stream.last_sample_truth_sequence,
                        last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                        source_id=evidence.source_id,
                        session_id=evidence.session_id,
                        model_generation=evidence.model_generation,
                        reset_generation=evidence.reset_generation,
                    )
                target.report_sensor_stream_active(
                    stream.sensor_id,
                    published_frames=stream.published_frames,
                    last_sample_truth_sequence=stream.last_sample_truth_sequence,
                    last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._stream_states[stream.sensor_id] = stream.state
                self._stream_published_frames[stream.sensor_id] = stream.published_frames
                self._stream_last_truth_sequences[stream.sensor_id] = (
                    stream.last_sample_truth_sequence
                )
                self._stream_last_sim_times_ns[stream.sensor_id] = (
                    stream.last_sample_sim_time_ns
                )
            elif (
                stream.state is EvidenceState.ACTIVE
                and previous_frames != stream.published_frames
            ):
                target.report_sensor_stream_active(
                    stream.sensor_id,
                    published_frames=stream.published_frames,
                    last_sample_truth_sequence=stream.last_sample_truth_sequence,
                    last_sample_sim_time_ns=stream.last_sample_sim_time_ns,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._stream_published_frames[stream.sensor_id] = stream.published_frames
                self._stream_last_truth_sequences[stream.sensor_id] = (
                    stream.last_sample_truth_sequence
                )
                self._stream_last_sim_times_ns[stream.sensor_id] = (
                    stream.last_sample_sim_time_ns
                )

        # A rendered camera frame proves the camera stream, not the Visual
        # binding. Visual ACTIVE is admitted only with the explicit snapshot-
        # application basis from the Visual Runtime.
        if evidence.basis == _VISUAL_ACTIVE_BASIS:
            current = self._visual_state
            if evidence.visual_state is EvidenceState.FAILED and current is not EvidenceState.FAILED:
                if evidence.visual_reason is None:
                    raise CoordinatorError("external visual evidence missing failure reason")
                target.report_binding_failed(
                    "visual",
                    reason=evidence.visual_reason,
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._visual_state = evidence.visual_state
            elif evidence.visual_state is EvidenceState.PREPARED and current is EvidenceState.PREPARING:
                target.report_binding_prepared(
                    "visual",
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._visual_state = evidence.visual_state
            elif evidence.visual_state is EvidenceState.ACTIVE and current is not EvidenceState.ACTIVE:
                if current is EvidenceState.PREPARING:
                    target.report_binding_prepared(
                        "visual",
                        source_id=evidence.source_id,
                        session_id=evidence.session_id,
                        model_generation=evidence.model_generation,
                        reset_generation=evidence.reset_generation,
                    )
                target.report_binding_active(
                    "visual",
                    source_id=evidence.source_id,
                    session_id=evidence.session_id,
                    model_generation=evidence.model_generation,
                    reset_generation=evidence.reset_generation,
                )
                self._visual_state = evidence.visual_state
        self._last_file_version = version_after_read
        return True

    def _named_file_version(
        self,
        path: Path,
        identity: tuple[int, int],
    ) -> tuple[int, int, int, int, int] | None:
        try:
            metadata = path.lstat()
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise CoordinatorError(f"cannot inspect external evidence {path}: {exc}") from exc
        current_identity = (metadata.st_dev, metadata.st_ino)
        if current_identity != identity:
            raise _EvidenceFileReplaced(path)
        if not stat.S_ISREG(metadata.st_mode) or (
            getattr(metadata, "st_file_attributes", 0) & _FILE_ATTRIBUTE_REPARSE_POINT
        ):
            raise CoordinatorError(f"external evidence path is not a regular file: {path}")
        return (
            metadata.st_dev,
            metadata.st_ino,
            metadata.st_size,
            metadata.st_mtime_ns,
            metadata.st_ctime_ns,
        )


__all__ = [
    "EvidenceState",
    "ExternalEvidenceWatcher",
    "ExternalRuntimeEvidence",
    "StreamEvidence",
]
