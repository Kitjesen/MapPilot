"""Per-stream qualification state for the simulation Sensor Runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Iterable, Mapping, cast

from .contracts import SensorStreamPlan
from .runtime import SensorRuntime

_PRESERVE_FAILURE_REASON = object()


class SensorStreamState(str, Enum):
    """Qualification state of one compiled SensorPlan stream."""

    UNBOUND = "UNBOUND"
    PREPARED = "PREPARED"
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"


class SensorReadinessError(RuntimeError):
    """Raised when stream qualification fails closed."""

    def __init__(
        self,
        message: str,
        *,
        reasons: Mapping[str, str] | None = None,
    ) -> None:
        self.reasons = MappingProxyType(dict(reasons or {}))
        super().__init__(message)


@dataclass(frozen=True)
class SensorStreamQualification:
    """Immutable state and generation stamp for one compiled stream."""

    stream_id: str
    stream_kind: str
    source: str
    owner: str
    transport: str
    state: SensorStreamState = SensorStreamState.UNBOUND
    model_generation: int = 0
    reset_generation: int = 0
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        _validate_text(self.stream_id, "stream_id")
        _validate_text(self.stream_kind, "stream_kind")
        _validate_text(self.source, "source")
        _validate_text(self.owner, "owner")
        _validate_text(self.transport, "transport")
        if not isinstance(self.state, SensorStreamState):
            raise ValueError("state must be a SensorStreamState")
        _validate_generation(self.model_generation, "model_generation")
        _validate_generation(self.reset_generation, "reset_generation")
        if self.state is SensorStreamState.FAILED:
            if (
                not isinstance(self.failure_reason, str)
                or not self.failure_reason.strip()
            ):
                raise ValueError("FAILED stream requires a non-empty failure_reason")
        elif self.failure_reason is not None:
            raise ValueError("failure_reason is only valid for FAILED streams")

    def to_manifest(self, *, required: bool) -> dict[str, Any]:
        """Return a JSON-serializable manifest entry."""

        return {
            "stream_id": self.stream_id,
            "stream_kind": self.stream_kind,
            "required": required,
            "state": self.state.value,
            "source": self.source,
            "owner": self.owner,
            "transport": self.transport,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "failure_reason": self.failure_reason,
        }


@dataclass(frozen=True)
class SensorReadiness:
    """Immutable readiness model for all streams in one SensorPlan generation."""

    required_stream_ids: frozenset[str]
    _streams: Mapping[str, SensorStreamQualification] = field(
        repr=False,
        compare=True,
    )
    model_generation: int = 0
    reset_generation: int = 0

    def __post_init__(self) -> None:
        _validate_generation(self.model_generation, "model_generation")
        _validate_generation(self.reset_generation, "reset_generation")
        streams = dict(self._streams)
        if not streams:
            raise ValueError("streams must contain at least one SensorPlan stream")
        for stream_id, qualification in streams.items():
            if (
                not isinstance(qualification, SensorStreamQualification)
                or qualification.stream_id != stream_id
            ):
                raise ValueError("streams must contain matching qualifications")
            if (
                qualification.model_generation != self.model_generation
                or qualification.reset_generation != self.reset_generation
            ):
                raise ValueError("stream generation does not match readiness generation")
        required = _normalize_stream_ids(
            self.required_stream_ids,
            "required_stream_ids",
        )
        unknown = sorted(required - set(streams))
        if unknown:
            raise ValueError(f"required_stream_ids contains unknown stream(s): {unknown}")
        object.__setattr__(self, "required_stream_ids", required)
        object.__setattr__(self, "_streams", MappingProxyType(streams))

    @classmethod
    def from_plan(
        cls,
        plan: Mapping[str, Any],
        *,
        required_stream_ids: Iterable[str] | None = None,
        optional_stream_ids: Iterable[str] = (),
        sensors_required: bool = True,
        model_generation: int = 0,
        reset_generation: int = 0,
    ) -> SensorReadiness:
        """Build readiness from a compiled SensorPlan stream list."""

        return cls.from_runtime(
            SensorRuntime.from_plan(plan),
            required_stream_ids=required_stream_ids,
            optional_stream_ids=optional_stream_ids,
            sensors_required=sensors_required,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    @classmethod
    def from_runtime(
        cls,
        runtime: SensorRuntime,
        *,
        required_stream_ids: Iterable[str] | None = None,
        optional_stream_ids: Iterable[str] = (),
        sensors_required: bool = True,
        model_generation: int = 0,
        reset_generation: int = 0,
    ) -> SensorReadiness:
        """Build readiness from an already validated SensorRuntime."""

        streams = {
            stream.sensor_id: _qualification_from_stream(
                stream,
                model_generation=model_generation,
                reset_generation=reset_generation,
            )
            for stream in runtime.streams
        }
        optional = _normalize_stream_ids(optional_stream_ids, "optional_stream_ids")
        unknown_optional = sorted(optional - set(streams))
        if unknown_optional:
            raise ValueError(
                f"optional_stream_ids contains unknown stream(s): {unknown_optional}"
            )
        if required_stream_ids is None:
            required = frozenset(streams) - optional if sensors_required else frozenset()
        else:
            required = _normalize_stream_ids(required_stream_ids, "required_stream_ids")
            overlap = sorted(required & optional)
            if overlap:
                raise ValueError(f"streams cannot be both required and optional: {overlap}")
        return cls(
            required,
            streams,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    @property
    def streams(self) -> Mapping[str, SensorStreamQualification]:
        """Read-only qualification records keyed by compiled stream ID."""

        return self._streams

    @property
    def is_ready(self) -> bool:
        """Whether every required stream is ACTIVE for this generation."""

        return not self.blocking_reasons

    @property
    def blocking_reasons(self) -> Mapping[str, str]:
        """Return why each required stream currently blocks readiness."""

        reasons: dict[str, str] = {}
        for stream_id in sorted(self.required_stream_ids):
            stream = self._streams[stream_id]
            if stream.model_generation != self.model_generation:
                reasons[stream_id] = "activation belongs to an older model_generation"
            elif stream.reset_generation != self.reset_generation:
                reasons[stream_id] = "activation belongs to an older reset_generation"
            elif stream.state is SensorStreamState.FAILED:
                reasons[stream_id] = stream.failure_reason or "stream failed"
            elif stream.state is not SensorStreamState.ACTIVE:
                reasons[stream_id] = f"stream is {stream.state.value}"
        return MappingProxyType(reasons)

    @property
    def failures(self) -> Mapping[str, str]:
        """Failure reasons for failed streams, including optional streams."""

        return MappingProxyType(
            {
                stream_id: stream.failure_reason
                for stream_id, stream in self._streams.items()
                if stream.state is SensorStreamState.FAILED
                and stream.failure_reason is not None
            }
        )

    def state(self, stream_id: str) -> SensorStreamState:
        """Return the current state of one compiled stream."""

        return self._stream(stream_id).state

    def failure_reason(self, stream_id: str) -> str | None:
        """Return the recorded failure reason for one stream, when present."""

        return self._stream(stream_id).failure_reason

    def require_ready(self) -> None:
        """Raise with stream-specific reasons unless required streams are ready."""

        if not self.is_ready:
            reasons = dict(self.blocking_reasons)
            detail = "; ".join(
                f"{stream_id}: {reason}" for stream_id, reason in reasons.items()
            )
            raise SensorReadinessError(
                "sensor streams are not READY; required streams are not ACTIVE "
                f"({detail})",
                reasons=reasons,
            )

    def mark_prepared(
        self,
        stream_id: str,
        *,
        source: str,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> SensorReadiness:
        """Return a copy with one stream in PREPARED for this generation."""

        return self._transition(
            stream_id,
            SensorStreamState.PREPARED,
            source=source,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def mark_active(
        self,
        stream_id: str,
        *,
        source: str,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> SensorReadiness:
        """Return a copy with one prepared stream promoted to ACTIVE."""

        return self._transition(
            stream_id,
            SensorStreamState.ACTIVE,
            source=source,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def mark_failed(
        self,
        stream_id: str,
        reason: str,
        *,
        source: str,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> SensorReadiness:
        """Return a copy with one stream failed and its reason retained."""

        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("failure reason must be a non-empty string")
        return self._transition(
            stream_id,
            SensorStreamState.FAILED,
            source=source,
            model_generation=model_generation,
            reset_generation=reset_generation,
            failure_reason=reason.strip(),
        )

    def with_generations(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> SensorReadiness:
        """Advance generation stamps for the current SensorPlan stream set.

        A reset of the same model keeps prepared/active/failed bindings and
        stamps them with the new reset generation.  A model rebuild invalidates
        every runtime binding because source indexes and dense runtime handles
        may no longer describe the same stream.
        """

        _validate_generation(model_generation, "model_generation")
        _validate_generation(reset_generation, "reset_generation")
        if (model_generation, reset_generation) == (
            self.model_generation,
            self.reset_generation,
        ):
            return self
        if model_generation != self.model_generation:
            rebound = {
                stream_id: _replace_stream(
                    stream,
                    state=SensorStreamState.UNBOUND,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    failure_reason=None,
                )
                for stream_id, stream in self._streams.items()
            }
        else:
            rebound = {
                stream_id: _replace_stream(
                    stream,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
                for stream_id, stream in self._streams.items()
            }
        return SensorReadiness(
            self.required_stream_ids,
            rebound,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def to_manifest(self) -> dict[str, Any]:
        """Return a JSON-serializable readiness manifest snapshot."""

        return {
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "is_ready": self.is_ready,
            "required_stream_ids": sorted(self.required_stream_ids),
            "blocking_reasons": dict(self.blocking_reasons),
            "failures": dict(self.failures),
            "streams": {
                stream_id: self._streams[stream_id].to_manifest(
                    required=stream_id in self.required_stream_ids
                )
                for stream_id in sorted(self._streams)
            },
        }

    def _transition(
        self,
        stream_id: str,
        target: SensorStreamState,
        *,
        source: str,
        model_generation: int | None,
        reset_generation: int | None,
        failure_reason: str | None = None,
    ) -> SensorReadiness:
        stream = self._stream(stream_id)
        _validate_source(stream, source)
        _check_generation(self.model_generation, model_generation, "model_generation")
        _check_generation(self.reset_generation, reset_generation, "reset_generation")
        if target is SensorStreamState.PREPARED and stream.state not in {
            SensorStreamState.UNBOUND,
            SensorStreamState.PREPARED,
            SensorStreamState.ACTIVE,
        }:
            raise SensorReadinessError(
                f"{stream_id} must be UNBOUND before PREPARED"
            )
        if target is SensorStreamState.ACTIVE and stream.state not in {
            SensorStreamState.PREPARED,
            SensorStreamState.ACTIVE,
        }:
            raise SensorReadinessError(
                f"{stream_id} must be PREPARED before ACTIVE"
            )
        updated = dict(self._streams)
        updated[stream_id] = _replace_stream(
            stream,
            state=target,
            failure_reason=failure_reason,
        )
        return SensorReadiness(
            self.required_stream_ids,
            updated,
            model_generation=self.model_generation,
            reset_generation=self.reset_generation,
        )

    def _stream(self, stream_id: str) -> SensorStreamQualification:
        _validate_text(stream_id, "stream_id")
        try:
            return self._streams[stream_id]
        except KeyError as exc:
            raise SensorReadinessError(f"unknown stream: {stream_id!r}") from exc


def _qualification_from_stream(
    stream: SensorStreamPlan,
    *,
    model_generation: int,
    reset_generation: int,
) -> SensorStreamQualification:
    return SensorStreamQualification(
        stream_id=stream.sensor_id,
        stream_kind=stream.stream_kind,
        source=stream.route.source,
        owner=stream.route.owner,
        transport=stream.route.transport,
        model_generation=model_generation,
        reset_generation=reset_generation,
    )


def _replace_stream(
    stream: SensorStreamQualification,
    *,
    state: SensorStreamState | None = None,
    model_generation: int | None = None,
    reset_generation: int | None = None,
    failure_reason: str | None | object = _PRESERVE_FAILURE_REASON,
) -> SensorStreamQualification:
    next_state = stream.state if state is None else state
    next_failure_reason: str | None = (
        stream.failure_reason
        if failure_reason is _PRESERVE_FAILURE_REASON
        else cast(str | None, failure_reason)
    )
    return SensorStreamQualification(
        stream_id=stream.stream_id,
        stream_kind=stream.stream_kind,
        source=stream.source,
        owner=stream.owner,
        transport=stream.transport,
        state=next_state,
        model_generation=(
            stream.model_generation
            if model_generation is None
            else model_generation
        ),
        reset_generation=(
            stream.reset_generation
            if reset_generation is None
            else reset_generation
        ),
        failure_reason=(
            next_failure_reason if next_state is SensorStreamState.FAILED else None
        ),
    )


def _normalize_stream_ids(values: Iterable[str], field_name: str) -> frozenset[str]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{field_name} must be an iterable of stream IDs")
    try:
        stream_ids = tuple(_validate_text(value, f"{field_name} item") for value in values)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be an iterable of stream IDs") from exc
    if len(set(stream_ids)) != len(stream_ids):
        raise ValueError(f"{field_name} must contain unique stream IDs")
    return frozenset(stream_ids)


def _validate_text(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field_name} must be a non-empty trimmed string")
    return value


def _validate_generation(value: int, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")


def _check_generation(expected: int, actual: int | None, field_name: str) -> None:
    if actual is not None:
        _validate_generation(actual, field_name)
        if actual != expected:
            raise SensorReadinessError(
                f"{field_name} {actual} does not match current {field_name} {expected}"
            )


def _validate_source(stream: SensorStreamQualification, source: str) -> None:
    _validate_text(source, "source")
    if source != stream.source:
        raise SensorReadinessError(
            f"source {source!r} does not own stream {stream.stream_id!r}; "
            f"expected {stream.source!r}"
        )
