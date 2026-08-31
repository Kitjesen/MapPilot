"""Plan loading and simulation-clock scheduling for sensor streams."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any, Mapping

from .contracts import (
    DeadlinePolicy,
    DroppedDeadlineRange,
    ScheduledSensorSample,
    SensorRoute,
    SensorScheduleBatch,
    SensorStreamPlan,
)

_DEFAULT_POLICY_BY_TRANSPORT = {
    "camera_shm": DeadlinePolicy.DROP_INTERMEDIATE,
    "typed_dds": DeadlinePolicy.CATCH_UP,
}
_POLICY_BY_STREAM_KIND = {
    "mid360": DeadlinePolicy.DROP_INTERMEDIATE,
}


class SensorPlanError(ValueError):
    """Raised when a SensorPlan cannot be consumed safely."""


class SensorClockError(RuntimeError):
    """Raised when the simulation clock or reset generation moves backward."""


@dataclass(slots=True)
class _StreamState:
    stream: SensorStreamPlan
    policy: DeadlinePolicy
    next_sequence: int = 0


def _text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SensorPlanError(f"{field} must be a non-empty trimmed string")
    return value


def _rate(value: Any, field: str) -> Fraction:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SensorPlanError(f"{field} must be a positive finite number")
    if isinstance(value, float) and not math.isfinite(value):
        raise SensorPlanError(f"{field} must be a positive finite number")
    rate = Fraction(str(value))
    if rate <= 0 or rate > 1_000_000_000:
        raise SensorPlanError(f"{field} must be a positive finite number")
    return rate


def _policy(value: DeadlinePolicy | str, field: str) -> DeadlinePolicy:
    try:
        return DeadlinePolicy(value)
    except (TypeError, ValueError) as exc:
        choices = ", ".join(policy.value for policy in DeadlinePolicy)
        raise SensorPlanError(f"{field} must be one of: {choices}") from exc


def _strict_json(path: Path) -> dict[str, Any]:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise SensorPlanError(f"{path.name} contains duplicate key {key!r}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=object_from_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(
                SensorPlanError(f"{path.name} contains non-finite value {value}")
            ),
        )
    except SensorPlanError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise SensorPlanError(f"cannot read {path}: {exc}") from exc
    if type(value) is not dict:
        raise SensorPlanError(f"{path.name} must contain a JSON object")
    return value


class SensorRuntime:
    """Consume one SensorPlan and own its stream scheduling state."""

    def __init__(
        self,
        session_id: str,
        streams: tuple[SensorStreamPlan, ...],
        *,
        policy_by_transport: Mapping[str, DeadlinePolicy | str] | None = None,
    ) -> None:
        self._session_id = session_id
        self._streams = streams
        policies = dict(_DEFAULT_POLICY_BY_TRANSPORT)
        if policy_by_transport is not None:
            policies.update(
                {
                    _text(transport, "policy_by_transport key"): _policy(
                        value, f"policy_by_transport.{transport}"
                    )
                    for transport, value in policy_by_transport.items()
                }
            )
        states: list[_StreamState] = []
        for stream in streams:
            policy = _POLICY_BY_STREAM_KIND.get(stream.stream_kind)
            if policy is None:
                try:
                    policy = policies[stream.route.transport]
                except KeyError as exc:
                    raise SensorPlanError(
                        "no deadline policy configured for transport "
                        f"{stream.route.transport!r}"
                    ) from exc
            states.append(_StreamState(stream=stream, policy=policy))
        self._states = states
        self._model_generation: int | None = None
        self._reset_generation: int | None = None
        self._last_sim_time_ns: int | None = None

    @classmethod
    def from_path(
        cls,
        path: Path,
        *,
        policy_by_transport: Mapping[str, DeadlinePolicy | str] | None = None,
    ) -> SensorRuntime:
        """Load one SensorPlan JSON file without consulting package manifests."""

        return cls.from_plan(
            _strict_json(Path(path)), policy_by_transport=policy_by_transport
        )

    @classmethod
    def from_plan(
        cls,
        plan: Mapping[str, Any],
        *,
        policy_by_transport: Mapping[str, DeadlinePolicy | str] | None = None,
    ) -> SensorRuntime:
        """Validate the scheduling view of one compiled SensorPlan."""

        if plan.get("schema") != "lingtu.sim.sensor-plan.v1":
            raise SensorPlanError("sensor plan has an unsupported schema")
        if plan.get("env") != "sim":
            raise SensorPlanError("sensor plan env must be 'sim'")
        session_id = _text(
            plan.get("session_id"), "sensor_plan.session_id"
        )
        raw_streams = plan.get("streams")
        if type(raw_streams) is not dict:
            raise SensorPlanError("sensor_plan.streams must be an object")

        streams: list[SensorStreamPlan] = []
        sensor_ids: set[str] = set()
        for stream_kind, declarations in raw_streams.items():
            kind = _text(stream_kind, "sensor_plan.streams key")
            if not isinstance(declarations, list):
                raise SensorPlanError(f"sensor_plan.streams.{kind} must be a list")
            for index, declaration in enumerate(declarations):
                field = f"sensor_plan.streams.{kind}[{index}]"
                if type(declaration) is not dict:
                    raise SensorPlanError(f"{field} must be an object")
                sensor_id = _text(declaration.get("sensor_id"), f"{field}.sensor_id")
                if sensor_id in sensor_ids:
                    raise SensorPlanError(f"duplicate sensor_id: {sensor_id}")
                sensor_ids.add(sensor_id)
                if kind == "mid360":
                    if "raycast_frame_stable_id" not in declaration:
                        raise SensorPlanError(f"{field} mid360 raycast_frame_stable_id is required")
                    raycast_frame_stable_id = _text(
                        declaration.get("raycast_frame_stable_id"),
                        f"{field}.raycast_frame_stable_id",
                    )
                elif "raycast_frame_stable_id" in declaration:
                    raise SensorPlanError(f"{field} {kind} streams must not declare raycast_frame_stable_id")
                else:
                    raycast_frame_stable_id = None
                streams.append(
                    SensorStreamPlan(
                        stream_kind=kind,
                        instance_id=_text(
                            declaration.get("instance_id"), f"{field}.instance_id"
                        ),
                        sensor_id=sensor_id,
                        frame_id=_text(
                            declaration.get("frame_id"), f"{field}.frame_id"
                        ),
                        message_type=_text(
                            declaration.get("message_type"), f"{field}.message_type"
                        ),
                        rate_hz=_rate(declaration.get("rate_hz"), f"{field}.rate_hz"),
                        route=SensorRoute(
                            owner=_text(declaration.get("owner"), f"{field}.owner"),
                            source=_text(declaration.get("source"), f"{field}.source"),
                            transport=_text(
                                declaration.get("transport"), f"{field}.transport"
                            ),
                        ),
                        raycast_frame_stable_id=raycast_frame_stable_id,
                    )
                )
        return cls(
            session_id,
            tuple(sorted(streams, key=lambda stream: stream.sensor_id)),
            policy_by_transport=policy_by_transport,
        )

    @property
    def streams(self) -> tuple[SensorStreamPlan, ...]:
        """Return the immutable, deterministic stream schedule declarations."""

        return self._streams

    @property
    def session_id(self) -> str:
        """Return the deterministic session identity carried by every request."""

        return self._session_id

    def advance(
        self,
        *,
        sim_time_ns: int,
        reset_generation: int,
        model_generation: int = 0,
    ) -> SensorScheduleBatch:
        """Advance scheduling to one authoritative simulation-clock value."""

        self._validate_clock(sim_time_ns, model_generation, reset_generation)
        generation_changed = (
            model_generation != self._model_generation
            or reset_generation != self._reset_generation
        )
        if generation_changed:
            for state in self._states:
                state.next_sequence = 0

        samples: list[ScheduledSensorSample] = []
        drops: list[DroppedDeadlineRange] = []
        next_sequences: list[int] = []
        for state in self._states:
            latest_sequence = state.stream.latest_sequence_at(sim_time_ns)
            first_sequence = state.next_sequence
            if first_sequence > latest_sequence:
                next_sequences.append(first_sequence)
                continue
            sequences: Iterable[int]
            if state.policy is DeadlinePolicy.CATCH_UP:
                sequences = range(first_sequence, latest_sequence + 1)
            else:
                sequences = (latest_sequence,)
                if first_sequence < latest_sequence:
                    drops.append(
                        DroppedDeadlineRange(
                            stream=state.stream,
                            model_generation=model_generation,
                            reset_generation=reset_generation,
                            first_sequence=first_sequence,
                            last_sequence=latest_sequence - 1,
                        )
                    )
            samples.extend(
                ScheduledSensorSample(
                    session_id=self._session_id,
                    stream=state.stream,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    sequence=sequence,
                    deadline_ns=state.stream.deadline_ns(sequence),
                    policy=state.policy,
                )
                for sequence in sequences
            )
            next_sequences.append(latest_sequence + 1)

        for state, next_sequence in zip(self._states, next_sequences):
            state.next_sequence = next_sequence
        self._model_generation = model_generation
        self._reset_generation = reset_generation
        self._last_sim_time_ns = sim_time_ns
        return SensorScheduleBatch(
            model_generation=model_generation,
            reset_generation=reset_generation,
            sim_time_ns=sim_time_ns,
            generation_changed=generation_changed,
            samples=tuple(
                sorted(
                    samples,
                    key=lambda sample: (
                        sample.deadline_ns,
                        sample.sensor_id,
                        sample.sequence,
                    ),
                )
            ),
            drops=tuple(sorted(drops, key=lambda drop: drop.sensor_id)),
        )

    def _validate_clock(
        self,
        sim_time_ns: int,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        if (
            isinstance(sim_time_ns, bool)
            or not isinstance(sim_time_ns, int)
            or sim_time_ns < 0
        ):
            raise SensorClockError("sim_time_ns must be a non-negative integer")
        if (
            isinstance(model_generation, bool)
            or not isinstance(model_generation, int)
            or model_generation < 0
        ):
            raise SensorClockError("model_generation must be a non-negative integer")
        if (
            isinstance(reset_generation, bool)
            or not isinstance(reset_generation, int)
            or reset_generation < 0
        ):
            raise SensorClockError("reset_generation must be a non-negative integer")
        if self._model_generation is None or self._reset_generation is None:
            return
        if model_generation < self._model_generation:
            raise SensorClockError("model_generation must not move backward")
        if model_generation > self._model_generation:
            return
        if reset_generation < self._reset_generation:
            raise SensorClockError("reset_generation must not move backward")
        if (
            reset_generation == self._reset_generation
            and self._last_sim_time_ns is not None
            and sim_time_ns < self._last_sim_time_ns
        ):
            raise SensorClockError(
                "sim_time_ns must not move backward within one reset generation"
            )
