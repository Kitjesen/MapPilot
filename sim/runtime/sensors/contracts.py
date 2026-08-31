"""Transport-neutral contracts owned by the simulation Sensor Runtime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction


@dataclass(frozen=True, order=True, slots=True)
class SensorRoute:
    """Exact plan-declared ownership and dispatch route for one stream."""

    owner: str
    source: str
    transport: str


class DeadlinePolicy(str, Enum):
    """Behavior when one clock update crosses multiple sample deadlines."""

    CATCH_UP = "catch_up"
    DROP_INTERMEDIATE = "drop_intermediate"


@dataclass(frozen=True, slots=True)
class SensorStreamPlan:
    """Validated scheduling fields consumed from one SensorPlan stream."""

    stream_kind: str
    instance_id: str
    sensor_id: str
    frame_id: str
    message_type: str
    rate_hz: Fraction
    route: SensorRoute
    raycast_frame_stable_id: str | None = None

    def deadline_ns(self, sequence: int) -> int:
        """Return the drift-free integer deadline for a generation sequence."""

        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        scaled = sequence * 1_000_000_000 * self.rate_hz.denominator
        return (scaled + self.rate_hz.numerator - 1) // self.rate_hz.numerator

    def latest_sequence_at(self, sim_time_ns: int) -> int:
        """Return the latest sequence whose deadline is due at ``sim_time_ns``."""

        return (
            sim_time_ns * self.rate_hz.numerator
            // (1_000_000_000 * self.rate_hz.denominator)
        )


@dataclass(frozen=True, slots=True)
class ScheduledSensorSample:
    """One plan-routed sample request at an exact simulation-clock deadline."""

    session_id: str
    stream: SensorStreamPlan
    model_generation: int
    reset_generation: int
    sequence: int
    deadline_ns: int
    policy: DeadlinePolicy

    @property
    def sensor_id(self) -> str:
        """Return the stable plan-declared sensor ID."""

        return self.stream.sensor_id

    @property
    def route(self) -> SensorRoute:
        """Return the exact owner/source/transport route from the plan."""

        return self.stream.route


@dataclass(frozen=True, slots=True)
class DroppedDeadlineRange:
    """An explicit contiguous range skipped by a latest-only stream."""

    stream: SensorStreamPlan
    model_generation: int
    reset_generation: int
    first_sequence: int
    last_sequence: int

    @property
    def sensor_id(self) -> str:
        """Return the affected stable sensor ID."""

        return self.stream.sensor_id

    @property
    def count(self) -> int:
        """Return the number of skipped sample deadlines."""

        return self.last_sequence - self.first_sequence + 1


@dataclass(frozen=True, slots=True)
class SensorScheduleBatch:
    """All sample requests and explicit drops produced by one clock update."""

    model_generation: int
    reset_generation: int
    sim_time_ns: int
    generation_changed: bool
    samples: tuple[ScheduledSensorSample, ...]
    drops: tuple[DroppedDeadlineRange, ...]

    def for_sensor(self, sensor_id: str) -> tuple[ScheduledSensorSample, ...]:
        """Return this batch's requests for one stable sensor ID."""

        return tuple(sample for sample in self.samples if sample.sensor_id == sensor_id)

    @property
    def routes(self) -> tuple[SensorRoute, ...]:
        """Return the deterministic set of routes represented in this batch."""

        return tuple(sorted({sample.route for sample in self.samples}))

    def for_route(self, route: SensorRoute) -> tuple[ScheduledSensorSample, ...]:
        """Return requests routed to one exact owner/source/transport triple."""

        return tuple(sample for sample in self.samples if sample.route == route)
