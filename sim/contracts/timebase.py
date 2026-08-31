"""Exact periodic-sensor compatibility with the authoritative physics clock."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from fractions import Fraction
from typing import Any


def _positive_numeric(value: Any, field: str) -> int | float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0
    ):
        raise ValueError(f"{field} must be positive finite numeric data")
    return value


def _positive_fraction(value: Any, field: str) -> Fraction:
    value = _positive_numeric(value, field)
    return Fraction(str(value))


def _scalar(value: Fraction) -> int | str:
    return (
        value.numerator
        if value.denominator == 1
        else f"{value.numerator}/{value.denominator}"
    )


@dataclass(frozen=True, slots=True)
class PhysicsSensorTimebaseViolation:
    """One exact-snapshot stream that cannot land on the physics timebase."""

    sensor_id: str
    rate_hz: int | float
    timestep_s: int | float
    period_ns: int | str
    timestep_ns: int | str

    def details(self) -> dict[str, int | float | str]:
        """Return stable machine-facing diagnostics shared by both callers."""

        return {
            "period_ns": self.period_ns,
            "rate_hz": self.rate_hz,
            "required_relation": (
                "sensor_period_ns % physics_timestep_ns == 0"
            ),
            "sensor_id": self.sensor_id,
            "timestep_ns": self.timestep_ns,
        }


def physics_sensor_timebase_violation(
    *,
    timestep_s: object,
    streams: Iterable[Mapping[str, Any]],
) -> PhysicsSensorTimebaseViolation | None:
    """Return the first deterministic physics-stream timebase violation."""

    timestep_value = _positive_numeric(timestep_s, "physics timestep_s")
    timestep = _positive_fraction(timestep_value, "physics timestep_s")
    timestep_ns = timestep * 1_000_000_000
    physics_streams = sorted(
        (stream for stream in streams if stream.get("owner") == "physics"),
        key=lambda stream: str(stream.get("sensor_id", "")),
    )
    for stream in physics_streams:
        sensor_id = stream.get("sensor_id")
        if not isinstance(sensor_id, str) or not sensor_id or sensor_id != sensor_id.strip():
            raise ValueError("physics sensor_id must be non-empty trimmed text")
        rate_value = stream.get("rate_hz")
        rate_source = _positive_numeric(rate_value, f"{sensor_id}.rate_hz")
        rate = _positive_fraction(rate_source, f"{sensor_id}.rate_hz")
        period_ns = Fraction(1_000_000_000, 1) / rate
        if (
            timestep_ns.denominator == 1
            and period_ns.denominator == 1
            and period_ns.numerator % timestep_ns.numerator == 0
        ):
            continue
        return PhysicsSensorTimebaseViolation(
            sensor_id=sensor_id,
            rate_hz=rate_source,
            timestep_s=timestep_value,
            period_ns=_scalar(period_ns),
            timestep_ns=_scalar(timestep_ns),
        )
    return None
