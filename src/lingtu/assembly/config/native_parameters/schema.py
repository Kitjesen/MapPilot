"""Shared native parameter types."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class ParameterSpec:
    """One bounded parameter accepted by native navigation."""

    name: str
    env_key: str
    parse: Callable[[Any], int | float]
    minimum: int | float
    maximum: int | float

    def coerce(self, value: Any) -> int | float:
        if isinstance(value, bool):
            raise ValueError(f"parameter {self.name} must be a number, not a boolean")
        try:
            parsed = self.parse(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"parameter {self.name} has invalid value {value!r}") from exc
        if isinstance(parsed, float) and not math.isfinite(parsed):
            raise ValueError(f"parameter {self.name} must be finite")
        if parsed < self.minimum or parsed > self.maximum:
            raise ValueError(
                f"parameter {self.name}={parsed!r} is outside "
                f"[{self.minimum}, {self.maximum}]"
            )
        return parsed


@dataclass(frozen=True)
class ResolvedParameter:
    """One final native environment value."""

    value: int | float
    env_key: str


@dataclass(frozen=True)
class RuntimeParameterSet:
    """Validated native parameters."""

    values: Mapping[str, ResolvedParameter]

    def as_dict(self) -> dict[str, int | float]:
        """Return final values keyed by canonical parameter name."""
        return {name: resolved.value for name, resolved in self.values.items()}

    def environment(self) -> dict[str, str]:
        return {
            resolved.env_key: _environment_value(resolved.value)
            for resolved in self.values.values()
        }


def _integer(value: Any) -> int:
    if isinstance(value, float) and not value.is_integer():
        raise ValueError("integer parameter must not contain a fraction")
    return int(value)


def _positive_float(value: Any) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise ValueError("value must be positive")
    return parsed



def _environment_value(value: int | float) -> str:
    return str(value) if isinstance(value, int) else repr(value)
