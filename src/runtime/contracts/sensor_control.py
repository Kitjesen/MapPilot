"""Typed source-owner sensor commands, separate from motion and task control."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from message.generated.enums import SensorControlKind


class SensorControlConflict(ValueError):
    """A previously sent request ID was reused with different control content."""


@dataclass(frozen=True)
class SensorControlRequest:
    request_id: str
    product_session_id: str
    session_id: str
    sensor_id: str
    model_generation: int
    reset_generation: int
    deadline_s: float
    operation: SensorControlKind
    enabled: bool = False
    rate_hz: float = 0.
    noise_stddev: float = 0.
    noise_seed: int = 0
    noise_unit: str = ""
    pan_deg: float = 0.
    tilt_deg: float = 0.

    def __post_init__(self) -> None:
        for name in ("request_id", "product_session_id", "session_id", "sensor_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value or value != value.strip() or "\0" in value or len(value.encode()) > 255:
                raise ValueError(f"{name} must be nonempty bounded text")
        for name in ("model_generation", "reset_generation", "noise_seed"):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value < 2**64:
                raise ValueError(f"{name} must be uint64")
        for name in ("deadline_s", "rate_hz", "noise_stddev", "pan_deg", "tilt_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite numeric data")
        if self.deadline_s <= 0 or type(self.enabled) is not bool:
            raise ValueError("deadline must be positive and enabled must be boolean")
        if isinstance(self.operation, bool):
            raise ValueError("operation must be SensorControlKind")
        object.__setattr__(self, "operation", SensorControlKind(self.operation))
        if not isinstance(self.noise_unit, str) or "\0" in self.noise_unit or len(self.noise_unit.encode()) > 15:
            raise ValueError("noise_unit exceeds native bounds")


@dataclass(frozen=True)
class SensorControlResult:
    request_id: str
    product_session_id: str
    session_id: str
    sensor_id: str
    model_generation: int
    reset_generation: int
    applied_sim_time_ns: int
    applied: bool
    reason: str
    settings: dict[str, Any] = field(default_factory=dict)
