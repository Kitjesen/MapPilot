"""Explicit measurement-error configuration, independent of physics truth."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class RangeNoiseConfig:
    """Zero-mean Gaussian range error, saturated at the sensor range limits.

    This is an experimental measurement model, not a calibrated Livox model.
    Missing configuration leaves noise disabled and preserves existing data.
    """

    enabled: bool = False
    stddev_m: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError("range_noise.enabled must be a boolean")
        try:
            valid_stddev = (
                not isinstance(self.stddev_m, bool)
                and isinstance(self.stddev_m, (int, float))
                and math.isfinite(self.stddev_m)
                and self.stddev_m >= 0
            )
        except OverflowError:
            valid_stddev = False
        if not valid_stddev:
            raise ValueError("range_noise.stddev_m must be a non-negative finite number")
        if type(self.seed) is not int or not 0 <= self.seed < (1 << 64):
            raise ValueError("range_noise.seed must be a uint64 integer")
        object.__setattr__(self, "stddev_m", float(self.stddev_m))

    @classmethod
    def from_mapping(cls, value: Any) -> RangeNoiseConfig:
        """Reject incomplete or misspelled declarations rather than ignoring them."""

        if not isinstance(value, Mapping) or set(value) != {"enabled", "stddev_m", "seed"}:
            raise ValueError("range_noise requires exactly enabled, stddev_m and seed")
        return cls(enabled=value["enabled"], stddev_m=value["stddev_m"], seed=value["seed"])

    def to_dict(self) -> dict[str, bool | float | int]:
        """Return the effective configuration for recorded sample evidence."""

        return {"enabled": self.enabled, "stddev_m": self.stddev_m, "seed": self.seed}
