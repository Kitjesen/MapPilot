"""Shared, unit-labelled measurement noise; never a physics perturbation."""

from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass

import numpy as np

from .samples import SensorSampleStamp


@dataclass(frozen=True, slots=True)
class MeasurementNoise:
    unit: str
    enabled: bool = False
    stddev: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.unit, str) or not self.unit or self.unit != self.unit.strip():
            raise ValueError("measurement noise requires an explicit unit")
        if type(self.enabled) is not bool:
            raise ValueError("measurement noise enabled must be a boolean")
        if (isinstance(self.stddev, bool) or not isinstance(self.stddev, (float, int))
                or not math.isfinite(self.stddev) or self.stddev < 0):
            raise ValueError("measurement noise stddev must be finite and non-negative")
        if type(self.seed) is not int or not 0 <= self.seed < 1 << 64:
            raise ValueError("measurement noise seed must be a uint64")

    def generator(self, stamp: SensorSampleStamp) -> np.random.Generator:
        """Stable per sample, sensor and generation, independent of call order."""
        if not isinstance(stamp, SensorSampleStamp):
            raise TypeError("noise requires a SensorSampleStamp")
        seed = random.Random(f"{self.seed}:{self.unit}:{stamp!r}").getrandbits(128)
        return np.random.default_rng(seed)

    def to_dict(self) -> dict:
        return asdict(self)
