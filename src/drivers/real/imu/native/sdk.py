"""Real IMU interface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from runtime.msgs.sensor import Imu


@dataclass(frozen=True)
class Sample:
    imu: Imu | None = None
    timestamp: float = 0.0


class Source(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def read(self, timeout_ms: int = 0) -> Sample: ...
