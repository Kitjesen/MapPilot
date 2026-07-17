"""Simulation LiDAR interface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from runtime.msgs.sensor import Imu, PointCloud2


@dataclass(frozen=True)
class Config:
    name: str = "mujoco_lidar"


@dataclass(frozen=True)
class Sample:
    scan: PointCloud2 | None = None
    raw_scan: Any | None = None
    imu: Imu | None = None
    status: dict[str, Any] | None = None
    timestamp: float = 0.0


class Source(Protocol):
    def connect(self, config: Config) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def read(self, timeout_ms: int = 0) -> Sample: ...
