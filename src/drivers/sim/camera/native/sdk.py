"""Simulation camera interface.

This mirrors the real camera native interface shape without depending on a
specific simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from runtime.msgs.sensor import CameraIntrinsics, Image, PointCloud2


@dataclass(frozen=True)
class Config:
    name: str = "front_camera"
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass(frozen=True)
class Info:
    name: str
    backend: str = "sim"


@dataclass(frozen=True)
class Sample:
    color: Image | None = None
    depth: Image | None = None
    points: PointCloud2 | None = None
    imu: Any | None = None
    intrinsics: CameraIntrinsics | None = None
    status: dict[str, Any] | None = None
    timestamp: float = 0.0


class Source(Protocol):
    def connect(self, config: Config) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def info(self) -> Info: ...
    def read(self, timeout_ms: int = 0) -> Sample: ...
