"""Source Interface for MID-360 LiDAR adapters."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol

from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu

from .model import LidarHealth, LidarState


class LidarSource(Protocol):
    """Interface consumed by LidarModule."""

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> Any: ...
    def on_raw_cloud(self, callback: Callable[[Any], None]) -> Any: ...
    def on_imu(self, callback: Callable[[Imu], None]) -> Any: ...
    def connect(self, ip: str | None = None) -> Any: ...
    def disconnect(self) -> None: ...

    @property
    def health(self) -> LidarHealth: ...

    @property
    def ip(self) -> str | None: ...

    @property
    def state(self) -> LidarState: ...


LidarSourceFactory = Callable[..., LidarSource]


def create_lidar_source(
    *,
    ip: str | None = None,
) -> LidarSource:
    """Create the default Livox source."""
    from ..impl.livox.sdk2_stream_source import Sdk2Source

    return Sdk2Source(ip=ip)
