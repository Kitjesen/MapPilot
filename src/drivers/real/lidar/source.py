"""Source Interface for MID-360 LiDAR adapters."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol

from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import TOPICS


class LidarHealthLike(Protocol):
    def to_dict(self) -> dict[str, Any]: ...


class LidarStateLike(Protocol):
    value: str


class LidarSource(Protocol):
    """Interface consumed by LidarModule."""

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> Any: ...
    def on_raw_cloud(self, callback: Callable[[Any], None]) -> Any: ...
    def on_imu(self, callback: Callable[[Imu], None]) -> Any: ...
    def connect(self, ip: str | None = None) -> Any: ...
    def disconnect(self) -> None: ...

    @property
    def health(self) -> LidarHealthLike: ...

    @property
    def ip(self) -> str | None: ...

    @property
    def state(self) -> LidarStateLike: ...


LidarSourceFactory = Callable[..., LidarSource]


def create_lidar_source(
    *,
    ip: str | None = None,
    scan_topic: str = TOPICS.lidar_scan,
    imu_topic: str = TOPICS.imu,
    start_driver: bool = False,
) -> LidarSource:
    """Create the default Livox source."""
    if not start_driver:
        from .sdk2_stream_source import Sdk2Source

        return Sdk2Source(ip=ip)

    from .lidar import Lidar

    return Lidar(
        ip=ip,
        scan_topic=scan_topic,
        imu_topic=imu_topic,
        start_driver=start_driver,
    )
