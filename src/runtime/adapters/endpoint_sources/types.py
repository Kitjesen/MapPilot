"""Transport-neutral endpoint source interfaces."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class EndpointEvent:
    """Decoded message received from LingTu by an endpoint service."""

    topic: str
    channel: str
    schema: str
    message: Any
    ts: float


class EndpointService(Protocol):
    """Minimal service surface used by endpoint sources."""

    @property
    def contract(self) -> Any:
        """Return the endpoint contract."""

    def publish_to_lingtu(self, topic: str, msg: Any) -> None:
        """Publish one endpoint-to-LingTu message."""

    def publish_sensor_snapshot(
        self,
        *,
        lidar_scan: Any | None = None,
        imu: Any | None = None,
    ) -> int:
        """Publish available sensor inputs."""

    def publish_localization_snapshot(
        self,
        *,
        odometry: Any | None = None,
        registered_cloud: Any | None = None,
        map_cloud: Any | None = None,
        localization_health: Mapping[str, Any] | None = None,
        localization_quality: float | None = None,
    ) -> int:
        """Publish available localization outputs."""
