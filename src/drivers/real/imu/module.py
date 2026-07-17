"""Real IMU runtime module facade.

The current field IMU stream is synchronized inside the Livox MID-360 LiDAR
source. This facade makes that ownership explicit without opening another
hardware reader or publishing duplicate IMU samples.
"""

from __future__ import annotations

from typing import Any

from runtime.contracts import IMU_BACKEND_LIVOX, IMU_ROLE
from runtime.module import Module
from runtime.msgs.sensor import Imu
from runtime.registry import register
from runtime.stream import Out


@register(IMU_ROLE, IMU_BACKEND_LIVOX, description="Livox MID-360 IMU facade")
class ImuModule(Module, layer=1):
    """Document the field IMU source without double-publishing it."""

    imu: Out[Imu]
    alive: Out[bool]
    runtime_id = IMU_ROLE

    def __init__(
        self,
        *,
        source_role: str = "lidar",
        source_port: str = "imu",
        publish_samples: bool = False,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._source_role = source_role
        self._source_port = source_port
        self._publish_samples = bool(publish_samples)

    def setup(self) -> None:
        self.alive.publish(False)

    def health(self) -> dict[str, object]:
        info = super().port_summary()
        info["role"] = IMU_ROLE
        info["backend"] = IMU_BACKEND_LIVOX
        info["source"] = f"{self._source_role}.{self._source_port}"
        info["status"] = "facade"
        info["connected"] = False
        info["samples"] = 0
        info["stale_ms"] = None
        info["error"] = ""
        info["publishes_samples"] = self._publish_samples
        info["note"] = "field IMU is synchronized and published by the lidar role"
        return info
