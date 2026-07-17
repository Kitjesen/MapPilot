"""MuJoCo LiDAR source wrapper."""

from __future__ import annotations

import time
from typing import Any

from runtime.msgs.sensor import PointCloud2
from runtime.runtime_interface import TOPICS, topic_default_frame_id

from ...native import Config, Sample


class Lidar:
    """Read LiDAR frames from a MuJoCo engine."""

    def __init__(self, engine: Any | None = None) -> None:
        self._engine = engine
        self._config = Config()
        self._connected = engine is not None

    def bind(self, engine: Any) -> None:
        self._engine = engine
        self._connected = True

    def connect(self, config: Config) -> None:
        self._config = config
        self._connected = self._engine is not None

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._engine is not None

    def read(self, timeout_ms: int = 0) -> Sample:
        del timeout_ms
        if not self.is_connected():
            return Sample(status={"connected": False})
        points = self._engine.get_lidar_points()
        if points is None or len(points) == 0:
            return Sample(status={"connected": True, "empty": True})
        ts = time.time()
        return Sample(
            scan=PointCloud2(
                points=points,
                frame_id=topic_default_frame_id(TOPICS.lidar_scan),
                ts=ts,
            ),
            timestamp=ts,
            status={"connected": True},
        )
