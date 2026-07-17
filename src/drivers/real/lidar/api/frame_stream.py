"""Pure Python frame stream state for LiDAR sources."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


@dataclass
class LidarFrameMetrics:
    total_frames: int = 0
    total_points: int = 0
    last_frame_time: float = 0.0
    last_frame_points: int = 0


class _FPSCounter:
    """Sliding-window frame rate estimator."""

    def __init__(self, window: float = 1.0) -> None:
        self._window = window
        self._timestamps: list[float] = []

    def tick(self) -> float:
        now = time.monotonic()
        self._timestamps.append(now)
        cutoff = now - self._window
        self._timestamps = [t for t in self._timestamps if t > cutoff]
        return float(len(self._timestamps)) / self._window

    def fps(self) -> float:
        now = time.monotonic()
        cutoff = now - self._window
        active = [t for t in self._timestamps if t > cutoff]
        return float(len(active)) / self._window


class LidarFrameStream:
    """Store latest LiDAR frames and fan out callbacks.

    This class has no ROS, DDS, NativeModule, or robot configuration dependency.
    Adapters ingest normalized numpy point clouds and core IMU messages.
    """

    def __init__(self) -> None:
        self._cloud_lock = threading.Lock()
        self._latest_cloud: np.ndarray | None = None
        self._latest_raw_cloud: Any = None
        self._latest_imu: Any = None
        self._cloud_event = threading.Event()
        self._cloud_callbacks: list[Callable[[np.ndarray], None]] = []
        self._raw_cloud_callbacks: list[Callable[[Any], None]] = []
        self._imu_callbacks: list[Callable[[Any], None]] = []
        self._fps_counter = _FPSCounter()
        self.metrics = LidarFrameMetrics()

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> LidarFrameStream:
        self._cloud_callbacks.append(callback)
        return self

    def on_raw_cloud(self, callback: Callable[[Any], None]) -> LidarFrameStream:
        self._raw_cloud_callbacks.append(callback)
        return self

    def on_imu(self, callback: Callable[[Any], None]) -> LidarFrameStream:
        self._imu_callbacks.append(callback)
        return self

    def ingest_point_frame(self, frame: Any) -> bool:
        if frame is None:
            return False

        with self._cloud_lock:
            self._latest_raw_cloud = frame

        for callback in self._raw_cloud_callbacks:
            try:
                callback(frame)
            except Exception as exc:
                logger.warning("Lidar raw cloud callback error: %s", exc)
        return self.ingest_cloud(frame.to_xyzi())

    def ingest_cloud(self, pts: np.ndarray | None) -> bool:
        if pts is None:
            return False

        with self._cloud_lock:
            self._latest_cloud = pts
        self._cloud_event.set()

        self._fps_counter.tick()
        self.metrics.total_frames += 1
        self.metrics.total_points += len(pts)
        self.metrics.last_frame_points = len(pts)
        self.metrics.last_frame_time = time.monotonic()

        for callback in self._cloud_callbacks:
            try:
                callback(pts)
            except Exception as exc:
                logger.warning("Lidar cloud callback error: %s", exc)
        return True

    def ingest_imu(self, imu_msg: Any) -> None:
        self._latest_imu = imu_msg
        for callback in self._imu_callbacks:
            try:
                callback(imu_msg)
            except Exception as exc:
                logger.warning("Lidar IMU callback error: %s", exc)

    def get_cloud(self) -> np.ndarray | None:
        with self._cloud_lock:
            return self._latest_cloud

    def get_raw_cloud(self) -> Any:
        with self._cloud_lock:
            return self._latest_raw_cloud

    def get_imu(self) -> Any:
        return self._latest_imu

    def wait_for_cloud(self, timeout: float = 5.0) -> np.ndarray | None:
        self._cloud_event.clear()
        if self.get_cloud() is not None:
            return self.get_cloud()
        self._cloud_event.wait(timeout=timeout)
        return self.get_cloud()

    @property
    def fps(self) -> float:
        return self._fps_counter.fps()
