"""Gateway localization runtime cache."""

from __future__ import annotations

from typing import Any

from runtime.msgs.numpy_compat import np


class LocCache:
    """Owns last-known localization values for Gateway consumers."""

    def __init__(self) -> None:
        self._odom: dict | None = None
        self._last_invalid_odometry: dict | None = None
        self._odom_timestamps: list[float] = []
        self._T_map_odom: np.ndarray | None = None
        self._has_map_odom_tf = False

    def clear(self) -> None:
        self._odom = None
        self._last_invalid_odometry = None
        self._odom_timestamps.clear()
        self._T_map_odom = None
        self._has_map_odom_tf = False

    def record_odometry(
        self,
        odom: dict[str, Any],
        *,
        ts: float,
        max_samples: int = 20,
    ) -> None:
        self._last_invalid_odometry = None
        self._odom = odom
        self._odom_timestamps.append(float(ts))
        if len(self._odom_timestamps) > max_samples:
            del self._odom_timestamps[:-max_samples]


LocalizationRuntimeCacheService = LocCache
