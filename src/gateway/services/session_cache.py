"""Gateway product-session and SLAM runtime cache."""

from __future__ import annotations

import time
from typing import Any


class SessionCache:
    """Owns product-session and SLAM diagnostic values."""

    def __init__(self) -> None:
        self._session_mode = "idle"
        self._session_product_session = "idle"
        self._session_product: str | None = None
        self._session_map: str | None = None
        self._session_slam_profile = "stopped"
        self._session_since = time.time()
        self._session_error = ""
        self._session_pending = False
        self._icp_quality = 0.0
        self._localization_status: dict | None = None
        self._cached_slam_profile = ""
        self._slam_profile_ts = 0.0

    def set_icp_quality(self, value: Any) -> None:
        self._icp_quality = float(value)

    def set_localization_status(self, status: dict[str, Any]) -> None:
        self._localization_status = dict(status)

    def remember_slam_profile(self, profile: str, *, ts: float | None = None) -> str:
        self._cached_slam_profile = str(profile or "").strip().lower()
        self._slam_profile_ts = time.time() if ts is None else float(ts)
        return self._cached_slam_profile


SessionRuntimeCacheService = SessionCache
