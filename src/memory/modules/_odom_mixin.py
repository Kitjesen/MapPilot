"""Shared mixin for memory modules that track robot position via odometry."""
from __future__ import annotations

from runtime.msgs.numpy_compat import np
from runtime.msgs.nav import Odometry


class OdomTrackingMixin:
    """Mixin providing validated odometry caching.

    Provides:
      - _last_odom: Optional[Odometry] -- last valid odometry
      - _on_odom(odom) -- validates and caches odometry
      - _robot_position -- convenience property returning (x, y, z) as ndarray

    Usage: Call ``self.odometry.subscribe(self._on_odom)`` in setup().
    """

    _last_odom: Odometry | None = None

    def _on_odom(self, odom: Odometry) -> None:
        if not (np.isfinite(odom.x) and np.isfinite(odom.y)):
            return
        self._last_odom = odom

    @property
    def _robot_position(self) -> np.ndarray | None:
        if self._last_odom is None:
            return None
        o = self._last_odom
        return np.array([o.x, o.y, o.z], dtype=np.float64)
