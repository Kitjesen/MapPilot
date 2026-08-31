"""Tracking service skeleton — thin pass-through to the tracker backend."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


class TrackingService:
    """Encapsulates tracker updates.

    Phase 2 behavior: direct delegate to the tracker backend.  Phase 3 will
    move the instance tracking state machine here so PerceptionModule only
    consumes the resulting tracks.
    """

    def __init__(self, tracker: Any | None = None) -> None:
        self._tracker = tracker

    @property
    def tracker(self) -> Any | None:
        return self._tracker

    @tracker.setter
    def tracker(self, value: Any | None) -> None:
        self._tracker = value

    def update(
        self,
        detections_3d: list[Any],
        camera_pos: np.ndarray | None = None,
        camera_forward: np.ndarray | None = None,
        intrinsics_fx: float = 0.0,
    ) -> list[Any]:
        """Update the tracker with a batch of 3D detections.

        Returns the updated/created track list, or an empty list if the tracker
        is unavailable or the call fails.
        """
        if self._tracker is None:
            return []

        try:
            return self._tracker.update(
                detections_3d,
                camera_pos=camera_pos,
                camera_forward=camera_forward,
                intrinsics_fx=intrinsics_fx,
            )
        except Exception as e:
            logger.warning("TrackingService update() failed: %s", e)
            return []
