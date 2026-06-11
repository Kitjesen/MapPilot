"""WaypointTracker — waypoint advancement and stuck detection.

Not a Module. Used internally by NavigationModule to separate tracking
logic from the mission FSM.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

from core.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

# Event tokens returned by update()
EV_WAYPOINT_REACHED = "waypoint_reached"
EV_PATH_COMPLETE    = "path_complete"
EV_STUCK_WARN       = "stuck_warn"
EV_STUCK            = "stuck"


@dataclass
class TrackerStatus:
    """Snapshot of tracker state for one odometry tick."""
    wp_index: int
    wp_total: int
    event: str | None = field(default=None)


class WaypointTracker:
    """Tracks progress along a waypoint path.

    Handles:
      - Waypoint arrival detection (2D distance threshold, optional Z gate)
      - Stuck detection with progressive warn (50%) → stuck (100%) thresholds
      - Each event fires exactly once per stuck/arrival occurrence

    Usage::

        tracker = WaypointTracker(threshold=1.5, stuck_timeout=10.0,
                                   stuck_dist=0.15)

        # On new path:
        tracker.reset(path, robot_pos)

        # In odometry callback:
        status = tracker.update(robot_pos)
        if status.event == EV_PATH_COMPLETE:
            ...
        elif status.event == EV_WAYPOINT_REACHED:
            next_wp = tracker.current_waypoint  # already advanced
            ...
        elif status.event == EV_STUCK:
            ...  # trigger replan
    """

    def __init__(
        self,
        threshold: float = 1.5,
        final_threshold: float | None = None,
        z_threshold: float | None = None,
        stuck_timeout: float = 10.0,
        stuck_dist: float = 0.15,
        stuck_yaw_rad: float = 0.35,  # ~20° — yaw drift below this counts as "stuck"
    ) -> None:
        self._threshold = threshold
        self._final_threshold = threshold if final_threshold is None else final_threshold
        self._z_threshold = self._normalise_z_threshold(z_threshold)
        self._stuck_timeout = stuck_timeout
        self._stuck_dist = stuck_dist
        self._stuck_yaw = stuck_yaw_rad

        self._path: list[np.ndarray] = []
        self._wp_index: int = 0
        self._last_progress_time: float = 0.0
        self._last_progress_pos = [0.0, 0.0, 0.0]
        self._last_progress_yaw: float | None = None  # None until first yaw-aware update
        self._stuck_warn_sent: bool = False
        self._stuck_sent: bool = False

    # ------------------------------------------------------------------ #
    # Public API                                                           #
    # ------------------------------------------------------------------ #

    def reset(
        self,
        path: list[np.ndarray],
        robot_pos: np.ndarray,
        robot_yaw: float | None = None,
    ) -> None:
        """Start tracking a new path. Clears all stuck state.

        Pass ``robot_yaw`` (radians) to enable yaw-aware stuck detection.
        Without yaw, legacy distance-only behaviour applies.
        """
        self._path = [np.asarray(point, dtype=float) for point in path]
        self._wp_index = 0
        self._last_progress_time = time.time()
        self._last_progress_pos = np.asarray(robot_pos, dtype=float).copy()
        self._last_progress_yaw = robot_yaw
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def clear(self) -> None:
        """Discard current path (e.g. on stop/cancel)."""
        self._path = []
        self._wp_index = 0
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def pause(self) -> None:
        """Pause tracking — reset stuck timer but keep waypoint state."""
        self._last_progress_time = time.time()
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def update(
        self,
        robot_pos: np.ndarray,
        robot_yaw: float | None = None,
    ) -> TrackerStatus:
        """Process a new odometry position.

        Pass ``robot_yaw`` (radians) to enable yaw-aware stuck detection:
        rotation in place counts as progress (prevents "robot spins a bit and
        resets the timer forever" bug). Without yaw, legacy behaviour applies.
        """
        robot_pos_arr = np.asarray(robot_pos, dtype=float)
        if not self._path or self._wp_index >= len(self._path):
            return TrackerStatus(self._wp_index, len(self._path))

        # -- Arrival check ---------------------------------------------------
        wp = self._path[self._wp_index]
        is_final_wp = self._wp_index >= len(self._path) - 1
        threshold = self._final_threshold if is_final_wp else self._threshold
        if self._is_waypoint_reached(robot_pos_arr, wp, threshold):
            self._wp_index += 1
            self._last_progress_time = time.time()
            self._last_progress_pos = robot_pos_arr.copy()
            self._last_progress_yaw = robot_yaw
            self._stuck_warn_sent = False
            self._stuck_sent = False

            if self._wp_index >= len(self._path):
                return TrackerStatus(self._wp_index, len(self._path),
                                     event=EV_PATH_COMPLETE)
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_WAYPOINT_REACHED)

        # -- Stuck detection -------------------------------------------------
        now = time.time()
        moved = self._progress_distance(robot_pos_arr, self._last_progress_pos)
        elapsed = now - self._last_progress_time

        # Yaw progress — only when caller supplies yaw on both reset/update
        yaw_progress = False
        if robot_yaw is not None and self._last_progress_yaw is not None:
            dyaw = abs(
                math.atan2(
                    math.sin(robot_yaw - self._last_progress_yaw),
                    math.cos(robot_yaw - self._last_progress_yaw),
                )
            )
            yaw_progress = dyaw >= self._stuck_yaw

        # Reset if robot has moved enough OR rotated enough
        if moved >= self._stuck_dist or yaw_progress:
            self._last_progress_time = now
            self._last_progress_pos = robot_pos_arr.copy()
            if robot_yaw is not None:
                self._last_progress_yaw = robot_yaw
            self._stuck_warn_sent = False
            self._stuck_sent = False
            return TrackerStatus(self._wp_index, len(self._path))

        # Warn at 50% of timeout (fires once)
        if elapsed > self._stuck_timeout * 0.5 and not self._stuck_warn_sent:
            self._stuck_warn_sent = True
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_STUCK_WARN)

        # Stuck at 100% of timeout (fires once per stuck episode)
        if elapsed > self._stuck_timeout and not self._stuck_sent:
            self._stuck_sent = True
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_STUCK)

        return TrackerStatus(self._wp_index, len(self._path))

    @staticmethod
    def _normalise_z_threshold(value: float | None) -> float | None:
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(parsed) or parsed < 0.0:
            return None
        return parsed

    def _is_waypoint_reached(
        self,
        robot_pos: np.ndarray,
        waypoint: np.ndarray,
        threshold: float,
    ) -> bool:
        robot_pos = np.asarray(robot_pos, dtype=float)
        waypoint = np.asarray(waypoint, dtype=float)
        if not np.isfinite(robot_pos).all() or not np.isfinite(waypoint).all():
            logger.warning("Non-finite position in waypoint check, skipping")
            return False
        if np.linalg.norm(robot_pos[:2] - waypoint[:2]) >= threshold:
            return False
        if self._z_threshold is None:
            return True
        if len(robot_pos) < 3 or len(waypoint) < 3:
            return True
        robot_z = float(robot_pos[2])
        waypoint_z = float(waypoint[2])
        if not (math.isfinite(robot_z) and math.isfinite(waypoint_z)):
            return False
        return abs(robot_z - waypoint_z) <= self._z_threshold

    def _progress_distance(self, robot_pos: np.ndarray, last_pos: np.ndarray) -> float:
        robot_pos = np.asarray(robot_pos, dtype=float)
        last_pos = np.asarray(last_pos, dtype=float)
        if not np.isfinite(robot_pos).all() or not np.isfinite(last_pos).all():
            logger.warning("Non-finite robot position in progress check, returning inf")
            return float('inf')
        xy_dist = float(np.linalg.norm(robot_pos[:2] - last_pos[:2]))
        if self._z_threshold is None or len(robot_pos) < 3 or len(last_pos) < 3:
            return xy_dist
        robot_z = float(robot_pos[2])
        last_z = float(last_pos[2])
        if not (math.isfinite(robot_z) and math.isfinite(last_z)):
            return xy_dist
        return float(math.hypot(xy_dist, robot_z - last_z))

    # ------------------------------------------------------------------ #
    # Read-only properties                                                 #
    # ------------------------------------------------------------------ #

    @property
    def current_waypoint(self) -> np.ndarray | None:
        """Current waypoint to pursue, or None if path complete/empty."""
        if self._wp_index < len(self._path):
            return self._path[self._wp_index]
        return None

    @property
    def wp_index(self) -> int:
        return self._wp_index

    @property
    def path_length(self) -> int:
        return len(self._path)

    @property
    def has_path(self) -> bool:
        return bool(self._path) and self._wp_index < len(self._path)
