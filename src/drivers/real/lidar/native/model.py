"""Runtime status model shared by canonical LiDAR sources."""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Any


class LidarState(enum.Enum):
    """Lifecycle state reported by a LiDAR source."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class LidarHealth:
    """Observable health metrics reported by a LiDAR source."""

    state: LidarState = LidarState.DISCONNECTED
    ip: str = ""
    fps: float = 0.0
    total_frames: int = 0
    total_points: int = 0
    last_frame_time: float = 0.0
    last_frame_points: int = 0
    uptime_s: float = 0.0
    driver_pid: int | None = None
    driver_restarts: int = 0
    last_error: str = ""

    def to_dict(self) -> dict[str, Any]:
        """Return the stable monitoring representation."""

        return {
            "state": self.state.value,
            "ip": self.ip,
            "fps": round(self.fps, 1),
            "total_frames": self.total_frames,
            "total_points": self.total_points,
            "last_frame_points": self.last_frame_points,
            "uptime_s": round(self.uptime_s, 1),
            "driver_pid": self.driver_pid,
            "driver_restarts": self.driver_restarts,
            "last_error": self.last_error,
        }


__all__ = ["LidarHealth", "LidarState"]
