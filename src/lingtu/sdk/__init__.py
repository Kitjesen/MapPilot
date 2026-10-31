"""Public synchronous and asynchronous clients for the LingTu Gateway.

Domain clients share Gateway address, authentication and timeout settings.
All clients use only the Python standard library; commands retain the Gateway's admission semantics.
"""

from __future__ import annotations

from .async_client import AsyncLingTuClient
from .async_domains import (
    AsyncLocalizationClient,
    AsyncMapClient,
    AsyncNavigationClient,
    AsyncObservationClient,
    AsyncSystemClient,
    AsyncTaskClient,
)
from .client import LingTuClient
from .localization import LocalizationClient
from .maps import MapClient
from .models import (
    CommandResult,
    HealthStatus,
    MapInfo,
    MapList,
    NavigationControlState,
    NavigationGoalAdmission,
    NavigationMotionState,
    NavigationStatus,
    NavigationTask,
    Pose2D,
    Position,
    RobotState,
    SessionInfo,
)
from .navigation import NavigationClient
from .observations import ObservationClient
from .system import SystemClient
from .tasks import TaskClient

__all__ = [
    "AsyncLingTuClient",
    "AsyncLocalizationClient",
    "AsyncMapClient",
    "AsyncNavigationClient",
    "AsyncObservationClient",
    "AsyncSystemClient",
    "AsyncTaskClient",
    "CommandResult",
    "HealthStatus",
    "LingTuClient",
    "LocalizationClient",
    "MapClient",
    "MapInfo",
    "MapList",
    "NavigationClient",
    "NavigationControlState",
    "NavigationGoalAdmission",
    "NavigationMotionState",
    "NavigationStatus",
    "NavigationTask",
    "ObservationClient",
    "Pose2D",
    "Position",
    "RobotState",
    "SessionInfo",
    "SystemClient",
    "TaskClient",
]
