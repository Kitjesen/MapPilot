"""Compatibility wrapper for ``explore.tare.supervisor``."""

from explore.tare.supervisor import (
    MODE_DEGRADED,
    MODE_FALLBACK,
    MODE_FINISHED,
    MODE_HEALTHY,
    MODE_STARTING,
    MODE_UNINIT,
    ExplorationSupervisorModule,
)

__all__ = [
    "MODE_DEGRADED",
    "MODE_FALLBACK",
    "MODE_FINISHED",
    "MODE_HEALTHY",
    "MODE_STARTING",
    "MODE_UNINIT",
    "ExplorationSupervisorModule",
]
