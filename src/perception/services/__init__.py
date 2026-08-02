"""Thin service-layer wrappers around perception backends.

These classes are intentionally pass-through in Phase 2.  They provide a stable
boundary for Phase 3 refactors (e.g., splitting instance_tracker into a
standalone tracking service) without changing any observable runtime behavior.
"""

from .detection_service import DetectionService
from .scene_graph_service import SceneGraphService
from .tracking_service import TrackingService

__all__ = [
    "DetectionService",
    "SceneGraphService",
    "TrackingService",
]
