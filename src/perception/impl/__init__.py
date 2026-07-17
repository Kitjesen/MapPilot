"""Semantic Perception - legacy implementation shim.

The concrete implementations have been moved to the canonical packages:
  - perception.detection
  - perception.encoding
  - perception.tracking

This module only re-exports symbols for backward compatibility.  New code
should import from the canonical locations or use ``perception.api.factory``.
"""

from perception.detection.yolo_world_detector import YOLOWorldDetector
from perception.encoding.clip_encoder import CLIPEncoder
from perception.tracking.instance_tracker import InstanceTracker

__all__ = [
    "CLIPEncoder",
    "InstanceTracker",
    "YOLOWorldDetector",
]
