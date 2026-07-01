"""
Semantic Perception API

统一的语义感知接口层
"""

from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .exceptions import (
    ConfigurationError,
    DetectorError,
    DetectorInferenceError,
    DetectorInitError,
    EncoderError,
    EncoderInferenceError,
    EncoderInitError,
    InvalidCameraInfoError,
    InvalidDepthError,
    InvalidImageError,
    PerceptionAPIError,
    SceneGraphError,
    TrackerError,
)
from .factory import PerceptionFactory
from .perception_api import PerceptionAPI
from .tracker_api import TrackerAPI
from .types import (
    BBox2D,
    CameraInfo,
    Detection2D,
    Detection3D,
    PerceptionConfig,
    Position3D,
    Region,
    Relation,
    SceneGraph,
)

__all__ = [
    # Types
    "BBox2D",
    "CameraInfo",
    "ConfigurationError",
    "Detection2D",
    "Detection3D",
    "DetectorAPI",
    "DetectorError",
    "DetectorInferenceError",
    "DetectorInitError",
    "EncoderAPI",
    "EncoderError",
    "EncoderInferenceError",
    "EncoderInitError",
    "InvalidCameraInfoError",
    "InvalidDepthError",
    "InvalidImageError",
    # APIs
    "PerceptionAPI",
    # Exceptions
    "PerceptionAPIError",
    "PerceptionConfig",
    # Factory
    "PerceptionFactory",
    "Position3D",
    "Region",
    "Relation",
    "SceneGraph",
    "SceneGraphError",
    "TrackerAPI",
    "TrackerError",
]

__version__ = "1.0.0"
