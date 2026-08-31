"""Semantic Perception API - Factory layer.

Factories are a thin adapter layer over the canonical
implementations in perception.detection / perception.encoding /
perception.tracking.  They adapt the canonical duck-typed APIs to the
DetectorAPI / EncoderAPI / TrackerAPI ABCs used by external callers.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from runtime.registry import get, list_plugins, register

from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .exceptions import ConfigurationError
from .tracker_api import TrackerAPI
from .types import Detection2D as APIDetection2D
from .types import Detection3D as APIDetection3D
from .types import PerceptionConfig

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Detector adapter: canonical YOLOWorldDetector -> DetectorAPI
# ---------------------------------------------------------------------------
class _YOLOWorldDetectorAdapter(DetectorAPI):
    """Adapter that exposes the canonical YOLOWorldDetector through DetectorAPI.

    The canonical detector uses ``detect(rgb, text_prompt)`` while the API
    expects stateful classes set via ``set_classes``.  This adapter stores the
    class list and forwards it as a dot-separated prompt.
    """

    def __init__(self, config: PerceptionConfig | None) -> None:
        cfg = config or PerceptionConfig()
        from ..detection.yolo_world_detector import YOLOWorldDetector

        self._impl = YOLOWorldDetector(
            model_size=getattr(cfg, "model_size", "l"),
            confidence=cfg.confidence_threshold if cfg else 0.3,
            iou_threshold=cfg.iou_threshold if cfg else 0.5,
        )
        self._classes: list[str] = []

    def detect(self, image: np.ndarray) -> list[APIDetection2D]:
        if not self._classes:
            logger.warning("No classes set, returning empty detections")
            return []
        prompt = " . ".join(self._classes)
        raw = self._impl.detect(image, prompt)
        return [
            APIDetection2D(
                label=d.label,
                confidence=d.score,
                bbox=self._to_bbox(d.bbox),
            )
            for d in raw
        ]

    @staticmethod
    def _to_bbox(bbox: np.ndarray | Any) -> Any:
        from .types import BBox2D

        if hasattr(bbox, "x1"):
            return bbox
        arr = np.asarray(bbox).reshape(-1)
        return BBox2D(
            x1=float(arr[0]),
            y1=float(arr[1]),
            x2=float(arr[2]),
            y2=float(arr[3]),
        )

    def set_classes(self, classes: list[str]) -> None:
        if not classes:
            raise ConfigurationError("Classes list cannot be empty")
        self._classes = list(classes)

    def get_classes(self) -> list[str]:
        return list(self._classes)

    def get_model_info(self) -> dict:
        stats = self._impl.get_statistics()
        return {
            "name": "YOLO-World",
            "version": f"v8{self._impl.model_size}-worldv2",
            "input_size": (640, 640),
            "backend": "tensorrt" if self._impl.tensorrt else "pytorch",
            "device": self._impl.device or "auto",
            "tensorrt_int8": self._impl.tensorrt_int8,
            "avg_fps": stats.get("avg_fps", 0.0),
            "detect_count": stats.get("detect_count", 0),
        }

    def set_confidence_threshold(self, threshold: float) -> None:
        self._impl.confidence = threshold

    def get_confidence_threshold(self) -> float:
        return float(self._impl.confidence)

    def warmup(self, num_iterations: int = 10) -> None:
        self._impl.load_model()


# ---------------------------------------------------------------------------
# Encoder adapter: canonical CLIPEncoder -> EncoderAPI
# ---------------------------------------------------------------------------
class _CLIPEncoderAdapter(EncoderAPI):
    """Adapter that exposes the canonical CLIPEncoder through EncoderAPI."""

    def __init__(self, config: PerceptionConfig | None) -> None:
        from ..encoding.clip_encoder import CLIPEncoder

        self._impl = CLIPEncoder()

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        return self._impl.encode_image_crops(image, [np.array([0, 0, image.shape[1], image.shape[0]])])[0]

    def encode_text(self, text: str) -> np.ndarray:
        result = self._impl.encode_text([text])
        if result.ndim == 2 and result.shape[0] > 0:
            return result[0]
        return result

    def encode_images_batch(self, images: list[np.ndarray]) -> np.ndarray:
        if not images:
            return np.array([])
        features = [self.encode_image(img) for img in images]
        valid = [f for f in features if f.size > 0]
        if not valid:
            return np.array([])
        return np.stack(valid)

    def encode_texts_batch(self, texts: list[str]) -> np.ndarray:
        if not texts:
            return np.array([])
        return self._impl.encode_text(texts)

    def compute_similarity(
        self,
        image_features: np.ndarray,
        text_features: np.ndarray,
    ) -> float:
        similarity = float(np.dot(image_features, text_features))
        return (similarity + 1.0) / 2.0

    def compute_similarity_matrix(
        self,
        image_features: np.ndarray,
        text_features: np.ndarray,
    ) -> np.ndarray:
        return (image_features @ text_features.T + 1.0) / 2.0

    def get_feature_dim(self) -> int:
        return self._impl.feature_dim

    def get_model_info(self) -> dict:
        return {
            "name": "CLIP",
            "version": self._impl._model_name,
            "feature_dim": self._impl.feature_dim,
            "device": self._impl._device,
            "cache_enabled": self._impl._enable_cache,
            "cache_hit_rate": self._impl.cache_hit_rate,
        }

    def warmup(self, num_iterations: int = 10) -> None:
        self._impl.load_model()


# ---------------------------------------------------------------------------
# Tracker adapter: canonical InstanceTracker -> TrackerAPI
# ---------------------------------------------------------------------------
class _InstanceTrackerAdapter(TrackerAPI):
    """Adapter that exposes the canonical InstanceTracker through TrackerAPI.

    The canonical tracker uses ``tracking.projection.Detection3D`` and extra
    camera metadata; this adapter performs the minimal conversion required by
    the ABC contract.
    """

    def __init__(self, config: PerceptionConfig | None) -> None:
        cfg = config or PerceptionConfig()
        from ..tracking.instance_tracker import InstanceTracker

        self._impl = InstanceTracker(
            merge_distance=cfg.merge_distance if cfg else 0.5,
            iou_threshold=cfg.iou_threshold if cfg else 0.5,
        )

    def update(
        self,
        detections: list[APIDetection3D],
        timestamp: float | None = None,
    ) -> list[APIDetection3D]:
        if not detections:
            return []
        proj_dets = self._convert_to_projection(detections)
        tracked = self._impl.update(proj_dets)
        return self._convert_to_api(tracked)

    @staticmethod
    def _convert_to_projection(detections: list[APIDetection3D]) -> list[Any]:
        from ..tracking.projection import Detection3D as ProjDetection3D

        results = []
        for d in detections:
            pos = d.position_3d
            bbox = d.bbox_2d
            if hasattr(bbox, "to_list"):
                bbox_list = bbox.to_list()
            elif hasattr(bbox, "x1"):
                bbox_list = [bbox.x1, bbox.y1, bbox.x2, bbox.y2]
            else:
                bbox_list = [float(x) for x in bbox]
            results.append(
                ProjDetection3D(
                    position=np.array([pos.x, pos.y, pos.z]),
                    label=d.label,
                    score=d.confidence,
                    bbox_2d=np.array(bbox_list),
                    features=d.clip_feature if d.clip_feature is not None else np.array([]),
                    points=np.empty((0, 3)),
                    track_id=int(d.id.split("_")[-1]) if d.id and d.id.startswith("track_") else None,
                )
            )
        return results

    @staticmethod
    def _convert_to_api(tracked: list[Any]) -> list[APIDetection3D]:
        from .types import BBox2D, Position3D

        results = []
        for obj in tracked:
            pos = getattr(obj, "position", np.zeros(3))
            bbox = getattr(obj, "bbox_2d", [0.0, 0.0, 0.0, 0.0])
            results.append(
                APIDetection3D(
                    id=f"track_{getattr(obj, 'object_id', 0)}",
                    label=getattr(obj, "label", ""),
                    confidence=getattr(obj, "best_score", 0.0),
                    bbox_2d=BBox2D(
                        x1=float(bbox[0]),
                        y1=float(bbox[1]),
                        x2=float(bbox[2]),
                        y2=float(bbox[3]),
                    ),
                    position_3d=Position3D(
                        x=float(pos[0]),
                        y=float(pos[1]),
                        z=float(pos[2]),
                    ),
                    clip_feature=getattr(obj, "features", None),
                )
            )
        return results

    def get_all_tracks(self) -> list[APIDetection3D]:
        return self._convert_to_api(list(self._impl.objects.values()))

    def get_track_by_id(self, track_id: str) -> APIDetection3D | None:
        try:
            key = int(track_id.split("_")[-1])
        except ValueError:
            return None
        obj = self._impl.objects.get(key)
        if obj is None:
            return None
        tracks = self._convert_to_api([obj])
        return tracks[0] if tracks else None

    def remove_track(self, track_id: str) -> None:
        try:
            key = int(track_id.split("_")[-1])
            self._impl.objects.pop(key, None)
        except ValueError:
            pass

    def reset(self) -> None:
        self._impl.objects.clear()

    def get_track_count(self) -> int:
        return len(self._impl.objects)

    def configure(self, config: dict) -> None:
        if "merge_distance" in config:
            self._impl.merge_distance = config["merge_distance"]
        if "iou_threshold" in config:
            self._impl.iou_threshold = config["iou_threshold"]


# ---------------------------------------------------------------------------
# Registry providers
# ---------------------------------------------------------------------------
@register("detector_factory", "yolo_world", description="YOLO-World DetectorAPI adapter")
class _YOLOWorldFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> DetectorAPI:
        return _YOLOWorldDetectorAdapter(config)


@register("encoder_factory", "clip", description="CLIP EncoderAPI adapter")
class _CLIPFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> EncoderAPI:
        return _CLIPEncoderAdapter(config)


@register("perception_factory_tracker", "instance", description="Instance TrackerAPI adapter")
class _InstanceTrackerFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> TrackerAPI:
        return _InstanceTrackerAdapter(config)


class PerceptionFactory:
    """Component factory over the canonical perception backends."""

    @staticmethod
    def create_detector(
        detector_type: str,
        config: PerceptionConfig | None = None,
    ) -> DetectorAPI:
        """Create a detector matching the DetectorAPI contract."""
        try:
            provider = get("detector_factory", detector_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown detector type: {detector_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_detectors())}"
            ) from None

    @staticmethod
    def create_encoder(
        encoder_type: str,
        config: PerceptionConfig | None = None,
    ) -> EncoderAPI:
        """Create an encoder matching the EncoderAPI contract."""
        try:
            provider = get("encoder_factory", encoder_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown encoder type: {encoder_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_encoders())}"
            ) from None

    @staticmethod
    def create_tracker(
        tracker_type: str,
        config: PerceptionConfig | None = None,
    ) -> TrackerAPI:
        """Create a tracker matching the TrackerAPI contract."""
        try:
            provider = get("perception_factory_tracker", tracker_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown tracker type: {tracker_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_trackers())}"
            ) from None

    @staticmethod
    def get_available_detectors() -> list[str]:
        return list_plugins("detector_factory")

    @staticmethod
    def get_available_encoders() -> list[str]:
        return list_plugins("encoder_factory")

    @staticmethod
    def get_available_trackers() -> list[str]:
        return list_plugins("perception_factory_tracker")
