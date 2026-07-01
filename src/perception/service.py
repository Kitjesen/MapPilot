"""Pure perception pipeline service.

The service coordinates injected detector, encoder, and tracker components.
It owns no ROS, TF, GPU, or transport setup; callers provide already-decoded
numpy arrays and the camera-to-world transform.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class FrameResult:
    """Output from one processed perception frame."""

    detections_3d: list[Any]
    scene_graph_json: str
    n_objects: int = 0
    timestamp: float = 0.0
    detect_ms: float = 0.0
    project_ms: float = 0.0
    track_ms: float = 0.0

    @property
    def total_ms(self) -> float:
        return self.detect_ms + self.project_ms + self.track_ms


class PerceptionService:
    """Framework-free detect -> project -> encode -> track pipeline."""

    def __init__(
        self,
        detector: Any = None,
        encoder: Any = None,
        tracker: Any = None,
        intrinsics: Any = None,
        max_depth: float = 6.0,
        min_depth: float = 0.3,
        depth_scale: float = 0.001,
        laplacian_threshold: float = 100.0,
    ) -> None:
        self.detector = detector
        self.encoder = encoder
        self.tracker = tracker
        self.intrinsics = intrinsics
        self._max_depth = max_depth
        self._min_depth = min_depth
        self._depth_scale = depth_scale
        self._laplacian_threshold = laplacian_threshold
        self._frame_count = 0

    def set_intrinsics(self, intrinsics: Any) -> None:
        self.intrinsics = intrinsics

    def process_frame(
        self,
        bgr: np.ndarray,
        depth: np.ndarray,
        tf_camera_to_world: np.ndarray,
        text_prompt: str = "",
    ) -> FrameResult | None:
        """Run the injected perception stack for one RGB-D frame."""

        self._frame_count += 1

        if self._is_blurry(bgr):
            return None

        start = time.perf_counter()
        detections_2d = self._detect(bgr, text_prompt)
        detect_ms = (time.perf_counter() - start) * 1000
        if not detections_2d:
            return None

        start = time.perf_counter()
        detections_3d = self._project(detections_2d, depth, tf_camera_to_world)
        project_ms = (time.perf_counter() - start) * 1000
        if not detections_3d:
            return None

        if self.encoder is not None:
            self._encode_features(bgr, detections_2d, detections_3d)

        start = time.perf_counter()
        self._track(detections_3d, tf_camera_to_world)
        track_ms = (time.perf_counter() - start) * 1000

        scene_graph_json = self.tracker.get_scene_graph_json() if self.tracker else "{}"
        tracked_objects = getattr(self.tracker, "_tracked_objects", {}) if self.tracker else {}

        return FrameResult(
            detections_3d=detections_3d,
            scene_graph_json=scene_graph_json,
            n_objects=len(tracked_objects),
            timestamp=time.time(),
            detect_ms=detect_ms,
            project_ms=project_ms,
            track_ms=track_ms,
        )

    def _is_blurry(self, bgr: np.ndarray) -> bool:
        try:
            from .laplacian_filter import is_blurry
        except ImportError:
            return False

        return is_blurry(bgr, threshold=self._laplacian_threshold)

    def _detect(self, bgr: np.ndarray, text_prompt: str) -> list[Any]:
        if self.detector is None:
            return []

        try:
            return self.detector.detect(bgr, text_prompt)
        except Exception:
            logger.exception("Detection failed")
            return []

    def _project(
        self,
        detections_2d: list[Any],
        depth: np.ndarray,
        tf_camera_to_world: np.ndarray,
    ) -> list[Any]:
        if self.intrinsics is None:
            return []

        try:
            from .projection import (
                Detection3D,
                bbox_center_depth,
                mask_to_pointcloud,
                pointcloud_centroid,
                project_to_3d,
                transform_point,
            )
        except ImportError:
            logger.warning("projection module not available")
            return []

        detections_3d = []
        camera_origin = tf_camera_to_world[:3, 3]

        for detection in detections_2d:
            try:
                points_world = np.empty((0, 3))
                mask = getattr(detection, "mask", None)

                if mask is not None:
                    points_camera = mask_to_pointcloud(
                        mask,
                        depth,
                        self.intrinsics,
                        max_depth=self._max_depth,
                        min_depth=self._min_depth,
                        depth_scale=self._depth_scale,
                    )
                    if points_camera is None or len(points_camera) == 0:
                        continue

                    points_world = (
                        tf_camera_to_world[:3, :3] @ points_camera.T
                    ).T + camera_origin
                    centroid = pointcloud_centroid(points_world)
                else:
                    bbox = getattr(detection, "bbox", None)
                    if bbox is None:
                        continue

                    z = bbox_center_depth(bbox, depth, self._depth_scale)
                    if z is None or z < self._min_depth or z > self._max_depth:
                        continue

                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                    point_camera = project_to_3d(center_x, center_y, z, self.intrinsics)
                    centroid = transform_point(point_camera, tf_camera_to_world)

                detections_3d.append(
                    Detection3D(
                        position=centroid,
                        label=detection.label,
                        score=detection.score,
                        bbox_2d=detection.bbox,
                        depth=float(np.linalg.norm(centroid - camera_origin)),
                        features=getattr(detection, "features", np.array([])),
                        points=points_world,
                    )
                )
            except Exception:
                logger.debug(
                    "Projection failed for detection %r",
                    getattr(detection, "label", "<unknown>"),
                    exc_info=True,
                )

        return detections_3d

    def _encode_features(
        self,
        bgr: np.ndarray,
        detections_2d: list[Any],
        detections_3d: list[Any],
    ) -> None:
        for detection_2d, detection_3d in zip(detections_2d, detections_3d):
            try:
                x1, y1, x2, y2 = [int(value) for value in detection_2d.bbox[:4]]
                crop = bgr[max(0, y1) : y2, max(0, x1) : x2]
                if crop.size == 0:
                    continue

                features = self.encoder.encode_image(crop)
                if features is not None:
                    detection_3d.features = features
            except Exception:
                logger.debug(
                    "Feature encoding failed for detection %r",
                    getattr(detection_2d, "label", "<unknown>"),
                    exc_info=True,
                )

    def _track(self, detections_3d: list[Any], tf_camera_to_world: np.ndarray) -> None:
        if self.tracker is None:
            return

        try:
            self.tracker.update(
                detections_3d,
                camera_pos=tf_camera_to_world[:3, 3],
                camera_forward=tf_camera_to_world[:3, 2],
                intrinsics_fx=getattr(self.intrinsics, "fx", 0.0),
            )
        except Exception:
            logger.exception("Tracker update failed")

    def health(self) -> dict[str, Any]:
        return {
            "detector": type(self.detector).__name__ if self.detector else None,
            "encoder": type(self.encoder).__name__ if self.encoder else None,
            "tracker": self.tracker is not None,
            "intrinsics": self.intrinsics is not None,
            "frames_processed": self._frame_count,
        }


__all__ = ["FrameResult", "PerceptionService"]
