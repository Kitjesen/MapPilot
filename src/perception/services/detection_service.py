"""Detection service skeleton — thin pass-through to the detector backend."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D as CoreDetection3D

logger = logging.getLogger(__name__)


class DetectionService:
    """Encapsulates detector invocations, 2D->3D projection and conversion.

    Phase 2 behavior: detect() is a direct delegate to the configured detector.
    Phase 3 moves the 2D->3D projection and core-message conversion here so
    PerceptionModule remains a pure orchestration shell.
    """

    def __init__(
        self,
        detector: Any | None = None,
        confidence_threshold: float = 0.0,
        depth_scale: float = 0.001,
        min_depth: float = 0.3,
        max_depth: float = 6.0,
    ) -> None:
        self._detector = detector
        self._confidence_threshold = confidence_threshold
        self._depth_scale = depth_scale
        self._min_depth = min_depth
        self._max_depth = max_depth

    @property
    def detector(self) -> Any | None:
        return self._detector

    @detector.setter
    def detector(self, value: Any | None) -> None:
        self._detector = value

    def detect(self, image: np.ndarray, text_prompt: str) -> list[Any]:
        """Run the detector and return a list of 2D detections.

        Args:
            image: BGR image, shape (H, W, 3), uint8.
            text_prompt: dot-separated class labels.

        Returns:
            Detection2D list; empty if the detector is missing or fails.
        """
        if self._detector is None:
            return []

        try:
            detections = self._detector.detect(image, text_prompt)
        except Exception as e:
            logger.warning("DetectionService detect() failed: %s", e)
            return []

        if not detections:
            return []

        # Minimal fuzzy filter: drop obvious low-confidence results.  The
        # detector already applies its own threshold; this guard is for future
        # service-level policy without changing the current contract.
        if self._confidence_threshold > 0.0:
            detections = [d for d in detections if getattr(d, "score", 1.0) >= self._confidence_threshold]

        return detections

    def project_to_3d(
        self,
        detections_2d: list[Any],
        depth: np.ndarray,
        tf_camera_to_world: np.ndarray,
        intrinsics: Any,
    ) -> list[Any]:
        """Project 2D detections to 3D (delegates to projection module)."""
        from perception.tracking.projection import (
            Detection3D as ProjDetection3D,
        )
        from perception.tracking.projection import (
            bbox_center_depth,
            mask_to_pointcloud,
            pointcloud_centroid,
            transform_point,
        )
        from perception.tracking.projection import (
            project_to_3d as project_single_point,
        )

        results = []
        for det2d in detections_2d:
            centroid = None
            center_depth = None
            points = None

            # Try mask -> pointcloud (USS-Nav main path)
            if getattr(det2d, "mask", None) is not None:
                points = mask_to_pointcloud(
                    mask=det2d.mask,
                    depth_image=depth,
                    intrinsics=intrinsics,
                    tf_camera_to_world=tf_camera_to_world,
                    depth_scale=self._depth_scale,
                    min_depth=self._min_depth,
                    max_depth=self._max_depth,
                )
                if points is not None and len(points) > 0:
                    centroid = pointcloud_centroid(points)
                    center_depth = float(np.linalg.norm(centroid - tf_camera_to_world[:3, 3]))

            # Fallback: bbox center depth
            if centroid is None:
                d = bbox_center_depth(depth, det2d.bbox, depth_scale=self._depth_scale)
                if d is None or d < self._min_depth or d > self._max_depth:
                    continue
                cx = (det2d.bbox[0] + det2d.bbox[2]) / 2
                cy = (det2d.bbox[1] + det2d.bbox[3]) / 2
                p_camera = project_single_point(cx, cy, d, intrinsics)
                centroid = transform_point(p_camera, tf_camera_to_world)
                center_depth = d

            results.append(
                ProjDetection3D(
                    position=centroid,
                    label=det2d.label,
                    score=det2d.score,
                    bbox_2d=det2d.bbox,
                    depth=center_depth,
                    features=getattr(det2d, "features", np.array([])),
                    points=points if points is not None else np.empty((0, 3)),
                    track_id=getattr(det2d, "track_id", None),
                )
            )
        return results

    def convert_to_core_detections(
        self,
        detections_3d: list[Any],
        *,
        source_ts: float = 0.0,
    ) -> list[CoreDetection3D]:
        """projection.Detection3D -> runtime.msgs.Detection3D."""
        results = []
        for d in detections_3d:
            pos = d.position
            feat = getattr(d, "features", None)
            has_feat = feat is not None and hasattr(feat, "size") and feat.size > 0
            track_id = getattr(d, "track_id", None)
            results.append(
                CoreDetection3D(
                    id=f"track_{track_id}" if track_id is not None else "",
                    label=d.label,
                    confidence=d.score,
                    position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                    bbox_2d=[float(x) for x in d.bbox_2d],
                    clip_feature=feat if has_feat else None,
                    ts=source_ts,
                )
            )
        return results
