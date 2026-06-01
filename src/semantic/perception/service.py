"""PerceptionService — pure algorithm orchestration, zero framework dependency.

Coordinates detector → projector → encoder → tracker pipeline.
All components are injected, not created internally.
Testable with mocks, no ROS2/GPU required.

Usage::

    svc = PerceptionService(
        detector=YOLOEDetector(model_size="l"),
        encoder=MobileCLIPEncoder(),
        tracker=InstanceTracker(merge_distance=0.5),
    )
    svc.detector.load_model()
    svc.encoder.load_model()

    result = svc.process_frame(bgr, depth, tf_matrix, text_prompt="door . chair")
    print(result.scene_graph_json)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Result types
# ---------------------------------------------------------------------------

@dataclass
class FrameResult:
    """Output of a single perception frame."""
    detections_3d: list         # list[Detection3D] from projection
    scene_graph_json: str       # JSON string from tracker
    n_objects: int = 0          # number of tracked objects
    timestamp: float = 0.0
    detect_ms: float = 0.0
    project_ms: float = 0.0
    track_ms: float = 0.0

    @property
    def total_ms(self) -> float:
        return self.detect_ms + self.project_ms + self.track_ms


# ---------------------------------------------------------------------------
# PerceptionService
# ---------------------------------------------------------------------------

class PerceptionService:
    """Pure perception pipeline: detect → project → encode → track.

    All components are injected via constructor. No ROS2, no TF, no cv_bridge.
    The caller is responsible for:
      - Converting ROS2 messages to numpy arrays
      - Looking up TF transforms
      - Publishing results to topics/ports
    """

    def __init__(
        self,
        detector: Any = None,           # DetectorBase subclass
        encoder: Any = None,            # CLIPEncoder / MobileCLIPEncoder
        tracker: Any = None,            # InstanceTracker
        intrinsics: Any = None,         # projection.CameraIntrinsics
        max_depth: float = 6.0,
        min_depth: float = 0.3,
        depth_scale: float = 0.001,
        laplacian_threshold: float = 100.0,
    ):
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
        """Update camera intrinsics (called once when CameraInfo arrives)."""
        self.intrinsics = intrinsics

    def process_frame(
        self,
        bgr: np.ndarray,
        depth: np.ndarray,
        tf_camera_to_world: np.ndarray,
        text_prompt: str = "",
    ) -> FrameResult | None:
        """Run full perception pipeline on one frame.

        Args:
            bgr: HxWx3 uint8 BGR image
            depth: HxW uint16 depth map (raw, scaled by depth_scale)
            tf_camera_to_world: 4x4 transform matrix
            text_prompt: detection classes, dot-separated ("door . chair")

        Returns:
            FrameResult or None if frame is skipped (blurry/no detections)
        """
        self._frame_count += 1

        # 1. Blur detection
        if self._is_blurry(bgr):
            return None

        # 2. Detect
        t0 = time.time()
        dets_2d = self._detect(bgr, text_prompt)
        detect_ms = (time.time() - t0) * 1000
        if not dets_2d:
            return None

        # 3. Project 2D → 3D
        t1 = time.time()
        dets_3d = self._project(dets_2d, depth, tf_camera_to_world)
        project_ms = (time.time() - t1) * 1000
        if not dets_3d:
            return None

        # 4. Encode features (optional)
        if self.encoder is not None:
            self._encode_features(bgr, dets_2d, dets_3d)

        # 5. Track
        t2 = time.time()
        self._track(dets_3d, tf_camera_to_world)
        track_ms = (time.time() - t2) * 1000

        # 6. Build result
        sg_json = self.tracker.get_scene_graph_json() if self.tracker else "{}"
        n_objects = (
            len(self.tracker._tracked_objects)
            if self.tracker and hasattr(self.tracker, '_tracked_objects')
            else 0
        )

        return FrameResult(
            detections_3d=dets_3d,
            scene_graph_json=sg_json,
            n_objects=n_objects,
            timestamp=time.time(),
            detect_ms=detect_ms,
            project_ms=project_ms,
            track_ms=track_ms,
        )

    # -- Pipeline steps (private) ---------------------------------------------

    def _is_blurry(self, bgr: np.ndarray) -> bool:
        try:
            from .laplacian_filter import is_blurry
            return is_blurry(bgr, threshold=self._laplacian_threshold)
        except ImportError:
            return False

    def _detect(self, bgr: np.ndarray, text_prompt: str) -> list:
        if self.detector is None:
            return []
        try:
            return self.detector.detect(bgr, text_prompt)
        except Exception:
            logger.exception("Detection failed")
            return []

    def _project(self, dets_2d: list, depth: np.ndarray,
                 tf_camera_to_world: np.ndarray) -> list:
        if self.intrinsics is None:
            return []
        try:
            from .projection import (
                bbox_center_depth,
                mask_to_pointcloud,
                pointcloud_centroid,
                project_to_3d,
                transform_point,
            )
        except ImportError:
            logger.warning("projection module not available")
            return []

        dets_3d = []
        for d in dets_2d:
            try:
                # USS-Nav: mask → pointcloud path
                if hasattr(d, 'mask') and d.mask is not None:
                    pts_cam = mask_to_pointcloud(
                        d.mask, depth, self.intrinsics,
                        max_depth=self._max_depth, min_depth=self._min_depth,
                        depth_scale=self._depth_scale,
                    )
                    if pts_cam is not None and len(pts_cam) > 0:
                        pts_world = (tf_camera_to_world[:3, :3] @ pts_cam.T).T + tf_camera_to_world[:3, 3]
                        centroid = pointcloud_centroid(pts_world)
                    else:
                        continue
                else:
                    # Fallback: bbox center depth
                    z = bbox_center_depth(d.bbox, depth, self._depth_scale)
                    if z is None or z < self._min_depth or z > self._max_depth:
                        continue
                    cx = (d.bbox[0] + d.bbox[2]) / 2
                    cy = (d.bbox[1] + d.bbox[3]) / 2
                    pt_cam = project_to_3d(cx, cy, z, self.intrinsics)
                    centroid = transform_point(pt_cam, tf_camera_to_world)
                    pts_world = np.empty((0, 3))

                from .projection import Detection3D
                det3d = Detection3D(
                    position=centroid,
                    label=d.label,
                    score=d.score,
                    bbox_2d=d.bbox,
                    depth=float(np.linalg.norm(centroid - tf_camera_to_world[:3, 3])),
                    features=getattr(d, 'features', np.array([])),
                    points=pts_world if 'pts_world' in dir() else np.empty((0, 3)),
                )
                dets_3d.append(det3d)
            except Exception:
                logger.debug("Projection failed for detection '%s'", d.label, exc_info=True)
                continue

        return dets_3d

    def _encode_features(self, bgr: np.ndarray, dets_2d: list, dets_3d: list) -> None:
        """Encode CLIP features for each detection (crop from bbox)."""
        for d2d, d3d in zip(dets_2d, dets_3d):
            try:
                x1, y1, x2, y2 = [int(v) for v in d2d.bbox[:4]]
                crop = bgr[max(0, y1):y2, max(0, x1):x2]
                if crop.size == 0:
                    continue
                feat = self.encoder.encode_image(crop)
                if feat is not None:
                    d3d.features = feat
            except Exception:
                pass

    def _track(self, dets_3d: list, tf_camera_to_world: np.ndarray) -> None:
        if self.tracker is None:
            return
        try:
            cam_pos = tf_camera_to_world[:3, 3]
            cam_fwd = tf_camera_to_world[:3, 2]
            fx = getattr(self.intrinsics, "fx", 0.0)
            self.tracker.update(
                dets_3d,
                camera_pos=cam_pos,
                camera_forward=cam_fwd,
                intrinsics_fx=fx,
            )
        except Exception:
            logger.exception("Tracker update failed")

    # -- Health ---------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        return {
            "detector": type(self.detector).__name__ if self.detector else None,
            "encoder": type(self.encoder).__name__ if self.encoder else None,
            "tracker": self.tracker is not None,
            "intrinsics": self.intrinsics is not None,
            "frames_processed": self._frame_count,
        }
