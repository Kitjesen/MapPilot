"""Reusable per-frame RGB-D perception for Habitat experiments."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from PIL import Image
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from perception.backends import DetectorSpec
from perception.tracking.projection import POINTCLOUD_MIN_POINTS, Detection3D, mask_to_pointcloud
from runtime.msgs.sensor import CameraIntrinsics
from runtime.registry import get

DEFAULT_OBJECT_LABELS: tuple[str, ...] = (
    "chair",
    "table",
    "cabinet",
    "shelf",
    "sofa",
    "bed",
    "bottle",
    "cup",
    "door",
    "refrigerator",
    "sink",
    "television",
)


def build_observation_prompt(
    target: str,
    observation_labels: Sequence[str] = DEFAULT_OBJECT_LABELS,
) -> str:
    """Combine the task target with a compact, deduplicated object vocabulary."""
    labels: list[str] = []
    seen: set[str] = set()
    for value in (target, *observation_labels):
        for label in value.split("."):
            label = label.strip()
            key = label.casefold()
            if label and key not in seen:
                labels.append(label)
                seen.add(key)
    return " . ".join(labels)


@dataclass(frozen=True)
class ProjectionQuality:
    """Geometry retained while turning one predicted mask into a 3D detection."""

    raw_mask_pixels: int
    valid_depth_pixels: int
    cleaned_pixels: int
    projected_points: int
    component_count: int
    rejected_reason: str | None = None


@dataclass
class VisionObservation:
    """One YOLO/FastSAM observation and its optional 3D projection."""

    label: str
    bbox_xyxy: np.ndarray
    confidence: float
    mask: np.ndarray | None
    quality: ProjectionQuality
    detection: Detection3D | None = None


@dataclass
class VisionResult:
    """Per-frame detections plus the evidence used to construct them."""

    detections: list[Detection3D]
    observations: list[VisionObservation]


@dataclass(frozen=True)
class _CleanedMask:
    mask: np.ndarray
    raw_pixels: int
    valid_pixels: int
    component_count: int


def clean_depth_mask(
    mask: np.ndarray,
    depth_m: np.ndarray,
    *,
    min_depth: float = 0.3,
    max_depth: float = 15.0,
) -> _CleanedMask:
    """Keep the nearest substantial depth-connected component in a mask.

    FastSAM masks can cross an object silhouette into a wall or floor. Adjacent
    pixels are connected only when their metric depth is locally continuous;
    among substantial components, the nearest one is the foreground candidate.
    """
    mask = np.asarray(mask, dtype=bool)
    depth_m = np.asarray(depth_m)
    if mask.shape != depth_m.shape:
        raise ValueError("mask and depth must have the same shape")

    valid = mask & np.isfinite(depth_m) & (depth_m > min_depth) & (depth_m < max_depth)
    raw_pixels = int(mask.sum())
    valid_pixels = int(valid.sum())
    if valid_pixels == 0:
        return _CleanedMask(np.zeros_like(mask), raw_pixels, 0, 0)

    flat_ids = np.full(mask.size, -1, dtype=np.int64)
    valid_flat = np.flatnonzero(valid)
    flat_ids[valid_flat] = np.arange(valid_pixels)
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []

    def connect(a: np.ndarray, b: np.ndarray) -> None:
        a_ids = flat_ids[a]
        b_ids = flat_ids[b]
        present = (a_ids >= 0) & (b_ids >= 0)
        if not np.any(present):
            return
        a = a[present]
        b = b[present]
        a_ids = a_ids[present]
        b_ids = b_ids[present]
        a_depth = depth_m.flat[a]
        b_depth = depth_m.flat[b]
        continuous = np.abs(a_depth - b_depth) <= np.maximum(
            0.12,
            0.04 * np.minimum(a_depth, b_depth),
        )
        rows.extend((a_ids[continuous], b_ids[continuous]))
        cols.extend((b_ids[continuous], a_ids[continuous]))

    pixel_ids = np.arange(mask.size).reshape(mask.shape)
    connect(pixel_ids[:, :-1].ravel(), pixel_ids[:, 1:].ravel())
    connect(pixel_ids[:-1, :].ravel(), pixel_ids[1:, :].ravel())
    if rows:
        row = np.concatenate(rows)
        col = np.concatenate(cols)
        graph = coo_matrix(
            (np.ones(len(row), dtype=np.uint8), (row, col)),
            shape=(valid_pixels, valid_pixels),
        )
    else:
        graph = coo_matrix((valid_pixels, valid_pixels), dtype=np.uint8)
    component_count, labels = connected_components(graph, directed=False)
    counts = np.bincount(labels, minlength=component_count)
    substantial = np.flatnonzero(
        counts >= max(POINTCLOUD_MIN_POINTS, int(counts.max() * 0.2)),
    )
    candidates = substantial if len(substantial) else np.array([int(counts.argmax())])
    depths = depth_m.flat[valid_flat]
    selected = min(candidates, key=lambda label: float(np.median(depths[labels == label])))
    cleaned = np.zeros(mask.size, dtype=bool)
    cleaned[valid_flat[labels == selected]] = True
    return _CleanedMask(cleaned.reshape(mask.shape), raw_pixels, valid_pixels, int(component_count))


class Vision:
    """Load YOLO-World and FastSAM once, then infer consecutive RGB-D frames."""

    def __init__(
        self,
        prompt: str,
        yolo_weights: Path | str,
        fastsam_weights: Path | str,
        clip_weights: Path | str | None = None,
        *,
        device: str = "cpu",
        confidence: float = 0.25,
        min_depth: float = 0.3,
        max_depth: float = 15.0,
        observation_labels: Sequence[str] = DEFAULT_OBJECT_LABELS,
    ) -> None:
        import torch
        from ultralytics import FastSAM

        torch.set_num_threads(4)
        self.prompt = build_observation_prompt(prompt, observation_labels)
        self.device = device
        self.min_depth = min_depth
        self.max_depth = max_depth
        provider = get("detector", "yolo_world")
        self.detector = provider.create(DetectorSpec(
            name="yolo_world",
            model_path=str(Path(yolo_weights)),
            confidence=confidence,
            device=device,
        ))
        self.detector.load_model()
        self.segmenter = FastSAM(str(Path(fastsam_weights)))
        encoder_provider = get("encoder", "clip")
        model_path = str(Path(clip_weights)) if clip_weights else None
        self.encoder = encoder_provider.create(model_path=model_path, device=device)
        self.encoder.load_model()
        self._closed = False

    def infer(
        self,
        rgb: Image.Image | np.ndarray,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        optical_to_world: np.ndarray,
    ) -> VisionResult:
        """Infer one RGB-D frame expressed in the caller's chosen world frame."""
        image = (
            rgb.convert("RGB")
            if isinstance(rgb, Image.Image)
            else Image.fromarray(np.asarray(rgb, dtype=np.uint8), "RGB")
        )
        rgb_array = np.asarray(image)
        depth_m = np.asarray(depth_m)
        if depth_m.shape != rgb_array.shape[:2]:
            raise ValueError("RGB and depth dimensions must match")

        boxes = self.detector.detect(rgb_array[:, :, ::-1].copy(), self.prompt)
        if not boxes:
            return VisionResult([], [])
        results = self.segmenter.predict(
            image,
            device=self.device,
            retina_masks=True,
            imgsz=640,
            conf=0.25,
            iou=0.9,
            verbose=False,
        )

        detections: list[Detection3D] = []
        observations: list[VisionObservation] = []
        accepted_masks: list[np.ndarray] = []
        projected: list[tuple[VisionObservation, Any, np.ndarray, np.ndarray]] = []
        crop_boxes: list[np.ndarray] = []
        for box in sorted(boxes, key=lambda item: float(item.score), reverse=True):
            bbox = np.asarray(box.bbox, dtype=float)
            selected = self.segmenter.predictor.prompt(results, bboxes=[bbox.tolist()])[0]
            mask = self._first_mask(selected)
            if mask is None:
                observations.append(self._rejected(box, bbox, None, "no_mask"))
                continue
            if mask.shape != depth_m.shape:
                observations.append(self._rejected(box, bbox, mask, "mask_depth_shape_mismatch"))
                continue
            if any(np.array_equal(mask, accepted) for accepted in accepted_masks):
                observations.append(self._rejected(box, bbox, mask, "duplicate_mask"))
                continue
            accepted_masks.append(mask)

            cleaned = clean_depth_mask(
                mask,
                depth_m,
                min_depth=self.min_depth,
                max_depth=self.max_depth,
            )
            cleaned_pixels = int(cleaned.mask.sum())
            if cleaned_pixels < POINTCLOUD_MIN_POINTS:
                reason = (
                    "insufficient_valid_depth"
                    if cleaned.valid_pixels < POINTCLOUD_MIN_POINTS
                    else "insufficient_main_cluster"
                )
                observations.append(
                    self._rejected(box, bbox, mask, reason, cleaned=cleaned),
                )
                continue
            points = mask_to_pointcloud(
                cleaned.mask,
                depth_m,
                intrinsics,
                optical_to_world,
                depth_scale=1.0,
                min_depth=self.min_depth,
                max_depth=self.max_depth,
                max_points=512,
            )
            if points is None:
                observations.append(self._rejected(box, bbox, mask, "projection_failed", cleaned=cleaned))
                continue

            valid_depth = depth_m[cleaned.mask]
            quality = ProjectionQuality(
                raw_mask_pixels=cleaned.raw_pixels,
                valid_depth_pixels=cleaned.valid_pixels,
                cleaned_pixels=cleaned_pixels,
                projected_points=len(points),
                component_count=cleaned.component_count,
            )
            observation = VisionObservation(
                label=box.label,
                bbox_xyxy=bbox,
                confidence=float(box.score),
                mask=mask,
                quality=quality,
            )
            observations.append(observation)
            projected.append((observation, box, points, valid_depth))
            crop_boxes.append(bbox)

        if not projected:
            return VisionResult([], observations)
        features = self.encoder.encode_image_crops(
            rgb_array[:, :, ::-1].copy(),
            crop_boxes,
            use_cache=False,
        )
        if len(features) != len(projected):
            raise RuntimeError("CLIP returned the wrong number of object features")
        for (observation, box, points, valid_depth), feature in zip(projected, features):
            feature = np.asarray(feature, dtype=np.float32)
            norm = float(np.linalg.norm(feature))
            if feature.size == 0 or not np.isfinite(norm) or norm <= 0:
                raise RuntimeError("CLIP did not return a valid object feature")
            detection = Detection3D(
                position=points.mean(axis=0),
                label=box.label,
                score=float(box.score),
                bbox_2d=observation.bbox_xyxy,
                depth=float(np.median(valid_depth)),
                features=feature/norm,
                points=points,
            )
            detections.append(detection)
            observation.detection = detection
        return VisionResult(detections, observations)

    @staticmethod
    def _first_mask(selected: Any) -> np.ndarray | None:
        if getattr(selected, "masks", None) is None or len(selected) == 0:
            return None
        return selected.masks.data[0].cpu().numpy().astype(bool)

    @staticmethod
    def _rejected(
        box: Any,
        bbox: np.ndarray,
        mask: np.ndarray | None,
        reason: str,
        *,
        cleaned: _CleanedMask | None = None,
    ) -> VisionObservation:
        raw_pixels = int(mask.sum()) if mask is not None else 0
        quality = ProjectionQuality(
            raw_mask_pixels=cleaned.raw_pixels if cleaned else raw_pixels,
            valid_depth_pixels=cleaned.valid_pixels if cleaned else 0,
            cleaned_pixels=int(cleaned.mask.sum()) if cleaned else 0,
            projected_points=0,
            component_count=cleaned.component_count if cleaned else 0,
            rejected_reason=reason,
        )
        return VisionObservation(box.label, bbox, float(box.score), mask, quality)

    def close(self) -> None:
        if not self._closed:
            self.detector.shutdown()
            self.encoder.shutdown()
            self._closed = True

    def __enter__(self) -> Vision:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
