"""Canonical perception backend providers and observation adapters."""

from __future__ import annotations

import logging
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol, cast, overload

import numpy as np

from perception.tracking.projection import (
    Detection3D,
    bbox_center_depth,
    depth_scale_for_image,
    mask_to_pointcloud,
    pointcloud_centroid,
    project_to_3d,
    transform_point,
)
from runtime.registry import get, register

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectorSpec:
    """All construction data needed by a detector provider."""

    name: str = "yoloe"
    model_size: str = "l"
    confidence: float = 0.3
    iou_threshold: float = 0.45
    max_detections: int = 64
    min_box_size_px: int = 12
    device: str = ""
    model_path: str = ""
    world: str = ""
    scenario_entities: tuple[Mapping[str, Any], ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", self.name.strip().lower())
        object.__setattr__(self, "scenario_entities", tuple(self.scenario_entities))


@dataclass(frozen=True)
class ObservationBatch(Sequence[Detection3D]):
    """Projected observations plus the pre-projection observation count."""

    detections: tuple[Detection3D, ...]
    observed_count: int

    def __iter__(self) -> Iterator[Detection3D]:
        return iter(self.detections)

    def __len__(self) -> int:
        return len(self.detections)

    @overload
    def __getitem__(self, index: int) -> Detection3D: ...

    @overload
    def __getitem__(self, index: slice) -> tuple[Detection3D, ...]: ...

    def __getitem__(
        self,
        index: int | slice,
    ) -> Detection3D | tuple[Detection3D, ...]:
        return self.detections[index]


class ObservationSource(Protocol):
    """Real seam between the RGB-D and simulation observation adapters."""

    @property
    def name(self) -> str: ...

    def load(self) -> None: ...

    def observe(self, frame: Any, text_prompt: str) -> ObservationBatch: ...

    def health(self) -> dict[str, Any]: ...

    def close(self) -> None: ...


class RgbdObservationSource:
    """Run a 2-D detector and project its observations into the map frame."""

    def __init__(
        self,
        detector: Any,
        *,
        min_depth: float,
        max_depth: float,
        u16_depth_scale: float,
        detector_tracker: Any | None = None,
        name: str = "rgbd",
    ) -> None:
        self._detector = detector
        self._detector_tracker = detector_tracker
        self._min_depth = float(min_depth)
        self._max_depth = float(max_depth)
        self._u16_depth_scale = float(u16_depth_scale)
        self._name = name
        self._loaded = False
        self._closed = False
        self._tracker_fallbacks = 0

    @property
    def name(self) -> str:
        return self._name

    def load(self) -> None:
        load_model = getattr(self._detector, "load_model", None)
        if callable(load_model):
            load_model()
        self._loaded = True

    def observe(self, frame: Any, text_prompt: str) -> ObservationBatch:
        if not self._loaded:
            raise RuntimeError("observation_source_not_loaded")

        color = frame.color
        bgr = color.to_bgr().data
        depth = frame.depth
        intrinsics = frame.intrinsics
        depth_scale = depth_scale_for_image(
            depth,
            intrinsics,
            u16_fallback=self._u16_depth_scale,
        )
        detections_2d: list[Any]
        if self._detector_tracker is not None:
            try:
                detections_2d = list(self._detector_tracker.track(bgr, text_prompt))
            except Exception as exc:
                self._tracker_fallbacks += 1
                logger.warning("2-D tracker failed; using raw detector output: %s", exc)
                detections_2d = list(self._detector.detect(bgr, text_prompt))
        else:
            detections_2d = list(self._detector.detect(bgr, text_prompt))

        if not detections_2d:
            return ObservationBatch((), observed_count=0)

        transform = np.asarray(frame.map_from_camera, dtype=np.float64)
        K = intrinsics.K_matrix
        D = intrinsics.D_vector
        projected: list[Detection3D] = []
        for detection in detections_2d:
            result = self._project_detection(
                detection,
                depth.data,
                intrinsics,
                transform,
                depth_scale,
                K,
                D,
            )
            if result is not None:
                projected.append(result)
        return ObservationBatch(tuple(projected), observed_count=len(detections_2d))

    def _project_detection(
        self,
        detection: Any,
        depth: np.ndarray,
        intrinsics: Any,
        transform: np.ndarray,
        depth_scale: float,
        K: np.ndarray,
        D: np.ndarray,
    ) -> Detection3D | None:
        centroid = None
        center_depth = None
        points = None
        mask = getattr(detection, "mask", None)
        if mask is not None:
            points = mask_to_pointcloud(
                mask=mask,
                depth_image=depth,
                intrinsics=intrinsics,
                tf_camera_to_world=transform,
                depth_scale=depth_scale,
                min_depth=self._min_depth,
                max_depth=self._max_depth,
                K=K,
                D=D,
            )
            if points is not None and len(points) > 0:
                centroid = pointcloud_centroid(points)
                center_depth = float(np.linalg.norm(centroid - transform[:3, 3]))

        if centroid is None:
            center_depth = bbox_center_depth(
                depth,
                detection.bbox,
                depth_scale=depth_scale,
            )
            if (
                center_depth is None
                or center_depth < self._min_depth
                or center_depth > self._max_depth
            ):
                return None
            cx = (detection.bbox[0] + detection.bbox[2]) / 2.0
            cy = (detection.bbox[1] + detection.bbox[3]) / 2.0
            camera_point = project_to_3d(
                cx,
                cy,
                center_depth,
                intrinsics,
                K=K,
                D=D,
            )
            centroid = transform_point(camera_point, transform)

        if center_depth is None:
            return None
        return Detection3D(
            position=centroid,
            label=detection.label,
            score=float(detection.score),
            bbox_2d=np.asarray(detection.bbox),
            depth=float(center_depth),
            features=np.asarray(getattr(detection, "features", np.array([]))),
            points=points if points is not None else np.empty((0, 3)),
            track_id=getattr(detection, "track_id", None),
        )

    def health(self) -> dict[str, Any]:
        info: dict[str, Any] = {
            "backend": self._name,
            "loaded": self._loaded and not self._closed,
            "detector_tracker_ready": self._detector_tracker is not None,
            "detector_tracker_fallbacks": self._tracker_fallbacks,
        }
        if self._detector_tracker is not None:
            info["detector_tracker_backend"] = getattr(
                self._detector_tracker,
                "backend_name",
                "unknown",
            )
            info["person_tracking"] = getattr(
                self._detector_tracker,
                "person_counts",
                {},
            )
        return info

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        _close_resources(self._detector_tracker, self._detector)


class SimSceneObservationSource:
    """Read semantic observations directly from simulation scene metadata."""

    def __init__(self, observer: Any, *, name: str = "sim_scene") -> None:
        self._observer = observer
        self._name = name
        self._loaded = False
        self._closed = False

    @property
    def name(self) -> str:
        return self._name

    def load(self) -> None:
        load_model = getattr(self._observer, "load_model", None)
        if callable(load_model):
            load_model()
        self._loaded = True

    def observe(self, frame: Any, text_prompt: str) -> ObservationBatch:
        if not self._loaded:
            raise RuntimeError("observation_source_not_loaded")
        detections = tuple(
            self._observer.observe(
                tf_camera_to_world=frame.map_from_camera,
                intrinsics=frame.intrinsics,
                text_prompt=text_prompt,
            )
        )
        return ObservationBatch(detections, observed_count=len(detections))

    def health(self) -> dict[str, Any]:
        return {
            "backend": self._name,
            "loaded": self._loaded and not self._closed,
            "detector_tracker_ready": False,
            "detector_tracker_fallbacks": 0,
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        _close_resources(self._observer)


@register("detector", "yoloe", description="YOLO-E open-vocabulary instance detector")
class YOLOEDetectorProvider:
    label = "YOLOEDetector"

    @staticmethod
    def create(spec: DetectorSpec) -> Any:
        from perception.detection.yoloe_detector import YOLOEDetector

        return YOLOEDetector(
            model_size=spec.model_size,
            confidence=spec.confidence,
            iou_threshold=spec.iou_threshold,
            device=spec.device,
            max_detections=spec.max_detections,
        )


@register("detector", "yolo_world", description="YOLO-World open-vocabulary detector")
class YOLOWorldDetectorProvider:
    label = "YOLOWorldDetector"

    @staticmethod
    def create(spec: DetectorSpec) -> Any:
        from perception.detection.yolo_world_detector import YOLOWorldDetector

        return YOLOWorldDetector(
            model_size=spec.model_size,
            confidence=spec.confidence,
            iou_threshold=spec.iou_threshold,
            device=spec.device,
        )


@register("detector", "bpu", description="D-Robotics Nash BPU detector")
class BPUDetectorProvider:
    label = "BPUDetector"

    @staticmethod
    def create(spec: DetectorSpec) -> Any:
        from perception.detection.bpu_detector import BPUDetector

        return BPUDetector(
            model_path=spec.model_path or None,
            confidence=spec.confidence,
            iou_threshold=spec.iou_threshold,
            max_detections=spec.max_detections,
            min_box_size_px=spec.min_box_size_px,
        )


@register("detector", "grounding_dino", description="GroundingDINO open-vocabulary detector")
class GroundingDINODetectorProvider:
    label = "GroundingDINODetector"

    @staticmethod
    def create(spec: DetectorSpec) -> Any:
        from perception.detection.grounding_dino_detector import GroundingDINODetector

        kwargs: dict[str, Any] = {"box_threshold": spec.confidence}
        if spec.device:
            kwargs["device"] = spec.device
        if spec.model_path:
            kwargs["weights_path"] = spec.model_path
        return GroundingDINODetector(**kwargs)


@register("detector", "sim_scene", description="Simulation scene observer")
class SimSceneDetectorProvider:
    label = "SimSceneObserver"

    @staticmethod
    def create(spec: DetectorSpec) -> Any:
        from perception.detection.sim_scene_observer import SimSceneObserver

        return SimSceneObserver(
            world=spec.world,
            scenario_entities=spec.scenario_entities,
        )


@register("encoder", "clip", description="CLIP image/text encoder")
class CLIPEncoderProvider:
    label = "CLIPEncoder"

    @staticmethod
    def create() -> Any:
        from perception.encoding.clip_encoder import CLIPEncoder

        return CLIPEncoder()


@register("encoder", "mobileclip", description="MobileCLIP text encoder")
class MobileCLIPEncoderProvider:
    label = "MobileCLIPEncoder"

    @staticmethod
    def create() -> Any:
        from perception.encoding.mobileclip_encoder import MobileCLIPEncoder

        return MobileCLIPEncoder()


def create_observation_source(
    spec: DetectorSpec,
    *,
    min_depth: float,
    max_depth: float,
    u16_depth_scale: float,
) -> ObservationSource:
    """Construct the configured observation adapter; loading remains explicit."""
    if not spec.name:
        raise ValueError("detector name must not be empty")
    provider = get("detector", spec.name)
    backend = cast(Any, provider).create(spec)
    if spec.name == "sim_scene":
        return SimSceneObservationSource(backend, name=spec.name)

    detector_tracker = None
    if spec.name == "bpu":
        try:
            from perception.tracking.bpu_tracker import BPUTracker

            detector_tracker = BPUTracker(backend, tracker_type="botsort")
        except Exception as exc:
            logger.warning("BPU 2-D tracker unavailable; using raw detector: %s", exc)
    return RgbdObservationSource(
        backend,
        min_depth=min_depth,
        max_depth=max_depth,
        u16_depth_scale=u16_depth_scale,
        detector_tracker=detector_tracker,
        name=spec.name,
    )


def _close_resources(*resources: Any | None) -> None:
    seen: set[int] = set()
    for resource in resources:
        if resource is None or id(resource) in seen:
            continue
        seen.add(id(resource))
        for method_name in ("shutdown", "close", "reset"):
            method = getattr(resource, method_name, None)
            if not callable(method):
                continue
            try:
                method()
            except Exception as exc:
                logger.warning("%s.%s() failed: %s", type(resource).__name__, method_name, exc)
            break
