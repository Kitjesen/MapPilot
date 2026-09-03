"""Deep single-frame semantic perception pipeline."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Literal, TypeAlias

import numpy as np

from perception.backends import (
    DetectorSpec,
    ObservationBatch,
    ObservationSource,
    create_observation_source,
)
from perception.tracking.projection import Detection3D
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.semantic import Detection3D as CoreDetection3D
from runtime.msgs.semantic import Region, Relation, SceneGraph

logger = logging.getLogger(__name__)

PerceptionStatus: TypeAlias = Literal[
    "positive",
    "negative",
    "dropped",
    "degraded",
    "failed",
]
DiscardedStatus: TypeAlias = Literal["dropped", "failed"]
CompleteStatus: TypeAlias = Literal["positive", "negative", "degraded"]


@dataclass(frozen=True)
class PerceptionSettings:
    """Resolved, immutable settings for one perception pipeline."""

    default_classes: str = "door . chair . person . desk . stairs . elevator . sign"
    min_depth: float = 0.3
    max_depth: float = 6.0
    u16_depth_scale: float = 0.001
    laplacian_threshold: float = 100.0
    merge_distance: float = 0.5
    tracking_iou_threshold: float = 0.3
    max_objects: int = 200


@dataclass(frozen=True)
class PerceptionOutcome:
    """One atomic set of values for the runtime Module to publish."""

    status: PerceptionStatus
    reason: str
    robot_pose: PoseStamped | None
    detections: tuple[CoreDetection3D, ...] | None
    scene_graph: SceneGraph | None


class PerceptionPipeline:
    """Detect, project, track and assemble a scene graph behind one interface."""

    def __init__(
        self,
        settings: PerceptionSettings,
        detector: DetectorSpec | ObservationSource,
    ) -> None:
        self._settings = settings
        self._detector_spec = detector if isinstance(detector, DetectorSpec) else None
        self._source = None if isinstance(detector, DetectorSpec) else detector
        self._tracker: Any | None = None
        self._ready = False
        self._closed = False
        self._processed_frames = 0
        self._status_counts: dict[str, int] = {
            "positive": 0,
            "negative": 0,
            "dropped": 0,
            "degraded": 0,
            "failed": 0,
        }
        self._last_status = ""
        self._last_reason = ""
        self._last_valid_timestamp = 0.0
        self._latest_detection_count = 0

    def setup(self) -> None:
        """Load the configured source and create the canonical instance tracker."""
        if self._closed:
            raise RuntimeError("perception_pipeline_closed")
        if self._ready:
            return

        source = self._source
        if source is None:
            assert self._detector_spec is not None
            source = create_observation_source(
                self._detector_spec,
                min_depth=self._settings.min_depth,
                max_depth=self._settings.max_depth,
                u16_depth_scale=self._settings.u16_depth_scale,
            )
            self._source = source

        try:
            source.load()
            from perception.tracking.instance_tracker import InstanceTracker

            self._tracker = InstanceTracker(
                merge_distance=self._settings.merge_distance,
                iou_threshold=self._settings.tracking_iou_threshold,
                max_objects=self._settings.max_objects,
            )
        except Exception:
            source.close()
            self._source = None
            self._tracker = None
            raise
        self._ready = True

    def process(self, frame: Any) -> PerceptionOutcome:
        """Process one synchronized frame without publishing side effects."""
        if not self._ready or self._source is None or self._tracker is None:
            raise RuntimeError("perception_pipeline_not_ready")

        robot_pose = _robot_pose(frame.map_from_body, frame.timestamp, frame.frame_id)
        if self._source.name != "sim_scene":
            try:
                bgr = frame.color.to_bgr().data
                if _image_is_blurry(bgr, threshold=self._settings.laplacian_threshold):
                    return self._discard("dropped", "blurry_image", frame.timestamp)
            except (TypeError, ValueError):
                return self._discard("dropped", "invalid_color_image", frame.timestamp)

        try:
            raw_batch = self._source.observe(frame, self._settings.default_classes)
            batch = (
                raw_batch
                if isinstance(raw_batch, ObservationBatch)
                else ObservationBatch(tuple(raw_batch), observed_count=len(raw_batch))
            )
        except ValueError as exc:
            reason = str(exc)
            if reason.startswith(("unsupported_depth_format", "invalid_u16_depth_scale")):
                return self._discard("dropped", reason, frame.timestamp)
            logger.warning("Perception observation rejected: %s", exc)
            return self._discard("failed", "detector_error", frame.timestamp)
        except Exception as exc:
            logger.warning("Perception observation failed: %s", exc)
            return self._discard("failed", "detector_error", frame.timestamp)

        if batch.observed_count > 0 and not batch.detections:
            return self._discard("dropped", "projection_empty", frame.timestamp)

        detections = tuple(batch.detections)
        runtime_detections = to_runtime_detections(
            detections,
            source_ts=frame.timestamp,
        )
        try:
            self._tracker.update(
                list(detections),
                camera_pos=frame.map_from_camera[:3, 3],
                camera_forward=frame.map_from_camera[:3, 2],
                intrinsics_fx=float(frame.intrinsics.fx),
            )
        except Exception as exc:
            logger.warning("Instance tracking failed; publishing current observations: %s", exc)
            return self._complete(
                "degraded",
                "tracker_error",
                robot_pose,
                runtime_detections,
                _fallback_scene_graph(
                    runtime_detections,
                    frame_id=frame.frame_id,
                    source_ts=frame.timestamp,
                ),
                frame.timestamp,
            )

        if not detections:
            return self._complete(
                "negative",
                "",
                robot_pose,
                (),
                SceneGraph(
                    objects=[],
                    relations=[],
                    regions=[],
                    ts=frame.timestamp,
                    frame_id=frame.frame_id,
                ),
                frame.timestamp,
            )

        try:
            scene_graph = _scene_graph_from_tracker(
                self._tracker,
                runtime_detections,
                frame_id=frame.frame_id,
                source_ts=frame.timestamp,
            )
        except Exception as exc:
            logger.warning("Scene-graph serialization failed; using current observations: %s", exc)
            return self._complete(
                "degraded",
                "scene_graph_error",
                robot_pose,
                runtime_detections,
                _fallback_scene_graph(
                    runtime_detections,
                    frame_id=frame.frame_id,
                    source_ts=frame.timestamp,
                ),
                frame.timestamp,
            )

        return self._complete(
            "positive",
            "",
            robot_pose,
            runtime_detections,
            scene_graph,
            frame.timestamp,
        )

    def health(self) -> dict[str, Any]:
        source_health = self._source.health() if self._source is not None else {}
        tracked_objects = 0
        if self._tracker is not None:
            tracked_objects = len(getattr(self._tracker, "objects", {}))
        return {
            "detector_type": (
                self._source.name
                if self._source is not None
                else self._detector_spec.name if self._detector_spec is not None else ""
            ),
            "detector_ready": self._ready and bool(source_health.get("loaded", False)),
            "tracker_ready": self._tracker is not None,
            "processed_frames": self._processed_frames,
            "status_counts": dict(self._status_counts),
            "last_status": self._last_status,
            "last_reason": self._last_reason,
            "last_valid_timestamp": self._last_valid_timestamp,
            "latest_detections": self._latest_detection_count,
            "tracked_objects": tracked_objects,
            "source": source_health,
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._ready = False
        if self._source is not None:
            self._source.close()
        _close_resource(self._tracker)
        self._tracker = None

    def _record(self, outcome: PerceptionOutcome, timestamp: float) -> PerceptionOutcome:
        self._processed_frames += 1
        self._status_counts[outcome.status] += 1
        self._last_status = outcome.status
        self._last_reason = outcome.reason
        self._latest_detection_count = (
            len(outcome.detections) if outcome.detections is not None else 0
        )
        if outcome.status in {"positive", "negative", "degraded"}:
            self._last_valid_timestamp = float(timestamp)
        return outcome

    def _discard(
        self,
        status: DiscardedStatus,
        reason: str,
        timestamp: float,
    ) -> PerceptionOutcome:
        return self._record(
            PerceptionOutcome(status, reason, None, None, None),
            timestamp,
        )

    def _complete(
        self,
        status: CompleteStatus,
        reason: str,
        robot_pose: PoseStamped,
        detections: tuple[CoreDetection3D, ...],
        scene_graph: SceneGraph,
        timestamp: float,
    ) -> PerceptionOutcome:
        return self._record(
            PerceptionOutcome(
                status,
                reason,
                robot_pose,
                detections,
                scene_graph,
            ),
            timestamp,
        )


def to_runtime_detections(
    detections: list[Any] | tuple[Any, ...],
    *,
    source_ts: float,
) -> tuple[CoreDetection3D, ...]:
    """Convert projection/simulation detections to runtime messages."""
    converted: list[CoreDetection3D] = []
    for detection in detections:
        position = detection.position
        feature = getattr(detection, "features", None)
        has_feature = feature is not None and np.asarray(feature).size > 0
        track_id = getattr(detection, "track_id", None)
        converted.append(
            CoreDetection3D(
                id=f"track_{track_id}" if track_id is not None else "",
                label=str(detection.label),
                confidence=float(detection.score),
                position=Vector3(
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                ),
                bbox_2d=[float(value) for value in detection.bbox_2d],
                clip_feature=np.asarray(feature) if has_feature else None,
                ts=float(source_ts),
            )
        )
    return tuple(converted)


def _robot_pose(matrix: np.ndarray, source_ts: float, frame_id: str) -> PoseStamped:
    yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return PoseStamped(
        pose=Pose(
            position=Vector3(
                float(matrix[0, 3]),
                float(matrix[1, 3]),
                float(matrix[2, 3]),
            ),
            orientation=Quaternion.from_yaw(yaw),
        ),
        ts=float(source_ts),
        frame_id=frame_id,
    )


def _image_is_blurry(image: np.ndarray, *, threshold: float) -> bool:
    try:
        from perception.detection.laplacian_filter import is_blurry

        return bool(is_blurry(image, threshold=threshold))
    except ImportError:
        if image.ndim == 3:
            b, g, r = image[..., 0], image[..., 1], image[..., 2]
            gray = 0.114 * b + 0.587 * g + 0.299 * r
        elif image.ndim == 2:
            gray = image.astype(np.float64, copy=False)
        else:
            raise ValueError("image must be HxW or HxWx3") from None
        padded = np.pad(gray.astype(np.float64, copy=False), 1, mode="edge")
        laplacian = (
            padded[:-2, 1:-1]
            + padded[2:, 1:-1]
            + padded[1:-1, :-2]
            + padded[1:-1, 2:]
            - 4.0 * padded[1:-1, 1:-1]
        )
        return float(laplacian.var()) < float(threshold)


def _scene_graph_from_tracker(
    tracker: Any,
    latest_detections: tuple[CoreDetection3D, ...],
    *,
    frame_id: str,
    source_ts: float,
) -> SceneGraph:
    data = json.loads(tracker.get_scene_graph_json())
    objects: list[CoreDetection3D] = []
    for raw in data.get("objects", []):
        position = raw.get("position", [0.0, 0.0, 0.0])
        if isinstance(position, dict):
            px = float(position.get("x", 0.0))
            py = float(position.get("y", 0.0))
            pz = float(position.get("z", 0.0))
        else:
            px, py, pz = (float(position[index]) for index in range(3))
        label = str(raw.get("label", ""))
        matched = _match_detection(latest_detections, label, px, py, pz)
        object_id = str(raw.get("id", ""))
        bbox_2d: list[float] = []
        clip_feature = None
        if matched is not None:
            object_id = matched.id or object_id
            bbox_2d = list(matched.bbox_2d)
            if matched.clip_feature is not None:
                clip_feature = np.array(matched.clip_feature, copy=True)
        objects.append(
            CoreDetection3D(
                id=object_id,
                label=label,
                confidence=float(raw.get("confidence", raw.get("score", 0.0))),
                position=Vector3(px, py, pz),
                bbox_2d=bbox_2d,
                clip_feature=clip_feature,
                ts=float(source_ts),
            )
        )

    relations = [
        Relation(
            subject_id=str(raw.get("subject_id", raw.get("subject", ""))),
            predicate=str(raw.get("predicate", raw.get("relation", ""))),
            object_id=str(raw.get("object_id", raw.get("object", ""))),
            confidence=float(raw.get("confidence", 1.0)),
        )
        for raw in data.get("relations", [])
    ]
    regions = [
        Region(
            name=str(raw.get("name", raw.get("room_type", ""))),
            object_ids=[str(value) for value in raw.get("object_ids", [])],
        )
        for raw in data.get("rooms", data.get("regions", []))
    ]
    return SceneGraph(
        objects=objects,
        relations=relations,
        regions=regions,
        ts=float(source_ts),
        frame_id=frame_id,
    )


def _fallback_scene_graph(
    detections: tuple[CoreDetection3D, ...],
    *,
    frame_id: str,
    source_ts: float,
) -> SceneGraph:
    return SceneGraph(
        objects=[_clone_detection(detection) for detection in detections],
        relations=[],
        regions=[],
        ts=float(source_ts),
        frame_id=frame_id,
    )


def _clone_detection(detection: CoreDetection3D) -> CoreDetection3D:
    return CoreDetection3D(
        id=detection.id,
        label=detection.label,
        confidence=detection.confidence,
        position=Vector3(
            detection.position.x,
            detection.position.y,
            detection.position.z,
        ),
        bbox_2d=list(detection.bbox_2d),
        clip_feature=(
            np.array(detection.clip_feature, copy=True)
            if detection.clip_feature is not None
            else None
        ),
        ts=detection.ts,
    )


def _match_detection(
    detections: tuple[CoreDetection3D, ...],
    label: str,
    px: float,
    py: float,
    pz: float,
) -> CoreDetection3D | None:
    best = None
    best_distance = 1.5
    for detection in detections:
        if label and detection.label.lower() != label.lower():
            continue
        distance = float(
            np.linalg.norm(
                np.array(
                    [
                        detection.position.x - px,
                        detection.position.y - py,
                        detection.position.z - pz,
                    ]
                )
            )
        )
        if distance < best_distance:
            best_distance = distance
            best = detection
    return best


def _close_resource(resource: Any | None) -> None:
    if resource is None:
        return
    for method_name in ("shutdown", "close", "reset"):
        method = getattr(resource, method_name, None)
        if callable(method):
            try:
                method()
            except Exception as exc:
                logger.warning("%s.%s() failed: %s", type(resource).__name__, method_name, exc)
            return
