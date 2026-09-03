from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from perception.backends import DetectorSpec, ObservationBatch
from perception.frames import SynchronizedFrame
from perception.pipeline import (
    PerceptionPipeline,
    PerceptionSettings,
    to_runtime_detections,
)
from perception.tracking.projection import Detection3D
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


class _ObservationSource:
    def __init__(
        self,
        detections: list[Detection3D] | ObservationBatch,
        *,
        name: str = "test",
        observe_error: Exception | None = None,
        load_error: Exception | None = None,
    ) -> None:
        self.name = name
        self.detections = detections
        self.observe_error = observe_error
        self.load_error = load_error
        self.loaded = False
        self.closed = False
        self.observe_calls = 0
        self.close_calls = 0

    def load(self) -> None:
        if self.load_error is not None:
            raise self.load_error
        self.loaded = True

    def observe(
        self,
        _frame: SynchronizedFrame,
        _prompt: str,
    ) -> list[Detection3D] | ObservationBatch:
        self.observe_calls += 1
        if self.observe_error is not None:
            raise self.observe_error
        if isinstance(self.detections, ObservationBatch):
            return self.detections
        return list(self.detections)

    def health(self) -> dict[str, object]:
        return {"backend": self.name, "loaded": self.loaded and not self.closed}

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True


class _ArrayThatFailsTrackerCopy(np.ndarray):
    def copy(self, *_args: object, **_kwargs: object) -> np.ndarray:
        raise RuntimeError("tracker storage failed")


def _settings() -> PerceptionSettings:
    return PerceptionSettings(
        default_classes="chair",
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
        laplacian_threshold=0.0,
        merge_distance=0.5,
        tracking_iou_threshold=0.3,
        max_objects=20,
    )


def _frame() -> SynchronizedFrame:
    map_from_body = np.eye(4)
    map_from_body[:3, 3] = [1.0, 2.0, 0.25]
    map_from_camera = map_from_body.copy()
    return SynchronizedFrame(
        color=Image(
            data=np.zeros((20, 20, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=10.0,
            frame_id="camera",
        ),
        depth=Image(
            data=np.full((20, 20), 1000, dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=10.0,
            frame_id="camera",
        ),
        intrinsics=CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=10.0,
            cy=10.0,
            width=20,
            height=20,
            depth_scale=0.001,
            ts=10.0,
            frame_id="camera",
        ),
        odometry=Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.25)),
            ts=10.0,
            frame_id="map",
        ),
        map_odom=None,
        timestamp=10.0,
        frame_id="map",
        map_from_body=map_from_body,
        map_from_camera=map_from_camera,
    )


def _detection() -> Detection3D:
    return Detection3D(
        position=np.array([2.0, 2.0, 0.5]),
        label="chair",
        score=0.9,
        bbox_2d=np.array([4.0, 4.0, 16.0, 16.0]),
        depth=1.0,
        features=np.array([]),
    )


def test_positive_observation_returns_one_coherent_emission() -> None:
    source = _ObservationSource([_detection()])
    pipeline = PerceptionPipeline(_settings(), source)
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "positive"
    assert outcome.reason == ""
    assert outcome.robot_pose is not None
    assert outcome.robot_pose.position == Vector3(1.0, 2.0, 0.25)
    assert outcome.detections is not None
    assert len(outcome.detections) == 1
    assert outcome.detections[0].label == "chair"
    assert outcome.detections[0].ts == 10.0
    assert outcome.scene_graph is not None
    assert outcome.scene_graph.ts == 10.0
    assert outcome.scene_graph.frame_id == "map"

    pipeline.close()
    assert source.closed is True


def test_tracker_failure_falls_back_to_current_frame_not_stale_state() -> None:
    source = _ObservationSource([_detection()])
    pipeline = PerceptionPipeline(_settings(), source)
    pipeline.setup()
    first = pipeline.process(_frame())
    assert first.status == "positive"

    current = _detection()
    current.label = "person"
    current.position = np.array([3.0, 2.0, 0.5]).view(_ArrayThatFailsTrackerCopy)
    source.detections = [current]

    outcome = pipeline.process(_frame())

    assert outcome.status == "degraded"
    assert outcome.reason == "tracker_error"
    assert outcome.detections is not None
    assert [detection.label for detection in outcome.detections] == ["person"]
    assert outcome.scene_graph is not None
    assert [obj.label for obj in outcome.scene_graph.objects] == ["person"]
    assert outcome.scene_graph.ts == 10.0


def test_valid_zero_observation_is_negative_not_a_processing_failure() -> None:
    pipeline = PerceptionPipeline(_settings(), _ObservationSource([]))
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "negative"
    assert outcome.reason == ""
    assert outcome.robot_pose is not None
    assert outcome.detections == ()
    assert outcome.scene_graph is not None
    assert outcome.scene_graph.objects == []


def test_failed_projection_drops_frame_without_fabricating_empty_observation() -> None:
    source = _ObservationSource(ObservationBatch((), observed_count=1))
    pipeline = PerceptionPipeline(_settings(), source)
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "dropped"
    assert outcome.reason == "projection_empty"
    assert outcome.robot_pose is None
    assert outcome.detections is None
    assert outcome.scene_graph is None


def test_blurry_real_frame_is_dropped_before_detector_runs() -> None:
    source = _ObservationSource([_detection()])
    settings = replace(_settings(), laplacian_threshold=100.0)
    pipeline = PerceptionPipeline(settings, source)
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "dropped"
    assert outcome.reason == "blurry_image"
    assert source.observe_calls == 0


def test_sim_scene_observation_bypasses_image_blur_filter() -> None:
    source = _ObservationSource([_detection()], name="sim_scene")
    settings = replace(_settings(), laplacian_threshold=100.0)
    pipeline = PerceptionPipeline(settings, source)
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "positive"
    assert source.observe_calls == 1


def test_detector_exception_is_failed_without_publishable_values() -> None:
    source = _ObservationSource([], observe_error=RuntimeError("model crashed"))
    pipeline = PerceptionPipeline(_settings(), source)
    pipeline.setup()

    outcome = pipeline.process(_frame())

    assert outcome.status == "failed"
    assert outcome.reason == "detector_error"
    assert outcome.robot_pose is None
    assert outcome.detections is None
    assert outcome.scene_graph is None


def test_configured_backend_load_failure_aborts_setup_and_closes_candidate() -> None:
    source = _ObservationSource([], load_error=RuntimeError("weights missing"))
    pipeline = PerceptionPipeline(_settings(), source)

    with pytest.raises(RuntimeError, match="weights missing"):
        pipeline.setup()

    assert source.close_calls == 1


def test_declared_missing_sim_world_aborts_setup() -> None:
    pipeline = PerceptionPipeline(
        _settings(),
        DetectorSpec(
            name="sim_scene",
            world="sim/packages/worlds/missing/physics/world.xml",
        ),
    )

    with pytest.raises(FileNotFoundError, match="simulation world does not exist"):
        pipeline.setup()
    assert pipeline.health()["detector_ready"] is False


def test_close_is_idempotent_and_health_reports_outcome_counts() -> None:
    source = _ObservationSource([])
    pipeline = PerceptionPipeline(_settings(), source)
    pipeline.setup()
    pipeline.process(_frame())

    health = pipeline.health()
    pipeline.close()
    pipeline.close()

    assert health["processed_frames"] == 1
    assert health["status_counts"]["negative"] == 1
    assert health["last_valid_timestamp"] == 10.0
    assert source.close_calls == 1


def test_runtime_detection_conversion_preserves_sim_track_identity() -> None:
    detection = _detection()
    detection.track_id = "sim-person-7"

    converted = to_runtime_detections([detection], source_ts=42.0)

    assert len(converted) == 1
    assert converted[0].id == "track_sim-person-7"
    assert converted[0].label == "chair"
    assert converted[0].ts == 42.0
