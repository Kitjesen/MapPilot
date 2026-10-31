from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any

import numpy as np

import perception.module as perception_module
from perception.frames import FrameDrop
from perception.module import PerceptionModule
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.nav import NavigationState
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


class _Synchronizer:
    def __init__(self, **_kwargs: Any) -> None:
        self.color_frames = 0
        self._revision = 0
        self._generation: int | None = None
        self._map_id = ""
        self._content_epoch = 0

    def push_color(self, frame: Any) -> tuple[Any, ...]:
        self.color_frames += 1
        generation = int(getattr(frame, "source_generation", self._generation or 0) or 0)
        if self._generation is None:
            self._generation = generation
        elif generation != self._generation:
            self._generation = generation
            self._revision += 1
        frame.color = Image(
            data=np.zeros((4, 4, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=frame.ts,
            source_generation=int(self._generation or 0),
            source_sequence=int(getattr(frame, "source_sequence", 0) or 0),
        )
        frame.depth = Image(
            data=np.ones((4, 4), dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=frame.ts,
            source_generation=int(self._generation or 0),
            source_sequence=int(getattr(frame, "source_sequence", 0) or 0),
        )
        frame.intrinsics = CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            width=4,
            height=4,
            ts=frame.ts,
            source_generation=int(self._generation or 0),
            source_sequence=int(getattr(frame, "source_sequence", 0) or 0),
        )
        frame.odometry_ts = frame.ts
        frame.map_odom_ts = frame.ts
        frame.map_from_camera = np.eye(4)
        frame.source_generation = int(self._generation or 0)
        frame.map_id = self._map_id
        frame.content_epoch = self._content_epoch
        frame.context_revision = self._revision
        frame.context_token = (
            int(self._generation or 0),
            self._map_id,
            self._content_epoch,
        )
        return (frame,)

    def push_depth(self, _frame: Any) -> tuple[Any, ...]:
        return ()

    def push_camera_info(self, _info: Any) -> tuple[Any, ...]:
        return ()

    def push_odometry(self, _odom: Any) -> tuple[Any, ...]:
        return ()

    def push_map_odom(self, _transform: Any) -> tuple[Any, ...]:
        return ()

    def push_navigation_state(self, state: NavigationState) -> tuple[Any, ...]:
        token = (state.map_id, state.map_content_epoch)
        if token == (self._map_id, self._content_epoch):
            return ()
        self._map_id, self._content_epoch = token
        self._revision += 1
        return (FrameDrop("map_changed", state.ts),)

    def context_revision(self) -> int:
        return self._revision

    def context_token(self) -> tuple[int, str, int]:
        return int(self._generation or 0), self._map_id, self._content_epoch

    def health(self) -> dict[str, Any]:
        return {
            "color_frames": self.color_frames,
            "matched_frames": self.color_frames,
            "dropped_frames": 0,
            "last_drop_reason": "",
            "drop_reasons": {},
            "buffer_sizes": {},
        }


class _Pipeline:
    def __init__(
        self,
        *,
        block: bool = False,
        fail_setup: bool = False,
        outcome_status: str = "negative",
    ) -> None:
        self.block = block
        self.fail_setup = fail_setup
        self.outcome_status = outcome_status
        self.entered = threading.Event()
        self.release = threading.Event()
        self.processed: list[Any] = []
        self.close_count = 0
        self.reset_count = 0

    def setup(self) -> None:
        if self.fail_setup:
            raise RuntimeError("backend load failed")

    def process(self, frame: Any) -> Any:
        self.entered.set()
        if self.block:
            assert self.release.wait(2.0)
        self.processed.append(frame)
        return SimpleNamespace(
            status=self.outcome_status,
            reason="detector_error" if self.outcome_status == "failed" else "",
            robot_pose=PoseStamped(ts=float(frame.ts), frame_id="map"),
            detections=(),
            scene_graph=SceneGraph(
                objects=[],
                relations=[],
                regions=[],
                ts=float(frame.ts),
                frame_id="map",
            ),
        )

    def health(self) -> dict[str, Any]:
        return {
            "detector_ready": True,
            "tracker_ready": True,
            "tracked_objects": 0,
            "detector_tracker_ready": False,
        }

    def reset_spatial_context(self) -> None:
        self.reset_count += 1

    def close(self) -> None:
        self.close_count += 1


def _module(monkeypatch: Any, pipeline: _Pipeline) -> PerceptionModule:
    monkeypatch.setattr(perception_module, "FrameSynchronizer", _Synchronizer)
    monkeypatch.setattr(
        perception_module,
        "PerceptionPipeline",
        lambda _settings, _detector: pipeline,
    )
    return PerceptionModule(
        settings=object(),
        detector=SimpleNamespace(name="fake"),
        camera_to_body=np.eye(4),
        skip_frames=1,
        max_rgbd_skew_s=0.05,
        max_odom_age_s=0.1,
        max_map_odom_age_s=0.5,
    )


def _wait_until(predicate: Any, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not reached before timeout")


def test_module_keeps_the_host_builder_contract(monkeypatch: Any) -> None:
    module = _module(monkeypatch, _Pipeline())

    assert set(module.ports_in) == {
        "color_image",
        "depth_image",
        "camera_info",
        "odometry",
        "map_odom_tf",
        "navigation_state",
    }
    assert set(module.ports_out) == {
        "observation",
        "scene_graph",
        "detections_3d",
        "robot_pose",
        "observation_image",
    }
    assert module.layer == 3


def test_worker_coalesces_pending_frames_and_preserves_publish_order(monkeypatch: Any) -> None:
    pipeline = _Pipeline(block=True)
    module = _module(monkeypatch, pipeline)
    published: list[tuple[str, float]] = []
    module.observation_image.subscribe(lambda msg: published.append(("image", msg.ts)))
    module.robot_pose.subscribe(lambda msg: published.append(("pose", msg.ts)))
    module.detections_3d.subscribe(lambda _msg: published.append(("detections", 0.0)))
    module.scene_graph.subscribe(lambda msg: published.append(("graph", msg.ts)))
    module.setup()
    module.start()
    try:
        started = time.monotonic()
        module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
        assert time.monotonic() - started < 0.1
        assert pipeline.entered.wait(1.0)

        module.color_image._deliver(SimpleNamespace(ts=2.0, timestamp=2.0))
        module.color_image._deliver(SimpleNamespace(ts=3.0, timestamp=3.0))
        pipeline.release.set()
        _wait_until(lambda: len(pipeline.processed) == 2)

        assert [frame.ts for frame in pipeline.processed] == [1.0, 3.0]
        assert [name for name, _ts in published] == [
            "image",
            "pose",
            "detections",
            "graph",
            "image",
            "pose",
            "detections",
            "graph",
        ]
        assert [stamp for name, stamp in published if name == "image"] == [1.0, 3.0]
        health = module.health()
        assert health["processed_frames"] == 2
        assert health["coalesced_frames"] == 1
        assert module.startup_readiness() is None
    finally:
        module.stop()

    module.stop()
    assert pipeline.close_count == 1


def test_common_observation_is_one_atomic_source_and_map_snapshot(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline()
    module = _module(monkeypatch, pipeline)
    module._product_session_id = "product-session-7"
    observations: list[Any] = []
    module.observation.subscribe(observations.append)
    module.setup()
    module.start()
    try:
        module.color_image._deliver(
            SimpleNamespace(
                ts=12.0,
                timestamp=12.0,
                source_generation=44,
                source_sequence=8,
            )
        )
        _wait_until(lambda: any(value.status == "negative" for value in observations))

        observation = next(value for value in observations if value.status == "negative")
        assert observation.product_session_id == "product-session-7"
        assert observation.source_ts == 12.0
        assert observation.source_generation == 44
        assert observation.image is not None
        assert observation.depth is not None
        assert observation.camera_info is not None
        assert observation.robot_pose is not None
        assert observation.map_from_camera is not None
        assert observation.detections == ()
        assert observation.scene_graph is not None
        assert observation.odometry_ts == 12.0
        assert observation.map_odom_ts == 12.0
    finally:
        module.stop()


def test_reverse_timestamp_is_ignored_without_invalidating_current_follow(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline()
    module = _module(monkeypatch, pipeline)
    observations: list[Any] = []
    module.observation.subscribe(observations.append)
    module.setup()
    module.start()
    try:
        module.color_image._deliver(SimpleNamespace(ts=2.0, timestamp=2.0))
        _wait_until(lambda: any(value.status == "negative" for value in observations))
        count = len(observations)

        module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
        _wait_until(lambda: module.health()["failure_counts"].get("stale_frame") == 1)

        assert len(observations) == count
        assert [frame.ts for frame in pipeline.processed] == [2.0]
    finally:
        module.stop()


def test_map_switch_notifies_immediately_and_discards_inflight_evidence(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline(block=True)
    module = _module(monkeypatch, pipeline)
    observations: list[Any] = []
    module.observation.subscribe(observations.append)
    module.setup()
    module.start()
    try:
        module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
        assert pipeline.entered.wait(1.0)

        module.navigation_state._deliver(
            NavigationState(
                ts=1.1,
                boot_id="nav-boot",
                sequence=1,
                map_id="building-b",
                map_content_epoch=3,
            )
        )

        _wait_until(lambda: any(value.reason == "map_changed" for value in observations))
        assert not any(value.status in {"positive", "negative", "degraded"} for value in observations)
        pipeline.release.set()
        _wait_until(lambda: len(pipeline.processed) == 1)
        time.sleep(0.02)
        assert not any(value.status in {"positive", "negative", "degraded"} for value in observations)

        module.color_image._deliver(SimpleNamespace(ts=2.0, timestamp=2.0))
        _wait_until(lambda: any(value.status == "negative" for value in observations))
        valid = next(value for value in observations if value.status == "negative")
        assert valid.map_id == "building-b"
        assert valid.content_epoch == 3
        assert pipeline.reset_count == 1
    finally:
        module.stop()


def test_stop_waits_for_inflight_work_and_suppresses_its_output(monkeypatch: Any) -> None:
    pipeline = _Pipeline(block=True)
    module = _module(monkeypatch, pipeline)
    outputs: list[Any] = []
    module.scene_graph.subscribe(outputs.append)
    module.setup()
    module.start()
    module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
    assert pipeline.entered.wait(1.0)

    stopped = threading.Event()

    def stop() -> None:
        module.stop()
        stopped.set()

    thread = threading.Thread(target=stop)
    thread.start()
    time.sleep(0.02)
    assert not stopped.is_set()
    pipeline.release.set()
    thread.join(1.0)

    assert stopped.is_set()
    assert outputs == []
    assert pipeline.close_count == 1


def test_stop_discards_pending_frame_while_waiting_for_inflight_work(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline(block=True)
    module = _module(monkeypatch, pipeline)
    outputs: list[Any] = []
    module.scene_graph.subscribe(outputs.append)
    module.setup()
    module.start()
    module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
    assert pipeline.entered.wait(1.0)
    module.color_image._deliver(SimpleNamespace(ts=2.0, timestamp=2.0))

    stopped = threading.Event()
    thread = threading.Thread(target=lambda: (module.stop(), stopped.set()))
    thread.start()
    time.sleep(0.02)
    assert not stopped.is_set()
    pipeline.release.set()
    thread.join(1.0)

    assert stopped.is_set()
    assert [frame.ts for frame in pipeline.processed] == [1.0]
    assert outputs == []
    assert pipeline.close_count == 1


def test_failed_pipeline_outcome_is_not_published(monkeypatch: Any) -> None:
    pipeline = _Pipeline(outcome_status="failed")
    module = _module(monkeypatch, pipeline)
    published: list[str] = []
    module.robot_pose.subscribe(lambda _msg: published.append("pose"))
    module.detections_3d.subscribe(lambda _msg: published.append("detections"))
    module.scene_graph.subscribe(lambda _msg: published.append("graph"))
    module.setup()
    module.start()
    try:
        module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))
        _wait_until(lambda: len(pipeline.processed) == 1)

        assert published == []
        assert module.health()["failure_counts"] == {"detector_error": 1}
    finally:
        module.stop()


def test_processed_negative_outcome_satisfies_readiness_without_timestamp_sentinel(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline()
    module = _module(monkeypatch, pipeline)
    module.setup()
    module.start()
    try:
        module.color_image._deliver(SimpleNamespace(ts=0.0, timestamp=0.0))
        _wait_until(lambda: len(pipeline.processed) == 1)

        assert module.startup_readiness() is None
        assert module.health()["last_valid_frame_ts"] == 0.0
    finally:
        module.stop()


def test_output_callback_can_stop_module_without_deadlock(monkeypatch: Any) -> None:
    pipeline = _Pipeline()
    module = _module(monkeypatch, pipeline)
    stop_returned = threading.Event()
    later_outputs: list[str] = []

    def stop_from_pose(_msg: Any) -> None:
        module.stop()
        stop_returned.set()

    module.robot_pose.subscribe(stop_from_pose)
    module.detections_3d.subscribe(lambda _msg: later_outputs.append("detections"))
    module.scene_graph.subscribe(lambda _msg: later_outputs.append("graph"))
    module.setup()
    module.start()
    module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))

    assert stop_returned.wait(1.0)
    _wait_until(lambda: pipeline.close_count == 1)
    assert later_outputs == []


def test_common_observation_callback_can_stop_without_recursive_deadlock(
    monkeypatch: Any,
) -> None:
    pipeline = _Pipeline()
    module = _module(monkeypatch, pipeline)
    stop_returned = threading.Event()

    def stop_from_observation(_msg: Any) -> None:
        module.stop()
        stop_returned.set()

    module.observation.subscribe(stop_from_observation)
    module.setup()
    module.start()
    module.color_image._deliver(SimpleNamespace(ts=1.0, timestamp=1.0))

    assert stop_returned.wait(1.0)
    _wait_until(lambda: pipeline.close_count == 1)


def test_setup_failure_releases_pipeline_and_propagates(monkeypatch: Any) -> None:
    import pytest

    pipeline = _Pipeline(fail_setup=True)
    module = _module(monkeypatch, pipeline)

    with pytest.raises(RuntimeError, match="backend load failed"):
        module.setup()

    assert pipeline.close_count == 1
