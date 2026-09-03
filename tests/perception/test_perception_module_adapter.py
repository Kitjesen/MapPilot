from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any

import numpy as np

import perception.module as perception_module
from perception.module import PerceptionModule
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.semantic import SceneGraph


class _Synchronizer:
    def __init__(self, **_kwargs: Any) -> None:
        self.color_frames = 0

    def push_color(self, frame: Any) -> tuple[Any, ...]:
        self.color_frames += 1
        return (frame,)

    def push_depth(self, _frame: Any) -> tuple[Any, ...]:
        return ()

    def push_camera_info(self, _info: Any) -> tuple[Any, ...]:
        return ()

    def push_odometry(self, _odom: Any) -> tuple[Any, ...]:
        return ()

    def push_map_odom(self, _transform: Any) -> tuple[Any, ...]:
        return ()

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


def test_module_keeps_the_blueprint_contract(monkeypatch: Any) -> None:
    module = _module(monkeypatch, _Pipeline())

    assert set(module.ports_in) == {
        "color_image",
        "depth_image",
        "camera_info",
        "odometry",
        "map_odom_tf",
    }
    assert set(module.ports_out) == {
        "scene_graph",
        "detections_3d",
        "robot_pose",
    }
    assert module.layer == 3


def test_worker_coalesces_pending_frames_and_preserves_publish_order(monkeypatch: Any) -> None:
    pipeline = _Pipeline(block=True)
    module = _module(monkeypatch, pipeline)
    published: list[tuple[str, float]] = []
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
            "pose",
            "detections",
            "graph",
            "pose",
            "detections",
            "graph",
        ]
        health = module.health()
        assert health["processed_frames"] == 2
        assert health["coalesced_frames"] == 1
        assert module.startup_readiness() is None
    finally:
        module.stop()

    module.stop()
    assert pipeline.close_count == 1


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


def test_setup_failure_releases_pipeline_and_propagates(monkeypatch: Any) -> None:
    import pytest

    pipeline = _Pipeline(fail_setup=True)
    module = _module(monkeypatch, pipeline)

    with pytest.raises(RuntimeError, match="backend load failed"):
        module.setup()

    assert pipeline.close_count == 1
