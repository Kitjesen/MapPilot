"""Product-shaped camera-to-Follow integration without claiming field motion."""

from __future__ import annotations

import asyncio
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("fastapi")

from decision.modules.visual_servo import VisualServoModule
from drivers.real.camera.dds_module import DdsCameraModule
from drivers.real.camera.shm import ShmFrameWriter, StreamKind
from gateway.gateway_module import GatewayModule
from gateway.schemas import FollowTaskCancelRequest, VisualServoRequest
from perception.backends import RgbdObservationSource
from perception.module import PerceptionModule
from perception.pipeline import PerceptionSettings
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import (
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationState,
    Odometry,
)

WIDTH = 640
HEIGHT = 480
SESSION_ID = "camera-follow-integration"


def _endpoint(gateway: GatewayModule, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _wait_for(predicate, *, message: str) -> None:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(message)


class _Detector:
    """Controlled detector; camera transport, projection and Follow remain real."""

    def detect(self, _image, _prompt):
        feature = np.zeros(384, dtype=np.float32)
        feature[0] = 1.0
        return [
            SimpleNamespace(
                bbox=np.array([280, 120, 360, 420]),
                label="person",
                score=0.95,
                track_id=7,
                dino_feature=feature,
            )
        ]


class _DinoBoundary:
    """Only marks the appearance encoder boundary as present.

    The detector supplies deterministic features, so this method should not run.
    """

    def encode(self, _crop):
        raise AssertionError("the controlled detector feature should be reused")


class _NativeMotionBoundary:
    """Record native requests and return only explicit cancellation feedback."""

    def __init__(self, follow: VisualServoModule) -> None:
        self.follow = follow
        self.goals = []
        self.cancels = []
        self._sequence = 0
        follow.goal_pose.subscribe(self.goals.append)
        follow.goal_cancel.subscribe(self._cancel)

    def _cancel(self, request) -> None:
        self.cancels.append(request)
        for task_id in request.task_ids:
            goal = next(item for item in reversed(self.goals) if item.task_id == task_id)
            self._sequence += 1
            self.follow.navigation_goal_status._deliver(
                NavigationGoalStatus(
                    boot_id="controlled-native-motion",
                    sequence=self._sequence,
                    task_id=task_id,
                    request_id=goal.request_id,
                    state=int(NavigationGoalState.CANCELLED),
                    reason=request.reason,
                )
            )


class _Writers:
    def __init__(self, root: Path, generation: int) -> None:
        self.paths = {
            "color": root / "camera_color.shm",
            "depth": root / "camera_depth.shm",
            "info": root / "camera_info.shm",
        }
        self.color = ShmFrameWriter(
            self.paths["color"],
            stream_kind=StreamKind.COLOR,
            slot_capacity=WIDTH * HEIGHT * 3,
            source_generation=generation,
        )
        self.depth = ShmFrameWriter(
            self.paths["depth"],
            stream_kind=StreamKind.DEPTH,
            slot_capacity=WIDTH * HEIGHT * 2,
            source_generation=generation,
        )
        self.info = ShmFrameWriter(
            self.paths["info"],
            stream_kind=StreamKind.INFO,
            slot_capacity=1,
            source_generation=generation,
        )

    def publish(self, stamp: float) -> None:
        timestamp_ns = int(stamp * 1_000_000_000)
        self.color.publish(
            timestamp_ns=timestamp_ns,
            width=WIDTH,
            height=HEIGHT,
            stride=WIDTH * 3,
            encoding="bgr8",
            frame_id="camera_optical",
            payload=np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8).tobytes(),
        )
        self.depth.publish(
            timestamp_ns=timestamp_ns,
            width=WIDTH,
            height=HEIGHT,
            stride=WIDTH * 2,
            encoding="16UC1",
            frame_id="camera_optical",
            payload=np.full((HEIGHT, WIDTH), 3000, dtype=np.uint16).tobytes(),
        )
        self.info.publish(
            timestamp_ns=timestamp_ns,
            width=WIDTH,
            height=HEIGHT,
            stride=0,
            encoding="bgr8",
            frame_id="camera_optical",
            payload=b"",
            fx=400.0,
            fy=400.0,
            cx=WIDTH / 2,
            cy=HEIGHT / 2,
            depth_scale=0.001,
        )

    def close(self) -> None:
        self.color.close()
        self.depth.close()
        self.info.close()


class _Chain:
    def __init__(self, root: Path) -> None:
        self.writers = _Writers(root, generation=100)
        self.camera = DdsCameraModule(
            color_shm_path=str(self.writers.paths["color"]),
            depth_shm_path=str(self.writers.paths["depth"]),
            info_shm_path=str(self.writers.paths["info"]),
            stale_timeout_s=5.0,
            poll_interval_s=1.0,
        )
        source = RgbdObservationSource(
            _Detector(), min_depth=0.3, max_depth=6.0, u16_depth_scale=0.001
        )
        camera_to_body = np.eye(4)
        camera_to_body[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        self.perception = PerceptionModule(
            settings=PerceptionSettings(laplacian_threshold=0),
            detector=source,
            camera_to_body=camera_to_body,
            skip_frames=1,
            max_rgbd_skew_s=0.05,
            max_odom_age_s=0.2,
            max_map_odom_age_s=0.5,
            product_session_id=SESSION_ID,
        )
        self.follow = VisualServoModule(
            product_session_id=SESSION_ID,
            lost_timeout=10.0,
            goal_rate_hz=1000.0,
            goal_deadband_m=0.0,
        )
        self.follow._person_tracker.set_dino_encoder(_DinoBoundary())
        self.motion = _NativeMotionBoundary(self.follow)
        self.observations = []

        self.camera.color_image.subscribe(self.perception.color_image._deliver)
        self.camera.depth_image.subscribe(self.perception.depth_image._deliver)
        self.camera.camera_info.subscribe(self.perception.camera_info._deliver)
        self.perception.observation.subscribe(self.follow.observation._deliver)
        self.perception.observation.subscribe(self.observations.append)

        self.gateway = GatewayModule()
        self.gateway.setup()
        self.gateway._all_modules = {
            "VisualServoModule": self.follow,
            "PerceptionModule": self.perception,
        }
        self.start_api = _endpoint(self.gateway, "/api/v1/visual_servo")
        self.query_api = _endpoint(self.gateway, "/api/v1/follow/{follow_task_id}")
        self.cancel_api = _endpoint(
            self.gateway, "/api/v1/follow/{follow_task_id}/cancel"
        )

        self.follow.setup()
        self.perception.setup()
        self.camera.setup()
        self.perception.start()
        self.follow.start()
        self._map_sequence = 1
        self.map("map-a")

    def map(self, map_id: str) -> None:
        self.perception.navigation_state._deliver(
            NavigationState(
                boot_id="controlled-navigation-state",
                sequence=self._map_sequence,
                map_id=map_id,
                map_content_epoch=1,
            )
        )
        self._map_sequence += 1

    def feed(self, *, stamp: float | None = None, x: float = 0.0) -> float:
        source_ts = time.time() if stamp is None else stamp
        self.perception.odometry._deliver(
            Odometry(
                pose=Pose(position=Vector3(x, 0.0, 0.0)),
                ts=source_ts,
                frame_id="map",
            )
        )
        self.writers.publish(source_ts)
        assert self.camera.poll_once() == 3
        return source_ts

    def restart_camera_source(self, generation: int) -> None:
        self.writers.close()
        self.writers = _Writers(self.writers.paths["color"].parent, generation)

    def positive(self, stamp: float):
        return next(
            (
                item
                for item in self.observations
                if abs(item.source_ts - stamp) < 1e-6 and item.status == "positive"
            ),
            None,
        )

    def wait_positive(self, stamp: float) -> None:
        _wait_for(
            lambda: self.positive(stamp) is not None,
            message="SHM RGB-D frame did not become a positive semantic observation",
        )

    def start_follow(self, request_id: str):
        return asyncio.run(
            self.start_api(
                VisualServoRequest(
                    mode="follow",
                    target_id="track_7",
                    request_id=request_id,
                    client_id="integration-test",
                )
            )
        )

    def cancel_follow(self, receipt: dict, request_id: str):
        return asyncio.run(
            self.cancel_api(
                receipt["follow_task_id"],
                FollowTaskCancelRequest(
                    product_session_id=receipt["product_session_id"],
                    reason="operator_cancel",
                    request_id=request_id,
                    client_id="integration-test",
                ),
            )
        )

    def close(self) -> None:
        self.writers.close()
        self.camera.stop()
        self.perception.stop()
        self.follow.stop()
        self.gateway.stop()


@pytest.fixture
def chain(tmp_path):
    value = _Chain(tmp_path)
    try:
        yield value
    finally:
        value.close()


def test_shm_camera_follow_api_lifecycle_keeps_old_cancel_scoped(chain: _Chain):
    stamp = chain.feed()
    chain.wait_positive(stamp)

    first = chain.start_follow("start-follow-1")
    assert first["accepted"] is True
    assert first["product_session_id"] == SESSION_ID
    assert chain.motion.goals
    first_view = asyncio.run(
        chain.query_api(first["follow_task_id"], first["product_session_id"])
    )
    assert first_view["found"] is True
    assert first_view["state"] == "running"

    first_cancel = chain.cancel_follow(first, "cancel-follow-1")
    assert first_cancel["accepted"] is True
    assert first_cancel["state"] == "cancelled"
    assert chain.motion.cancels[-1].task_ids == (chain.motion.goals[0].task_id,)

    second = chain.start_follow("start-follow-2")
    assert second["accepted"] is True
    assert second["follow_task_id"] != first["follow_task_id"]
    cancel_count = len(chain.motion.cancels)
    old_cancel = chain.cancel_follow(first, "cancel-old-follow-again")
    current = asyncio.run(
        chain.query_api(second["follow_task_id"], second["product_session_id"])
    )

    assert old_cancel["reason"] == "already_terminal"
    assert len(chain.motion.cancels) == cancel_count
    assert current["found"] is True
    assert current["state"] == "running"
    assert current["terminal"] is False
    assert chain.follow.get_servo_status()["follow_task_id"] == second["follow_task_id"]


@pytest.mark.parametrize("context_change", ["camera_restart", "map_switch"])
def test_context_change_requires_three_distinct_views_before_follow_can_recover(
    chain: _Chain, context_change: str
):
    stamp = chain.feed()
    chain.wait_positive(stamp)
    receipt = chain.start_follow(f"start-{context_change}")
    assert receipt["accepted"] is True
    assert chain.motion.goals
    old_observation = chain.positive(stamp)
    goal_count = len(chain.motion.goals)
    cancel_count = len(chain.motion.cancels)

    stale_stamp = stamp - 0.01
    chain.feed(stamp=stale_stamp, x=50.0)
    time.sleep(0.02)
    assert len(chain.motion.goals) == goal_count
    assert len(chain.motion.cancels) == cancel_count

    if context_change == "camera_restart":
        chain.restart_camera_source(generation=200)
    else:
        chain.map("map-b")

    first_new_stamp = chain.feed()
    chain.wait_positive(first_new_stamp)
    _wait_for(
        lambda: len(chain.motion.cancels) > cancel_count,
        message="context change did not cancel the owned native goal",
    )
    assert len(chain.motion.goals) == goal_count

    latest_sequence = chain.follow._latest_observation_sequence
    chain.follow.observation._deliver(old_observation)
    assert chain.follow._latest_observation_sequence == latest_sequence
    assert len(chain.motion.goals) == goal_count

    second_new_stamp = chain.feed()
    chain.wait_positive(second_new_stamp)
    assert len(chain.motion.goals) == goal_count
    third_new_stamp = chain.feed()
    chain.wait_positive(third_new_stamp)
    assert len(chain.motion.goals) == goal_count

    candidate = chain.follow._person_tracker.identity_candidate()
    assert candidate is not None, {
        "person": chain.follow._person_tracker.status(),
        "follow": chain.follow.get_servo_status(),
        "observations": [
            (item.sequence, item.source_ts, item.status, item.reason)
            for item in chain.observations[-8:]
        ],
    }
    assert all(
        abs(actual - expected) < 1e-6
        for actual, expected in zip(
            candidate["crop_timestamps"],
            [first_new_stamp, second_new_stamp, third_new_stamp],
            strict=True,
        )
    )
    # Identity classification is the controlled model boundary in this test;
    # the three-view evidence collection and subsequent Follow state are real.
    assert chain.follow._person_tracker.resolve_identity(
        candidate["object"]["id"], candidate["object"]["ts"], "MATCH"
    )

    recovery_stamp = chain.feed()
    chain.wait_positive(recovery_stamp)
    _wait_for(
        lambda: len(chain.motion.goals) > goal_count,
        message="verified three-view identity did not resume Follow goal output",
    )
    task = asyncio.run(
        chain.query_api(receipt["follow_task_id"], receipt["product_session_id"])
    )
    assert task["found"] is True
    assert task["state"] == "running"
    assert task["terminal"] is False
