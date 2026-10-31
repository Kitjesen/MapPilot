"""RGB-D synchronization and projection through the actual Follow input port."""

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from decision.modules.visual_servo import VisualServoModule
from perception.backends import RgbdObservationSource
from perception.module import PerceptionModule
from perception.pipeline import PerceptionSettings
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import NavigationState, Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


class Detector:
    """Controlled detections; synchronization, projection, tracking and Follow are real."""

    def __init__(self):
        self.block = False
        self.entered = threading.Event()
        self.release = threading.Event()

    def detect(self, _image, _prompt):
        if self.block:
            self.entered.set()
            assert self.release.wait(3.0)
        return [SimpleNamespace(
            bbox=np.array([8, 8, 56, 56]), label="person", score=0.95,
            track_id=7, dino_feature=np.eye(1, 384, dtype=np.float32)[0],
        )]


def wait_for(predicate):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("pipeline did not deliver the expected observation")


class Chain:
    def __init__(self):
        self.detector = Detector()
        source = RgbdObservationSource(
            self.detector, min_depth=0.3, max_depth=6.0, u16_depth_scale=0.001,
        )
        transform = np.eye(4)
        transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        self.perception = PerceptionModule(
            settings=PerceptionSettings(laplacian_threshold=0), detector=source,
            camera_to_body=transform, skip_frames=1, max_rgbd_skew_s=0.05,
            max_odom_age_s=0.1, max_map_odom_age_s=0.5, product_session_id="run-a",
        )
        self.follow = VisualServoModule(product_session_id="run-a", lost_timeout=10.0)
        self.observations = []
        self.goals = []
        self.cancels = []
        self.perception.observation.subscribe(self.follow.observation._deliver)
        self.perception.observation.subscribe(self.observations.append)
        self.follow.goal_pose.subscribe(self.goals.append)
        self.follow.goal_cancel.subscribe(self.cancels.append)
        self.follow.setup()
        self.perception.setup()
        self.perception.start()
        self.map("map-a", 1)

    def map(self, name, sequence):
        self.perception.navigation_state._deliver(NavigationState(
            map_id=name, map_content_epoch=1, boot_id="nav", sequence=sequence,
        ))

    def feed(self, generation=100, sequence=1, timestamp=None, x=0.0):
        stamp = time.time() if timestamp is None else timestamp
        info = CameraIntrinsics(
            fx=100, fy=100, cx=32, cy=32, width=64, height=64,
            depth_scale=0.001, ts=stamp, frame_id="camera_optical",
            source_generation=generation, source_sequence=sequence + 20,
        )
        self.perception.camera_info._deliver(info)
        self.perception.odometry._deliver(Odometry(
            pose=Pose(position=Vector3(x, 0, 0)), ts=stamp, frame_id="map",
        ))
        self.perception.depth_image._deliver(Image(
            data=np.full((64, 64), 3000, dtype=np.uint16), format=ImageFormat.DEPTH_U16,
            ts=stamp, frame_id="camera_optical", source_generation=generation,
            source_sequence=sequence + 10,
        ))
        self.perception.color_image._deliver(Image(
            data=np.ones((64, 64, 3), dtype=np.uint8), format=ImageFormat.BGR,
            ts=stamp, frame_id="camera_optical", source_generation=generation,
            source_sequence=sequence,
        ))
        return stamp

    def positive(self, stamp):
        return next((item for item in self.observations
                     if item.source_ts == stamp and item.status == "positive"), None)

    def lock(self):
        stamp = self.feed()
        wait_for(lambda: self.positive(stamp))
        receipt = self.follow.start_follow(target_id="track_7")
        assert receipt["accepted"]
        stamp = self.feed(sequence=2)
        wait_for(lambda: self.positive(stamp) and self.goals)
        return receipt, stamp

    def close(self):
        self.detector.release.set()
        self.perception.stop()
        self.follow.stop()


@pytest.fixture
def chain():
    value = Chain()
    try:
        yield value
    finally:
        value.close()


def test_projected_target_uses_one_frame_and_old_frames_do_not_cancel_follow(chain):
    _, stamp = chain.lock()
    observation = chain.positive(stamp)
    assert observation.product_session_id == "run-a"
    assert (observation.map_id, observation.content_epoch) == ("map-a", 1)
    assert observation.image.source_sequence != observation.depth.source_sequence
    assert observation.image.ts == observation.depth.ts == observation.robot_pose.ts == stamp
    assert observation.detections[0].position.x == pytest.approx(3.0)
    current = chain.follow._latest_observation_sequence
    before = len(chain.cancels), len(chain.goals)
    chain.feed(sequence=1, timestamp=stamp - 1.0, x=100)
    chain.follow.observation._deliver(observation)
    assert chain.follow._latest_observation_sequence == current
    assert (len(chain.cancels), len(chain.goals)) == before


@pytest.mark.parametrize("change", ["camera", "map"])
def test_context_change_cancels_before_blocked_old_inference_returns(chain, change):
    receipt, stamp = chain.lock()
    old = chain.positive(stamp)
    identity = chain.follow._person_tracker.status()["identity_id"]
    chain.detector.block = True
    blocked_stamp = chain.feed(sequence=3)
    assert chain.detector.entered.wait(2.0)
    if change == "camera":
        new_stamp = chain.feed(generation=200, sequence=1)
    else:
        chain.map("map-b", 2)
        new_stamp = chain.feed(sequence=4, x=10)
    assert chain.cancels, "context invalidation must not wait for inference"
    assert not chain.follow._motion_observation_ready
    assert chain.follow._person_tracker.status()["identity_id"] == identity
    count = len(chain.goals)
    chain.detector.block = False
    chain.detector.release.set()
    wait_for(lambda: chain.positive(new_stamp))
    assert chain.positive(blocked_stamp) is None
    assert len(chain.goals) == count, "one new frame cannot bypass identity re-verification"
    latest = chain.follow._latest_observation_sequence
    chain.follow.observation._deliver(old)
    assert chain.follow._latest_observation_sequence == latest
    assert len(chain.goals) == count
    task = chain.follow.get_follow_task(receipt["follow_task_id"], "run-a")
    assert task["state"] == "running"
