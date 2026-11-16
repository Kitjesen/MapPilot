"""The controlled follow fixture must derive positions from rendered RGB-D."""

import math
import time
from io import StringIO
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine
from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco.rgbd_follow_worker import FollowController, observe_frame
from sim.scripts.mujoco.rgbd_goal_acceptance import camera_optical_transform

from decision.modules.visual_servo import VisualServoModule
from perception.detection.detector_base import Detection2D
from perception.backends import RgbdObservationSource
from runtime.msgs.sensor import Image, ImageFormat
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import VisualGoalCancel, VisualGoalRequest


class SyntheticPersonDetector:
    """Deterministic RGB fixture owned by projection tests."""

    def detect(self, bgr, prompt):
        b, g, r = cv2.split(bgr.astype(np.int16))
        mask = ((r > g * 1.6 + 20) & (b > g * 1.4 + 20) & (r > 65)).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count < 2:
            return []
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, w, h, area = stats[index]
        if area < 18:
            return []
        return [
            Detection2D(
                bbox=np.array([x, y, x + w, y + h], dtype=np.float32),
                score=1.0,
                label="person",
                mask=labels == index,
            )
        ]


def test_shirt_detector_uses_pixels_and_rejects_empty_image():
    detector = SyntheticPersonDetector()
    image = np.zeros((120, 160, 3), np.uint8)
    assert detector.detect(image, "person") == []
    image[20:100, 50:80] = (190, 5, 230)
    found = detector.detect(image, "person")
    assert len(found) == 1
    assert found[0].bbox.tolist() == [50, 20, 80, 100]
    assert np.count_nonzero(found[0].mask) == 2400


def test_follow_controller_publishes_one_qualified_semantic_observation(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-follow")
    monkeypatch.setenv("LINGTU_MAP_ID", "building")
    monkeypatch.setenv("LINGTU_MAP_CONTENT_EPOCH", "7")
    received = []
    original_observer = VisualServoModule._on_observation

    def capture(servo, observation):
        received.append(observation)
        original_observer(servo, observation)

    monkeypatch.setattr(VisualServoModule, "_on_observation", capture)
    controller = FollowController("unused", 0, StringIO())
    controller.dispatch = lambda: None
    source_ts = time.time()
    transform = np.eye(4, dtype=np.float64)
    try:
        controller.update(
            {"t": source_ts, "x": 1.0, "y": 2.0, "z": 0.5, "yaw": 0.25},
            Image(
                data=np.zeros((16, 16, 3), dtype=np.uint8),
                format=ImageFormat.RGB,
                ts=source_ts,
            ),
            [
                SimpleNamespace(
                    score=0.9,
                    position=np.array([3.0, 2.0, 1.0]),
                    bbox_2d=np.array([0.0, 0.0, 16.0, 16.0]),
                )
            ],
            transform,
        )

        observation = received[-1]
        assert observation.product_session_id == "session-follow"
        assert observation.map_id == "building"
        assert observation.content_epoch == 7
        assert observation.sequence == 1
        assert observation.source_ts == source_ts
        assert [
            observation.robot_pose.x,
            observation.robot_pose.y,
            observation.robot_pose.z,
        ] == pytest.approx([1.0, 2.0, 0.5])
        assert np.array_equal(observation.map_from_camera, transform)
        assert observation.detections[0].id == "person_0"
    finally:
        controller.servo.stop()


def test_follow_controller_refuses_motion_without_runtime_map_identity(monkeypatch):
    for name in (
        "LINGTU_PRODUCT_SESSION_ID",
        "LINGTU_MAP_ID",
        "LINGTU_MAP_CONTENT_EPOCH",
    ):
        monkeypatch.delenv(name, raising=False)

    with pytest.raises(RuntimeError, match="requires_product_map_identity"):
        FollowController("unused", 0, StringIO())


def test_follow_controller_dispatches_visual_goal_request_pose(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-follow")
    monkeypatch.setenv("LINGTU_MAP_ID", "building")
    monkeypatch.setenv("LINGTU_MAP_CONTENT_EPOCH", "7")
    controller = FollowController("unused", 0, StringIO())
    commands = []
    controller.command = lambda arguments: commands.append(arguments) or {"ok": True, "stderr": ""}
    goal = VisualGoalRequest(
        goal=PoseStamped(
            pose=Pose(
                position=Vector3(3.0, 2.0, 0.5),
                orientation=Quaternion.from_yaw(0.25),
            ),
            frame_id="map",
        ),
        task_id="follow-task",
        request_id="follow-request",
    )
    try:
        controller.queue_goal(goal)
        controller.dispatch()
        assert commands == [[
            "goal", "3.0", "2.0", "0.5", "0.25", "--task-id", "follow-task",
        ]]
        assert controller.accepted == 1
        assert controller.task == "follow-task"
        controller.queue_cancel(VisualGoalCancel(task_ids=("follow-task",), reason="target_lost"))
        controller.dispatch()
        assert commands[-1] == ["cancel", "follow-task", "target_lost"]
    finally:
        controller.servo.stop()


def test_factory_camera_projects_visible_person_and_rejects_invalid_depth(tmp_path):
    pytest.importorskip("mujoco")
    manifest = native._load_manifest(native.ROOT / "config/acceptance/mujoco/rgbd_follow.json")
    world, _ = native._prepare_dynamic_obstacle(manifest, native.ROOT / manifest["world"], tmp_path)
    robot = RobotConfig.default_thunder_v4()
    robot.init_position = [41, 4, 0.6]
    engine = MuJoCoEngine(
        robot_config=robot, camera_configs=[CameraConfig(width=320, height=240)], drive_mode="kinematic"
    )
    source = RgbdObservationSource(SyntheticPersonDetector(), min_depth=0.3, max_depth=8, u16_depth_scale=0.001)
    source.load()
    try:
        engine.load(str(world))
        engine.reset()
        engine.set_robot_pose(np.array([41, 4, 0.6]), np.array([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]))
        frame = engine.get_camera_data()
        transform = camera_optical_transform(engine._data, engine._cameras["front_camera"].cam_id)
        _, detections = observe_frame(source, frame, transform, 1)
        assert len(detections) == 1
        assert math.dist(detections[0].position[:2], [41, 6.5]) < 0.25
        assert len(detections[0].points) > 18
        frame.depth[:] = 0
        assert len(observe_frame(source, frame, transform, 2)[1]) == 0
        frame.rgb[:] = 0
        assert SyntheticPersonDetector().detect(cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR), "person") == []
    finally:
        source.close()
        engine.close()
