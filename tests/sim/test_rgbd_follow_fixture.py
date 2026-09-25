"""The controlled follow fixture must derive positions from rendered RGB-D."""

import math

import cv2
import numpy as np
import pytest
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine
from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco.rgbd_follow_worker import (
    ShirtPixelDetector,
    observe_frame,
)
from sim.scripts.mujoco.rgbd_goal_acceptance import camera_optical_transform

from perception.backends import RgbdObservationSource


def test_shirt_detector_uses_pixels_and_rejects_empty_image():
    detector = ShirtPixelDetector()
    image = np.zeros((120, 160, 3), np.uint8)
    assert detector.detect(image, "person") == []
    image[20:100, 50:80] = (190, 5, 230)
    found = detector.detect(image, "person")
    assert len(found) == 1
    assert found[0].bbox.tolist() == [50, 20, 80, 100]
    assert np.count_nonzero(found[0].mask) == 2400


def test_factory_camera_projects_visible_person_and_rejects_invalid_depth(tmp_path):
    pytest.importorskip("mujoco")
    manifest = native._load_manifest(native.ROOT / "config/acceptance/mujoco/rgbd_follow.json")
    world, _ = native._prepare_dynamic_obstacle(manifest, native.ROOT / manifest["world"], tmp_path)
    robot = RobotConfig.default_thunder_v4()
    robot.init_position = [41, 4, 0.6]
    engine = MuJoCoEngine(
        robot_config=robot, camera_configs=[CameraConfig(width=320, height=240)], drive_mode="kinematic"
    )
    source = RgbdObservationSource(ShirtPixelDetector(), min_depth=0.3, max_depth=8, u16_depth_scale=0.001)
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
        assert ShirtPixelDetector().detect(cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR), "person") == []
    finally:
        source.close()
        engine.close()
