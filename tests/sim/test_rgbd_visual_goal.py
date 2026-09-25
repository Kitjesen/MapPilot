"""Rendered RGB-D evidence for visual navigation, without scene target coordinates."""

import mujoco
import numpy as np
import pytest
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.camera import MuJoCoCamera
from sim.scripts.mujoco.rgbd_goal_acceptance import camera_optical_transform, project_selected_goal

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3


@pytest.mark.parametrize("distance", [0.4, 1.2, 2.5])
def test_rendered_near_wall_depth_stays_in_metres(distance):
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco>
      <worldbody>
        <camera name="front_camera" pos="0 0 0"/>
        <geom type="box" pos="0 0 {-distance - 0.05}"
              size="10 10 0.05" rgba="0.2 0.7 0.3 1"/>
      </worldbody>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = MuJoCoCamera(model, CameraConfig(width=64, height=48))
    try:
        frame = camera.render(data)
        assert float(np.median(frame.depth)) == pytest.approx(distance, abs=0.002)
    finally:
        camera.close()


@pytest.mark.parametrize("yaw", [0.0, np.pi / 2])
def test_rendered_roi_becomes_map_goal_with_camera_rotation(yaw):
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco>
      <compiler angle="radian"/>
      <worldbody>
        <body pos="5 7 0.6" euler="0 0 {yaw}">
          <camera name="front_camera" xyaxes="0 -1 0 0 0 1"/>
          <geom type="box" pos="4 0 0" size="0.05 10 10"/>
        </body>
      </worldbody>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = MuJoCoCamera(model, CameraConfig(width=64, height=48))
    pose = PoseStamped(pose=Pose(position=Vector3(5, 7, 0.6), orientation=Quaternion.from_yaw(yaw)), frame_id="map")
    try:
        frame = camera.render(data)
        transform = camera_optical_transform(data, camera.cam_id)
        target, goal = project_selected_goal(frame, transform, pose, [24, 16, 40, 32])
        direction = np.array([np.cos(yaw), np.sin(yaw)])
        assert target.position[:2] == pytest.approx(np.array([5, 7]) + direction * 3.95, abs=0.002)
        assert [goal.x, goal.y] == pytest.approx(np.array([5, 7]) + direction * 2.45, abs=0.002)
        assert goal.z == pytest.approx(0.6)
        frame.depth[:] = 0
        with pytest.raises(ValueError, match="no_valid_depth"):
            project_selected_goal(frame, transform, pose, [24, 16, 40, 32])
    finally:
        camera.close()
