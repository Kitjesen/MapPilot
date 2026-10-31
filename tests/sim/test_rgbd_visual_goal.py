"""Rendered RGB-D evidence for visual navigation, without scene target coordinates."""

import mujoco
import numpy as np
import pytest
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.camera import MuJoCoCamera
from sim.scripts.mujoco.rgbd_goal_acceptance import camera_optical_transform, project_selected_goal

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3


def test_rgbd_camera_reuses_one_renderer_and_returns_independent_arrays(monkeypatch):
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <worldbody><camera name="front_camera"/></worldbody>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    renderers = []

    class FakeRenderer:
        def __init__(self, _model, height, width):
            self.height = height
            self.width = width
            self.depth = False
            self.close_count = 0
            self.rgb = np.full((height, width, 3), 7, dtype=np.uint8)
            self.depth_image = np.full((height, width), 2.5, dtype=np.float32)
            renderers.append(self)

        def update_scene(self, _data, *, camera):
            assert camera == "front_camera"

        def render(self):
            return self.depth_image if self.depth else self.rgb

        def enable_depth_rendering(self):
            self.depth = True

        def disable_depth_rendering(self):
            self.depth = False

        def close(self):
            self.close_count += 1

    monkeypatch.setattr(mujoco, "Renderer", FakeRenderer)
    camera = MuJoCoCamera(model, CameraConfig(width=64, height=48))
    try:
        frame = camera.render(data)

        assert len(renderers) == 1
        assert renderers[0].depth is False
        assert np.all(frame.rgb == 7)
        assert np.all(frame.depth == 2.5)

        renderers[0].rgb[:] = 9
        renderers[0].depth_image[:] = 4.0
        assert np.all(frame.rgb == 7)
        assert np.all(frame.depth == 2.5)
        assert np.all(camera.get_rgb(data) == 9)
        assert len(renderers) == 1

        data.time = 1.25
        snapshot = np.empty(
            mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION),
            dtype=np.float64,
        )
        mujoco.mj_getState(
            model,
            data,
            snapshot,
            mujoco.mjtState.mjSTATE_INTEGRATION,
        )
        data.time = 9.0
        snapshot_frame = camera.render_snapshot(snapshot)
        assert snapshot_frame.timestamp == pytest.approx(1.25)
        assert len(renderers) == 1
    finally:
        camera.close()

    camera.close()
    assert renderers[0].close_count == 1


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
        target, goal = project_selected_goal(
            frame,
            transform,
            pose,
            [24, 16, 40, 32],
            product_session_id="test-session",
            map_id="rendered-wall-map",
            content_epoch=1,
        )
        direction = np.array([np.cos(yaw), np.sin(yaw)])
        assert target.position[:2] == pytest.approx(np.array([5, 7]) + direction * 3.95, abs=0.002)
        position = goal.pose.position
        assert [position.x, position.y] == pytest.approx(np.array([5, 7]) + direction * 2.45, abs=0.002)
        assert position.z == pytest.approx(0.6)
        assert goal.frame_id == "map"
        frame.depth[:] = 0
        with pytest.raises(ValueError, match="no_valid_depth"):
            project_selected_goal(
                frame,
                transform,
                pose,
                [24, 16, 40, 32],
                product_session_id="test-session",
                map_id="rendered-wall-map",
                content_epoch=1,
            )
    finally:
        camera.close()
