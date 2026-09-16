from types import SimpleNamespace

import numpy as np
import pytest

from drivers.sim.mujoco.goal_picker import ClickGesture, GoalPicker


def test_click_requires_focus_press_release_and_excludes_drag_round_trip():
    gesture = ClickGesture()
    assert not gesture.update(True, True, 10, 10)
    assert not gesture.update(True, False, 10, 10)
    assert not gesture.update(True, True, 10, 10)
    assert gesture.update(True, False, 11, 11)
    assert not gesture.update(True, False, 11, 11)
    assert not gesture.update(True, True, 10, 10)
    assert not gesture.update(True, True, 20, 20)
    assert not gesture.update(True, False, 10, 10)
    assert not gesture.update(True, True, 10, 10)
    assert not gesture.update(False, False, 10, 10)
    assert not gesture.update(True, False, 10, 10)


@pytest.fixture
def picking_scene():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string('''
        <mujoco><worldbody>
          <geom type="plane" size="20 20 .1"/>
          <geom type="box" pos="3 0 .5" size=".6 .6 .5"/>
          <body name="robot" pos="0 0 .5"><freejoint/><geom size=".3"/>
            <body name="leg" pos=".5 0 0"><geom size=".15"/></body>
          </body>
        </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 6
    camera.elevation = -90
    viewer = SimpleNamespace(
        opt=mujoco.MjvOption(), cam=camera,
        viewport=mujoco.MjrRect(0, 0, 800, 600),
        user_scn=mujoco.MjvScene(model, maxgeom=10),
    )
    return model, data, viewer, GoalPicker(model, "robot")


@pytest.mark.parametrize("target", [(2, 3, 0), (3, 0, 1)])
def test_native_raycast_preserves_ground_and_platform_height(picking_scene, target):
    model, data, viewer, picker = picking_scene
    viewer.cam.lookat[:] = target
    point = picker.pick(model, data, viewer, .5, .5)
    np.testing.assert_allclose(point, target, atol=1e-6)
    picker.marker = point
    picker.draw(viewer)
    assert viewer.user_scn.ngeom == 1
    np.testing.assert_allclose(viewer.user_scn.geoms[0].pos, point + np.array([0, 0, .08]))


@pytest.mark.parametrize("x", [0, .5])
def test_native_raycast_rejects_robot_and_child_body(picking_scene, x):
    model, data, viewer, picker = picking_scene
    viewer.cam.lookat[:] = [x, 0, .5]
    with pytest.raises(ValueError, match="Robot selected"):
        picker.pick(model, data, viewer, .5, .5)


def test_native_raycast_rejects_sky(picking_scene):
    model, data, viewer, picker = picking_scene
    viewer.cam.lookat[:] = [10, 10, 10]
    viewer.cam.elevation = 30
    with pytest.raises(ValueError, match="No surface"):
        picker.pick(model, data, viewer, .5, .5)



@pytest.fixture
def click_window(picking_scene):
    from drivers.sim.mujoco.click_viewer import ClickViewer

    model, data, _, picker = picking_scene
    window = ClickViewer(model, data, picker, visible=False)
    window.cam.lookat[:] = [3, 0, 1]
    window.cam.distance = 6
    window.cam.elevation = -90
    try:
        window.sync()
        yield window
    finally:
        window.dispose()


def queue_center_click(monkeypatch, window):
    import glfw

    width, height = window._displayed_size
    monkeypatch.setattr(glfw, "get_cursor_pos", lambda _: (width / 2, height / 2))
    monkeypatch.setattr(glfw, "get_window_attrib", lambda *_: True)

    def events():
        window._button(window._window, glfw.MOUSE_BUTTON_LEFT, glfw.PRESS, 0)
        window._button(window._window, glfw.MOUSE_BUTTON_LEFT, glfw.RELEASE, 0)

    monkeypatch.setattr(glfw, "poll_events", events)


def test_click_uses_displayed_camera_before_pending_camera_change(click_window, monkeypatch):
    window = click_window
    window.cam.lookat[:] = [8, 5, 0]
    queue_center_click(monkeypatch, window)
    window.sync()
    np.testing.assert_allclose(window.poll_click(), [3, 0, 1], atol=1e-6)
    window.sync()
    np.testing.assert_allclose(window.poll_click(), [8, 5, 0], atol=1e-6)


def test_click_uses_displayed_body_pose_before_pending_physics_change(click_window, monkeypatch):
    import mujoco

    window = click_window
    # Move the robot into the ray after the platform frame was displayed.
    window._data.qpos[:3] = [3, 0, 1.5]
    mujoco.mj_forward(window._model, window._data)
    queue_center_click(monkeypatch, window)
    window.sync()
    np.testing.assert_allclose(window.poll_click(), [3, 0, 1], atol=1e-6)
    window.sync()
    with pytest.raises(ValueError, match="Robot selected"):
        window.poll_click()


def test_resizing_between_frame_and_click_requests_another_click(click_window, monkeypatch):
    import glfw

    queue_center_click(monkeypatch, click_window)
    monkeypatch.setattr(glfw, "get_window_size", lambda _: (600, 400))
    click_window.sync()
    with pytest.raises(ValueError, match="resized view"):
        click_window.poll_click()


def test_window_renders_navigation_marker_and_health_text(click_window):
    import mujoco

    click_window._picker.marker = np.array([3, 0, 1])
    click_window._picker.draw(click_window)
    click_window.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,
                           mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           "Left click: goal", "Waiting for control loop health"))
    click_window.sync()
    assert click_window._scene.ngeom == click_window._model.ngeom + 1


@pytest.mark.parametrize("localized", [False, True])
def test_native_click_window_renders_and_closes_on_live_viewer_thread(picking_scene, monkeypatch, localized):
    import os
    import threading

    import mujoco

    from drivers.sim.mujoco import click_viewer
    from drivers.sim.mujoco.runtime import LiveViewer
    from lingtu.sim.viewer_goal import world_point_to_map

    if os.name != "nt":
        pytest.skip("Windows click-navigation window")
    model, data, _, _ = picking_scene
    state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
    rendered = threading.Event()
    transform_attempted = threading.Event()
    real_window = click_viewer.ClickViewer

    class HiddenWindow(real_window):
        def __init__(self, *args):
            super().__init__(*args, visible=False)

        def sync(self):
            super().sync()
            if self._texts is not None and transform_attempted.is_set():
                rendered.set()

    monkeypatch.setattr(click_viewer, "ClickViewer", HiddenWindow)

    def transform():
        transform_attempted.set()
        if not localized:
            world_point_to_map(np.zeros(3), np.zeros(3), np.eye(3), {
                "map_odom_tf": None, "odometry": {"frame_id": "odom"},
            })
        return np.eye(3), np.zeros(3)

    goal = SimpleNamespace(
        poll=lambda: "Ready", close=lambda: None,
        world_to_map_transform=transform,
    )
    viewer = LiveViewer(model, state, [0, 0, .5], lambda: {}, goal_input=goal, body_name="robot")
    try:
        viewer.submit(state, [0, 0, .5], [])
        assert rendered.wait(3)
        assert viewer.is_running()
    finally:
        viewer.close()
    assert not viewer._thread.is_alive()
    assert viewer._failure is None
