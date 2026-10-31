"""Recorded obstacles and scene framing must not fabricate replay motion."""

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from tools.bench.body_motion_view import camera, restore_obstacles, route_description


def test_recorded_obstacle_pose_is_restored_without_interpolation():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="actor" mocap="true" pos="3 -2 .75"><geom type="box" size=".25 .25 .75"/></body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    sample = {"dynamic_obstacles": [{"name": "actor", "position": [3, -.713, .75]}]}
    checks = restore_obstacles(model, data, sample)
    mujoco.mj_forward(model, data)
    assert data.geom_xpos[0] == pytest.approx([3, -.713, .75])
    assert checks[0]["restored_position"] == sample["dynamic_obstacles"][0]["position"]
    assert data.time == 0


def test_supplied_route_is_not_described_as_automatic_global_search():
    assert route_description({"command": ["probe", "--use-route"]}) == "Native checked supplied pose route"
    assert route_description({"command": ["probe"]}) == "Native global search route"


def test_fixed_camera_fits_22m_route_and_compensates_oblique_projection():
    route = np.array([[0, 0, .45], [8, 0, .45], [8, 6, .45], [16, 6, .45]])
    top = camera(route, top=True)
    oblique = camera(route, top=False)
    assert top.lookat == pytest.approx([8, 3, .20])
    assert oblique.lookat == pytest.approx(top.lookat)
    assert oblique.distance > top.distance
    tall_route = np.array([[0, 0, .45], [0, 22, .45]])
    assert camera(tall_route, top=False).distance > camera(tall_route, top=True).distance * 1.8
