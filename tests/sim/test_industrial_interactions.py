"""Real MuJoCo checks for draft prop mechanics, not UE or product qualification."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
from sim.tools.assets.interaction_lab import build_scene


@pytest.fixture
def lab():
    model = mujoco.MjModel.from_xml_string(build_scene())
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=500)
    return model, data


def joint(model, data, name):
    return float(data.qpos[model.joint(name).qposadr[0]])


def test_scene_has_dynamic_chassis_and_anchored_equipment(lab):
    model, data = lab
    for name in ["truck/free", "bin/free", "forklift/free"]:
        assert model.joint(name).type[0] == mujoco.mjtJoint.mjJNT_FREE
    for name in ["cabinet/frame", "hvac/base", "hydrant/base", "valve/base", "barrier/base", "service_door/frame"]:
        assert model.body(name).jntnum[0] == 0
    assert np.isfinite(data.qpos).all()
    assert data.ncon > 0


def test_bin_moves_under_force_and_wheels_roll(lab):
    model, data = lab
    bid = model.body("bin/chassis").id
    before = data.xpos[bid].copy()
    data.xfrc_applied[bid, 0] = 140
    mujoco.mj_step(model, data, nstep=500)
    data.xfrc_applied[:] = 0
    assert data.xpos[bid, 0] - before[0] > 0.08
    assert abs(joint(model, data, "bin/roll_0")) > 0.05
    assert data.ncon > 0


def test_truck_drives_from_wheel_actuation_not_pose_writes(lab):
    model, data = lab
    bid = model.body("truck/chassis").id
    before = data.xpos[bid].copy()
    for side in ["l", "r"]:
        data.ctrl[model.actuator(f"truck/drive_{side}").id] = 3
    mujoco.mj_step(model, data, nstep=1500)
    assert data.xpos[bid, 0] - before[0] > 0.3
    assert abs(joint(model, data, "truck/spin_rear_l")) > 1
    assert np.isfinite(data.qpos).all()


@pytest.mark.parametrize(
    "actuator,joint_name,effort,minimum,maximum",
    [
        ("cabinet/torque_left", "cabinet/hinge_left", 15, 0.3, 2.0),
        ("service_door/torque_left", "service_door/hinge_left", 25, 0.3, 2.0),
        ("bin/lid_torque", "bin/lid", 45, 0.2, 2.0),
        ("barrier/open", "barrier/lift", 1.4, 1.0, 1.62),
        ("hydrant/torque", "hydrant/turn", 1, 0.3, 32),
        ("forklift/lift", "forklift/lift", 0.8, 0.6, 0.95),
    ],
)
def test_articulated_parts_move_and_respect_limits(lab, actuator, joint_name, effort, minimum, maximum):
    model, data = lab
    data.ctrl[model.actuator(actuator).id] = effort
    mujoco.mj_step(model, data, nstep=1500)
    assert minimum < joint(model, data, joint_name) < maximum


def test_valve_spindle_follows_declared_screw_pitch(lab):
    model, data = lab
    data.ctrl[model.actuator("valve/torque").id] = 1
    mujoco.mj_step(model, data, nstep=1500)
    angle = joint(model, data, "valve/turn")
    height = joint(model, data, "valve/spindle_lift")
    assert angle > 0.5
    assert height == pytest.approx(angle * 0.12 / (12 * np.pi), abs=0.002)


def test_fans_rotate_without_moving_anchored_casing(lab):
    model, data = lab
    before = data.xpos[model.body("hvac/base").id].copy()
    data.ctrl[model.actuator("hvac/fan_drive_0").id] = 30
    mujoco.mj_step(model, data, nstep=500)
    assert joint(model, data, "hvac/fan_0") > 5
    np.testing.assert_array_equal(data.xpos[model.body("hvac/base").id], before)


def test_reset_restores_initial_pose_and_stops_commands(lab):
    model, data = lab
    data.ctrl[model.actuator("cabinet/torque_left").id] = 15
    mujoco.mj_step(model, data, nstep=500)
    mujoco.mj_resetData(model, data)
    np.testing.assert_array_equal(data.qpos, model.qpos0)
    assert not data.ctrl.any()
    assert not data.qvel.any()


def test_barrier_rest_and_return_are_not_blocked_by_housing(lab):
    model, data = lab
    assert abs(joint(model, data, "barrier/lift")) < 0.05
    data.ctrl[model.actuator("barrier/open").id] = 1.4
    mujoco.mj_step(model, data, nstep=1500)
    assert 1.0 < joint(model, data, "barrier/lift") < 1.5
    data.ctrl[model.actuator("barrier/open").id] = 0
    mujoco.mj_step(model, data, nstep=1500)
    assert abs(joint(model, data, "barrier/lift")) < 0.05


@pytest.mark.parametrize("asset_id,version", [("boom_barrier", "1.0.1"), ("service_door", "1.0.0")])
def test_archived_mechanism_is_independently_simulatable(asset_id, version):
    root = Path(__file__).resolve().parents[2]
    model = mujoco.MjModel.from_xml_path(
        str(root / "sim/library/industrial" / asset_id / version / "physics.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl[:] = 1.4 if asset_id == "boom_barrier" else 25
    mujoco.mj_step(model, data, nstep=1500)
    assert np.all(data.qpos > 0.5)
    assert np.all(data.qpos < model.jnt_range[:, 1] + 0.05)
    assert np.isfinite(data.qvel).all()
    if asset_id == "boom_barrier":
        data.ctrl[:] = 0
        mujoco.mj_step(model, data, nstep=1500)
        assert abs(float(data.qpos[0])) < 0.05
