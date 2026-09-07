from __future__ import annotations

import copy
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from sim.packages.robots.doso.thunder_v4.tools import generate_thunderv4_mjcf as generator
from sim.scripts.mujoco import continuous_walk

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"
ASSET_ROOT = SIM_ROOT / "packages" / "robots" / "doso" / "thunder_v4"
MJCF_ROOT = ASSET_ROOT / "mjcf"

ACTUATOR_ORDER = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
)
EXPECTED_VELOCITY_LIMITS = np.asarray([17.48] * 12 + [44.0] * 4)


def _model(filename: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MJCF_ROOT / filename))


def _actuator_names(model: mujoco.MjModel) -> tuple[str, ...]:
    return tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) for actuator_id in range(model.nu))


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert joint_id >= 0, f"missing joint {name}"
    return int(joint_id)


def _keyboard_module():
    pytest.importorskip("torch", reason="Thunder V4 keyboard checks require torch")
    script_path = (
        REPO_ROOT
        / "sim" / "packages" / "controllers"
        / "doso"
        / "thunder_v4"
        / "locomotion"
        / "keyboard.py"
    )
    spec = importlib.util.spec_from_file_location("thunderv4_keyboard", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_flat_and_stairs_use_the_same_real_v4_hardware_contract() -> None:
    for filename in ("thunderv4.xml", "thunderv4_stairs.xml"):
        model = _model(filename)

        assert np.isclose(model.body_mass.sum(), 45.8086, atol=1e-3)
        assert model.nu == 16
        assert _actuator_names(model) == ACTUATOR_ORDER

        for joint_name in ACTUATOR_ORDER[-4:]:
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            assert actuator_id >= 0
            assert np.allclose(model.actuator_ctrlrange[actuator_id], (-17.0, 17.0))

        numeric_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "hardware_joint_velocity_limit_rad_s")
        assert numeric_id >= 0
        start = model.numeric_adr[numeric_id]
        size = model.numeric_size[numeric_id]
        assert np.allclose(model.numeric_data[start : start + size], EXPECTED_VELOCITY_LIMITS)

        for wheel_name in ("FR_wheel", "FL_wheel", "RR_wheel", "RL_wheel"):
            wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, wheel_name)
            assert wheel_id >= 0, f"missing V4 wheel geom {wheel_name} in {filename}"
            assert np.isclose(model.geom_size[wheel_id, 0], 0.093, atol=1e-6)
            assert np.allclose(model.geom_friction[wheel_id], (1.0, 0.005, 0.0001))

        assert np.allclose(model.geom("ground").friction, (1.0, 0.005, 0.0001))

        sensor_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id) for sensor_id in range(model.nsensor)
        }
        assert {
            "lidar-orientation",
            "lidar-position",
            "lidar-angular-velocity",
            "lidar-linear-velocity",
            "lidar-linear-acceleration",
        }.issubset(sensor_names)

        collision_alpha = model.geom_rgba[model.geom_group == 3, 3]
        assert collision_alpha.size > 0
        assert np.allclose(collision_alpha, 0.0)

    stairs = _model("thunderv4_stairs.xml")
    for step_name in ("step_1", "step_2", "step_3"):
        assert mujoco.mj_name2id(stairs, mujoco.mjtObj.mjOBJ_GEOM, step_name) >= 0
        assert np.allclose(stairs.geom(step_name).friction, (1.0, 0.005, 0.0001))


def test_generated_wheel_uses_rubber_friction_after_collision_annotation() -> None:
    body = ET.fromstring('<body name="fr_foot_Link"><geom type="cylinder" size="0.093 0.026"/></body>')
    generator._rename_tree(body, show_collisions=False)
    generator._mark_wheel_collisions(body)
    root = ET.Element("mujoco")
    root.append(copy.deepcopy(ET.parse(MJCF_ROOT / "thunderv4.xml").getroot().find("default")))
    ET.SubElement(root, "worldbody").append(body)

    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))

    assert np.allclose(model.geom("FR_wheel").friction, (1.0, 0.005, 0.0001))


def test_merged_open_field_keeps_floor_friction_and_low_wheel_contact_torsion() -> None:
    world = mujoco.MjModel.from_xml_path(str(continuous_walk.WORLD_XML))
    engine = continuous_walk._build_engine()
    try:
        assert np.allclose(engine._model.geom("floor").friction, world.geom("floor").friction)
        for _ in range(25):
            engine.step(continuous_walk.VelocityCommand())
        wheels = {engine._model.geom(f"{leg}_wheel").id for leg in ("FR", "FL", "RR", "RL")}
        contacts = [c for c in engine._data.contact if c.geom1 in wheels or c.geom2 in wheels]
        assert contacts
        assert all(contact.dim == 4 for contact in contacts)
        assert all(np.isclose(contact.friction[2], 0.005) for contact in contacts)
    finally:
        engine.close()


def test_manual_high_stand_pose_stays_within_true_v4_joint_limits() -> None:
    module = _keyboard_module()

    model = _model("thunderv4.xml")
    for joint_name, target in module.POSE_B_ANGLES.items():
        joint_id = _joint_id(model, joint_name)
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        lower, upper = model.jnt_range[joint_id]
        if np.isclose(lower, upper):
            continue
        assert lower <= target <= upper, f"{joint_name} target {target} outside true V4 range [{lower}, {upper}]"


def test_keyboard_observation_and_action_layout_follow_loaded_v4_names() -> None:
    module = _keyboard_module()
    model = _model("thunderv4.xml")
    qpos_ids, qvel_ids, actuator_ids = module.resolve_joint_addresses(model)
    velocity_limits = module.resolve_hardware_velocity_limits(model)

    assert tuple(actuator_ids) == tuple(range(16))
    assert tuple(qpos_ids) == (7, 8, 9, 11, 12, 13, 15, 16, 17, 19, 20, 21, 10, 14, 18, 22)
    assert tuple(qvel_ids) == (6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20, 9, 13, 17, 21)
    assert np.allclose(velocity_limits, EXPECTED_VELOCITY_LIMITS)

    tau = np.ones(16, dtype=np.float64)
    qvel = velocity_limits * 0.95
    guarded_tau = module.attenuate_overspeed_torque(tau, qvel, velocity_limits)
    assert np.allclose(guarded_tau, 0.5)

    data = mujoco.MjData(model)
    stand_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "v4_nominal_stand")
    data.qpos[:] = model.key_qpos[stand_key]
    mujoco.mj_forward(model, data)
    obs = module.get_obs(
        data,
        module.Cmd(),
        np.zeros(16, dtype=np.float32),
        qpos_ids,
        qvel_ids,
    )

    assert obs.shape == (57,)
    assert np.isfinite(obs).all()
