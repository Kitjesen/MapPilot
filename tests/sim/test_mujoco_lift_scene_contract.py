from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "sim" / "compat" / "engine" / "worlds" / "lift_building_scene.xml"


def test_lift_building_scene_declares_deterministic_joint_contract() -> None:
    root = ET.parse(SCENE).getroot()
    joints = {joint.get("name"): joint for joint in root.findall(".//joint")}

    assert joints["lift_cabin_z"].get("type") == "slide"
    assert joints["lift_cabin_z"].get("axis") == "0 0 1"
    assert joints["lift_cabin_z"].get("range") == "0 3.5"
    assert joints["lift_door_left_slide"].get("range") == "-0.65 0"
    assert joints["lift_door_right_slide"].get("range") == "0 0.65"

    bodies = {body.get("name") for body in root.findall(".//body")}
    assert {"lift_cabin", "lift_door_left", "lift_door_right"} <= bodies
    assert root.find(".//body[@name='robot_placeholder']") is not None


def test_lift_building_scene_loads_and_moves_cabin_and_doors() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    cabin_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_cabin_z")
    left_joint = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "lift_door_left_slide",
    )
    right_joint = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "lift_door_right_slide",
    )
    cabin_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lift_cabin")
    left_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lift_door_left")
    right_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lift_door_right")

    data.qpos[model.jnt_qposadr[cabin_joint]] = 3.5
    data.qpos[model.jnt_qposadr[left_joint]] = -0.65
    data.qpos[model.jnt_qposadr[right_joint]] = 0.65
    mujoco.mj_forward(model, data)

    assert data.xpos[cabin_body][2] == pytest.approx(3.5)
    assert data.xpos[left_body][0] < 5.1
    assert data.xpos[right_body][0] > 6.9


def test_lift_building_world_alias_resolves() -> None:
    from drivers.sim.mujoco.runtime import resolve_world

    assert resolve_world("lift_building") == SCENE.resolve()


def test_lift_scene_merges_with_configured_thunder_robot_model() -> None:
    mujoco = pytest.importorskip("mujoco")
    from drivers.sim.mujoco.runtime import build_engine, resolve_world

    engine = build_engine(
        world=resolve_world("lift_building"),
        drive_mode="kinematic",
        start=[6.0, 2.6, 0.48],
        mujoco_memory="",
        mid360_pattern=None,
        lidar_backend="mujoco_lidar",
        require_product_lidar_backend=False,
    )
    try:
        assert (
            mujoco.mj_name2id(
                engine._model,
                mujoco.mjtObj.mjOBJ_BODY,
                "base_link",
            )
            >= 0
        )
        assert (
            mujoco.mj_name2id(
                engine._model,
                mujoco.mjtObj.mjOBJ_BODY,
                "lift_cabin",
            )
            >= 0
        )
        assert (
            mujoco.mj_name2id(
                engine._model,
                mujoco.mjtObj.mjOBJ_JOINT,
                "lift_cabin_z",
            )
            >= 0
        )
    finally:
        engine.close()
