# ruff: noqa: S101

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import mujoco
import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
MODEL_PATH = WORKSPACE_ROOT / "brain" / "lingtu" / "sim" / "packages" / "robots" / "thunderv4" / "mjcf" / "thunderv4.xml"
PLAYER_PATH = WORKSPACE_ROOT / "tools" / "rl_eval" / "mujoco_v86_recovery_play_aligned.py"


def _player_module():
    spec = importlib.util.spec_from_file_location("v86_recovery_player_test", PLAYER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_v86_transfer_layout_uses_named_v4_joint_and_velocity_contract() -> None:
    player = _player_module()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    qpos_ids, qvel_ids, actuator_ids = player.control_addresses(model)
    velocity_limits = player.resolve_hardware_velocity_limits(model)

    assert tuple(actuator_ids) == tuple(range(16))
    assert tuple(qpos_ids) == (7, 8, 9, 11, 12, 13, 15, 16, 17, 19, 20, 21, 10, 14, 18, 22)
    assert tuple(qvel_ids) == (6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20, 9, 13, 17, 21)
    assert np.allclose(velocity_limits, np.asarray([17.48] * 12 + [44.0] * 4))

    data = mujoco.MjData(model)
    data.qpos[3] = 1.0
    data.qpos[qpos_ids] = player.DEFAULT_POSE
    mujoco.mj_forward(model, data)
    obs = player.build_obs(
        data,
        qpos_ids,
        qvel_ids,
        np.zeros(16, dtype=np.float64),
        policy_step=0,
    )
    assert obs.shape == (60,)
    assert np.isfinite(obs).all()


def test_v86_speed_guard_preserves_braking_torque() -> None:
    player = _player_module()
    limits = np.asarray([17.48] * 12 + [44.0] * 4)
    velocity = limits * 0.95

    accelerating = player.attenuate_overspeed_torque(np.ones(16), velocity, limits)
    braking = player.attenuate_overspeed_torque(-np.ones(16), velocity, limits)
    beyond_limit = player.attenuate_overspeed_torque(np.ones(16), limits * 1.05, limits)

    assert np.allclose(accelerating, 0.5)
    assert np.allclose(braking, -1.0)
    assert np.allclose(beyond_limit, 0.0)


def test_v86_training_timing_is_5ms_physics_and_20ms_policy() -> None:
    player = _player_module()

    assert np.isclose(player.TRAINING_PHYSICS_DT, 0.005)
    assert np.isclose(player.POLICY_DT, 0.02)
    assert player.policy_substeps(player.TRAINING_PHYSICS_DT) == 4


def test_v86_inverted_case_labels_expose_the_real_rotation_axis() -> None:
    player = _player_module()

    assert "inverted roll 180" in player.CASES["supine"].label
    assert "inverted pitch 180" in player.CASES["prone"].label


def test_black_render_hides_collision_geoms_without_changing_contact_groups() -> None:
    player = _player_module()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    original_contype = model.geom_contype.copy()
    player.configure_render_appearance(model, "black")

    collision_mask = model.geom_group == 3
    visible_robot_mask = (model.geom_bodyid > 0) & ~collision_mask & (model.geom_rgba[:, 3] > 0.0)
    assert np.allclose(model.geom_rgba[collision_mask, 3], 0.0)
    assert np.array_equal(model.geom_contype, original_contype)
    assert np.allclose(model.geom_rgba[visible_robot_mask, :3], player.BLACK_ROBOT_RGB)


def test_hd_framebuffer_is_expanded_before_renderer_creation() -> None:
    player = _player_module()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    player.configure_offscreen_framebuffer(model, 1920, 1080)

    assert model.vis.global_.offwidth >= 1920
    assert model.vis.global_.offheight >= 1080


def test_video_frame_count_preserves_8_second_30_fps_duration() -> None:
    player = _player_module()

    assert player.expected_video_frame_count(8.0, 30) == 240
