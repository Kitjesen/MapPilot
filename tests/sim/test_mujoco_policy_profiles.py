from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sim.compat.engine.core.engine import VelocityCommand
from sim.scripts.mujoco.formal_feeder import _PolicyProfiles


class _Session:
    def __init__(self):
        self.request = None
        self.states = []

    def take_profile_request(self):
        request, self.request = self.request, None
        return request

    def publish_profile_state(self, **state):
        self.states.append(state)


class _Engine:
    def __init__(self, ordinary):
        self.active_policy_path = str(ordinary)
        self.sim_time = 0.0
        self.switches = []

    def switch_policy(self, path):
        changed = path != self.active_policy_path
        self.active_policy_path = path
        if changed:
            self.switches.append(path)
        return changed


def _fixture(tmp_path):
    ordinary = (tmp_path / "policies/ordinary.onnx").resolve()
    engine = _Engine(ordinary)
    config = SimpleNamespace(policy="policies/ordinary.onnx", policy_profiles=(
        ("stairs_up", "policies/up.onnx", "policies/up.json"),
        ("stairs_down", "policies/down.onnx", "policies/down.json"),
    ))
    profiles = _PolicyProfiles(engine, ordinary, config)
    return engine, profiles, _Session()


def _tick(engine, profiles, session, t, *, speed=0.0, vertical_speed=0.0, angular_speed=0.0, command=0.0):
    engine.sim_time = t
    return profiles.step(session, VelocityCommand(command), SimpleNamespace(
        linear_velocity=np.array([speed, 0.0, vertical_speed]), angular_velocity=np.array([0.0, 0.0, angular_speed]),
    ), t)


def test_profile_switch_waits_for_actual_quiet_then_independent_readback(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    session.request = dict(task_id="one", requirement="stairs_up")
    _tick(engine, profiles, session, 0, speed=.2)
    _tick(engine, profiles, session, .3, command=.1)
    assert not engine.switches
    _tick(engine, profiles, session, .4)
    _tick(engine, profiles, session, .61)
    assert session.states[-1]["phase"] == "switching"
    assert engine.active_policy_path.endswith("up.onnx")
    _tick(engine, profiles, session, .62)
    assert session.states[-1]["phase"] == "switching"
    _tick(engine, profiles, session, .83)
    assert session.states[-1]["phase"] == "ready"
    assert session.states[-1]["requested_profile"] == session.states[-1]["active_profile"]
    for t in (.9, 1., 1.1):
        session.request = dict(task_id="one", requirement="stairs_up")
        _tick(engine, profiles, session, t)
    assert len(engine.switches) == 1


@pytest.mark.parametrize("motion", [dict(vertical_speed=-.2125), dict(speed=.05), dict(angular_speed=.13), dict(command=.01)])
def test_loaded_profile_waits_for_post_switch_motion_to_settle(tmp_path, motion):
    engine, profiles, session = _fixture(tmp_path)
    session.request = dict(task_id="one", requirement="stairs_up")
    for t in (0, .21, .22):
        _tick(engine, profiles, session, t)
    assert engine.active_policy_path.endswith("up.onnx")
    _tick(engine, profiles, session, .4, **motion)
    _tick(engine, profiles, session, .5)
    _tick(engine, profiles, session, .69)
    assert session.states[-1]["phase"] == "switching"
    _tick(engine, profiles, session, .71)
    assert session.states[-1]["phase"] == "ready"
    assert len(engine.switches) == 1


def test_same_policy_for_flat_and_stairs_is_ready_without_stopping_or_reloading(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    ordinary = engine.active_policy_path
    profiles.paths = {name: Path(ordinary) for name in ("stairs_up", "stairs_down")}
    for index, requirement in enumerate(("stairs_up", "stairs_down", "restore")):
        session.request = dict(task_id="one", requirement=requirement)
        _tick(engine, profiles, session, index * .02, speed=.4, vertical_speed=.15, command=.4)
        assert session.states[-1]["phase"] == "ready"
        assert session.states[-1]["active_profile"] == session.states[-1]["requested_profile"]
        assert engine.active_policy_path == ordinary
    assert engine.switches == []


def test_profile_hold_preserves_actual_policy_and_restore_returns_initial_file(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    original = engine.active_policy_path
    session.request = dict(task_id="one", requirement="stairs_up")
    for t in (0, .21, .22, .43):
        _tick(engine, profiles, session, t)
    session.request = dict(task_id="one", requirement="hold")
    _tick(engine, profiles, session, .5)
    _tick(engine, profiles, session, 1)
    assert engine.active_policy_path.endswith("up.onnx")
    assert len(engine.switches) == 1
    session.request = dict(task_id="one", requirement="restore")
    for t in (1.1, 1.31, 1.32, 1.53):
        _tick(engine, profiles, session, t)
    assert engine.active_policy_path == original
    assert session.states[-1]["phase"] == "ready"
    assert len(engine.switches) == 2


def test_resume_held_profile_preserves_restore_target_across_next_task(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    original = engine.active_policy_path
    session.request = dict(task_id="one", requirement="stairs_up")
    for t in (0, .21, .22, .43):
        _tick(engine, profiles, session, t)
    session.request = dict(task_id="one", requirement="hold")
    _tick(engine, profiles, session, .5)
    assert session.states[-1]["phase"] == "waiting"
    session.request = dict(task_id="one", requirement="stairs_up")
    for t in (.6, .81, .82):
        _tick(engine, profiles, session, t)
    assert session.states[-1]["phase"] == "ready"
    assert len(engine.switches) == 1
    session.request = dict(task_id="one", requirement="restore")
    for t in (.9, 1.11, 1.12, 1.33):
        _tick(engine, profiles, session, t)
    assert engine.active_policy_path == original
    session.request = dict(task_id="two", requirement="stairs_down")
    for t in (1.4, 1.61, 1.62, 1.83):
        _tick(engine, profiles, session, t)
    assert engine.active_policy_path.endswith("down.onnx")
    session.request = dict(task_id="two", requirement="restore")
    for t in (1.9, 2.11, 2.12, 2.33):
        _tick(engine, profiles, session, t)
    assert engine.active_policy_path == original
    assert session.states[-1]["phase"] == "ready"


def test_profile_switch_does_not_confirm_wrong_readback(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    engine.switch_policy = lambda path: True
    session.request = dict(task_id="one", requirement="stairs_down")
    for t in (0, .21, .22):
        _tick(engine, profiles, session, t)
    assert session.states[-1]["phase"] == "unsupported"
    assert session.states[-1]["reason"] == "profile_readback_mismatch"


def test_unconfigured_profile_is_reported_without_relabeling_actual_policy(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    profiles.paths.clear()
    session.request = dict(task_id="one", requirement="stairs_up")
    _tick(engine, profiles, session, 0)
    assert session.states[-1]["phase"] == "unsupported"
    assert session.states[-1]["active_profile"] == "policies/ordinary.onnx"
    assert not engine.switches


def test_restore_without_prior_transition_cannot_claim_success(tmp_path):
    engine, profiles, session = _fixture(tmp_path)
    session.request = dict(task_id="one", requirement="restore")
    _tick(engine, profiles, session, 0)
    assert session.states[-1]["phase"] == "unsupported"
    assert session.states[-1]["reason"] == "previous_profile_unknown"
    assert not engine.switches


def test_real_onnx_swap_restores_contract_without_mutating_physics_truth():
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    from drivers.sim.mujoco.driver import MujocoDriverModule

    root = Path(__file__).resolve().parents[2]
    folder = root / "sim/packages/controllers/doso/thunder_v4/locomotion/policy"
    original = folder / "policy_1119.onnx"
    alternate = folder / "policy_4998.onnx"
    driver = MujocoDriverModule(
        world="open_field", render=False, enable_camera=False,
        drive_mode="policy", policy_path=str(original),
    )
    driver.setup()
    engine = driver._engine
    try:
        original_kp = engine._robot_cfg.torque_kp.copy()
        original_pose = engine._robot_cfg.standing_pose.copy()
        before_position, before_velocity = engine.data.qpos.copy(), engine.data.qvel.copy()
        assert engine.switch_policy(str(alternate))
        assert engine.active_policy_path == str(alternate.resolve())
        assert (engine._robot_cfg.obs_dim, engine._robot_cfg.history_len) == (53, 1)
        assert engine._velocity_feedback is not None
        np.testing.assert_array_equal(engine.data.qpos, before_position)
        np.testing.assert_array_equal(engine.data.qvel, before_velocity)
        assert not engine.switch_policy(str(alternate))
        engine.step(VelocityCommand())
        assert np.isfinite(engine.data.ctrl).all()
        before_position, before_velocity = engine.data.qpos.copy(), engine.data.qvel.copy()
        assert engine.switch_policy(str(original))
        assert engine.active_policy_path == str(original.resolve())
        assert (engine._robot_cfg.obs_dim, engine._robot_cfg.history_len) == (57, 5)
        assert engine._robot_cfg.torque_kp == original_kp
        assert engine._robot_cfg.standing_pose == original_pose
        assert engine._velocity_feedback is None
        np.testing.assert_array_equal(engine.data.qpos, before_position)
        np.testing.assert_array_equal(engine.data.qvel, before_velocity)
        engine.step(VelocityCommand())
        assert np.isfinite(engine.data.ctrl).all()
        engine.step(VelocityCommand(.1))
        with pytest.raises(RuntimeError, match="zero velocity"):
            engine.switch_policy(str(alternate))
        assert engine.active_policy_path == str(original.resolve())
    finally:
        engine.close()
