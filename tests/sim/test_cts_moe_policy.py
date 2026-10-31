import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine
from sim.runtime.control.thunder.cts_policy import LEG_NAMES, TERM_DIMS, WHEEL_NAMES, CtsPolicy
from sim.runtime.control.thunder.joints import DART_TO_MJ, MJ_TO_DART
from sim.runtime.control.thunder.legacy_policy import FlatPolicy


def _contract():
    policy_order = list(LEG_NAMES + WHEEL_NAMES)
    physics_order = sorted(policy_order)
    default = np.array([-.12, -.93, 1.96, .12, .93, -1.96, .12, .93, -1.96, -.12, -.93, 1.96] + [0.] * 4)
    by_name = dict(zip(policy_order, default))
    return dict(
        policy_leg_order=list(LEG_NAMES), policy_wheel_order=list(WHEEL_NAMES), physics_order=physics_order,
        default_q=[by_name[name] for name in physics_order],
        soft_joint_pos_limits=[[-3., 3.]] * 16, position_scale=[.125, .25, .25] * 4,
        clip_actions=2., dt=.02, physics_dt=.005, velocity_scale=5.,
        actuator_limits=dict(leg_target_rate_rad_s=12., wheel_speed_rad_s=30., leg_torque_nm=120., wheel_torque_nm=17.),
        actuator_gains=dict(hip=dict(stiffness=120., damping=8.), thigh=dict(stiffness=120., damping=8.),
                            calf=dict(stiffness=120., damping=8.), wheel=dict(stiffness=0., damping=1.)),
        actuator_velocity_limit=dict(hip=15., thigh=15., calf=15., wheel=30.),
        actuator_saturation_effort=dict(hip=120., thigh=120., calf=120., wheel=17.),
    )


class _Session:
    def __init__(self, contract=None, action=None):
        self.contract = contract or _contract()
        self.action = np.zeros(16, dtype=np.float32) if action is None else np.asarray(action, dtype=np.float32)
        self.feeds = []

    def get_modelmeta(self):
        return SimpleNamespace(custom_metadata_map=dict(
            policy_contract="thunder_cts_moe_v1", runtime_contract=json.dumps(self.contract),
        ))

    def get_inputs(self):
        return [SimpleNamespace(name="history", shape=["batch", 530]),
                SimpleNamespace(name="single_obs", shape=["batch", 53])]

    def get_outputs(self):
        return [SimpleNamespace(name="actions", shape=["batch", 16])]

    def run(self, names, feeds):
        assert names == ["actions"]
        self.feeds.append({name: value.copy() for name, value in feeds.items()})
        return [self.action[None].copy()]


def _runner(contract=None, action=None):
    return CtsPolicy("cts.onnx", session=_Session(contract, action))


def _observation(runner, *, position=None, velocity=None, gyro=None, command=None):
    position = runner.default if position is None else position
    velocity = np.zeros(16) if velocity is None else velocity
    return runner.build_obs(
        np.zeros(3) if gyro is None else gyro, np.array([0., 0., -1.]),
        np.zeros(3) if command is None else command, position[DART_TO_MJ], velocity[DART_TO_MJ],
    )


def test_clip_terms_before_scaling_and_use_twelve_relative_leg_positions():
    runner = _runner()
    runner.last_action[:] = np.arange(16)
    position = runner.default + np.arange(16) * .01
    observation = _observation(
        runner, position=position, velocity=np.array([200., -200.] + [20.] * 14),
        gyro=np.array([200., -200., 4.]), command=np.array([.15, -.12, .8]),
    )
    np.testing.assert_array_equal(observation[:3], [25., -25., 1.])
    np.testing.assert_allclose(observation[6:9], [.15, -.12, .8])
    np.testing.assert_allclose(observation[9:21], np.arange(12) * .01, atol=1e-7)
    np.testing.assert_array_equal(observation[21:37], [5., -5.] + [1.] * 14)
    np.testing.assert_array_equal(observation[37:], np.arange(16))


def test_history_is_term_major_oldest_first_with_first_frame_padding():
    runner = _runner()
    _observation(runner)
    frames = []
    for index in range(12):
        single = np.arange(53, dtype=np.float32) + index * 100
        frames.append(single)
        runner.infer(single)
        if index == 0:
            expected_frames = np.repeat(single[None], 10, axis=0)
            expected = np.concatenate([
                block.reshape(-1) for block in np.split(expected_frames, np.cumsum(TERM_DIMS)[:-1], axis=1)
            ])[None]
            np.testing.assert_array_equal(runner.session.feeds[-1]["history"], expected)
    recent = np.array(frames[-10:])
    expected = np.concatenate([
        block.reshape(-1) for block in np.split(recent, np.cumsum(TERM_DIMS)[:-1], axis=1)
    ])[None]
    feed = runner.session.feeds[-1]
    np.testing.assert_array_equal(feed["history"], expected)
    np.testing.assert_array_equal(feed["single_obs"], frames[-1][None])
    assert not np.array_equal(feed["history"], recent.reshape(1, 530))


def test_slew_starts_from_measured_position_and_history_keeps_clipped_raw_action():
    runner = _runner(action=np.full(16, 100.))
    measured = runner.default + .05
    observation = _observation(runner, position=measured)
    targets = runner.infer(observation)
    requested = runner.default[:12] + 2. * runner.position_scale
    expected = measured[:12] + np.minimum(requested - measured[:12], .24)
    np.testing.assert_allclose(targets[:12], expected, atol=1e-7)
    np.testing.assert_array_equal(targets[12:], [10.] * 4)
    np.testing.assert_array_equal(runner.last_action, [2.] * 16)
    np.testing.assert_array_equal(_observation(runner, position=measured)[-16:], [2.] * 16)
    second = runner.infer(observation)
    assert np.all(second[:12] >= targets[:12])
    assert np.all(second[:12] - targets[:12] <= .240001)


def test_soft_bounds_wheel_speed_and_reset_do_not_reuse_previous_target():
    contract = _contract()
    contract["clip_actions"] = None
    contract["soft_joint_pos_limits"] = [[-.2, .2]] * 16
    runner = _runner(contract, action=np.full(16, 100.))
    observation = _observation(runner, position=np.zeros(16))
    targets = runner.infer(observation)
    np.testing.assert_allclose(targets[:12], [.2] * 12)
    np.testing.assert_array_equal(targets[12:], [30.] * 4)
    runner.reset()
    measured = np.full(16, -.1)
    runner.warm_up(np.zeros(3), [0., 0., -1.], measured[DART_TO_MJ], np.zeros(16))
    np.testing.assert_allclose(runner.previous_target, [-.1] * 12)
    np.testing.assert_array_equal(runner.last_action, np.zeros(16))
    assert runner._frames.shape == (10, 53)


@pytest.mark.parametrize("speed,positive,negative", [(0., 120., -120.), (7.5, 60., -120.), (15., 0., -120.),
                                                    (-7.5, 120., -60.), (-15., 120., 0.),
                                                    (22.5, -60., -120.), (-22.5, 120., 60.),
                                                    (40., -120., -120.), (-40., 120., 120.)])
def test_leg_torque_speed_envelope_preserves_braking_and_constant_wheel_caps(speed, positive, negative):
    runner = _runner()
    velocity = np.full(16, speed)
    for request, expected in ((200., positive), (-200., negative)):
        torque = runner.clip_torques(np.full(16, request), velocity)
        np.testing.assert_array_equal(torque[:12], [expected] * 12)
        np.testing.assert_array_equal(torque[12:], [np.sign(request) * 17.] * 4)


def test_supplied_session_is_reused_without_loading_onnx_again(monkeypatch):
    from sim.runtime.control import policy_loader

    def unexpected_session(*_args):
        raise AssertionError("the supplied session must be reused")

    session = _Session()
    monkeypatch.setattr(policy_loader, "open_session", unexpected_session)
    runner = policy_loader.load_policy("model.onnx", session=session)
    assert isinstance(runner, CtsPolicy)
    assert runner.session is session


def test_metadata_selects_cts_without_relying_on_path_or_first_input_dimension(monkeypatch):
    from sim.runtime.control import policy_loader

    monkeypatch.setattr(policy_loader, "open_session", lambda *args: _Session())
    runner = policy_loader.load_policy("unrelated/place/model.onnx")
    assert isinstance(runner, CtsPolicy)
    assert runner.run_at_idle and runner.zero_wheels_at_idle


def test_idle_brakes_wheels_and_nonzero_command_resumes_without_resetting_history():
    runner = _runner(action=np.ones(16))
    engine = MuJoCoEngine(robot_config=RobotConfig())
    engine._policy = runner
    engine._get_imu = lambda: (np.zeros(3), np.array([0., 0., -1.]))
    engine._get_joint_state = lambda: (runner.default[DART_TO_MJ], np.zeros(16))
    written = []
    engine._write_leg_ctrl = lambda targets: written.append(targets[MJ_TO_DART].copy())

    for command in ([0., 0., 0.], [.2, 0., 0.], [0., .2, 0.], [0., 0., .2], [0., 0., 0.]):
        engine._cmd_vel[:] = command
        engine._step_policy()
        np.testing.assert_array_equal(written[-1][12:], [5. if any(command) else 0.] * 4)
        np.testing.assert_allclose(written[-1][:12], runner.previous_target)
        np.testing.assert_array_equal(runner.last_action, np.ones(16))
    assert len(runner.session.feeds) == 5
    np.testing.assert_array_equal(runner.session.feeds[-1]["single_obs"][0, -16:], np.ones(16))


def test_cts_install_restores_legacy_gains_limits_and_feedback_without_pose_mutation(monkeypatch):
    from sim.compat.engine.mujoco import engine as engine_module

    engine = MuJoCoEngine(robot_config=RobotConfig())
    measured = np.arange(16) * .01
    engine._get_imu = lambda: (np.zeros(3), np.array([0., 0., -1.]))
    engine._get_joint_state = lambda: (measured[DART_TO_MJ].copy(), np.zeros(16))
    cts = _runner()
    flat = object.__new__(FlatPolicy)
    monkeypatch.setattr(engine_module, "load_policy", lambda path, **kwargs: cts if "cts" in path else flat)
    initial = {name: value.copy() if isinstance(value, list) else value for name, value in engine._policy_defaults.items()}
    engine._install_policy("cts.onnx", warm_up=True)
    assert engine._velocity_feedback is None
    assert (engine._robot_cfg.obs_dim, engine._robot_cfg.history_len) == (53, 10)
    np.testing.assert_array_equal(engine._robot_cfg.torque_kp, [120.] * 12 + [0.] * 4)
    np.testing.assert_array_equal(engine._robot_cfg.torque_kd, [8.] * 12 + [1.] * 4)
    np.testing.assert_allclose(cts.previous_target, measured[:12])
    np.testing.assert_array_equal(engine._get_joint_state()[0][MJ_TO_DART], measured)
    engine._install_policy("flat.onnx", warm_up=False)
    assert engine._velocity_feedback is not None
    assert (engine._robot_cfg.obs_dim, engine._robot_cfg.history_len) == (53, 1)
    assert engine._robot_cfg.torque_limit == initial["torque_limit"]
    from sim.runtime.control.thunder import flat as thunderv4_flat

    np.testing.assert_array_equal(engine._robot_cfg.torque_kp, thunderv4_flat.KP)


def test_engine_applies_cts_motor_envelope_in_correct_joint_order_and_removes_it_on_restore(monkeypatch):
    from sim.compat.engine.mujoco import engine as engine_module

    engine = MuJoCoEngine(robot_config=RobotConfig())
    cts = _runner()
    flat = object.__new__(FlatPolicy)
    monkeypatch.setattr(engine_module, "load_policy", lambda path, **kwargs: cts if "cts" in path else flat)
    position = cts.default.copy()
    velocity = np.array([7.5] * 12 + [0.] * 4)
    engine._get_joint_state = lambda: (position[DART_TO_MJ], velocity[DART_TO_MJ])
    target = position + 2.
    target[12:] = 100.
    engine._install_policy("cts.onnx", warm_up=False)
    torque = engine._compute_leg_torques(target[DART_TO_MJ])[MJ_TO_DART]
    np.testing.assert_array_equal(torque, [60.] * 12 + [17.] * 4)
    engine._install_policy("flat.onnx", warm_up=False)
    torque = engine._compute_leg_torques(target[DART_TO_MJ])[MJ_TO_DART]
    np.testing.assert_array_equal(torque, [120.] * 12 + [17.] * 4)


@pytest.mark.parametrize("original_dt", [.002, .004, .005])
def test_cts_switch_restores_previous_world_physics_timestep(monkeypatch, original_dt):
    from sim.compat.engine.mujoco import engine as engine_module

    engine = MuJoCoEngine(robot_config=RobotConfig())
    engine._model = SimpleNamespace(opt=SimpleNamespace(timestep=original_dt))
    engine._physics_dt = original_dt
    cts = _runner()
    flat = object.__new__(FlatPolicy)
    monkeypatch.setattr(engine_module, "load_policy", lambda path, **kwargs: cts if "cts" in path else flat)
    engine._install_policy("cts.onnx", warm_up=False)
    assert engine.dt == engine._model.opt.timestep == .005
    engine._install_policy("cts-second.onnx", warm_up=False)
    assert engine._cts_previous_physics_dt == original_dt
    engine._install_policy("flat.onnx", warm_up=False)
    assert engine.dt == engine._model.opt.timestep == original_dt
    assert engine._cts_previous_physics_dt is None


def test_actual_onnx_direct_load_and_switch_keep_physics_truth_and_restore_timestep():
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    from drivers.sim.mujoco.driver import MujocoDriverModule

    root = Path(__file__).resolve().parents[2]
    cts = root / "sim/packages/controllers/doso/thunder_v4/cts_moe/policy/policy_cts_moe_v39_61128.onnx"
    if not cts.is_file():
        pytest.skip("CTS exported policy is not installed")
    legacy = root / "sim/packages/controllers/doso/thunder_v4/locomotion/policy"
    driver = MujocoDriverModule(
        world="open_field", render=False, enable_camera=False, drive_mode="policy", policy_path=str(cts),
    )
    driver.setup()
    engine = driver._engine
    try:
        assert isinstance(engine._policy, CtsPolicy)
        np.testing.assert_allclose(engine._policy.default[[0, 3, 6, 9]], [-.12, .12, .12, -.12])
        assert engine.dt == engine.model.opt.timestep == .005
        assert engine.control_dt == .02
        assert engine._velocity_feedback is None
        qpos, qvel = engine.data.qpos.copy(), engine.data.qvel.copy()
        engine.switch_policy(str(legacy / "policy_4998.onnx"))
        np.testing.assert_array_equal(engine.data.qpos, qpos)
        np.testing.assert_array_equal(engine.data.qvel, qvel)
        assert engine._velocity_feedback is not None
        engine.set_physics_timestep(.004)
        engine.switch_policy(str(cts))
        assert engine.dt == .005
        np.testing.assert_array_equal(engine.data.qpos, qpos)
        np.testing.assert_array_equal(engine.data.qvel, qvel)
        engine.switch_policy(str(legacy / "policy_1119.onnx"))
        assert engine.dt == engine.model.opt.timestep == .004
        assert (engine._robot_cfg.obs_dim, engine._robot_cfg.history_len) == (57, 5)
        np.testing.assert_array_equal(engine.data.qpos, qpos)
        np.testing.assert_array_equal(engine.data.qvel, qvel)
        assert engine._cts_previous_physics_dt is None
    finally:
        engine.close()
