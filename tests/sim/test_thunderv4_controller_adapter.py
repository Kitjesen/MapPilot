# ruff: noqa: S101

import inspect
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

import sim.runtime.control.thunderv4 as thunderv4_module
from sim.runtime.control import create_thunderv4_components
from sim.runtime.control.contracts import (
    ControllerCommand,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
)
from sim.runtime.control.plan import (
    ActuatorLayout,
    AdapterSpec,
    ControllerSpec,
    PolicySpec,
)
from sim.runtime.control.thunderv4 import (
    ThunderV4ControllerAdapter,
    ThunderV4OnnxPolicy,
    ThunderV4TorchScriptPolicy,
)

_ACTUATOR_CHANNELS = (
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
_MUJOCO_ACTUATOR_CHANNELS = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FR_foot_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FL_foot_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RR_foot_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RL_foot_joint",
)
_STANDING_POSE = np.asarray(
    (
        -0.1,
        -0.8,
        1.8,
        0.1,
        0.8,
        -1.8,
        0.1,
        0.8,
        -1.8,
        -0.1,
        -0.8,
        1.8,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
    dtype=np.float64,
)


def _controller_spec(*, actuators: tuple[str, ...] = _ACTUATOR_CHANNELS) -> ControllerSpec:
    return ControllerSpec(
        controller_id="thunder_01.thunderv4_locomotion",
        instance_id="thunder_01",
        adapter=AdapterSpec(
            plugin="quadruped_him",
            abi="lingtu.sim.controller-adapter.v1",
        ),
        policy=PolicySpec(
            runtime="torchscript",
            artifact="artifacts/controller.pt",
            manifest="artifacts/policy_manifest.json",
        ),
        inference_hz=50.0,
        low_level_hz=500.0,
        state_channels=(
            "joint_position",
            "joint_velocity",
            "base_angular_velocity",
            "projected_gravity",
        ),
        command_channels=(
            "thunder_01.control.base_twist",
            "thunder_01.control.joint_torque",
        ),
        actuators=ActuatorLayout(actuators),
    )


def _state(
    *,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
) -> ControllerState:
    return ControllerState(
        session_id="c" * 64,
        instance_id="thunder_01",
        generation=GenerationStamp(1, 2),
        sequence=3,
        sim_time_ns=4,
        channels={
            "joint_position": tuple(joint_position),
            "joint_velocity": tuple(joint_velocity),
            "base_angular_velocity": (4.0, 8.0, 12.0),
            "projected_gravity": (0.1, 0.2, -0.9),
        },
    )


def _command() -> ControllerCommand:
    return _command_with_payload({"linear_x": 0.6, "angular_z": -0.25})


def _command_with_payload(payload: dict[str, float]) -> ControllerCommand:
    return ControllerCommand(
        channel_id="thunder_01.control.base_twist",
        instance_id="thunder_01",
        generation=GenerationStamp(1, 2),
        sequence=5,
        apply_time_ns=4,
        payload=payload,
    )


def _neutral_state() -> ControllerState:
    return _state(
        joint_position=_STANDING_POSE.copy(),
        joint_velocity=np.zeros(16, dtype=np.float64),
    )


class _FakeModel:
    def __init__(self) -> None:
        self.eval_called = False

    def eval(self) -> "_FakeModel":
        self.eval_called = True
        return self


class _FakeTorchScriptBackend:
    def __init__(self, output: np.ndarray | None = None) -> None:
        self.loaded_paths: list[Path] = []
        self.observations: list[np.ndarray] = []
        self.model = _FakeModel()
        self.output = np.zeros(16, dtype=np.float32) if output is None else output

    def load(self, path: Path) -> _FakeModel:
        self.loaded_paths.append(path)
        return self.model

    def infer(self, model: _FakeModel, observation: np.ndarray) -> np.ndarray:
        assert model is self.model
        self.observations.append(observation.copy())
        return self.output.copy()


class _FakeIo:
    def __init__(self, name: str, shape: list[int]) -> None:
        self.name = name
        self.shape = shape


class _FakeOnnxSession:
    def __init__(self, output: np.ndarray | None = None) -> None:
        self.observations: list[np.ndarray] = []
        self.output = np.zeros((1, 16), dtype=np.float32) if output is None else output

    @staticmethod
    def get_inputs() -> list[_FakeIo]:
        return [_FakeIo("obs_history", [1, 285])]

    @staticmethod
    def get_outputs() -> list[_FakeIo]:
        return [_FakeIo("actions", [1, 16])]

    def run(self, output_names: list[str], inputs: dict[str, np.ndarray]) -> list[np.ndarray]:
        assert output_names == ["actions"]
        observation = inputs["obs_history"]
        self.observations.append(observation.copy())
        return [self.output.copy()]


def test_policy_resolves_plan_artifact_from_repo_root(tmp_path: Path) -> None:
    artifact = tmp_path / "artifacts" / "controller.pt"
    artifact.parent.mkdir()
    artifact.write_bytes(b"fake-torchscript-policy")
    backend = _FakeTorchScriptBackend()
    spec = PolicySpec(
        runtime="torchscript",
        artifact="artifacts/controller.pt",
        manifest="artifacts/policy_manifest.json",
    )

    policy = ThunderV4TorchScriptPolicy(spec, tmp_path, backend=backend)

    assert policy.artifact_path == artifact.resolve()
    assert backend.loaded_paths == [artifact.resolve()]
    assert backend.model.eval_called is True


def test_policy_rejects_missing_artifact_before_loading_model(tmp_path: Path) -> None:
    backend = _FakeTorchScriptBackend()
    spec = PolicySpec(
        runtime="torchscript",
        artifact="missing.pt",
        manifest="policy_manifest.json",
    )

    with pytest.raises(ControllerRuntimeError, match="not a file"):
        ThunderV4TorchScriptPolicy(spec, tmp_path, backend=backend)

    assert backend.loaded_paths == []


def test_adapter_builds_quadruped_him_observation_from_typed_runtime_inputs() -> None:
    adapter = ThunderV4ControllerAdapter(_controller_spec())
    position_offset = np.arange(16, dtype=np.float64) * 0.1
    velocity = np.arange(16, dtype=np.float64)

    observation = adapter.observe(
        _state(
            joint_position=_STANDING_POSE + position_offset,
            joint_velocity=velocity,
        ),
        _command(),
        ActuatorLayout(_ACTUATOR_CHANNELS),
    )

    expected = np.concatenate(
        (
            np.asarray((1.0, 2.0, 3.0)),
            np.asarray((0.1, 0.2, -0.9)),
            np.asarray((0.6, 0.0, -0.25)),
            np.concatenate((position_offset[:12], np.zeros(4))),
            velocity * 0.05,
            np.zeros(16),
        )
    ).astype(np.float32)
    assert observation.dtype == np.float32
    np.testing.assert_allclose(observation, expected)


def test_adapter_passes_policy_yaw_observation_without_mutating_base_twist_command() -> None:
    adapter = ThunderV4ControllerAdapter(_controller_spec())
    payload = {"linear_x": 0.1, "linear_y": -0.2, "angular_z": 0.35}
    command = _command_with_payload(payload)

    observation = adapter.observe(
        _neutral_state(),
        command,
        ActuatorLayout(_ACTUATOR_CHANNELS),
    )

    assert command.payload is payload
    assert payload == {"linear_x": 0.1, "linear_y": -0.2, "angular_z": 0.35}
    np.testing.assert_allclose(observation[6:9], np.asarray((0.1, -0.2, 0.35), dtype=np.float32))


def test_adapter_passes_policy_yaw_observation_directly() -> None:
    adapter = ThunderV4ControllerAdapter(_controller_spec())
    layout = ActuatorLayout(_ACTUATOR_CHANNELS)

    negative = adapter.observe(_neutral_state(), _command_with_payload({"angular_z": -0.35}), layout)
    high = adapter.observe(_neutral_state(), _command_with_payload({"angular_z": 1.5}), layout)
    low = adapter.observe(_neutral_state(), _command_with_payload({"angular_z": -1.5}), layout)

    assert negative[8] == pytest.approx(-0.35)
    assert high[8] == pytest.approx(1.5)
    assert low[8] == pytest.approx(-1.5)


@pytest.mark.parametrize("yaw", [float("nan"), float("inf"), -float("inf")])
def test_adapter_rejects_non_finite_policy_yaw_observation_inputs(yaw: float) -> None:
    adapter = ThunderV4ControllerAdapter(_controller_spec())

    with pytest.raises(ControllerRuntimeError, match="base_twist command"):
        adapter.observe(
            _neutral_state(),
            _command_with_payload({"angular_z": yaw}),
            ActuatorLayout(_ACTUATOR_CHANNELS),
        )


def test_thunderv4_controller_package_does_not_rewrite_base_twist() -> None:
    manifest = Path("sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml").read_text(
        encoding="utf-8"
    )

    assert "command_calibration:" not in manifest


def test_policy_infers_float32_batch_and_holds_standing_pose_at_startup(
    tmp_path: Path,
) -> None:
    artifact = tmp_path / "controller.pt"
    artifact.write_bytes(b"fake-torchscript-policy")
    raw_action = np.arange(16, dtype=np.float32) * 0.1
    backend = _FakeTorchScriptBackend(raw_action)
    policy = ThunderV4TorchScriptPolicy(
        PolicySpec(
            runtime="torchscript",
            artifact="controller.pt",
            manifest="policy_manifest.json",
        ),
        tmp_path,
        backend=backend,
    )
    observation = np.arange(57, dtype=np.float64)

    action = policy.infer(observation)

    assert backend.observations[0].shape == (1, 57)
    assert backend.observations[0].dtype == np.float32
    np.testing.assert_allclose(action, _STANDING_POSE)


def test_policy_scales_raw_action_after_startup_hold(tmp_path: Path) -> None:
    artifact = tmp_path / "controller.pt"
    artifact.write_bytes(b"scaled-policy")
    raw_action = np.arange(16, dtype=np.float32) * 0.1
    backend = _FakeTorchScriptBackend(raw_action)
    policy = ThunderV4TorchScriptPolicy(
        PolicySpec(
            runtime="torchscript",
            artifact="controller.pt",
            manifest="policy_manifest.json",
        ),
        tmp_path,
        backend=backend,
    )
    observation = np.zeros(57, dtype=np.float32)
    for _ in range(10):
        np.testing.assert_allclose(policy.infer(observation), _STANDING_POSE)

    action = policy.infer(observation)

    action_scale = np.asarray(
        (0.125, 0.25, 0.25) * 4 + (5.0, 5.0, 5.0, 5.0),
        dtype=np.float64,
    )
    np.testing.assert_allclose(action, _STANDING_POSE + raw_action * action_scale)


def test_policy_1119_stacks_five_observation_frames_and_scales_actions(tmp_path: Path) -> None:
    artifact = tmp_path / "policy_1119.onnx"
    artifact.write_bytes(b"fake-onnx-policy")
    raw_action = (np.arange(16, dtype=np.float32) * 0.1).reshape(1, 16)
    session = _FakeOnnxSession(raw_action)
    policy = ThunderV4OnnxPolicy(
        PolicySpec(
            runtime="onnxruntime",
            artifact="policy_1119.onnx",
            manifest="policy_manifest.json",
        ),
        tmp_path,
        session=session,
    )
    first = np.arange(57, dtype=np.float32)
    second = first + 100.0

    action = policy.infer(first)
    policy.infer(second)

    assert session.observations[0].shape == (1, 285)
    assert session.observations[0].dtype == np.float32
    idle_first = first.copy()
    idle_first[6:9] = 0.0
    expected_first_history = np.concatenate((np.tile(idle_first, 4), first)).reshape(1, 285)
    np.testing.assert_allclose(session.observations[0], expected_first_history)
    expected_second_history = np.concatenate((np.tile(idle_first, 3), first, second)).reshape(1, 285)
    np.testing.assert_allclose(session.observations[1], expected_second_history)
    action_scale = np.asarray(
        (0.125, 0.25, 0.25) * 4 + (5.0, 5.0, 5.0, 5.0),
        dtype=np.float64,
    )
    np.testing.assert_allclose(action, _STANDING_POSE)
    for _ in range(23):
        np.testing.assert_allclose(policy.infer(second), _STANDING_POSE)
    np.testing.assert_allclose(
        policy.infer(second),
        _STANDING_POSE + raw_action.reshape(-1) * action_scale,
    )


def test_adapter_converts_dart_action_to_plan_ordered_named_pd_torques() -> None:
    plan_layout = ActuatorLayout(_MUJOCO_ACTUATOR_CHANNELS)
    adapter = ThunderV4ControllerAdapter(_controller_spec(actuators=_MUJOCO_ACTUATOR_CHANNELS))
    target_dart = _STANDING_POSE.copy()
    target_dart[12:] = (20.0, -20.0, 5.0, -5.0)
    position_dart = _STANDING_POSE.copy()
    position_dart[0] -= 3.0
    position_dart[1] -= 1.0
    velocity_dart = np.zeros(16, dtype=np.float64)
    velocity_dart[1] = 2.0
    velocity_dart[14:] = (10.0, -10.0)
    position_by_channel = dict(zip(_ACTUATOR_CHANNELS, position_dart))
    velocity_by_channel = dict(zip(_ACTUATOR_CHANNELS, velocity_dart))
    position_plan = np.asarray([position_by_channel[channel] for channel in plan_layout.channels])
    velocity_plan = np.asarray([velocity_by_channel[channel] for channel in plan_layout.channels])

    torques = adapter.actuate(
        _state(joint_position=position_plan, joint_velocity=velocity_plan),
        target_dart,
        plan_layout,
    )

    expected_dart = np.zeros(16, dtype=np.float64)
    expected_dart[0] = 120.0
    expected_dart[1] = 70.0
    expected_dart[12:] = (17.0, -17.0, -5.0, 5.0)
    expected_by_channel = dict(zip(_ACTUATOR_CHANNELS, expected_dart))
    assert tuple(torques) == plan_layout.channels
    assert torques == pytest.approx({channel: expected_by_channel[channel] for channel in plan_layout.channels})


def test_adapter_safe_stop_holds_standing_pose_and_brakes_wheels_in_plan_order() -> None:
    plan_layout = ActuatorLayout(_MUJOCO_ACTUATOR_CHANNELS)
    adapter = ThunderV4ControllerAdapter(_controller_spec(actuators=_MUJOCO_ACTUATOR_CHANNELS))
    position_dart = _STANDING_POSE.copy()
    position_dart[0] += 0.4
    position_dart[1] -= 0.1
    velocity_dart = np.zeros(16, dtype=np.float64)
    velocity_dart[1] = 0.4
    velocity_dart[12:] = (3.0, -4.0, 5.0, -6.0)
    position_by_channel = dict(zip(_ACTUATOR_CHANNELS, position_dart))
    velocity_by_channel = dict(zip(_ACTUATOR_CHANNELS, velocity_dart))
    position_plan = np.asarray([position_by_channel[channel] for channel in plan_layout.channels])
    velocity_plan = np.asarray([velocity_by_channel[channel] for channel in plan_layout.channels])

    safe_torques = adapter.safe_stop(
        _state(
            joint_position=position_plan,
            joint_velocity=velocity_plan,
        ),
        plan_layout,
    )

    expected_dart = np.zeros(16, dtype=np.float64)
    expected_dart[0] = -28.0
    expected_dart[1] = 4.0
    expected_dart[12:] = (-3.0, 4.0, -5.0, 6.0)
    expected_by_channel = dict(zip(_ACTUATOR_CHANNELS, expected_dart))
    assert tuple(safe_torques) == plan_layout.channels
    assert safe_torques == pytest.approx(
        {channel: expected_by_channel[channel] for channel in plan_layout.channels}
    )


def test_public_component_factory_has_runtime_signature_and_shared_policy_memory(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    artifact = tmp_path / "artifacts" / "controller.pt"
    artifact.parent.mkdir()
    artifact.write_bytes(b"factory-policy")
    raw_action = np.arange(16, dtype=np.float32) * 0.2
    backend = _FakeTorchScriptBackend(raw_action)
    monkeypatch.setattr(
        thunderv4_module,
        "_DefaultTorchScriptBackend",
        lambda: backend,
    )
    controller = replace(
        _controller_spec(),
        policy=PolicySpec(
            runtime="torchscript",
            artifact="artifacts/controller.pt",
            manifest="artifacts/policy_manifest.json",
        ),
    )

    adapter, policy = create_thunderv4_components(controller, tmp_path)
    policy.infer(np.zeros(57, dtype=np.float32))
    next_observation = adapter.observe(
        _state(
            joint_position=_STANDING_POSE,
            joint_velocity=np.zeros(16, dtype=np.float64),
        ),
        _command(),
        controller.actuators,
    )

    assert tuple(inspect.signature(create_thunderv4_components).parameters) == (
        "controller",
        "repo_root",
    )
    np.testing.assert_allclose(next_observation[-16:], raw_action)


def test_default_torchscript_backend_reports_a_missing_runtime_cleanly(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(sys.modules, "torch", None)

    with pytest.raises(
        ControllerRuntimeError,
        match="production controller requires PyTorch",
    ):
        thunderv4_module._DefaultTorchScriptBackend()
