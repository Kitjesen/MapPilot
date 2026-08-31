"""Plan-driven ThunderV4 ``quadruped_him`` controller components."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any, Mapping, Protocol, TypeAlias, cast

import numpy as np
from numpy.typing import NDArray

from .contracts import (
    ControllerAdapter,
    ControllerCommand,
    ControllerPolicy,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
)
from .plan import ActuatorLayout, ControllerSpec, PolicySpec

DART_ACTUATOR_ORDER = (
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
MUJOCO_ACTUATOR_ORDER = (
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
MJ_TO_DART_INDICES = (0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15)
DART_TO_MJ_INDICES = (0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15)
STANDING_POSE_DART = (
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
)
ACTION_SCALE_DART = (
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    0.125,
    0.25,
    0.25,
    5.0,
    5.0,
    5.0,
    5.0,
)
KP_DART = (70.0, 100.0, 120.0) * 4 + (0.0,) * 4
KD_DART = (15.0, 15.0, 20.0) * 4 + (1.0,) * 4
TORQUE_LIMIT_DART = (120.0,) * 12 + (17.0,) * 4

_OBSERVATION_DIM = 57
_HISTORY_FRAMES = 5
_HISTORY_OBSERVATION_DIM = _OBSERVATION_DIM * _HISTORY_FRAMES
_ACTION_DIM = 16
_GYROSCOPE_SCALE = 0.25
_JOINT_VELOCITY_SCALE = 0.05
_STARTUP_HOLD_STEPS = 10
_POLICY_1119_STARTUP_HOLD_STEPS = 25
_STANDING_POSE = np.asarray(STANDING_POSE_DART, dtype=np.float64)
_ACTION_SCALE = np.asarray(ACTION_SCALE_DART, dtype=np.float64)
_KP = np.asarray(KP_DART, dtype=np.float64)
_KD = np.asarray(KD_DART, dtype=np.float64)
_TORQUE_LIMIT = np.asarray(TORQUE_LIMIT_DART, dtype=np.float64)
_DART_INDEX = {channel: index for index, channel in enumerate(DART_ACTUATOR_ORDER)}
Float32Array: TypeAlias = NDArray[np.float32]
Float64Array: TypeAlias = NDArray[np.float64]
_REQUIRED_STATE_CHANNELS = frozenset(
    {
        "joint_position",
        "joint_velocity",
        "base_angular_velocity",
        "projected_gravity",
    }
)


class _PolicyMemory:
    def __init__(self) -> None:
        self.last_action = np.zeros(len(DART_ACTUATOR_ORDER), dtype=np.float64)

    def reset(self) -> None:
        self.last_action.fill(0.0)


def _finite_vector(value: Any, size: int, field: str) -> Float64Array:
    try:
        vector = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ControllerRuntimeError(f"{field} must be a numeric vector") from exc
    if vector.size != size:
        raise ControllerRuntimeError(f"{field} must contain exactly {size} values")
    if not np.isfinite(vector).all():
        raise ControllerRuntimeError(f"{field} must contain only finite values")
    return cast(Float64Array, vector)


class ThunderV4ControllerAdapter:
    """Convert typed runtime inputs to the ThunderV4 ``quadruped_him`` ABI."""

    def __init__(
        self,
        controller: ControllerSpec,
        *,
        _memory: _PolicyMemory | None = None,
    ) -> None:
        if not isinstance(controller, ControllerSpec):
            raise TypeError("controller must be a compiled ControllerSpec")
        if controller.adapter.plugin != "quadruped_him":
            raise ControllerRuntimeError(
                f"ThunderV4 adapter plugin must be 'quadruped_him', got {controller.adapter.plugin!r}"
            )
        if controller.adapter.abi != "lingtu.sim.controller-adapter.v1":
            raise ControllerRuntimeError(f"unsupported ThunderV4 adapter ABI {controller.adapter.abi!r}")
        missing_state = _REQUIRED_STATE_CHANNELS - set(controller.state_channels)
        if missing_state:
            raise ControllerRuntimeError(f"ThunderV4 controller is missing state channels {sorted(missing_state)!r}")
        actual_actuators = set(controller.actuators.channels)
        expected_actuators = set(DART_ACTUATOR_ORDER)
        if actual_actuators != expected_actuators:
            raise ControllerRuntimeError(
                "ThunderV4 actuator channels do not match the quadruped_him contract; "
                f"missing={sorted(expected_actuators - actual_actuators)!r}, "
                f"extra={sorted(actual_actuators - expected_actuators)!r}"
            )
        self._controller = controller
        self._actuators = controller.actuators.channels
        self._plan_to_dart = np.asarray(
            [controller.actuators.index_of(channel) for channel in DART_ACTUATOR_ORDER],
            dtype=np.int64,
        )
        self._memory = _memory or _PolicyMemory()

    def observe(
        self,
        state: ControllerState,
        command: ControllerCommand,
        actuators: ActuatorLayout,
    ) -> np.ndarray:
        """Build one deterministic 57-D ``quadruped_him`` observation."""

        self._validate_actuators(actuators)
        joint_position_plan = _finite_vector(
            state.channels.get("joint_position"), len(self._actuators), "joint_position"
        )
        joint_velocity_plan = _finite_vector(
            state.channels.get("joint_velocity"), len(self._actuators), "joint_velocity"
        )
        joint_position = joint_position_plan[self._plan_to_dart]
        joint_velocity = joint_velocity_plan[self._plan_to_dart]
        position_observation = joint_position - _STANDING_POSE
        position_observation[12:] = 0.0
        direction = self._command_direction(command.payload)
        observation: np.ndarray = np.concatenate(
            (
                _finite_vector(
                    state.channels.get("base_angular_velocity"),
                    3,
                    "base_angular_velocity",
                )
                * _GYROSCOPE_SCALE,
                _finite_vector(state.channels.get("projected_gravity"), 3, "projected_gravity"),
                direction,
                position_observation,
                joint_velocity * _JOINT_VELOCITY_SCALE,
                self._memory.last_action,
            )
        ).astype(np.float32)
        if observation.size != _OBSERVATION_DIM:
            raise ControllerRuntimeError(f"quadruped_him observation must contain {_OBSERVATION_DIM} values")
        return observation

    def actuate(
        self,
        state: ControllerState,
        action: Any,
        actuators: ActuatorLayout,
    ) -> Mapping[str, float]:
        """Convert one Dart-ordered target to plan-keyed joint torques."""

        self._validate_actuators(actuators)
        target = _finite_vector(action, _ACTION_DIM, "ThunderV4 policy action")
        joint_position_plan = _finite_vector(
            state.channels.get("joint_position"), len(self._actuators), "joint_position"
        )
        joint_velocity_plan = _finite_vector(
            state.channels.get("joint_velocity"), len(self._actuators), "joint_velocity"
        )
        joint_position = joint_position_plan[self._plan_to_dart]
        joint_velocity = joint_velocity_plan[self._plan_to_dart]

        torque = np.empty(_ACTION_DIM, dtype=np.float64)
        position_control = _KP > 0.0
        torque[position_control] = (
            _KP[position_control] * (target[position_control] - joint_position[position_control])
            - _KD[position_control] * joint_velocity[position_control]
        )
        velocity_control = ~position_control
        torque[velocity_control] = _KD[velocity_control] * (target[velocity_control] - joint_velocity[velocity_control])
        torque = np.clip(torque, -_TORQUE_LIMIT, _TORQUE_LIMIT)
        return {channel: float(torque[_DART_INDEX[channel]]) for channel in actuators.channels}

    def safe_stop(
        self,
        state: ControllerState,
        actuators: ActuatorLayout,
    ) -> Mapping[str, float]:
        """Hold the standing pose while braking all four wheel joints."""

        return self.actuate(state, _STANDING_POSE, actuators)

    def reset(self, generation: GenerationStamp) -> None:
        """Clear the previous raw policy action after a generation switch."""

        del generation
        self._memory.reset()

    def _validate_actuators(self, actuators: ActuatorLayout) -> None:
        if not isinstance(actuators, ActuatorLayout) or actuators.channels != self._actuators:
            raise ControllerRuntimeError("runtime actuator layout does not match the compiled ThunderV4 controller")

    @staticmethod
    def _command_direction(payload: Any) -> Float64Array:
        if not isinstance(payload, Mapping):
            raise ControllerRuntimeError("base_twist command payload must be a mapping")
        direction = tuple(payload.get(field, 0.0) for field in ("linear_x", "linear_y", "angular_z"))
        return _finite_vector(direction, 3, "base_twist command").copy()


class _TorchScriptBackend(Protocol):
    def load(self, path: Path) -> Any: ...

    def infer(self, model: Any, observation: Float32Array) -> Any: ...


class _DefaultTorchScriptBackend:
    def __init__(self) -> None:
        try:
            import torch
        except (ImportError, OSError) as exc:
            raise ControllerRuntimeError(
                "ThunderV4 production controller requires PyTorch in the selected "
                "RuntimeProfile Python environment"
            ) from exc

        self._torch = torch

    def load(self, path: Path) -> Any:
        return self._torch.jit.load(str(path), map_location="cpu")

    def infer(self, model: Any, observation: Float32Array) -> Any:
        with self._torch.inference_mode():
            tensor = self._torch.from_numpy(observation).to(dtype=self._torch.float32)
            result = model(tensor)
        try:
            return result.detach().cpu().numpy()
        except AttributeError as exc:
            raise ControllerRuntimeError("TorchScript policy output must be a tensor") from exc


def _resolve_policy_artifact(policy: PolicySpec, repo_root: Path) -> Path:
    root = Path(repo_root).resolve()
    artifact = Path(policy.artifact)
    if artifact.is_absolute():
        raise ControllerRuntimeError("policy artifact must be relative to repo_root")
    artifact_path = (root / artifact).resolve()
    try:
        artifact_path.relative_to(root)
    except ValueError as exc:
        raise ControllerRuntimeError("policy artifact escapes repo_root") from exc
    if not artifact_path.is_file():
        raise ControllerRuntimeError(f"policy artifact is not a file: {artifact_path}")
    return artifact_path


class ThunderV4TorchScriptPolicy:
    """Verified CPU TorchScript policy selected by a compiled ``PolicySpec``."""

    def __init__(
        self,
        policy: PolicySpec,
        repo_root: Path,
        *,
        backend: _TorchScriptBackend | None = None,
        _memory: _PolicyMemory | None = None,
    ) -> None:
        if not isinstance(policy, PolicySpec):
            raise TypeError("policy must be a compiled PolicySpec")
        if policy.runtime != "torchscript":
            raise ControllerRuntimeError(f"ThunderV4 policy runtime must be 'torchscript', got {policy.runtime!r}")
        self._artifact_path = _resolve_policy_artifact(policy, repo_root)
        self._backend = backend or _DefaultTorchScriptBackend()
        self._model = self._backend.load(self._artifact_path)
        eval_model = getattr(self._model, "eval", None)
        if not callable(eval_model):
            raise ControllerRuntimeError("TorchScript policy model does not provide eval()")
        eval_model()
        self._memory = _memory or _PolicyMemory()
        self._startup_hold_remaining = _STARTUP_HOLD_STEPS

    @property
    def artifact_path(self) -> Path:
        """Return the verified absolute artifact path loaded by this policy."""

        return self._artifact_path

    def infer(self, observation: Any) -> Float64Array:
        """Run one CPU TorchScript inference and return a Dart-ordered target."""

        observation_batch = (
            _finite_vector(observation, _OBSERVATION_DIM, "quadruped_him observation")
            .astype(np.float32, copy=False)
            .reshape(1, _OBSERVATION_DIM)
        )
        raw_action = _finite_vector(
            self._backend.infer(self._model, observation_batch),
            _ACTION_DIM,
            "TorchScript policy action",
        )
        self._memory.last_action = raw_action.copy()
        if self._startup_hold_remaining > 0:
            self._startup_hold_remaining -= 1
            return cast(Float64Array, _STANDING_POSE.copy())
        return cast(Float64Array, raw_action * _ACTION_SCALE + _STANDING_POSE)

    def reset(self, generation: GenerationStamp) -> None:
        """Clear policy memory and restore the startup standing hold."""

        del generation
        self._memory.reset()
        self._startup_hold_remaining = _STARTUP_HOLD_STEPS


class ThunderV4OnnxPolicy:
    """Brainstem policy_1119 ONNX runtime with five 57-D history frames."""

    def __init__(
        self,
        policy: PolicySpec,
        repo_root: Path,
        *,
        session: Any | None = None,
        _memory: _PolicyMemory | None = None,
    ) -> None:
        if not isinstance(policy, PolicySpec):
            raise TypeError("policy must be a compiled PolicySpec")
        if policy.runtime != "onnxruntime":
            raise ControllerRuntimeError(
                f"ThunderV4 policy runtime must be 'onnxruntime', got {policy.runtime!r}"
            )
        self._artifact_path = _resolve_policy_artifact(policy, repo_root)
        if session is None:
            try:
                import onnxruntime as ort
            except (ImportError, OSError) as exc:
                raise ControllerRuntimeError(
                    "ThunderV4 ONNX controller requires onnxruntime in the selected RuntimeProfile"
                ) from exc
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            session = ort.InferenceSession(
                str(self._artifact_path),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if len(inputs) != 1 or tuple(inputs[0].shape) != (1, _HISTORY_OBSERVATION_DIM):
            shape = None if len(inputs) != 1 else tuple(inputs[0].shape)
            raise ControllerRuntimeError(
                f"policy_1119 input must be [1, {_HISTORY_OBSERVATION_DIM}], got {shape!r}"
            )
        if len(outputs) != 1 or tuple(outputs[0].shape) != (1, _ACTION_DIM):
            shape = None if len(outputs) != 1 else tuple(outputs[0].shape)
            raise ControllerRuntimeError(
                f"policy_1119 output must be [1, {_ACTION_DIM}], got {shape!r}"
            )
        self._session = session
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        self._history: deque[Float32Array] = deque(maxlen=_HISTORY_FRAMES)
        self._memory = _memory or _PolicyMemory()
        self._startup_hold_remaining = _POLICY_1119_STARTUP_HOLD_STEPS

    @property
    def artifact_path(self) -> Path:
        return self._artifact_path

    def infer(self, observation: Any) -> Float64Array:
        frame = (
            _finite_vector(observation, _OBSERVATION_DIM, "quadruped_him observation")
            .astype(np.float32, copy=False)
        )
        if not self._history:
            idle_frame = frame.copy()
            idle_frame[6:9] = 0.0
            for _ in range(_HISTORY_FRAMES - 1):
                self._history.append(idle_frame.copy())
        self._history.append(frame.copy())
        history = np.concatenate(tuple(self._history)).reshape(1, _HISTORY_OBSERVATION_DIM)
        raw_action = _finite_vector(
            self._session.run(
                [self._output_name],
                {self._input_name: history.astype(np.float32, copy=False)},
            )[0],
            _ACTION_DIM,
            "ONNX policy action",
        )
        self._memory.last_action = raw_action.copy()
        if self._startup_hold_remaining > 0:
            self._startup_hold_remaining -= 1
            return cast(Float64Array, _STANDING_POSE.copy())
        return cast(Float64Array, raw_action * _ACTION_SCALE + _STANDING_POSE)

    def reset(self, generation: GenerationStamp) -> None:
        del generation
        self._history.clear()
        self._memory.reset()
        self._startup_hold_remaining = _POLICY_1119_STARTUP_HOLD_STEPS


def create_thunderv4_components(
    controller: ControllerSpec,
    repo_root: Path,
) -> tuple[ControllerAdapter, ControllerPolicy]:
    """Build the components required by ``ControllerComponentFactory``."""

    memory = _PolicyMemory()
    adapter = ThunderV4ControllerAdapter(controller, _memory=memory)
    policy = ThunderV4TorchScriptPolicy(
        controller.policy,
        repo_root,
        _memory=memory,
    )
    return adapter, policy


def create_thunderv4_onnx_components(
    controller: ControllerSpec,
    repo_root: Path,
) -> tuple[ControllerAdapter, ControllerPolicy]:
    """Build the policy_1119 ONNX adapter and policy with shared action memory."""

    memory = _PolicyMemory()
    adapter = ThunderV4ControllerAdapter(controller, _memory=memory)
    policy = ThunderV4OnnxPolicy(
        controller.policy,
        repo_root,
        _memory=memory,
    )
    return adapter, policy
