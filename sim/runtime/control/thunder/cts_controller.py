"""Plan-driven bridge to the exported Thunder CTS MoE training contract."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from ..contracts import (
    ControllerAdapter,
    ControllerCommand,
    ControllerPolicy,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
)
from ..plan import ActuatorLayout, ControllerSpec
from .cts_policy import CtsPolicy
from .legacy_controller import (
    DART_ACTUATOR_ORDER,
    MUJOCO_ACTUATOR_ORDER,
    ThunderV4ControllerAdapter,
    _finite_vector,
    _resolve_policy_artifact,
)


class ThunderCtsController:
    """Share the existing CTS runner between observation, inference and torque output."""

    def __init__(self, controller: ControllerSpec, repo_root: Path, *, session: Any = None) -> None:
        if not isinstance(controller, ControllerSpec):
            raise TypeError("controller must be a compiled ControllerSpec")
        if (controller.adapter.abi, controller.adapter.plugin, controller.policy.runtime) != (
            "lingtu.sim.controller-adapter.v1", "thunderv4_cts_moe", "onnxruntime",
        ):
            raise ControllerRuntimeError("Thunder CTS requires the thunderv4_cts_moe ONNX controller contract")
        required = {"joint_position", "joint_velocity", "base_angular_velocity", "projected_gravity"}
        if missing := required - set(controller.state_channels):
            raise ControllerRuntimeError(f"Thunder CTS is missing state channels {sorted(missing)!r}")
        if set(controller.actuators.channels) != set(DART_ACTUATOR_ORDER):
            raise ControllerRuntimeError("Thunder CTS actuator channels must match the sixteen Thunder joints")
        self.artifact_path = _resolve_policy_artifact(controller.policy, repo_root)
        from sim.runtime.control.policy_loader import load_policy

        try:
            self.runner = load_policy(str(self.artifact_path), cpu_threads=1, session=session)
        except (ImportError, OSError) as exc:
            raise ControllerRuntimeError("Thunder CTS requires onnxruntime in the selected RuntimeProfile") from exc
        except ValueError as exc:
            raise ControllerRuntimeError(f"Thunder CTS policy load failed: {exc}") from exc
        if not isinstance(self.runner, CtsPolicy):
            raise ControllerRuntimeError("Thunder CTS ONNX metadata must declare 'thunder_cts_moe_v1'")
        for name, interval in (("inference_hz", "dt"), ("low_level_hz", "physics_dt")):
            if not math.isclose(getattr(controller, name), 1.0 / self.runner.contract[interval]):
                raise ControllerRuntimeError(f"Thunder CTS {name} must match ONNX metadata {interval}")
        self._actuators = controller.actuators
        self._plan_to_mj = np.array([controller.actuators.index_of(name) for name in MUJOCO_ACTUATOR_ORDER])
        self._plan_to_dart = np.array([controller.actuators.index_of(name) for name in DART_ACTUATOR_ORDER])
        self._dart_index = {name: index for index, name in enumerate(DART_ACTUATOR_ORDER)}
        self._idle = True

    def _joint_state(self, state: ControllerState, actuators: ActuatorLayout) -> tuple[np.ndarray, np.ndarray]:
        if actuators != self._actuators:
            raise ControllerRuntimeError("runtime actuator layout does not match the compiled Thunder CTS controller")
        return (
            _finite_vector(state.channels.get("joint_position"), 16, "joint_position"),
            _finite_vector(state.channels.get("joint_velocity"), 16, "joint_velocity"),
        )

    def observe(self, state: ControllerState, command: ControllerCommand, actuators: ActuatorLayout) -> np.ndarray:
        position, velocity = self._joint_state(state, actuators)
        direction = ThunderV4ControllerAdapter._command_direction(command.payload)
        self._idle = bool(np.linalg.norm(direction) <= 1e-4)
        return self.runner.build_obs(
            _finite_vector(state.channels.get("base_angular_velocity"), 3, "base_angular_velocity"),
            _finite_vector(state.channels.get("projected_gravity"), 3, "projected_gravity"),
            direction, position[self._plan_to_mj], velocity[self._plan_to_mj],
        )

    def infer(self, observation: Any) -> np.ndarray:
        targets = self.runner.infer(_finite_vector(observation, 53, "Thunder CTS observation"))
        if self._idle:
            targets[12:] = 0.0
        return targets

    def actuate(self, state: ControllerState, action: Any, actuators: ActuatorLayout) -> dict[str, float]:
        position, velocity = self._joint_state(state, actuators)
        position, velocity = position[self._plan_to_dart], velocity[self._plan_to_dart]
        target = _finite_vector(action, 16, "Thunder CTS target")
        runner = self.runner
        torque = np.empty(16)
        position_control = runner.kp > 0.0
        torque[position_control] = (
            runner.kp[position_control] * (target[position_control] - position[position_control])
            - runner.kd[position_control] * velocity[position_control]
        )
        torque[~position_control] = runner.kd[~position_control] * (target[~position_control] - velocity[~position_control])
        torque = runner.clip_torques(torque, velocity)
        return {name: float(torque[self._dart_index[name]]) for name in actuators.channels}

    def safe_stop(self, state: ControllerState, actuators: ActuatorLayout) -> dict[str, float]:
        self.runner.reset()
        targets = self.runner.default.copy()
        targets[12:] = 0.0
        return self.actuate(state, targets, actuators)

    def reset(self, generation: GenerationStamp) -> None:
        del generation
        self.runner.reset()
        self._idle = True


def create_thunderv4_cts_components(
    controller: ControllerSpec, repo_root: Path,
) -> tuple[ControllerAdapter, ControllerPolicy]:
    """Use one CTS runner so adapter and policy reset the same temporal state."""
    component = ThunderCtsController(controller, repo_root)
    return component, component
