"""Strict bridge between compiled controller contracts and Physics Runtime."""

from __future__ import annotations

import math
from typing import Any, Mapping, Protocol

from .contracts import (
    ActuatorCommand,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
)
from .plan import CommandChannelSpec, ControllerSpec, ControlPlan


class PhysicsActuatorHost(Protocol):
    """Minimal actuator surface implemented by a Physics Runtime host."""

    def bind_actuators(
        self,
        *,
        source_id: str,
        instance_id: str,
        command_type: str,
        stale_timeout_ns: int,
        channels: tuple[str, ...],
    ) -> Mapping[str, Any]:
        """Resolve one controller's stable actuator channel order."""
        ...

    def apply_actuator_command(
        self, command: ActuatorCommand
    ) -> Mapping[str, Any]:
        """Apply one generation-stamped actuator command."""
        ...

    def advance_sampled_with_actuator(
        self,
        command: ActuatorCommand,
        steps: int,
    ) -> tuple[Mapping[str, Any], ...]:
        """Apply one command and advance one sampled physics transaction."""
        ...


def _nonnegative_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ControllerRuntimeError(f"physics snapshot {field} must be a non-negative integer")
    return value


def _finite_vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ControllerRuntimeError(
            f"physics snapshot {field} must contain exactly {size} values"
        )
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ControllerRuntimeError(f"physics snapshot {field} must be numeric")
        number = float(item)
        if not math.isfinite(number):
            raise ControllerRuntimeError(f"physics snapshot {field} must be finite")
        result.append(number)
    return tuple(result)


def _world_to_body(
    quaternion_wxyz: tuple[float, float, float, float],
    vector_world: tuple[float, float, float],
) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion_wxyz))
    if norm <= 1e-12:
        raise ControllerRuntimeError("physics snapshot base quaternion has zero norm")
    w, x, y, z = (value / norm for value in quaternion_wxyz)
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )
    return tuple(
        sum(rotation[row][column] * vector_world[row] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


class ControllerPhysicsBridge:
    """Bind one controller and project immutable MuJoCo truth into its state ABI."""

    def __init__(
        self,
        *,
        plan: ControlPlan,
        controller_id: str,
        attach_root: str,
        host: PhysicsActuatorHost,
    ) -> None:
        if not isinstance(plan, ControlPlan):
            raise TypeError("plan must be a validated ControlPlan")
        try:
            controller = plan.controller(controller_id)
        except KeyError as exc:
            raise ControllerRuntimeError(
                f"controller {controller_id!r} is not declared by the plan"
            ) from exc
        if not isinstance(attach_root, str) or not attach_root or attach_root != attach_root.strip():
            raise ValueError("attach_root must be a non-empty trimmed string")
        self._plan = plan
        self._controller = controller
        self._attach_root = attach_root
        self._host = host
        self._output = self._actuator_output(plan, controller)
        self._bound = False

    @property
    def controller(self) -> ControllerSpec:
        """Return the controller contract bound by this bridge."""
        return self._controller

    def bind(self) -> Mapping[str, Any]:
        """Resolve all stable actuator names once, before physics advances."""

        if self._bound:
            raise ControllerRuntimeError(
                f"controller {self._controller.controller_id!r} is already bound"
            )
        event = self._host.bind_actuators(
            source_id=self._controller.controller_id,
            instance_id=self._controller.instance_id,
            command_type=self._output.command_type,
            stale_timeout_ns=self._plan.stale_timeout_ns,
            channels=self._controller.actuators.channels,
        )
        if event.get("event") != "actuator_bound":
            raise ControllerRuntimeError("Physics Runtime did not confirm actuator binding")
        self._bound = True
        return event

    def project_state(self, snapshot: Mapping[str, Any]) -> ControllerState:
        """Create the exact plan-declared controller state from one truth snapshot."""

        if not self._bound:
            raise ControllerRuntimeError("controller actuators must be bound before state projection")
        if not isinstance(snapshot, Mapping) or snapshot.get("event") != "snapshot":
            raise ControllerRuntimeError("controller state source must be a physics snapshot")
        if snapshot.get("session_id") != self._plan.session_id:
            raise ControllerRuntimeError("physics snapshot session_id does not match control plan")

        generation = GenerationStamp(
            model_generation=_nonnegative_integer(
                snapshot.get("model_generation"), "model_generation"
            ),
            reset_generation=_nonnegative_integer(
                snapshot.get("reset_generation"), "reset_generation"
            ),
        )
        sequence = _nonnegative_integer(snapshot.get("sequence"), "sequence")
        sim_time_ns = _nonnegative_integer(snapshot.get("sim_time_ns"), "sim_time_ns")
        joints = self._indexed_entities(snapshot.get("joints"), "joints")
        bodies = self._indexed_entities(snapshot.get("bodies"), "bodies")

        joint_positions: list[float] = []
        joint_velocities: list[float] = []
        for channel in self._controller.actuators.channels:
            stable_id = f"{self._controller.instance_id}/{channel}"
            try:
                joint = joints[stable_id]
            except KeyError as exc:
                raise ControllerRuntimeError(
                    f"physics snapshot is missing actuator joint {stable_id!r}"
                ) from exc
            position = _finite_vector(joint.get("position_rad"), 1, f"{stable_id}.position_rad")
            velocity = _finite_vector(joint.get("velocity_rps"), 1, f"{stable_id}.velocity_rps")
            joint_positions.append(position[0])
            joint_velocities.append(velocity[0])

        base_id = f"{self._controller.instance_id}/{self._attach_root}"
        try:
            base = bodies[base_id]
        except KeyError as exc:
            raise ControllerRuntimeError(
                f"physics snapshot is missing controller base frame {base_id!r}"
            ) from exc
        quaternion = _finite_vector(
            base.get("quaternion_wxyz"), 4, f"{base_id}.quaternion_wxyz"
        )
        angular_velocity_world = _finite_vector(
            base.get("angular_velocity_rps"), 3, f"{base_id}.angular_velocity_rps"
        )
        angular_velocity_body = _world_to_body(quaternion, angular_velocity_world)  # type: ignore[arg-type]
        projected_gravity = _world_to_body(quaternion, (0.0, 0.0, -1.0))  # type: ignore[arg-type]

        available: dict[str, Any] = {
            "joint_position": tuple(joint_positions),
            "joint_velocity": tuple(joint_velocities),
            "base_angular_velocity": angular_velocity_body,
            "projected_gravity": projected_gravity,
        }
        unknown = set(self._controller.state_channels) - set(available)
        if unknown:
            raise ControllerRuntimeError(
                f"controller requests unsupported physics state channels {sorted(unknown)!r}"
            )
        return ControllerState(
            session_id=self._plan.session_id,
            instance_id=self._controller.instance_id,
            generation=generation,
            sequence=sequence,
            sim_time_ns=sim_time_ns,
            channels={name: available[name] for name in self._controller.state_channels},
        )

    def apply(self, command: ActuatorCommand) -> Mapping[str, Any]:
        """Apply one output and surface every fail-closed Physics rejection."""

        if not self._bound:
            raise ControllerRuntimeError("controller actuators have not been bound")
        event = self._host.apply_actuator_command(command)
        result = event.get("result")
        if result != "applied":
            raise ControllerRuntimeError(f"Physics Runtime rejected actuator command: {result}")
        return event

    @staticmethod
    def _actuator_output(
        plan: ControlPlan, controller: ControllerSpec
    ) -> CommandChannelSpec:
        outputs = [
            plan.command_channel(channel_id)
            for channel_id in controller.command_channels
            if plan.command_channel(channel_id).direction == "publish"
            and plan.command_channel(channel_id).owner == "physics"
            and plan.command_channel(channel_id).target == "actuators"
            and plan.command_channel(channel_id).transport == "in_process"
            and plan.command_channel(channel_id).source == controller.controller_id
        ]
        if len(outputs) != 1:
            raise ControllerRuntimeError(
                f"controller {controller.controller_id!r} must declare exactly one in-process Physics actuator output"
            )
        return outputs[0]

    @staticmethod
    def _indexed_entities(raw: Any, field: str) -> dict[str, Mapping[str, Any]]:
        if not isinstance(raw, list):
            raise ControllerRuntimeError(f"physics snapshot {field} must be a list")
        result: dict[str, Mapping[str, Any]] = {}
        for index, value in enumerate(raw):
            if not isinstance(value, Mapping):
                raise ControllerRuntimeError(
                    f"physics snapshot {field}[{index}] must be an object"
                )
            stable_id = value.get("stable_id")
            if not isinstance(stable_id, str) or not stable_id:
                raise ControllerRuntimeError(
                    f"physics snapshot {field}[{index}].stable_id is invalid"
                )
            if stable_id in result:
                raise ControllerRuntimeError(
                    f"physics snapshot contains duplicate stable_id {stable_id!r}"
                )
            result[stable_id] = value
        return result
