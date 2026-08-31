"""Session-level orchestration for plan-declared controller instances."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

from .contracts import (
    ActuatorCommand,
    CommandSubmitResult,
    ControllerAdapter,
    ControllerCommand,
    ControllerPolicy,
    ControllerRuntimeError,
    GenerationStamp,
)
from .physics import ControllerPhysicsBridge, PhysicsActuatorHost
from .plan import ControllerSpec, ControlPlan
from .runtime import ControllerRuntime

ControllerComponentFactory = Callable[
    [ControllerSpec, Path], tuple[ControllerAdapter, ControllerPolicy]
]


@dataclass(frozen=True)
class _ControllerBinding:
    runtime: ControllerRuntime
    bridge: ControllerPhysicsBridge


def _generation(event: Mapping[str, Any]) -> GenerationStamp:
    try:
        return GenerationStamp(
            model_generation=event["model_generation"],
            reset_generation=event["reset_generation"],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ControllerRuntimeError(
            "physics event does not contain a valid model/reset generation"
        ) from exc


class SessionControlRuntime:
    """Own all Controller Runtime instances for one resolved simulation session."""

    def __init__(
        self,
        *,
        plan: ControlPlan,
        attach_roots: Mapping[str, str],
        physics_host: PhysicsActuatorHost,
        component_factory: ControllerComponentFactory,
        repo_root: Path,
    ) -> None:
        if not isinstance(plan, ControlPlan):
            raise TypeError("plan must be a validated ControlPlan")
        self._plan = plan
        self._attach_roots = dict(attach_roots)
        self._host = physics_host
        self._factory = component_factory
        self._repo_root = Path(repo_root).resolve()
        self._bindings: dict[str, _ControllerBinding] = {}

    @property
    def controller_ids(self) -> tuple[str, ...]:
        """Return controller IDs in their compiled plan order."""

        return tuple(controller.controller_id for controller in self._plan.controllers)

    def prepare(self, ready_event: Mapping[str, Any]) -> None:
        """Create components and resolve all actuator names while physics is paused."""

        if self._bindings:
            raise ControllerRuntimeError("session controllers are already prepared")
        generation = _generation(ready_event)
        start_time_ns = ready_event.get("sim_time_ns")
        if (
            isinstance(start_time_ns, bool)
            or not isinstance(start_time_ns, int)
            or start_time_ns < 0
        ):
            raise ControllerRuntimeError("physics READY event has invalid sim_time_ns")
        staged: dict[str, _ControllerBinding] = {}
        for controller in self._plan.controllers:
            try:
                attach_root = self._attach_roots[controller.instance_id]
            except KeyError as exc:
                raise ControllerRuntimeError(
                    f"controller {controller.controller_id!r} references a robot without a PhysicsPlan"
                ) from exc
            adapter, policy = self._factory(controller, self._repo_root)
            runtime = ControllerRuntime(
                plan=self._plan,
                controller_id=controller.controller_id,
                generation=generation,
                adapter=adapter,
                policy=policy,
                start_time_ns=start_time_ns,
            )
            bridge = ControllerPhysicsBridge(
                plan=self._plan,
                controller_id=controller.controller_id,
                attach_root=attach_root,
                host=self._host,
            )
            bridge.bind()
            staged[controller.controller_id] = _ControllerBinding(runtime, bridge)
        self._bindings = staged

    def submit_command(
        self, controller_id: str, command: ControllerCommand
    ) -> CommandSubmitResult:
        """Route one typed command to exactly one controller instance."""

        try:
            binding = self._bindings[controller_id]
        except KeyError as exc:
            raise ControllerRuntimeError(
                f"controller {controller_id!r} is not prepared"
            ) from exc
        return binding.runtime.submit_command(command)

    def hold(self) -> None:
        """Clear active commands and actions for every prepared controller."""

        for binding in self._bindings.values():
            binding.runtime.hold()

    def step(self, snapshot: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
        """Run every controller once against the same immutable physics snapshot."""

        events: list[Mapping[str, Any]] = []
        for binding, command in self._commands_for_snapshot(snapshot):
            events.append(binding.bridge.apply(command))
        return tuple(events)

    def step_and_advance_sampled(
        self,
        snapshot: Mapping[str, Any],
        steps: int,
    ) -> tuple[Mapping[str, Any], ...]:
        """Run control and advance sampled Physics with the narrowest safe path."""

        commands = self._commands_for_snapshot(snapshot)
        fused_advance = getattr(self._host, "advance_sampled_with_actuator", None)
        if len(commands) == 1 and callable(fused_advance):
            return tuple(fused_advance(commands[0][1], steps))
        for binding, command in commands:
            binding.bridge.apply(command)
        sampled_advance = getattr(self._host, "advance_sampled", None)
        if not callable(sampled_advance):
            raise ControllerRuntimeError(
                "Physics Runtime does not support sampled realtime advance"
            )
        return tuple(sampled_advance(steps))

    def _commands_for_snapshot(
        self,
        snapshot: Mapping[str, Any],
    ) -> tuple[tuple[_ControllerBinding, ActuatorCommand], ...]:
        commands: list[tuple[_ControllerBinding, ActuatorCommand]] = []
        for controller in self._plan.controllers:
            binding = self._bindings[controller.controller_id]
            state = binding.bridge.project_state(snapshot)
            result = binding.runtime.step(state)
            if result.actuator_command is not None:
                commands.append((binding, result.actuator_command))
        return tuple(commands)

    def set_generation(self, event: Mapping[str, Any]) -> None:
        """Invalidate all controller state after a Physics reset or model switch."""

        generation = _generation(event)
        start_time_ns = event.get("sim_time_ns")
        if (
            isinstance(start_time_ns, bool)
            or not isinstance(start_time_ns, int)
            or start_time_ns < 0
        ):
            raise ControllerRuntimeError("physics generation event has invalid sim_time_ns")
        for binding in self._bindings.values():
            binding.runtime.set_generation(generation, start_time_ns=start_time_ns)
