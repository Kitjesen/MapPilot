"""Deterministic controller doubles for runtime tests and offline bring-up."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Sequence

from .contracts import ControllerCommand, ControllerState, GenerationStamp
from .plan import ActuatorLayout


@dataclass(frozen=True)
class FakeObservation:
    """Transparent observation produced by the deterministic fake adapter."""

    state: ControllerState
    command: ControllerCommand
    actuator_channels: tuple[str, ...]


class DeterministicFakeAdapter:
    """Identity-like adapter with explicit zero safe-stop behavior."""

    def observe(
        self,
        state: ControllerState,
        command: ControllerCommand,
        actuators: ActuatorLayout,
    ) -> FakeObservation:
        """Expose the inputs unchanged in a deterministic observation."""

        return FakeObservation(state, command, actuators.channels)

    def actuate(
        self,
        state: ControllerState,
        action: Any,
        actuators: ActuatorLayout,
    ) -> Mapping[str, Any] | Sequence[Any]:
        """Forward the fake policy action to the runtime's stable binder."""

        del state, actuators
        if isinstance(action, Mapping):
            named: dict[str, Any] = {}
            for channel, value in action.items():
                if not isinstance(channel, str):
                    raise ValueError("fake policy actuator names must be strings")
                named[channel] = value
            return named
        if isinstance(action, Sequence) and not isinstance(action, (str, bytes)):
            return tuple(action)
        raise ValueError("fake policy action must contain actuator values")

    def safe_stop(
        self,
        state: ControllerState,
        actuators: ActuatorLayout,
    ) -> tuple[float, ...]:
        """Return deterministic zero output for every declared actuator."""

        del state
        return (0.0,) * len(actuators.channels)

    def reset(self, generation: GenerationStamp) -> None:
        """Accept generation switches; this stateless adapter has nothing to clear."""

        del generation


class DeterministicFakePolicy:
    """Return one frozen action on every inference without loading Torch."""

    def __init__(self, action: Mapping[str, Any] | Sequence[Any]) -> None:
        frozen_action: Mapping[str, Any] | tuple[Any, ...]
        if isinstance(action, Mapping):
            frozen_action = MappingProxyType(dict(action))
        elif isinstance(action, (str, bytes)):
            raise ValueError("fake policy action must contain actuator values")
        else:
            frozen_action = tuple(action)
        self._action = frozen_action

    def infer(self, observation: Any) -> Mapping[str, Any] | tuple[Any, ...]:
        """Return the configured immutable action."""

        del observation
        return self._action

    def reset(self, generation: GenerationStamp) -> None:
        """Accept generation switches; this stateless policy has nothing to clear."""

        del generation


def zero_output_components(controller: Any, repo_root: Any) -> tuple[Any, Any]:
    """Build a deterministic torque-safe controller for smoke/visual bring-up."""

    del repo_root
    return (
        DeterministicFakeAdapter(),
        DeterministicFakePolicy((0.0,) * len(controller.actuators.channels)),
    )
