"""In-process contracts for the plan-driven Controller Runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Mapping, Protocol, Sequence

if TYPE_CHECKING:
    from .plan import ActuatorLayout


class ControllerRuntimeError(RuntimeError):
    """Raised when runtime state cannot be accepted or driven safely."""


class CommandSubmitResult(Enum):
    """Explicit result of submitting a command to the active generation."""

    ACCEPTED = "accepted"
    REJECTED_INSTANCE = "rejected_instance"
    REJECTED_CHANNEL = "rejected_channel"
    REJECTED_STALE_MODEL_GENERATION = "rejected_stale_model_generation"
    REJECTED_FUTURE_MODEL_GENERATION = "rejected_future_model_generation"
    REJECTED_STALE_RESET_GENERATION = "rejected_stale_reset_generation"
    REJECTED_FUTURE_RESET_GENERATION = "rejected_future_reset_generation"
    REJECTED_OUT_OF_ORDER = "rejected_out_of_order"


class SafeStopReason(Enum):
    """Why a scheduled actuator output was replaced by a safe stop."""

    NO_COMMAND = "no_command"
    COMMAND_NOT_DUE = "command_not_due"
    STALE_COMMAND = "stale_command"
    WAITING_FOR_INFERENCE = "waiting_for_inference"


def _nonnegative_integer(value: Any, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")


def _identifier(value: Any, field_name: str) -> None:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field_name} must be a non-empty trimmed string")


@dataclass(frozen=True)
class GenerationStamp:
    """Runtime model/reset identity shared with physics snapshots and commands."""

    model_generation: int
    reset_generation: int

    def __post_init__(self) -> None:
        _nonnegative_integer(self.model_generation, "model_generation")
        _nonnegative_integer(self.reset_generation, "reset_generation")


@dataclass(frozen=True)
class ControllerCommand:
    """One generation-stamped input command for a controller instance."""

    channel_id: str
    instance_id: str
    generation: GenerationStamp
    sequence: int
    apply_time_ns: int
    payload: Any

    def __post_init__(self) -> None:
        _identifier(self.channel_id, "channel_id")
        _identifier(self.instance_id, "instance_id")
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.apply_time_ns, "apply_time_ns")
        if not isinstance(self.generation, GenerationStamp):
            raise ValueError("generation must be a GenerationStamp")


@dataclass(frozen=True)
class ControllerState:
    """Immutable state view delivered by the Physics Runtime."""

    session_id: str
    instance_id: str
    generation: GenerationStamp
    sequence: int
    sim_time_ns: int
    channels: Mapping[str, Any]

    def __post_init__(self) -> None:
        _identifier(self.session_id, "session_id")
        _identifier(self.instance_id, "instance_id")
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.sim_time_ns, "sim_time_ns")
        if not isinstance(self.generation, GenerationStamp):
            raise ValueError("generation must be a GenerationStamp")
        if not isinstance(self.channels, Mapping):
            raise ValueError("channels must be a mapping")
        copied = dict(self.channels)
        for name in copied:
            _identifier(name, "state channel name")
        object.__setattr__(self, "channels", MappingProxyType(copied))


@dataclass(frozen=True)
class ActuatorCommand:
    """One ordered, generation-stamped output destined for Physics Runtime."""

    session_id: str
    channel_id: str
    controller_id: str
    instance_id: str
    generation: GenerationStamp
    sequence: int
    apply_time_ns: int
    command_type: str
    channels: tuple[str, ...]
    values: tuple[float, ...]
    safe_stop: bool
    safe_stop_reason: SafeStopReason | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.session_id, str) or not self.session_id.strip():
            raise ValueError("session_id must be non-empty")
        _identifier(self.channel_id, "channel_id")
        _identifier(self.controller_id, "controller_id")
        _identifier(self.instance_id, "instance_id")
        _identifier(self.command_type, "command_type")
        if not isinstance(self.generation, GenerationStamp):
            raise ValueError("generation must be a GenerationStamp")
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.apply_time_ns, "apply_time_ns")
        if not isinstance(self.channels, tuple) or not self.channels:
            raise ValueError("actuator channels must be a non-empty tuple")
        for channel in self.channels:
            _identifier(channel, "actuator channel")
        if len(set(self.channels)) != len(self.channels):
            raise ValueError("actuator channels must be unique")
        if not isinstance(self.values, tuple):
            raise ValueError("actuator values must be a tuple")
        if len(self.channels) != len(self.values):
            raise ValueError("actuator channels and values must have equal length")
        if not all(
            not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
            for value in self.values
        ):
            raise ValueError("actuator values must be finite")
        if self.safe_stop_reason is not None and not isinstance(self.safe_stop_reason, SafeStopReason):
            raise ValueError("safe_stop_reason must be a SafeStopReason")
        if self.safe_stop != (self.safe_stop_reason is not None):
            raise ValueError("safe-stop flag and reason must agree")

    def value_for(self, channel: str) -> float:
        """Return a value by stable actuator-channel name."""

        return self.values[self.channels.index(channel)]


@dataclass(frozen=True)
class ControllerStep:
    """Observable result of consuming one state snapshot."""

    state_sequence: int
    sim_time_ns: int
    inference_ran: bool
    actuator_command: ActuatorCommand | None


class ControllerAdapter(Protocol):
    """Robot-specific conversion seam around a generic policy."""

    def observe(
        self,
        state: ControllerState,
        command: ControllerCommand,
        actuators: ActuatorLayout,
    ) -> Any:
        """Build one policy observation from plan-declared runtime inputs."""

    def actuate(
        self,
        state: ControllerState,
        action: Any,
        actuators: ActuatorLayout,
    ) -> Mapping[str, Any] | Sequence[Any]:
        """Convert a policy action to named or plan-ordered actuator values."""

    def safe_stop(
        self,
        state: ControllerState,
        actuators: ActuatorLayout,
    ) -> Mapping[str, Any] | Sequence[Any]:
        """Return the robot-specific safe output for every actuator."""

    def reset(self, generation: GenerationStamp) -> None:
        """Clear adapter state after a generation switch."""


class ControllerPolicy(Protocol):
    """Inference seam; TorchScript is one future implementation, not the core."""

    def infer(self, observation: Any) -> Any:
        """Produce one policy action."""

    def reset(self, generation: GenerationStamp) -> None:
        """Clear recurrent policy state after a generation switch."""
