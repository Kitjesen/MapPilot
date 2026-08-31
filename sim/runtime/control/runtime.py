"""Robot-agnostic scheduling and safety core for simulation controllers."""

from __future__ import annotations

import math
from fractions import Fraction
from typing import Any

from .contracts import (
    ActuatorCommand,
    CommandSubmitResult,
    ControllerAdapter,
    ControllerCommand,
    ControllerPolicy,
    ControllerRuntimeError,
    ControllerState,
    ControllerStep,
    GenerationStamp,
    SafeStopReason,
)
from .plan import CommandChannelSpec, ControllerSpec, ControlPlan

_NANOSECONDS_PER_SECOND = 1_000_000_000


def _validate_start_time_ns(start_time_ns: int) -> None:
    if isinstance(start_time_ns, bool) or not isinstance(start_time_ns, int) or start_time_ns < 0:
        raise ValueError("start_time_ns must be a non-negative integer")


class _FixedRateSchedule:
    """Phase-stable schedule that drops, rather than replays, missed periods."""

    def __init__(self, rate_hz: float, start_time_ns: int) -> None:
        self._rate = Fraction(str(rate_hz))
        self.reset(start_time_ns)

    def reset(self, start_time_ns: int) -> None:
        _validate_start_time_ns(start_time_ns)
        self._origin_ns = start_time_ns
        self._next_tick = 0

    def consume(self, sim_time_ns: int) -> bool:
        if sim_time_ns < self._origin_ns:
            raise ControllerRuntimeError("state sim_time_ns precedes the active generation schedule")
        elapsed_ns = sim_time_ns - self._origin_ns
        last_due_tick = (elapsed_ns * self._rate.numerator) // (_NANOSECONDS_PER_SECOND * self._rate.denominator)
        if self._next_tick > last_due_tick:
            return False
        self._next_tick = last_due_tick + 1
        return True


class ControllerRuntime:
    """Execute one compiled controller against generation-stamped state."""

    def __init__(
        self,
        *,
        plan: ControlPlan,
        controller_id: str,
        generation: GenerationStamp,
        adapter: ControllerAdapter,
        policy: ControllerPolicy,
        start_time_ns: int = 0,
    ) -> None:
        if not isinstance(plan, ControlPlan):
            raise TypeError("plan must be a validated ControlPlan")
        if not isinstance(generation, GenerationStamp):
            raise TypeError("generation must be a GenerationStamp")
        try:
            controller = plan.controller(controller_id)
        except KeyError as exc:
            raise ControllerRuntimeError(f"controller {controller_id!r} is not declared by the plan") from exc
        input_channels, output_channel = self._resolve_channels(plan, controller)
        self._plan = plan
        self._controller = controller
        self._input_channels = input_channels
        self._output_channel = output_channel
        self._generation = generation
        self._adapter = adapter
        self._policy = policy
        self._inference_schedule = _FixedRateSchedule(controller.inference_hz, start_time_ns)
        self._low_level_schedule = _FixedRateSchedule(controller.low_level_hz, start_time_ns)
        self._latest_command: ControllerCommand | None = None
        self._command_watermark: tuple[int, int] | None = None
        self._last_action: Any = None
        self._last_state_sequence: int | None = None
        self._last_sim_time_ns: int | None = None
        self._output_sequence = 0

    @property
    def generation(self) -> GenerationStamp:
        """Return the generation currently accepted by the runtime."""

        return self._generation

    @property
    def controller(self) -> ControllerSpec:
        """Return the selected immutable controller declaration."""

        return self._controller

    def submit_command(self, command: ControllerCommand) -> CommandSubmitResult:
        """Accept only current-generation commands for this controller input."""

        if command.instance_id != self._controller.instance_id:
            return CommandSubmitResult.REJECTED_INSTANCE
        if command.channel_id not in self._input_channels:
            return CommandSubmitResult.REJECTED_CHANNEL
        generation_result = self._command_generation_result(command.generation)
        if generation_result is not CommandSubmitResult.ACCEPTED:
            return generation_result
        if self._command_watermark is not None:
            sequence, apply_time_ns = self._command_watermark
            if (
                command.sequence <= sequence
                or command.apply_time_ns < apply_time_ns
            ):
                return CommandSubmitResult.REJECTED_OUT_OF_ORDER
        self._latest_command = command
        self._command_watermark = (command.sequence, command.apply_time_ns)
        return CommandSubmitResult.ACCEPTED

    def hold(self) -> None:
        """Clear active control state until a fresh command is submitted."""

        self._latest_command = None
        self._last_action = None

    def step(self, state: ControllerState) -> ControllerStep:
        """Consume one immutable state snapshot and run all currently due work."""

        self._validate_state(state)
        stop_reason = self._safe_stop_reason(state.sim_time_ns)
        inference_due = self._inference_schedule.consume(state.sim_time_ns)
        inference_ran = False
        if stop_reason is None and inference_due:
            command = self._latest_command
            if command is None:
                raise ControllerRuntimeError("fresh-command invariant failed without an active command")
            try:
                observation = self._adapter.observe(state, command, self._controller.actuators)
                self._last_action = self._policy.infer(observation)
            except Exception as exc:
                self._last_action = None
                raise ControllerRuntimeError(f"controller inference failed: {exc}") from exc
            inference_ran = True
        elif stop_reason is not None:
            self._last_action = None

        actuator_command: ActuatorCommand | None = None
        if self._low_level_schedule.consume(state.sim_time_ns):
            if stop_reason is None and self._last_action is not None:
                raw_values = self._adapter.actuate(state, self._last_action, self._controller.actuators)
                safe_stop_reason = None
            else:
                raw_values = self._adapter.safe_stop(state, self._controller.actuators)
                safe_stop_reason = stop_reason or SafeStopReason.WAITING_FOR_INFERENCE
            values = self._ordered_finite_values(raw_values)
            self._output_sequence += 1
            actuator_command = ActuatorCommand(
                session_id=self._plan.session_id,
                channel_id=self._output_channel.channel_id,
                controller_id=self._controller.controller_id,
                instance_id=self._controller.instance_id,
                generation=self._generation,
                sequence=self._output_sequence,
                apply_time_ns=state.sim_time_ns,
                command_type=self._output_channel.command_type,
                channels=self._controller.actuators.channels,
                values=values,
                safe_stop=safe_stop_reason is not None,
                safe_stop_reason=safe_stop_reason,
            )
        self._last_state_sequence = state.sequence
        self._last_sim_time_ns = state.sim_time_ns
        return ControllerStep(
            state_sequence=state.sequence,
            sim_time_ns=state.sim_time_ns,
            inference_ran=inference_ran,
            actuator_command=actuator_command,
        )

    def set_generation(self, generation: GenerationStamp, *, start_time_ns: int = 0) -> None:
        """Switch generation and invalidate all prior state, actions, and commands."""

        if not isinstance(generation, GenerationStamp):
            raise TypeError("generation must be a GenerationStamp")
        if generation == self._generation:
            return
        _validate_start_time_ns(start_time_ns)
        if generation.model_generation < self._generation.model_generation or (
            generation.model_generation == self._generation.model_generation
            and generation.reset_generation < self._generation.reset_generation
        ):
            raise ControllerRuntimeError("cannot reactivate a stale generation")
        self._adapter.reset(generation)
        self._policy.reset(generation)
        self._generation = generation
        self._latest_command = None
        self._command_watermark = None
        self._last_action = None
        self._last_state_sequence = None
        self._last_sim_time_ns = None
        self._output_sequence = 0
        self._inference_schedule.reset(start_time_ns)
        self._low_level_schedule.reset(start_time_ns)

    @staticmethod
    def _resolve_channels(plan: ControlPlan, controller: ControllerSpec) -> tuple[frozenset[str], CommandChannelSpec]:
        referenced = [plan.command_channel(item) for item in controller.command_channels]
        inputs = frozenset(channel.channel_id for channel in referenced if channel.direction == "subscribe")
        outputs = [
            channel
            for channel in referenced
            if channel.direction == "publish"
            and channel.target == "actuators"
            and channel.source == controller.controller_id
        ]
        if not inputs:
            raise ControllerRuntimeError(f"controller {controller.controller_id!r} has no subscribed command channel")
        if len(outputs) != 1:
            raise ControllerRuntimeError(
                f"controller {controller.controller_id!r} must have exactly one actuator output"
            )
        return inputs, outputs[0]

    def _command_generation_result(self, generation: GenerationStamp) -> CommandSubmitResult:
        if generation.model_generation < self._generation.model_generation:
            return CommandSubmitResult.REJECTED_STALE_MODEL_GENERATION
        if generation.model_generation > self._generation.model_generation:
            return CommandSubmitResult.REJECTED_FUTURE_MODEL_GENERATION
        if generation.reset_generation < self._generation.reset_generation:
            return CommandSubmitResult.REJECTED_STALE_RESET_GENERATION
        if generation.reset_generation > self._generation.reset_generation:
            return CommandSubmitResult.REJECTED_FUTURE_RESET_GENERATION
        return CommandSubmitResult.ACCEPTED

    def _validate_state(self, state: ControllerState) -> None:
        if state.session_id != self._plan.session_id:
            raise ControllerRuntimeError("state session_id does not match the plan")
        if state.instance_id != self._controller.instance_id:
            raise ControllerRuntimeError("state instance_id does not match the controller")
        if state.generation != self._generation:
            raise ControllerRuntimeError("state model/reset generation does not match the active generation")
        missing_channels = set(self._controller.state_channels) - set(state.channels)
        if missing_channels:
            raise ControllerRuntimeError(f"state is missing required channels {sorted(missing_channels)!r}")
        if self._last_state_sequence is not None and state.sequence <= self._last_state_sequence:
            raise ControllerRuntimeError("state sequence must increase monotonically")
        if self._last_sim_time_ns is not None and state.sim_time_ns < self._last_sim_time_ns:
            raise ControllerRuntimeError("state sim_time_ns must not move backwards")

    def _safe_stop_reason(self, sim_time_ns: int) -> SafeStopReason | None:
        command = self._latest_command
        if command is None:
            return SafeStopReason.NO_COMMAND
        if command.apply_time_ns > sim_time_ns:
            return SafeStopReason.COMMAND_NOT_DUE
        if sim_time_ns - command.apply_time_ns >= self._plan.stale_timeout_ns:
            return SafeStopReason.STALE_COMMAND
        return None

    def _ordered_finite_values(self, raw_values: Any) -> tuple[float, ...]:
        try:
            ordered = self._controller.actuators.order(raw_values)
            values = tuple(float(value) for value in ordered)
        except (TypeError, ValueError) as exc:
            raise ControllerRuntimeError(f"invalid actuator output: {exc}") from exc
        if not all(math.isfinite(value) for value in values):
            raise ControllerRuntimeError("invalid actuator output: values must be finite")
        return values
