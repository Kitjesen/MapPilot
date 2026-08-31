"""Thread-owned interactive simulation session control."""

# ruff: noqa: D102

from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Callable, Mapping
from contextlib import AbstractContextManager
from enum import Enum
from typing import Any, Protocol

from sim.runtime.recording import SensorPayloadSample, SensorPayloadSource
from sim.runtime.windows_cpu_isolation import (
    bind_current_thread_affinity,
    validate_windows_affinity_mask,
)

from .coordinator import CoordinatorError, RuntimeState

MIN_ADVANCE_WAIT_S = 0.001
DEFAULT_ADVANCE_WAIT_S = 0.02
MAX_REALTIME_CATCH_UP_TICKS = 8


class InteractiveCoordinator(Protocol):
    """Coordinator surface owned by InteractiveSimulationSession."""

    @property
    def state(self) -> RuntimeState: ...

    def prepare(self) -> Mapping[str, Any]: ...

    def start(self) -> Mapping[str, Any]: ...

    def advance(self, steps: int = 1) -> Mapping[str, Any]: ...

    def pause(self) -> Mapping[str, Any]: ...

    def reset(self) -> Mapping[str, Any]: ...

    def hold_controller_commands(self) -> None: ...

    def stop(self, *, failure_reason: str | None = None) -> Mapping[str, Any]: ...


class InteractiveControlRequest(str, Enum):
    """Lifecycle dispositions returned by an interactive control pump."""

    PAUSE = "pause"
    RESUME = "resume"
    EXIT = "exit"


class InteractiveControlPump(Protocol):
    """Owner-thread seam between transport inboxes and simulation control."""

    def process_runtime_requests(self) -> InteractiveControlRequest | None: ...

    def process_before_advance(self, current_event: Mapping[str, Any]) -> None: ...

    def publish_status_after_advance(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int: ...

    def publish_terminal_status_after_stop(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int: ...

    def clear(self, *, reason: str) -> None: ...

    def clear_motion(self, *, reason: str) -> None: ...


class InteractiveSimulationSession:
    """Single-owner interactive facade with one serialized background advance loop."""

    def __init__(
        self,
        coordinator: InteractiveCoordinator,
        *,
        steps_per_tick: int = 1,
        advance_wait_s: float = DEFAULT_ADVANCE_WAIT_S,
        join_timeout_s: float = 2.0,
        sensor_payload_source: SensorPayloadSource | None = None,
        control_pump: InteractiveControlPump | None = None,
        owner_thread_affinity_mask: int | None = None,
        thread_affinity_scope_factory: (
            Callable[[int], AbstractContextManager[None]] | None
        ) = None,
    ) -> None:
        if isinstance(steps_per_tick, bool) or not isinstance(steps_per_tick, int) or steps_per_tick <= 0:
            raise ValueError("steps_per_tick must be a positive integer")
        if isinstance(advance_wait_s, bool) or not isinstance(advance_wait_s, (int, float)):
            raise ValueError("advance_wait_s must be numeric")
        advance_wait = float(advance_wait_s)
        if not math.isfinite(advance_wait):
            raise ValueError("advance_wait_s must be finite")
        if isinstance(join_timeout_s, bool) or not isinstance(join_timeout_s, (int, float)):
            raise ValueError("join_timeout_s must be positive and finite")
        join_timeout = float(join_timeout_s)
        if join_timeout <= 0 or not math.isfinite(join_timeout):
            raise ValueError("join_timeout_s must be positive and finite")
        self._coordinator = coordinator
        self._steps_per_tick = steps_per_tick
        self._advance_wait_s = max(advance_wait, MIN_ADVANCE_WAIT_S)
        self._join_timeout_s = join_timeout
        self._condition = threading.Condition(threading.RLock())
        self._operation_lock = threading.Lock()
        self._stop_requested = False
        self._closed = False
        self._coordinator_stopped = False
        self._thread: threading.Thread | None = None
        self._last_event: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._state = RuntimeState.NEW
        self._model_generation: int | None = None
        self._reset_generation: int | None = None
        self._event_observers: dict[int, Callable[[Mapping[str, Any]], object]] = {}
        self._next_observer_token = 1
        self._sensor_payload_source = sensor_payload_source
        self._control_pump = control_pump
        if owner_thread_affinity_mask is None:
            if thread_affinity_scope_factory is not None:
                raise ValueError(
                    "thread_affinity_scope_factory requires an owner affinity mask"
                )
            self._owner_thread_affinity_mask = None
            self._thread_affinity_scope_factory = None
        else:
            self._owner_thread_affinity_mask = validate_windows_affinity_mask(
                owner_thread_affinity_mask,
                "owner thread affinity mask",
            )
            self._thread_affinity_scope_factory = (
                thread_affinity_scope_factory or bind_current_thread_affinity
            )

    @property
    def state(self) -> RuntimeState:
        with self._condition:
            return self._state

    @property
    def model_generation(self) -> int | None:
        with self._condition:
            return self._model_generation

    @property
    def reset_generation(self) -> int | None:
        with self._condition:
            return self._reset_generation

    @property
    def last_event(self) -> dict[str, Any] | None:
        with self._condition:
            return None if self._last_event is None else dict(self._last_event)

    @property
    def last_error(self) -> str | None:
        with self._condition:
            return self._last_error

    @property
    def background_thread_alive(self) -> bool:
        with self._condition:
            thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def advance_wait_s(self) -> float:
        return self._advance_wait_s

    def attach_event_observer(
        self,
        observer: Callable[[Mapping[str, Any]], object],
        *,
        replay_latest_snapshot: bool = False,
    ) -> int:
        """Attach an ordered in-process event observer and return its token.

        Observer delivery is synchronous with event acceptance so detaching the
        token establishes a hard boundary: no later event can reach that
        observer.  This is used by bounded recording without exposing the
        coordinator or its background thread.
        """

        if not callable(observer):
            raise TypeError("observer must be callable")
        with self._condition:
            self._raise_if_closing("attach_event_observer")
            latest = self._last_event
            if (
                replay_latest_snapshot
                and latest is not None
                and latest.get("event") == "snapshot"
            ):
                observer(copy.deepcopy(latest))
            token = self._next_observer_token
            self._next_observer_token += 1
            self._event_observers[token] = observer
            return token

    def detach_event_observer(self, token: int) -> bool:
        """Detach one observer at an ordered event boundary."""

        if isinstance(token, bool) or not isinstance(token, int) or token < 1:
            raise ValueError("observer token must be a positive integer")
        with self._condition:
            return self._event_observers.pop(token, None) is not None

    def capture_sensor_payloads(
        self,
        snapshot: Mapping[str, Any],
    ) -> tuple[SensorPayloadSample, ...]:
        """Capture unseen payloads at the same accepted snapshot boundary."""

        source = self._sensor_payload_source
        if source is None:
            return ()
        return tuple(source.capture(snapshot))

    def prepare(self) -> dict[str, Any]:
        with self._condition:
            self._raise_if_closing("prepare")
        operation_started = False
        try:
            with self._operation_lock:
                with self._condition:
                    self._raise_if_closing("prepare")
                    self._require(RuntimeState.NEW, "prepare")
                    self._state = RuntimeState.PREPARING
                    operation_started = True
                event = self._coordinator.prepare()
                with self._condition:
                    self._state = self._coordinator.state
                    return self._accept_event(event)
        except BaseException as exc:
            if operation_started:
                self._abort_failed(exc)
            raise

    def start(self) -> dict[str, Any]:
        with self._condition:
            self._raise_if_closing("start")
        operation_started = False
        try:
            with self._operation_lock:
                with self._condition:
                    self._raise_if_closing("start")
                    if self._state not in {RuntimeState.READY, RuntimeState.PAUSED}:
                        raise CoordinatorError(f"start is invalid in state {self._state.value}")
                    was_paused = self._state is RuntimeState.PAUSED
                    operation_started = True
                if was_paused:
                    self._clear_control_motion("resume")
                event = self._coordinator.start()
                with self._condition:
                    self._state = RuntimeState.RUNNING
                    accepted = self._accept_event(event)
                physics_step = accepted.get("physics_step")
                if isinstance(physics_step, int) and not isinstance(physics_step, bool):
                    remainder = physics_step % self._steps_per_tick
                    if remainder:
                        alignment_event = self._coordinator.advance(
                            self._steps_per_tick - remainder
                        )
                        with self._condition:
                            accepted = self._accept_event(alignment_event)
                with self._condition:
                    if not self._stop_requested and not self._closed:
                        self._ensure_thread_locked()
                    self._condition.notify_all()
                    return accepted
        except BaseException as exc:
            if operation_started:
                self._abort_failed(exc)
            raise

    def pause(self) -> dict[str, Any]:
        with self._condition:
            self._raise_if_closing("pause")
        operation_started = False
        try:
            with self._operation_lock:
                with self._condition:
                    self._raise_if_closing("pause")
                    self._require(RuntimeState.RUNNING, "pause")
                    operation_started = True
                self._quiesce_control("pause")
                event = self._coordinator.pause()
                with self._condition:
                    self._state = RuntimeState.PAUSED
                    accepted = self._accept_event(event)
                    self._condition.notify_all()
                    return accepted
        except BaseException as exc:
            if operation_started:
                self._abort_failed(exc)
            raise

    def reset(self) -> dict[str, Any]:
        with self._condition:
            self._raise_if_closing("reset")
        operation_started = False
        try:
            with self._operation_lock:
                with self._condition:
                    self._raise_if_closing("reset")
                    if self._state not in {RuntimeState.READY, RuntimeState.RUNNING, RuntimeState.PAUSED}:
                        raise CoordinatorError(f"reset is invalid in state {self._state.value}")
                    was_running = self._state is RuntimeState.RUNNING
                    operation_started = True
                self._quiesce_control("reset")
                event = self._coordinator.reset()
                with self._condition:
                    self._state = RuntimeState.RUNNING if was_running else self._coordinator.state
                    accepted = self._accept_event(event, allow_reset_increment=True)
                    self._condition.notify_all()
                    return accepted
        except BaseException as exc:
            if operation_started:
                self._abort_failed(exc)
            raise

    def stop(self) -> dict[str, Any]:
        return self._stop(failure_reason=None)

    def cleanup(self) -> dict[str, Any]:
        return self.stop()

    def _stop(self, *, failure_reason: str | None) -> dict[str, Any]:
        thread: threading.Thread | None
        with self._condition:
            if self._thread is threading.current_thread() and self._thread.is_alive():
                error = CoordinatorError("stop cannot be called from the advance thread")
                self._fail_locked(error)
                raise error
            if self._coordinator_stopped:
                return dict(self._last_event or {})
            if self._closed and self._state is RuntimeState.STOPPED and self._last_event is None:
                return {}
            if self._state is RuntimeState.NEW:
                control_pump = self._control_pump
                if control_pump is not None:
                    control_pump.clear(reason="stop")
                self._closed = True
                self._stop_requested = True
                self._state = RuntimeState.STOPPED
                self._condition.notify_all()
                return {}
            self._closed = True
            self._stop_requested = True
            self._condition.notify_all()
            thread = self._thread if self._thread is not threading.current_thread() else None

        if thread is not None:
            thread.join(self._join_timeout_s)
            if thread.is_alive():
                with self._condition:
                    error = CoordinatorError("background advance thread did not exit")
                    self._fail_locked(error)
                raise error

        with self._operation_lock:
            with self._condition:
                if self._coordinator_stopped:
                    return dict(self._last_event or {})
            try:
                self._quiesce_control("stop")
                event = self._coordinator.stop(failure_reason=failure_reason or self._last_error)
                with self._condition:
                    accepted = self._accept_event(event)
                    self._coordinator_stopped = True
                    if failure_reason is not None or self._last_error is not None:
                        self._state = RuntimeState.FAILED
                    else:
                        self._state = RuntimeState.STOPPED
                    return accepted
            except BaseException as exc:
                with self._condition:
                    self._fail_locked(exc)
                raise

    def _advance_loop(self) -> None:
        next_realtime_deadline = time.monotonic()
        was_running = False
        affinity_scope: AbstractContextManager[None] | None = None
        affinity_entered = False
        loop_error: BaseException | None = None
        try:
            if self._owner_thread_affinity_mask is not None:
                factory = self._thread_affinity_scope_factory
                if factory is None:  # pragma: no cover - constructor invariant
                    raise CoordinatorError("owner thread affinity scope is unavailable")
                affinity_scope = factory(self._owner_thread_affinity_mask)
                try:
                    affinity_scope.__enter__()
                except BaseException as enter_error:
                    try:
                        affinity_scope.__exit__(
                            type(enter_error),
                            enter_error,
                            enter_error.__traceback__,
                        )
                    except BaseException as restore_error:
                        raise CoordinatorError(
                            "owner thread affinity entry failed and rollback also failed: "
                            f"{type(restore_error).__name__}: {restore_error}"
                        ) from enter_error
                    finally:
                        affinity_scope = None
                    raise
                affinity_entered = True
            while True:
                iteration_started = time.monotonic()
                with self._condition:
                    if self._state is not RuntimeState.RUNNING:
                        was_running = False
                        next_realtime_deadline = time.monotonic()
                    while (
                        not self._stop_requested
                        and self._state is not RuntimeState.RUNNING
                        and self._control_pump is None
                    ):
                        self._condition.wait(self._advance_wait_s)
                    if self._stop_requested:
                        return
                with self._operation_lock:
                    try:
                        control_pump = self._control_pump
                        if control_pump is not None:
                            request = control_pump.process_runtime_requests()
                            if request is not None:
                                self._apply_control_request(request)
                        with self._condition:
                            if self._stop_requested:
                                return
                            is_running = self._state is RuntimeState.RUNNING
                        if is_running and control_pump is not None:
                            control_pump.process_before_advance(
                                self._control_event_for_pump()
                            )
                        if is_running:
                            now = time.monotonic()
                            if not was_running:
                                next_realtime_deadline = now
                            overdue_s = max(0.0, now - next_realtime_deadline)
                            due_ticks = min(
                                MAX_REALTIME_CATCH_UP_TICKS,
                                1 + int(overdue_s / self._advance_wait_s),
                            )
                            if due_ticks == MAX_REALTIME_CATCH_UP_TICKS:
                                # A debugger pause or a suspended laptop must not
                                # create an unbounded catch-up spiral.  Keep one
                                # bounded batch of debt and drop older wall time.
                                next_realtime_deadline = now - (
                                    (due_ticks - 1) * self._advance_wait_s
                                )
                            next_realtime_deadline += (
                                due_ticks * self._advance_wait_s
                            )
                            realtime_advance = getattr(
                                self._coordinator,
                                "advance_realtime",
                                None,
                            )
                            event = None
                            for _ in range(due_ticks):
                                event = (
                                    realtime_advance(self._steps_per_tick)
                                    if callable(realtime_advance)
                                    else self._coordinator.advance(
                                        self._steps_per_tick
                                    )
                                )
                            if event is None:  # pragma: no cover - due_ticks >= 1
                                raise CoordinatorError(
                                    "realtime scheduler produced no advance event"
                                )
                            was_running = True
                            with self._condition:
                                self._accept_event(event)
                            if control_pump is not None:
                                control_pump.publish_status_after_advance(
                                    event,
                                runtime_state=RuntimeState.RUNNING.value,
                            )
                        elif control_pump is not None:
                            was_running = False
                            next_realtime_deadline = time.monotonic()
                            with self._condition:
                                status_state = self._state.value
                            control_pump.publish_status_after_advance(
                                self._control_event_for_pump(),
                                runtime_state=status_state,
                            )
                    except BaseException as exc:
                        loop_error = exc
                        with self._condition:
                            self._fail_locked(exc)
                        return
                with self._condition:
                    if self._stop_requested:
                        return
                    if was_running:
                        remaining = next_realtime_deadline - time.monotonic()
                    else:
                        elapsed = time.monotonic() - iteration_started
                        remaining = self._advance_wait_s - elapsed
                    if remaining > 0.0:
                        self._condition.wait(remaining)
        except BaseException as exc:
            loop_error = exc
            with self._condition:
                self._fail_locked(exc)
        finally:
            restore_error: BaseException | None = None
            if affinity_entered and affinity_scope is not None:
                try:
                    affinity_scope.__exit__(None, None, None)
                except BaseException as exc:
                    restore_error = exc
            with self._condition:
                if restore_error is not None:
                    if loop_error is None:
                        self._fail_locked(restore_error)
                    else:
                        _add_exception_note(
                            loop_error,
                            "owner thread affinity restoration also failed: "
                            f"{_error_summary(restore_error)}",
                        )
                        self._fail_locked(loop_error)
                if self._thread is threading.current_thread():
                    self._thread = None
                    self._condition.notify_all()

    def _apply_control_request(self, request: InteractiveControlRequest) -> None:
        if not isinstance(request, InteractiveControlRequest):
            raise CoordinatorError("control pump returned an invalid lifecycle request")
        with self._condition:
            state = self._state
        if request is InteractiveControlRequest.PAUSE:
            if state is not RuntimeState.RUNNING:
                return
            self._quiesce_control("pause")
            event = self._coordinator.pause()
            with self._condition:
                self._state = RuntimeState.PAUSED
                self._accept_event(event)
                self._condition.notify_all()
            return
        if request is InteractiveControlRequest.RESUME:
            if state is not RuntimeState.PAUSED:
                return
            self._clear_control_motion("resume")
            event = self._coordinator.start()
            with self._condition:
                self._state = RuntimeState.RUNNING
                self._accept_event(event)
                self._condition.notify_all()
            return
        stop_runtime = getattr(
            self._coordinator,
            "stop_runtime_before_visual",
            None,
        )
        finalize_visual = getattr(
            self._coordinator,
            "finalize_visual_after_terminal",
            None,
        )
        if not callable(stop_runtime) or not callable(finalize_visual):
            raise CoordinatorError(
                "runtime Exit requires two-phase visual shutdown support"
            )
        try:
            self._quiesce_control("exit")
            event = stop_runtime(failure_reason=None)
            with self._condition:
                self._accept_event(event)
                self._coordinator_stopped = True
            control_pump = self._control_pump
            if control_pump is None:
                raise CoordinatorError(
                    "runtime exit requires a terminal control status publisher"
                )
            published = control_pump.publish_terminal_status_after_stop(
                event,
                runtime_state=RuntimeState.STOPPED.value,
            )
            if (
                isinstance(published, bool)
                or not isinstance(published, int)
                or published <= 0
            ):
                raise CoordinatorError(
                    "runtime exit terminal control status was not completely published"
                )
            exit_code = finalize_visual()
            if (
                isinstance(exit_code, bool)
                or not isinstance(exit_code, int)
                or exit_code != 0
            ):
                raise CoordinatorError(
                    "runtime exit did not receive a successful Unreal process readback"
                )
        except BaseException as exc:
            self._close_after_exit_failure(exc)
            raise
        with self._condition:
            self._closed = True
            self._stop_requested = True
            self._state = RuntimeState.STOPPED
            self._condition.notify_all()

    def _close_after_exit_failure(self, error: BaseException) -> None:
        close = getattr(self._coordinator, "close", None)
        if not callable(close):
            return
        try:
            close(failure_reason=_error_summary(error))
        except TypeError:
            try:
                close()
            except BaseException as cleanup_error:
                _note_exit_cleanup_failure(error, cleanup_error)
        except BaseException as cleanup_error:
            _note_exit_cleanup_failure(error, cleanup_error)

    def _quiesce_control(self, reason: str) -> None:
        control_pump = self._control_pump
        clear_error: BaseException | None = None
        if control_pump is not None:
            try:
                control_pump.clear(reason=reason)
            except BaseException as exc:
                clear_error = exc
        try:
            self._coordinator.hold_controller_commands()
        except BaseException as hold_error:
            if clear_error is not None:
                _add_exception_note(
                    hold_error,
                    f"control pump clear also failed: {_error_summary(clear_error)}",
                )
            raise
        if clear_error is not None:
            raise clear_error

    def _clear_control_motion(self, reason: str) -> None:
        control_pump = self._control_pump
        if control_pump is not None:
            control_pump.clear_motion(reason=reason)

    def _control_event_for_pump(self) -> dict[str, Any]:
        with self._condition:
            current_event = copy.deepcopy(self._last_event)
        if current_event is None:
            raise CoordinatorError("control pump requires a current coordinator event")
        for field in ("model_generation", "reset_generation", "sequence", "sim_time_ns"):
            value = current_event.get(field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise CoordinatorError(f"control pump event has invalid {field}")
        return current_event

    def _ensure_thread_locked(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_requested = False
        self._thread = threading.Thread(
            target=self._advance_loop,
            name="lingtu-interactive-sim-advance",
            daemon=True,
        )
        self._thread.start()

    def _accept_event(
        self,
        event: Mapping[str, Any],
        *,
        allow_reset_increment: bool = False,
    ) -> dict[str, Any]:
        if not isinstance(event, Mapping):
            raise CoordinatorError("coordinator event must be an object")
        accepted = dict(event)
        event_name = accepted.get("event")
        if not isinstance(event_name, str) or not event_name.strip():
            raise CoordinatorError("coordinator event must contain a non-empty event")
        model_generation = _required_generation(accepted, "model_generation")
        reset_generation = _required_generation(accepted, "reset_generation")
        if self._model_generation is not None and model_generation != self._model_generation:
            raise CoordinatorError(
                "coordinator event model_generation does not match the current session"
            )
        if self._reset_generation is not None:
            expected_reset = self._reset_generation
            valid_reset = reset_generation == expected_reset or (
                allow_reset_increment and reset_generation == expected_reset + 1
            )
            if not valid_reset:
                raise CoordinatorError(
                    "coordinator event reset_generation does not match the current session"
                )
        self._model_generation = model_generation
        self._reset_generation = reset_generation
        self._last_event = accepted
        for observer in tuple(self._event_observers.values()):
            observer(copy.deepcopy(accepted))
        return accepted

    def _require(self, expected: RuntimeState, operation: str) -> None:
        if self._state is not expected:
            raise CoordinatorError(f"{operation} requires {expected.value}, current state is {self._state.value}")

    def _raise_if_closing(self, operation: str) -> None:
        if self._closed or self._stop_requested:
            raise CoordinatorError(f"{operation} is invalid after stop/cleanup has begun")

    def _fail_locked(self, error: BaseException) -> None:
        self._state = RuntimeState.FAILED
        self._last_error = _error_summary(error)
        self._stop_requested = True
        self._condition.notify_all()

    def _abort_failed(self, error: BaseException) -> None:
        """Record a lifecycle failure and synchronously drain its worker."""

        with self._condition:
            self._fail_locked(error)
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(self._join_timeout_s)


def _required_generation(event: Mapping[str, Any], field: str) -> int:
    value = event.get(field)
    if field not in event:
        raise CoordinatorError(f"coordinator event is missing {field}")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"{field} must be a non-negative integer when present")
    return value


def _error_summary(error: BaseException) -> str:
    message = str(error).strip()
    summary = f"{type(error).__name__}: {message}" if message else type(error).__name__
    notes = tuple(getattr(error, "__notes__", ()))
    if notes:
        summary += "; notes: " + " | ".join(str(note) for note in notes)
    return summary


def _add_exception_note(error: BaseException, note: str) -> None:
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)
        return
    notes = list(getattr(error, "__notes__", ()))
    notes.append(note)
    try:
        error.__notes__ = notes
    except (AttributeError, TypeError):
        pass


def _note_exit_cleanup_failure(error: BaseException, cleanup_error: BaseException) -> None:
    _add_exception_note(
        error,
        "two-phase visual cleanup failed: "
        f"{_error_summary(cleanup_error)}",
    )


__all__ = [
    "DEFAULT_ADVANCE_WAIT_S",
    "MIN_ADVANCE_WAIT_S",
    "InteractiveControlPump",
    "InteractiveControlRequest",
    "InteractiveSimulationSession",
]
