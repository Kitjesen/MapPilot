# ruff: noqa: B009, D103, S101

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any

import pytest
from sim.runtime.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.interactive_session import (
    MIN_ADVANCE_WAIT_S,
    InteractiveControlRequest,
    InteractiveSimulationSession,
)


class _FakeCoordinator:
    def __init__(self) -> None:
        self.state = RuntimeState.NEW
        self.calls: list[Any] = []
        self.model_generation = 3
        self.reset_generation = 0
        self.sequence = 0
        self.fail: dict[str, BaseException] = {}
        self.advance_entered = threading.Event()
        self.block_advance = False
        self.release_advance = threading.Event()
        self.advance_hook: Any = None
        self.bad_events: dict[str, Any] = {}

    def prepare(self) -> dict[str, Any]:
        self._maybe_raise("prepare")
        self.calls.append("prepare")
        self.state = RuntimeState.READY
        return self._event("ready")

    def start(self) -> dict[str, Any]:
        self._maybe_raise("start")
        self.calls.append("start")
        self.state = RuntimeState.RUNNING
        return self._event("running")

    def advance(self, steps: int = 1) -> dict[str, Any]:
        self._maybe_raise("advance")
        self.calls.append(("advance", steps))
        self.advance_entered.set()
        if self.advance_hook is not None:
            self.advance_hook()
        if self.block_advance:
            self.release_advance.wait(1.0)
        self.sequence += steps
        return self._event("snapshot")

    def pause(self) -> dict[str, Any]:
        self._maybe_raise("pause")
        self.calls.append("pause")
        self.state = RuntimeState.PAUSED
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        self._maybe_raise("reset")
        self.calls.append("reset")
        self.reset_generation += 1
        self.sequence = 0
        return self._event("snapshot")

    def hold_controller_commands(self) -> None:
        self.calls.append("hold")

    def stop(self, *, failure_reason: str | None = None) -> dict[str, Any]:
        self._maybe_raise("stop")
        self.calls.append(("stop", failure_reason))
        self.state = RuntimeState.FAILED if failure_reason else RuntimeState.STOPPED
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        if event in self.bad_events:
            return self.bad_events[event]
        return {
            "event": event,
            "session_id": "interactive-session",
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "sequence": self.sequence,
            "physics_step": self.sequence,
            "sim_time_ns": self.sequence * 2_000_000,
        }

    def _maybe_raise(self, operation: str) -> None:
        error = self.fail.get(operation)
        if error is not None:
            raise error


class _ExitCoordinator(_FakeCoordinator):
    def __init__(self) -> None:
        super().__init__()
        self.close_reasons: list[str | None] = []
        self.closed = False
        self.finalize_exit_code = 0

    def stop_runtime_before_visual(
        self,
        *,
        failure_reason: str | None = None,
    ) -> dict[str, Any]:
        self._maybe_raise("stop_runtime_before_visual")
        self.calls.append(("stop_runtime_before_visual", failure_reason))
        self.state = RuntimeState.STOPPED
        return self._event("stopped")

    def finalize_visual_after_terminal(self) -> int:
        self._maybe_raise("finalize_visual_after_terminal")
        self.calls.append("finalize_visual_after_terminal")
        return self.finalize_exit_code

    def close(self, *, failure_reason: str | None = None) -> None:
        self.calls.append(("close", failure_reason))
        self.close_reasons.append(failure_reason)
        self.closed = True
        if failure_reason is not None:
            self.state = RuntimeState.FAILED


class _ExitPump:
    def __init__(self, *, fail_clear: bool = False, fail_terminal: bool = False) -> None:
        self.fail_clear = fail_clear
        self.fail_terminal = fail_terminal
        self.requests = 0
        self.calls: list[Any] = []

    def process_runtime_requests(self) -> InteractiveControlRequest | None:
        self.requests += 1
        if self.requests == 1:
            return InteractiveControlRequest.EXIT
        return None

    def process_before_advance(self, current_event: Mapping[str, Any]) -> None:
        self.calls.append(("before", current_event["event"]))

    def publish_status_after_advance(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        self.calls.append(("status", snapshot["event"], runtime_state))
        return 1

    def publish_terminal_status_after_stop(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        self.calls.append(("terminal", snapshot["event"], runtime_state))
        if self.fail_terminal:
            raise RuntimeError("terminal status send failed")
        return 1

    def clear(self, *, reason: str) -> None:
        self.calls.append(("clear", reason))
        if self.fail_clear:
            raise RuntimeError("clear failed before exit")

    def clear_motion(self, *, reason: str) -> None:
        self.calls.append(("clear_motion", reason))


def _wait_until(predicate: Any, *, timeout_s: float = 0.5) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def test_interactive_session_runs_and_stops_background_loop() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=2,
        advance_wait_s=0.01,
    )

    ready = session.prepare()
    running = session.start()

    assert ready["event"] == "ready"
    assert running["event"] == "running"
    assert session.state is RuntimeState.RUNNING
    assert session.model_generation == 3
    assert session.reset_generation == 0
    assert _wait_until(lambda: any(call == ("advance", 2) for call in coordinator.calls))
    assert session.background_thread_alive

    stopped = session.stop()
    again = session.cleanup()

    assert stopped["event"] == "stopped"
    assert again == stopped
    assert session.state is RuntimeState.STOPPED
    assert not session.background_thread_alive
    assert coordinator.calls.count(("stop", None)) == 1


def test_pause_and_resume_serializes_background_advance() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)
    session.prepare()
    session.start()
    assert _wait_until(lambda: coordinator.sequence >= 1)

    session.pause()
    paused_sequence = coordinator.sequence
    time.sleep(0.04)

    assert session.state is RuntimeState.PAUSED
    assert coordinator.sequence == paused_sequence

    session.start()
    assert _wait_until(lambda: coordinator.sequence > paused_sequence)
    session.stop()


def test_event_observer_attaches_at_an_ordered_snapshot_boundary() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)
    session.prepare()
    session.start()
    assert _wait_until(
        lambda: session.last_event is not None
        and session.last_event.get("event") == "snapshot"
    )
    observed: list[dict[str, Any]] = []

    token = session.attach_event_observer(observed.append, replay_latest_snapshot=True)
    assert observed[0]["event"] == "snapshot"
    assert _wait_until(lambda: len(observed) >= 2)

    assert session.detach_event_observer(token) is True
    assert session.detach_event_observer(token) is False
    detached_count = len(observed)
    time.sleep(0.04)
    assert len(observed) == detached_count
    session.stop()


def test_event_observer_must_be_callable() -> None:
    session = InteractiveSimulationSession(_FakeCoordinator())

    with pytest.raises(TypeError, match="observer"):
        session.attach_event_observer(None)  # type: ignore[arg-type]


def test_reset_updates_generation_and_keeps_running_session_running() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.02)
    session.prepare()
    session.start()
    assert _wait_until(lambda: coordinator.sequence >= 1)

    snapshot = session.reset()

    assert snapshot["reset_generation"] == 1
    assert session.reset_generation == 1
    assert session.model_generation == 3
    assert session.state is RuntimeState.RUNNING
    session.stop()


@pytest.mark.parametrize(
    ("operation", "message"),
    [
        ("start", "NEW"),
        ("pause", "NEW"),
        ("reset", "NEW"),
    ],
)
def test_illegal_transitions_raise_without_marking_failed(operation: str, message: str) -> None:
    session = InteractiveSimulationSession(_FakeCoordinator())

    with pytest.raises(CoordinatorError, match=message):
        getattr(session, operation)()

    assert session.state is RuntimeState.NEW


def test_repeated_stop_before_prepare_is_idempotent_and_does_not_start_thread() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator)

    assert session.stop() == {}
    assert session.stop() == {}
    assert session.state is RuntimeState.STOPPED
    assert not session.background_thread_alive
    assert coordinator.calls == []


def test_advance_exception_marks_failed_and_preserves_error_info() -> None:
    coordinator = _FakeCoordinator()
    coordinator.fail["advance"] = RuntimeError("physics exploded")
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)

    session.prepare()
    session.start()

    assert _wait_until(lambda: session.state is RuntimeState.FAILED)
    assert session.last_error == "RuntimeError: physics exploded"
    assert not session.background_thread_alive
    stopped = session.stop()
    assert stopped["event"] == "stopped"
    assert coordinator.calls[-1] == ("stop", "RuntimeError: physics exploded")


def test_malformed_advance_event_fails_closed_and_drains_thread() -> None:
    coordinator = _FakeCoordinator()
    coordinator.bad_events["snapshot"] = ["not", "an", "event"]
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)

    session.prepare()
    session.start()

    assert _wait_until(lambda: session.state is RuntimeState.FAILED)
    assert _wait_until(lambda: not session.background_thread_alive)
    assert session.last_error == "CoordinatorError: coordinator event must be an object"

    stopped = session.stop()
    assert stopped["event"] == "stopped"
    assert session.stop() == stopped


def test_generation_mismatch_fails_closed_and_drains_thread() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)

    def change_model_generation() -> None:
        coordinator.model_generation = 4
        coordinator.advance_hook = None

    coordinator.advance_hook = change_model_generation
    session.prepare()
    session.start()

    assert _wait_until(lambda: session.state is RuntimeState.FAILED)
    assert _wait_until(lambda: not session.background_thread_alive)
    assert "model_generation does not match" in (session.last_error or "")
    coordinator.model_generation = 3
    session.stop()


def test_stop_from_advance_thread_fails_fast_without_deadlock() -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)
    stop_errors: list[BaseException] = []
    invoked = threading.Event()

    def stop_reentrantly() -> None:
        if invoked.is_set():
            return
        invoked.set()
        try:
            session.stop()
        except BaseException as exc:
            stop_errors.append(exc)

    coordinator.advance_hook = stop_reentrantly
    session.prepare()
    session.start()

    assert invoked.wait(0.5)
    assert _wait_until(lambda: not session.background_thread_alive)
    assert session.state is RuntimeState.FAILED
    assert len(stop_errors) == 1
    assert isinstance(stop_errors[0], CoordinatorError)
    assert "advance thread" in str(stop_errors[0])

    stopped = session.stop()
    assert stopped["event"] == "stopped"
    assert session.stop() == stopped


@pytest.mark.parametrize("operation", ["start", "pause", "reset"])
def test_lifecycle_failure_terminates_existing_advance_thread(operation: str) -> None:
    coordinator = _FakeCoordinator()
    session = InteractiveSimulationSession(coordinator, advance_wait_s=0.01)
    session.prepare()
    session.start()
    assert _wait_until(lambda: session.background_thread_alive)

    if operation == "start":
        session.pause()
    coordinator.fail[operation] = RuntimeError(f"{operation} failed")

    with pytest.raises(RuntimeError, match=f"{operation} failed"):
        getattr(session, operation)()

    assert session.state is RuntimeState.FAILED
    assert _wait_until(lambda: not session.background_thread_alive)
    stopped = session.stop()
    assert stopped["event"] == "stopped"
    assert session.stop() == stopped


def test_transition_exception_marks_failed_and_reports_stop_reason() -> None:
    coordinator = _FakeCoordinator()
    coordinator.fail["start"] = RuntimeError("host refused start")
    session = InteractiveSimulationSession(coordinator)
    session.prepare()

    with pytest.raises(RuntimeError, match="host refused start"):
        session.start()

    assert session.state is RuntimeState.FAILED
    assert session.last_error == "RuntimeError: host refused start"
    stopped = session.stop()
    assert stopped["event"] == "stopped"
    assert coordinator.calls[-1] == ("stop", "RuntimeError: host refused start")


def test_thread_exit_waits_for_inflight_advance_without_busy_spin() -> None:
    coordinator = _FakeCoordinator()
    coordinator.block_advance = True
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0,
        join_timeout_s=1.0,
    )

    assert session.advance_wait_s == MIN_ADVANCE_WAIT_S
    session.prepare()
    session.start()
    assert coordinator.advance_entered.wait(0.5)
    assert coordinator.calls.count(("advance", 1)) == 1
    time.sleep(0.03)
    assert coordinator.calls.count(("advance", 1)) == 1

    coordinator.release_advance.set()
    session.stop()

    assert not session.background_thread_alive


def test_stop_gate_blocks_lifecycle_calls_while_advance_is_exiting() -> None:
    coordinator = _FakeCoordinator()
    coordinator.block_advance = True
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0.01,
        join_timeout_s=1.0,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_entered.wait(0.5)

    stop_error: list[BaseException] = []

    def stop_session() -> None:
        try:
            session.stop()
        except BaseException as exc:
            stop_error.append(exc)

    stopper = threading.Thread(target=stop_session)
    stopper.start()
    assert _wait_until(lambda: getattr(session, "_closed"))
    calls_before = list(coordinator.calls)

    for operation in (session.start, session.pause, session.reset):
        with pytest.raises(CoordinatorError, match="stop/cleanup has begun"):
            operation()

    assert coordinator.calls == calls_before
    coordinator.release_advance.set()
    stopper.join(1.0)

    assert stop_error == []
    assert not stopper.is_alive()
    assert not session.background_thread_alive
    assert coordinator.calls[-1] == ("stop", None)


def test_stop_timeout_can_retry_cleanup_after_background_thread_exits() -> None:
    coordinator = _FakeCoordinator()
    coordinator.block_advance = True
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0.01,
        join_timeout_s=0.01,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_entered.wait(0.5)

    with pytest.raises(CoordinatorError, match="background advance thread did not exit"):
        session.stop()

    assert session.state is RuntimeState.FAILED
    assert session.last_error == "CoordinatorError: background advance thread did not exit"
    assert coordinator.calls.count(("stop", None)) == 0
    assert coordinator.calls.count(("stop", session.last_error)) == 0

    coordinator.release_advance.set()
    assert _wait_until(lambda: not session.background_thread_alive)
    stopped = session.stop()
    again = session.stop()

    assert stopped["event"] == "stopped"
    assert again == stopped
    assert coordinator.calls.count(("stop", "CoordinatorError: background advance thread did not exit")) == 1


def test_exit_terminal_status_failure_closes_two_phase_runtime_with_exact_reason() -> None:
    coordinator = _ExitCoordinator()
    pump = _ExitPump(fail_terminal=True)
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0.01,
        control_pump=pump,
    )
    session.prepare()
    session.start()

    assert _wait_until(lambda: session.state is RuntimeState.FAILED)
    assert not session.background_thread_alive
    assert session.last_error == "RuntimeError: terminal status send failed"
    assert coordinator.closed
    assert coordinator.close_reasons == ["RuntimeError: terminal status send failed"]
    assert coordinator.calls.count(("stop_runtime_before_visual", None)) == 1
    assert ("close", "RuntimeError: terminal status send failed") in coordinator.calls


def test_exit_quiesce_failure_still_closes_runtime_with_exact_reason() -> None:
    coordinator = _ExitCoordinator()
    pump = _ExitPump(fail_clear=True)
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0.01,
        control_pump=pump,
    )
    session.prepare()
    session.start()

    assert _wait_until(lambda: session.state is RuntimeState.FAILED)
    assert not session.background_thread_alive
    assert session.last_error == "RuntimeError: clear failed before exit"
    assert coordinator.closed
    assert coordinator.close_reasons == ["RuntimeError: clear failed before exit"]
    assert ("stop_runtime_before_visual", None) not in coordinator.calls
    assert ("close", "RuntimeError: clear failed before exit") in coordinator.calls


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"advance_wait_s": float("nan")}, "advance_wait_s"),
        ({"advance_wait_s": float("inf")}, "advance_wait_s"),
        ({"join_timeout_s": float("nan")}, "join_timeout_s"),
        ({"join_timeout_s": float("inf")}, "join_timeout_s"),
        ({"join_timeout_s": 0}, "join_timeout_s"),
    ],
)
def test_wait_configuration_rejects_non_finite_values(kwargs: dict[str, Any], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        InteractiveSimulationSession(_FakeCoordinator(), **kwargs)
