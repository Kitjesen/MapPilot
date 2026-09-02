# ruff: noqa: S101

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Mapping
from contextlib import contextmanager
from typing import Any

import pytest

from sim.runtime.control.contracts import (
    CommandSubmitResult,
    ControllerCommand,
    GenerationStamp,
)
from sim.runtime.coordinator.coordinator import RuntimeState
from sim.runtime.coordinator.interactive_session import (
    InteractiveControlRequest,
    InteractiveSimulationSession,
)
from sim.runtime.coordinator.session_host import SessionHost


class _RecordingCoordinator:
    def __init__(self) -> None:
        self._state = RuntimeState.NEW
        self._reset_generation = 0
        self._sequence = 0
        self.calls: list[tuple[str, int]] = []
        self.advance_seen = threading.Event()

    @property
    def state(self) -> RuntimeState:
        return self._state

    def prepare(self) -> Mapping[str, Any]:
        self._state = RuntimeState.READY
        return self._event("ready")

    def start(self) -> Mapping[str, Any]:
        self.calls.append(("start", threading.get_ident()))
        self._state = RuntimeState.RUNNING
        return self._event("running")

    def advance(self, steps: int = 1) -> Mapping[str, Any]:
        assert steps >= 1
        self.calls.append(("advance", threading.get_ident()))
        self._sequence += steps
        self.advance_seen.set()
        return self._event("snapshot")

    def pause(self) -> Mapping[str, Any]:
        self.calls.append(("pause", threading.get_ident()))
        self._state = RuntimeState.PAUSED
        return self._event("paused")

    def reset(self) -> Mapping[str, Any]:
        self.calls.append(("reset", threading.get_ident()))
        self._reset_generation += 1
        self._sequence = 0
        return self._event("snapshot")

    def hold_controller_commands(self) -> None:
        self.calls.append(("hold", threading.get_ident()))

    def submit_from_pump(self) -> None:
        self.calls.append(("submit", threading.get_ident()))

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult:
        del controller_id, command
        self.submit_from_pump()
        return CommandSubmitResult.ACCEPTED

    def stop(self, *, failure_reason: str | None = None) -> Mapping[str, Any]:
        del failure_reason
        self.calls.append(("stop", threading.get_ident()))
        self._state = RuntimeState.STOPPED
        return self._event("stopped")

    def stop_runtime_before_visual(
        self,
        *,
        failure_reason: str | None = None,
    ) -> Mapping[str, Any]:
        del failure_reason
        self.calls.append(("stop_runtime_before_visual", threading.get_ident()))
        self._state = RuntimeState.STOPPED
        return self._event("stopped")

    def finalize_visual_after_terminal(self) -> int:
        self.calls.append(("finalize_visual_after_terminal", threading.get_ident()))
        return 0

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "model_generation": 1,
            "reset_generation": self._reset_generation,
            "sequence": self._sequence,
            "sim_time_ns": self._sequence * 2_000_000,
        }


class _PhaseRecordingCoordinator(_RecordingCoordinator):
    def __init__(self, *, physics_step: int) -> None:
        super().__init__()
        self._physics_step = physics_step
        self.realtime_advance_seen = threading.Event()

    def advance(self, steps: int = 1) -> Mapping[str, Any]:
        self.calls.append((f"advance:{steps}", threading.get_ident()))
        self._physics_step += steps
        self._sequence += steps
        return self._event("snapshot")

    def advance_realtime(self, steps: int = 1) -> Mapping[str, Any]:
        self.calls.append((f"advance_realtime:{steps}", threading.get_ident()))
        self._physics_step += steps
        self._sequence += steps
        self.realtime_advance_seen.set()
        return self._event("snapshot")

    def _event(self, event: str) -> dict[str, Any]:
        accepted = super()._event(event)
        accepted["physics_step"] = self._physics_step
        return accepted


class _SlowPhaseRecordingCoordinator(_PhaseRecordingCoordinator):
    def __init__(self, *, physics_step: int, delay_s: float) -> None:
        super().__init__(physics_step=physics_step)
        self._delay_s = delay_s
        self.realtime_steps: list[int] = []
        self.enough_realtime_calls = threading.Event()

    def advance_realtime(self, steps: int = 1) -> Mapping[str, Any]:
        self.realtime_steps.append(steps)
        if len(self.realtime_steps) >= 4:
            self.enough_realtime_calls.set()
        time.sleep(self._delay_s)
        return super().advance_realtime(steps)


class _RecordingControlPump:
    def __init__(self, coordinator: _RecordingCoordinator) -> None:
        self._coordinator = coordinator
        self._received: deque[object] = deque()
        self._lock = threading.Lock()
        self.receiver_thread_id: int | None = None
        self.clear_reasons: list[str] = []
        self.motion_clear_reasons: list[str] = []
        self.requests: deque[InteractiveControlRequest] = deque()
        self.current_events: list[Mapping[str, Any]] = []
        self.status_events: list[tuple[Mapping[str, Any], str, int]] = []

    def receive(self, payload: object) -> None:
        self.receiver_thread_id = threading.get_ident()
        with self._lock:
            self._received.append(payload)

    def process_runtime_requests(self) -> InteractiveControlRequest | None:
        self._coordinator.calls.append(("runtime_requests", threading.get_ident()))
        try:
            return self.requests.popleft()
        except IndexError:
            return None

    def process_before_advance(self, current_event: Mapping[str, Any]) -> None:
        self.current_events.append(current_event)
        self._coordinator.calls.append(("pump", threading.get_ident()))
        with self._lock:
            payload = self._received.popleft() if self._received else None
        if payload is not None:
            self._coordinator.submit_from_pump()

    def publish_status_after_advance(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        thread_id = threading.get_ident()
        self.status_events.append((dict(snapshot), runtime_state, thread_id))
        self._coordinator.calls.append((f"status:{runtime_state}", thread_id))
        return 1

    def publish_terminal_status_after_stop(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        thread_id = threading.get_ident()
        self.status_events.append((dict(snapshot), runtime_state, thread_id))
        self._coordinator.calls.append((f"terminal_status:{runtime_state}", thread_id))
        return 1

    def clear(self, *, reason: str) -> None:
        self.clear_reasons.append(reason)
        self._coordinator.calls.append((f"clear:{reason}", threading.get_ident()))
        with self._lock:
            self._received.clear()

    def clear_motion(self, *, reason: str) -> None:
        self.motion_clear_reasons.append(reason)
        self._coordinator.calls.append((f"clear_motion:{reason}", threading.get_ident()))
        with self._lock:
            self._received.clear()


def test_start_aligns_physics_step_before_realtime_chunks() -> None:
    coordinator = _PhaseRecordingCoordinator(physics_step=3)
    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.001,
    )
    session.prepare()

    started = session.start()
    assert coordinator.realtime_advance_seen.wait(1.0)
    session.pause()
    session.stop()

    names = [name for name, _thread_id in coordinator.calls]
    assert started["physics_step"] == 5
    assert names.index("advance:2") < names.index("advance_realtime:5")


def test_owner_thread_affinity_is_bound_before_advance_and_restored_on_exit() -> None:
    coordinator = _PhaseRecordingCoordinator(physics_step=0)
    affinity_events: list[tuple[str, int, int]] = []

    @contextmanager
    def affinity_scope(mask: int):
        affinity_events.append(("bind", mask, threading.get_ident()))
        try:
            yield
        finally:
            affinity_events.append(("restore", mask, threading.get_ident()))

    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.005,
        owner_thread_affinity_mask=0b0011,
        thread_affinity_scope_factory=affinity_scope,
    )
    session.prepare()
    session.start()
    assert coordinator.realtime_advance_seen.wait(1.0)

    session.stop()

    assert [event[0] for event in affinity_events] == ["bind", "restore"]
    owner_thread_id = next(
        thread_id
        for name, thread_id in coordinator.calls
        if name == "advance_realtime:5"
    )
    assert affinity_events == [
        ("bind", 0b0011, owner_thread_id),
        ("restore", 0b0011, owner_thread_id),
    ]


def test_owner_thread_affinity_partial_enter_failure_restores_before_failure() -> None:
    coordinator = _PhaseRecordingCoordinator(physics_step=0)
    affinity_events: list[tuple[str, int, int]] = []

    class PartialEnterScope:
        def __init__(self, mask: int) -> None:
            self._mask = mask

        def __enter__(self) -> None:
            affinity_events.append(("bind", self._mask, threading.get_ident()))
            raise RuntimeError("affinity enter failed")

        def __exit__(self, *_exc: object) -> bool:
            affinity_events.append(("restore", self._mask, threading.get_ident()))
            return False

    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.005,
        owner_thread_affinity_mask=0b0011,
        thread_affinity_scope_factory=PartialEnterScope,
    )
    session.prepare()
    session.start()
    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.FAILED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.FAILED
    assert session.last_error == "RuntimeError: affinity enter failed"
    assert [event[0] for event in affinity_events] == ["bind", "restore"]
    assert affinity_events[0][2] == affinity_events[1][2]
    assert not coordinator.realtime_advance_seen.is_set()
    session.stop()


def test_owner_thread_affinity_restore_failure_preserves_runtime_failure() -> None:
    class FailingCoordinator(_PhaseRecordingCoordinator):
        def advance_realtime(self, steps: int = 1) -> Mapping[str, Any]:
            del steps
            raise ValueError("advance failed")

    class FailingExitScope:
        def __init__(self, mask: int) -> None:
            self._mask = mask

        def __enter__(self) -> None:
            return None

        def __exit__(self, *_exc: object) -> bool:
            raise RuntimeError(f"restore failed for mask {self._mask}")

    coordinator = FailingCoordinator(physics_step=0)
    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.005,
        owner_thread_affinity_mask=0b0011,
        thread_affinity_scope_factory=FailingExitScope,
    )
    session.prepare()
    session.start()
    deadline = time.monotonic() + 1.0
    while session.background_thread_alive and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.FAILED
    assert session.last_error is not None
    assert session.last_error.startswith("ValueError: advance failed")
    assert "restore failed for mask 3" in session.last_error
    session.stop()


def test_control_quiesce_preserves_both_failures_on_python310() -> None:
    class FailingCoordinator(_RecordingCoordinator):
        def hold_controller_commands(self) -> None:
            raise RuntimeError("hold failed")

    class FailingPump(_RecordingControlPump):
        def clear(self, *, reason: str) -> None:
            del reason
            raise ValueError("clear failed")

    coordinator = FailingCoordinator()
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=FailingPump(coordinator),
    )

    with pytest.raises(RuntimeError, match="hold failed") as error:
        session._quiesce_control("test")

    assert "clear failed" in "\n".join(getattr(error.value, "__notes__", ()))


def test_realtime_scheduler_catches_up_with_atomic_base_ticks() -> None:
    coordinator = _SlowPhaseRecordingCoordinator(
        physics_step=0,
        delay_s=0.012,
    )
    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.005,
    )
    session.prepare()

    session.start()
    assert coordinator.enough_realtime_calls.wait(1.0)
    session.pause()
    session.stop()

    assert coordinator.realtime_steps[0] == 5
    assert len(coordinator.realtime_steps) >= 4
    assert set(coordinator.realtime_steps) == {5}


def test_realtime_scheduler_does_not_repay_paused_wall_time() -> None:
    coordinator = _SlowPhaseRecordingCoordinator(
        physics_step=0,
        delay_s=0.001,
    )
    session = InteractiveSimulationSession(
        coordinator,
        steps_per_tick=5,
        advance_wait_s=0.005,
    )
    session.prepare()
    session.start()
    assert coordinator.realtime_advance_seen.wait(1.0)
    session.pause()

    coordinator.realtime_steps.clear()
    coordinator.realtime_advance_seen.clear()
    time.sleep(0.05)
    session.start()
    assert coordinator.realtime_advance_seen.wait(1.0)
    session.pause()
    session.stop()

    assert coordinator.realtime_steps[0] == 5


def test_receiver_only_enqueues_and_owner_pumps_before_advance() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()

    receiver = threading.Thread(target=pump.receive, args=({"linear_x": 1.0},))
    receiver.start()
    receiver.join()

    session.start()
    assert coordinator.advance_seen.wait(1.0)
    session.pause()
    session.stop()

    names = [name for name, _thread_id in coordinator.calls]
    first_advance = names.index("advance")
    assert names[:first_advance][-3:] == ["runtime_requests", "pump", "submit"]
    assert names[first_advance + 1] == "status:RUNNING"
    owner_thread_id = coordinator.calls[first_advance][1]
    assert next(thread_id for name, thread_id in coordinator.calls if name == "pump") == owner_thread_id
    assert next(thread_id for name, thread_id in coordinator.calls if name == "submit") == owner_thread_id
    assert pump.current_events[0]["event"] == "running"
    assert pump.current_events[0]["model_generation"] == 1
    assert pump.current_events[0]["reset_generation"] == 0
    assert pump.current_events[0]["sequence"] == 0
    assert pump.current_events[0]["sim_time_ns"] == 0
    assert pump.receiver_thread_id is not None
    assert pump.receiver_thread_id != owner_thread_id
    assert pump.status_events[0][0]["event"] == "snapshot"
    assert pump.status_events[0][1] == "RUNNING"
    assert pump.status_events[0][2] == owner_thread_id


def test_status_is_published_on_owner_thread_after_advance_and_while_paused() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)
    session.pause()

    deadline = time.monotonic() + 1.0
    while (
        not any(state == "PAUSED" for _event, state, _thread in pump.status_events)
        and time.monotonic() < deadline
    ):
        time.sleep(0.001)

    running = next(item for item in pump.status_events if item[1] == "RUNNING")
    paused = next(item for item in pump.status_events if item[1] == "PAUSED")
    names = [name for name, _thread in coordinator.calls]
    assert names.index("advance") < names.index("status:RUNNING")
    assert running[2] == paused[2]
    assert paused[0]["event"] == "paused"
    session.stop()


def test_paused_owner_processes_resume_and_exit_requests() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)
    session.pause()

    coordinator.advance_seen.clear()
    pump.requests.append(InteractiveControlRequest.RESUME)
    assert coordinator.advance_seen.wait(1.0)
    owner_thread_id = next(
        thread_id
        for name, thread_id in reversed(coordinator.calls)
        if name == "advance"
    )

    pump.requests.append(InteractiveControlRequest.EXIT)
    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.STOPPED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.STOPPED
    assert not session.background_thread_alive
    assert [thread_id for name, thread_id in coordinator.calls if name == "start"][-1] == owner_thread_id
    assert [
        thread_id
        for name, thread_id in coordinator.calls
        if name == "stop_runtime_before_visual"
    ][-1] == owner_thread_id
    assert [
        thread_id
        for name, thread_id in coordinator.calls
        if name == "finalize_visual_after_terminal"
    ][-1] == owner_thread_id
    terminal = [
        (name, thread_id)
        for name, thread_id in coordinator.calls
        if name == "terminal_status:STOPPED"
    ]
    assert terminal == [("terminal_status:STOPPED", owner_thread_id)]
    session.stop()


def test_public_resume_discards_motion_received_while_paused() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)
    session.pause()

    pump.receive({"linear_x": 1.0})
    coordinator.advance_seen.clear()
    session.start()
    assert coordinator.advance_seen.wait(1.0)
    time.sleep(0.01)

    assert "submit" not in [name for name, _thread_id in coordinator.calls]
    assert pump.motion_clear_reasons == ["resume"]

    pump.receive({"linear_x": 0.5})
    deadline = time.monotonic() + 1.0
    while "submit" not in [name for name, _thread_id in coordinator.calls] and time.monotonic() < deadline:
        time.sleep(0.001)
    assert "submit" in [name for name, _thread_id in coordinator.calls]
    session.pause()
    session.stop()


def test_runtime_resume_discards_paused_motion_but_preserves_exit_request() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)
    session.pause()

    pump.receive({"linear_x": 1.0})
    pump.requests.extend(
        (InteractiveControlRequest.RESUME, InteractiveControlRequest.EXIT)
    )
    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.STOPPED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.STOPPED
    assert "submit" not in [name for name, _thread_id in coordinator.calls]
    assert pump.motion_clear_reasons == ["resume"]
    session.stop()


def test_pause_reset_and_stop_clear_input_then_hold_controller() -> None:
    coordinator = _RecordingCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)

    session.pause()
    session.reset()
    session.stop()

    names = [name for name, _thread_id in coordinator.calls]
    pause_index = names.index("pause")
    reset_index = names.index("reset")
    stop_index = names.index("stop")
    assert names[pause_index - 2 : pause_index + 1] == ["clear:pause", "hold", "pause"]
    assert names[reset_index - 2 : reset_index + 1] == ["clear:reset", "hold", "reset"]
    assert names[stop_index - 2 : stop_index + 1] == ["clear:stop", "hold", "stop"]
    assert pump.clear_reasons == ["pause", "reset", "stop"]


def test_session_without_pump_still_holds_on_pause_and_stop() -> None:
    coordinator = _RecordingCoordinator()
    session = InteractiveSimulationSession(
        coordinator,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.advance_seen.wait(1.0)

    session.pause()
    session.start()
    session.pause()
    session.stop()

    lifecycle = [
        name
        for name, _thread_id in coordinator.calls
        if name in {"hold", "pause", "start", "stop"}
    ]
    assert lifecycle[-7:] == [
        "hold",
        "pause",
        "start",
        "hold",
        "pause",
        "hold",
        "stop",
    ]


def test_control_pump_fails_closed_without_a_stamped_current_event() -> None:
    class InvalidStartEventCoordinator(_RecordingCoordinator):
        def start(self) -> Mapping[str, Any]:
            event = dict(super().start())
            del event["sim_time_ns"]
            return event

    coordinator = InvalidStartEventCoordinator()
    pump = _RecordingControlPump(coordinator)
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()

    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.FAILED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.FAILED
    assert session.last_error == "CoordinatorError: control pump event has invalid sim_time_ns"
    assert "advance" not in [name for name, _thread_id in coordinator.calls]
    session.stop()


class _UnusedVisualProcess:
    def start(self, **kwargs: Any) -> None:
        del kwargs

    def poll(self) -> int | None:
        return None

    def terminate(self) -> None:
        return None


class _UnusedPublisher:
    def publish(self, snapshot: Mapping[str, Any]) -> int:
        del snapshot
        return 1

    def close(self) -> None:
        return None


def test_session_host_exposes_submit_and_hold_owner_seams() -> None:
    coordinator = _RecordingCoordinator()
    host = SessionHost(
        coordinator=coordinator,  # type: ignore[arg-type]
        unreal_process=_UnusedVisualProcess(),
        publisher=_UnusedPublisher(),
    )
    command = ControllerCommand(
        channel_id="thunder_01.control.base_twist",
        instance_id="thunder_01",
        generation=GenerationStamp(model_generation=1, reset_generation=0),
        sequence=1,
        apply_time_ns=0,
        payload={"linear_x": 0.1},
    )

    result = host.submit_controller_command("thunder_01.thunderv4_locomotion", command)
    host.hold_controller_commands()

    assert result is CommandSubmitResult.ACCEPTED
    assert [name for name, _thread_id in coordinator.calls] == ["submit", "hold"]
