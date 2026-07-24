from __future__ import annotations

import threading
import time

from nav.commands.operator_motion import OperatorMotion


class RecordingCommands:
    def __init__(self) -> None:
        self.teleop_calls: list[tuple[float, float, float, str | None]] = []
        self.called = threading.Event()

    def send_teleop(
        self,
        vx: float,
        vy: float,
        wz: float,
        request_id: str | None = None,
    ) -> bool:
        self.teleop_calls.append((vx, vy, wz, request_id))
        self.called.set()
        return True


class BlockingCommands(RecordingCommands):
    def __init__(self) -> None:
        super().__init__()
        self.first_started = threading.Event()
        self.unblock_first = threading.Event()
        self.second_called = threading.Event()

    def send_teleop(
        self,
        vx: float,
        vy: float,
        wz: float,
        request_id: str | None = None,
    ) -> bool:
        self.teleop_calls.append((vx, vy, wz, request_id))
        self.called.set()
        if len(self.teleop_calls) == 1:
            self.first_started.set()
            assert self.unblock_first.wait(1.0)
        elif len(self.teleop_calls) == 2:
            self.second_called.set()
        return True


class ManualClock:
    def __init__(self) -> None:
        self.value = 0.0
        self.checked = threading.Event()

    def __call__(self) -> float:
        self.checked.set()
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def wait_until(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_open_grants_one_operator_session_without_sending_motion() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        first = motion.open(source_id="ws:alice", adapter="websocket", lease_ttl_s=1.0)
        conflict = motion.open(source_id="board:bob", adapter="motion_board", lease_ttl_s=1.0)

        assert first["accepted"] is True
        assert first["stage"] == "opened"
        assert first["session_id"]
        assert conflict["accepted"] is False
        assert conflict["stage"] == "rejected"
        assert conflict["reason"] == "lease_conflict"
        assert commands.teleop_calls == []
    finally:
        motion.stop()


def test_expired_session_confirms_zero_before_new_source_acquires_control() -> None:
    clock = ManualClock()
    commands = RecordingCommands()
    motion = OperatorMotion(clock=clock)
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        first = motion.open(source_id="ws:alice", adapter="websocket", lease_ttl_s=1.0)
        clock.advance(1.1)

        second = motion.open(source_id="board:bob", adapter="motion_board", lease_ttl_s=1.0)

        assert second["accepted"] is True
        assert second["stage"] == "opened"
        assert second["session_id"] != first["session_id"]
        assert commands.teleop_calls == [(0.0, 0.0, 0.0, commands.teleop_calls[0][3])]
    finally:
        motion.stop()


def test_release_waits_for_inflight_then_confirms_zero_and_drops_pending() -> None:
    commands = BlockingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket")
        session_id = str(session["session_id"])
        motion.submit(session_id, 0.1, 0.0, 0.0, deadman=True, sequence=1, request_id="joy-1")
        assert commands.first_started.wait(1.0)
        motion.submit(session_id, 0.2, 0.0, 0.0, deadman=True, sequence=2, request_id="joy-2")

        result: dict[str, dict] = {}
        release_started = threading.Event()

        def release() -> None:
            release_started.set()
            result["receipt"] = motion.release(
                session_id,
                disposition="hold",
                reason="deadman_released",
                request_id="release-1",
                timeout_s=1.0,
            )

        release_thread = threading.Thread(target=release)
        release_thread.start()
        assert release_started.wait(1.0)
        assert commands.teleop_calls == [(0.1, 0.0, 0.0, "joy-1")]

        commands.unblock_first.set()
        release_thread.join(timeout=1.0)

        assert release_thread.is_alive() is False
        assert commands.teleop_calls == [
            (0.1, 0.0, 0.0, "joy-1"),
            (0.0, 0.0, 0.0, "release-1"),
        ]
        assert result["receipt"]["accepted"] is True
        assert result["receipt"]["stage"] == "held"
        assert result["receipt"]["zero_confirmed"] is True
    finally:
        commands.unblock_first.set()
        motion.stop()


def test_submit_reports_queued_before_native_admission() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket")

        receipt = motion.submit(
            session_id=str(session["session_id"]),
            sequence=1,
            request_id="joy-1",
            vx_mps=0.2,
            vy_mps=-0.1,
            wz_radps=0.3,
            deadman=True,
            ttl_ms=350,
        )

        assert receipt["accepted"] is True
        assert receipt["stage"] == "queued"
        assert receipt["request_id"] == "joy-1"
        assert commands.called.wait(1.0)
        assert commands.teleop_calls == [(0.2, -0.1, 0.3, "joy-1")]
    finally:
        motion.stop()


def test_deadman_release_confirms_zero_and_keeps_session() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket")
        session_id = str(session["session_id"])

        receipt = motion.submit(
            session_id,
            0.4,
            0.0,
            0.0,
            deadman=False,
            request_id="deadman-1",
        )

        assert receipt["accepted"] is True
        assert receipt["stage"] == "held"
        assert receipt["zero_confirmed"] is True
        assert commands.teleop_calls == [(0.0, 0.0, 0.0, "deadman-1")]

        resumed = motion.submit(session_id, 0.1, 0.0, 0.0, deadman=True, request_id="joy-after-hold")
        assert resumed["accepted"] is True
        assert resumed["stage"] == "queued"
    finally:
        motion.stop()


def test_submit_keeps_only_latest_sample_while_native_ack_is_inflight() -> None:
    commands = BlockingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket")
        session_id = str(session["session_id"])
        motion.submit(session_id, 0.1, 0.0, 0.0, deadman=True, sequence=1, request_id="joy-1")
        assert commands.first_started.wait(1.0)

        motion.submit(session_id, 0.2, 0.0, 0.0, deadman=True, sequence=2, request_id="joy-2")
        motion.submit(session_id, 0.3, 0.0, 0.0, deadman=True, sequence=3, request_id="joy-3")
        commands.unblock_first.set()

        assert commands.second_called.wait(1.0)
        assert commands.teleop_calls == [
            (0.1, 0.0, 0.0, "joy-1"),
            (0.3, 0.0, 0.0, "joy-3"),
        ]
    finally:
        commands.unblock_first.set()
        motion.stop()


def test_duplicate_and_out_of_order_sequences_do_not_send_again() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="board:remote", adapter="motion_board")
        session_id = str(session["session_id"])
        accepted = motion.submit(
            session_id,
            0.2,
            0.0,
            0.0,
            deadman=True,
            sequence=2,
            request_id="board-2",
        )
        assert accepted["accepted"] is True
        assert commands.called.wait(1.0)

        duplicate = motion.submit(
            session_id,
            0.3,
            0.0,
            0.0,
            deadman=True,
            sequence=2,
            request_id="board-2-duplicate",
        )
        stale = motion.submit(
            session_id,
            0.4,
            0.0,
            0.0,
            deadman=True,
            sequence=1,
            request_id="board-1-late",
        )

        assert duplicate["accepted"] is False
        assert duplicate["reason"] == "sequence_not_increasing"
        assert stale["accepted"] is False
        assert stale["reason"] == "sequence_not_increasing"
        assert commands.teleop_calls == [(0.2, 0.0, 0.0, "board-2")]
    finally:
        motion.stop()


def test_expired_pending_sample_is_dropped_without_faulting_fresh_input() -> None:
    clock = ManualClock()
    commands = BlockingCommands()
    motion = OperatorMotion(clock=clock)
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket", lease_ttl_s=10.0)
        session_id = str(session["session_id"])
        motion.submit(
            session_id,
            0.1,
            0.0,
            0.0,
            deadman=True,
            sequence=1,
            request_id="joy-1",
            ttl_ms=5000,
        )
        assert commands.first_started.wait(1.0)
        motion.submit(
            session_id,
            0.2,
            0.0,
            0.0,
            deadman=True,
            sequence=2,
            request_id="joy-expired",
            ttl_ms=100,
        )

        clock.advance(1.0)
        clock.checked.clear()
        commands.unblock_first.set()
        assert clock.checked.wait(1.0)
        assert wait_until(lambda: motion.health()["stream"]["inflight"] is False)
        assert commands.teleop_calls == [(0.1, 0.0, 0.0, "joy-1")]

        fresh = motion.submit(
            session_id,
            0.3,
            0.0,
            0.0,
            deadman=True,
            sequence=3,
            request_id="joy-fresh",
        )
        assert fresh["accepted"] is True
        assert commands.second_called.wait(1.0)
        assert commands.teleop_calls[-1] == (0.3, 0.0, 0.0, "joy-fresh")
    finally:
        commands.unblock_first.set()
        motion.stop()


def test_health_reports_ingress_truth_without_claiming_safe_output() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="ws:alice", adapter="websocket")

        health = motion.health()

        assert health["state"] == "ready"
        assert health["command_module_ready"] is True
        assert health["accepting"] is True
        assert health["session"]["active"] is True
        assert health["session"]["session_id"] == session["session_id"]
        assert health["session"]["source_id"] == "ws:alice"
        assert health["session"]["adapter"] == "websocket"
        assert health["stream"] == {
            "pending": False,
            "inflight": False,
            "last_native_request_id": None,
            "last_error": None,
        }
        assert "safe_output" not in health
        assert "driver_acked" not in health
    finally:
        motion.stop()


def test_module_stop_confirms_zero_for_an_active_session() -> None:
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    try:
        session = motion.open(source_id="board:remote", adapter="motion_board")
        motion.submit(
            str(session["session_id"]),
            0.2,
            0.0,
            0.0,
            deadman=True,
            request_id="board-motion",
        )
        assert commands.called.wait(1.0)

        motion.stop()

        assert commands.teleop_calls[-1][:3] == (0.0, 0.0, 0.0)
        assert motion.health()["state"] == "stopped"
    finally:
        motion.stop()
