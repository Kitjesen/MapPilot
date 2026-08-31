from __future__ import annotations

import inspect
from collections import deque
from dataclasses import FrozenInstanceError
from typing import Callable

import pytest

from sim.scripts.mujoco import driver_bridge_session as session_module
from sim.scripts.mujoco.driver_bridge_session import (
    DriverBridgeSession,
    DriverBridgeSessionError,
)

BRIDGE_BOOT_ID = "a" * 32
PRODUCT_SESSION_ID = "b" * 32


class ScriptedTransport:
    def __init__(self, *incoming: str | Callable[[ScriptedTransport], str]) -> None:
        self.incoming = deque(incoming)
        self.sent: list[str] = []
        self.close_calls = 0

    def send_line(self, line: str) -> None:
        self.sent.append(line)

    def recv_line(self) -> str:
        item = self.incoming.popleft()
        return item(self) if callable(item) else item

    def close(self) -> None:
        self.close_calls += 1


class SequenceClock:
    def __init__(self, *values: float) -> None:
        self.values = deque(values)

    def __call__(self) -> float:
        return self.values.popleft()


def _activation_command(transport: ScriptedTransport) -> str:
    _, bridge_boot_id, controller_boot_id, _ = transport.sent[-1].split("\t")
    return (
        "LT_DRIVER_COMMAND_V2"
        f"\t{bridge_boot_id}\t{controller_boot_id}\t1"
        "\tactivation_zero\t-\t0\t0\t0\t0"
    )


def _ready_for_last_sent_command(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    applied = next(
        line
        for line in reversed(transport.sent)
        if line.startswith("LT_DRIVER_APPLIED_V2\t")
    ).split("\t")
    return (
        f"LT_DRIVER_READY_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        f"\t{applied[3]}\t{applied[5]}\t{applied[6]}"
    )


def _nav_command(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_COMMAND_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        f"\t2\tnav\t{PRODUCT_SESSION_ID}:1234:567890\t91\t1\t-0.5\t0.25"
    )


def _deactivate_command(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_COMMAND_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t3\tdeactivate_zero\t-\t0\t0\t0\t0"
    )


def _late_nav_command(transport: ScriptedTransport) -> str:
    return _nav_command(transport).replace("\t2\tnav\t", "\t3\tnav\t").replace(
        "\t91\t", "\t92\t"
    )


def _pending_writer_fault_zero(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_COMMAND_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t2\twriter_fault_zero\t-\t0\t0\t0\t0"
    )


def _pending_safety_zero(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_COMMAND_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t3\tsafety_zero\t-\t0\t0\t0\t0"
    )


def _stopped(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_STOPPED_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t3\t5\tdeactivate_zero"
    )


def _stopped_after_pending_nav(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_STOPPED_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t3\t4\tdeactivate_zero"
    )


def _active_session(
    *incoming: str | Callable[[ScriptedTransport], str],
) -> tuple[DriverBridgeSession, ScriptedTransport]:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
        _ready_for_last_sent_command,
        *incoming,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )
    activation = session.activate()
    session.complete_step(activation, step_seq=1)
    session.heartbeat(step_seq=2)
    session.confirm_ready()
    return session, transport


def test_activate_returns_typed_immutable_physical_zero() -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    command = session.activate()

    assert transport.sent == [
        "LT_DRIVER_ACTIVATE_V2"
        f"\t{BRIDGE_BOOT_ID}\t{session.controller_boot_id}\t1"
    ]
    assert command.bridge_boot_id == BRIDGE_BOOT_ID
    assert command.controller_boot_id == session.controller_boot_id
    assert command.bridge_command_seq == 1
    assert command.kind == "activation_zero"
    assert command.producer_boot_id == ""
    assert command.output_sequence == 0
    assert (command.walk_x, command.walk_y, command.walk_z) == (0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        command.kind = "nav"  # type: ignore[misc]


def test_session_preserves_physical_apply_and_terminal_ack_order() -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
        _ready_for_last_sent_command,
        _nav_command,
        _ready_for_last_sent_command,
        _deactivate_command,
        _stopped,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    activation = session.activate()
    session.complete_step(activation, step_seq=1)
    session.heartbeat(step_seq=2)
    session.confirm_ready()
    nav = session.receive_command()
    session.complete_step(nav, step_seq=3)
    session.heartbeat(step_seq=4)
    session.confirm_ready()
    deactivation = session.begin_deactivate()
    session.complete_step(deactivation, step_seq=5)
    stopped = session.wait_stopped()

    assert nav.kind == "nav"
    assert nav.producer_boot_id == f"{PRODUCT_SESSION_ID}:1234:567890"
    assert nav.output_sequence == 91
    assert (nav.walk_x, nav.walk_y, nav.walk_z) == (1.0, -0.5, 0.25)
    assert stopped.bridge_boot_id == BRIDGE_BOOT_ID
    assert stopped.controller_boot_id == session.controller_boot_id
    assert stopped.bridge_command_seq == deactivation.bridge_command_seq == 3
    assert stopped.applied_step_seq == 5
    assert stopped.kind == "deactivate_zero"
    assert stopped.producer_boot_id == ""
    assert stopped.output_sequence == 0
    assert (stopped.walk_x, stopped.walk_y, stopped.walk_z) == (0.0, 0.0, 0.0)
    assert stopped.terminal_ack is True
    with pytest.raises(FrozenInstanceError):
        stopped.terminal_ack = False  # type: ignore[misc]

    prefixes = [line.split("\t", 1)[0] for line in transport.sent]
    assert prefixes == [
        "LT_DRIVER_ACTIVATE_V2",
        "LT_DRIVER_APPLIED_V2",
        "LT_DRIVER_HEARTBEAT_V2",
        "LT_DRIVER_APPLIED_V2",
        "LT_DRIVER_HEARTBEAT_V2",
        "LT_DRIVER_DEACTIVATE_V2",
        "LT_DRIVER_APPLIED_V2",
    ]


def test_close_only_closes_transport_and_is_idempotent() -> None:
    transport = ScriptedTransport()
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    session.close()
    session.close()

    assert transport.close_calls == 1
    assert transport.sent == []
    assert list(transport.incoming) == []
    with pytest.raises(RuntimeError, match="closed"):
        session.activate()


def _malformed_nav(case: str) -> Callable[[ScriptedTransport], str]:
    def build(transport: ScriptedTransport) -> str:
        parts = _nav_command(transport).split("\t")
        mutations = {
            "wrong_bridge": (1, "c" * 32),
            "wrong_controller": (2, "d" * 32),
            "stale_sequence": (3, "1"),
            "unknown_kind": (4, "other"),
            "wrong_host": (5, f"{'c' * 32}:1234:567890"),
            "missing_process": (5, PRODUCT_SESSION_ID),
            "zero_process": (5, f"{PRODUCT_SESSION_ID}:0:567890"),
            "zero_start": (5, f"{PRODUCT_SESSION_ID}:1234:0"),
            "noncanonical_process": (5, f"{PRODUCT_SESSION_ID}:01234:567890"),
            "noncanonical_start": (5, f"{PRODUCT_SESSION_ID}:1234:0567890"),
            "missing_output": (6, "0"),
            "negative_zero": (7, "-0.0"),
            "nan": (8, "nan"),
            "infinity": (9, "inf"),
            "out_of_range": (7, "1.0000001"),
            "unknown_prefix": (0, "LT_DRIVER_OTHER_V2"),
        }
        if case == "extra_field":
            parts.append("extra")
        else:
            index, value = mutations[case]
            parts[index] = value
        return "\t".join(parts)

    return build


@pytest.mark.parametrize(
    "case",
    (
        "wrong_bridge",
        "wrong_controller",
        "stale_sequence",
        "unknown_kind",
        "wrong_host",
        "missing_process",
        "zero_process",
        "zero_start",
        "noncanonical_process",
        "noncanonical_start",
        "missing_output",
        "negative_zero",
        "nan",
        "infinity",
        "out_of_range",
        "unknown_prefix",
        "extra_field",
    ),
)
def test_receive_command_rejects_noncanonical_or_unbound_nav(case: str) -> None:
    session, _ = _active_session(_malformed_nav(case))

    with pytest.raises(DriverBridgeSessionError):
        session.receive_command()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.receive_command()


@pytest.mark.parametrize("step_seq", (False, 0, -1, (1 << 64)))
def test_invalid_applied_step_fault_closes_the_session(step_seq: int) -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )
    activation = session.activate()

    with pytest.raises(DriverBridgeSessionError, match="step_seq"):
        session.complete_step(activation, step_seq=step_seq)
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.complete_step(activation, step_seq=1)


@pytest.mark.parametrize("eof", ("", None, EOFError("closed")))
def test_eof_or_transport_failure_fault_closes_without_peer_evidence(eof) -> None:
    def recv_line():
        if isinstance(eof, BaseException):
            raise eof
        return eof

    transport = ScriptedTransport()
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    with pytest.raises(DriverBridgeSessionError):
        session.activate()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.activate()


def test_absolute_operation_deadline_expires_after_a_slow_callback() -> None:
    transport = ScriptedTransport(f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}")
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
        operation_timeout_s=1.0,
        clock=SequenceClock(0.0, 1.0),
    )

    with pytest.raises(DriverBridgeSessionError, match="deadline"):
        session.activate()
    assert transport.sent == []


def _malformed_ready(case: str) -> Callable[[ScriptedTransport], str]:
    def build(transport: ScriptedTransport) -> str:
        parts = _ready_for_last_sent_command(transport).split("\t")
        mutations = {
            "wrong_bridge": (1, "c" * 32),
            "wrong_controller": (2, "d" * 32),
            "wrong_sequence": (3, "2"),
            "wrong_producer": (4, f"{PRODUCT_SESSION_ID}:1234:567890"),
            "wrong_output": (5, "91"),
            "zero_sequence": (3, "0"),
            "negative_sequence": (3, "-1"),
            "unknown_prefix": (0, "LT_DRIVER_OTHER_V2"),
        }
        if case == "extra_field":
            parts.append("extra")
        elif case == "fault":
            return (
                f"LT_DRIVER_FAULT_V2\t{BRIDGE_BOOT_ID}"
                f"\t{transport.sent[0].split(chr(9))[2]}\tapply_timeout"
            )
        else:
            index, value = mutations[case]
            parts[index] = value
        return "\t".join(parts)

    return build


@pytest.mark.parametrize(
    "case",
    (
        "wrong_bridge",
        "wrong_controller",
        "wrong_sequence",
        "wrong_producer",
        "wrong_output",
        "zero_sequence",
        "negative_sequence",
        "unknown_prefix",
        "extra_field",
        "fault",
    ),
)
def test_ready_must_ack_the_exact_physically_applied_zero(case: str) -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
        _malformed_ready(case),
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )
    activation = session.activate()
    session.complete_step(activation, step_seq=1)
    session.heartbeat(step_seq=2)

    expected_error = "apply_timeout" if case == "fault" else None
    with pytest.raises(DriverBridgeSessionError, match=expected_error):
        session.confirm_ready()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.confirm_ready()


def _direct_deactivate_command(transport: ScriptedTransport) -> str:
    controller_boot_id = transport.sent[0].split("\t")[2]
    return (
        f"LT_DRIVER_COMMAND_V2\t{BRIDGE_BOOT_ID}\t{controller_boot_id}"
        "\t2\tdeactivate_zero\t-\t0\t0\t0\t0"
    )


def _malformed_stopped(case: str) -> Callable[[ScriptedTransport], str]:
    def build(transport: ScriptedTransport) -> str:
        controller_boot_id = transport.sent[0].split("\t")[2]
        parts = [
            "LT_DRIVER_STOPPED_V2",
            BRIDGE_BOOT_ID,
            controller_boot_id,
            "2",
            "3",
            "deactivate_zero",
        ]
        mutations = {
            "wrong_bridge": (1, "c" * 32),
            "wrong_controller": (2, "d" * 32),
            "wrong_command": (3, "3"),
            "wrong_step": (4, "4"),
            "zero_command": (3, "0"),
            "zero_step": (4, "0"),
            "wrong_kind": (5, "safety_zero"),
            "unknown_prefix": (0, "LT_DRIVER_OTHER_V2"),
        }
        if case == "extra_field":
            parts.append("extra")
        elif case == "fault":
            return (
                f"LT_DRIVER_FAULT_V2\t{BRIDGE_BOOT_ID}"
                f"\t{controller_boot_id}\tapply_timeout"
            )
        else:
            index, value = mutations[case]
            parts[index] = value
        return "\t".join(parts)

    return build


@pytest.mark.parametrize(
    "case",
    (
        "wrong_bridge",
        "wrong_controller",
        "wrong_command",
        "wrong_step",
        "zero_command",
        "zero_step",
        "wrong_kind",
        "unknown_prefix",
        "extra_field",
        "fault",
    ),
)
def test_stopped_must_match_the_exact_deactivate_applied(case: str) -> None:
    session, transport = _active_session(
        _direct_deactivate_command,
        _malformed_stopped(case),
    )
    deactivation = session.begin_deactivate()
    session.complete_step(deactivation, step_seq=3)

    with pytest.raises(DriverBridgeSessionError):
        session.wait_stopped()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.wait_stopped()
    assert transport.close_calls == 0


def test_activation_command_sequence_starts_at_one_without_gaps() -> None:
    def skipped_activation(transport: ScriptedTransport) -> str:
        return _activation_command(transport).replace(
            "\t1\tactivation_zero", "\t2\tactivation_zero"
        )

    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        skipped_activation,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    with pytest.raises(DriverBridgeSessionError, match="command_seq"):
        session.activate()


def test_inflight_nav_must_be_applied_before_deactivation() -> None:
    session, transport = _active_session(_nav_command)
    nav = session.receive_command()
    sent_before = list(transport.sent)

    with pytest.raises(DriverBridgeSessionError, match="out of order"):
        session.begin_deactivate()
    assert transport.sent == sent_before

    session.complete_step(nav, step_seq=3)


def test_deactivate_drains_one_existing_pending_nav_before_terminal_zero() -> None:
    session, transport = _active_session(
        _nav_command,
        _deactivate_command,
        _stopped_after_pending_nav,
    )

    pending_nav = session.begin_deactivate()
    session.complete_step(pending_nav, step_seq=3)
    deactivation = session.begin_deactivate()
    session.complete_step(deactivation, step_seq=4)
    stopped = session.wait_stopped()

    assert pending_nav.kind == "nav"
    assert pending_nav.bridge_command_seq == 2
    assert deactivation.kind == "deactivate_zero"
    assert deactivation.bridge_command_seq == 3
    assert stopped.bridge_command_seq == 3
    assert stopped.applied_step_seq == 4
    prefixes = [line.split("\t", 1)[0] for line in transport.sent]
    assert prefixes == [
        "LT_DRIVER_ACTIVATE_V2",
        "LT_DRIVER_APPLIED_V2",
        "LT_DRIVER_HEARTBEAT_V2",
        "LT_DRIVER_DEACTIVATE_V2",
        "LT_DRIVER_APPLIED_V2",
        "LT_DRIVER_APPLIED_V2",
    ]


def test_deactivate_drains_a_command_queued_before_ready_exactly_once() -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
        _nav_command,
        _ready_for_last_sent_command,
        _deactivate_command,
        _stopped_after_pending_nav,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )
    activation = session.activate()
    session.complete_step(activation, step_seq=1)
    session.heartbeat(step_seq=2)
    session.confirm_ready()

    pending_nav = session.begin_deactivate()
    session.complete_step(pending_nav, step_seq=3)
    deactivation = session.begin_deactivate()
    session.complete_step(deactivation, step_seq=4)
    session.wait_stopped()

    assert (pending_nav.kind, deactivation.kind) == ("nav", "deactivate_zero")
    assert sum(
        line.startswith("LT_DRIVER_DEACTIVATE_V2\t") for line in transport.sent
    ) == 1


def test_deactivate_rejects_a_new_nav_after_the_existing_pending_nav() -> None:
    session, _ = _active_session(_nav_command, _late_nav_command)
    pending_nav = session.begin_deactivate()
    session.complete_step(pending_nav, step_seq=3)

    with pytest.raises(DriverBridgeSessionError, match="kind"):
        session.begin_deactivate()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.begin_deactivate()


@pytest.mark.parametrize(
    "invalid_pending",
    (
        lambda transport: _nav_command(transport).replace("\t2\tnav\t", "\t3\tnav\t"),
        _pending_writer_fault_zero,
        "",
    ),
    ids=("wrong-sequence", "wrong-kind", "eof"),
)
def test_deactivate_fault_closes_on_invalid_pending_command(invalid_pending) -> None:
    session, _ = _active_session(invalid_pending)

    with pytest.raises(DriverBridgeSessionError):
        session.begin_deactivate()
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.begin_deactivate()


def test_deactivate_fault_closes_on_nonadvancing_pending_applied_step() -> None:
    session, _ = _active_session(_nav_command)
    pending_nav = session.begin_deactivate()

    with pytest.raises(DriverBridgeSessionError, match="step_seq"):
        session.complete_step(pending_nav, step_seq=2)
    with pytest.raises(DriverBridgeSessionError, match="fault-closed"):
        session.begin_deactivate()


def test_outbound_records_obey_the_same_configured_byte_limit() -> None:
    transport = ScriptedTransport(f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}")
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
        max_line_bytes=64,
    )

    with pytest.raises(DriverBridgeSessionError, match="byte limit"):
        session.activate()
    assert transport.sent == []


@pytest.mark.parametrize(
    "hello",
    (
        f"LT_DRIVER_OTHER_V2\t{BRIDGE_BOOT_ID}",
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}\textra",
        f"LT_DRIVER_HELLO_V2\t{'A' * 32}",
        "LT_DRIVER_HELLO_V2\tabc",
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}\n",
        "LT_DRIVER_HELLO_V2\té",
        f"LT_DRIVER_FAULT_V2\t{BRIDGE_BOOT_ID}\t-\tprotocol_violation",
    ),
)
def test_hello_requires_one_exact_lowercase_bridge_identity(hello: str) -> None:
    transport = ScriptedTransport(hello)
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )

    with pytest.raises(DriverBridgeSessionError):
        session.activate()
    assert transport.sent == []


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("expected_product_session_id", ""),
        ("expected_product_session_id", "unsafe value"),
        ("expected_product_session_id", "-session"),
        ("expected_product_session_id", ".session"),
        ("expected_product_session_id", "a" * 64),
        ("expected_product_session_id", "unsafe/session"),
        ("operation_timeout_s", False),
        ("operation_timeout_s", 0.0),
        ("operation_timeout_s", float("nan")),
        ("operation_timeout_s", float("inf")),
        ("max_line_bytes", False),
        ("max_line_bytes", 0),
        ("max_line_bytes", 513),
    ),
)
def test_constructor_rejects_ambiguous_identity_timeout_or_limit(
    field: str, value
) -> None:
    transport = ScriptedTransport()
    arguments = {
        "send_line": transport.send_line,
        "recv_line": transport.recv_line,
        "close": transport.close,
        "expected_product_session_id": PRODUCT_SESSION_ID,
    }
    arguments[field] = value

    with pytest.raises((TypeError, ValueError)):
        DriverBridgeSession(**arguments)


def test_ready_confirmation_queues_a_command_emitted_before_ready() -> None:
    transport = ScriptedTransport(
        f"LT_DRIVER_HELLO_V2\t{BRIDGE_BOOT_ID}",
        _activation_command,
        _nav_command,
        _ready_for_last_sent_command,
    )
    session = DriverBridgeSession(
        send_line=transport.send_line,
        recv_line=transport.recv_line,
        close=transport.close,
        expected_product_session_id=PRODUCT_SESSION_ID,
    )
    activation = session.activate()
    session.complete_step(activation, step_seq=1)
    session.heartbeat(step_seq=2)

    session.confirm_ready()
    nav = session.receive_command()

    assert nav.kind == "nav"
    assert nav.bridge_command_seq == 2
    assert list(transport.incoming) == []


def test_ready_confirmation_applies_safety_zero_without_waiting_for_suppressed_ack() -> None:
    session, transport = _active_session(
        _nav_command,
        _pending_safety_zero,
        _ready_for_last_sent_command,
    )
    nav = session.receive_command()
    session.complete_step(nav, step_seq=3)
    session.heartbeat(step_seq=4)

    session.confirm_ready()
    safety_zero = session.receive_command()
    session.complete_step(safety_zero, step_seq=5)
    session.heartbeat(step_seq=6)
    session.confirm_ready()

    assert safety_zero.kind == "safety_zero"
    assert list(transport.incoming) == []


def test_public_surface_is_protocol_only_without_lifecycle_ownership() -> None:
    assert session_module.__all__ == [
        "DriverBridgeCommand",
        "DriverBridgeSession",
        "DriverBridgeSessionError",
        "DriverBridgeStoppedEvidence",
    ]
    assert list(inspect.signature(DriverBridgeSession).parameters) == [
        "send_line",
        "recv_line",
        "close",
        "expected_product_session_id",
        "operation_timeout_s",
        "max_line_bytes",
        "clock",
    ]
    source = inspect.getsource(session_module)
    forbidden = (
        "sub" + "process",
        "P" + "open",
        "Runtime" + "Graph",
        "Run" + "Plan",
        "readi" + "ness",
        "pid_" + "file",
        "ready_" + "file",
    )
    assert all(token not in source for token in forbidden)
