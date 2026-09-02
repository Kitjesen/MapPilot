# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import threading
import time
from typing import Any

import pytest

from sim.runtime.coordinator.control_intent_udp import (
    MAX_CONTROL_DATAGRAM_BYTES,
    BoundedRuntimeRequestInbox,
    ControlIntentValidationError,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    OperatorMotionIntent,
    OperatorRuntimeRequest,
    UdpLoopbackControlAckPublisher,
    UdpLoopbackOperatorIntentReceiver,
    decode_operator_intent_datagram,
    encode_control_ack,
)


def _identity() -> OperatorIntentIdentity:
    return OperatorIntentIdentity(
        run_id="run-playable-001",
        session_id="a" * 64,
        boot_id="boot-playable-001",
        model_generation=3,
        reset_generation=2,
        source_id="robotsimue.local_player.0",
    )


def _motion_document(**overrides: Any) -> dict[str, Any]:
    document: dict[str, Any] = {
        "schema": "lingtu.sim.ue-control-intent.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": 42,
        "event_id": "boot-playable-001:1:42",
        "input_mode": "drive",
        "ui_mode": "drive",
        "camera_mode": "follow",
        "input_device": "keyboard",
        "viewport_focused": True,
        "deadman": True,
        "axes": {"forward": 1.0, "left": 0.0, "yaw_left": 0.0},
        "active_controls": ["keyboard.left_shift", "keyboard.w"],
        "source_monotonic_ns": 123_456_789,
    }
    document.update(overrides)
    return document


def _payload(document: dict[str, Any]) -> bytes:
    return json.dumps(document, separators=(",", ":")).encode("utf-8")


def _request_document(**overrides: Any) -> dict[str, Any]:
    document: dict[str, Any] = {
        "schema": "lingtu.sim.ue-runtime-request.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": 43,
        "event_id": "boot-playable-001:1:43",
        "request": "record_start",
        "ui_mode": "menu",
        "camera_mode": "inspection",
        "source_monotonic_ns": 123_456_999,
    }
    document.update(overrides)
    return document


def test_valid_exact_motion_datagram_preserves_server_arrival_and_wire_hash() -> None:
    raw = _payload(_motion_document())

    intent = decode_operator_intent_datagram(
        raw,
        expected_identity=_identity(),
        arrival_monotonic_ns=987_654_321,
    )

    assert isinstance(intent, OperatorMotionIntent)
    assert intent.identity == _identity()
    assert intent.source_epoch == 1
    assert intent.source_sequence == 42
    assert intent.event_id == "boot-playable-001:1:42"
    assert intent.ui_mode == "drive"
    assert intent.camera_mode == "follow"
    assert intent.axes.forward == 1.0
    assert intent.axes.left == 0.0
    assert intent.axes.yaw_left == 0.0
    assert intent.active_controls == (
        "keyboard.left_shift",
        "keyboard.w",
    )
    assert intent.arrival_monotonic_ns == 987_654_321
    assert intent.datagram_sha256 == hashlib.sha256(raw).hexdigest()


def test_valid_exact_runtime_request_uses_the_same_allocation_identity() -> None:
    raw = _payload(_request_document())

    request = decode_operator_intent_datagram(
        raw,
        expected_identity=_identity(),
        arrival_monotonic_ns=987_654_999,
    )

    assert isinstance(request, OperatorRuntimeRequest)
    assert request.identity == _identity()
    assert request.request == "record_start"
    assert request.ui_mode == "menu"
    assert request.camera_mode == "inspection"
    assert request.event_id == "boot-playable-001:1:43"
    assert request.arrival_monotonic_ns == 987_654_999
    assert request.datagram_sha256 == hashlib.sha256(raw).hexdigest()


def test_ui_echo_fields_are_mandatory_exact_and_ui_state_update_is_not_motion() -> None:
    for document in (_motion_document(), _request_document()):
        for field in ("ui_mode", "camera_mode"):
            missing = dict(document)
            missing.pop(field)
            with pytest.raises(ControlIntentValidationError, match="missing"):
                decode_operator_intent_datagram(
                    _payload(missing),
                    expected_identity=_identity(),
                    arrival_monotonic_ns=987_654_999,
                )

    update = decode_operator_intent_datagram(
        _payload(_request_document(request="ui_state_update")),
        expected_identity=_identity(),
        arrival_monotonic_ns=987_655_000,
    )
    assert isinstance(update, OperatorRuntimeRequest)
    assert update.request == "ui_state_update"

    with pytest.raises(ControlIntentValidationError, match="ui_mode"):
        decode_operator_intent_datagram(
            _payload(_motion_document(ui_mode="tactical")),
            expected_identity=_identity(),
            arrival_monotonic_ns=987_655_001,
        )
    with pytest.raises(ControlIntentValidationError, match="camera_mode"):
        decode_operator_intent_datagram(
            _payload(_request_document(camera_mode="orbit")),
            expected_identity=_identity(),
            arrival_monotonic_ns=987_655_002,
        )


@pytest.mark.parametrize(
    ("raw", "message"),
    [
        (
            (
                b'{"schema":"lingtu.sim.ue-control-intent.v1",'
                b'"schema":"lingtu.sim.ue-control-intent.v1"}'
            ),
            "duplicate JSON key",
        ),
        (_payload(_motion_document(unexpected=True)), "unknown field"),
        (b"\xff", "valid UTF-8"),
        (b" " * (MAX_CONTROL_DATAGRAM_BYTES + 1), "exceeds 4096 bytes"),
        (
            json.dumps(
                _motion_document(
                    axes={"forward": float("nan"), "left": 0.0, "yaw_left": 0.0}
                ),
                allow_nan=True,
            ).encode(),
            "non-finite JSON value",
        ),
        (
            json.dumps(
                _motion_document(
                    axes={"forward": float("inf"), "left": 0.0, "yaw_left": 0.0}
                ),
                allow_nan=True,
            ).encode(),
            "non-finite JSON value",
        ),
    ],
)
def test_wire_decoder_rejects_noncanonical_or_unsafe_bytes(
    raw: bytes, message: str
) -> None:
    with pytest.raises(ControlIntentValidationError, match=message):
        decode_operator_intent_datagram(
            raw,
            expected_identity=_identity(),
            arrival_monotonic_ns=987_654_321,
        )


@pytest.mark.parametrize(
    ("field", "wrong_value"),
    [
        ("run_id", "run-other"),
        ("session_id", "b" * 64),
        ("boot_id", "boot-other"),
        ("model_generation", 4),
        ("reset_generation", 3),
        ("source_id", "robotsimue.local_player.1"),
    ],
)
def test_wire_decoder_rejects_every_wrong_allocation_identity_field(
    field: str, wrong_value: object
) -> None:
    with pytest.raises(ControlIntentValidationError, match=f"{field}.*allocation"):
        decode_operator_intent_datagram(
            _payload(_motion_document(**{field: wrong_value})),
            expected_identity=_identity(),
            arrival_monotonic_ns=987_654_321,
        )


def test_motion_inbox_is_latest_wins_and_take_is_atomic() -> None:
    inbox = LatestOperatorIntentInbox()
    first = decode_operator_intent_datagram(
        _payload(_motion_document(source_sequence=1, event_id="boot-playable-001:1:1")),
        expected_identity=_identity(),
        arrival_monotonic_ns=100,
    )
    latest = decode_operator_intent_datagram(
        _payload(_motion_document(source_sequence=2, event_id="boot-playable-001:1:2")),
        expected_identity=_identity(),
        arrival_monotonic_ns=200,
    )
    assert isinstance(first, OperatorMotionIntent)
    assert isinstance(latest, OperatorMotionIntent)

    inbox.put(first)
    inbox.put(latest)

    assert len(inbox) == 1
    assert inbox.peek_latest() == latest
    assert inbox.take_latest() == latest
    assert inbox.take_latest() is None
    assert len(inbox) == 0


def test_runtime_request_inbox_is_bounded_fifo_and_never_silently_drops() -> None:
    inbox = BoundedRuntimeRequestInbox(capacity=2)

    def request(sequence: int, action: str) -> OperatorRuntimeRequest:
        result = decode_operator_intent_datagram(
            _payload(
                _request_document(
                    source_sequence=sequence,
                    event_id=f"boot-playable-001:1:{sequence}",
                    request=action,
                )
            ),
            expected_identity=_identity(),
            arrival_monotonic_ns=sequence * 100,
        )
        assert isinstance(result, OperatorRuntimeRequest)
        return result

    first = request(1, "control_claim")
    second = request(2, "record_start")
    third = request(3, "pause")
    inbox.put(first)
    inbox.put(second)

    with pytest.raises(ControlIntentValidationError, match="queue is full"):
        inbox.put(third)
    assert len(inbox) == 2
    assert inbox.pop_request() == first
    assert inbox.pop_request() == second
    assert inbox.pop_request() is None

    with pytest.raises(ControlIntentValidationError, match="duplicate event_id"):
        inbox.put(first)
    inbox.clear()
    inbox.put(first)
    assert inbox.pop_request() == first


class _FakeReceiveSocket:
    def __init__(self, datagrams: list[tuple[bytes, tuple[str, int]]]) -> None:
        self.datagrams = list(datagrams)
        self.bound: tuple[str, int] | None = None
        self.timeout: float | None = None
        self.closed = False
        self.receive_sizes: list[int] = []

    def bind(self, address: tuple[str, int]) -> None:
        self.bound = address

    def settimeout(self, value: float) -> None:
        self.timeout = value

    def recvfrom(self, size: int) -> tuple[bytes, tuple[str, int]]:
        self.receive_sizes.append(size)
        if not self.datagrams:
            raise TimeoutError
        return self.datagrams.pop(0)

    def close(self) -> None:
        self.closed = True


def test_loopback_receiver_validates_one_global_source_order_and_only_writes_inboxes() -> None:
    motion_inbox = LatestOperatorIntentInbox()
    request_inbox = BoundedRuntimeRequestInbox()
    fake = _FakeReceiveSocket(
        [
            (
                _payload(
                    _motion_document(
                        source_sequence=1,
                        event_id="boot-playable-001:1:1",
                    )
                ),
                ("127.0.0.1", 41000),
            ),
            (
                _payload(
                    _request_document(
                        source_sequence=2,
                        event_id="boot-playable-001:1:2",
                    )
                ),
                ("127.0.0.1", 41000),
            ),
        ]
    )
    arrivals = iter((1000, 2000))
    receiver = UdpLoopbackOperatorIntentReceiver(
        25124,
        expected_identity=_identity(),
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
        socket_factory=lambda *_: fake,  # type: ignore[arg-type]
        monotonic_ns=lambda: next(arrivals),
    )

    assert isinstance(receiver.receive_once(), OperatorMotionIntent)
    assert isinstance(receiver.receive_once(), OperatorRuntimeRequest)
    assert motion_inbox.take_latest() is not None
    assert request_inbox.pop_request() is not None
    assert fake.bound == ("127.0.0.1", 25124)
    assert fake.receive_sizes == [MAX_CONTROL_DATAGRAM_BYTES + 1] * 2
    receiver.close()
    assert fake.closed


def test_receiver_rejects_non_loopback_and_duplicate_or_out_of_order_source_events() -> None:
    datagrams = [
        (_payload(_motion_document()), ("192.0.2.10", 41000)),
        (_payload(_motion_document()), ("127.0.0.1", 41000)),
        (_payload(_motion_document()), ("127.0.0.1", 41000)),
        (
            _payload(
                _motion_document(
                    source_sequence=41,
                    event_id="boot-playable-001:1:41",
                )
            ),
            ("127.0.0.1", 41000),
        ),
        (
            _payload(
                _motion_document(
                    source_epoch=2,
                    source_sequence=1,
                    event_id="boot-playable-001:2:1",
                )
            ),
            ("127.9.8.7", 41000),
        ),
        (
            _payload(
                _motion_document(
                    source_epoch=1,
                    source_sequence=100,
                    event_id="boot-playable-001:1:100",
                )
            ),
            ("127.0.0.1", 41000),
        ),
    ]
    inbox = LatestOperatorIntentInbox()
    receiver = UdpLoopbackOperatorIntentReceiver(
        25124,
        expected_identity=_identity(),
        motion_inbox=inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        socket_factory=lambda *_: _FakeReceiveSocket(datagrams),  # type: ignore[arg-type]
        monotonic_ns=iter(range(1000, 1006)).__next__,
    )

    with pytest.raises(ControlIntentValidationError, match="loopback"):
        receiver.receive_once()
    assert isinstance(receiver.receive_once(), OperatorMotionIntent)
    with pytest.raises(ControlIntentValidationError, match="source_sequence"):
        receiver.receive_once()
    with pytest.raises(ControlIntentValidationError, match="source_sequence"):
        receiver.receive_once()
    epoch_two = receiver.receive_once()
    assert isinstance(epoch_two, OperatorMotionIntent)
    assert epoch_two.source_epoch == 2
    with pytest.raises(ControlIntentValidationError, match="source_epoch"):
        receiver.receive_once()


def test_receiver_background_thread_continues_after_rejection_and_only_deposits() -> None:
    consumed = threading.Event()

    class SignallingSocket(_FakeReceiveSocket):
        def recvfrom(self, size: int) -> tuple[bytes, tuple[str, int]]:
            result = super().recvfrom(size)
            if not self.datagrams:
                consumed.set()
            return result

    fake = SignallingSocket(
        [
            (_payload(_motion_document()), ("192.0.2.10", 41000)),
            (_payload(_motion_document()), ("127.0.0.1", 41000)),
        ]
    )
    inbox = LatestOperatorIntentInbox()
    receiver = UdpLoopbackOperatorIntentReceiver(
        25124,
        expected_identity=_identity(),
        motion_inbox=inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        socket_factory=lambda *_: fake,  # type: ignore[arg-type]
        monotonic_ns=lambda: 1000,
    )
    receiver.start()
    assert consumed.wait(timeout=1.0)
    deadline = time.monotonic() + 1.0
    while len(inbox) == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    receiver.close()

    assert inbox.take_latest() is not None
    errors = receiver.drain_validation_errors()
    assert len(errors) == 1
    assert "loopback" in str(errors[0])


def _ack_document(**overrides: Any) -> dict[str, Any]:
    document: dict[str, Any] = {
        "schema": "lingtu.sim.ue-control-ack.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
        "server_status_sequence": 8,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": 42,
        "event_id": "boot-playable-001:1:42",
        "intent_datagram_sha256": "b" * 64,
        "status": "accepted",
        "reason": "",
    }
    document.update(overrides)
    return document


class _FakeSendSocket:
    def __init__(self, *_: object) -> None:
        self.blocking: bool | None = None
        self.sent: list[tuple[bytes, tuple[str, int]]] = []
        self.closed = False

    def setblocking(self, value: bool) -> None:
        self.blocking = value

    def sendto(self, payload: bytes, destination: tuple[str, int]) -> int:
        self.sent.append((payload, destination))
        return len(payload)

    def close(self) -> None:
        self.closed = True


def test_control_ack_is_exact_hashed_correlation_and_loopback_only() -> None:
    encoded = encode_control_ack(_ack_document())
    assert json.loads(encoded)["intent_datagram_sha256"] == "b" * 64
    assert len(encoded) <= MAX_CONTROL_DATAGRAM_BYTES

    fake = _FakeSendSocket()
    publisher = UdpLoopbackControlAckPublisher(
        25125,
        socket_factory=lambda *_: fake,  # type: ignore[arg-type]
    )
    assert publisher.publish(_ack_document()) == len(encoded)
    assert fake.blocking is False
    assert fake.sent == [(encoded, ("127.0.0.1", 25125))]
    publisher.close()
    assert fake.closed

    with pytest.raises(ValueError, match=r"127\.0\.0\.1"):
        UdpLoopbackControlAckPublisher(25125, host="192.0.2.10")
    with pytest.raises(ControlIntentValidationError, match="unknown field"):
        encode_control_ack(_ack_document(unexpected=True))
    with pytest.raises(ControlIntentValidationError, match="reason must be non-empty"):
        encode_control_ack(_ack_document(status="rejected", reason=""))
