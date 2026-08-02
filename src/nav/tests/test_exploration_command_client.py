from __future__ import annotations

import pytest

from nav.adapters.native.abi import (
    NATIVE_COMMAND_ABI_VERSION,
    NATIVE_COMMAND_CAP_EXPLORATION_RUN_EVENTS,
    NativeCommandClientError,
    NativeCommandSession,
)
from nav.adapters.native.exploration_commands import NativeExplorationCommandClient


RUN_ID = "01ARZ3NDEKTSV4RRFFQ69G5FAV"


class _Function:
    def __init__(self, implementation):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class _Library:
    def __init__(self, *, capabilities: int = 0x0F) -> None:
        self.calls: list[tuple[str, tuple[object, ...]]] = []
        self.exploration_receipts: list[dict[str, object]] = []
        self.exploration_run_events: list[dict[str, object]] = []
        self.lingtu_nav_client_abi_version = _Function(lambda: NATIVE_COMMAND_ABI_VERSION)
        self.lingtu_nav_client_capabilities = _Function(lambda: capabilities)
        self.lingtu_nav_client_create = _Function(lambda _domain: 1)
        self.lingtu_nav_client_destroy = _Function(lambda _handle: None)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: b"native failure")
        for name in (
            "lingtu_nav_client_start_exploration_with_receipt_v1",
            "lingtu_nav_client_pause_exploration_with_receipt_v1",
            "lingtu_nav_client_resume_exploration_with_receipt_v1",
            "lingtu_nav_client_stop_exploration_with_receipt_v1",
            "lingtu_nav_client_set_directed_exploration_target_with_receipt_v1",
            "lingtu_nav_client_clear_directed_exploration_target_with_receipt_v1",
        ):
            setattr(self, name, _Function(self._record(name)))
        self.lingtu_nav_client_take_exploration_run_event_v1 = _Function(
            self._take_exploration_run_event
        )

    def _record(self, name: str):
        def invoke(*args):
            self.calls.append((name, args))
            receipt = args[-1]._obj
            response = (
                self.exploration_receipts.pop(0)
                if self.exploration_receipts
                else {}
            )
            receipt.accepted = int(bool(response.get("accepted", True)))
            receipt.request_id = args[1]
            receipt.exploration_run_id = args[2]
            receipt.reason = str(response.get("reason", "accepted")).encode()
            receipt.duplicate = int(bool(response.get("duplicate", False)))
            return 0

        return invoke

    def _take_exploration_run_event(self, _handle, output) -> int:
        if not self.exploration_run_events:
            return 0
        values = self.exploration_run_events.pop(0)
        event = output._obj
        for name, value in values.items():
            setattr(event, name, value)
        return 1


def _client(library: _Library) -> NativeExplorationCommandClient:
    return NativeExplorationCommandClient(
        "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=2500,
        library=library,
    )


def test_exploration_client_uses_typed_native_abi() -> None:
    library = _Library()
    client = _client(library)

    receipts = [
        client.start(RUN_ID, "session-a", reason="web_start", request_id="start-1"),
        client.pause(RUN_ID, "session-a", "web_pause", request_id="pause-1"),
        client.resume(RUN_ID, "session-a", "web_resume", request_id="resume-1"),
        client.stop(RUN_ID, "session-a", "web_stop", request_id="stop-1"),
    ]

    assert [name for name, _args in library.calls] == [
        "lingtu_nav_client_start_exploration_with_receipt_v1",
        "lingtu_nav_client_pause_exploration_with_receipt_v1",
        "lingtu_nav_client_resume_exploration_with_receipt_v1",
        "lingtu_nav_client_stop_exploration_with_receipt_v1",
    ]
    assert receipts == [
        {
            "accepted": True,
            "request_id": request_id,
            "exploration_run_id": RUN_ID,
            "reason": "accepted",
            "duplicate": False,
        }
        for request_id in ("start-1", "pause-1", "resume-1", "stop-1")
    ]
    start_args = library.calls[0][1]
    assert start_args[1:5] == (b"start-1", RUN_ID.encode(), b"session-a", b"web_start")
    assert start_args[-2] == 2500
    assert library.calls[-1][1][1:5] == (
        b"stop-1",
        RUN_ID.encode(),
        b"session-a",
        b"web_stop",
    )
    client.close()


def test_exploration_client_requires_capability() -> None:
    with pytest.raises(NativeCommandClientError, match="exploration commands capability"):
        _client(_Library(capabilities=0x03))


@pytest.mark.parametrize(
    "exploration_run_id",
    (
        "",
        "01arz3ndektsv4rrffq69g5fav",
        "81ARZ3NDEKTSV4RRFFQ69G5FAV",
        "01ARZ3NDEKTSV4RRFFQ69G5FAI",
        "01ARZ3NDEKTSV4RRFFQ69G5FA",
    ),
)
def test_exploration_client_rejects_noncanonical_run_id_before_native_call(
    exploration_run_id: str,
) -> None:
    library = _Library()
    client = _client(library)

    with pytest.raises(ValueError, match="canonical uppercase 26-character ULID"):
        client.start(exploration_run_id, "session-a")

    assert library.calls == []
    client.close()


def test_exploration_client_requires_product_session_for_every_command() -> None:
    library = _Library()
    client = _client(library)
    commands = (
        lambda: client.start(RUN_ID, ""),
        lambda: client.pause(RUN_ID, ""),
        lambda: client.resume(RUN_ID, ""),
        lambda: client.stop(RUN_ID, ""),
        lambda: client.set_directed_target(1.0, 2.0, 30.0, RUN_ID, ""),
        lambda: client.clear_directed_target(RUN_ID, ""),
    )

    for command in commands:
        with pytest.raises(ValueError, match="exploration session_id is required"):
            command()

    assert library.calls == []
    client.close()


def test_exploration_client_returns_business_rejection_and_duplicate() -> None:
    library = _Library()
    library.exploration_receipts.append(
        {"accepted": False, "reason": "already_terminal", "duplicate": True}
    )
    client = _client(library)

    assert client.stop(
        RUN_ID,
        "session-a",
        request_id="stop-duplicate",
    ) == {
        "accepted": False,
        "request_id": "stop-duplicate",
        "exploration_run_id": RUN_ID,
        "reason": "already_terminal",
        "duplicate": True,
    }
    client.close()


def test_exploration_client_sends_typed_directed_target_commands() -> None:
    library = _Library()
    client = _client(library)

    client.set_directed_target(
        12.5,
        -8.25,
        45.0,
        RUN_ID,
        "session-a",
        reason="web_directed_target",
        request_id="directed-set-1",
    )
    client.clear_directed_target(
        RUN_ID,
        "session-a",
        "web_clear_directed_target",
        request_id="directed-clear-1",
    )

    assert [name for name, _args in library.calls] == [
        "lingtu_nav_client_set_directed_exploration_target_with_receipt_v1",
        "lingtu_nav_client_clear_directed_exploration_target_with_receipt_v1",
    ]
    assert library.calls[0][1][:-1] == (
        1,
        b"directed-set-1",
        RUN_ID.encode(),
        12.5,
        -8.25,
        45.0,
        b"session-a",
        b"web_directed_target",
        2500,
    )
    assert library.calls[1][1][:-1] == (
        1,
        b"directed-clear-1",
        RUN_ID.encode(),
        b"session-a",
        b"web_clear_directed_target",
        2500,
    )
    client.close()


@pytest.mark.parametrize("session_id", ("", None))
def test_exploration_client_rejects_empty_directed_clear_session_before_native_call(
    session_id: str | None,
) -> None:
    library = _Library()
    client = _client(library)

    with pytest.raises(ValueError, match="exploration session_id is required"):
        client.clear_directed_target(RUN_ID, session_id)  # type: ignore[arg-type]

    assert library.calls == []
    client.close()


@pytest.mark.parametrize(
    ("x", "y", "ttl_s"),
    [
        (float("nan"), 0.0, 30.0),
        (0.0, float("inf"), 30.0),
        (1_000_000.1, 0.0, 30.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 3_600.1),
    ],
)
def test_exploration_client_rejects_invalid_directed_target_before_native_call(
    x: float,
    y: float,
    ttl_s: float,
) -> None:
    library = _Library()
    client = _client(library)

    with pytest.raises(ValueError, match="directed target"):
        client.set_directed_target(x, y, ttl_s, RUN_ID, "session-a")

    assert library.calls == []
    client.close()


def test_exploration_client_requires_directed_target_capability() -> None:
    library = _Library(capabilities=0x07)
    client = _client(library)

    with pytest.raises(NativeCommandClientError, match="directed exploration commands capability"):
        client.set_directed_target(1.0, 2.0, 30.0, RUN_ID, "session-a")

    assert library.calls == []
    client.close()


def test_native_session_takes_full_exploration_run_event() -> None:
    library = _Library(capabilities=0x0F | NATIVE_COMMAND_CAP_EXPLORATION_RUN_EVENTS)
    library.exploration_run_events.append(
        {
            "timestamp_s": 123.5,
            "frame_id": b"map",
            "boot_id": b"explore-boot-a",
            "event_sequence": 7,
            "kind": 2,
            "exploration_run_id": RUN_ID.encode(),
            "start_request_id": b"start-1",
            "command_request_id": b"pause-1",
            "product_session_id": b"product-session-a",
            "state": 4,
            "route": b"map",
            "map_id": b"orchard",
            "map_version": 3,
            "artifact_hash": b"a" * 64,
            "reason": b"paused",
            "motion_stop_confirmed": 1,
            "motion_stop_reason": b"cmd_vel_zero_confirmed",
        }
    )
    session = NativeCommandSession(
        "liblingtu_nav_client.so",
        domain_id=7,
        library=library,
    )

    assert session.take_exploration_run_event() == {
        "timestamp_s": 123.5,
        "frame_id": "map",
        "boot_id": "explore-boot-a",
        "event_sequence": 7,
        "kind": 2,
        "exploration_run_id": RUN_ID,
        "start_request_id": "start-1",
        "command_request_id": "pause-1",
        "product_session_id": "product-session-a",
        "state": 4,
        "route": "map",
        "map_id": "orchard",
        "map_version": 3,
        "artifact_hash": "a" * 64,
        "reason": "paused",
        "motion_stop_confirmed": True,
        "motion_stop_reason": "cmd_vel_zero_confirmed",
    }
    assert session.take_exploration_run_event() is None
    session.close()
