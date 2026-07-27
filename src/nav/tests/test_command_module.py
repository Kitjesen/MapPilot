from __future__ import annotations

import pytest

from nav.adapters.native.commands import NativeNavigationClient
from nav.commands import module as command_module
from runtime import Module
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt


class _NavigationClient:
    def __init__(self) -> None:
        self.calls = []

    def start_task(self, x, y, z, yaw, *, task_id, request_id=None):
        self.calls.append(("goal", x, y, z, yaw, task_id, request_id))
        return _receipt(
            NavigationCommandKind.GOAL,
            task_id=task_id,
            request_id=request_id or "native-goal-request",
            reason="accepted",
        )

    def cancel_task(self, task_id, reason, *, request_id=None):
        self.calls.append(("cancel_task", task_id, reason, request_id))
        return _receipt(
            NavigationCommandKind.CANCEL,
            task_id=task_id,
            request_id=request_id or "native-cancel-request",
            reason="cancel_requested",
        )

    def stop(self, reason, *, request_id=None):
        self.calls.append(("stop", reason, request_id))


class _InspectionClient:
    def __init__(self) -> None:
        self.calls = []

    def start(self, route_id, *, revision=0, request_id=None):
        self.calls.append(("start", route_id, revision, request_id))


def _receipt(
    kind: NavigationCommandKind,
    *,
    task_id: str = "task-1",
    request_id: str = "request-1",
    accepted: bool = True,
    reason: str = "accepted",
) -> NavigationCommandReceipt:
    return NavigationCommandReceipt(
        accepted=accepted,
        kind=int(kind),
        task_id=task_id,
        request_id=request_id,
        endpoint_timestamp_s=123.5,
        reason=reason,
    )


class _NativeSession:
    def __init__(self) -> None:
        self.calls = []
        self.start_receipt = _receipt(NavigationCommandKind.GOAL)
        self.cancel_receipt = _receipt(
            NavigationCommandKind.CANCEL,
            reason="cancel_requested",
        )

    def start_navigation_task(self, task_id, request_id, x, y, z, yaw):
        self.calls.append(("start", task_id, request_id, x, y, z, yaw))
        return self.start_receipt

    def cancel_navigation_task(self, task_id, request_id, reason):
        self.calls.append(("cancel", task_id, request_id, reason))
        return self.cancel_receipt


def _native_client(session: _NativeSession) -> NativeNavigationClient:
    client = NativeNavigationClient.__new__(NativeNavigationClient)
    client._session = session
    client._owns_session = False
    return client


def _replace_receipt(receipt: NavigationCommandReceipt, **changes) -> NavigationCommandReceipt:
    payload = receipt.to_dict()
    payload.pop("kind_name")
    payload.update(changes)
    return NavigationCommandReceipt(**payload)


def test_command_rpc_names_do_not_override_module_lifecycle(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    assert command_module.Commands.stop is Module.stop
    commands.start()
    assert commands.running is True
    commands.stop()
    assert commands.running is False
    assert navigation.calls == []


def test_commands_forward_typed_navigation_and_inspection_requests(monkeypatch) -> None:
    navigation = _NavigationClient()
    inspection = _InspectionClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    monkeypatch.setattr(
        command_module,
        "get_native_inspection_command_client",
        lambda *, required: inspection,
    )
    commands = command_module.Commands()

    goal_receipt = commands.send_goal(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="goal-1",
    )
    cancel_receipt = commands.cancel_task(
        "task-1",
        "operator",
        request_id="cancel-1",
    )
    assert commands.stop_motion("operator", request_id="stop-1") is True
    assert commands.start_inspection("route-a", 7, request_id="route-1") is True

    assert goal_receipt.accepted is True
    assert goal_receipt.task_id == "task-1"
    assert goal_receipt.request_id == "goal-1"
    assert goal_receipt.reason == "accepted"
    assert cancel_receipt.accepted is True
    assert cancel_receipt.task_id == "task-1"
    assert cancel_receipt.request_id == "cancel-1"
    assert cancel_receipt.reason == "cancel_requested"
    assert navigation.calls == [
        ("goal", 1.0, 2.0, 0.3, 0.4, "task-1", "goal-1"),
        ("cancel_task", "task-1", "operator", "cancel-1"),
        ("stop", "operator", "stop-1"),
    ]
    assert inspection.calls == [("start", "route-a", 7, "route-1")]


def test_commands_preserve_explicit_native_business_rejection(monkeypatch) -> None:
    class RejectingNavigation(_NavigationClient):
        def start_task(self, x, y, z, yaw, *, task_id, request_id=None):
            del x, y, z, yaw
            return _receipt(
                NavigationCommandKind.GOAL,
                task_id=task_id,
                request_id=request_id,
                accepted=False,
                reason="other_task_active",
            )

    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: RejectingNavigation(),
    )

    receipt = command_module.Commands().send_goal(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="request-1",
    )

    assert receipt.accepted is False
    assert receipt.reason == "other_task_active"


@pytest.mark.parametrize(
    ("receipt", "message"),
    [
        (True, "invalid receipt"),
        (_receipt(NavigationCommandKind.CANCEL), "wrong command kind"),
        (
            _receipt(NavigationCommandKind.GOAL, task_id="task-other"),
            "wrong task_id",
        ),
        (
            _receipt(NavigationCommandKind.GOAL, request_id="request-other"),
            "wrong request_id",
        ),
        (_receipt(NavigationCommandKind.GOAL, reason=""), "empty reason"),
    ],
)
def test_commands_reject_invalid_native_task_receipts(
    monkeypatch,
    receipt,
    message,
) -> None:
    class InvalidNavigation(_NavigationClient):
        def start_task(self, x, y, z, yaw, *, task_id, request_id=None):
            del x, y, z, yaw, task_id, request_id
            return receipt

    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: InvalidNavigation(),
    )

    with pytest.raises(RuntimeError, match=message):
        command_module.Commands().send_goal(
            1,
            2,
            0.3,
            0.4,
            task_id="task-1",
            request_id="request-1",
        )


def test_commands_accept_native_clock_retry_request_identity(monkeypatch) -> None:
    class RetriedNavigation(_NavigationClient):
        def start_task(self, x, y, z, yaw, *, task_id, request_id=None):
            del x, y, z, yaw
            return _receipt(
                NavigationCommandKind.GOAL,
                task_id=task_id,
                request_id=f"{request_id}-clock-retry-1",
            )

    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: RetriedNavigation(),
    )

    receipt = command_module.Commands().send_goal(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="request-1",
    )

    assert receipt.request_id == "request-1-clock-retry-1"


@pytest.mark.parametrize(
    ("task_id", "request_id"),
    [
        ("", "request-1"),
        ("   ", "request-1"),
        (None, "request-1"),
        ("task-1", ""),
        ("task-1", "   "),
        ("task-1", None),
        ("same-id", "same-id"),
    ],
)
def test_commands_fail_closed_without_distinct_task_attempt_identity(
    monkeypatch,
    task_id,
    request_id,
) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )

    with pytest.raises(ValueError, match=r"task_id|request_id|distinct"):
        command_module.Commands().send_goal(
            1,
            2,
            0.3,
            0.4,
            task_id=task_id,
            request_id=request_id,
        )

    assert navigation.calls == []


def test_native_navigation_client_forwards_stable_task_and_attempt_identity() -> None:
    session = _NativeSession()
    session.cancel_receipt = _receipt(
        NavigationCommandKind.CANCEL,
        request_id="cancel-1",
        reason="cancel_requested",
    )
    client = _native_client(session)

    goal = client.start_task(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="request-1",
    )
    cancel = client.cancel_task(
        "task-1",
        "operator_cancel",
        request_id="cancel-1",
    )

    assert goal.accepted is True
    assert goal.reason == "accepted"
    assert cancel.accepted is True
    assert cancel.reason == "cancel_requested"
    assert session.calls == [
        ("start", "task-1", "request-1", 1.0, 2.0, 0.3, 0.4),
        ("cancel", "task-1", "cancel-1", "operator_cancel"),
    ]


@pytest.mark.parametrize(
    ("task_id", "request_id"),
    [
        ("", "request-1"),
        ("task-1", ""),
        ("same-id", "same-id"),
    ],
)
@pytest.mark.parametrize("action", ["start", "cancel"])
def test_native_navigation_client_rejects_invalid_identity_before_native_call(
    action,
    task_id,
    request_id,
) -> None:
    session = _NativeSession()
    client = _native_client(session)

    with pytest.raises(ValueError, match=r"task_id|request_id|distinct"):
        if action == "start":
            client.start_task(
                1,
                2,
                0.3,
                0.4,
                task_id=task_id,
                request_id=request_id,
            )
        else:
            client.cancel_task(
                task_id,
                "operator_cancel",
                request_id=request_id,
            )

    assert session.calls == []


@pytest.mark.parametrize(
    ("receipt", "message"),
    [
        (True, "invalid receipt"),
        (_receipt(NavigationCommandKind.CANCEL), "wrong command kind"),
        (
            _receipt(NavigationCommandKind.GOAL, task_id="task-other"),
            "wrong task_id",
        ),
        (
            _receipt(NavigationCommandKind.GOAL, request_id="request-other"),
            "wrong request_id",
        ),
        (_receipt(NavigationCommandKind.GOAL, reason=""), "empty reason"),
    ],
)
def test_native_navigation_client_rejects_invalid_native_receipt(
    receipt,
    message,
) -> None:
    session = _NativeSession()
    session.start_receipt = receipt
    client = _native_client(session)

    with pytest.raises(RuntimeError, match=message):
        client.start_task(
            1,
            2,
            0.3,
            0.4,
            task_id="task-1",
            request_id="request-1",
        )


def test_native_navigation_client_preserves_business_rejection() -> None:
    session = _NativeSession()
    session.start_receipt = _replace_receipt(
        session.start_receipt,
        accepted=False,
        reason="other_task_active",
    )

    receipt = _native_client(session).start_task(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="request-1",
    )

    assert receipt.accepted is False
    assert receipt.reason == "other_task_active"


def test_commands_reject_invalid_inspection_identity_before_native_call(monkeypatch) -> None:
    inspection = _InspectionClient()
    monkeypatch.setattr(
        command_module,
        "get_native_inspection_command_client",
        lambda *, required: inspection,
    )
    commands = command_module.Commands()

    with pytest.raises(RuntimeError, match="route_id is required"):
        commands.start_inspection("  ")
    with pytest.raises(ValueError, match="UINT64_MAX"):
        commands.start_inspection("route-a", -1)

    assert inspection.calls == []


def test_commands_read_current_navigation_state_without_inventing_one(monkeypatch) -> None:
    snapshot = {
        "active_task_id": "task-1",
        "active_request_id": "request-1",
        "sequence": 7,
    }

    class StateNavigation(_NavigationClient):
        def read_navigation_state(self):
            self.calls.append(("read_navigation_state",))
            return snapshot

    navigation = StateNavigation()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    assert commands.read_navigation_state() == snapshot
    assert navigation.calls == [("read_navigation_state",)]

    navigation.read_navigation_state = lambda: None
    assert commands.read_navigation_state() is None


def test_native_navigation_client_reads_authoritative_state_snapshot() -> None:
    session = _NativeSession()
    snapshot = {
        "active_task_id": "task-1",
        "active_request_id": "request-1",
        "sequence": 7,
    }
    calls = []

    def read_state():
        calls.append("read")
        return snapshot

    session.read_navigation_state = read_state

    assert _native_client(session).read_navigation_state() == snapshot
    assert calls == ["read"]


def test_native_navigation_client_gets_status_for_exact_task_identity() -> None:
    session = _NativeSession()
    status = {
        "task_id": "task-1",
        "request_id": "request-1",
        "sequence": 9,
        "state": 2,
    }
    calls = []

    def get_task_status(task_id):
        calls.append(task_id)
        return status

    session.get_navigation_task_status = get_task_status
    client = _native_client(session)

    assert client.get_navigation_task_status(" task-1 ") == status
    assert calls == ["task-1"]

    session.get_navigation_task_status = lambda _task_id: None
    assert client.get_navigation_task_status("task-1") is None


@pytest.mark.parametrize(
    ("status", "message"),
    [
        (True, "invalid status"),
        (
            {"task_id": "task-other", "request_id": "request-1"},
            "wrong task_id",
        ),
        ({"task_id": "task-1", "request_id": ""}, "request_id"),
    ],
)
def test_native_navigation_client_rejects_uncorrelated_task_status(
    status,
    message,
) -> None:
    session = _NativeSession()
    session.get_navigation_task_status = lambda _task_id: status

    with pytest.raises(RuntimeError, match=message):
        _native_client(session).get_navigation_task_status("task-1")


@pytest.mark.parametrize("task_id", ["", "   ", None])
def test_native_navigation_client_rejects_blank_status_task_before_native_call(
    task_id,
) -> None:
    session = _NativeSession()
    calls = []

    def get_task_status(value):
        calls.append(value)

    session.get_navigation_task_status = get_task_status

    with pytest.raises(ValueError, match="task_id"):
        _native_client(session).get_navigation_task_status(task_id)

    assert calls == []


def test_commands_get_status_for_exact_navigation_task(monkeypatch) -> None:
    status = {
        "task_id": "task-1",
        "request_id": "request-1",
        "sequence": 9,
        "state": 2,
    }

    class StatusNavigation(_NavigationClient):
        def get_navigation_task_status(self, task_id):
            self.calls.append(("get_navigation_task_status", task_id))
            return status

    navigation = StatusNavigation()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    assert commands.get_navigation_task_status(" task-1 ") == status
    assert navigation.calls == [
        ("get_navigation_task_status", "task-1"),
    ]

    navigation.get_navigation_task_status = lambda _task_id: None
    assert commands.get_navigation_task_status("task-1") is None


@pytest.mark.parametrize(
    ("status", "message"),
    [
        (True, "invalid status"),
        (
            {"task_id": "task-other", "request_id": "request-1"},
            "wrong task_id",
        ),
        ({"task_id": "task-1", "request_id": ""}, "request_id"),
    ],
)
def test_commands_reject_uncorrelated_navigation_task_status(
    monkeypatch,
    status,
    message,
) -> None:
    class StatusNavigation(_NavigationClient):
        def get_navigation_task_status(self, _task_id):
            return status

    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: StatusNavigation(),
    )

    with pytest.raises(RuntimeError, match=message):
        command_module.Commands().get_navigation_task_status("task-1")


@pytest.mark.parametrize("task_id", ["", "   ", None])
def test_commands_reject_blank_status_task_before_native_call(
    monkeypatch,
    task_id,
) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )

    with pytest.raises(ValueError, match="task_id"):
        command_module.Commands().get_navigation_task_status(task_id)

    assert navigation.calls == []


@pytest.mark.parametrize(
    ("snapshot", "message"),
    [
        (True, "invalid state"),
        (
            {"active_task_id": "task-1", "active_request_id": ""},
            "present together",
        ),
        (
            {"active_task_id": "", "active_request_id": "request-1"},
            "present together",
        ),
        (
            {"active_task_id": "same-id", "active_request_id": "same-id"},
            "distinct",
        ),
    ],
)
def test_native_navigation_client_rejects_invalid_state_identity(
    snapshot,
    message,
) -> None:
    session = _NativeSession()
    session.read_navigation_state = lambda: snapshot

    with pytest.raises(RuntimeError, match=message):
        _native_client(session).read_navigation_state()


@pytest.mark.parametrize(
    ("snapshot", "message"),
    [
        (True, "invalid state"),
        (
            {"active_task_id": "task-1", "active_request_id": ""},
            "present together",
        ),
        (
            {"active_task_id": "same-id", "active_request_id": "same-id"},
            "distinct",
        ),
    ],
)
def test_commands_reject_invalid_navigation_state_identity(
    monkeypatch,
    snapshot,
    message,
) -> None:
    class StateNavigation(_NavigationClient):
        def read_navigation_state(self):
            return snapshot

    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: StateNavigation(),
    )

    with pytest.raises(RuntimeError, match=message):
        command_module.Commands().read_navigation_state()
