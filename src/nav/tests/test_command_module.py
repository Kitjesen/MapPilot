from __future__ import annotations

import pytest

from nav.commands import module as command_module
from runtime import Module


class _NavigationClient:
    def __init__(self) -> None:
        self.calls = []

    def send_goal(self, x, y, z, yaw, *, request_id=None):
        self.calls.append(("goal", x, y, z, yaw, request_id))

    def stop(self, reason, *, request_id=None):
        self.calls.append(("stop", reason, request_id))


class _InspectionClient:
    def __init__(self) -> None:
        self.calls = []

    def start(self, route_id, *, revision=0, request_id=None):
        self.calls.append(("start", route_id, revision, request_id))


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

    assert commands.send_goal(1, 2, 0.3, 0.4, request_id="goal-1") is True
    assert commands.stop_motion("operator", request_id="stop-1") is True
    assert commands.start_inspection("route-a", 7, request_id="route-1") is True

    assert navigation.calls == [
        ("goal", 1.0, 2.0, 0.3, 0.4, "goal-1"),
        ("stop", "operator", "stop-1"),
    ]
    assert inspection.calls == [("start", "route-a", 7, "route-1")]


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
