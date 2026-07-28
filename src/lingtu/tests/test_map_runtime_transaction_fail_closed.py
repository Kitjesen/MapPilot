from __future__ import annotations

import pytest

from lingtu.map_runtime_transaction import MapRuntimeTransaction


@pytest.mark.parametrize(
    "query_response",
    [None, {}, {"active": "warehouse"}, {"success": "true", "active": "warehouse"}],
)
def test_active_map_query_requires_typed_success(query_response) -> None:
    commands: list[dict[str, object]] = []

    def map_command(command):
        commands.append(dict(command))
        return query_response

    result = MapRuntimeTransaction(map_command, lambda _path: {"ok": True}).activate("office")

    assert result["success"] is False
    assert result["reason_code"] == "active_map_query_failed"
    assert result["transaction"]["runtime_consistent"] is False
    assert commands == [{"action": "get_active"}]


def test_first_activation_failure_cannot_confirm_planner_clear() -> None:
    active = ""

    def map_command(command):
        nonlocal active
        if command["action"] == "get_active":
            return {"success": True, "active": active}
        if command["action"] == "set_active":
            active = str(command["name"])
            return {"success": True, "active": active, "octomap": "/maps/office/octomap.ot"}
        if command["action"] == "clear_active":
            active = ""
            return {"success": True, "active": ""}
        raise AssertionError(command)

    result = MapRuntimeTransaction(
        map_command,
        lambda _path: {"ok": False, "reason": "planner_reload_failed"},
    ).activate("office")

    assert active == ""
    assert result["success"] is False
    assert result["transaction"]["state"] == "rollback_failed"
    assert result["transaction"]["runtime_consistent"] is False
    assert result["rollback"]["planner_reload"]["reason"] == "planner_clear_unconfirmed"
