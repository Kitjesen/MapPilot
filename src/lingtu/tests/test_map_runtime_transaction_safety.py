from __future__ import annotations

from lingtu.map_runtime_transaction import MapRuntimeTransaction


class FakeMaps:
    def __init__(self, active: str = "previous") -> None:
        self.active = active
        self.commands: list[dict[str, object]] = []

    def command(self, command: dict[str, object]) -> dict[str, object]:
        self.commands.append(dict(command))
        action = command["action"]
        if action == "get_active":
            return {"success": True, "active": self.active}
        if action == "set_active":
            name = str(command["name"])
            if name == "missing":
                return {"success": False, "message": "map not found: missing"}
            self.active = name
            return {
                "success": True,
                "active": name,
                "octomap": f"/maps/{name}/octomap.ot",
            }
        if action == "clear_active":
            self.active = ""
            return {"success": True, "active": ""}
        raise AssertionError(command)


def test_external_planner_without_ack_rolls_active_map_back() -> None:
    maps = FakeMaps()
    result = MapRuntimeTransaction(
        maps.command,
        lambda path: {
            "ok": True,
            "delegated": True,
            "reason": "planner_not_in_process",
            "map_path": path,
        },
    ).activate("target")

    assert result["success"] is False
    assert result["reason_code"] == "runtime_binding_unconfirmed"
    assert result["transaction"]["state"] == "rollback_failed"
    assert maps.active == "previous"


def test_same_session_map_rejects_stale_active_map_cache() -> None:
    maps = FakeMaps(active="other")

    result = MapRuntimeTransaction(maps.command, lambda _path: {"ok": True}).activate(
        "warehouse",
        session_mode="navigating",
        session_map="warehouse",
    )

    assert result["success"] is False
    assert result["reason_code"] == "session_active_map_drift"
    assert maps.commands == [{"action": "get_active"}]


def test_failed_first_activation_restores_no_active_map() -> None:
    maps = FakeMaps(active="")

    result = MapRuntimeTransaction(
        maps.command,
        lambda _path: {"ok": False, "reason": "planner_reload_failed"},
    ).activate("target")

    assert result["success"] is False
    assert maps.active == ""
    assert {"action": "clear_active"} in maps.commands


def test_missing_map_preserves_not_found_reason() -> None:
    maps = FakeMaps()

    result = MapRuntimeTransaction(maps.command, lambda _path: {"ok": True}).activate("missing")

    assert result["success"] is False
    assert result["reason_code"] == "map_not_found"


def test_first_activation_accepts_native_no_active_map_response() -> None:
    commands: list[dict[str, object]] = []

    def map_command(command: dict[str, object]) -> dict[str, object]:
        commands.append(dict(command))
        if command["action"] == "get_active":
            return {
                "success": False,
                "reason_code": "map_not_found",
                "message": "no active map",
            }
        if command["action"] == "set_active":
            return {
                "success": True,
                "active": command["name"],
                "octomap": "/maps/first/octomap.ot",
            }
        raise AssertionError(command)

    result = MapRuntimeTransaction(
        map_command,
        lambda path: {"ok": True, "map_path": path},
    ).activate("first")

    assert result["success"] is True
    assert result["transaction"]["previous_active"] is None
    assert commands == [{"action": "get_active"}, {"action": "set_active", "name": "first"}]
