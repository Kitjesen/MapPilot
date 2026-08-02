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
            self.active = str(command["name"])
            return {
                "success": True,
                "active": self.active,
                "octomap": f"/maps/{self.active}/octomap.ot",
            }
        raise AssertionError(f"unexpected command: {command}")


def test_activation_rejects_cross_map_change_during_navigation() -> None:
    maps = FakeMaps(active="warehouse")
    transaction = MapRuntimeTransaction(maps.command, lambda _path: {"ok": True})

    result = transaction.activate(
        "office",
        session_mode="navigating",
        session_map="warehouse",
    )

    assert result["success"] is False
    assert result["reason_code"] == "active_session_map_conflict"
    assert result["transaction"]["state"] == "rejected"
    assert maps.commands == []


def test_activation_commits_only_after_planner_confirms_reload() -> None:
    maps = FakeMaps()
    reloads: list[str] = []

    def reload(path: str) -> dict[str, object]:
        reloads.append(path)
        return {"ok": True, "map_path": path}

    result = MapRuntimeTransaction(maps.command, reload).activate("target")

    assert result["success"] is True
    assert result["active"] == "target"
    assert result["transaction"] == {
        "operation": "activate_map",
        "state": "committed",
        "previous_active": "previous",
        "target_map": "target",
        "runtime_consistent": True,
        "restart_required": False,
    }
    assert reloads == ["/maps/target/octomap.ot"]


def test_activation_rolls_map_and_planner_back_when_reload_fails() -> None:
    maps = FakeMaps()
    reloads: list[str] = []

    def reload(path: str) -> dict[str, object]:
        reloads.append(path)
        return {"ok": "previous" in path, "map_path": path}

    result = MapRuntimeTransaction(maps.command, reload).activate("target")

    assert result["success"] is False
    assert result["reason_code"] == "planner_reload_failed"
    assert result["transaction"]["state"] == "rolled_back"
    assert result["transaction"]["runtime_consistent"] is True
    assert result["rollback"]["success"] is True
    assert result["rollback"]["planner_reload"]["ok"] is True
    assert maps.active == "previous"
    assert reloads == [
        "/maps/target/octomap.ot",
        "/maps/previous/octomap.ot",
    ]


def test_activation_does_not_reload_when_map_store_rejects_target() -> None:
    reloads: list[str] = []

    def command(request: dict[str, object]) -> dict[str, object]:
        if request["action"] == "get_active":
            return {"success": True, "active": "previous"}
        return {
            "success": False,
            "reason_code": "artifact_gate_failed",
            "message": "octomap is missing",
        }

    result = MapRuntimeTransaction(command, lambda path: reloads.append(path)).activate("target")

    assert result["success"] is False
    assert result["reason_code"] == "artifact_gate_failed"
    assert result["transaction"]["state"] == "rejected"
    assert reloads == []
