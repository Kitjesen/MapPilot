"""Assembly coverage for the optional building-navigation ingress."""

from __future__ import annotations

from pathlib import Path

from lingtu.assembly.stacks.services import services


def _entry(bp, name: str):
    return next(entry for entry in bp._entries if entry.name == name)


def test_services_does_not_mount_building_navigation_by_default() -> None:
    bp = services()

    assert "nav.building" not in bp.module_names
    assert "building_module" not in _entry(bp, "nav.goals").config


def test_services_mounts_building_navigation_only_when_enabled() -> None:
    bp = services(
        enable_building=True,
        building_mission_module="site.lift_mission",
    )

    building = _entry(bp, "nav.building")
    goals = _entry(bp, "nav.goals")
    assert building.config == {
        "maps_module": "maps.service",
        "mission_module": "site.lift_mission",
    }
    assert goals.config["building_module"] == "nav.building"


def test_native_services_mounts_persistent_task_history(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_TASK_LEDGER_PATH", "state/navigation-tasks.sqlite3")

    bp = services(
        native_navigation_endpoint=True,
        product_fingerprint="product-sha256",
        map_identity={"map_id": "yard", "map_version": "v3"},
    )

    goals = _entry(bp, "nav.goals")
    assert goals.config["command_module"] == "nav.commands"
    assert Path(goals.config["task_ledger_path"]) == Path("state/navigation-tasks.sqlite3")
    assert goals.config["product_fingerprint"] == "product-sha256"
    assert goals.config["map_identity"] == {
        "map_id": "yard",
        "map_version": "v3",
    }


def test_local_services_leave_goal_history_in_memory_by_default() -> None:
    bp = services()

    assert "task_ledger_path" not in _entry(bp, "nav.goals").config
