from __future__ import annotations

import json
import subprocess
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from runtime.adapters.ros2.map_save import (
    MapSaveError,
    MapSaveTimeout,
    MapSaveUnavailable,
    Ros2MapSaveAdapter,
    save_nav_map,
    save_pgo_map,
)
from runtime.map_save import default_map_save_adapter
from runtime.registry import clear, list_plugins, restore, snapshot
from lingtu.plugin_seed import seed_builtin_plugins


def test_save_pgo_map_builds_legacy_ros2_command() -> None:
    with patch("runtime.adapters.ros2.map_save.subprocess.run") as run:
        run.return_value = MagicMock(returncode=0, stdout="ok", stderr="")

        resp = save_pgo_map(Path("map.pcd"), timeout_sec=7.5)

    cmd = run.call_args.args[0]
    payload = json.loads(cmd[5])

    assert cmd[:5] == [
        "ros2",
        "service",
        "call",
        "/pgo/save_maps",
        "interface/srv/SaveMaps",
    ]
    assert payload == {"file_path": "map.pcd", "save_patches": True}
    assert run.call_args.kwargs["timeout"] == 7.5
    assert resp["success"] is True
    assert resp["source"] == "ros2_pgo_save_maps"


def test_save_pgo_map_reports_unavailable_runtime() -> None:
    with patch("runtime.adapters.ros2.map_save.subprocess.run", side_effect=FileNotFoundError):
        with pytest.raises(MapSaveUnavailable):
            save_pgo_map("map.pcd")


def test_save_pgo_map_reports_timeout() -> None:
    error = subprocess.TimeoutExpired(cmd=["ros2"], timeout=1.0)
    with patch("runtime.adapters.ros2.map_save.subprocess.run", side_effect=error):
        with pytest.raises(MapSaveTimeout):
            save_pgo_map("map.pcd")


def test_save_pgo_map_reports_service_failure() -> None:
    with patch("runtime.adapters.ros2.map_save.subprocess.run") as run:
        run.return_value = MagicMock(returncode=1, stderr="boom")

        with pytest.raises(MapSaveError, match="boom"):
            save_pgo_map("map.pcd")


def test_save_nav_map_builds_legacy_ros2_command() -> None:
    with patch("runtime.adapters.ros2.map_save.subprocess.run") as run:
        run.return_value = MagicMock(returncode=0, stdout="success=True\n", stderr="")

        resp = save_nav_map(Path("map with spaces.pcd"), timeout_sec=8.0)

    cmd = run.call_args.args[0]
    command = cmd[2]

    assert cmd[:2] == ["bash", "-c"]
    assert "ros2 service call /slam/save_map interface/srv/SaveMaps" in command
    assert "map with spaces.pcd" in command
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in command
    assert run.call_args.kwargs["timeout"] == 8.0
    assert resp["success"] is True
    assert resp["source"] == "ros2_nav_save_map"


def test_save_nav_map_reports_service_failure() -> None:
    with patch("runtime.adapters.ros2.map_save.subprocess.run") as run:
        run.return_value = MagicMock(returncode=0, stdout="", stderr="service down")

        with pytest.raises(MapSaveError, match="service down"):
            save_nav_map("map.pcd")


def test_ros2_map_save_adapter_delegates_to_legacy_services() -> None:
    adapter = Ros2MapSaveAdapter()
    with patch("runtime.adapters.ros2.map_save.save_nav_map") as nav, patch(
        "runtime.adapters.ros2.map_save.save_pgo_map"
    ) as pgo:
        nav.return_value = {"success": True, "source": "nav"}
        pgo.return_value = {"success": True, "source": "pgo"}

        nav_resp = adapter.save_nav_map("map.pcd", timeout_sec=3.0)
        pgo_resp = adapter.save_pgo_map(
            "map_dir",
            save_patches=False,
            timeout_sec=4.0,
        )

    nav.assert_called_once_with("map.pcd", timeout_sec=3.0)
    pgo.assert_called_once_with(
        "map_dir",
        save_patches=False,
        timeout_sec=4.0,
    )
    assert nav_resp["source"] == "nav"
    assert pgo_resp["source"] == "pgo"


def test_default_map_save_adapter_resolves_ros2_plugin_after_registry_clear(
    monkeypatch,
) -> None:
    state = snapshot()
    clear()
    try:
        monkeypatch.delenv("LINGTU_MAP_SAVE_ADAPTER", raising=False)
        seed_builtin_plugins(groups=("map_save_adapter",), reload_loaded=True)

        adapter = default_map_save_adapter()

        assert type(adapter).__module__ == "runtime.adapters.ros2.map_save"
        assert type(adapter).__name__ == "Ros2MapSaveAdapter"
        assert callable(adapter.save_nav_map)
        assert callable(adapter.save_pgo_map)
        assert "ros2" in list_plugins("map_save_adapter")
    finally:
        restore(state)
