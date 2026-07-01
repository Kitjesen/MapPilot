"""Security regression contracts for the LingTu manager script."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
MANAGER_PATH = REPO_ROOT / "scripts" / "manager" / "manager.py"


def _load_manager(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    *,
    gateway_url: str | None = None,
):
    monkeypatch.setenv("NAV_DIR", str(REPO_ROOT))
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path / "maps"))
    monkeypatch.setenv("LINGTU_PORT", "5050")
    if gateway_url is None:
        monkeypatch.delenv("LINGTU_GATEWAY_URL", raising=False)
    else:
        monkeypatch.setenv("LINGTU_GATEWAY_URL", gateway_url)
    monkeypatch.delenv("LINGTU_MANAGER_HOST", raising=False)
    monkeypatch.delenv("LINGTU_MANAGER_ORIGINS", raising=False)
    spec = importlib.util.spec_from_file_location("lingtu_manager_under_test", MANAGER_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_manager_defaults_to_loopback_and_restricted_cors(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    manager = _load_manager(monkeypatch, tmp_path)

    assert manager.MANAGER_HOST == "127.0.0.1"
    assert "*" not in manager.MANAGER_ALLOWED_ORIGINS
    assert manager.MANAGER_ALLOWED_ORIGINS == [
        "http://127.0.0.1:5050",
        "http://localhost:5050",
    ]
    assert manager.GATEWAY_URL == "http://127.0.0.1:5051"
    assert manager._gateway_url("/api/v1/goal") == "http://127.0.0.1:5051/api/v1/goal"


def test_manager_gateway_url_can_be_overridden(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    manager = _load_manager(monkeypatch, tmp_path, gateway_url="http://192.168.66.190:5051/")

    assert manager.GATEWAY_URL == "http://192.168.66.190:5051"
    assert (
        manager._gateway_url("/api/v1/camera/snapshot")
        == "http://192.168.66.190:5051/api/v1/camera/snapshot"
    )


def test_safe_map_path_rejects_traversal_and_shell_payloads(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    manager = _load_manager(monkeypatch, tmp_path)

    safe = manager._safe_map_path("lab_01")
    assert safe == (tmp_path / "maps" / "lab_01").resolve()

    for name in ["../escape", "sub/dir", "bad;touch-pwned", "bad name", "", "."]:
        with pytest.raises(ValueError):
            manager._safe_map_path(name)


def test_start_lingtu_uses_argv_and_cwd_without_shell(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    manager = _load_manager(monkeypatch, tmp_path)
    calls = []

    class DummyProc:
        pid = 12345

    def fake_popen(args, **kwargs):
        calls.append((args, kwargs))
        return DummyProc()

    monkeypatch.setattr(manager.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(manager, "_pid_alive", lambda pid: False)
    monkeypatch.setattr(manager.os, "setsid", lambda: None, raising=False)

    assert manager._start_lingtu("nav", "--llm mock") is True
    assert len(calls) == 1
    args, kwargs = calls[0]
    assert args == ["python3", "lingtu.py", "nav", "--no-repl", "--llm", "mock"]
    assert kwargs["cwd"] == str(REPO_ROOT)

    calls.clear()
    assert manager._start_lingtu("nav;touch-pwned") is False
    assert calls == []


def test_start_rerun_uses_gateway_viewer_without_ros_shell(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    manager = _load_manager(monkeypatch, tmp_path)
    calls = []

    class DummyProc:
        pid = 12345

    def fake_popen(args, **kwargs):
        calls.append((args, kwargs))
        return DummyProc()

    monkeypatch.setattr(manager.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(manager.os, "setsid", lambda: None, raising=False)

    manager._start_rerun()

    assert len(calls) == 1
    args, kwargs = calls[0]
    assert args == [
        "python3",
        "-u",
        "scripts/visualization/rerun_gateway_live.py",
        "--gateway-url",
        "http://127.0.0.1:5051",
    ]
    assert kwargs["cwd"] == str(REPO_ROOT)
    assert "bash" not in args
    assert "rerun_live.py" not in " ".join(args)


def test_manager_save_map_uses_adapter_boundary(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    manager = _load_manager(monkeypatch, tmp_path)
    calls: list[Path] = []

    def fake_snapshot(pcd_path: Path) -> dict:
        calls.append(pcd_path)
        pcd_path.parent.mkdir(parents=True, exist_ok=True)
        pcd_path.write_text("VERSION 0.7\nDATA ascii\n", encoding="utf-8")
        return {"success": True, "source": "fake_adapter"}

    monkeypatch.setattr(manager, "_save_nav_map_snapshot", fake_snapshot)

    response = manager._save_map("lab_01")

    expected_pcd = (tmp_path / "maps" / "lab_01" / "map.pcd").resolve()
    assert response == {"success": True, "pcd": str(expected_pcd)}
    assert calls == [expected_pcd]
    assert expected_pcd.is_file()


def test_manager_does_not_construct_ros_map_save_commands() -> None:
    text = MANAGER_PATH.read_text(encoding="utf-8-sig")

    assert "ros2 service call" not in text
    assert "/nav/save_map" not in text
    assert "interface/srv/SaveMaps" not in text


def test_manager_camera_snapshot_proxies_gateway_without_ros_reader() -> None:
    text = MANAGER_PATH.read_text(encoding="utf-8-sig")

    assert "/api/v1/camera/snapshot" in text
    assert "ROS2TopicReader" not in text
    assert "runtime.dds" not in text
    assert "/camera/color/image_raw" not in text
