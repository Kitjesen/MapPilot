"""Contract tests for the simulation-local SimStudio launcher."""

# ruff: noqa: D103,S101

from __future__ import annotations

import inspect
from pathlib import Path
from types import SimpleNamespace

import pytest
from tools.simstudio import __main__ as launcher

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "sim" / "run_simstudio.ps1"


def test_default_host_is_loopback() -> None:
    assert launcher.DEFAULT_HOST == "127.0.0.1"


def test_launcher_has_no_public_host_or_field_controls() -> None:
    source = inspect.getsource(launcher)
    assert "--host" not in source
    assert "--workers" not in source
    assert "--reload" not in source
    assert "--executable" not in source
    assert "ROBOT_HOST" not in source
    assert "LINGTU_" not in source
    assert "DDS_DOMAIN" not in source
    assert "CYCLONEDDS_URI" not in source
    assert "ProductControl" not in source
    assert "lingtu.real.systemd" not in source
    assert "Gateway" not in source
    assert "multiprocessing" not in source


def test_parser_exposes_only_studio_port_and_controlled_root() -> None:
    args = launcher._parser().parse_args(["--port", "8877", "--repo-root", str(REPO_ROOT)])
    assert args.port == 8877
    assert args.repo_root == REPO_ROOT
    assert not hasattr(args, "host")

    for option in ("--host", "--workers", "--reload", "--executable", "--robot-host", "--env", "--environment", "--domain", "--dds-domain"):
        with pytest.raises(SystemExit):
            launcher._parser().parse_args([option, "value"])


def test_server_config_is_loopback_single_worker_without_reload(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    class FakeConfig:
        def __init__(self, app: object, **kwargs: object) -> None:
            captured["app"] = app
            captured.update(kwargs)

    class FakeServer:
        def __init__(self, config: object) -> None:
            captured["config"] = config

        def run(self) -> None:
            raise AssertionError("test must not run the server")

    monkeypatch.setattr(launcher, "build_app", lambda repo_root=None: object())
    monkeypatch.setitem(__import__("sys").modules, "uvicorn", type("Uvicorn", (), {"Config": FakeConfig, "Server": FakeServer}))

    launcher.build_server(REPO_ROOT, port=8877)

    assert captured["host"] == launcher.DEFAULT_HOST
    assert captured["port"] == 8877
    assert captured["workers"] == 1
    assert captured["reload"] is False


def test_repo_root_is_auto_discovered_and_explicit_root_is_controlled() -> None:
    assert launcher.discover_repo_root() == REPO_ROOT.resolve()
    assert launcher.discover_repo_root(REPO_ROOT) == REPO_ROOT.resolve()

    with pytest.raises((FileNotFoundError, ValueError)):
        launcher.discover_repo_root(REPO_ROOT / "build")


def test_app_builder_uses_repository_service_factory(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: dict[str, object] = {}

    class FakeService:
        @classmethod
        def from_repository(cls, root: Path) -> object:
            calls["root"] = root
            return object()

    monkeypatch.setattr(
        "tools.simstudio.service.application.SimulationStudioService",
        FakeService,
    )
    monkeypatch.setattr(
        "tools.simstudio.http.create_app",
        lambda service: (calls.__setitem__("service", service) or "app"),
    )

    assert launcher.build_app(REPO_ROOT) == "app"
    assert calls["root"] == REPO_ROOT.resolve()
    assert calls["service"] is not None


def test_app_builder_auto_discovers_repo_local_ui(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from fastapi.testclient import TestClient

    repo_root = tmp_path / "controlled-repo"
    (repo_root / "sim").mkdir(parents=True)
    (repo_root / "tools" / "simstudio").mkdir(parents=True)
    (repo_root / "config").mkdir()
    (repo_root / "pyproject.toml").write_text("[project]\nname = 'synthetic'\n", encoding="utf-8")
    dist = repo_root / "tools" / "simstudio" / "ui" / "dist"
    (dist / "assets").mkdir(parents=True)
    (dist / "index.html").write_text(
        "<!doctype html><html><head><title>Synthetic Studio</title></head></html>",
        encoding="utf-8",
    )
    (dist / "assets" / "app.js").write_text("console.log('synthetic');\n", encoding="utf-8")

    calls: dict[str, Path] = {}

    class FakeService:
        @classmethod
        def from_repository(cls, root: Path) -> object:
            calls["root"] = root
            return SimpleNamespace(package_service=SimpleNamespace(repo_root=root))

    monkeypatch.setattr(launcher, "__file__", str(repo_root / "tools" / "simstudio" / "__main__.py"))
    monkeypatch.setattr(
        "tools.simstudio.service.application.SimulationStudioService",
        FakeService,
    )

    with TestClient(launcher.build_app()) as client:
        response = client.get("/")

    assert response.status_code == 200
    assert "<title>Synthetic Studio</title>" in response.text
    assert calls["root"] == repo_root.resolve()


def test_power_shell_launcher_is_local_and_does_not_expose_field_controls() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert "python.exe" in source
    assert "-m\", \"tools.simstudio" in source
    assert "-WindowStyle Hidden" in source
    assert "ROBOT_HOST" not in source
    assert "DDS_DOMAIN" not in source
    assert "-FilePath" in source
    assert "-WorkingDirectory $repoRoot" in source
    assert "-Executable" not in source
    assert "-Host" not in source
