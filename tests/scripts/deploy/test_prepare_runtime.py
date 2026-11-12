import importlib.util
import json
import subprocess
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location("prepare_runtime", Path(__file__).resolve().parents[3] / "scripts/deploy/prepare_runtime.py")
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


@pytest.fixture
def package(tmp_path):
    root = tmp_path / "release"
    root.mkdir()
    (root / "metadata.json").write_text(json.dumps({"schema_version": "lingtu.native-release.v1",
                                                  "version": "v2.3.0", "architecture": "x86_64"}))
    (root / "src").mkdir()
    for name in ("pyproject.toml", "uv.lock", "install_nav.sh"):
        (root / name).write_text("fixture")
    return root


def test_plan_is_read_only_and_pins_environment_outside_release(package, tmp_path):
    target = tmp_path / "runtime"
    plan = runtime.prepare_plan(package, target)
    assert not target.exists() and plan["activates_product"] is False
    assert "--locked" in plan["command"] and "--no-editable" in plan["command"]
    assert plan["extras"] == ["gateway", "llm", "agent", "vision"]
    assert str(target / "bin/python") in plan["install_command"]
    with pytest.raises(ValueError, match="outside"):
        runtime.prepare_plan(package, package / ".venv")
    target.mkdir()
    with pytest.raises(ValueError, match="already exists"):
        runtime.prepare_plan(package, target)


def test_apply_checks_target_and_imports_without_activating(package, tmp_path, monkeypatch):
    plan = runtime.prepare_plan(package, tmp_path / "runtime")
    monkeypatch.setattr(runtime.platform, "system", lambda: "Linux")
    monkeypatch.setattr(runtime.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(runtime.shutil, "which", lambda _name: "/usr/bin/uv")
    monkeypatch.setenv("PYTHONPATH", "unrelated-source")
    monkeypatch.setenv("VIRTUAL_ENV", "unrelated-venv")
    run = Mock()
    monkeypatch.setattr(runtime.subprocess, "run", run)
    assert runtime.apply_plan(plan)["imports_verified"]
    assert run.call_count == 2
    assert run.call_args_list[0].kwargs["env"]["UV_PROJECT_ENVIRONMENT"] == plan["environment"]
    assert "PYTHONPATH" not in run.call_args.kwargs["env"]
    assert run.call_args.args[0][:3] == [plan["python"], "-I", "-c"]
    run.side_effect = subprocess.CalledProcessError(1, "uv")
    with pytest.raises(subprocess.CalledProcessError):
        runtime.apply_plan(plan)


def test_wrong_target_never_executes(package, tmp_path, monkeypatch):
    plan = runtime.prepare_plan(package, tmp_path / "runtime")
    monkeypatch.setattr(runtime.platform, "system", lambda: "Linux")
    monkeypatch.setattr(runtime.platform, "machine", lambda: "aarch64")
    run = Mock()
    monkeypatch.setattr(runtime.subprocess, "run", run)
    with pytest.raises(ValueError, match="matching"):
        runtime.apply_plan(plan)
    run.assert_not_called()
