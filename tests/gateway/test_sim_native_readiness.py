import json
from types import SimpleNamespace

from gateway.navigation.status import _native_endpoint_readiness
from gateway.services.native_status import read_navigation_status, read_traversability_status
from lingtu.assembly.compiler import compile_run_plan


def test_sim_nav_requires_native_role_even_when_process_is_named_nav_runtime(monkeypatch, tmp_path):
    plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    assert plan.process("nav").name == "nav_runtime"
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.delenv("LINGTU_NAV_STATUS_FILE", raising=False)
    result = _native_endpoint_readiness({"mode": "navigating"}, SimpleNamespace(_compiled_run_plan=plan))
    assert result["required"] is True
    assert result["blockers"] == ["native_endpoint_status_missing_or_stale"]


def test_navigation_snapshot_uses_session_root_and_preserves_explicit_override(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.delenv("LINGTU_NAV_STATUS_FILE", raising=False)
    (tmp_path / "nav.status.json").write_text(json.dumps({"source": "sim"}))
    assert read_navigation_status() == {"source": "sim"}
    explicit = tmp_path / "override.json"
    explicit.write_text(json.dumps({"source": "explicit"}))
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(explicit))
    assert read_navigation_status() == {"source": "explicit"}


def test_traversability_snapshot_follows_sim_session_and_explicit_override(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.delenv("LINGTU_TRAVERSABILITY_STATUS_FILE", raising=False)
    (tmp_path / "traversability.status.json").write_text('{"source":"sim"}')
    assert read_traversability_status() == {"source": "sim"}
    explicit = tmp_path / "explicit.json"
    explicit.write_text('{"source":"explicit"}')
    monkeypatch.setenv("LINGTU_TRAVERSABILITY_STATUS_FILE", str(explicit))
    assert read_traversability_status() == {"source": "explicit"}


def test_default_sim_keeps_localization_in_slam_and_sensors_in_their_own_processes():
    plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    assert plan.process("slam").name == "slam_runtime"
    assert plan.process("imu").provides == ("imu",)
    assert plan.process("lidar").provides == ("lidar",)
    assert "--navigation-fixture" not in plan.process("imu").command.argv
    assert "--navigation-fixture" not in plan.process("lidar").command.argv
