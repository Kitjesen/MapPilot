"""Unit tests for cli.run_state save/read/update/compute_uptime."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

# Add project root so `cli` is importable
_ROOT = Path(__file__).resolve().parent.parent.parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


@pytest.fixture(autouse=True)
def isolated_run_dir(tmp_path, monkeypatch):
    """Redirect run_dir() / pid_file() / run_json_file() to a tmp directory."""
    from cli import paths, run_state

    rd = tmp_path / ".lingtu"
    monkeypatch.setattr(paths, "run_dir", lambda: rd)
    monkeypatch.setattr(paths, "pid_file", lambda: rd / "run.pid")
    monkeypatch.setattr(paths, "run_json_file", lambda: rd / "run.json")
    monkeypatch.setattr(run_state, "run_dir", lambda: rd)
    monkeypatch.setattr(run_state, "pid_file", lambda: rd / "run.pid")
    monkeypatch.setattr(run_state, "run_json_file", lambda: rd / "run.json")
    yield rd


class TestSaveReadClear:
    def test_save_and_read(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state

        cfg = {"robot": "stub", "llm": "mock", "_desc": "ignored"}
        save_run_state("stub", cfg, str(isolated_run_dir), argv=["stub"], daemon=False)

        state = read_run_state()
        assert state is not None
        assert state["profile"] == "stub"
        assert state["pid"] == os.getpid()
        assert "_desc" not in state["config"]  # underscore keys filtered
        assert state["config"]["robot"] == "stub"

    def test_save_populates_new_fields(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state

        save_run_state(
            "stub",
            {"robot": "stub"},
            str(isolated_run_dir),
            status="running",
            module_count=19,
            wire_count=48,
        )
        state = read_run_state()
        assert state["status"] == "running"
        assert state["module_count"] == 19
        assert state["wire_count"] == 48
        assert "start_ts" in state and isinstance(state["start_ts"], (int, float))
        assert "host" in state
        assert "version" in state

    def test_save_persists_runtime_boundary(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state

        runtime = {
            "endpoint": "thunder_field",
            "data_source": "thunder_field",
            "runtime_contract": "thunder_field",
            "command_sink": "driver",
            "resolved_runtime_data_flow": [
                {
                    "name": "command_boundary",
                    "outputs": ["driver"],
                }
            ],
        }
        save_run_state(
            "nav",
            {"robot": "s100p"},
            str(isolated_run_dir),
            runtime=runtime,
        )

        state = read_run_state()
        assert state["runtime"] == runtime

    def test_save_omits_none_counts(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state

        save_run_state("stub", {}, str(isolated_run_dir))
        state = read_run_state()
        assert "module_count" not in state
        assert "wire_count" not in state

    def test_read_missing_returns_none(self, isolated_run_dir):
        from cli.run_state import read_run_state

        assert read_run_state() is None

    def test_clear_removes_files(self, isolated_run_dir):
        from cli.run_state import clear_run_state, read_run_state, save_run_state

        save_run_state("stub", {}, str(isolated_run_dir))
        assert read_run_state() is not None
        clear_run_state()
        assert read_run_state() is None


class TestUpdateRunState:
    def test_update_patches_fields(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state, update_run_state

        save_run_state("stub", {}, str(isolated_run_dir), status="initializing")
        assert update_run_state(status="running", module_count=19) is True

        state = read_run_state()
        assert state["status"] == "running"
        assert state["module_count"] == 19
        assert state["profile"] == "stub"  # original fields preserved

    def test_update_missing_returns_false(self, isolated_run_dir):
        from cli.run_state import update_run_state

        assert update_run_state(status="running") is False

    def test_update_preserves_other_fields(self, isolated_run_dir):
        from cli.run_state import read_run_state, save_run_state, update_run_state

        save_run_state("nav", {"robot": "thunder"}, str(isolated_run_dir), argv=["nav"])
        update_run_state(status="running")
        state = read_run_state()
        assert state["profile"] == "nav"
        assert state["argv"] == ["nav"]
        assert state["config"]["robot"] == "thunder"


class TestComputeUptime:
    def test_none_state(self):
        from cli.run_state import compute_uptime

        assert compute_uptime(None) is None
        assert compute_uptime({}) is None

    def test_missing_start_ts(self):
        from cli.run_state import compute_uptime

        assert compute_uptime({"pid": 123}) is None

    def test_invalid_start_ts(self):
        from cli.run_state import compute_uptime

        assert compute_uptime({"start_ts": "not a number"}) is None

    def test_positive_uptime(self):
        from cli.run_state import compute_uptime

        state = {"start_ts": time.time() - 60.0}
        uptime = compute_uptime(state)
        assert uptime is not None
        assert 59 <= uptime <= 61  # tolerance for test execution time

    def test_clock_skew_never_negative(self):
        from cli.run_state import compute_uptime

        state = {"start_ts": time.time() + 100.0}  # future
        assert compute_uptime(state) == 0.0


class TestFormatUptime:
    def test_none(self):
        from cli.run_state import format_uptime

        assert format_uptime(None) == "?"

    def test_seconds_only(self):
        from cli.run_state import format_uptime

        assert format_uptime(0.0) == "0s"
        assert format_uptime(45.0) == "45s"

    def test_minutes(self):
        from cli.run_state import format_uptime

        assert format_uptime(125.0) == "2m 5s"

    def test_hours(self):
        from cli.run_state import format_uptime

        assert format_uptime(3665.0) == "1h 1m 5s"

    def test_days(self):
        from cli.run_state import format_uptime

        assert format_uptime(90061.0) == "1d 1h 1m 1s"


class TestLingtuVersion:
    def test_returns_string(self):
        from cli.run_state import _lingtu_version

        v = _lingtu_version()
        assert isinstance(v, str)
        assert v.count(".") >= 1, f"Expected semver-like version string, got {v!r}"


def _runtime_status_state() -> dict:
    return {
        "pid": os.getpid(),
        "profile": "nav",
        "started_at": "2026-05-23T00:00:00",
        "start_ts": time.time(),
        "status": "running",
        "cwd": str(Path.cwd()),
        "argv": ["nav"],
        "daemon": False,
        "log_dir": "logs/nav",
        "log_file": "logs/nav/lingtu.log",
        "host": "test-host",
        "version": "test",
        "runtime": {
            "endpoint": "thunder_field",
            "data_source": "thunder_field",
            "runtime_contract": "thunder_field",
            "command_sink": "driver",
            "simulation_only": False,
            "slam_source": "lingtu_fastlio_or_external_robot_slam",
            "localization_source": "slam_localizer",
            "mapping_source": "slam_map_cloud",
            "frame_links": {
                "map_to_odom": {
                    "parent": "map",
                    "child": "odom",
                    "required": True,
                },
                "odom_to_body": {
                    "parent": "odom",
                    "child": "body",
                    "required": True,
                },
                "body_to_lidar": {
                    "parent": "body",
                    "child": "lidar_link",
                    "required": True,
                },
            },
            "frames": {
                "map": "map",
                "odom": "odom",
                "body": "body",
                "lidar": "lidar_link",
                "real_lidar": "livox_frame",
                "camera": "camera_link",
                "axis_convention": "x_forward_y_left_z_up",
            },
            "topic_allowed_frame_ids": {
                "/slam/odometry": ["odom", "map"],
                "/slam/registered_cloud": ["body"],
                "/slam/map_cloud": ["map"],
                "/nav/global_path": ["map"],
                "/nav/local_path": ["map", "odom", "body"],
                "/nav/cmd_vel": ["body"],
            },
            "topic_default_frame_ids": {
                "/slam/odometry": "odom",
                "/slam/registered_cloud": "body",
                "/slam/map_cloud": "map",
                "/nav/global_path": "map",
                "/nav/local_path": "map",
                "/nav/cmd_vel": "body",
            },
            "resolved_runtime_data_flow": [
                {"name": "endpoint_adapter", "inputs": ["/nav/lidar_scan", "/nav/imu"]},
                {
                    "name": "slam_or_relayed_localization_map",
                    "outputs": ["/nav/odometry", "/nav/registered_cloud", "/nav/map_cloud"],
                },
                {
                    "name": "command_boundary",
                    "outputs": ["driver"],
                },
            ],
            "validation": {"ok": True, "blockers": []},
        },
    }


def test_status_human_output_includes_runtime_boundary(monkeypatch, capsys):
    from cli import ui

    state = _runtime_status_state()
    monkeypatch.setattr(ui, "read_run_state", lambda: state)
    monkeypatch.setattr(ui, "is_pid_alive", lambda _pid: True)

    ui.cmd_status_external(as_json=False)

    out = capsys.readouterr().out
    assert "Runtime:  endpoint=thunder_field data_source=thunder_field" in out
    assert "SLAM:     slam_source=lingtu_fastlio_or_external_robot_slam" in out
    assert (
        "Frame ids: map=map odom=odom body=body lidar=lidar_link "
        "real_lidar=livox_frame camera=camera_link "
        "axis_convention=x_forward_y_left_z_up"
    ) in out
    assert "Frames:   map->odom, odom->body, body->lidar_link" in out
    assert (
        "Topic frames: odometry=odom,map registered_cloud=body "
        "map_cloud=map global_path=map local_path=map,odom,body cmd_vel=body"
    ) in out
    assert "Path:     sensors=/nav/lidar_scan,/nav/imu" in out
    assert "command=driver" in out


def test_status_json_output_includes_runtime_boundary(monkeypatch, capsys):
    from cli import ui

    state = _runtime_status_state()
    monkeypatch.setattr(ui, "read_run_state", lambda: state)
    monkeypatch.setattr(ui, "is_pid_alive", lambda _pid: True)

    ui.cmd_status_external(as_json=True)

    report = json.loads(capsys.readouterr().out)
    assert report["runtime_status"] == "running"
    assert report["runtime"]["endpoint"] == "thunder_field"
    assert report["runtime"]["data_source"] == "thunder_field"
    assert report["runtime"]["command_sink"] == "driver"
    assert report["runtime"]["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert report["runtime"]["frames"]["axis_convention"] == "x_forward_y_left_z_up"
    assert report["runtime"]["topic_default_frame_ids"]["/nav/cmd_vel"] == "body"
    assert report["runtime"]["resolved_runtime_data_flow"][-1]["outputs"] == ["driver"]
    assert report["runtime"]["validation"] == {"ok": True, "blockers": []}


def test_stop_force_falls_back_when_sigkill_is_unavailable(monkeypatch, capsys):
    from cli import ui

    sent_signals: list[tuple[int, int]] = []
    cleared = {"called": False}

    monkeypatch.setattr(ui, "read_run_state", lambda: {"pid": 1234, "profile": "stub"})
    monkeypatch.setattr(ui, "is_pid_alive", lambda _pid: True)
    monkeypatch.delattr(ui.signal, "SIGKILL", raising=False)
    monkeypatch.setattr(
        ui.os,
        "kill",
        lambda pid, sig: sent_signals.append((pid, sig)),
    )
    monkeypatch.setattr(ui, "clear_run_state", lambda: cleared.__setitem__("called", True))

    ui.cmd_stop(force=True)

    assert sent_signals == [(1234, ui.signal.SIGTERM)]
    assert cleared["called"] is True
    assert "Force killed." in capsys.readouterr().out
