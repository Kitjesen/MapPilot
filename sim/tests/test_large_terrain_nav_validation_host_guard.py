from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts import large_terrain_nav_validation as gate


def test_large_terrain_preflight_writes_host_guard_without_importing_runtime(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    def fail_load_runtime() -> None:
        raise AssertionError("host guard should run before large-terrain runtime import")

    monkeypatch.setattr(gate, "_load_runtime", fail_load_runtime)
    monkeypatch.setattr(gate, "numpy_import_is_safe", lambda: False)
    monkeypatch.setattr(
        gate,
        "_pct_runtime_evidence",
        lambda: {
            "ok": False,
            "canonical_arch": "x86_64",
            "python_tag": "py313",
            "platform_system": "windows",
            "host_platform_supported": False,
            "host_platform_blocker": "PCT native artifacts are Linux ELF extension modules",
            "error": "No runnable PCT native modules",
        },
    )

    report = gate._environment_preflight_report(
        tmp_path,
        routes=("terrain_short",),
        planners=("pct", "astar"),
        platform_system="Windows",
    )

    assert report is not None
    assert report["ok"] is False
    assert report["execution_mode"] == "host_guard"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["environment"]["accepted_host"] is False
    assert report["environment"]["blocked_reason"] == "windows_mingw_numpy_not_accepted"
    assert report["environment"]["claim_boundary"] == "environment_blocked_no_algorithm_claim"
    assert report["native_runtime"]["python_tag"] == "py313"
    assert "PCT native runtime unavailable" in report["environment_blockers"]
    assert report["cases"][0]["planning"][0]["failure_category"] == "environment_runtime"


def test_large_terrain_preflight_does_not_block_linux_pct_diagnostics(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(gate, "numpy_import_is_safe", lambda: True)
    monkeypatch.setattr(
        gate,
        "_pct_runtime_evidence",
        lambda: {
            "ok": False,
            "canonical_arch": "x86_64",
            "python_tag": "py313",
            "platform_system": "linux",
            "error": "No runnable PCT native modules",
        },
    )

    report = gate._environment_preflight_report(
        tmp_path,
        routes=("terrain_short",),
        planners=("pct", "astar"),
        platform_system="Linux",
    )

    assert report is None
