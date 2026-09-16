import json
import time

import pytest

from gateway.navigation.status import _native_endpoint_readiness


def _write_native_status(path, loop_health, *, hold=False):
    path.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "input_gate": {"ready": True, "reason": "ready"},
                "active_cmd_source": "none",
                "control_authority": {
                    "owner": "native_endpoint",
                    "estop_latched": False,
                    "operator_takeover_latched": False,
                    "resume_required": False,
                    "control_loop_hold": hold,
                },
                "control_mode": "autonomy",
                "global_planner": "octoplanner3d",
                "planner_map": "/maps/active/octomap.ot",
                "publish_cmd_vel": True,
                "navigation_ready": not hold,
                "control_loop_health": loop_health,
            }
        ),
        encoding="utf-8",
    )


def _configure_native_status(monkeypatch, status_path):
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")


def test_native_endpoint_readiness_surfaces_loop_health_warmup_without_blocking(
    monkeypatch,
    tmp_path,
):
    status_path = tmp_path / "nav_endpoint_status.json"
    _configure_native_status(monkeypatch, status_path)
    _write_native_status(
        status_path,
        {
            "ready": False,
            "healthy": False,
            "reason": "warming_up",
            "window_samples": 20,
        },
    )

    result = _native_endpoint_readiness({"mode": "navigating"})

    assert result["ok"] is True
    assert result["control_loop_health"]["reason"] == "warming_up"
    assert "native_control_loop_unhealthy" not in result["blockers"]
    assert "native_control_loop_health_unavailable" not in result["blockers"]


def test_native_endpoint_readiness_blocks_missing_loop_health(
    monkeypatch,
    tmp_path,
):
    status_path = tmp_path / "nav_endpoint_status.json"
    _configure_native_status(monkeypatch, status_path)
    _write_native_status(status_path, None)

    result = _native_endpoint_readiness({"mode": "navigating"})

    assert result["ok"] is False
    assert result["control_loop_health"] == {}
    assert "native_control_loop_health_unavailable" in result["blockers"]


@pytest.mark.parametrize("reason", ["p95_utilization_high", "deadline_miss_ratio_high"])
def test_native_endpoint_readiness_preserves_native_permission_during_rolling_warning(
    monkeypatch,
    tmp_path,
    reason,
):
    status_path = tmp_path / "nav_endpoint_status.json"
    _configure_native_status(monkeypatch, status_path)
    _write_native_status(
        status_path,
        {
            "ready": True,
            "healthy": False,
            "reason": reason,
            "window_samples": 600,
            "deadline_miss_ratio": 0.08,
        },
    )

    result = _native_endpoint_readiness({"mode": "navigating"})

    assert result["ok"] is True
    assert result["control_loop_health"]["healthy"] is False
    assert "native_control_loop_unhealthy" not in result["blockers"]


@pytest.mark.parametrize("healthy", [False, True])
def test_native_endpoint_readiness_keeps_native_hold_until_explicit_recovery(
    monkeypatch, tmp_path, healthy,
):
    status_path = tmp_path / "nav_endpoint_status.json"
    _configure_native_status(monkeypatch, status_path)
    _write_native_status(status_path, {
        "ready": True, "healthy": healthy,
        "reason": "healthy" if healthy else "consecutive_deadline_misses",
    }, hold=True)

    result = _native_endpoint_readiness({"mode": "navigating"})
    assert result["ok"] is False
    assert "native_control_loop_unhealthy" in result["blockers"]


def test_native_endpoint_readiness_accepts_mature_healthy_loop(
    monkeypatch,
    tmp_path,
):
    status_path = tmp_path / "nav_endpoint_status.json"
    _configure_native_status(monkeypatch, status_path)
    _write_native_status(
        status_path,
        {
            "ready": True,
            "healthy": True,
            "reason": "healthy",
            "window_samples": 600,
            "deadline_miss_ratio": 0.0,
        },
    )

    result = _native_endpoint_readiness({"mode": "navigating"})

    assert result["ok"] is True
    assert result["control_loop_health"]["healthy"] is True
    assert "native_control_loop_unhealthy" not in result["blockers"]

