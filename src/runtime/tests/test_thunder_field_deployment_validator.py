from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from tools.validate import validate_thunder_field_deployment as validator

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_validate_thunder_field_deployment_contract_passes() -> None:
    result = validator.validate()

    assert result["ok"] is True
    assert result["profile"] == "nav"
    assert result["canonical_profile"] == "nav"
    assert result["endpoint"] == "thunder_field"
    assert result["contract"] == "thunder_field_dds_v1"
    assert result["runtime_endpoint_contract"] == "thunder_field_dds_v1"
    assert result["runtime_contract"] == "thunder_field"
    assert result["blockers"] == []
    assert "scripts/deploy/thunder/runtime-env.sh" in result["checked_files"]
    assert "scripts/deploy/thunder/lingtu-driver.service" in result["checked_files"]
    assert result["checked_graph_profiles"] == [
        "map",
        "nav",
        "tare_explore",
    ]


def test_validate_thunder_field_deployment_profile_option_checks_graph() -> None:
    result = validator.validate(profile="map")

    assert result["ok"] is True
    assert result["profile"] == "map"
    assert result["canonical_profile"] == "map"
    assert result["endpoint"] == "thunder_field"
    assert "map" in result["checked_graph_profiles"]


def test_validate_thunder_field_deployment_rejects_ros_compat_graph_module(
    monkeypatch,
) -> None:
    original = validator.blueprint_for_resolved_profile

    class FakeROSBridge:
        pass

    FakeROSBridge.__module__ = "runtime.adapters.ros2.fake_bridge"

    def _with_ros_bridge(profile, config):
        bp = original(profile, config)
        bp.add(FakeROSBridge, alias="FakeROSBridge")
        return bp

    monkeypatch.setattr(validator, "blueprint_for_resolved_profile", _with_ros_bridge)

    result = validator.validate()

    assert result["ok"] is False
    assert any("contains ROS compatibility module" in item for item in result["blockers"])


def test_validate_thunder_field_deployment_rejects_python_mux_in_endpoint_only_graph(
    monkeypatch,
) -> None:
    original = validator.blueprint_for_resolved_profile

    class FakeVelocityMux:
        pass

    def _with_python_mux(profile, config):
        bp = original(profile, config)
        bp.add(FakeVelocityMux, alias="nav.velocity_mux")
        return bp

    monkeypatch.setattr(validator, "blueprint_for_resolved_profile", _with_python_mux)

    result = validator.validate()

    assert result["ok"] is False
    assert any("endpoint-only Python control module nav.velocity_mux" in item for item in result["blockers"])


def test_validate_thunder_field_deployment_reports_service_drift(monkeypatch, tmp_path) -> None:
    service = tmp_path / "lingtu-driver.service"
    service.write_text(
        "\n".join(
            [
                "[Service]",
                "Environment=LINGTU_DRIVER_BIN=/tmp/not-the-driver",
                "Environment=LINGTU_BRAINSTEM_HOST=127.0.0.1",
                "EnvironmentFile=-/opt/lingtu/config/brainstem.env",
                "ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && python bad.py'",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "SERVICE_PATH", service)

    result = validator.validate()

    assert result["ok"] is False
    assert any("LINGTU_DRIVER_BIN expected" in item for item in result["blockers"])
    assert any("must not source ROS" in item for item in result["blockers"])
    assert any("must not execute Python" in item for item in result["blockers"])
    assert any("require the remote Brainstem" in item for item in result["blockers"])


def test_validate_thunder_field_deployment_cli_json() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "tools/validate/validate_thunder_field_deployment.py",
            "--json",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["endpoint"] == "thunder_field"
