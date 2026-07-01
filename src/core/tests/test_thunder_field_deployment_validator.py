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
    assert result["profile"] == "thunder-nav"
    assert result["canonical_profile"] == "nav"
    assert result["endpoint"] == "thunder_field"
    assert result["contract"] == "thunder_field_lcm_v1"
    assert result["runtime_contract"] == "thunder_field"
    assert result["blockers"] == []
    assert "scripts/deploy/thunder/runtime-env.sh" in result["checked_files"]
    assert result["checked_graph_profiles"] == [
        "thunder-explore",
        "thunder-map",
        "thunder-nav",
    ]


def test_validate_thunder_field_deployment_profile_option_checks_graph() -> None:
    result = validator.validate(profile="thunder-map")

    assert result["ok"] is True
    assert result["profile"] == "thunder-map"
    assert result["canonical_profile"] == "map"
    assert result["endpoint"] == "thunder_field"
    assert "thunder-map" in result["checked_graph_profiles"]


def test_validate_thunder_field_deployment_rejects_ros_compat_graph_module(
    monkeypatch,
) -> None:
    original = validator.blueprint_for_resolved_profile

    class FakeROSBridge:
        pass

    FakeROSBridge.__module__ = "compat.ros2.fake_bridge"

    def _with_ros_bridge(profile, config):
        bp = original(profile, config)
        bp.add(FakeROSBridge, alias="FakeROSBridge")
        return bp

    monkeypatch.setattr(validator, "blueprint_for_resolved_profile", _with_ros_bridge)

    result = validator.validate()

    assert result["ok"] is False
    assert any("contains ROS compatibility module" in item for item in result["blockers"])


def test_validate_thunder_field_deployment_reports_service_drift(monkeypatch, tmp_path) -> None:
    service = tmp_path / "lingtu-thunder-lcm-endpoint.service"
    service.write_text(
        "\n".join(
            [
                "[Service]",
                "Environment=LINGTU_PROFILE=thunder-nav",
                "Environment=LINGTU_ENDPOINT=thunder_field",
                "Environment=LINGTU_ENDPOINT_TRANSPORT=local",
                "Environment=LINGTU_ENDPOINT_CONTRACT=thunder_field_lcm_v1",
                "Environment=LINGTU_ENDPOINT_SOURCES=jsonl",
                "ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash'",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "SERVICE_PATH", service)

    result = validator.validate()

    assert result["ok"] is False
    assert any("LINGTU_ENDPOINT_TRANSPORT expected 'lcm'" in item for item in result["blockers"])
    assert any("must include thunder_field" in item for item in result["blockers"])
    assert any("must not source ROS" in item for item in result["blockers"])


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
