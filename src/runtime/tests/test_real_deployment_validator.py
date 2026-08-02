from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

from tools.validate import validate_real_deployment as validator

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_validate_real_deployment_contract_passes() -> None:
    result = validator.validate()

    assert result["ok"] is True
    assert result["product"] == "nav"
    assert result["canonical_product"] == "nav"
    assert result["env"] == "real"
    assert result["contract"] == "thunder_dds_v1"
    assert result["endpoint_contract"] == "thunder_dds_v1"
    assert result["runtime_contract"] == "real"
    assert isinstance(result["run_plan_fingerprint"], str)
    assert len(result["run_plan_fingerprint"]) == 64
    assert "host" in result["run_plan_processes"]
    assert "nav" in result["run_plan_processes"]
    assert result["blockers"] == []
    assert "scripts/deploy/thunder/runtime-env.sh" in result["checked_files"]
    assert "scripts/deploy/thunder/lingtu-driver.service" in result["checked_files"]
    assert "src/lingtu/control.py" in result["checked_files"]
    assert "src/lingtu/run_plan.py" in result["checked_files"]
    assert result["checked_graph_products"] == [
        "explore:live",
        "explore:map",
        "map",
        "nav",
    ]


def test_validate_real_deployment_uses_run_plan_not_runtime_run_spec() -> None:
    source = (REPO_ROOT / "tools" / "validate" / "validate_real_deployment.py").read_text(encoding="utf-8-sig")

    assert 'ProductControl(env="real", process_env={}).resolve(product)' in source
    assert "resolve_runtime_run_spec" not in source
    assert "EXPECTED_SPEC" not in source
    assert '"LINGTU_PROFILE":' not in source
    assert "runtime spec {field}" not in source


def test_validate_real_deployment_rejects_profile_env_in_run_plan() -> None:
    class FakePlan:
        product = "nav"
        env = "real"
        process_control = "systemd"
        module_transport = "local"
        fingerprint = "a" * 64
        native_process_environment = {
            "LINGTU_PRODUCT": "nav",
            "LINGTU_PROFILE": "nav",
            "LINGTU_NAV_CONTROL_MODE": "autonomy",
            "LINGTU_NAV_PUBLISH_CMD_VEL": "1",
            "LINGTU_NAV_CHECK_OBSTACLE": "1",
            "LINGTU_NAV_USE_TRAVERSABILITY_COST": "1",
            "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": "1",
            "LINGTU_TELEOP_LOCAL_PLANNER": "1",
        }

        def has_process(self, name: str) -> bool:
            return name in {"host", "nav"}

    blockers: list[str] = []
    validator._validate_runtime_layers(FakePlan(), dict(validator.EXPECTED_CONFIG), blockers)

    assert any("LINGTU_PROFILE" in blocker for blocker in blockers)


def test_validate_real_deployment_rejects_driver_before_nav() -> None:
    plan = validator.ProductControl(env="real", process_env={}).resolve("nav")
    bad_processes = tuple(
        replace(process, order=60) if process.name == "nav" else process
        for process in plan.processes
    )
    bad_plan = replace(plan, processes=bad_processes)
    blockers: list[str] = []

    validator._validate_runtime_layers(
        bad_plan,
        dict(bad_plan.host_config),
        blockers,
    )

    assert "RunPlan must start the native nav command producer before the driver" in blockers


def test_validate_real_deployment_product_option_checks_graph() -> None:
    result = validator.validate(product="map")

    assert result["ok"] is True
    assert result["product"] == "map"
    assert result["canonical_product"] == "map"
    assert result["env"] == "real"
    assert "map" in result["checked_graph_products"]


def test_validate_real_deployment_rejects_ros_compat_graph_module(
    monkeypatch,
) -> None:
    original = validator.blueprint_for_resolved_product

    class FakeROSBridge:
        pass

    FakeROSBridge.__module__ = "runtime.adapters.ros2.fake_bridge"

    def _with_ros_bridge(product, config):
        bp = original(product, config)
        bp.add(FakeROSBridge, alias="FakeROSBridge")
        return bp

    monkeypatch.setattr(validator, "blueprint_for_resolved_product", _with_ros_bridge)

    result = validator.validate()

    assert result["ok"] is False
    assert any("contains ROS compatibility module" in item for item in result["blockers"])


def test_validate_real_deployment_rejects_python_mux_in_endpoint_only_graph(
    monkeypatch,
) -> None:
    original = validator.blueprint_for_resolved_product

    class FakeVelocityMux:
        pass

    def _with_python_mux(product, config):
        bp = original(product, config)
        bp.add(FakeVelocityMux, alias="nav.velocity_mux")
        return bp

    monkeypatch.setattr(validator, "blueprint_for_resolved_product", _with_python_mux)

    result = validator.validate()

    assert result["ok"] is False
    assert any("endpoint-only Python control module nav.velocity_mux" in item for item in result["blockers"])


def test_validate_real_deployment_reports_service_drift(monkeypatch, tmp_path) -> None:
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


def test_validate_real_deployment_cli_json() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "tools/validate/validate_real_deployment.py",
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
    assert payload["env"] == "real"


def test_validate_real_deployment_rejects_legacy_deploy_lifecycle(monkeypatch, tmp_path) -> None:
    deploy = tmp_path / "deploy_thunder.sh"
    deploy.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                'PROFILE="${LINGTU_DEPLOY_PROFILE:-nav}"',
                'export LINGTU_PROFILE="${PROFILE}"',
                'nohup python3 lingtu.py "${PROFILE}" --daemon &',
                "rm -f .lingtu/run.json .lingtu/run.pid",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "DEPLOY_PATH", deploy)

    result = validator.validate()

    assert result["ok"] is False
    assert any("LINGTU_DEPLOY_PROFILE" in item for item in result["blockers"])
    assert any("nohup" in item for item in result["blockers"])
    assert any(".lingtu/run.json" in item for item in result["blockers"])


def test_validate_real_deployment_rejects_profile_named_deploy_activation(monkeypatch, tmp_path) -> None:
    deploy = tmp_path / "deploy_thunder.sh"
    deploy.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                'PRODUCT="${LINGTU_DEPLOY_PRODUCT:-${1:-}}"',
                (
                    'is_field_product() { case "${PRODUCT}" in '
                    '""|teleop|teleop_avoid|map|explore|nav|tracking|inspection|tare_explore) '
                    "return 0;; esac; }"
                ),
                'bash "${REPO}/scripts/build/build_driver.sh"',
                'bash "${REPO}/scripts/deploy/thunder/install_services.sh" field-cpp',
                'bash "${REPO}/scripts/lingtu" --env real mode switch "${PRODUCT}"',
                'echo "${LINGTU_DEPLOY_MAP:-}"',
                'case "${PRODUCT}" in lite|thunder-lite|basic|thunder-basic) return 0;; esac',
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(validator, "DEPLOY_PATH", deploy)

    result = validator.validate()

    assert result["ok"] is False
    for retired_name in (
        "tare_explore",
        "lite",
        "thunder-lite",
        "basic",
        "thunder-basic",
    ):
        assert any(
            f"forbidden legacy deploy Product name {retired_name!r}" in item
            for item in result["blockers"]
        )
