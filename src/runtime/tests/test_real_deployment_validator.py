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
    assert result["contract"] == "field_dds_v1"
    assert result["endpoint_contract"] == "field_dds_v1"
    assert result["runtime_contract"] == "real"
    assert "host" in result["run_plan_processes"]
    assert "nav" in result["run_plan_processes"]
    assert result["blockers"] == []
    assert "scripts/deploy/thunder/runtime-env.sh" in result["checked_files"]
    assert "scripts/deploy/thunder/lt-driver.service" in result["checked_files"]
    assert "scripts/deploy/deploy_robot.sh" in result["checked_files"]
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

    assert 'ProductControl(env="real", process_env={})._resolve(product)' in source
    assert "resolve_runtime_run_spec" not in source
    assert "EXPECTED_SPEC" not in source
    assert "runtime spec {field}" not in source


def test_validate_real_deployment_rejects_driver_before_nav() -> None:
    plan = validator.ProductControl(env="real", process_env={})._resolve("nav")
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


def test_validate_real_deployment_uses_native_command_service_chain() -> None:
    plan = validator.ProductControl(env="real", process_env={})._resolve("nav")
    blueprint = validator.blueprint_for_resolved_product(
        plan.product,
        dict(plan.host_config),
    )
    entries = {
        entry.name: getattr(entry.module_cls, "__name__", "")
        for entry in blueprint._entries
    }
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in blueprint._wires
    }

    assert entries["nav.goals"] == "GoalService"
    assert entries["nav.commands"] == "Commands"
    assert not validator.FORBIDDEN_GATEWAY_COMMAND_WIRES & wires


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


def test_validate_real_deployment_reports_service_drift(monkeypatch, tmp_path) -> None:
    service = tmp_path / "lt-driver.service"
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
    assert any("consume the Product session environment" in item for item in result["blockers"])


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
