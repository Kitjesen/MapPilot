"""Deployment installer instructions must preserve the ProductControl boundary."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import uuid
from pathlib import Path

from lingtu.run_plan import RunPlan
from runtime.graph import ProcessSpec

ROOT = Path(__file__).resolve().parents[3]


def _read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def _write_current_run_plan(path: Path) -> None:
    maps = ProcessSpec(
        name="maps",
        manager="systemd",
        target="lingtu-mapd.service",
        order=10,
        timeout_s=30,
        lifecycle="mode",
    )
    driver = ProcessSpec(
        name="driver",
        manager="systemd",
        target="lingtu-driver.service",
        order=20,
        timeout_s=30,
        lifecycle="persistent",
    )
    RunPlan.create(
        product="map",
        env="real",
        process_control="systemd",
        modules=(),
        processes=(maps, driver),
        available_processes=(maps, driver),
        stop_targets=(),
        contracts=("lingtu.product.map.v1",),
        critical_modules=(),
        route_contract=None,
        module_transport="local",
        host_config={},
        lifecycle={},
        compiled_against={},
        native_nav={
            "control_mode": "mapping",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    ).write(path)


def _run_native_installer_plan_reader(path: Path) -> subprocess.CompletedProcess[str]:
    source = _read("scripts/deploy/install_native_release.sh")
    import_anchor = "from lingtu.run_plan import RunPlan"
    import_offset = source.index(import_anchor)
    program_start = source.rfind("<<'PY'\n", 0, import_offset) + len("<<'PY'\n")
    program_end = source.index("\nPY\n", import_offset)
    program = source[program_start:program_end]
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        item
        for item in (str(ROOT / "src"), env.get("PYTHONPATH", ""))
        if item
    )
    return subprocess.run(  # noqa: S603 - executes the checked-in installer reader.
        [sys.executable, "-", str(path)],
        input=program,
        text=True,
        capture_output=True,
        check=False,
        cwd=ROOT,
        env=env,
    )


def test_native_release_installer_reads_current_run_plan_process_catalog() -> None:
    run_plan_path = ROOT / f".installer-run-plan-{uuid.uuid4().hex}.json"
    try:
        _write_current_run_plan(run_plan_path)

        result = _run_native_installer_plan_reader(run_plan_path)
    finally:
        run_plan_path.unlink(missing_ok=True)

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["1", "driver"]


def test_native_release_installer_rejects_legacy_run_plan_shape() -> None:
    run_plan_path = ROOT / f".installer-legacy-plan-{uuid.uuid4().hex}.json"
    try:
        run_plan_path.write_text(
            json.dumps(
                {
                    "product_mode_switch": {
                        "product": {
                            "processes": [
                                {
                                    "name": "maps",
                                    "target": "lingtu-mapd.service",
                                    "lifecycle": "persistent",
                                }
                            ]
                        }
                    }
                }
            ),
            encoding="utf-8",
        )

        result = _run_native_installer_plan_reader(run_plan_path)
    finally:
        run_plan_path.unlink(missing_ok=True)

    assert result.returncode != 0
    assert "RunPlan top level has invalid fields" in result.stderr
    installer = _read("scripts/deploy/install_native_release.sh")
    assert 'PYTHONPATH="${PACKAGE_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"' in installer
    assert "from lingtu.run_plan import RunPlan" in installer
    assert "plan = RunPlan.load(sys.argv[1])" in installer
    assert "plan.processes" in installer
    assert "product_mode_switch" not in installer
    assert "set(payload)" not in installer
    assert 'catalog.get("selected")' not in installer


def test_catalog_installer_does_not_activate_a_product_process() -> None:
    script = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert "systemctl start" not in script
    assert "does not activate Product processes" in script
    assert "scripts/lingtu --env real svc reapply" in script
    assert "scripts/lingtu --env real mode switch <product>" in script


def test_catalog_installer_boot_enablement_is_catalog_gated() -> None:
    script = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert 'REQUESTED_ENABLE="${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}"' in script
    assert 'if [ "${CATALOG_ENABLE_DEFAULT}" = "1" ]; then' in script
    assert 'systemctl enable "${SERVICE_NAME}"' in script
    assert "ProductControl owns Product role activation" in script


def test_gateway_key_helper_delegates_host_restart_to_product_control() -> None:
    script = _read("scripts/deploy/thunder/configure_gateway_api_key.sh")

    assert "systemctl restart lingtu.service" not in script
    assert "scripts/lingtu --env real svc restart host" in script


def test_slam_restart_documentation_keeps_bash_thin() -> None:
    guide = _read("docs/04-deployment/README.md")

    assert "`svc restart slam` is a thin operator entry" in guide
    assert "sends the logical `slam` label" in guide
    assert "from the committed RunPlan and owns its readiness and failure handling" in guide
    assert "does not select a backend, sequence a service chain, or poll readiness" in guide


def test_web_guide_keeps_restart_ownership_in_product_control() -> None:
    guide = _read("docs/04-deployment/WEB_GUIDE.md")

    assert "可按各自角色独立观测" in guide
    assert "当前 RunPlan 与 ProductControl 统一拥有" in guide
    assert "可被独立观测和重启" not in guide
