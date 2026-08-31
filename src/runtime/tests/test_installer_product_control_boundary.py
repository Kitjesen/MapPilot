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


def _write_current_run_plan(path: Path, current_path: Path) -> None:
    maps = ProcessSpec(
        name="maps",
        manager="systemd",
        target="lingtu-lt-maps.service",
        order=10,
        timeout_s=30,
        lifecycle="mode",
    )
    driver = ProcessSpec(
        name="driver",
        manager="systemd",
        target="lt-driver.service",
        order=20,
        timeout_s=30,
        lifecycle="mode",
    )
    RunPlan.create(
        product="nav",
        env="real",
        robot="unitree/go2",
        process_control="systemd",
        modules=(),
        processes=(maps, driver),
        available_processes=(maps, driver),
        stop_before_start=(),
        contracts=("lingtu.product.nav.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={},
        native_nav={
            "control_mode": "mapping",
            "global_planner": "octoplanner3d",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    ).write(path)
    current_path.write_text(
        json.dumps(
            {
                "product": "nav",
                "env": "real",
                "map_name": "plant-a",
            }
        ),
        encoding="utf-8",
    )


def _run_native_installer_plan_reader(
    path: Path,
    current_path: Path,
) -> subprocess.CompletedProcess[str]:
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
        [sys.executable, "-", str(path), str(current_path)],
        input=program,
        text=True,
        capture_output=True,
        check=False,
        cwd=ROOT,
        env=env,
    )


def test_native_release_installer_reads_current_product_identity() -> None:
    run_plan_path = ROOT / f".installer-run-plan-{uuid.uuid4().hex}.json"
    current_path = ROOT / f".installer-current-{uuid.uuid4().hex}.json"
    try:
        _write_current_run_plan(run_plan_path, current_path)

        result = _run_native_installer_plan_reader(run_plan_path, current_path)
    finally:
        run_plan_path.unlink(missing_ok=True)
        current_path.unlink(missing_ok=True)

    assert result.returncode == 0, result.stderr
    assert result.stdout.split("\0") == [
        "nav",
        "real",
        "unitree/go2",
        "plant-a",
        "1",
        "",
    ]


def test_native_release_installer_rejects_legacy_run_plan_shape() -> None:
    run_plan_path = ROOT / f".installer-legacy-plan-{uuid.uuid4().hex}.json"
    current_path = ROOT / f".installer-current-{uuid.uuid4().hex}.json"
    try:
        run_plan_path.write_text(
            json.dumps(
                {
                    "product_mode_switch": {
                        "product": {
                            "processes": [
                                {
                                    "name": "maps",
                                    "target": "lingtu-lt-maps.service",
                                    "lifecycle": "persistent",
                                }
                            ]
                        }
                    }
                }
            ),
            encoding="utf-8",
        )
        current_path.write_text("{}\n", encoding="utf-8")

        result = _run_native_installer_plan_reader(run_plan_path, current_path)
    finally:
        run_plan_path.unlink(missing_ok=True)
        current_path.unlink(missing_ok=True)

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
    assert "sudo chown lingtu:lingtu" in script
    assert "/var/lib/lingtu/task_journal" in script
    assert "does not activate Product processes" in script
    assert "scripts/lingtu switch <product>" in script
    assert "--robot <vendor/model> --env real" in script
    assert "reapply" not in script


def test_catalog_installer_boot_enablement_is_catalog_gated() -> None:
    script = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert 'REQUESTED_ENABLE="${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}"' in script
    assert 'if [ "${CATALOG_ENABLE_DEFAULT}" = "1" ]; then' in script
    assert 'systemctl enable "${SERVICE_NAME}"' in script
    assert "ProductControl owns Product role activation" in script


def test_catalog_installer_disables_retired_long_unit_names() -> None:
    script = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert 'SERVICE_NAME="${SERVICE_FILE}"' in script
    assert "LINGTU_SERVICE_NAME" not in script
    assert 'catalog retired-units "${SERVICE}"' in script
    assert 'systemctl disable --now "${unit}"' in script


def test_gateway_key_helper_points_to_product_switch() -> None:
    script = _read("scripts/deploy/thunder/configure_gateway_api_key.sh")

    assert "systemctl restart lt-host.service" not in script
    assert "scripts/lingtu switch <product>" in script
    assert "--robot <vendor/model> --env real" in script
    assert "svc restart" not in script


def test_deployment_guide_uses_product_lifecycle_commands() -> None:
    guide = _read("docs/04-deployment/README.md")

    assert "switch / status / stop" in guide
    assert "svc restart" not in guide
    assert "svc reapply" not in guide


def test_web_guide_keeps_restart_ownership_in_product_control() -> None:
    guide = _read("docs/04-deployment/WEB_GUIDE.md")

    assert "可按各自角色独立观测" in guide
    assert "当前 RunPlan 与 ProductControl 统一拥有" in guide
    assert "可被独立观测和重启" not in guide
