"""Deployment installer instructions must preserve the ProductControl boundary."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
import uuid
from pathlib import Path

import pytest

from lingtu.assembly.graph import ProcessSpec
from lingtu.run_plan import RunPlan

ROOT = Path(__file__).resolve().parents[2]


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
        required_topics=(),
        required_capabilities=(),
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
    return subprocess.run(
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
    assert result.stdout.removesuffix("\0").split("\0") == [
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
    assert 'systemctl disable "${SERVICE_NAME}"' in script
    assert 'systemctl disable --now "${SERVICE_NAME}"' not in script
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
    guide = _read("docs/operations.md")

    assert "switch / status / stop" in guide
    assert "svc restart" not in guide
    assert "svc reapply" not in guide


def test_web_guide_keeps_restart_ownership_in_product_control() -> None:
    guide = " ".join(_read("docs/operations.md").split())

    assert "可按各自角色独立观测" in guide
    assert "当前 RunPlan 与 ProductControl 统一拥有" in guide
    assert "可被独立观测和重启" not in guide


def _installer_python_fixture(tmp_path: Path) -> tuple[Path, Path, Path, dict[str, str]]:
    package = tmp_path / "package"
    (package / "scripts/deploy").mkdir(parents=True)
    (package / "config").mkdir()
    (package / "bin").mkdir()
    (package / "src").symlink_to(ROOT / "src", target_is_directory=True)
    shutil.copy2(ROOT / "scripts/deploy/python-runtime.sh", package / "scripts/deploy/python-runtime.sh")
    (package / "metadata.json").write_text('{"version":"v0.0.1"}\n', encoding="utf-8")
    (package / "bin/fixture").write_text("fixture\n", encoding="utf-8")
    checksums = subprocess.run(
        ["sha256sum", "bin/fixture"], cwd=package, check=True, capture_output=True, text=True,
    )
    (package / "config/native-release-sha256.txt").write_text(checksums.stdout, encoding="utf-8")
    for executable in ("mapd", "lingtu-mapctl"):
        path = package / "bin" / executable
        path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        path.chmod(0o755)

    state = tmp_path / "state"
    state.mkdir()
    plan_path = state / "run-plan.json"
    current_path = state / "current.json"
    _write_current_run_plan(plan_path, current_path)
    current = json.loads(current_path.read_text(encoding="utf-8"))
    current["run_plan_path"] = str(plan_path)
    current_path.write_text(json.dumps(current), encoding="utf-8")
    old_release = tmp_path / "old-release"
    old_release.mkdir()
    (tmp_path / "current").symlink_to(old_release, target_is_directory=True)

    selected = tmp_path / "venv/bin/python"
    selected.parent.mkdir(parents=True)
    selected.write_text(
        "#!/bin/sh\nset -eu\n"
        'printf "%s|%s|%s\\n" "$0" "$PWD" "$*" >> "$LINGTU_TEST_PYTHON_LOG"\n'
        'if [ "${1:-}" = -m ] && [ "${2:-}" = lingtu.control ]; then\n'
        '  if [ "${LINGTU_TEST_FAIL_NEW:-0}" = 1 ] && [ "${PWD##*/}" != old-release ]; then exit 1; fi\n'
        '  exit 0\nfi\n'
        f"exec {shlex.quote(sys.executable)} \"$@\"\n",
        encoding="utf-8",
    )
    selected.chmod(0o755)
    bad_bin = tmp_path / "system-bin"
    bad_bin.mkdir()
    bad_python = bad_bin / "python3"
    bad_python.write_text(
        '#!/bin/sh\nprintf "system python3 used\\n" >> "$LINGTU_TEST_SYSTEM_PYTHON_LOG"\nexit 99\n',
        encoding="utf-8",
    )
    bad_python.chmod(0o755)
    control = tmp_path / "fake-control"
    control.write_text(
        '#!/bin/sh\nset -eu\ncd "$1"\nexec "$LINGTU_PYTHON" -m lingtu.control\n',
        encoding="utf-8",
    )
    control.chmod(0o755)
    env = os.environ.copy()
    env.pop("LINGTU_PYTHON", None)
    env.pop("LINGTU_PYTHON_ENV", None)
    env.update({
        "PATH": f"{bad_bin}{os.pathsep}{env['PATH']}",
        "LINGTU_CONFIG_DIR": str(tmp_path / "machine-config"),
        "LINGTU_TEST_PYTHON_LOG": str(tmp_path / "python.log"),
        "LINGTU_TEST_SYSTEM_PYTHON_LOG": str(tmp_path / "system-python.log"),
        "LINGTU_NATIVE_RELEASE_TEST_ROOT": str(tmp_path),
        "LINGTU_NATIVE_RELEASE_TEST_CONTROL": str(control),
    })
    return package, state, selected, env


def _install_with_python_fixture(package: Path, state: Path, env: dict[str, str], *args: str):
    return subprocess.run(
        [
            "bash", str(ROOT / "scripts/deploy/install_native_release.sh"),
            "--package-dir", str(package),
            "--releases-dir", str(state.parent / "releases"),
            "--current-link", str(state.parent / "current"),
            "--state-dir", str(state), *args,
        ],
        env=env, capture_output=True, text=True, check=False, timeout=15,
    )


@pytest.mark.skipif(os.name == "nt", reason="Installer behavior requires native Bash")
@pytest.mark.parametrize("fail_new", [False, True])
@pytest.mark.parametrize("explicit_env_file", [False, True])
def test_native_installer_uses_saved_python_for_reads_switch_and_rollback(
    tmp_path: Path, fail_new: bool, explicit_env_file: bool,
) -> None:
    package, state, selected, env = _installer_python_fixture(tmp_path)
    python_env = tmp_path / "machine-config/python.env"
    if explicit_env_file:
        python_env = tmp_path / "custom-python.env"
        env["LINGTU_PYTHON_ENV"] = str(python_env)
    python_env.parent.mkdir(exist_ok=True)
    python_env.write_text(f"LINGTU_PYTHON={shlex.quote(str(selected))}\nexport LINGTU_PYTHON\n", encoding="utf-8")
    env["LINGTU_TEST_FAIL_NEW"] = "1" if fail_new else "0"

    result = _install_with_python_fixture(package, state, env)

    assert (result.returncode != 0) == fail_new, result.stderr
    assert not (tmp_path / "system-python.log").exists(), result.stderr
    calls = (tmp_path / "python.log").read_text(encoding="utf-8").splitlines()
    assert all(call.startswith(f"{selected}|") for call in calls)
    for source in (package / "metadata.json", state / "current.json", state / "run-plan.json"):
        assert any(str(source) in call for call in calls), (source, calls)
    control_calls = [call.split("|")[1] for call in calls if "|-m lingtu.control" in call]
    expected = [str(tmp_path / "releases/v0.0.1")]
    if fail_new:
        expected.append(str(tmp_path / "old-release"))
    assert control_calls == expected
    assert (tmp_path / "current").resolve() == Path(expected[-1])


@pytest.mark.skipif(os.name == "nt", reason="Installer behavior requires native Bash")
def test_native_installer_explicit_python_overrides_saved_machine_python(tmp_path: Path) -> None:
    package, state, selected, env = _installer_python_fixture(tmp_path)
    config = tmp_path / "machine-config"
    config.mkdir()
    (config / "python.env").write_text("LINGTU_PYTHON=/unavailable/saved/python\n", encoding="utf-8")
    env["LINGTU_PYTHON"] = str(selected)

    result = _install_with_python_fixture(package, state, env, "--dry-run")

    assert result.returncode == 0, result.stderr
    assert "Native release verified: v0.0.1" in result.stdout
    assert not (tmp_path / "system-python.log").exists()
    assert not (tmp_path / "releases").exists()


@pytest.mark.skipif(os.name == "nt", reason="Installer behavior requires native Bash")
def test_native_installer_help_does_not_require_python() -> None:
    env = os.environ.copy()
    env["LINGTU_PYTHON"] = "/unavailable/python"

    result = subprocess.run(
        ["bash", str(ROOT / "scripts/deploy/install_native_release.sh"), "--help"],
        env=env, capture_output=True, text=True, check=False, timeout=10,
    )

    assert result.returncode == 0, result.stderr
    assert "Usage: install_native_release.sh" in result.stdout


@pytest.mark.skipif(os.name == "nt", reason="Installer behavior requires native Bash")
def test_native_installer_product_control_command_uses_resolved_python(tmp_path: Path) -> None:
    _, _, selected, env = _installer_python_fixture(tmp_path)
    source = _read("scripts/deploy/install_native_release.sh")
    function = source.split("product_control() {", 1)[1].split("\nrestore_previous_release()", 1)[0]
    script = "product_control() {" + function + '\nproduct_control "$LINGTU_TEST_REPO"\n'
    env.update({
        "PYTHON_BIN": str(selected), "LINGTU_PYTHON": str(selected),
        "TEST_MODE": "0", "ACTIVE_PRODUCT": "nav", "ACTIVE_ROBOT": "unitree/go2",
        "ACTIVE_ENV": "real", "ACTIVE_MAP": "plant-a", "ACTIVE_VARIANT": "camera",
        "STATE_DIR": str(tmp_path / "state"), "LINGTU_TEST_REPO": str(tmp_path / "old-release"),
    })

    result = subprocess.run(["bash", "-c", script], env=env, capture_output=True, text=True, check=False, timeout=10)

    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "system-python.log").exists()
    call = (tmp_path / "python.log").read_text(encoding="utf-8")
    assert call.startswith(f"{selected}|{tmp_path / 'old-release'}|-m lingtu.control switch nav")
    assert "--map plant-a --variant camera" in call
