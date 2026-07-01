from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml
from tools import package_thunder_lite as packager
from tools.validate import validate_thunder_lite_package as validator

ROOT = Path(__file__).resolve().parents[3]


def test_thunder_lite_package_manifest_names_minimal_runtime_surface() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))

    assert manifest["profile"] == "thunder-lite"
    assert manifest["runtime"]["expected_config"]["slam_profile"] == "none"
    assert manifest["runtime"]["expected_config"]["enable_gateway"] is False
    assert manifest["runtime"]["expected_config"]["enable_semantic"] is False
    assert manifest["runtime"]["expected_config"]["enable_map_modules"] is False
    assert manifest["runtime"]["expected_config"]["enable_gnss"] is False
    assert manifest["runtime"]["expected_spec"]["module_transport"] == "local"
    assert manifest["runtime"]["expected_spec"]["endpoint_transport"] == "local"
    assert manifest["runtime"]["expected_spec"]["simulation_only"] is False
    assert (
        manifest["runtime"]["expected_spec"]["command_sink"]
        == "hardware_driver_after_cmd_vel_mux"
    )
    assert "scripts/deploy/thunder/runtime-env.sh" in manifest["deploy"]["required_files"]


def test_thunder_lite_package_manifest_declares_package_boundary() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    package = manifest["package"]
    include_paths = {validator._normalize_manifest_path(path) for path in package["include_paths"]}
    exclude_paths = {validator._normalize_manifest_path(path) for path in package["exclude_paths"]}
    omit_paths = {validator._normalize_manifest_path(path) for path in package["omit_paths"]}

    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_INCLUDE_PATHS
    } <= include_paths
    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_EXCLUDE_PATHS
    } <= exclude_paths
    assert {
        validator._normalize_manifest_path(path)
        for path in validator.REQUIRED_PACKAGE_OMIT_PATHS
    } <= omit_paths
    assert "src/slam" not in include_paths
    assert "src/compat/ros2" not in include_paths


def test_pyproject_keeps_gateway_and_heavy_dependencies_out_of_core_install() -> None:
    base_dependencies, optional_dependencies = validator.load_pyproject_dependencies(ROOT / "pyproject.toml")
    base_names = {validator._dependency_name(dep) for dep in base_dependencies}

    assert "lite" in optional_dependencies
    assert optional_dependencies["lite"] == []
    assert {"fastapi", "uvicorn", "websockets"}.isdisjoint(base_names)
    assert {"fastapi", "uvicorn", "websockets"} <= {
        validator._dependency_name(dep) for dep in optional_dependencies["gateway"]
    }
    assert {"torch", "ultralytics", "chromadb", "openai", "anthropic"}.isdisjoint(base_names)


def test_requirements_lite_contains_only_minimal_python_runtime_dependencies() -> None:
    requirements = validator._read_requirements(ROOT / "requirements-lite.txt")
    names = {validator._dependency_name(dep) for dep in requirements}

    assert {"numpy", "scipy", "pyyaml", "pydantic"} <= names
    assert {"fastapi", "uvicorn", "websockets", "torch", "ultralytics", "chromadb"}.isdisjoint(names)


def test_runtime_env_defaults_match_thunder_lite_runtime_spec() -> None:
    defaults = validator._parse_shell_default_env(
        (ROOT / "scripts/deploy/thunder/runtime-env.sh").read_text(
            encoding="utf-8-sig"
        )
    )

    assert defaults["LINGTU_PROFILE"] == "thunder-lite"
    assert defaults["LINGTU_MODULE_TRANSPORT"] == "local"
    assert defaults["LINGTU_ENDPOINT"] == "thunder_lite"
    assert defaults["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert defaults["LINGTU_ENDPOINT_CONTRACT"] == ""
    assert defaults["LINGTU_SIMULATION_ONLY"] == "0"
    assert defaults["LINGTU_COMMAND_OUTPUT_MODE"] == "local_driver"
    assert defaults["LINGTU_HARDWARE_CONTROL_BOUNDARY"] == "module_graph_driver"


def test_thunder_lite_package_validator_rejects_ros_compat_graph_module() -> None:
    class FakeROSBridge:
        pass

    FakeROSBridge.__module__ = "compat.ros2.fake_bridge"
    fake_entry = type(
        "FakeEntry",
        (),
        {"name": "FakeROSBridge", "module_cls": FakeROSBridge},
    )()
    fake_blueprint = type("FakeBlueprint", (), {"_entries": [fake_entry]})()
    blockers: list[str] = []

    validator._validate_product_graph("thunder-lite", fake_blueprint, blockers)

    assert any("contains ROS compatibility module" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_slam_package_graph_module() -> None:
    class FakeGnss:
        pass

    FakeGnss.__module__ = "slam.gnss_module"
    fake_entry = type(
        "FakeEntry",
        (),
        {"name": "GnssModule", "module_cls": FakeGnss},
    )()
    fake_blueprint = type("FakeBlueprint", (), {"_entries": [fake_entry]})()
    blockers: list[str] = []

    validator._validate_product_graph("thunder-lite", fake_blueprint, blockers)

    assert any("contains SLAM package module" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_missing_required_package_exclusion() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    manifest["package"]["exclude_paths"] = [
        path
        for path in manifest["package"]["exclude_paths"]
        if validator._normalize_manifest_path(path) != "src/slam"
    ]
    blockers: list[str] = []

    validator._validate_package_boundary(manifest, blockers, [])

    assert any("missing required Lite exclusions" in item for item in blockers)


def test_thunder_lite_package_validator_rejects_missing_required_package_omit() -> None:
    manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    manifest["package"]["omit_paths"] = [
        path
        for path in manifest["package"]["omit_paths"]
        if validator._normalize_manifest_path(path) != "src/core/dds.py"
    ]
    blockers: list[str] = []

    validator._validate_package_boundary(manifest, blockers, [])

    assert any("missing required Lite omissions" in item for item in blockers)


def test_thunder_lite_packager_dry_run_applies_package_boundary(tmp_path: Path) -> None:
    summary = packager.build_package(
        output_dir=tmp_path / "thunder-lite",
        dry_run=True,
    )
    copied_files = set(summary["copied_files"])

    assert "lingtu.py" in copied_files
    assert "cli/lifecycle.py" in copied_files
    assert "cli/runtime_audit.py" not in copied_files
    assert "cli/runtime_extra.py" not in copied_files
    assert "src/core/dds.py" not in copied_files
    assert "src/core/native_install.py" not in copied_files
    assert "src/core/blueprints/full_stack.py" not in copied_files
    assert "src/core/blueprints/full_stack_wiring.py" not in copied_files
    assert "src/core/blueprints/stacks/slam.py" not in copied_files
    assert "src/core/blueprints/stacks/gateway.py" not in copied_files
    assert "src/core/blueprints/wires/slam.py" not in copied_files
    assert not any(path.startswith("src/core/devices/") for path in copied_files)
    assert not any(path.startswith("src/slam/") for path in copied_files)
    assert not any(path.startswith("src/compat/ros2/") for path in copied_files)
    assert not any("/tests/" in f"/{path}/" for path in copied_files)


def test_thunder_lite_packager_builds_filtered_package(tmp_path: Path) -> None:
    output_dir = tmp_path / "thunder-lite"
    summary = packager.build_package(output_dir=output_dir)

    assert summary["dry_run"] is False
    assert summary["file_count"] > 0
    assert (output_dir / "lingtu.py").is_file()
    assert (output_dir / "cli" / "lifecycle.py").is_file()
    assert not (output_dir / "cli" / "runtime_audit.py").exists()
    assert not (output_dir / "cli" / "runtime_extra.py").exists()
    assert (output_dir / "src" / "core" / "blueprint.py").is_file()
    assert (output_dir / "src" / "drivers" / "real" / "thunder").is_dir()
    assert not (output_dir / "src" / "core" / "blueprints" / "full_stack.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "full_stack_wiring.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "stacks" / "slam.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "stacks" / "gateway.py").exists()
    assert not (output_dir / "src" / "core" / "blueprints" / "wires").exists()
    assert not (output_dir / "src" / "core" / "devices").exists()
    assert not (output_dir / "src" / "slam").exists()
    assert not (output_dir / "src" / "compat" / "ros2").exists()
    assert not (output_dir / "src" / "core" / "tests").exists()
    assert not (output_dir / "src" / "core" / "dds.py").exists()
    assert (output_dir / packager.SUMMARY_FILENAME).is_file()


def test_thunder_lite_package_validator_passes_json_mode() -> None:
    result = subprocess.run(
        [sys.executable, "tools/validate/validate_thunder_lite_package.py", "--json"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["blockers"] == []
    assert "requirements-lite.txt" in payload["checked_files"]
