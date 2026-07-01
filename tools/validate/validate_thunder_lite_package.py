#!/usr/bin/env python3
"""Validate the Thunder Lite package and deployment contract."""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from pathlib import Path
from typing import Any

import yaml

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
MANIFEST_PATH = ROOT_DIR / "config" / "thunder_lite_package.yaml"
PYPROJECT_PATH = ROOT_DIR / "pyproject.toml"
RUNTIME_ENV_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "runtime-env.sh"
FORBIDDEN_PRODUCT_MODULE_PREFIXES = {
    "compat.ros2": "ROS compatibility",
    "slam.": "SLAM package",
}
REQUIRED_PACKAGE_INCLUDE_PATHS = (
    "lingtu.py",
    "requirements-lite.txt",
    "config/thunder_lite_package.yaml",
    "scripts/deploy/thunder/runtime-env.sh",
    "scripts/deploy/thunder/lingtu-thunder-lite.service",
    "scripts/deploy/thunder/install_lite_service.sh",
    "src/core/",
    "src/lingtu_runtime/",
    "src/drivers/real/thunder/",
    "src/nav/navigation_module.py",
    "src/nav/safety_ring_module.py",
    "src/nav/cmd_vel_mux_module.py",
    "src/base_autonomy/modules/",
)
REQUIRED_PACKAGE_EXCLUDE_PATHS = (
    "src/compat/ros2/",
    "src/slam/",
    "src/semantic/",
    "src/memory/",
    "src/gateway/",
    "src/webrtc/",
    "src/exploration/",
    "src/drivers/sim/",
    "src/drivers/livox_ros_driver2/",
    "src/global_planning/",
    "sim/",
    "web/",
    "third_party/",
    "calibration/",
    "launch/",
)
REQUIRED_PACKAGE_OMIT_PATHS = (
    "**/__pycache__/",
    "**/*.pyc",
    "**/tests/",
    "cli/runtime_audit.py",
    "cli/runtime_extra.py",
    "src/core/blueprints/full_stack.py",
    "src/core/blueprints/full_stack_wiring.py",
    "src/core/blueprints/multi_robot.py",
    "src/core/blueprints/profile_graph.py",
    "src/core/blueprints/simulation_contract.py",
    "src/core/blueprints/stacks/composition.py",
    "src/core/blueprints/stacks/exploration.py",
    "src/core/blueprints/stacks/gateway.py",
    "src/core/blueprints/stacks/lidar.py",
    "src/core/blueprints/stacks/maps.py",
    "src/core/blueprints/stacks/memory.py",
    "src/core/blueprints/stacks/perception.py",
    "src/core/blueprints/stacks/planner.py",
    "src/core/blueprints/stacks/sim_lidar.py",
    "src/core/blueprints/stacks/slam.py",
    "src/core/blueprints/stacks/system.py",
    "src/core/blueprints/wires/",
    "src/core/dds.py",
    "src/core/devices/",
    "src/core/dimos_gap.py",
    "src/core/native_install.py",
    "src/core/rerun_module.py",
    "src/core/transport/dds.py",
    "src/core/transport/dual.py",
    "src/core/transport/lcm.py",
    "src/core/transport/shm.py",
)
REQUIRED_PACKAGE_FORBIDDEN_MARKERS = (
    "rclpy",
    "sensor_msgs",
    "nav_msgs",
    "geometry_msgs",
    "launch_ros",
    "/opt/ros",
    "colcon",
)

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def _dependency_name(requirement: str) -> str:
    value = str(requirement).strip().lower()
    value = value.split(";", 1)[0].strip()
    value = value.split(" @ ", 1)[0].strip()
    value = re.split(r"[\[<>=!~]", value, maxsplit=1)[0].strip()
    return value.replace("_", "-")


def _strip_toml_comment(line: str) -> str:
    """Strip TOML comments without truncating # inside quoted strings."""

    quote: str | None = None
    escaped = False
    for index, char in enumerate(line):
        if escaped:
            escaped = False
            continue
        if char == "\\" and quote is not None:
            escaped = True
            continue
        if char in {'"', "'"}:
            if quote == char:
                quote = None
            elif quote is None:
                quote = char
            continue
        if char == "#" and quote is None:
            return line[:index]
    return line


def _parse_toml_string_lists(text: str) -> tuple[list[str], dict[str, list[str]]]:
    project_dependencies: list[str] = []
    optional_dependencies: dict[str, list[str]] = {}
    current_section = ""
    collecting_key: str | None = None
    collecting_section = ""
    collecting_lines: list[str] = []

    def finish_collection() -> None:
        nonlocal collecting_key, collecting_section, collecting_lines, project_dependencies
        if collecting_key is None:
            return
        value = ast.literal_eval(" ".join(collecting_lines))
        if collecting_section == "project" and collecting_key == "dependencies":
            project_dependencies = list(value)
        elif collecting_section == "project.optional-dependencies":
            optional_dependencies[collecting_key] = list(value)
        collecting_key = None
        collecting_section = ""
        collecting_lines = []

    for raw_line in text.splitlines():
        line = _strip_toml_comment(raw_line).strip()
        if not line:
            continue
        if collecting_key is not None:
            collecting_lines.append(line)
            if "]" in line:
                finish_collection()
            continue
        if line.startswith("[") and line.endswith("]"):
            current_section = line.strip("[]")
            continue
        if current_section not in {"project", "project.optional-dependencies"}:
            continue
        if "=" not in line:
            continue
        key, raw_value = (part.strip() for part in line.split("=", 1))
        if not raw_value.startswith("["):
            continue
        if "]" in raw_value:
            value = ast.literal_eval(raw_value)
            if current_section == "project" and key == "dependencies":
                project_dependencies = list(value)
            elif current_section == "project.optional-dependencies":
                optional_dependencies[key] = list(value)
            continue
        collecting_key = key
        collecting_section = current_section
        collecting_lines = [raw_value]

    finish_collection()
    return project_dependencies, optional_dependencies


def load_pyproject_dependencies(path: Path = PYPROJECT_PATH) -> tuple[list[str], dict[str, list[str]]]:
    """Return base and optional dependency declarations from pyproject.toml."""

    try:
        import tomllib  # type: ignore[attr-defined]
    except ModuleNotFoundError:
        tomllib = None  # type: ignore[assignment]

    if tomllib is not None:
        data = tomllib.loads(path.read_text(encoding="utf-8-sig"))
        project = data.get("project") or {}
        return (
            list(project.get("dependencies") or []),
            {
                str(key): list(value or [])
                for key, value in (project.get("optional-dependencies") or {}).items()
            },
        )
    return _parse_toml_string_lists(path.read_text(encoding="utf-8-sig"))


def _read_requirements(path: Path) -> list[str]:
    requirements: list[str] = []
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = _strip_toml_comment(raw_line).strip()
        if line:
            requirements.append(line)
    return requirements


def _parse_shell_default_env(text: str) -> dict[str, str]:
    defaults: dict[str, str] = {}
    pattern = re.compile(r':\s*"\$\{([A-Za-z_][A-Za-z0-9_]*):=([^}]*)\}"')
    for raw_line in text.splitlines():
        match = pattern.search(raw_line.strip())
        if match:
            defaults[match.group(1)] = match.group(2)
    return defaults


def _load_manifest(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def _normalize_manifest_path(value: Any) -> str:
    return str(value).replace("\\", "/").strip().lstrip("./").rstrip("/")


def _validate_deploy_files(manifest: dict[str, Any], blockers: list[str], checked_files: list[str]) -> None:
    deploy = manifest.get("deploy") or {}
    required_files = tuple(deploy.get("required_files") or ())
    forbidden_markers = tuple(str(marker) for marker in deploy.get("forbidden_markers") or ())
    if not required_files:
        blockers.append("deploy.required_files must not be empty")
        return

    for rel_path in required_files:
        path = ROOT_DIR / str(rel_path)
        checked_files.append(str(rel_path))
        if not path.is_file():
            blockers.append(f"{rel_path}: required deploy file is missing")
            continue
        text = path.read_text(encoding="utf-8-sig", errors="ignore")
        folded = text.lower()
        for marker in forbidden_markers:
            if marker.lower() in folded:
                blockers.append(f"{rel_path}: contains forbidden marker {marker!r}")


def _validate_package_boundary(
    manifest: dict[str, Any],
    blockers: list[str],
    checked_files: list[str],
) -> None:
    package_cfg = manifest.get("package") or {}
    include_paths = tuple(package_cfg.get("include_paths") or ())
    exclude_paths = tuple(package_cfg.get("exclude_paths") or ())
    omit_paths = tuple(package_cfg.get("omit_paths") or ())
    forbidden_markers = tuple(str(marker) for marker in package_cfg.get("forbidden_markers") or ())

    if not include_paths:
        blockers.append("package.include_paths must not be empty")
        return
    if not exclude_paths:
        blockers.append("package.exclude_paths must not be empty")
        return

    include_set = {_normalize_manifest_path(path) for path in include_paths}
    exclude_set = {_normalize_manifest_path(path) for path in exclude_paths}
    omit_set = {_normalize_manifest_path(path) for path in omit_paths}
    required_includes = {_normalize_manifest_path(path) for path in REQUIRED_PACKAGE_INCLUDE_PATHS}
    required_excludes = {_normalize_manifest_path(path) for path in REQUIRED_PACKAGE_EXCLUDE_PATHS}
    required_omits = {_normalize_manifest_path(path) for path in REQUIRED_PACKAGE_OMIT_PATHS}

    missing_includes = sorted(required_includes - include_set)
    if missing_includes:
        blockers.append(
            "package.include_paths missing required Lite runtime paths: "
            + ", ".join(missing_includes)
        )

    missing_excludes = sorted(required_excludes - exclude_set)
    if missing_excludes:
        blockers.append(
            "package.exclude_paths missing required Lite exclusions: "
            + ", ".join(missing_excludes)
        )

    missing_omits = sorted(required_omits - omit_set)
    if missing_omits:
        blockers.append(
            "package.omit_paths missing required Lite omissions: "
            + ", ".join(missing_omits)
        )

    marker_set = {marker.lower() for marker in forbidden_markers}
    missing_markers = sorted(marker for marker in REQUIRED_PACKAGE_FORBIDDEN_MARKERS if marker.lower() not in marker_set)
    if missing_markers:
        blockers.append(
            "package.forbidden_markers missing required ROS/native build markers: "
            + ", ".join(missing_markers)
        )

    overlap = sorted(include_set & exclude_set)
    if overlap:
        blockers.append(
            "package paths cannot be both included and excluded: " + ", ".join(overlap)
        )

    for raw_path in include_paths:
        normalized = _normalize_manifest_path(raw_path)
        if not normalized:
            blockers.append("package.include_paths contains an empty path")
            continue
        path = ROOT_DIR / normalized
        checked_files.append(normalized)
        if not path.exists():
            blockers.append(f"package.include_paths entry is missing: {normalized}")
            continue
        nested_excludes = sorted(
            exclude
            for exclude in exclude_set
            if normalized == exclude or normalized.startswith(f"{exclude}/")
        )
        if nested_excludes:
            blockers.append(
                f"package.include_paths entry {normalized} is under excluded path(s): "
                + ", ".join(nested_excludes)
            )


def _validate_python_dependencies(manifest: dict[str, Any], blockers: list[str], checked_files: list[str]) -> None:
    python_cfg = manifest.get("python") or {}
    base_dependencies, optional_dependencies = load_pyproject_dependencies(PYPROJECT_PATH)
    checked_files.append("pyproject.toml")

    base_names = {_dependency_name(dep) for dep in base_dependencies}
    forbidden_base = {_dependency_name(dep) for dep in python_cfg.get("forbidden_base_dependencies") or ()}
    leaked_base = sorted(base_names & forbidden_base)
    if leaked_base:
        blockers.append(f"pyproject.toml: core dependencies include Lite-forbidden packages: {', '.join(leaked_base)}")

    lite_extra = str(python_cfg.get("lite_extra") or "lite")
    if lite_extra not in optional_dependencies:
        blockers.append(f"pyproject.toml: missing optional dependency extra {lite_extra!r}")
        lite_dependencies: list[str] = []
    else:
        lite_dependencies = optional_dependencies[lite_extra]

    lite_names = {_dependency_name(dep) for dep in lite_dependencies}
    forbidden_lite = {_dependency_name(dep) for dep in python_cfg.get("forbidden_lite_dependencies") or ()}
    leaked_lite = sorted(lite_names & forbidden_lite)
    if leaked_lite:
        blockers.append(
            f"pyproject.toml: {lite_extra!r} extra includes "
            f"Lite-forbidden packages: {', '.join(leaked_lite)}"
        )

    gateway_extra = str(python_cfg.get("gateway_extra") or "gateway")
    gateway_dependencies = {_dependency_name(dep) for dep in python_cfg.get("gateway_dependencies") or ()}
    actual_gateway = {_dependency_name(dep) for dep in optional_dependencies.get(gateway_extra, [])}
    missing_gateway = sorted(gateway_dependencies - actual_gateway)
    if gateway_extra not in optional_dependencies:
        blockers.append(f"pyproject.toml: missing optional dependency extra {gateway_extra!r}")
    elif missing_gateway:
        blockers.append(f"pyproject.toml: {gateway_extra!r} extra missing dependencies: {', '.join(missing_gateway)}")

    lite_requirements = python_cfg.get("lite_requirements")
    if not lite_requirements:
        blockers.append("python.lite_requirements must be set")
        return
    requirements_path = ROOT_DIR / str(lite_requirements)
    checked_files.append(str(lite_requirements))
    if not requirements_path.is_file():
        blockers.append(f"{lite_requirements}: Lite requirements file is missing")
        return

    requirement_names = {_dependency_name(dep) for dep in _read_requirements(requirements_path)}
    required_lite = {_dependency_name(dep) for dep in python_cfg.get("required_lite_dependencies") or ()}
    missing_lite = sorted(required_lite - requirement_names)
    leaked_requirements = sorted(requirement_names & forbidden_lite)
    if missing_lite:
        blockers.append(f"{lite_requirements}: missing required Lite dependencies: {', '.join(missing_lite)}")
    if leaked_requirements:
        blockers.append(f"{lite_requirements}: includes Lite-forbidden packages: {', '.join(leaked_requirements)}")


def _validate_runtime_contract(
    manifest: dict[str, Any],
    blockers: list[str],
    checked_files: list[str],
) -> None:
    from core.blueprints.profile_builder import blueprint_for_resolved_profile
    from core.blueprints.runtime_endpoint import resolve_runtime_run_spec
    from core.runtime.resolver import canonical_profile_name, resolve_profile_config
    from lingtu_runtime.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()

    profile = str(manifest.get("profile") or "thunder-lite")
    canonical_profile = canonical_profile_name(profile)
    config = resolve_profile_config(profile)
    spec = resolve_runtime_run_spec(canonical_profile, config)

    runtime_cfg = manifest.get("runtime") or {}
    for field, expected in (runtime_cfg.get("expected_spec") or {}).items():
        actual = getattr(spec, str(field), None)
        if actual != expected:
            blockers.append(f"runtime-spec {profile}: {field} expected {expected!r}, got {actual!r}")
    for field, expected in (runtime_cfg.get("expected_config") or {}).items():
        actual = config.get(str(field))
        if actual != expected:
            blockers.append(f"profile config {profile}: {field} expected {expected!r}, got {actual!r}")

    _validate_runtime_env_defaults(manifest, spec, blockers, checked_files)
    _validate_product_graph(
        profile,
        blueprint_for_resolved_profile(canonical_profile, config),
        blockers,
    )


def _validate_product_graph(profile: str, bp: Any, blockers: list[str]) -> None:
    for entry in bp._entries:
        module_name = getattr(entry.module_cls, "__module__", "")
        for prefix, label in FORBIDDEN_PRODUCT_MODULE_PREFIXES.items():
            if module_name.startswith(prefix):
                class_name = getattr(entry.module_cls, "__name__", str(entry.module_cls))
                blockers.append(
                    f"{profile} graph contains {label} module "
                    f"{entry.name} ({module_name}.{class_name})"
                )


def _validate_runtime_env_defaults(
    manifest: dict[str, Any],
    spec: Any,
    blockers: list[str],
    checked_files: list[str],
) -> None:
    rel_path = "scripts/deploy/thunder/runtime-env.sh"
    if rel_path not in checked_files:
        checked_files.append(rel_path)
    if not RUNTIME_ENV_PATH.is_file():
        blockers.append(f"{rel_path}: runtime environment file is missing")
        return

    defaults = _parse_shell_default_env(
        RUNTIME_ENV_PATH.read_text(encoding="utf-8-sig", errors="ignore")
    )
    expected_defaults = {
        "LINGTU_PROFILE": str(manifest.get("profile") or "thunder-lite"),
        "LINGTU_MODULE_TRANSPORT": spec.module_transport,
        "LINGTU_ENDPOINT": spec.endpoint,
        "LINGTU_ENDPOINT_TRANSPORT": spec.endpoint_transport,
        "LINGTU_ENDPOINT_CONTRACT": spec.endpoint_contract or "",
        "LINGTU_SIMULATION_ONLY": spec.env.get("LINGTU_SIMULATION_ONLY"),
        "LINGTU_COMMAND_OUTPUT_MODE": "local_driver",
        "LINGTU_HARDWARE_CONTROL_BOUNDARY": "module_graph_driver",
    }
    for key, expected in expected_defaults.items():
        actual = defaults.get(key)
        if actual != expected:
            blockers.append(f"{rel_path}: {key} default expected {expected!r}, got {actual!r}")


def validate(manifest_path: Path = MANIFEST_PATH) -> dict[str, Any]:
    """Validate the Thunder Lite manifest and return a structured result."""

    blockers: list[str] = []
    checked_files: list[str] = []
    if not manifest_path.is_file():
        return {
            "ok": False,
            "blockers": [f"{manifest_path.relative_to(ROOT_DIR)} is missing"],
            "checked_files": [],
        }

    checked_files.append(_normalize_manifest_path(manifest_path.relative_to(ROOT_DIR)))
    manifest = _load_manifest(manifest_path)
    if manifest.get("schema_version") != "lingtu.thunder_lite_package.v1":
        blockers.append("schema_version must be lingtu.thunder_lite_package.v1")
    if manifest.get("profile") != "thunder-lite":
        blockers.append("profile must be thunder-lite")

    _validate_deploy_files(manifest, blockers, checked_files)
    _validate_python_dependencies(manifest, blockers, checked_files)
    _validate_package_boundary(manifest, blockers, checked_files)
    _validate_runtime_contract(manifest, blockers, checked_files)

    return {
        "ok": not blockers,
        "profile": manifest.get("profile"),
        "blockers": blockers,
        "checked_files": sorted(dict.fromkeys(checked_files)),
    }


def main(argv: list[str] | None = None) -> int:
    """Run the Thunder Lite package validator CLI."""

    parser = argparse.ArgumentParser(description="Validate Thunder Lite packaging boundaries")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    parser.add_argument(
        "--manifest",
        default=str(MANIFEST_PATH),
        help="Path to config/thunder_lite_package.yaml",
    )
    args = parser.parse_args(argv)

    result = validate(Path(args.manifest))
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    elif result["ok"]:
        checked = ", ".join(result["checked_files"])
        print(f"Thunder Lite package check: OK ({checked})")
    else:
        print("Thunder Lite package check: FAIL")
        for blocker in result["blockers"]:
            print(f"  ERROR: {blocker}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
