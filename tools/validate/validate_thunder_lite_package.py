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
    "runtime.adapters.ros2": "ROS compatibility",
    "localization.": "SLAM package",
}
REQUIRED_PACKAGE_INCLUDE_PATHS = (
    "lingtu.py",
    "requirements-lite.txt",
    "config/thunder_lite_package.yaml",
    "config/robots/thunder.yaml",
    "scripts/deploy/thunder/runtime-env.sh",
    "scripts/deploy/thunder/lingtu-thunder-lite.service",
    "scripts/deploy/thunder/install_lite_service.sh",
    "src/runtime/",
    "src/lingtu/",
    "src/drivers/real/thunder/",
    "src/nav/__init__.py",
    "src/nav/navigation.py",
    "src/nav/runtime/",
    "src/nav/model/",
    "src/nav/tracking/",
    "src/nav/services/plan/__init__.py",
    "src/nav/services/plan/contracts.py",
    "src/nav/services/plan/factory.py",
    "src/nav/services/plan/preview.py",
    "src/nav/services/plan/compat/__init__.py",
    "src/nav/services/plan/compat/direct.py",
    "src/nav/services/plan/compat/direct_path.py",
    "src/nav/services/plan/global_planner/__init__.py",
    "src/nav/services/plan/global_planner/algorithm/__init__.py",
    "src/nav/services/safety/",
    "src/nav/services/safety/safety_ring.py",
    "src/nav/services/safety/velocity_mux.py",
    "src/nav/services/__init__.py",
    "src/nav/services/frame_transforms.py",
    "src/nav/services/goals.py",
    "src/nav/services/geofence.py",
    "src/nav/local/",
    "src/nav/kernel/__init__.py",
    "src/nav/kernel/loader.py",
    "src/nav/kernel/paths.py",
)
REQUIRED_PACKAGE_EXCLUDE_PATHS = (
    "src/*/adapters/ros2/",
    "src/localization/",
    "src/perception/",
    "src/decision/",
    "src/memory/",
    "src/gateway/",
    "src/gateway/media/",
    "src/drivers/sim/",
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
    "cli/repl.py",
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/",
    "src/nav/services/plan/global_planner/service.py",
    "src/nav/services/plan/global_planner/algorithm/pct/",
    "src/nav/services/plan/global_planner/algorithm/pct/runtime/",
    "src/nav/cpp/planning/global/octoplanner/",
    "src/nav/services/plan/global_planner/artifacts.py",
    "src/nav/services/plan/global_planner/path_feasibility.py",
    "cli/runtime_audit.py",
    "cli/runtime_extra.py",
    "src/lingtu/assembly/full_stack.py",
    "src/lingtu/assembly/full_stack_wiring.py",
    "src/lingtu/assembly/multi_robot.py",
    "src/lingtu/assembly/profile_graph.py",
    "src/lingtu/assembly/simulation_contract.py",
    "src/lingtu/assembly/graph.py",
    "src/lingtu/assembly/stacks/composition.py",
    "src/lingtu/assembly/stacks/exploration.py",
    "src/lingtu/assembly/stacks/gateway.py",
    "src/lingtu/assembly/stacks/lidar.py",
    "src/localization/adapters/resolver.py",
    "src/runtime/adapters/navigation_io.py",
    "src/lingtu/assembly/stacks/maps.py",
    "src/lingtu/assembly/stacks/memory.py",
    "src/lingtu/assembly/adapters/perception_gateway.py",
    "src/lingtu/assembly/stacks/perception.py",
    "src/lingtu/assembly/stacks/planner.py",
    "src/lingtu/assembly/stacks/sim_lidar.py",
    "src/lingtu/assembly/stacks/slam.py",
    "src/lingtu/assembly/stacks/system.py",
    "src/lingtu/assembly/wires/",
    "src/runtime/adapters/dds/reader.py",
    "src/runtime/devices/",
    "sim/diagnostics/gap_report.py",
    "sim/diagnostics/dataflow_report.py",
    "src/runtime/external_service_module.py",
    "src/diagnostics/field/gateway_acceptance.py",
    "src/diagnostics/field/inspection.py",
    "src/diagnostics/field/field_check.py",
    "src/runtime/rerun_module.py",
    "src/diagnostics/field/evidence.py",
    "src/diagnostics/field/gates.py",
    "src/runtime/transport/dds.py",
    "src/runtime/transport/shm.py",
    "src/localization/adapters/relocalization.py",
    "src/drivers/real/thunder/blueprints.py",
    "src/drivers/real/thunder/connection.py",
    "src/lingtu/sdk/",
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
LITE_FORBIDDEN_LOCAL_PLANNER_BACKENDS = frozenset({"nanobind", "cmu", "cmu_py"})
LITE_FORBIDDEN_PATH_FOLLOWER_BACKENDS = frozenset({"nav_kernel"})
LITE_EXPECTED_PYTHON_AUTONOMY_BACKEND = "simple"
LITE_EXPECTED_PYTHON_PATH_FOLLOWER_BACKEND = "pid"

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
            {str(key): list(value or []) for key, value in (project.get("optional-dependencies") or {}).items()},
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


def _src_path_to_module_prefix(value: Any) -> str | None:
    rel = _normalize_manifest_path(value)
    if "*" in rel or not rel.startswith("src/"):
        return None
    module_path = rel.removeprefix("src/")
    if not module_path:
        return None
    if module_path.endswith(".py"):
        return module_path[:-3].replace("/", ".")
    return module_path.replace("/", ".").rstrip(".") + "."


def _module_matches_prefix(module_name: str, prefix: str) -> bool:
    normalized = prefix.rstrip(".")
    return module_name == normalized or module_name.startswith(f"{normalized}.")


def _lite_excluded_module_prefixes(manifest: dict[str, Any]) -> dict[str, str]:
    package_cfg = manifest.get("package") or {}
    prefixes: dict[str, str] = {}
    for rel_path in package_cfg.get("exclude_paths") or ():
        prefix = _src_path_to_module_prefix(rel_path)
        if prefix:
            prefixes[prefix] = f"Lite-excluded package {rel_path}"
    return prefixes


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
        blockers.append("package.include_paths missing required Lite runtime paths: " + ", ".join(missing_includes))

    missing_excludes = sorted(required_excludes - exclude_set)
    if missing_excludes:
        blockers.append("package.exclude_paths missing required Lite exclusions: " + ", ".join(missing_excludes))

    missing_omits = sorted(required_omits - omit_set)
    if missing_omits:
        blockers.append("package.omit_paths missing required Lite omissions: " + ", ".join(missing_omits))

    marker_set = {marker.lower() for marker in forbidden_markers}
    missing_markers = sorted(
        marker for marker in REQUIRED_PACKAGE_FORBIDDEN_MARKERS if marker.lower() not in marker_set
    )
    if missing_markers:
        blockers.append(
            "package.forbidden_markers missing required ROS/native build markers: " + ", ".join(missing_markers)
        )

    overlap = sorted(include_set & exclude_set)
    if overlap:
        blockers.append("package paths cannot be both included and excluded: " + ", ".join(overlap))

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
            exclude for exclude in exclude_set if normalized == exclude or normalized.startswith(f"{exclude}/")
        )
        if nested_excludes:
            blockers.append(
                f"package.include_paths entry {normalized} is under excluded path(s): " + ", ".join(nested_excludes)
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
            f"pyproject.toml: {lite_extra!r} extra includes Lite-forbidden packages: {', '.join(leaked_lite)}"
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
    from lingtu.plugin_seed import install_builtin_plugin_catalog
    from lingtu.assembly.profile_builder import blueprint_for_resolved_profile
    from runtime.profiles.endpoints import resolve_runtime_run_spec
    from runtime.profiles.resolver import canonical_profile_name, resolve_profile_config

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

    _validate_lite_runtime_defaults(profile, config, spec, blockers)
    _validate_runtime_env_defaults(manifest, spec, blockers, checked_files)
    _validate_product_graph(
        profile,
        blueprint_for_resolved_profile(canonical_profile, config),
        blockers,
        manifest,
    )


def _validate_lite_runtime_defaults(
    profile: str,
    config: dict[str, Any],
    spec: Any,
    blockers: list[str],
) -> None:
    from lingtu.assembly.stacks.autonomy_chain import autonomy_stack_config

    if config.get("enable_native") is not False:
        blockers.append(f"profile config {profile}: enable_native must be false for Lite runtime")

    expected_config = {
        "runtime_mode": "lite",
        "slam_profile": "none",
        "enable_gateway": False,
        "enable_semantic": False,
        "enable_map_modules": False,
        "enable_gnss": False,
        "manage_external_services": False,
        "run_startup_checks": False,
        "planner": "direct",
        "python_autonomy_backend": LITE_EXPECTED_PYTHON_AUTONOMY_BACKEND,
        "python_path_follower_backend": LITE_EXPECTED_PYTHON_PATH_FOLLOWER_BACKEND,
    }
    for field, expected in expected_config.items():
        actual = config.get(field)
        if actual != expected:
            blockers.append(f"profile config {profile}: {field} expected {expected!r}, got {actual!r}")

    if config.get("local_planner_backend") in LITE_FORBIDDEN_LOCAL_PLANNER_BACKENDS:
        blockers.append(
            f"profile config {profile}: local_planner_backend must not use "
            f"native/heavy backend {config.get('local_planner_backend')!r}"
        )
    if config.get("path_follower_backend") in LITE_FORBIDDEN_PATH_FOLLOWER_BACKENDS:
        blockers.append(
            f"profile config {profile}: path_follower_backend must not use "
            f"native/heavy backend {config.get('path_follower_backend')!r}"
        )

    autonomy_config = dict(config)
    enable_native = bool(autonomy_config.pop("enable_native", False))
    effective = autonomy_stack_config(enable_native, **autonomy_config)
    if effective["backend"] != LITE_EXPECTED_PYTHON_AUTONOMY_BACKEND:
        blockers.append(
            f"profile config {profile}: effective local planner backend expected "
            f"{LITE_EXPECTED_PYTHON_AUTONOMY_BACKEND!r}, got {effective['backend']!r}"
        )
    if effective["path_follower_backend"] != LITE_EXPECTED_PYTHON_PATH_FOLLOWER_BACKEND:
        blockers.append(
            f"profile config {profile}: effective path follower backend expected "
            f"{LITE_EXPECTED_PYTHON_PATH_FOLLOWER_BACKEND!r}, got "
            f"{effective['path_follower_backend']!r}"
        )

    expected_spec = {
        "module_transport": "local",
        "endpoint_transport": "local",
        "endpoint_contract": None,
        "localization_adapter": None,
        "simulation_only": False,
    }
    for field, expected in expected_spec.items():
        actual = getattr(spec, field, None)
        if actual != expected:
            blockers.append(f"runtime-spec {profile}: {field} expected {expected!r}, got {actual!r}")


def _validate_product_graph(
    profile: str,
    bp: Any,
    blockers: list[str],
    manifest: dict[str, Any] | None = None,
) -> None:
    manifest = manifest or _load_manifest(MANIFEST_PATH)
    forbidden_prefixes = {
        **_lite_excluded_module_prefixes(manifest),
        **FORBIDDEN_PRODUCT_MODULE_PREFIXES,
    }
    for entry in bp._entries:
        module_name = getattr(entry.module_cls, "__module__", "")
        for prefix, label in forbidden_prefixes.items():
            if _module_matches_prefix(module_name, prefix):
                class_name = getattr(entry.module_cls, "__name__", str(entry.module_cls))
                blockers.append(f"{profile} graph contains {label} module {entry.name} ({module_name}.{class_name})")


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

    defaults = _parse_shell_default_env(RUNTIME_ENV_PATH.read_text(encoding="utf-8-sig", errors="ignore"))
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
