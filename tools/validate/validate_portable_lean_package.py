#!/usr/bin/env python3
"""Validate LingTu's portable lean dependency baseline."""

from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import re
import sys
import tomllib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
PYPROJECT_PATH = ROOT_DIR / "pyproject.toml"
REQUIREMENTS_CORE_PORTABLE_PATH = ROOT_DIR / "requirements-core-portable.txt"
REQUIREMENTS_LITE_PATH = ROOT_DIR / "requirements-lite.txt"
REQUIREMENTS_SIM_MUJOCO_PATH = ROOT_DIR / "requirements-sim-mujoco.txt"

CORE_PORTABLE_DEPS = frozenset({"numpy", "scipy", "pyyaml", "pydantic"})
SIM_MUJOCO_DEPS = frozenset({"mujoco", "mujoco-lidar"})

ROS_COMPAT_ONLY = frozenset(
    {
        "rclpy",
        "sensor-msgs",
        "nav-msgs",
        "geometry-msgs",
        "std-msgs",
        "tf2-ros",
        "launch-ros",
        "rosbag2-py",
        "rosbag",
        "rosidl-runtime-py",
    }
)
HEAVY_FEATURE_PACK = frozenset(
    {
        "fastapi",
        "uvicorn",
        "websockets",
        "starlette",
        "opencv-python-headless",
        "cv2",
        "pillow",
        "pil",
        "scikit-learn",
        "sklearn",
        "openai",
        "anthropic",
        "jieba",
        "langchain",
        "langchain-core",
        "langchain-openai",
        "torch",
        "ultralytics",
        "open-clip-torch",
        "open-clip",
        "chromadb",
        "rerun",
        "open3d",
        "gtsam",
        "pcl",
    }
)
ROBOT_OPTIONAL = frozenset({"brainstem-api", "grpc", "grpcio", "protobuf"})
FORBIDDEN_BASE_DEPS = ROS_COMPAT_ONLY | HEAVY_FEATURE_PACK | ROBOT_OPTIONAL | SIM_MUJOCO_DEPS
FORBIDDEN_TOP_LEVEL_IMPORTS = ROS_COMPAT_ONLY | HEAVY_FEATURE_PACK | ROBOT_OPTIONAL
EMPTY_PORTABLE_OPTIONAL_GROUPS = frozenset(
    {
        "lite",
        "core-portable",
        "portable",
        "endpoint-portable",
        "pcl-ops",
        "ros-compat",
        "slam-native",
    }
)
REQUIRED_OPTIONAL_GROUPS = EMPTY_PORTABLE_OPTIONAL_GROUPS | frozenset(
    {
        "sim-mujoco",
        "thunder",
        "robot-thunder",
        "gateway",
        "vision",
        "ml",
        "llm",
        "nlp",
        "agent",
        "perception",
        "vector",
        "dev",
    }
)
OPTIONAL_GROUP_ALLOWED_DEPS = {
    "sim-mujoco": SIM_MUJOCO_DEPS,
    "thunder": frozenset({"brainstem-api"}),
    "robot-thunder": frozenset({"brainstem-api"}),
}

CRITICAL_PORTABLE_SURFACES = (
    "lingtu.py",
    "cli",
    "src/runtime",
    "src/nav",
    "src/drivers/sim",
    "sim/validation",
)
CRITICAL_SCAN_EXCLUDES = (
    "src/runtime/tests/",
    "src/nav/tests/",
    "src/nav/tests/local/",
    "sim/tests/",
    "__pycache__/",
    "/build/",
    "/build_nb_win/",
    "/build_nb/",
    "/_deps/",
    # Legacy/manual-experiment PCT backend vendor code. Tracked as a known
    # native-heavy (Open3D) surface in docs/architecture/PORTABLE_LEAN_PACKAGE_MATRIX.md
    # ("Isolate PCT/GTSAM/Open3D"); not on the default portable runtime path.
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/",
)
BOTTOM_LAYER_SURFACES = (
    "src/runtime/portable",
    "src/runtime/msgs",
    "src/runtime/runtime_interface.py",
)
FORBIDDEN_BOTTOM_LAYER_IMPORTS = FORBIDDEN_TOP_LEVEL_IMPORTS | SIM_MUJOCO_DEPS | frozenset(
    {"pcl", "pcl-ros", "pcl-conversions", "open3d", "gtsam", "rerun"}
)
FORBIDDEN_PROFILE_MODULES = (
    "ROS2",
    "GatewayModule",
    "MCPServerModule",
    "Semantic",
    "Perception",
    "VectorMemory",
)


@dataclass
class Check:
    name: str
    status: str
    summary: str
    evidence: dict[str, Any]


def _dep_name(requirement: str) -> str:
    value = requirement.strip().lower()
    value = value.split(";", 1)[0].strip()
    value = value.split(" @ ", 1)[0].strip()
    value = re.split(r"[\[<>=!~]", value, maxsplit=1)[0].strip()
    return value.replace("_", "-")


def _module_dep_name(module: str) -> str:
    return module.strip().lower().replace("_", "-")


def _read_pyproject() -> tuple[list[str], dict[str, list[str]]]:
    data = tomllib.loads(PYPROJECT_PATH.read_text(encoding="utf-8-sig"))
    project = data.get("project") or {}
    return (
        list(project.get("dependencies") or []),
        {
            str(key): list(value or [])
            for key, value in (project.get("optional-dependencies") or {}).items()
        },
    )


def _read_requirements(path: Path, *, _seen: set[Path] | None = None) -> list[str]:
    _seen = set() if _seen is None else _seen
    path = path.resolve()
    if path in _seen:
        return []
    _seen.add(path)
    requirements: list[str] = []
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line.startswith("-r ") or line.startswith("--requirement "):
            include_path = line.split(maxsplit=1)[1].strip()
            requirements.extend(_read_requirements((path.parent / include_path).resolve(), _seen=_seen))
            continue
        if line:
            requirements.append(line)
    return requirements


def _top_level_imports(path: Path) -> set[str]:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    except SyntaxError:
        return set()
    imports: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(_module_dep_name(alias.name.split(".", 1)[0]) for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(_module_dep_name(node.module.split(".", 1)[0]))
    return imports


def _iter_surface_files(surface: str) -> Iterable[Path]:
    path = ROOT_DIR / surface
    if path.is_file():
        yield path
        return
    if not path.exists():
        return
    yield from path.rglob("*.py")


def _import_blockers_for_surfaces(
    surfaces: Iterable[str],
    forbidden_imports: set[str] | frozenset[str],
) -> dict[str, list[str]]:
    blockers: dict[str, list[str]] = {}
    for surface in surfaces:
        for path in _iter_surface_files(surface):
            rel = path.relative_to(ROOT_DIR).as_posix()
            if any(marker in rel for marker in CRITICAL_SCAN_EXCLUDES):
                continue
            forbidden = sorted(_top_level_imports(path) & forbidden_imports)
            if forbidden:
                blockers[rel] = forbidden
    return blockers


def _portable_surface_import_blockers() -> dict[str, list[str]]:
    return _import_blockers_for_surfaces(CRITICAL_PORTABLE_SURFACES, FORBIDDEN_TOP_LEVEL_IMPORTS)


def _check_base_dependencies() -> Check:
    base_deps, optional_deps = _read_pyproject()
    base_names = {_dep_name(dep) for dep in base_deps}
    forbidden = sorted(base_names & FORBIDDEN_BASE_DEPS)
    unexpected = sorted(base_names - CORE_PORTABLE_DEPS)
    missing = sorted(CORE_PORTABLE_DEPS - base_names)
    status = "pass" if not forbidden and not unexpected and not missing else "fail"
    return Check(
        name="base_dependencies",
        status=status,
        summary=(
            "pyproject base dependencies are limited to the portable core set"
            if status == "pass"
            else "pyproject base dependencies are not portable-lean clean"
        ),
        evidence={
            "base_dependencies": sorted(base_names),
            "core_portable_allowed": sorted(CORE_PORTABLE_DEPS),
            "forbidden_in_base": forbidden,
            "unexpected_in_base": unexpected,
            "missing_core_dependencies": missing,
            "optional_groups": {key: sorted(_dep_name(dep) for dep in value) for key, value in optional_deps.items()},
        },
    )


def _check_lite_requirements() -> Check:
    lite_names = {_dep_name(dep) for dep in _read_requirements(REQUIREMENTS_LITE_PATH)}
    forbidden = sorted(lite_names & FORBIDDEN_BASE_DEPS)
    unexpected = sorted(lite_names - CORE_PORTABLE_DEPS)
    missing = sorted(CORE_PORTABLE_DEPS - lite_names)
    status = "pass" if not forbidden and not unexpected and not missing else "fail"
    return Check(
        name="requirements_lite",
        status=status,
        summary=(
            "requirements-lite.txt mirrors the portable core dependency set"
            if status == "pass"
            else "requirements-lite.txt is not portable-lean clean"
        ),
        evidence={
            "requirements_lite": sorted(lite_names),
            "forbidden_in_lite": forbidden,
            "unexpected_in_lite": unexpected,
            "missing_core_dependencies": missing,
        },
    )


def _check_core_portable_requirements() -> Check:
    core_names = {_dep_name(dep) for dep in _read_requirements(REQUIREMENTS_CORE_PORTABLE_PATH)}
    forbidden = sorted(core_names & FORBIDDEN_BASE_DEPS)
    unexpected = sorted(core_names - CORE_PORTABLE_DEPS)
    missing = sorted(CORE_PORTABLE_DEPS - core_names)
    status = "pass" if not forbidden and not unexpected and not missing else "fail"
    return Check(
        name="requirements_core_portable",
        status=status,
        summary=(
            "requirements-core-portable.txt is limited to the portable core dependency set"
            if status == "pass"
            else "requirements-core-portable.txt is not portable-lean clean"
        ),
        evidence={
            "requirements_core_portable": sorted(core_names),
            "forbidden_in_core_portable": forbidden,
            "unexpected_in_core_portable": unexpected,
            "missing_core_dependencies": missing,
        },
    )


def _check_sim_mujoco_requirements() -> Check:
    sim_names = {_dep_name(dep) for dep in _read_requirements(REQUIREMENTS_SIM_MUJOCO_PATH)}
    expected = CORE_PORTABLE_DEPS | SIM_MUJOCO_DEPS
    forbidden = sorted((sim_names & FORBIDDEN_BASE_DEPS) - SIM_MUJOCO_DEPS)
    unexpected = sorted(sim_names - expected)
    missing = sorted(expected - sim_names)
    status = "pass" if not forbidden and not unexpected and not missing else "fail"
    return Check(
        name="requirements_sim_mujoco",
        status=status,
        summary=(
            "requirements-sim-mujoco.txt layers only MuJoCo on top of portable core"
            if status == "pass"
            else "requirements-sim-mujoco.txt includes dependencies outside portable core + MuJoCo"
        ),
        evidence={
            "requirements_sim_mujoco": sorted(sim_names),
            "expected": sorted(expected),
            "forbidden_in_sim_mujoco": forbidden,
            "unexpected_in_sim_mujoco": unexpected,
            "missing_dependencies": missing,
        },
    )


def _check_optional_dependency_tiers() -> Check:
    _, optional_deps = _read_pyproject()
    normalized = {group: {_dep_name(dep) for dep in deps} for group, deps in optional_deps.items()}
    missing_groups = sorted(REQUIRED_OPTIONAL_GROUPS - normalized.keys())
    non_empty_marker_groups = {
        group: sorted(normalized.get(group, set()))
        for group in sorted(EMPTY_PORTABLE_OPTIONAL_GROUPS)
        if normalized.get(group, set())
    }
    wrong_group_deps: dict[str, dict[str, list[str]]] = {}
    for group, allowed in OPTIONAL_GROUP_ALLOWED_DEPS.items():
        actual = normalized.get(group, set())
        unexpected = sorted(actual - allowed)
        missing = sorted(allowed - actual)
        if unexpected or missing:
            wrong_group_deps[group] = {"unexpected": unexpected, "missing": missing}
    status = "pass" if not missing_groups and not non_empty_marker_groups and not wrong_group_deps else "fail"
    return Check(
        name="optional_dependency_tiers",
        status=status,
        summary=(
            "pyproject optional dependency groups make portable/ROS/native tiers explicit"
            if status == "pass"
            else "pyproject optional dependency groups are missing or blur package tiers"
        ),
        evidence={
            "required_groups": sorted(REQUIRED_OPTIONAL_GROUPS),
            "missing_groups": missing_groups,
            "empty_marker_groups_with_dependencies": non_empty_marker_groups,
            "wrong_group_deps": wrong_group_deps,
            "groups": {group: sorted(deps) for group, deps in sorted(normalized.items())},
        },
    )


def _check_profile(profile: str) -> Check:
    if str(SRC_DIR) not in sys.path:
        sys.path.insert(0, str(SRC_DIR))
    if str(ROOT_DIR) not in sys.path:
        sys.path.insert(0, str(ROOT_DIR))

    from runtime.introspection.profile_graph import graph_for_profile
    from runtime.profiles.resolver import resolve_profile_config
    from runtime.runtime_interface import DATA_SOURCE_CONTRACTS, profile_data_source

    config = resolve_profile_config(profile)
    graph = graph_for_profile(profile)
    source = DATA_SOURCE_CONTRACTS[profile_data_source(profile).data_source]
    module_names = sorted(graph.modules)
    forbidden_modules = [
        name
        for name in module_names
        if any(marker in name for marker in FORBIDDEN_PROFILE_MODULES)
    ]
    failures: list[str] = []
    if profile == "portable_mujoco":
        expected = {
            "robot": "sim_mujoco",
            "slam_profile": "none",
            "drive_mode": "kinematic",
            "enable_gateway": False,
            "enable_semantic": False,
            "enable_native": False,
            "enable_camera": True,
            "use_driver_camera": True,
        }
        for key, expected_value in expected.items():
            if config.get(key) != expected_value:
                failures.append(f"{key}={config.get(key)!r}, expected {expected_value!r}")
        if "MujocoDriverModule" not in graph.modules:
            failures.append("MujocoDriverModule missing")
    dangling = [str(wire) for wire in graph.dangling_wires()]
    if forbidden_modules:
        failures.append("forbidden modules present")
    if dangling:
        failures.append("dangling wires present")
    status = "pass" if not failures else "fail"
    return Check(
        name="profile_contract",
        status=status,
        summary=(
            f"{profile} resolves to a portable lean runtime graph"
            if status == "pass"
            else f"{profile} violates portable lean profile contract"
        ),
        evidence={
            "profile": profile,
            "config_subset": {
                key: config.get(key)
                for key in (
                    "robot",
                    "world",
                    "drive_mode",
                    "slam_profile",
                    "planner",
                    "enable_gateway",
                    "enable_semantic",
                    "enable_native",
                    "enable_camera",
                    "use_driver_camera",
                )
            },
            "data_source": source.name,
            "data_source_provider": source.provider,
            "modules": module_names,
            "forbidden_modules": forbidden_modules,
            "dangling_wires": dangling,
            "failures": failures,
        },
    )


def _check_surface_imports() -> Check:
    blockers = _portable_surface_import_blockers()
    status = "pass" if not blockers else "fail"
    return Check(
        name="critical_surface_top_level_imports",
        status=status,
        summary=(
            "critical portable runtime surfaces avoid forbidden heavy top-level imports"
            if status == "pass"
            else "critical portable runtime surfaces import forbidden heavy dependencies at module import time"
        ),
        evidence={
            "scanned_surfaces": list(CRITICAL_PORTABLE_SURFACES),
            "forbidden_imports": blockers,
            "forbidden_top_level_modules": sorted(FORBIDDEN_TOP_LEVEL_IMPORTS),
        },
    )


def _check_bottom_layer_imports() -> Check:
    blockers = _import_blockers_for_surfaces(BOTTOM_LAYER_SURFACES, FORBIDDEN_BOTTOM_LAYER_IMPORTS)
    status = "pass" if not blockers else "fail"
    return Check(
        name="bottom_layer_imports",
        status=status,
        summary=(
            "bottom-layer contracts/messages avoid adapter and heavy native imports"
            if status == "pass"
            else "bottom-layer contracts/messages import adapter or heavy native dependencies"
        ),
        evidence={
            "scanned_surfaces": list(BOTTOM_LAYER_SURFACES),
            "forbidden_imports": blockers,
            "forbidden_bottom_layer_modules": sorted(FORBIDDEN_BOTTOM_LAYER_IMPORTS),
        },
    )


def _check_optional_pcl_ops_boundary() -> Check:
    available = importlib.util.find_spec("lingtu_pcl_ops") is not None
    return Check(
        name="optional_pcl_ops_boundary",
        status="pass",
        summary=(
            "optional lingtu_pcl_ops plugin is installed and can accelerate point-cloud ops"
            if available
            else "optional lingtu_pcl_ops plugin is absent; portable runtime must use fallback point-cloud ops"
        ),
        evidence={
            "optional_module": "lingtu_pcl_ops",
            "available": available,
            "install_target": "pip install lingtu-pcl-ops (once prebuilt wheels are published)",
            "fallback_module": "nav.local.pcl_ops",
        },
    )


def _check_sim_dependency(profile: str) -> Check:
    requires_mujoco = profile == "portable_mujoco"
    available = importlib.util.find_spec("mujoco") is not None
    status = "pass" if (available or not requires_mujoco) else "fail"
    return Check(
        name="simulation_dependency_boundary",
        status=status,
        summary=(
            "MuJoCo is available as an explicit simulation dependency"
            if requires_mujoco and available
            else "MuJoCo is not required for this profile"
            if not requires_mujoco
            else "portable_mujoco requires the optional mujoco package"
        ),
        evidence={
            "profile": profile,
            "requires_mujoco": requires_mujoco,
            "mujoco_available": available,
            "boundary": "simulation_optional_not_core_base",
        },
    )


def run(profile: str) -> dict[str, Any]:
    checks = [
        _check_base_dependencies(),
        _check_optional_dependency_tiers(),
        _check_core_portable_requirements(),
        _check_lite_requirements(),
        _check_sim_mujoco_requirements(),
        _check_profile(profile),
        _check_surface_imports(),
        _check_bottom_layer_imports(),
        _check_optional_pcl_ops_boundary(),
        _check_sim_dependency(profile),
    ]
    summary = {
        "pass": sum(check.status == "pass" for check in checks),
        "fail": sum(check.status == "fail" for check in checks),
        "warn": sum(check.status == "warn" for check in checks),
    }
    return {
        "schema_version": "lingtu.portable_lean_validation.v1",
        "profile": profile,
        "passed": summary["fail"] == 0,
        "summary": summary,
        "checks": [asdict(check) for check in checks],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="portable_mujoco", help="Profile to validate; default: portable_mujoco")
    parser.add_argument("--json", action="store_true", help="Print full JSON report")
    parser.add_argument("--dry-run", action="store_true", help="Report failures but exit 0")
    args = parser.parse_args()

    report = run(args.profile)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        state = "PASS" if report["passed"] else "FAIL"
        print(f"Portable lean package validation: {state} {report['summary']}")
        for check in report["checks"]:
            print(f"- {check['name']}: {check['status']} — {check['summary']}")
            if check["status"] != "pass":
                print(json.dumps(check["evidence"], indent=2, sort_keys=True))
    return 0 if report["passed"] or args.dry_run else 1


if __name__ == "__main__":
    raise SystemExit(main())
