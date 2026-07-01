#!/usr/bin/env python3
"""Validate LingTu package import boundaries.

This is a lightweight CI/manual guard for Module-First cleanliness. It scans
all production Python imports, including lazy imports inside functions, while
skipping tests/examples, explicit composition layers, and TYPE_CHECKING-only
imports.

Usage:
    python tools/validate/validate_architecture_boundaries.py
    python tools/validate/validate_architecture_boundaries.py --verbose
"""

from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from compat.ros2.manifest import (  # noqa: E402
    ROS_IMPORT_ROOTS,
    ROS_SCAN_EXCLUDED_PREFIXES,
    ros_compat_boundary_for,
)

BOUNDARY_RULES: dict[str, set[str]] = {
    "gateway": {"nav", "semantic", "drivers", "slam"},
    "nav": {"semantic", "drivers", "gateway"},
    "semantic": {"nav", "drivers", "gateway"},
    "drivers": {"nav", "semantic", "gateway"},
    "core": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "slam": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "memory": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "base_autonomy": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "exploration": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "webrtc": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "global_planning",
        "lingtu",
    },
    "global_planning": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "lingtu",
    },
    "lingtu": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
    },
}

COMPOSITION_EXCEPTIONS = {
    # Blueprint/profile composition layers intentionally know multiple packages.
    "core/blueprints/",
    "core/blueprints/stacks/",
    "core/blueprints/full_stack.py",
    "core/blueprints/full_stack_wiring.py",
    "core/blueprints/stub.py",
    "drivers/sim/stub.py",
    "drivers/sim/test_full_pipeline_s100p.py",
    # Core glue/report files that aggregate contracts across packages.
    "core/gateway_runtime_acceptance.py",
    "core/product_field_check.py",
    "core/msgs/scene.py",
    "core/same_source_map_artifacts.py",
    # Public facade intentionally wraps internal packages.
    "lingtu/",
    # Vendored/legacy launch surfaces are not production Module imports.
    "global_planning/pct_planner/launch/",
    "global_planning/pct_planner/planner/lib/3rdparty/",
}

HARDWARE_COMPAT_FORBIDDEN_IMPORT_ROOTS = {"compat.ros2"}
HARDWARE_COMPAT_FORBIDDEN_DIRS = (
    SRC_DIR / "drivers" / "real" / "thunder",
)
CORE_COMPAT_FORBIDDEN_IMPORT_ROOTS = {"compat.ros2"}
CORE_COMPAT_FORBIDDEN_DIRS = (
    SRC_DIR / "core",
)


def _top_level(module: str) -> str:
    return module.split(".", 1)[0]


def _is_test_or_example(path: Path) -> bool:
    parts = set(path.parts)
    return (
        "tests" in parts
        or "test" in parts
        or "examples" in parts
        or "example" in parts
        or path.name.startswith("test_")
    )


def _is_composition_exception(path: Path) -> bool:
    rel = path.relative_to(SRC_DIR).as_posix()
    return any(rel == exc or rel.startswith(exc) for exc in COMPOSITION_EXCEPTIONS)


def _is_ros_scan_excluded(path: Path) -> bool:
    rel = path.relative_to(SRC_DIR).as_posix()
    return any(rel == prefix or rel.startswith(prefix) for prefix in ROS_SCAN_EXCLUDED_PREFIXES)


def _ros_compat_boundary(path: Path) -> str | None:
    rel = path.relative_to(SRC_DIR).as_posix()
    boundary = ros_compat_boundary_for(rel)
    return boundary.category if boundary is not None else None


def _is_ros_import(module: str) -> bool:
    return _top_level(module) in ROS_IMPORT_ROOTS


def _is_forbidden_hardware_compat_import(module: str) -> bool:
    return any(
        module == root or module.startswith(f"{root}.")
        for root in HARDWARE_COMPAT_FORBIDDEN_IMPORT_ROOTS
    )


def _is_forbidden_core_compat_import(module: str) -> bool:
    return any(
        module == root or module.startswith(f"{root}.")
        for root in CORE_COMPAT_FORBIDDEN_IMPORT_ROOTS
    )


def _is_type_checking_expr(node: ast.AST) -> bool:
    if isinstance(node, ast.Name):
        return node.id == "TYPE_CHECKING"
    if isinstance(node, ast.Attribute):
        return node.attr == "TYPE_CHECKING"
    return False


def _iter_imports(node: ast.AST, *, in_type_checking: bool = False):
    if isinstance(node, ast.If):
        is_type_checking = in_type_checking or _is_type_checking_expr(node.test)
        for child in node.body:
            yield from _iter_imports(child, in_type_checking=is_type_checking)
        for child in node.orelse:
            yield from _iter_imports(child, in_type_checking=in_type_checking)
        return
    if in_type_checking:
        return
    if isinstance(node, ast.Import):
        for alias in node.names:
            yield alias.name
        return
    if isinstance(node, ast.ImportFrom):
        if node.module and (node.level or 0) == 0:
            yield node.module
        return
    for child in ast.iter_child_nodes(node):
        yield from _iter_imports(child, in_type_checking=in_type_checking)


def _python_files(package: str) -> list[Path]:
    package_dir = SRC_DIR / package
    if not package_dir.exists():
        return []
    return sorted(
        path
        for path in package_dir.rglob("*.py")
        if "__pycache__" not in path.parts and not any(part.startswith(".") for part in path.parts)
    )


def _all_src_python_files() -> list[Path]:
    return sorted(
        path
        for path in SRC_DIR.rglob("*.py")
        if "__pycache__" not in path.parts and not any(part.startswith(".") for part in path.parts)
    )


def _hardware_boundary_files() -> list[Path]:
    files: set[Path] = set()
    for package_dir in HARDWARE_COMPAT_FORBIDDEN_DIRS:
        if not package_dir.exists():
            continue
        files.update(
            path
            for path in package_dir.rglob("*.py")
            if "__pycache__" not in path.parts and not any(part.startswith(".") for part in path.parts)
        )
    return sorted(files)


def _core_boundary_files() -> list[Path]:
    files: set[Path] = set()
    for package_dir in CORE_COMPAT_FORBIDDEN_DIRS:
        if not package_dir.exists():
            continue
        files.update(
            path
            for path in package_dir.rglob("*.py")
            if "__pycache__" not in path.parts and not any(part.startswith(".") for part in path.parts)
        )
    return sorted(files)


def validate_ros_import_boundaries() -> tuple[list[str], int, int]:
    violations: list[str] = []
    scanned = 0
    classified = 0
    for path in _all_src_python_files():
        if _is_test_or_example(path) or _is_ros_scan_excluded(path):
            continue
        rel = path.relative_to(ROOT_DIR).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError as exc:
            violations.append(f"{rel}: cannot parse Python source for ROS boundary scan: {exc}")
            continue
        scanned += 1
        ros_imports = sorted(
            {
                module
                for module in _iter_imports(tree)
                if _is_ros_import(module)
            }
        )
        if not ros_imports:
            continue
        classified += 1
        boundary = _ros_compat_boundary(path)
        if boundary is None:
            imports = ", ".join(ros_imports)
            violations.append(
                f"{rel}: imports ROS modules outside explicit compat boundary: {imports}"
            )
    return violations, scanned, classified


def validate_hardware_compat_boundaries() -> tuple[list[str], int]:
    violations: list[str] = []
    scanned = 0
    for path in _hardware_boundary_files():
        if _is_test_or_example(path):
            continue
        rel = path.relative_to(ROOT_DIR).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError as exc:
            violations.append(f"{rel}: cannot parse Python source for hardware boundary scan: {exc}")
            continue
        scanned += 1
        imports = sorted(
            {
                module
                for module in _iter_imports(tree)
                if _is_forbidden_hardware_compat_import(module)
            }
        )
        if imports:
            violations.append(
                f"{rel}: hardware package must not import ROS compatibility modules: "
                f"{', '.join(imports)}"
            )
    return violations, scanned


def validate_core_compat_boundaries() -> tuple[list[str], int]:
    violations: list[str] = []
    scanned = 0
    for path in _core_boundary_files():
        if _is_test_or_example(path):
            continue
        rel = path.relative_to(ROOT_DIR).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError as exc:
            violations.append(f"{rel}: cannot parse Python source for core compat scan: {exc}")
            continue
        scanned += 1
        imports = sorted(
            {
                module
                for module in _iter_imports(tree)
                if _is_forbidden_core_compat_import(module)
            }
        )
        if imports:
            violations.append(
                f"{rel}: core package must not import ROS compatibility modules: "
                f"{', '.join(imports)}"
            )
    return violations, scanned


def validate(verbose: bool = False) -> tuple[list[str], int]:
    violations: list[str] = []
    scanned = 0
    for package, forbidden in BOUNDARY_RULES.items():
        for path in _python_files(package):
            if _is_test_or_example(path) or _is_composition_exception(path):
                continue
            rel = path.relative_to(ROOT_DIR).as_posix()
            try:
                tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
            except SyntaxError as exc:
                violations.append(f"{rel}: cannot parse Python source: {exc}")
                continue
            scanned += 1
            for module in _iter_imports(tree):
                imported_top = _top_level(module)
                if imported_top in forbidden:
                    violations.append(f"{rel}: imports forbidden layer {module}")
    hardware_violations, hardware_scanned = validate_hardware_compat_boundaries()
    violations.extend(hardware_violations)
    core_violations, core_scanned = validate_core_compat_boundaries()
    violations.extend(core_violations)
    ros_violations, ros_scanned, ros_classified = validate_ros_import_boundaries()
    violations.extend(ros_violations)
    if verbose:
        print(f"Scanned {scanned} production Python files")
        print(f"Checked Thunder hardware compat boundary across {hardware_scanned} file(s)")
        print(f"Checked core compat boundary across {core_scanned} file(s)")
        print(
            "Classified ROS imports in "
            f"{ros_classified} explicit compat file(s) across {ros_scanned} scanned file(s)"
        )
    return violations, scanned


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate LingTu Module-First package boundaries")
    parser.add_argument("--verbose", action="store_true", help="Print scanned file count")
    args = parser.parse_args()

    violations, scanned = validate(verbose=args.verbose)
    if violations:
        print("Architecture boundary check: FAIL")
        for violation in violations:
            print(f"  ERROR: {violation}")
        print(f"\nResult: {len(violations)} violation(s) across {scanned} scanned file(s)")
        sys.exit(1)
    print(f"Architecture boundary check: OK ({scanned} production file(s) scanned)")


if __name__ == "__main__":
    main()
