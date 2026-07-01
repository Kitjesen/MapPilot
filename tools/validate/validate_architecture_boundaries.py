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

import yaml

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
ARCHITECTURE_LAYERS_PATH = ROOT_DIR / "config" / "architecture_layers.yaml"
ARCHITECTURE_LAYER_SCHEMA_VERSION = "lingtu.architecture_layers.v1"
ARCHITECTURE_LAYER_ORDER = (
    "L0_core_contract",
    "L1_runtime_model",
    "L2_product_composition",
    "L3_adapter_layer",
    "L4_capability_modules",
    "L5_algorithm_kernels",
    "L6_cli_deploy",
)
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from runtime.adapters.ros2.manifest import (  # noqa: E402
    ROS_IMPORT_ROOTS,
    ROS_SCAN_EXCLUDED_PREFIXES,
    ros_compat_boundary_for,
)

def load_architecture_layers(path: Path = ARCHITECTURE_LAYERS_PATH) -> dict:
    """Load the repository architecture layer manifest."""

    data = yaml.safe_load(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def _normalize_manifest_path(value: object) -> str:
    normalized = str(value).replace("\\", "/").strip()
    while normalized.startswith("./"):
        normalized = normalized[2:]
    return normalized


def _canonical_manifest_path(value: object) -> str:
    return _normalize_manifest_path(value).rstrip("/")


def _repo_path_from_manifest(value: object) -> Path:
    return ROOT_DIR / _canonical_manifest_path(value)


def _architecture_import_boundaries(manifest: dict) -> dict:
    boundaries = manifest.get("import_boundaries")
    if not isinstance(boundaries, dict):
        raise ValueError("config/architecture_layers.yaml: import_boundaries must be a mapping")
    return boundaries


def _architecture_string_list(manifest: dict, key: str) -> list[str]:
    values = _architecture_import_boundaries(manifest).get(key)
    if not isinstance(values, list):
        raise ValueError(f"config/architecture_layers.yaml: import_boundaries.{key} must be a list")
    normalized: list[str] = []
    for index, value in enumerate(values):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                "config/architecture_layers.yaml: "
                f"import_boundaries.{key}[{index}] must be a non-empty string"
            )
        normalized.append(_normalize_manifest_path(value))
    return normalized


def _architecture_package_rules(manifest: dict) -> dict[str, set[str]]:
    rules = _architecture_import_boundaries(manifest).get("package_forbidden_roots")
    if not isinstance(rules, dict):
        raise ValueError(
            "config/architecture_layers.yaml: "
            "import_boundaries.package_forbidden_roots must be a mapping"
        )
    parsed: dict[str, set[str]] = {}
    for package, roots in rules.items():
        if not isinstance(package, str) or not package.strip():
            raise ValueError(
                "config/architecture_layers.yaml: "
                "import_boundaries.package_forbidden_roots keys must be non-empty strings"
            )
        if not isinstance(roots, list):
            raise ValueError(
                "config/architecture_layers.yaml: "
                f"import_boundaries.package_forbidden_roots.{package} must be a list"
            )
        parsed[package] = set()
        for index, root in enumerate(roots):
            if not isinstance(root, str) or not root.strip():
                raise ValueError(
                    "config/architecture_layers.yaml: "
                    f"import_boundaries.package_forbidden_roots.{package}[{index}] "
                    "must be a non-empty string"
                )
            parsed[package].add(root)
    return parsed


def _architecture_path_set(manifest: dict, key: str) -> set[str]:
    return set(_architecture_string_list(manifest, key))


def _architecture_path_tuple(manifest: dict, key: str) -> tuple[Path, ...]:
    return tuple(_repo_path_from_manifest(value) for value in _architecture_string_list(manifest, key))


_ARCHITECTURE_LAYERS = load_architecture_layers()
BOUNDARY_RULES = _architecture_package_rules(_ARCHITECTURE_LAYERS)
COMPOSITION_EXCEPTIONS = _architecture_path_set(
    _ARCHITECTURE_LAYERS,
    "composition_exceptions",
)
HARDWARE_COMPAT_FORBIDDEN_IMPORT_ROOTS = _architecture_path_set(
    _ARCHITECTURE_LAYERS,
    "hardware_compat_forbidden_import_roots",
)
HARDWARE_COMPAT_FORBIDDEN_DIRS = _architecture_path_tuple(
    _ARCHITECTURE_LAYERS,
    "hardware_compat_forbidden_dirs",
)
CORE_COMPAT_FORBIDDEN_IMPORT_ROOTS = _architecture_path_set(
    _ARCHITECTURE_LAYERS,
    "core_compat_forbidden_import_roots",
)
CORE_COMPAT_FORBIDDEN_DIRS = _architecture_path_tuple(
    _ARCHITECTURE_LAYERS,
    "core_compat_forbidden_dirs",
)
SCAN_EXCLUDED_PARTS = {
    "__pycache__",
    "build",
    "build_nb",
    "build_nb_win",
    "_deps",
    "OrbbecSDK_ROS2",
}


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


def _is_scan_excluded_path(path: Path) -> bool:
    return any(
        part in SCAN_EXCLUDED_PARTS or part.startswith(".")
        for part in path.parts
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
        if not _is_scan_excluded_path(path)
    )


def _all_src_python_files() -> list[Path]:
    return sorted(
        path
        for path in SRC_DIR.rglob("*.py")
        if not _is_scan_excluded_path(path)
    )


def _manifest_owned_paths(manifest: dict) -> list[tuple[str, str]]:
    owned: list[tuple[str, str]] = []
    for layer in manifest.get("layers") or ():
        if not isinstance(layer, dict):
            continue
        layer_id = str(layer.get("id", ""))
        for value in layer.get("owns") or ():
            owned.append((layer_id, _normalize_manifest_path(value)))
    return owned


def _manifest_relpath(path: Path | str) -> str:
    candidate = Path(path)
    if candidate.is_absolute():
        try:
            return candidate.relative_to(ROOT_DIR).as_posix()
        except ValueError:
            return candidate.as_posix()
    return _normalize_manifest_path(path)


def _owned_path_matches(relpath: str, owned_path: str) -> bool:
    owned = owned_path.rstrip("/")
    if relpath == owned:
        return True
    return owned_path.endswith("/") and relpath.startswith(owned_path)


def architecture_layer_for_path(
    path: Path | str,
    manifest: dict | None = None,
) -> dict | None:
    """Return the most specific manifest layer owning a repository path."""

    manifest = manifest or _ARCHITECTURE_LAYERS
    relpath = _manifest_relpath(path)
    layers_by_id = {
        str(layer.get("id")): layer
        for layer in manifest.get("layers") or ()
        if isinstance(layer, dict)
    }
    matches = [
        (owned_path.rstrip("/"), layer_id)
        for layer_id, owned_path in _manifest_owned_paths(manifest)
        if _owned_path_matches(relpath, owned_path)
    ]
    if not matches:
        return None
    _owned_path, layer_id = max(matches, key=lambda item: len(item[0]))
    return layers_by_id.get(layer_id)


def _src_root_is_claimed(src_root: Path, owned_paths: list[str]) -> bool:
    rel = src_root.relative_to(ROOT_DIR).as_posix().rstrip("/") + "/"
    return any(path == rel.rstrip("/") or path.startswith(rel) for path in owned_paths)


def validate_architecture_layer_manifest(
    path: Path = ARCHITECTURE_LAYERS_PATH,
) -> tuple[list[str], dict]:
    """Validate the L0-L6 folder/layer ownership manifest."""

    violations: list[str] = []
    manifest = load_architecture_layers(path)
    if manifest.get("schema_version") != ARCHITECTURE_LAYER_SCHEMA_VERSION:
        violations.append(
            "config/architecture_layers.yaml: schema_version must be "
            f"{ARCHITECTURE_LAYER_SCHEMA_VERSION!r}"
        )

    layers = manifest.get("layers")
    if not isinstance(layers, list) or not layers:
        violations.append("config/architecture_layers.yaml: layers must be a non-empty list")
        layers = []
    layer_ids = [
        str(layer.get("id", ""))
        for layer in layers
        if isinstance(layer, dict)
    ]
    if tuple(layer_ids) != ARCHITECTURE_LAYER_ORDER:
        violations.append(
            "config/architecture_layers.yaml: layer ids must be ordered as "
            + ", ".join(ARCHITECTURE_LAYER_ORDER)
        )

    seen_owned: dict[str, str] = {}
    for layer in layers:
        if not isinstance(layer, dict):
            violations.append("config/architecture_layers.yaml: each layer must be a mapping")
            continue
        layer_id = str(layer.get("id", ""))
        if not layer.get("title"):
            violations.append(f"{layer_id}: title is required")
        owns = layer.get("owns")
        if not isinstance(owns, list) or not owns:
            violations.append(f"{layer_id}: owns must be a non-empty list")
            continue
        for raw_path in owns:
            if not isinstance(raw_path, str) or not raw_path.strip():
                violations.append(f"{layer_id}: owned paths must be non-empty strings")
                continue
            rel = _normalize_manifest_path(raw_path)
            canonical_rel = _canonical_manifest_path(raw_path)
            if canonical_rel in seen_owned:
                violations.append(
                    f"{layer_id}: owns duplicate path {rel!r}; already owned by "
                    f"{seen_owned[canonical_rel]}"
                )
            seen_owned[canonical_rel] = layer_id
            if not _repo_path_from_manifest(rel).exists():
                violations.append(f"{layer_id}: owned path does not exist: {rel}")

    owned_paths = [owned_path for _layer_id, owned_path in _manifest_owned_paths(manifest)]
    for src_root in sorted(path for path in SRC_DIR.iterdir() if path.is_dir()):
        if src_root.name == "__pycache__" or src_root.name.startswith("."):
            continue
        if not _src_root_is_claimed(src_root, owned_paths):
            violations.append(
                "config/architecture_layers.yaml: missing layer ownership for "
                f"{src_root.relative_to(ROOT_DIR).as_posix()}/"
            )
    for source_path in _all_src_python_files():
        if _is_test_or_example(source_path):
            continue
        if architecture_layer_for_path(source_path, manifest) is None:
            violations.append(
                "config/architecture_layers.yaml: missing layer ownership for "
                f"{source_path.relative_to(ROOT_DIR).as_posix()}"
            )

    boundaries = manifest.get("import_boundaries")
    if not isinstance(boundaries, dict):
        violations.append("config/architecture_layers.yaml: import_boundaries is required")
        boundaries = {}
    try:
        package_rules = _architecture_package_rules(manifest)
    except ValueError as exc:
        violations.append(str(exc))
        package_rules = {}
    if not package_rules:
        violations.append(
            "config/architecture_layers.yaml: import_boundaries.package_forbidden_roots "
            "must not be empty"
        )
    try:
        composition_exceptions = _architecture_path_set(manifest, "composition_exceptions")
    except ValueError as exc:
        violations.append(str(exc))
        composition_exceptions = set()
    if not composition_exceptions:
        violations.append(
            "config/architecture_layers.yaml: import_boundaries.composition_exceptions "
            "must not be empty"
        )
    for key in (
        "hardware_compat_forbidden_import_roots",
        "core_compat_forbidden_import_roots",
    ):
        try:
            _architecture_string_list(manifest, key)
        except ValueError as exc:
            violations.append(str(exc))
    for key in (
        "hardware_compat_forbidden_dirs",
        "core_compat_forbidden_dirs",
    ):
        try:
            boundary_paths = _architecture_string_list(manifest, key)
        except ValueError as exc:
            violations.append(str(exc))
            boundary_paths = []
        for raw_path in boundary_paths:
            rel = _normalize_manifest_path(raw_path)
            if not _repo_path_from_manifest(rel).exists():
                violations.append(f"config/architecture_layers.yaml: missing {key} path {rel}")

    return violations, manifest


def _hardware_boundary_files() -> list[Path]:
    files: set[Path] = set()
    for package_dir in HARDWARE_COMPAT_FORBIDDEN_DIRS:
        if not package_dir.exists():
            continue
        files.update(
            path
            for path in package_dir.rglob("*.py")
            if not _is_scan_excluded_path(path)
            and architecture_layer_for_path(path).get("id") != "L3_adapter_layer"
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
            if not _is_scan_excluded_path(path)
            and architecture_layer_for_path(path).get("id") != "L3_adapter_layer"
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
    manifest_violations, _manifest = validate_architecture_layer_manifest()
    violations.extend(manifest_violations)
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
