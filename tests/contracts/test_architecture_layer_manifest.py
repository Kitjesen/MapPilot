from __future__ import annotations

import ast
import copy
from pathlib import Path

import yaml
from tools.validate.validate_architecture_boundaries import (
    ARCHITECTURE_LAYER_ORDER,
    BOUNDARY_RULES,
    architecture_layer_for_path,
    load_architecture_layers,
    validate_architecture_layer_manifest,
)

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"


def _write_manifest(tmp_path: Path, manifest: dict) -> Path:
    path = tmp_path / "architecture_layers.yaml"
    path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return path


def _absolute_imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
            imports.add(node.module)
    return imports


def test_architecture_layer_manifest_is_complete_and_paths_exist() -> None:
    violations, manifest = validate_architecture_layer_manifest()

    assert violations == [], "\n".join(violations)
    assert [layer["id"] for layer in manifest["layers"]] == list(ARCHITECTURE_LAYER_ORDER)


def test_architecture_layer_manifest_drives_boundary_validator_rules() -> None:
    manifest = load_architecture_layers()
    expected_rules = {
        package: set(forbidden)
        for package, forbidden in manifest["import_boundaries"]["package_forbidden_roots"].items()
    }

    assert BOUNDARY_RULES == expected_rules


def test_architecture_layer_manifest_rejects_scalar_import_boundary_values(
    tmp_path: Path,
) -> None:
    manifest = copy.deepcopy(load_architecture_layers())
    manifest["import_boundaries"]["package_forbidden_roots"]["nav"] = "semantic"
    manifest["import_boundaries"]["composition_exceptions"] = "lingtu/assembly/"
    manifest["import_boundaries"]["hardware_compat_forbidden_dirs"] = "src/drivers/real/thunder/"

    violations, _manifest = validate_architecture_layer_manifest(_write_manifest(tmp_path, manifest))

    assert any("package_forbidden_roots.nav must be a list" in item for item in violations)
    assert any("composition_exceptions must be a list" in item for item in violations)
    assert any("hardware_compat_forbidden_dirs must be a list" in item for item in violations)


def test_architecture_layer_manifest_rejects_equal_paths_with_trailing_slash_drift(
    tmp_path: Path,
) -> None:
    manifest = copy.deepcopy(load_architecture_layers())
    manifest["layers"][3]["owns"].append("src/nav/kernel")

    violations, _manifest = validate_architecture_layer_manifest(_write_manifest(tmp_path, manifest))

    assert any("owns duplicate path" in item and "src/nav/kernel" in item for item in violations)


def test_architecture_layer_manifest_uses_most_specific_path_owner() -> None:
    assert architecture_layer_for_path("src/nav/local/path_follower.py")["id"] == "L4_capability_modules"
    assert architecture_layer_for_path("src/nav/local/paths")["id"] == "L5_algorithm_kernels"
    assert architecture_layer_for_path("src/runtime/adapters/dds/reader.py")["id"] == "L3_adapter_layer"
    assert architecture_layer_for_path("src/maps/adapters/native/map_save.py")["id"] == "L3_adapter_layer"
    assert architecture_layer_for_path("src/nav/cpp/CMakeLists.txt")["id"] == ("L5_algorithm_kernels")
    assert architecture_layer_for_path("src/nav/adapters/dds/nav/path_bridge.py")["id"] == ("L3_adapter_layer")


def test_lite_manifest_does_not_carry_retired_ros2_adapter_paths() -> None:
    lite_manifest = yaml.safe_load((ROOT / "config" / "thunder_lite_package.yaml").read_text(encoding="utf-8-sig"))
    lite_excludes = set(lite_manifest["package"]["exclude_paths"])
    lite_omits = set(lite_manifest["package"]["omit_paths"])

    assert "src/nav/adapters/dds/" in lite_excludes
    retired = {
        "src/runtime/adapters/ros2/",
        "src/localization/adapters/ros2/",
        "src/perception/adapters/ros2/",
        "src/lingtu/ros2_plugin_seed.py",
        "src/lingtu/ros2_shutdown.py",
    }
    assert retired.isdisjoint(lite_excludes)
    assert retired.isdisjoint(lite_omits)


def test_product_composition_does_not_import_runtime_bootstrap_surfaces() -> None:
    forbidden = {
        "cli.bootstrap",
        "cli.runtime_bootstrap",
        "cli.runtime_extra",
        "cli.runtime_audit",
        "lingtu.ros2_plugin_seed",
        "lingtu.ros2_shutdown",
    }
    violations: list[str] = []

    for path in (SRC / "runtime" / "blueprints" / "products").rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        imports = _absolute_imports(path)
        leaked = sorted(
            module for module in imports if any(module == root or module.startswith(f"{root}.") for root in forbidden)
        )
        if leaked:
            violations.append(f"{rel}: imports {', '.join(leaked)}")

    assert violations == [], "\n".join(violations)
