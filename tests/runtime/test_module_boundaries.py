from __future__ import annotations

import ast
from pathlib import Path

import pytest
from tools.validate.validate_architecture_boundaries import (
    ARCHITECTURE_LAYER_ORDER,
    BOUNDARY_RULES,
    COMPOSITION_EXCEPTIONS,
    _is_scan_excluded_path,
    _iter_imports,
    architecture_layer_for_path,
    validate_architecture_layer_manifest,
)

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"


def _python_files(package: str) -> list[Path]:
    return sorted(path for path in (SRC / package).rglob("*.py") if not _is_scan_excluded_path(path))


def _imported_modules(tree: ast.AST) -> list[str]:
    """Return top-level absolute import module names.

    Only checks direct children of the module body (skips imports inside
    functions/methods/classes; those are lazy and do not create hard
    cross-package coupling).  Also skips relative imports (``from .foo``)
    which stay within the same package.
    """
    return list(_iter_imports(tree))


def _all_absolute_imports(tree: ast.AST) -> list[str]:
    """Return absolute imports anywhere in a module, including lazy imports."""
    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
            modules.append(node.module)
    return modules


def _top_level(module: str) -> str:
    return module.split(".", 1)[0]


def _is_test_file(path: Path) -> bool:
    return "tests" in path.parts or "test" in path.parts or path.name.startswith("test_")


def _is_example_file(path: Path) -> bool:
    return "examples" in path.parts or "example" in path.parts


def _is_composition_exception(rel: str) -> bool:
    """Check if a file is exempt from boundary checks (composition/glue code)."""
    src_rel = rel.removeprefix("src/")
    for exc in COMPOSITION_EXCEPTIONS:
        if src_rel == exc or src_rel.startswith(exc):
            return True
    return False


def test_architecture_layer_manifest_is_valid_and_drives_boundary_rules() -> None:
    violations, manifest = validate_architecture_layer_manifest()

    assert violations == [], "\n".join(violations)
    assert [layer["id"] for layer in manifest["layers"]] == list(ARCHITECTURE_LAYER_ORDER)
    import_boundaries = manifest["import_boundaries"]
    assert BOUNDARY_RULES == {
        package: set(roots) for package, roots in import_boundaries["package_forbidden_roots"].items()
    }
    assert COMPOSITION_EXCEPTIONS == set(import_boundaries["composition_exceptions"])
    assert "src/lingtu/assembly/" in manifest["layers"][2]["owns"]
    assert "src/runtime/adapters/" in manifest["layers"][3]["owns"]
    assert "src/nav/adapters/" in manifest["layers"][3]["owns"]
    assert "src/localization/adapters/" in manifest["layers"][3]["owns"]
    assert "src/nav/" in manifest["layers"][4]["owns"]
    assert "src/nav/cpp/" in manifest["layers"][5]["owns"]
    assert "src/lingtu/control.py" in manifest["layers"][6]["owns"]
    assert "localization" in BOUNDARY_RULES["decision"]


def test_architecture_layer_lookup_uses_most_specific_path_owner() -> None:
    assert architecture_layer_for_path("src/nav/commands/module.py")["id"] == "L4_capability_modules"
    assert architecture_layer_for_path("src/nav/services/goals.py")["id"] == "L4_capability_modules"
    assert architecture_layer_for_path("src/nav/skills/skills_module.py")["id"] == "L4_capability_modules"
    assert architecture_layer_for_path("src/nav/cpp/CMakeLists.txt")["id"] == ("L5_algorithm_kernels")
    assert architecture_layer_for_path(
        "src/nav/cpp/planning/local/planner.hpp"
    )["id"] == "L5_algorithm_kernels"
    assert architecture_layer_for_path("src/nav/cpp/planning/local/cmu/paths")["id"] == "L5_algorithm_kernels"
    assert architecture_layer_for_path("src/runtime/adapters/dds/reader.py")["id"] == "L3_adapter_layer"
    assert architecture_layer_for_path("src/nav/adapters/dds/nav/map_out.py")["id"] == ("L3_adapter_layer")


@pytest.mark.parametrize("package,forbidden", BOUNDARY_RULES.items())
def test_package_does_not_import_forbidden_layers_directly(
    package: str,
    forbidden: set[str],
) -> None:
    violations: list[str] = []

    for path in _python_files(package):
        rel = path.relative_to(ROOT).as_posix()
        if _is_test_file(path) or _is_example_file(path) or _is_composition_exception(rel):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError as exc:
            violations.append(f"{rel}: cannot parse Python source: {exc}")
            continue
        for module in _iter_imports(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_product_assembly_does_not_import_cli_profile_surfaces() -> None:
    violations: list[str] = []

    for path in (SRC / "lingtu" / "assembly").rglob("*.py"):
        if "__pycache__" in path.parts or _is_test_file(path):
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules = [node.module]
            else:
                continue
            for module in modules:
                if _top_level(module) == "cli":
                    violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_product_assembly_uses_registered_plugin_catalog_boundary() -> None:
    violations: list[str] = []

    for path in (SRC / "lingtu" / "assembly").rglob("*.py"):
        if "__pycache__" in path.parts or _is_test_file(path):
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for module in _all_absolute_imports(tree):
            if module in {
                "lingtu.assembly.plugins",
                "lingtu.ros2_plugin_seed",
                "lingtu.ros2_shutdown",
            }:
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_runtime_does_not_import_product_assembly() -> None:
    violations: list[str] = []

    for path in _python_files("runtime"):
        if _is_test_file(path) or _is_example_file(path):
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for module in _all_absolute_imports(tree):
            if module == "lingtu.assembly" or module.startswith("lingtu.assembly."):
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_stack_factories_do_not_import_service_manager_directly() -> None:
    violations: list[str] = []

    for path in (SRC / "lingtu" / "assembly" / "stacks").glob("*.py"):
        if path.name == "__init__.py" or "__pycache__" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        imports = set(_all_absolute_imports(tree))
        if "runtime.service_manager" in imports:
            violations.append(f"{rel}: imports runtime.service_manager")

    assert violations == [], "\n".join(violations)


def test_navigation_split_stack_modules_keep_adapter_imports_lazy_and_isolated() -> None:
    def direct_module_imports(tree: ast.AST) -> set[str]:
        modules: set[str] = set()
        if not isinstance(tree, ast.Module):
            return modules
        for node in tree.body:
            if isinstance(node, ast.Import):
                modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
                modules.add(node.module)
        return modules

    stack_dir = SRC / "lingtu" / "assembly" / "stacks"
    forbidden_core_import_roots = {
        "adapters",
        "compat",
        "gateway",
        "nav",
        "slam",
    }

    navigation = ast.parse((stack_dir / "navigation.py").read_text(encoding="utf-8-sig"))
    navigation_import_roots = {_top_level(module) for module in direct_module_imports(navigation)}
    assert navigation_import_roots.isdisjoint(forbidden_core_import_roots)

def test_thunder_driver_blueprints_compatibility_file_is_removed() -> None:
    assert not (SRC / "drivers" / "real" / "thunder" / "blueprints.py").exists()


def test_driver_modules_do_not_import_slam_factories() -> None:
    violations: list[str] = []

    for path in _python_files("drivers"):
        rel = path.relative_to(ROOT).as_posix()
        if _is_test_file(path) or _is_example_file(path):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError as exc:
            violations.append(f"{rel}: cannot parse Python source: {exc}")
            continue
        for module in _all_absolute_imports(tree):
            if module == "localization.native_factories" or module.startswith("localization.native_factories."):
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_livox_ros2_driver_factory_is_removed() -> None:
    assert not (SRC / "drivers" / "real" / "lidar" / "compat" / "native_factory.py").exists()
    assert not (SRC / "drivers" / "real" / "lidar" / "impl" / "livox" / "native_factory.py").exists()


def test_lidar_module_source_does_not_import_ros_compat_directly() -> None:
    path = SRC / "drivers" / "real" / "lidar" / "module.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_all_absolute_imports(tree))

    assert "runtime.adapters.ros2.native_module" not in imports


def test_runtime_top_level_api_does_not_export_native_module() -> None:
    path = SRC / "runtime" / "__init__.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "from .native_module import" not in text
    assert '"NativeModule"' not in text
    assert '"NativeModuleConfig"' not in text


def test_ros2_adapter_packages_are_removed() -> None:
    retired = (
        SRC / "drivers" / "adapters" / "ros2",
        SRC / "localization" / "adapters" / "ros2",
        SRC / "localization" / "interface",
        SRC / "perception" / "adapters" / "ros2",
        SRC / "runtime" / "adapters" / "ros2",
    )

    assert [path for path in retired if path.exists()] == []


def test_product_sources_have_no_ros_imports() -> None:
    from tools.validate.validate_architecture_boundaries import (
        validate_ros_import_boundaries,
    )

    violations, scanned, classified = validate_ros_import_boundaries()

    assert violations == [], "\n".join(violations)
    assert scanned > 0
    assert classified == 0


def test_python_maps_service_is_removed() -> None:
    path = SRC / "maps" / "modules" / "service.py"
    assert not path.exists()


def test_gateway_map_routes_do_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "gateway" / "routes" / "maps.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "/nav/save_map" not in text
    assert "ros2 service call" not in text
    assert "runtime.adapters.ros2" not in text


def test_gateway_does_not_reach_into_private_planner_state() -> None:
    violations: list[str] = []
    for path in _python_files("gateway"):
        if _is_test_file(path):
            continue
        text = path.read_text(encoding="utf-8-sig")
        if "_planner_svc" in text:
            violations.append(str(path.relative_to(ROOT)))

    assert violations == []


def test_gateway_and_maps_do_not_control_services_through_runtime_manager() -> None:
    violations: list[str] = []
    for package in ("gateway", "maps"):
        for path in _python_files(package):
            if _is_test_file(path):
                continue
            text = path.read_text(encoding="utf-8-sig")
            if "runtime.service_manager" in text:
                violations.append(str(path.relative_to(ROOT)))

    assert violations == []


def test_gateway_module_does_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "gateway" / "gateway_module.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/nav/save_map" not in text
    assert "interface/srv/SaveMaps" not in text
    assert "ros2 service call" not in text
    assert "runtime.adapters.ros2" not in text


def test_gateway_camera_route_does_not_embed_ros2_snapshot_script() -> None:
    path = SRC / "gateway" / "routes" / "camera.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "rclpy" not in text
    assert "sensor_msgs" not in text
    assert "subprocess.run" not in text
    assert "runtime.adapters.ros2" not in text
    assert "camera_snapshot_adapter" in text


def test_removed_bus_package_is_not_imported_by_runtime_sources() -> None:
    removed_bus = "lc" + "m"
    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if _is_scan_excluded_path(path) or _is_test_file(path) or _is_example_file(path):
            continue
        rel = path.relative_to(ROOT).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError):
            continue

        imports: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
                imports.add(node.module)
        has_lcm_import = any(module == removed_bus or module.startswith(f"{removed_bus}.") for module in imports)
        if has_lcm_import:
            violations.append(f"{rel}: imports LCM package directly")

    assert violations == [], "\n".join(violations)


def test_ros_compat_manifest_is_removed() -> None:
    assert not (SRC / "runtime" / "adapters" / "ros2" / "manifest.py").exists()


def test_core_has_no_ros2_context_compatibility_proxy() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_forbidden_core_compat_import,
        validate_core_compat_boundaries,
    )

    violations, scanned = validate_core_compat_boundaries()

    assert not (SRC / "runtime" / "ros2_context.py").exists()
    assert scanned > 0
    assert violations == [], "\n".join(violations)
    assert _is_forbidden_core_compat_import("runtime.adapters.ros2")
    assert _is_forbidden_core_compat_import("runtime.adapters.ros2.context")


def test_product_runtime_paths_do_not_import_ros_modules() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    product_paths = (
        SRC / "lingtu" / "assembly" / "products",
        SRC / "runtime" / "profiles",
    )
    violations: list[str] = []

    for root in product_paths:
        for path in root.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            rel = path.relative_to(ROOT).as_posix()
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
            imports = sorted(module for module in _iter_imports(tree) if _is_ros_import(module))
            if imports:
                violations.append(f"{rel}: imports {', '.join(imports)}")

    assert violations == [], "\n".join(violations)


def test_legacy_slam_ros2_bridge_file_is_removed() -> None:
    assert not (ROOT / "src/localization/bridge.py").exists()


def test_legacy_slam_relocalization_service_file_is_removed() -> None:
    assert not (ROOT / "src/localization/relocalization.py").exists()
    assert not (ROOT / "src/runtime/relocalization.py").exists()
    assert not (ROOT / "src/runtime/adapters/native/relocalization.py").exists()
    assert (ROOT / "src/localization/service.py").is_file()
    assert (ROOT / "src/localization/adapters/relocalization.py").is_file()


def test_domain_adapters_do_not_live_under_runtime() -> None:
    assert not (ROOT / "src/runtime/adapters/native").exists()
    assert not (ROOT / "src/runtime/adapters/localization.py").exists()

    canonical = (
        "src/nav/adapters/native/abi.py",
        "src/nav/adapters/native/commands.py",
        "src/nav/adapters/native/inspection_commands.py",
        "src/nav/adapters/native/inspection_store.py",
        "src/localization/adapters/status.py",
        "src/localization/adapters/resolver.py",
    )
    assert [rel for rel in canonical if not (ROOT / rel).is_file()] == []

    retired_modules = {
        "runtime.adapters.localization",
        "runtime.adapters.native",
        "runtime.adapters.native.inspection",
        "runtime.adapters.native.inspection_commands",
        "runtime.adapters.native.localization_adapter",
        "runtime.adapters.native.navigation",
        "runtime.adapters.native.navigation_abi",
        "runtime.adapters.native.relocalization",
    }
    violations: list[str] = []
    for path in SRC.rglob("*.py"):
        if _is_scan_excluded_path(path) or path.resolve() == Path(__file__).resolve():
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError):
            continue
        for module in _all_absolute_imports(tree):
            if module in retired_modules or module.startswith("runtime.adapters.native."):
                violations.append(f"{path.relative_to(ROOT)}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_native_navigation_has_one_canonical_runtime_location() -> None:
    cpp = ROOT / "src/nav/cpp"
    assert (cpp / "planning/local/planner.hpp").is_file()
    assert (cpp / "planning/local/planner.cpp").is_file()
    assert (cpp / "planning/local/cmu/backend.cpp").is_file()
    assert (cpp / "planning/local/scan/backend.cpp").is_file()
    assert (cpp / "tracking/follower.hpp").is_file()
    assert (cpp / "tracking/follower.cpp").is_file()
    assert (cpp / "tracking/smoother.cpp").is_file()
    assert (cpp / "navigation/executor.hpp").is_file()
    assert (cpp / "navigation/executor.cpp").is_file()
def test_thunder_hardware_package_does_not_import_ros_compat_modules() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_forbidden_hardware_compat_import,
        validate_hardware_compat_boundaries,
    )

    violations, scanned = validate_hardware_compat_boundaries()

    assert scanned > 0
    assert violations == [], "\n".join(violations)
    assert _is_forbidden_hardware_compat_import("runtime.adapters.ros2")


def test_exploration_tare_package_has_no_ros2_bridge_shim() -> None:
    assert not (ROOT / "src/explore/tare/ros2_bridge.py").exists()

    text = (ROOT / "src/explore/tare/__init__.py").read_text(encoding="utf-8-sig")
    assert "TAREROS2BridgeModule" not in text


def test_legacy_gateway_rerun_bridge_file_is_removed() -> None:
    assert not (ROOT / "src/gateway/rerun_bridge_module.py").exists()


def test_legacy_perception_ros2_facade_files_are_removed() -> None:
    legacy_shims = (
        "src/perception/perception_publishers.py",
        "src/perception/reconstruction/bag_reader.py",
    )

    assert [rel for rel in legacy_shims if (ROOT / rel).exists()] == []


def test_runtime_ros2_shutdown_file_is_removed() -> None:
    assert not (ROOT / "src/lingtu/ros2_shutdown.py").exists()


def test_production_ros_bridges_use_external_adapter_context_import_path() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_ros_scan_excluded,
        _is_test_or_example,
    )

    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if (
            "__pycache__" in path.parts
            or _is_scan_excluded_path(path)
            or _is_test_or_example(path)
            or _is_ros_scan_excluded(path)
        ):
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        if "runtime.ros2_context" in set(_iter_imports(tree)):
            violations.append(f"{rel}: imports runtime.ros2_context")

    assert violations == [], "\n".join(violations)
