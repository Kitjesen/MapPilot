from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.validate.validate_architecture_boundaries import (
    ARCHITECTURE_LAYER_ORDER,
    BOUNDARY_RULES,
    COMPOSITION_EXCEPTIONS,
    _iter_imports,
    _is_scan_excluded_path,
    architecture_layer_for_path,
    validate_architecture_layer_manifest,
)

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"

def _python_files(package: str) -> list[Path]:
    return sorted(
        path
        for path in (SRC / package).rglob("*.py")
        if not _is_scan_excluded_path(path)
    )


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
    return (
        "tests" in path.parts
        or "test" in path.parts
        or path.name.startswith("test_")
    )


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
        package: set(roots)
        for package, roots in import_boundaries["package_forbidden_roots"].items()
    }
    assert COMPOSITION_EXCEPTIONS == set(import_boundaries["composition_exceptions"])
    assert "src/runtime/profiles/" in manifest["layers"][1]["owns"]
    assert "src/runtime/blueprints/" in manifest["layers"][2]["owns"]
    assert "src/runtime/adapters/" in manifest["layers"][3]["owns"]
    assert "src/nav/adapters/" in manifest["layers"][3]["owns"]
    assert "src/nav/local/" in manifest["layers"][4]["owns"]
    assert "src/nav/kernel/" in manifest["layers"][5]["owns"]
    assert "cli/" in manifest["layers"][6]["owns"]


def test_architecture_layer_lookup_uses_most_specific_path_owner() -> None:
    assert (
        architecture_layer_for_path("src/nav/mission/navigation.py")["id"]
        == "L4_capability_modules"
    )
    assert architecture_layer_for_path("src/nav/kernel/CMakeLists.txt")["id"] == (
        "L5_algorithm_kernels"
    )
    assert architecture_layer_for_path(
        "src/nav/services/plan/local_planner/service.py"
    )["id"] == "L5_algorithm_kernels"
    assert architecture_layer_for_path(
        "src/nav/services/plan/local_planner/paths"
    )["id"] == "L5_algorithm_kernels"
    assert architecture_layer_for_path("src/nav/local/legacy_ros/local_planner/src/pathFollower.cpp")[
        "id"
    ] == "L3_adapter_layer"
    assert architecture_layer_for_path("src/nav/local/legacy_ros/terrain_analysis_ext/src/terrainAnalysisExt.cpp")[
        "id"
    ] == "L3_adapter_layer"
    assert architecture_layer_for_path("src/nav/adapters/ros2/nav/map_out.py")["id"] == (
        "L3_adapter_layer"
    )


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


def test_core_blueprints_do_not_import_cli_profile_surfaces() -> None:
    violations: list[str] = []

    for path in (SRC / "core" / "blueprints").rglob("*.py"):
        if "__pycache__" in path.parts:
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


def test_core_blueprints_do_not_import_product_runtime_catalog() -> None:
    violations: list[str] = []

    for path in (SRC / "core" / "blueprints").rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for module in _all_absolute_imports(tree):
            if module in {
                "lingtu.plugin_seed",
                "lingtu.ros2_plugin_seed",
                "lingtu.ros2_shutdown",
            }:
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_stack_factories_do_not_import_service_manager_directly() -> None:
    violations: list[str] = []

    for path in (SRC / "core" / "blueprints" / "stacks").glob("*.py"):
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
            elif (
                isinstance(node, ast.ImportFrom)
                and node.module
                and (node.level or 0) == 0
            ):
                modules.add(node.module)
        return modules

    stack_dir = SRC / "runtime" / "blueprints" / "stacks"
    forbidden_core_import_roots = {
        "adapters",
        "compat",
        "gateway",
        "nav",
        "slam",
    }

    navigation_core = ast.parse(
        (stack_dir / "navigation_core.py").read_text(encoding="utf-8-sig")
    )
    navigation_core_import_roots = {
        _top_level(module) for module in direct_module_imports(navigation_core)
    }
    assert navigation_core_import_roots.isdisjoint(forbidden_core_import_roots)

    autonomy_chain = ast.parse(
        (stack_dir / "autonomy_chain.py").read_text(encoding="utf-8-sig")
    )
    assert "nav.local.stack" not in set(
        direct_module_imports(autonomy_chain)
    )
    assert "nav.local.stack" in set(
        _all_absolute_imports(autonomy_chain)
    )

    exploration_goal_sources = ast.parse(
        (stack_dir / "exploration_goal_sources.py").read_text(encoding="utf-8-sig")
    )
    exploration_imports = direct_module_imports(exploration_goal_sources)
    assert "nav.exploration.frontier_explorer_module" not in exploration_imports
    assert "nav.exploration.traversable_frontier_module" not in exploration_imports

    navigation = ast.parse(
        (stack_dir / "navigation.py").read_text(encoding="utf-8-sig")
    )
    assert "runtime.blueprints.adapters.navigation_io" not in set(
        direct_module_imports(navigation)
    )


def test_thunder_driver_blueprints_are_compatibility_shims() -> None:
    path = SRC / "drivers" / "real" / "thunder" / "blueprints.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

    imported = {
        module
        for module in _iter_imports(tree)
        if not module.startswith("typing")
    }
    assert "runtime.blueprints.products.thunder" in imported
    assert all(_top_level(module) not in {"nav", "perception", "decision", "gateway"} for module in imported)

    forbidden_calls = {"stack_module", "optional_stack_module"}
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert calls.isdisjoint(forbidden_calls)


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
            if module == "localization.native_factories" or module.startswith(
                "localization.native_factories."
            ):
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)


def test_livox_driver_native_factory_is_lazy_compatibility_shim() -> None:
    path = SRC / "drivers" / "real" / "lidar" / "native_factory.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))

    assert "runtime.native_module" not in imports


def test_lidar_module_source_does_not_import_ros_compat_directly() -> None:
    path = SRC / "drivers" / "real" / "lidar" / "lidar.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_all_absolute_imports(tree))

    assert "runtime.native_module" not in imports


def test_runtime_top_level_api_does_not_export_native_module() -> None:
    path = SRC / "runtime" / "__init__.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "from .native_module import" not in text
    assert '"NativeModule"' not in text
    assert '"NativeModuleConfig"' not in text


def test_native_module_helpers_are_limited_to_ros2_compat_adapters() -> None:
    allowed: set[str] = set()
    forbidden_imports = {"runtime.native_install", "runtime.native_module"}
    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if "__pycache__" in path.parts or _is_test_file(path) or _is_example_file(path):
            continue
        rel = path.relative_to(ROOT).as_posix()
        if rel in allowed or rel in {
            "src/runtime/native_install.py",
            "src/runtime/native_module.py",
        }:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError):
            continue
        imports = set(_all_absolute_imports(tree))
        used = sorted(imports & forbidden_imports)
        if used:
            violations.append(f"{rel}: imports {', '.join(used)}")

    assert violations == [], "\n".join(violations)


def test_ros_imports_are_explicit_compat_boundaries() -> None:
    from tools.validate.validate_architecture_boundaries import (
        validate_ros_import_boundaries,
    )

    from runtime.adapters.ros2.manifest import ROS_COMPAT_IMPORT_BOUNDARIES

    violations, scanned, classified = validate_ros_import_boundaries()
    allowed_prefixes = {boundary.prefix for boundary in ROS_COMPAT_IMPORT_BOUNDARIES}
    categories = {boundary.category for boundary in ROS_COMPAT_IMPORT_BOUNDARIES}

    assert violations == [], "\n".join(violations)
    assert scanned > 0
    assert classified > 0
    assert "slam_ros2_bridge" in categories
    assert "tare_ros2_bridge" in categories
    assert "simulation_endpoint_adapter" in categories
    assert "map_save_ros2_adapter" in categories
    assert "relocalization_ros2_adapter" in categories
    assert "runtime/adapters/ros2/context.py" in allowed_prefixes
    assert "runtime/adapters/ros2/map_save.py" in allowed_prefixes
    assert "drivers/adapters/ros2/camera_bridge.py" in allowed_prefixes
    assert "drivers/adapters/ros2/camera_snapshot.py" in allowed_prefixes
    assert "drivers/adapters/ros2/livox_driver.py" in allowed_prefixes
    assert "drivers/adapters/ros2/mujoco_ros2_bridge.py" in allowed_prefixes
    assert "drivers/adapters/ros2/mujoco_viz_bridge.py" in allowed_prefixes
    assert "drivers/adapters/ros2/nova_nav_bridge.py" in allowed_prefixes
    assert "drivers/adapters/ros2/sim_driver.py" in allowed_prefixes
    assert "nav/adapters/ros2/nav/map_out.py" in allowed_prefixes
    assert "nav/adapters/ros2/nav/nav_out.py" in allowed_prefixes
    assert "nav/adapters/ros2/tare_bridge.py" in allowed_prefixes
    assert "perception/adapters/ros2/bag_reader.py" in allowed_prefixes
    assert "perception/adapters/ros2/perception_publishers.py" in allowed_prefixes
    assert "localization/adapters/ros2/relocalization_service.py" in allowed_prefixes
    assert "localization/adapters/ros2/slam_bridge.py" in allowed_prefixes
    assert "gateway/visualization/rerun_bridge.py" in allowed_prefixes
    assert "drivers/sim/" not in allowed_prefixes
    assert "drivers/sim/mujoco_ros2_bridge.py" not in allowed_prefixes
    assert "drivers/sim/mujoco_viz_bridge.py" not in allowed_prefixes
    assert "drivers/sim/nova_nav_bridge.py" not in allowed_prefixes
    assert "drivers/sim/ros2_sim_driver.py" not in allowed_prefixes
    assert "drivers/real/thunder/camera_bridge_module.py" not in allowed_prefixes
    assert "exploration/module.py" not in allowed_prefixes
    assert "exploration/ros2_bridge.py" not in allowed_prefixes
    assert "gateway/rerun_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_grid_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_path_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_waypoint_bridge_module.py" not in allowed_prefixes
    assert "perception/perception_publishers.py" not in allowed_prefixes
    assert "perception/reconstruction/bag_reader.py" not in allowed_prefixes
    assert "core/ros2_context.py" not in allowed_prefixes
    assert "runtime/blueprints/" not in allowed_prefixes
    assert "core/runtime/" not in allowed_prefixes
    assert "runtime/blueprints/products/" not in allowed_prefixes


def test_map_manager_does_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "nav" / "services" / "maps.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "interface/srv/SaveMaps" not in text
    assert "subprocess.run" not in text
    assert "runtime.adapters.ros2" not in text


def test_lingtu_facades_do_not_construct_ros2_map_save_commands() -> None:
    for path in (
        SRC / "lingtu" / "runtime.py",
        SRC / "lingtu" / "robot.py",
    ):
        text = path.read_text(encoding="utf-8-sig")

        assert "/pgo/save_maps" not in text
        assert "interface/srv/SaveMaps" not in text
        assert "subprocess.run" not in text
        assert "runtime.adapters.ros2" not in text


def test_gateway_map_routes_do_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "gateway" / "routes" / "maps.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "/nav/save_map" not in text
    assert "ros2 service call" not in text
    assert "runtime.adapters.ros2" not in text


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


def test_lcm_imports_stay_in_core_transport_boundary() -> None:
    allowed_paths = {"src/runtime/transport/lcm.py"}
    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if "__pycache__" in path.parts or _is_test_file(path) or _is_example_file(path):
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
        has_lcm_import = any(module == "lcm" or module.startswith("lcm.") for module in imports)
        if has_lcm_import and rel not in allowed_paths:
            violations.append(f"{rel}: imports LCM package directly")

    assert violations == [], "\n".join(violations)


def test_ros_compat_manifest_has_unique_ordered_boundaries() -> None:
    from runtime.adapters.ros2.manifest import (
        ROS_COMPAT_IMPORT_BOUNDARIES,
        ROS_COMPAT_POLICY,
        ROS_COMPAT_STATUS,
        ROS_IMPORT_ROOTS,
        ros_compat_boundary_for,
    )

    prefixes = [boundary.prefix for boundary in ROS_COMPAT_IMPORT_BOUNDARIES]

    assert len(prefixes) == len(set(prefixes))
    assert list(prefixes) == sorted(prefixes)
    assert ROS_COMPAT_STATUS == "legacy_optional"
    assert "Product and portable runtime paths" in ROS_COMPAT_POLICY
    assert "rclpy" in ROS_IMPORT_ROOTS
    assert ros_compat_boundary_for("localization/adapters/ros2/slam_bridge.py").category == "slam_ros2_bridge"
    assert ros_compat_boundary_for("runtime/adapters/ros2/map_save.py").category == "map_save_ros2_adapter"
    assert (
        ros_compat_boundary_for("localization/adapters/ros2/relocalization_service.py").category
        == "relocalization_ros2_adapter"
    )
    assert ros_compat_boundary_for("slam/bridge.py") is None
    assert ros_compat_boundary_for("runtime/blueprints/products/thunder.py") is None


def test_core_has_no_ros2_context_compatibility_proxy() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_forbidden_core_compat_import,
        validate_core_compat_boundaries,
    )

    violations, scanned = validate_core_compat_boundaries()

    assert not (SRC / "core" / "ros2_context.py").exists()
    assert scanned > 0
    assert violations == [], "\n".join(violations)
    assert _is_forbidden_core_compat_import("runtime.adapters.ros2")
    assert _is_forbidden_core_compat_import("runtime.adapters.ros2.context")
    assert not _is_forbidden_core_compat_import("runtime.adapters.lcm.endpoint_runner")


def test_product_runtime_paths_do_not_import_ros_modules() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    product_paths = (
        SRC / "core" / "blueprints" / "products",
        SRC / "core" / "blueprints" / "catalog",
        SRC / "core" / "runtime",
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


def test_navigation_constructor_surface_is_endpoint_neutral() -> None:
    import inspect

    from nav.mission.navigation import Navigation

    signature = inspect.signature(Navigation)
    nav = Navigation()

    assert "enable_ros2_bridge" not in signature.parameters
    assert "enable_ros2_bridge" not in nav._config
    assert not hasattr(nav, "_legacy_enable_ros2_bridge")
    assert not hasattr(nav, "_enable_ros2_bridge")
    with pytest.raises(TypeError, match="no longer accepts enable_ros2_bridge"):
        Navigation(enable_ros2_bridge=False)


def test_legacy_nav_root_facade_files_are_removed() -> None:
    legacy_shims = (
        "src/nav/velocity_mux.py",
        "src/nav/elevation_map_module.py",
        "src/nav/esdf_module.py",
        "src/nav/frontier_explorer_module.py",
        "src/nav/global_planner.py",
        "src/nav/navigation.py",
        "src/nav/occupancy_grid_module.py",
        "src/nav/plan_safety.py",
        "src/nav/ros2_grid_bridge_module.py",
        "src/nav/ros2_path_bridge_module.py",
        "src/nav/ros2_waypoint_bridge_module.py",
        "src/nav/safety_ring.py",
        "src/nav/traversability_cost_module.py",
        "src/nav/traversable_frontier_module.py",
        "src/nav/voxel_grid_module.py",
        "src/nav/waypoint_tracker.py",
    )

    assert [rel for rel in legacy_shims if (ROOT / rel).exists()] == []


def test_internal_code_uses_compat_ros2_nav_bridge_imports() -> None:
    legacy_bridge_imports = {
        "nav.ros2_grid_bridge_module",
        "nav.ros2_path_bridge_module",
        "nav.ros2_waypoint_bridge_module",
    }
    self_file = "src/runtime/tests/test_module_boundaries.py"
    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        if rel == self_file:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError):
            continue
        imports = set(_all_absolute_imports(tree))
        used_legacy_imports = sorted(imports & legacy_bridge_imports)
        if used_legacy_imports:
            violations.append(f"{rel}: imports {', '.join(used_legacy_imports)}")

    assert violations == [], "\n".join(violations)


def test_legacy_ros2_shims_do_not_eagerly_import_compat_modules() -> None:
    import importlib
    import sys

    shims = {
        "localization.bridge": "localization.adapters.ros2.slam_bridge",
        "localization.relocalization": "localization.adapters.ros2.relocalization_service",
        "gateway.rerun_bridge_module": "gateway.visualization.rerun_bridge",
    }
    saved_modules = {
        name: sys.modules.get(name)
        for pair in shims.items()
        for name in pair
        if name in sys.modules
    }
    try:
        for shim, target in shims.items():
            sys.modules.pop(shim, None)
            sys.modules.pop(target, None)

            importlib.import_module(shim)

            assert target not in sys.modules, f"{shim} eagerly imported {target}"
    finally:
        for shim, target in shims.items():
            sys.modules.pop(shim, None)
            sys.modules.pop(target, None)
        sys.modules.update(saved_modules)


def test_legacy_slam_ros2_bridge_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/localization/bridge.py"
    path = ROOT / rel
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "localization.adapters.ros2.slam_bridge" not in imports
    assert "_COMPAT_MODULE = \"localization.adapters.ros2.slam_bridge\"" in text


def test_legacy_slam_relocalization_service_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/localization/relocalization.py"
    path = ROOT / rel
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "localization.adapters.ros2.relocalization_service" not in imports
    assert "_COMPAT_MODULE = \"localization.adapters.ros2.relocalization_service\"" in text
    assert "subprocess.run" not in text
    assert "ros2 service call" not in text


def test_thunder_hardware_package_does_not_import_ros_compat_modules() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_forbidden_hardware_compat_import,
        validate_hardware_compat_boundaries,
    )

    violations, scanned = validate_hardware_compat_boundaries()

    assert scanned > 0
    assert violations == [], "\n".join(violations)
    assert _is_forbidden_hardware_compat_import("runtime.adapters.ros2")


def test_legacy_sim_ros_bridge_files_are_deleted() -> None:
    paths = [
        "src/drivers/sim/compat",
        "src/drivers/sim/ros2_sim_driver.py",
        "src/drivers/sim/mujoco_ros2_bridge.py",
        "src/drivers/sim/mujoco_viz_bridge.py",
        "src/drivers/sim/nova_nav_bridge.py",
    ]

    assert [rel for rel in paths if (ROOT / rel).exists()] == []


def test_exploration_tare_package_has_no_ros2_bridge_shim() -> None:
    assert not (ROOT / "src/nav/exploration/tare/ros2_bridge.py").exists()

    text = (ROOT / "src/nav/exploration/tare/__init__.py").read_text(encoding="utf-8-sig")
    assert "TAREROS2BridgeModule" not in text


def test_legacy_gateway_rerun_bridge_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/gateway/rerun_bridge_module.py"
    path = ROOT / rel
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "gateway.visualization.rerun_bridge" not in imports
    assert "_COMPAT_MODULE = \"gateway.visualization.rerun_bridge\"" in text


def test_legacy_perception_ros2_facade_files_are_removed() -> None:
    legacy_shims = (
        "src/perception/perception_publishers.py",
        "src/perception/reconstruction/bag_reader.py",
    )

    assert [rel for rel in legacy_shims if (ROOT / rel).exists()] == []


def test_runtime_ros2_shutdown_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/lingtu/ros2_shutdown.py"
    path = ROOT / rel
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "runtime.adapters.ros2.context" not in imports
    assert "_COMPAT_MODULE = \"runtime.adapters.ros2.context\"" in text
    assert "import_module" not in text
    assert "rclpy" not in text


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
