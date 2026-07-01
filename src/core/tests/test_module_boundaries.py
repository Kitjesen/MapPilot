from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"

BOUNDARY_RULES = {
    # Existing: each package must NOT import from its forbidden set
    "gateway": {"nav", "semantic", "drivers", "slam"},
    "nav": {"semantic", "drivers", "gateway"},
    "semantic": {"nav", "drivers", "gateway"},
    "drivers": {"nav", "semantic", "gateway"},
    # NEW: core is the lowest layer -- must not import any domain package
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
    # All domain packages must not import each other sideways
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
    # Blueprint composition files — intentionally wire modules across layers
    "core/blueprints/",
    "core/blueprints/stacks/",
    "core/blueprints/full_stack.py",
    "core/blueprints/full_stack_wiring.py",
    "core/blueprints/stub.py",
    "drivers/sim/stub.py",
    "drivers/sim/test_full_pipeline_s100p.py",
    # Core glue files that aggregate sub-package protocols/types
    "core/gateway_runtime_acceptance.py",
    "core/product_field_check.py",
    "core/msgs/scene.py",
    "core/same_source_map_artifacts.py",
    # LingTu user-facing facade — wraps all internal packages
    "lingtu/",
    # Legacy ROS2 launch files with encoding corruption (unparseable)
    "global_planning/pct_planner/launch/",
}


def _python_files(package: str) -> list[Path]:
    return sorted(
        path
        for path in (SRC / package).rglob("*.py")
        if "__pycache__" not in path.parts
        and not any(part.startswith(".") for part in path.parts)
    )


def _imported_modules(tree: ast.AST) -> list[str]:
    """Return top-level absolute import module names.

    Only checks direct children of the module body (skips imports inside
    functions/methods/classes — those are lazy and do not create hard
    cross-package coupling).  Also skips relative imports (``from .foo``)
    which stay within the same package.
    """
    modules: list[str] = []
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
            modules.append(node.module)
    return modules


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
        except SyntaxError:
            # Unparseable file (e.g. legacy ROS2 launch with encoding corruption).
            # The file should be added to COMPOSITION_EXCEPTIONS if intentional.
            continue
        for module in _imported_modules(tree):
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
            if module == "lingtu_runtime" or module.startswith("lingtu_runtime."):
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
        if "core.service_manager" in imports:
            violations.append(f"{rel}: imports core.service_manager")

    assert violations == [], "\n".join(violations)


def test_thunder_driver_blueprints_are_compatibility_shims() -> None:
    path = SRC / "drivers" / "real" / "thunder" / "blueprints.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

    imported = {
        module
        for module in _imported_modules(tree)
        if not module.startswith("typing")
    }
    assert "core.blueprints.products.thunder" in imported
    assert all(_top_level(module) not in {"nav", "semantic", "gateway"} for module in imported)

    forbidden_calls = {"stack_module", "optional_stack_module"}
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert calls.isdisjoint(forbidden_calls)


def test_ros_imports_are_explicit_compat_boundaries() -> None:
    from tools.validate.validate_architecture_boundaries import (
        validate_ros_import_boundaries,
    )

    from compat.ros2.manifest import ROS_COMPAT_IMPORT_BOUNDARIES

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
    assert "compat/ros2/context.py" in allowed_prefixes
    assert "compat/ros2/bag_reader.py" in allowed_prefixes
    assert "compat/ros2/camera_bridge.py" in allowed_prefixes
    assert "compat/ros2/camera_snapshot.py" in allowed_prefixes
    assert "compat/ros2/map_save.py" in allowed_prefixes
    assert "compat/ros2/mujoco_ros2_bridge.py" in allowed_prefixes
    assert "compat/ros2/mujoco_viz_bridge.py" in allowed_prefixes
    assert "compat/ros2/nav/grid_bridge.py" in allowed_prefixes
    assert "compat/ros2/nova_nav_bridge.py" in allowed_prefixes
    assert "compat/ros2/perception_publishers.py" in allowed_prefixes
    assert "compat/ros2/relocalization_service.py" in allowed_prefixes
    assert "compat/ros2/rerun_bridge.py" in allowed_prefixes
    assert "compat/ros2/sim_driver.py" in allowed_prefixes
    assert "compat/ros2/tare_bridge.py" in allowed_prefixes
    assert "drivers/sim/" not in allowed_prefixes
    assert "drivers/sim/mujoco_ros2_bridge.py" not in allowed_prefixes
    assert "drivers/sim/mujoco_viz_bridge.py" not in allowed_prefixes
    assert "drivers/sim/nova_nav_bridge.py" not in allowed_prefixes
    assert "drivers/sim/ros2_sim_driver.py" not in allowed_prefixes
    assert "drivers/real/thunder/camera_bridge_module.py" not in allowed_prefixes
    assert "exploration/tare_explorer_module.py" not in allowed_prefixes
    assert "exploration/tare_ros2_bridge_module.py" not in allowed_prefixes
    assert "gateway/rerun_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_grid_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_path_bridge_module.py" not in allowed_prefixes
    assert "nav/ros2_waypoint_bridge_module.py" not in allowed_prefixes
    assert "semantic/perception/perception_publishers.py" not in allowed_prefixes
    assert "semantic/reconstruction/bag_reader.py" not in allowed_prefixes
    assert "core/ros2_context.py" not in allowed_prefixes
    assert "core/blueprints/" not in allowed_prefixes
    assert "core/runtime/" not in allowed_prefixes
    assert "core/blueprints/products/" not in allowed_prefixes


def test_map_manager_does_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "nav" / "services" / "map_manager_module.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "interface/srv/SaveMaps" not in text
    assert "subprocess.run" not in text
    assert "compat.ros2" not in text


def test_lingtu_slam_facade_does_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "lingtu" / "slam.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "interface/srv/SaveMaps" not in text
    assert "subprocess.run" not in text
    assert "compat.ros2" not in text


def test_gateway_map_routes_do_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "gateway" / "routes" / "maps.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/pgo/save_maps" not in text
    assert "/nav/save_map" not in text
    assert "ros2 service call" not in text
    assert "compat.ros2" not in text


def test_gateway_module_does_not_construct_ros2_map_save_commands() -> None:
    path = SRC / "gateway" / "gateway_module.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "/nav/save_map" not in text
    assert "interface/srv/SaveMaps" not in text
    assert "ros2 service call" not in text
    assert "compat.ros2" not in text


def test_gateway_camera_route_does_not_embed_ros2_snapshot_script() -> None:
    path = SRC / "gateway" / "routes" / "camera.py"
    text = path.read_text(encoding="utf-8-sig")

    assert "rclpy" not in text
    assert "sensor_msgs" not in text
    assert "subprocess.run" not in text
    assert "capture_compressed_camera_snapshot" in text


def test_lcm_imports_stay_in_core_transport_boundary() -> None:
    allowed_paths = {"src/core/transport/lcm.py"}
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
    from compat.ros2.manifest import (
        ROS_COMPAT_IMPORT_BOUNDARIES,
        ROS_IMPORT_ROOTS,
        ros_compat_boundary_for,
    )

    prefixes = [boundary.prefix for boundary in ROS_COMPAT_IMPORT_BOUNDARIES]

    assert len(prefixes) == len(set(prefixes))
    assert list(prefixes) == sorted(prefixes)
    assert "rclpy" in ROS_IMPORT_ROOTS
    assert ros_compat_boundary_for("compat/ros2/slam_bridge.py").category == "slam_ros2_bridge"
    assert ros_compat_boundary_for("compat/ros2/map_save.py").category == "map_save_ros2_adapter"
    assert (
        ros_compat_boundary_for("compat/ros2/relocalization_service.py").category
        == "relocalization_ros2_adapter"
    )
    assert ros_compat_boundary_for("slam/slam_bridge_module.py") is None
    assert ros_compat_boundary_for("core/blueprints/products/thunder.py") is None


def test_core_has_no_ros2_context_compatibility_proxy() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_forbidden_core_compat_import,
        validate_core_compat_boundaries,
    )

    violations, scanned = validate_core_compat_boundaries()

    assert not (SRC / "core" / "ros2_context.py").exists()
    assert scanned > 0
    assert violations == [], "\n".join(violations)
    assert _is_forbidden_core_compat_import("compat.ros2")
    assert _is_forbidden_core_compat_import("compat.ros2.context")
    assert not _is_forbidden_core_compat_import("compat.lcm.endpoint_runner")


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


def test_navigation_module_constructor_surface_is_endpoint_neutral() -> None:
    import inspect

    from nav.navigation_module import NavigationModule

    signature = inspect.signature(NavigationModule)
    nav = NavigationModule(enable_ros2_bridge=False)

    assert "enable_ros2_bridge" not in signature.parameters
    assert "enable_ros2_bridge" not in nav._config
    assert nav._legacy_enable_ros2_bridge is False


def test_legacy_nav_ros2_bridge_files_are_compatibility_shims() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    expected_targets = {
        "src/nav/ros2_grid_bridge_module.py": "compat.ros2.nav.grid_bridge",
        "src/nav/ros2_path_bridge_module.py": "compat.ros2.nav.path_bridge",
        "src/nav/ros2_waypoint_bridge_module.py": "compat.ros2.nav.waypoint_bridge",
    }

    violations: list[str] = []
    for rel, expected_target in expected_targets.items():
        path = ROOT / rel
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        imports = set(_iter_imports(tree))
        ros_imports = sorted(module for module in imports if _is_ros_import(module))
        if ros_imports:
            violations.append(f"{rel}: imports ROS modules {', '.join(ros_imports)}")
        if expected_target not in imports:
            violations.append(f"{rel}: missing compat target {expected_target}")

    assert violations == [], "\n".join(violations)


def test_internal_code_uses_compat_ros2_nav_bridge_imports() -> None:
    legacy_bridge_imports = {
        "nav.ros2_grid_bridge_module",
        "nav.ros2_path_bridge_module",
        "nav.ros2_waypoint_bridge_module",
    }
    shim_files = {
        "src/nav/ros2_grid_bridge_module.py",
        "src/nav/ros2_path_bridge_module.py",
        "src/nav/ros2_waypoint_bridge_module.py",
    }
    self_file = "src/core/tests/test_module_boundaries.py"
    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        if rel in shim_files or rel == self_file:
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


def test_legacy_slam_ros2_bridge_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/slam/slam_bridge_module.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.slam_bridge" in imports


def test_legacy_slam_relocalization_service_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/slam/relocalization_service.py"
    path = ROOT / rel
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.relocalization_service" in imports
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
    assert _is_forbidden_hardware_compat_import("compat.ros2")
    assert _is_forbidden_hardware_compat_import("compat.ros2.camera_bridge")
    assert not _is_forbidden_hardware_compat_import("compat.lcm.sources.thunder_brainstem")


def test_legacy_ros2_sim_driver_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/drivers/sim/ros2_sim_driver.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.sim_driver" in imports


def test_legacy_sim_ros_bridge_files_are_compatibility_shims() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    expected_targets = {
        "src/drivers/sim/mujoco_ros2_bridge.py": "compat.ros2.mujoco_ros2_bridge",
        "src/drivers/sim/mujoco_viz_bridge.py": "compat.ros2.mujoco_viz_bridge",
        "src/drivers/sim/nova_nav_bridge.py": "compat.ros2.nova_nav_bridge",
    }

    violations: list[str] = []
    for rel, expected_target in expected_targets.items():
        path = ROOT / rel
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        imports = set(_iter_imports(tree))
        ros_imports = sorted(module for module in imports if _is_ros_import(module))
        if ros_imports:
            violations.append(f"{rel}: imports ROS modules {', '.join(ros_imports)}")
        if expected_target not in imports:
            violations.append(f"{rel}: missing compat target {expected_target}")

    assert violations == [], "\n".join(violations)


def test_legacy_tare_ros2_bridge_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/exploration/tare_ros2_bridge_module.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.tare_bridge" in imports


def test_legacy_gateway_rerun_bridge_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/gateway/rerun_bridge_module.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.rerun_bridge" in imports


def test_legacy_perception_publishers_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/semantic/perception/perception_publishers.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.perception_publishers" in imports


def test_legacy_reconstruction_bag_reader_file_is_compatibility_shim() -> None:
    from tools.validate.validate_architecture_boundaries import _is_ros_import, _iter_imports

    rel = "src/semantic/reconstruction/bag_reader.py"
    path = ROOT / rel
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports = set(_iter_imports(tree))
    ros_imports = sorted(module for module in imports if _is_ros_import(module))

    assert ros_imports == []
    assert "compat.ros2.bag_reader" in imports


def test_production_ros_bridges_use_compat_context_import_path() -> None:
    from tools.validate.validate_architecture_boundaries import (
        _is_ros_scan_excluded,
        _is_test_or_example,
        _iter_imports,
    )

    violations: list[str] = []

    for path in SRC.rglob("*.py"):
        if (
            "__pycache__" in path.parts
            or _is_test_or_example(path)
            or _is_ros_scan_excluded(path)
        ):
            continue
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        if "core.ros2_context" in set(_iter_imports(tree)):
            violations.append(f"{rel}: imports core.ros2_context")

    assert violations == [], "\n".join(violations)
