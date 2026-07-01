from __future__ import annotations

import ast
from pathlib import Path

from tools.validate.validate_architecture_boundaries import validate


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"

CORE_RUNTIME_BOUNDARY_SOURCES = {
    "runtime_interface": SRC / "runtime" / "runtime_interface.py",
    "runtime_policy": SRC / "runtime" / "runtime_policy.py",
    "runtime_switch": SRC / "runtime" / "runtime_switch.py",
    "runtime_resolver": SRC / "runtime" / "profiles" / "resolver.py",
    "runtime_binding_policy": SRC / "runtime" / "profiles" / "binding_policy.py",
    "runtime_endpoint_config": SRC / "runtime" / "profiles" / "endpoint_config.py",
    "runtime_catalog": SRC / "runtime" / "profiles" / "catalog",
    "runtime_endpoints": SRC / "runtime" / "profiles" / "endpoints.py",
    "runtime_endpoint_compat_facade": SRC / "runtime" / "blueprints" / "runtime_endpoint.py",
}

RUNTIME_MODEL_OWNER_SOURCES = {
    "runtime_package": SRC / "runtime" / "profiles" / "__init__.py",
    "runtime_resolver": SRC / "runtime" / "profiles" / "resolver.py",
    "runtime_binding_policy": SRC / "runtime" / "profiles" / "binding_policy.py",
    "runtime_endpoint_config": SRC / "runtime" / "profiles" / "endpoint_config.py",
    "runtime_endpoints": SRC / "runtime" / "profiles" / "endpoints.py",
    "runtime_catalog_package": SRC / "runtime" / "profiles" / "catalog" / "__init__.py",
    "runtime_catalog_endpoints": SRC / "runtime" / "profiles" / "catalog" / "endpoints.py",
    "runtime_catalog_endpoint_adapter_configs": SRC
    / "runtime"
    / "profiles"
    / "catalog"
    / "endpoint_adapter_configs.py",
    "runtime_catalog_products": SRC / "runtime" / "profiles" / "catalog" / "products.py",
    "runtime_catalog_product_intents": SRC / "runtime" / "profiles" / "catalog" / "product_intents.py",
    "runtime_catalog_robots": SRC / "runtime" / "profiles" / "catalog" / "robots.py",
    "runtime_catalog_robot_runtime_defaults": SRC
    / "runtime"
    / "profiles"
    / "catalog"
    / "robot_runtime_defaults.py",
    "runtime_catalog_robot_archives": SRC
    / "runtime"
    / "profiles"
    / "catalog"
    / "robot_archives.py",
    "runtime_catalog_runtime_paths": SRC / "runtime" / "profiles" / "catalog" / "runtime_paths.py",
    "runtime_catalog_simulation_profiles": SRC / "runtime" / "profiles" / "catalog" / "simulation_profiles.py",
}

ALGORITHM_KERNEL_SEAM_SOURCES = {
    "nav_kernel_package": SRC / "nav" / "kernel" / "__init__.py",
    "nav_kernel_paths": SRC / "nav" / "kernel" / "paths.py",
    "nav_kernel_runtime": SRC / "nav" / "kernel" / "runtime.py",
    "nav_kernel_cmake": SRC / "nav" / "kernel" / "CMakeLists.txt",
}

LCM_EXTERNAL_ADAPTER_SOURCES = {
    "lcm_external_adapter_package": SRC / "runtime" / "adapters" / "lcm" / "__init__.py",
    "lcm_localization_adapter": SRC
    / "runtime"
    / "adapters"
    / "lcm"
    / "localization_adapter.py",
    "lcm_nav_input_adapter": SRC
    / "runtime"
    / "adapters"
    / "lcm"
    / "nav_input.py",
    "lcm_nav_output_adapter": SRC
    / "runtime"
    / "adapters"
    / "lcm"
    / "nav_output.py",
}

BASE_AUTONOMY_BACKEND_ADAPTER_SOURCES = {
    "path_follower_backend": SRC / "nav" / "local" / "path_follower_backend.py",
    "local_planner_backend": SRC
    / "nav"
    / "services"
    / "plan"
    / "local_planner"
    / "backend.py",
    "terrain_backend": SRC / "nav" / "local" / "terrain_backend.py",
}

BASE_AUTONOMY_BACKEND_RUNTIME_SOURCES = {
    "path_follower_runtime": SRC / "nav" / "local" / "path_follower_runtime.py",
    "local_planner_runtime": SRC
    / "nav"
    / "services"
    / "plan"
    / "local_planner"
    / "runtime.py",
}

BASE_AUTONOMY_MODULE_SOURCES = {
    "path_follower": SRC / "nav" / "local" / "path_follower.py",
    "local_planner": SRC
    / "nav"
    / "services"
    / "plan"
    / "local_planner"
    / "service.py",
    "terrain": SRC / "nav" / "local" / "terrain.py",
}

BLUEPRINT_COMPAT_ADAPTER_SOURCES = {
    "driver_runtime": SRC
    / "runtime"
    / "blueprints"
    / "adapters"
    / "driver_runtime.py",
    "mapping_slam": SRC
    / "runtime"
    / "blueprints"
    / "adapters"
    / "mapping_slam.py",
    "navigation_io": SRC
    / "runtime"
    / "blueprints"
    / "adapters"
    / "navigation_io.py",
    "perception_gateway": SRC
    / "runtime"
    / "blueprints"
    / "adapters"
    / "perception_gateway.py",
}

STACK_COMPOSITION_SOURCES_WITH_ADAPTER_BOUNDARY = {
    "driver_stack": SRC / "runtime" / "blueprints" / "stacks" / "driver.py",
    "gateway_stack": SRC / "runtime" / "blueprints" / "stacks" / "gateway.py",
    "maps_stack": SRC / "runtime" / "blueprints" / "stacks" / "maps.py",
    "navigation_stack": SRC / "runtime" / "blueprints" / "stacks" / "navigation.py",
    "perception_stack": SRC / "runtime" / "blueprints" / "stacks" / "perception.py",
    "slam_stack": SRC / "runtime" / "blueprints" / "stacks" / "localization.py",
}

EXPLICIT_COMPATIBILITY_FACADES = {
    SRC / "runtime" / "blueprints" / "runtime_endpoint.py": (
        "runtime.profiles.endpoints",
    ),
    SRC / "runtime" / "blueprints" / "catalog" / "endpoints.py": (
        "runtime.profiles.catalog.endpoints",
    ),
}

CANONICAL_COMPATIBILITY_TARGETS = (
    SRC / "nav" / "adapters" / "ros2" / "tare_bridge.py",
)

REMOVED_COMPATIBILITY_FACADE_PATHS = (
    SRC / "exploration" / "tare" / "ros2_bridge.py",
    SRC / "semantic" / "perception" / "perception_publishers.py",
    SRC / "semantic" / "reconstruction" / "bag_reader.py",
    SRC / "perception" / "perception_publishers.py",
    SRC / "perception" / "reconstruction" / "bag_reader.py",
)


def _repo_rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _assert_sources_exist_under_owner(sources: dict[str, Path], owner: Path) -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in sources.items()
        if not path.exists()
    ]
    assert missing == [], "Missing layer boundary source(s):\n" + "\n".join(missing)

    owner_rel = _repo_rel(owner)
    for name, path in sources.items():
        rel = _repo_rel(path)
        assert path == owner or owner in path.parents, (
            f"{name} moved outside {owner_rel} ownership boundary: {rel}"
        )
        assert "/tests/" not in rel and not rel.startswith("tests/"), (
            f"{name} must be production boundary code, not test-only code: {rel}"
        )


def test_module_first_architecture_validator_is_contract_entrypoint() -> None:
    violations, scanned = validate()

    assert scanned > 0
    assert violations == [], "\n".join(violations)


def test_runtime_contract_imports_through_core_boundary() -> None:
    from runtime.runtime_interface import (
        REAL_RUNTIME_CONTRACT,
        runtime_contract_manifest,
        runtime_data_flow_topics,
    )

    manifest = runtime_contract_manifest()

    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert REAL_RUNTIME_CONTRACT in manifest["resolved_runtime_data_flow"]
    assert tuple(manifest["runtime_data_flow_topics"][REAL_RUNTIME_CONTRACT]) == runtime_data_flow_topics(
        REAL_RUNTIME_CONTRACT
    )


def test_runtime_boundary_sources_remain_core_owned() -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in CORE_RUNTIME_BOUNDARY_SOURCES.items()
        if not path.exists()
    ]
    assert missing == [], "Missing runtime boundary source(s):\n" + "\n".join(missing)

    for name, path in CORE_RUNTIME_BOUNDARY_SOURCES.items():
        rel = _repo_rel(path)
        assert rel.startswith("src/runtime/"), f"{name} moved outside core boundary: {rel}"
        assert "/tests/" not in rel and not rel.startswith("tests/"), (
            f"{name} must be production core boundary code, not test-only code: {rel}"
        )


def test_directory_stage_keeps_runtime_model_under_core_runtime() -> None:
    _assert_sources_exist_under_owner(
        RUNTIME_MODEL_OWNER_SOURCES,
        SRC / "runtime" / "profiles",
    )


def test_resolver_endpoint_layer_uses_runtime_endpoint_config_helper() -> None:
    resolver_source = (SRC / "runtime" / "profiles" / "resolver.py").read_text(
        encoding="utf-8"
    )
    endpoint_source = (SRC / "runtime" / "profiles" / "endpoints.py").read_text(
        encoding="utf-8"
    )
    catalog_source = (
        SRC / "runtime" / "profiles" / "catalog" / "endpoints.py"
    ).read_text(encoding="utf-8")

    assert "from runtime.profiles.endpoint_config import" in resolver_source
    assert "endpoint_config_for_profile(" in resolver_source
    assert "merge_runtime_endpoint_config(" in resolver_source
    assert "apply_runtime_endpoint_config(" not in resolver_source

    assert "from runtime.profiles.endpoint_config import" in endpoint_source
    assert "endpoint_config_for_profile(endpoint, profile)" in endpoint_source
    assert "merge_runtime_endpoint_config(config, endpoint_config)" in endpoint_source

    assert "endpoint_config_for_profile(self, profile)" in catalog_source
    assert 'merged["_runtime_endpoint"]' not in catalog_source


def test_runtime_shared_defaults_are_not_duplicated_outside_runtime_paths() -> None:
    default_owner = SRC / "runtime" / "profiles" / "catalog" / "runtime_paths.py"
    forbidden_sources = {
        "product_intents": SRC / "runtime" / "profiles" / "catalog" / "product_intents.py",
        "simulation_profiles": SRC
        / "runtime"
        / "profiles"
        / "catalog"
        / "simulation_profiles.py",
        "endpoint_adapter_configs": SRC
        / "runtime"
        / "profiles"
        / "catalog"
        / "endpoint_adapter_configs.py",
        "endpoints": SRC / "runtime" / "profiles" / "catalog" / "endpoints.py",
    }
    forbidden_literals = {
        5050,
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle",
    }

    assert default_owner.exists()

    offenders: list[str] = []
    for name, path in forbidden_sources.items():
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and node.value in forbidden_literals:
                offenders.append(f"{name}:{node.lineno}: {node.value!r}")

    assert offenders == []


def test_directory_stage_keeps_nav_kernel_seam_under_nav_kernel() -> None:
    _assert_sources_exist_under_owner(
        ALGORITHM_KERNEL_SEAM_SOURCES,
        SRC / "nav" / "kernel",
    )


def test_directory_stage_keeps_lcm_adapter_layer_under_runtime_adapters() -> None:
    _assert_sources_exist_under_owner(
        LCM_EXTERNAL_ADAPTER_SOURCES,
        SRC / "runtime" / "adapters" / "lcm",
    )


def test_base_autonomy_backend_adapter_helpers_exist_beside_modules() -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in BASE_AUTONOMY_BACKEND_ADAPTER_SOURCES.items()
        if not path.exists()
    ]

    assert missing == [], "Missing nav.local backend adapter helper(s):\n" + "\n".join(missing)


def test_base_autonomy_backend_runtime_helpers_exist_beside_modules() -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in BASE_AUTONOMY_BACKEND_RUNTIME_SOURCES.items()
        if not path.exists()
    ]

    assert missing == [], "Missing nav.local backend runtime helper(s):\n" + "\n".join(missing)


def test_base_autonomy_modules_do_not_import_legacy_nav_kernel_loader() -> None:
    direct_loader_import = "from nav.local._nav_kernel_loader import"
    offenders = [
        _repo_rel(path)
        for path in BASE_AUTONOMY_MODULE_SOURCES.values()
        if direct_loader_import in path.read_text(encoding="utf-8")
    ]

    assert offenders == [], (
        "Module classes must keep ports/message handling only and route backend "
        "selection through backend helper interfaces:\n"
        + "\n".join(offenders)
    )


def test_base_autonomy_backend_adapters_route_nav_kernel_through_kernel_seam() -> None:
    kernel_runtime_import = "from nav.kernel import"
    missing_kernel_boundary = [
        _repo_rel(path)
        for path in BASE_AUTONOMY_BACKEND_ADAPTER_SOURCES.values()
        if path.exists() and kernel_runtime_import not in path.read_text(encoding="utf-8")
    ]

    assert missing_kernel_boundary == [], (
        "Backend adapter helpers must use the nav.kernel runtime seam:\n"
        + "\n".join(missing_kernel_boundary)
    )


def test_base_autonomy_modules_delegate_backend_setup_to_runtime_helpers() -> None:
    expected_runtime_calls = {
        "local_planner": "setup_local_planner_backend",
        "path_follower": "setup_path_follower_runtime",
    }
    forbidden_factory_calls = {
        "create_nanobind_backend(",
        "create_cmu_native_module(",
        "create_cmu_py_backend(",
        "create_nav_kernel_path_follower_adapter_from_tuning(",
        "create_pure_pursuit_native_adapter(",
        "read_pid_fallback_params(",
    }
    offenders: list[str] = []

    for name, expected_call in expected_runtime_calls.items():
        path = BASE_AUTONOMY_MODULE_SOURCES[name]
        source = path.read_text(encoding="utf-8")
        if expected_call not in source:
            offenders.append(f"{_repo_rel(path)} missing {expected_call}")
        for forbidden in forbidden_factory_calls:
            if forbidden in source:
                offenders.append(f"{_repo_rel(path)} contains {forbidden}")

    assert offenders == [], (
        "Base autonomy Modules must keep ports/lifecycle separate from backend "
        "adapter setup:\n" + "\n".join(offenders)
    )


def test_removed_module_shim_import_paths_stay_removed() -> None:
    scan_roots = (
        ROOT / "cli",
        ROOT / "scripts",
        ROOT / "sim",
        ROOT / "src",
        ROOT / "tests",
        ROOT / "tools",
    )
    forbidden = (
        "nav.local.local_planner_module",
        "nav.navigation_module",
        "nav.global_planner_service",
        "nav.cmd_vel_mux_module",
        "nav.safety_ring_module",
        "nav.waypoint_tracker",
        "nav.frontier_explorer_module",
        "nav.plan_safety",
        "nav.elevation_map_module",
        "nav.esdf_module",
        "nav.occupancy_grid_module",
        "nav.traversability_cost_module",
        "nav.voxel_grid_module",
    )
    offenders: list[str] = []

    for root in scan_roots:
        if not root.exists():
            continue
        for path in root.rglob("*.py"):
            if path == Path(__file__).resolve():
                continue
            source = path.read_text(encoding="utf-8", errors="ignore")
            for token in forbidden:
                if token in source:
                    offenders.append(f"{_repo_rel(path)} contains {token}")

    assert offenders == [], "Removed shim import path(s) found:\n" + "\n".join(offenders)


def test_compat_adapter_helpers_live_in_blueprint_adapter_layer() -> None:
    _assert_sources_exist_under_owner(
        BLUEPRINT_COMPAT_ADAPTER_SOURCES,
        SRC / "runtime" / "blueprints" / "adapters",
    )


def test_stack_composition_files_do_not_own_ros_fallback_strings() -> None:
    forbidden_marker = "runtime.adapters.ros2"
    offenders = [
        _repo_rel(path)
        for path in STACK_COMPOSITION_SOURCES_WITH_ADAPTER_BOUNDARY.values()
        if path.exists() and forbidden_marker in path.read_text(encoding="utf-8")
    ]

    assert offenders == [], (
        "Stack composition files must call explicit adapter helpers instead of "
        "owning ROS fallback strings:\n"
        + "\n".join(offenders)
    )


def test_old_compatibility_facades_remain_explicit_forwarders() -> None:
    assert [
        _repo_rel(path)
        for path in REMOVED_COMPATIBILITY_FACADE_PATHS
        if path.exists()
    ] == []
    assert [
        _repo_rel(path)
        for path in CANONICAL_COMPATIBILITY_TARGETS
        if not path.exists()
    ] == []

    missing = [
        _repo_rel(path)
        for path in EXPLICIT_COMPATIBILITY_FACADES
        if not path.exists()
    ]
    assert missing == [], "Missing compatibility facade(s):\n" + "\n".join(missing)

    for path, targets in EXPLICIT_COMPATIBILITY_FACADES.items():
        source = path.read_text(encoding="utf-8")
        assert "Compatibility" in source, (
            f"{_repo_rel(path)} must identify itself as an explicit compatibility facade"
        )
        for target in targets:
            has_explicit_import = f"from {target} import" in source
            has_lazy_target = f'_COMPAT_MODULE = "{target}"' in source
            assert has_explicit_import or has_lazy_target, (
                f"{_repo_rel(path)} must forward to canonical owner {target}"
            )
