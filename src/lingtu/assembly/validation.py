"""Cross-check LingTu product assembly against Runtime Graph contracts."""

from __future__ import annotations

from typing import Any

from runtime.graph import ProcessSpec, resolve_env_implementation
from runtime.graph.loader import RuntimeGraph, load_runtime_graph
from runtime.graph.validator import RuntimeGraphIssue

HOST_SIMULATION_PROFILES = frozenset({"sim_mujoco_live", "sim_mujoco_octo_live"})
NATIVE_MAPS_HOST_CONFLICT_MODULES = frozenset(
    {
        "OccupancyGridModule",
        "VoxelGridModule",
        "SemanticMapModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "map.out",
    }
)


def validate_profile(
    profile: str,
    graph: RuntimeGraph | None = None,
) -> list[RuntimeGraphIssue]:
    """Validate one local Host Profile contract."""

    graph = graph or load_runtime_graph()
    profile_name = str(profile)
    if profile_name in graph.products:
        raise ValueError(
            f"{profile_name!r} is a Product; use validate_product(...)"
        )
    if profile_name in HOST_SIMULATION_PROFILES:
        return _validate_host_simulation_profile(profile_name)
    return []


def validate_product(
    product: str,
    graph: RuntimeGraph | None = None,
    *,
    config: dict[str, Any] | None = None,
    module_names: tuple[str, ...] | None = None,
    env_name: str | None = None,
    env_config: dict[str, Any] | None = None,
    processes: tuple[ProcessSpec, ...] = (),
) -> list[RuntimeGraphIssue]:
    """Validate one field Product across Host and process contracts."""

    graph = graph or load_runtime_graph()
    product_name = str(product)
    if product_name not in graph.products:
        raise ValueError(
            f"{product_name!r} is not a Product; use validate_profile(...)"
        )
    return _validate_product(
        product_name,
        graph,
        config=config,
        module_names=module_names,
        env_name=env_name,
        env_config=env_config,
        processes=processes,
    )


def _validate_product(
    product_name: str,
    graph: RuntimeGraph,
    *,
    config: dict[str, Any] | None,
    module_names: tuple[str, ...] | None,
    env_name: str | None,
    env_config: dict[str, Any] | None,
    processes: tuple[ProcessSpec, ...],
) -> list[RuntimeGraphIssue]:
    from lingtu.assembly.products import resolve_product_host_config

    selected_env = str(env_name or (config or {}).get("_env") or "real").strip()
    selected_env_config = env_config
    if selected_env_config is None and selected_env == "sim":
        backend = str((config or {}).get("_env_backend") or "").strip()
        selected_env_config = {"backend": backend} if backend else None
    resolved_config = (
        config
        if config is not None
        else resolve_product_host_config(
            product_name,
            selected_env,
            env_config=selected_env_config,
        )
    )
    if module_names is None:
        from .profile_builder import blueprint_for_resolved_product

        module_names = blueprint_for_resolved_product(
            product_name,
            resolved_config,
        ).module_names
    modules = set(module_names)
    product = graph.products[product_name]
    implementation = resolve_env_implementation(
        selected_env,
        graph=graph,
        env_config=selected_env_config,
    )
    forbidden_modules = set(_string_tuple(implementation.get("forbidden_modules")))
    if selected_env == "real":
        forbidden_modules.update(_string_tuple(product.get("forbidden_modules")))
    issues: list[RuntimeGraphIssue] = []
    issues.extend(
        _validate_native_maps_single_owner(
            product_name,
            config=resolved_config,
            modules=modules,
            processes=processes,
        )
    )

    expected = (
        ("_env", "real", "real_product_env_drift", "use the real Env"),
        (
            "localization_adapter",
            "cpp_slam_status",
            "real_product_localization_adapter_drift",
            "use cpp_slam_status localization_adapter",
        ),
        (
            "command_output_mode",
            "endpoint_only",
            "real_product_command_output_drift",
            "use endpoint_only command output",
        ),
        (
            "hardware_control_boundary",
            "driver",
            "real_product_driver_boundary_drift",
            "use the canonical driver hardware boundary",
        ),
    )
    if selected_env == "real":
        for key, value, code, requirement in expected:
            if resolved_config.get(key) != value:
                issues.append(
                    _product_issue(
                        code,
                        f"Product {product_name} must {requirement}",
                        product_name,
                    )
                )

        if resolved_config.get("enable_robot_driver") is not False:
            issues.append(
                _product_issue(
                    "real_product_duplicate_driver",
                    f"Product {product_name} must not add the Python robot driver beside the native driver",
                    product_name,
                )
            )

    required_modules = set(
        _string_tuple(
            product.get("critical_modules")
            if selected_env == "real"
            else implementation.get("critical_modules")
        )
    )
    missing_required = sorted(required_modules - modules)
    if missing_required:
        issues.append(
            _product_issue(
                f"{selected_env}_product_critical_module_missing",
                f"Product {product_name} is missing critical modules: {', '.join(missing_required)}",
                product_name,
            )
        )

    forbidden_present = sorted(forbidden_modules & modules)
    if forbidden_present:
        issues.append(
            _product_issue(
                f"{selected_env}_product_forbidden_module",
                f"Product {product_name} includes forbidden modules: {', '.join(forbidden_present)}",
                product_name,
            )
        )
    return issues


def _validate_native_maps_single_owner(
    product: str,
    *,
    config: dict[str, Any],
    modules: set[str],
    processes: tuple[ProcessSpec, ...],
) -> list[RuntimeGraphIssue]:
    if not any(process.name == "maps" for process in processes):
        return []

    issues: list[RuntimeGraphIssue] = []
    if config.get("enable_map_layers") is not False:
        issues.append(
            _product_issue(
                "native_maps_layers_enabled",
                (
                    f"Product {product} selects the native maps process, so "
                    "enable_map_layers must be exactly False to keep one live map-layer owner"
                ),
                product,
            )
        )

    conflicts = sorted(NATIVE_MAPS_HOST_CONFLICT_MODULES & modules)
    if conflicts:
        issues.append(
            _product_issue(
                "native_maps_module_conflict",
                (
                    f"Product {product} selects the native maps process but the Host Blueprint "
                    f"includes conflicting live map modules: {', '.join(conflicts)}; "
                    "maps.service remains allowed as the low-rate control facade"
                ),
                product,
            )
        )
    return issues


def _validate_host_simulation_profile(profile: str) -> list[RuntimeGraphIssue]:
    from runtime.profiles.catalog.host_defaults import HOST_PROFILE_DEFAULTS

    config = HOST_PROFILE_DEFAULTS.get(profile, {})
    issues: list[RuntimeGraphIssue] = []
    if config.get("_runtime_graph_role") != "host_simulation":
        issues.append(
            _profile_issue(
                "host_simulation_role_missing",
                f"{profile} must be marked _runtime_graph_role=host_simulation",
                profile,
            )
        )
    if config.get("_real_equivalent") is not False:
        issues.append(
            _profile_issue(
                "host_simulation_real_equivalent_flag",
                f"{profile} must be marked _real_equivalent=False",
                profile,
            )
        )
    return issues


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set):
        return tuple(str(item) for item in value)
    return ()


def _profile_issue(code: str, message: str, profile: str) -> RuntimeGraphIssue:
    return RuntimeGraphIssue(code=code, message=message, scope=f"profile:{profile}")


def _product_issue(code: str, message: str, product: str) -> RuntimeGraphIssue:
    return RuntimeGraphIssue(code=code, message=message, scope=f"product:{product}")
