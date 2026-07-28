"""Cross-check LingTu product assembly against Runtime Graph contracts."""

from __future__ import annotations

from typing import Any

from runtime.graph import ProcessSpec
from runtime.graph.loader import RuntimeGraph, load_runtime_graph
from runtime.graph.validator import RuntimeGraphIssue

LEGACY_MODULE_SIM_PROFILES = frozenset({"sim_mujoco_live", "sim_mujoco_octo_live"})
MAPD_HOST_CONFLICT_MODULES = frozenset(
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
    *,
    config: dict[str, Any] | None = None,
    module_names: tuple[str, ...] | None = None,
    endpoint_name: str | None = None,
    processes: tuple[ProcessSpec, ...] = (),
) -> list[RuntimeGraphIssue]:
    """Validate one profile across product assembly and process contracts."""

    graph = graph or load_runtime_graph()
    profile_name = str(profile)
    if profile_name in graph.products:
        return _validate_real_product_profile(
            profile_name,
            graph,
            config=config,
            module_names=module_names,
            endpoint_name=endpoint_name,
            processes=processes,
        )
    if profile_name in LEGACY_MODULE_SIM_PROFILES:
        return _validate_legacy_module_sim_profile(profile_name)
    return []


def _validate_real_product_profile(
    profile: str,
    graph: RuntimeGraph,
    *,
    config: dict[str, Any] | None,
    module_names: tuple[str, ...] | None,
    endpoint_name: str | None,
    processes: tuple[ProcessSpec, ...],
) -> list[RuntimeGraphIssue]:
    from runtime.profiles.resolver import resolve_profile_config

    resolved_config = config if config is not None else resolve_profile_config(profile)
    if module_names is None:
        from .graph import graph_for_profile

        module_names = graph_for_profile(profile).modules
    modules = set(module_names)
    product = graph.products.get(profile, {})
    endpoint = graph.endpoints.get(endpoint_name or "thunder_field", {})
    forbidden_modules = set(_string_tuple(product.get("forbidden_modules"))) | set(
        _string_tuple(endpoint.get("forbidden_modules"))
    )
    issues: list[RuntimeGraphIssue] = []
    issues.extend(
        _validate_mapd_single_owner(
            profile,
            endpoint_name=endpoint_name,
            config=resolved_config,
            modules=modules,
            processes=processes,
        )
    )

    expected = (
        ("_runtime_endpoint", "thunder_field", "real_profile_endpoint_drift", "use thunder_field endpoint"),
        (
            "localization_adapter",
            "cpp_slam_status",
            "real_profile_localization_adapter_drift",
            "use cpp_slam_status localization_adapter",
        ),
        (
            "command_output_mode",
            "endpoint_only",
            "real_profile_command_output_drift",
            "use endpoint_only command output",
        ),
        (
            "hardware_control_boundary",
            "driver",
            "real_profile_driver_boundary_drift",
            "use the canonical driver hardware boundary",
        ),
    )
    for key, value, code, requirement in expected:
        if resolved_config.get(key) != value:
            issues.append(_issue(code, f"{profile} must {requirement}", profile))

    if resolved_config.get("enable_robot_driver") is not False:
        issues.append(
            _issue(
                "real_profile_duplicate_driver",
                f"{profile} must not add the Python robot driver beside the native driver",
                profile,
            )
        )

    required_modules = set(_string_tuple(product.get("critical_modules")))
    missing_required = sorted(required_modules - modules)
    if missing_required:
        issues.append(
            _issue(
                "real_profile_critical_module_missing",
                f"{profile} is missing critical modules: {', '.join(missing_required)}",
                profile,
            )
        )

    forbidden_present = sorted(forbidden_modules & modules)
    if forbidden_present:
        issues.append(
            _issue(
                "real_profile_forbidden_module",
                f"{profile} includes forbidden modules: {', '.join(forbidden_present)}",
                profile,
            )
        )
    return issues


def _validate_mapd_single_owner(
    profile: str,
    *,
    endpoint_name: str | None,
    config: dict[str, Any],
    modules: set[str],
    processes: tuple[ProcessSpec, ...],
) -> list[RuntimeGraphIssue]:
    if endpoint_name != "thunder_field" or not any(
        process.name == "maps" and process.target == "mapd.service"
        for process in processes
    ):
        return []

    issues: list[RuntimeGraphIssue] = []
    if config.get("enable_map_layers") is not False:
        issues.append(
            _issue(
                "real_profile_mapd_layers_enabled",
                (
                    f"{profile} selects maps process mapd.service on thunder_field, so "
                    "enable_map_layers must be exactly False to keep one live map-layer owner"
                ),
                profile,
            )
        )

    conflicts = sorted(MAPD_HOST_CONFLICT_MODULES & modules)
    if conflicts:
        issues.append(
            _issue(
                "real_profile_mapd_module_conflict",
                (
                    f"{profile} selects maps process mapd.service on thunder_field but the Host Blueprint "
                    f"includes conflicting live map modules: {', '.join(conflicts)}; "
                    "maps.service remains allowed as the low-rate control facade"
                ),
                profile,
            )
        )
    return issues


def _validate_legacy_module_sim_profile(profile: str) -> list[RuntimeGraphIssue]:
    from runtime.profiles.catalog.products import PROFILES

    config = PROFILES.get(profile, {})
    issues: list[RuntimeGraphIssue] = []
    if config.get("_runtime_graph_role") != "module_sim_harness":
        issues.append(
            _issue(
                "legacy_module_sim_role_missing",
                f"{profile} must be marked _runtime_graph_role=module_sim_harness",
                profile,
            )
        )
    if config.get("_real_equivalent") is not False:
        issues.append(
            _issue(
                "legacy_module_sim_real_equivalent_flag",
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


def _issue(code: str, message: str, profile: str) -> RuntimeGraphIssue:
    return RuntimeGraphIssue(code=code, message=message, scope=f"profile:{profile}")
