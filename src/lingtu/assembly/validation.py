"""Cross-check LingTu product assembly against Runtime Graph contracts."""

from __future__ import annotations

from typing import Any

from runtime.graph.loader import RuntimeGraph, load_runtime_graph
from runtime.graph.validator import RuntimeGraphIssue

from .graph import graph_for_profile

LEGACY_MODULE_SIM_PROFILES = frozenset({"sim_mujoco_live", "sim_mujoco_octo_live"})


def validate_profile(
    profile: str,
    graph: RuntimeGraph | None = None,
    *,
    config: dict[str, Any] | None = None,
    module_names: tuple[str, ...] | None = None,
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
) -> list[RuntimeGraphIssue]:
    from runtime.profiles.resolver import resolve_profile_config

    resolved_config = config if config is not None else resolve_profile_config(profile)
    modules = set(module_names if module_names is not None else graph_for_profile(profile).modules)
    product = graph.products.get(profile, {})
    endpoint = graph.endpoints.get("thunder_field", {})
    forbidden_modules = set(_string_tuple(product.get("forbidden_modules"))) | set(
        _string_tuple(endpoint.get("forbidden_modules"))
    )
    issues: list[RuntimeGraphIssue] = []

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
