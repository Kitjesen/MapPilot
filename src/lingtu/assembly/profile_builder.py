"""Compile one resolved LingTu product definition."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from lingtu.product import (
    PROCESS_CONTRACT_SCHEMA,
    PRODUCT_SCHEMA,
    Product,
    ProductManifest,
)
from runtime.blueprint import Blueprint
from runtime.graph import ProcessSpec, load_runtime_graph, resolve_processes
from runtime.graph.loader import RuntimeGraph
from runtime.profiles.resolver import canonical_profile_name

_MAP_CONTROL_PLANE_CONFIG_KEYS = (
    "_endpoint_transport",
    "_endpoint_contract",
    "slam_profile",
    "localization_adapter",
    "map_dir",
    "data_dir",
    "map_save_adapter",
    "map_save_timeout_sec",
    "source_profile",
    "data_source",
    "map_artifact_converter_command",
    "octomap_converter_command",
    "octomap_build_mode",
    "octomap_resolution",
    "octomap_free_layers_above",
    "octomap_free_dilation_cells",
    "octomap_build_timeout_sec",
    "build_octomap_on_save",
    "map_prune_command",
    "dynamic_filter_command",
    "map_opt",
    "map_optimization",
    "map_opt_command",
    "map_optimization_command",
    "map_opt_timeout_sec",
    "map_opt_required",
    "semantic_taxonomy_path",
    "semantic_save_timeout_sec",
    "octomap_editor_command",
    "octomap_edit_timeout_sec",
    "gateway_port",
    "mcp_port",
    "enable_gateway",
    "enable_teleop",
    "enable_camera",
    "command_output_mode",
    "hardware_control_boundary",
    "native_navigation_endpoint",
    "planning_frame_id",
    "planner",
    "planner_latency_budget_ms",
    "camera_backend",
    "manage_session_services",
    "camera_jpeg_quality",
    "camera_fps",
    "startup_timeout_s",
    "readiness_poll_interval_s",
    "stop_timeout_s",
    "runtime_failure_grace_s",
)

_MAP_CONTROL_PLANE_REQUIRED_FLAGS = (
    "enable_gateway",
    "enable_teleop",
    "enable_camera",
)


def blueprint_for_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the product Blueprint for a resolved profile."""

    canonical_profile = canonical_profile_name(profile)

    from lingtu.assembly.products import product_blueprint_for_profile

    return product_blueprint_for_profile(canonical_profile, config)


def compile_product(
    profile: str,
    config: Mapping[str, Any],
    *,
    endpoint: str | None = None,
    graph: RuntimeGraph | None = None,
) -> Product:
    """Compile one product without starting Modules or native processes."""

    canonical_profile = canonical_profile_name(profile)
    resolved_config = dict(config)
    graph = graph or load_runtime_graph()
    endpoint_name = _endpoint_name(resolved_config, endpoint=endpoint)
    endpoint_spec = graph.endpoints.get(endpoint_name, {}) if endpoint_name else {}
    process_control = str(endpoint_spec.get("process_control") or "module").strip()
    product_spec = graph.products.get(canonical_profile)
    required_topics = _string_tuple(product_spec.get("required_topics")) if product_spec else ()
    process_only = canonical_profile == "map" and endpoint_name == "thunder_field"

    validate_route_contract_for_resolved_config(resolved_config)
    route_contract = route_contract_name_for_resolved_config(resolved_config)

    processes: tuple[ProcessSpec, ...] = ()
    available_processes: tuple[ProcessSpec, ...] = ()
    process_conflicts: tuple[str, ...] = ()
    if product_spec is not None and endpoint_name and process_control == "launcher":
        processes, available_processes, process_conflicts = resolve_processes(
            canonical_profile,
            endpoint_name,
            graph=graph,
        )
    if process_only:
        application_config = _map_control_plane_config(resolved_config)
        processes = _with_process_application(
            processes,
            name="host",
            application="map_control_plane",
            config=application_config,
        )
        available_processes = _with_process_application(
            available_processes,
            name="host",
            application="map_control_plane",
            config=application_config,
        )

    required_capabilities = _string_tuple(product_spec.get("required_capabilities")) if product_spec else ()
    critical_modules = (
        _string_tuple(product_spec.get("critical_modules"))
        if product_spec is not None and not process_only
        else ()
    )
    native_nav = product_spec.get("native_nav") if product_spec else {}
    if native_nav is None:
        native_nav = {}
    if not isinstance(native_nav, Mapping):
        raise ValueError(f"product {canonical_profile!r} native_nav must be a mapping")

    blueprint: Blueprint | None = None
    if not process_only:
        blueprint_config = dict(resolved_config)
        blueprint_config["_product_required_topics"] = required_topics
        blueprint = blueprint_for_resolved_profile(canonical_profile, blueprint_config)
        if route_contract:
            blueprint.route_contract(route_contract)
        if critical_modules:
            blueprint.require_modules(*critical_modules)
    if product_spec is not None:
        from lingtu.assembly.validation import validate_profile

        issues = validate_profile(
            canonical_profile,
            graph,
            config=resolved_config,
            module_names=() if blueprint is None else blueprint.module_names,
            endpoint_name=endpoint_name,
            processes=processes,
        )
        if process_only:
            issues = [
                issue
                for issue in issues
                if issue.code != "real_profile_critical_module_missing"
            ]
        if issues:
            detail = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
            raise ValueError(f"product validation failed: {detail}")

    return Product(
        profile=canonical_profile,
        endpoint=endpoint_name,
        process_control=process_control,
        config=resolved_config,
        blueprint=blueprint,
        processes=processes,
        available_processes=available_processes,
        process_conflicts=process_conflicts,
        required_topics=required_topics,
        required_capabilities=required_capabilities,
        critical_modules=() if process_only else critical_modules,
        native_nav=dict(native_nav),
        route_contract=route_contract,
        module_transport="" if process_only else module_transport_name(resolved_config),
        manifest_schema=PROCESS_CONTRACT_SCHEMA if process_only else PRODUCT_SCHEMA,
    )


def module_transport_name(config: Mapping[str, Any]) -> str:
    """Return the effective ModulePort transport strategy for a resolved config."""

    return str(config.get("module_transport") or config.get("_module_transport") or "local").strip().lower()


def module_transport_for_resolved_config(config: Mapping[str, Any]) -> Any | None:
    """Create the build transport for a resolved profile config.

    ``None`` means Blueprint keeps its default fresh LocalTransport instance.
    Non-local strategies are wrapped so Out/In ports can use the simple
    publish/subscribe transport protocol.
    """

    strategy = module_transport_name(config)
    if strategy in ("", "local"):
        return None

    from runtime.transport.factory import create_transport_adapter

    return create_transport_adapter(strategy)


def blueprint_from_manifest(manifest: ProductManifest) -> Blueprint:
    """Recreate only the Host graph declared by a compiled Product manifest."""

    config = dict(manifest.host_config)
    endpoint = _endpoint_name(config, endpoint=None)
    if endpoint != manifest.endpoint:
        raise ValueError(f"Product manifest Host endpoint mismatch: manifest={manifest.endpoint!r} config={endpoint!r}")
    route_contract = route_contract_name_for_resolved_config(config)
    if route_contract != manifest.route_contract:
        raise ValueError(
            "Product manifest Host route contract mismatch: "
            f"manifest={manifest.route_contract!r} config={route_contract!r}"
        )
    module_transport = module_transport_name(config)
    if module_transport != manifest.module_transport:
        raise ValueError(
            "Product manifest Host transport mismatch: "
            f"manifest={manifest.module_transport!r} config={module_transport!r}"
        )

    validate_route_contract_for_resolved_config(config)
    assembly_config = dict(config)
    assembly_config["_product_required_topics"] = manifest.required_topics
    blueprint = blueprint_for_resolved_profile(manifest.profile, assembly_config)
    if manifest.route_contract:
        blueprint.route_contract(manifest.route_contract)
    if manifest.critical_modules:
        blueprint.require_modules(*manifest.critical_modules)
    if blueprint.module_names != manifest.modules:
        raise ValueError(
            "Product manifest Host modules do not match the current assembly: "
            f"manifest={manifest.modules!r} assembly={blueprint.module_names!r}"
        )
    return blueprint


def build_host_from_manifest(manifest: ProductManifest) -> Any:
    """Build a Host from immutable Product data without resolving processes."""

    blueprint = blueprint_from_manifest(manifest)
    transport = module_transport_for_resolved_config(manifest.host_config)
    if transport is None:
        return blueprint.build()
    return blueprint.build(transport=transport)


def route_contract_name_for_resolved_config(config: Mapping[str, Any]) -> str | None:
    """Return the external route contract selected by a resolved config.

    Route contracts are boundary metadata. They validate endpoint topics and
    adapter ownership, but they do not change ModulePort delivery unless a
    Blueprint explicitly calls ``Blueprint.routed_delivery(...)``.
    """

    from runtime.profiles.endpoints import route_contract_for_config

    return route_contract_for_config(config)


def validate_route_contract_for_resolved_config(config: Mapping[str, Any]) -> None:
    """Fail fast when a resolved endpoint references an invalid route contract."""

    route_contract = route_contract_name_for_resolved_config(config)
    if not route_contract:
        return

    from runtime.route_contract import load_route_contract, validate_route_contract

    contract = load_route_contract(route_contract)
    issues = validate_route_contract(contract)
    if issues:
        detail = "; ".join(f"{issue.code}:{issue.scope}: {issue.message}" for issue in issues)
        raise ValueError(f"invalid route contract '{route_contract}': {detail}")


def build_system_from_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Any:
    """Build a system from a resolved profile config and its transport contract."""

    return compile_product(profile, config).build()


def build_system_for_profile(
    profile: str,
    overrides: Mapping[str, Any] | None = None,
    **inline_overrides: Any,
) -> Any:
    """Resolve a profile, select the product Blueprint, then build System."""

    from runtime.profiles.resolver import resolve_profile_config

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    config = resolve_profile_config(profile, overrides=merged_overrides)
    return compile_product(profile, config).build()


def _endpoint_name(config: Mapping[str, Any], *, endpoint: str | None) -> str | None:
    value = endpoint or config.get("_runtime_endpoint") or config.get("runtime_endpoint")
    name = str(value or "").strip()
    return name or None


def _map_control_plane_config(config: Mapping[str, Any]) -> dict[str, Any]:
    selected = {
        key: config[key]
        for key in _MAP_CONTROL_PLANE_CONFIG_KEYS
        if key in config
    }
    invalid = tuple(
        key
        for key, value in selected.items()
        if value is not None and not isinstance(value, str | int | float | bool)
    )
    if invalid:
        raise ValueError(
            "map_control_plane process config must contain only JSON scalars: "
            f"{', '.join(invalid)}"
        )
    for flag in _MAP_CONTROL_PLANE_REQUIRED_FLAGS:
        if selected.get(flag) is not True:
            raise ValueError(f"map_control_plane requires {flag}=true")
    return selected


def _with_process_application(
    processes: tuple[ProcessSpec, ...],
    *,
    name: str,
    application: str,
    config: Mapping[str, Any],
) -> tuple[ProcessSpec, ...]:
    found = False
    configured: list[ProcessSpec] = []
    for process in processes:
        if process.name == name:
            configured.append(
                replace(process, application=application, config=config)
            )
            found = True
        else:
            configured.append(process)
    if not found:
        raise ValueError(f"process-only Product is missing required process {name!r}")
    return tuple(configured)


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set):
        return tuple(str(item) for item in value)
    return ()
