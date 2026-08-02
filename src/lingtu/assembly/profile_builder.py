"""Build local Profile Hosts and compile field Products."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from lingtu.run_plan import RunPlan
from lingtu.run_plan_contract import (
    compiled_against as compile_run_plan_identity,
)
from lingtu.run_plan_contract import (
    lifecycle_snapshot,
)
from runtime.blueprint import Blueprint
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import (
    ProcessSpec,
    load_runtime_graph,
    resolve_env_implementation,
    resolve_processes,
)
from runtime.graph.loader import RuntimeGraph, resolve_product_variant_spec
from runtime.profiles.native_nav_config import compile_native_nav_config
from runtime.profiles.product_lifecycle import product_name
from runtime.profiles.resolver import canonical_profile_name


def blueprint_for_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the Host Blueprint for an already resolved local Profile."""

    canonical_profile = canonical_profile_name(profile)

    from lingtu.assembly.products import host_blueprint_for_profile

    return host_blueprint_for_profile(canonical_profile, config)


def blueprint_for_resolved_product(
    product: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the Host Blueprint for an already resolved field Product."""

    resolved_product = product_name(product)

    from lingtu.assembly.products import host_blueprint_for_product

    return host_blueprint_for_product(resolved_product, config)


def compile_run_plan(
    product: str,
    env: str,
    config: Mapping[str, Any],
    *,
    product_variant: str | None = None,
    env_config: Mapping[str, Any] | None = None,
    graph: RuntimeGraph | None = None,
    environment: Mapping[str, str] | None = None,
) -> RunPlan:
    """Compile one Product without starting Modules or native processes."""

    try:
        resolved_product = product_name(product)
    except ValueError as exc:
        raise ValueError(
            f"{product!r} is not a Product; local Profiles do not compile to RunPlan"
        ) from exc
    resolved_config = dict(config)
    graph = graph or load_runtime_graph()
    env_name = _env_name(env)
    process_control = _process_control(graph, env_name, env_config)
    declared_product_spec = graph.products.get(resolved_product)
    if not isinstance(declared_product_spec, Mapping):
        raise ValueError(
            f"Product {resolved_product!r} is not declared in the Runtime Graph"
        )
    product_spec = resolve_product_variant_spec(
        resolved_product,
        declared_product_spec,
        product_variant=product_variant,
    )
    resolved_product_variant = _optional_product_variant(
        product_spec.get("product_variant")
    )
    config_product_variant = _optional_product_variant(
        resolved_config.get("_product_variant")
    )
    if config_product_variant != resolved_product_variant:
        raise ValueError(
            "Product Host config variant does not match the RunPlan selection: "
            f"config={config_product_variant!r} plan={resolved_product_variant!r}"
        )
    runtime_contracts = resolve_product_spec_contracts(
        resolved_product,
        product_spec,
    )
    required_topics = runtime_contracts.topics

    validate_route_contract_for_resolved_config(resolved_config)
    route_contract = route_contract_name_for_resolved_config(resolved_config)

    processes: tuple[ProcessSpec, ...] = ()
    available_processes: tuple[ProcessSpec, ...] = ()
    process_conflicts: tuple[str, ...] = ()
    if process_control == "systemd":
        processes, available_processes, process_conflicts = resolve_processes(
            resolved_product,
            env_name,
            graph=graph,
            env_config=env_config,
        )
    required_capabilities = runtime_contracts.capabilities
    implementation = resolve_env_implementation(
        env_name,
        graph=graph,
        env_config=env_config,
    )
    # Real Products use the native-adapter Host contract declared by the
    # Product. Simulation backends own their Host implementation and may
    # declare a different critical-module set; an external runner defaults to
    # no Host readiness assertion here.
    critical_modules = _string_tuple(
        product_spec.get("critical_modules")
        if env_name == "real"
        else implementation.get("critical_modules")
    )
    native_nav = product_spec.get("native_nav")
    if native_nav is None:
        native_nav = {}
    if not isinstance(native_nav, Mapping):
        raise ValueError(f"Product {resolved_product!r} native_nav must be a mapping")

    blueprint_config = dict(resolved_config)
    blueprint_config["_product_required_topics"] = required_topics
    blueprint_config["_product_required_capabilities"] = required_capabilities
    blueprint = blueprint_for_resolved_product(resolved_product, blueprint_config)
    if route_contract:
        blueprint.route_contract(route_contract)
    if critical_modules:
        blueprint.require_modules(*critical_modules)
    from lingtu.assembly.validation import validate_product

    issues = validate_product(
        resolved_product,
        graph,
        config=resolved_config,
        module_names=blueprint.module_names,
        env_name=env_name,
        env_config=dict(env_config) if env_config is not None else None,
        processes=processes,
    )
    if issues:
        detail = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
        raise ValueError(f"Product validation failed: {detail}")

    managed_stop_targets = tuple(
        process.target
        for process in reversed(available_processes)
        if process.lifecycle == "mode"
    )
    stop_targets = tuple(
        dict.fromkeys((*managed_stop_targets, *process_conflicts))
    )
    lifecycle = lifecycle_snapshot(
        resolved_product,
        graph,
        product_variant=resolved_product_variant,
    )
    native_runtime = compile_native_nav_config(
        resolved_product,
        {
            **resolved_config,
            "native_control_mode": lifecycle["native_control_mode"],
            "native_nav": dict(native_nav),
        },
    )
    parameter_profile = _optional_parameter_profile(
        product_spec.get("parameter_profile"),
        product=resolved_product,
    )
    parameter_overrides = _parameter_overrides(
        graph,
        env_name,
        env_config,
    )
    contract_identity = compile_run_plan_identity(
        product=resolved_product,
        product_variant=resolved_product_variant,
        env=env_name,
        blueprint=blueprint,
        processes=processes,
        graph=graph,
        env_config=env_config,
        environment=environment,
    )
    return RunPlan.create(
        product=resolved_product,
        product_variant=resolved_product_variant,
        env=env_name,
        process_control=process_control,
        modules=blueprint.module_names,
        processes=processes,
        available_processes=available_processes,
        stop_targets=stop_targets,
        contracts=runtime_contracts.contract_ids,
        critical_modules=critical_modules,
        native_nav=dict(native_nav),
        native_process_environment=native_runtime.environment,
        route_contract=route_contract,
        module_transport=module_transport_name(resolved_config),
        host_config=resolved_config,
        lifecycle=lifecycle,
        parameter_profile=parameter_profile,
        parameter_overrides=parameter_overrides,
        compiled_against=contract_identity,
    )


def module_transport_name(config: Mapping[str, Any]) -> str:
    """Return the effective ModulePort transport strategy for a resolved config."""

    return str(config.get("module_transport") or config.get("_module_transport") or "local").strip().lower()


def module_transport_for_resolved_config(config: Mapping[str, Any]) -> Any | None:
    """Create the build transport for an already resolved Host config.

    ``None`` means Blueprint keeps its default fresh LocalTransport instance.
    Non-local strategies are wrapped so Out/In ports can use the simple
    publish/subscribe transport protocol.
    """

    strategy = module_transport_name(config)
    if strategy in ("", "local"):
        return None

    from runtime.transport.factory import create_transport_adapter

    return create_transport_adapter(strategy)


def blueprint_from_run_plan(plan: RunPlan) -> Blueprint:
    """Recreate only the Host graph declared by a RunPlan."""

    plan.assert_compatible()
    config = dict(plan.host_config)
    config_env = _env_name(config.get("_env"))
    if config_env != plan.env:
        raise ValueError(
            f"RunPlan Host env mismatch: plan={plan.env!r} config={config_env!r}"
        )
    route_contract = route_contract_name_for_resolved_config(config)
    if route_contract != plan.route_contract:
        raise ValueError(
            "RunPlan Host route contract mismatch: "
            f"plan={plan.route_contract!r} config={route_contract!r}"
        )
    module_transport = module_transport_name(config)
    if module_transport != plan.module_transport:
        raise ValueError(
            "RunPlan Host transport mismatch: "
            f"plan={plan.module_transport!r} config={module_transport!r}"
        )

    validate_route_contract_for_resolved_config(config)
    assembly_config = dict(config)
    assembly_config["_product_required_topics"] = plan.required_topics
    assembly_config["_product_required_capabilities"] = plan.required_capabilities
    # The managed Host must hand the exact fingerprinted execution record to
    # its Gateway.  Re-resolving a Product from the current source tree would
    # combine a running plan with a different contract after an upgrade.
    assembly_config["_run_plan"] = plan
    assembly_config["_run_plan_fingerprint"] = plan.fingerprint
    blueprint = blueprint_for_resolved_product(plan.product, assembly_config)
    if plan.route_contract:
        blueprint.route_contract(plan.route_contract)
    if plan.critical_modules:
        blueprint.require_modules(*plan.critical_modules)
    if blueprint.module_names != plan.modules:
        raise ValueError(
            "RunPlan Host modules do not match the current assembly: "
            f"plan={plan.modules!r} assembly={blueprint.module_names!r}"
        )
    return blueprint


def build_host_from_run_plan(plan: RunPlan) -> Any:
    """Build a Host from immutable Product data without resolving processes."""

    blueprint = blueprint_from_run_plan(plan)
    transport = module_transport_for_resolved_config(plan.host_config)
    if transport is None:
        return blueprint.build()
    return blueprint.build(transport=transport)


def route_contract_name_for_resolved_config(config: Mapping[str, Any]) -> str | None:
    """Return the external route contract selected by a resolved config.

    Route contracts are boundary metadata. They validate endpoint topics and
    adapter ownership, but they do not change ModulePort delivery unless a
    Blueprint explicitly calls ``Blueprint.routed_delivery(...)``.
    """

    from runtime.profiles.profile_adapters import route_contract_for_config

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
    """Build a System from a resolved local Profile Host config."""

    blueprint = blueprint_for_resolved_profile(profile, config)
    transport = module_transport_for_resolved_config(config)
    if transport is None:
        return blueprint.build()
    return blueprint.build(transport=transport)


def build_system_for_profile(
    profile: str,
    overrides: Mapping[str, Any] | None = None,
    **inline_overrides: Any,
) -> Any:
    """Resolve a local Profile Host Blueprint, then build its System."""

    from runtime.profiles.resolver import resolve_profile_config

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    config = resolve_profile_config(profile, overrides=merged_overrides)
    return build_system_from_resolved_profile(profile, config)


def _env_name(value: Any) -> str:
    env = str(value or "").strip()
    if env not in {"real", "sim"}:
        raise ValueError(f"env must be 'real' or 'sim', received {env!r}")
    return env


def _process_control(
    graph: RuntimeGraph,
    env: str,
    env_config: Mapping[str, Any] | None,
) -> str:
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    return str(implementation.get("process_control") or "module").strip()


def _optional_parameter_profile(value: Any, *, product: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Product {product!r} parameter_profile must be a string")
    return value.strip()


def _optional_product_variant(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError("product_variant must be a non-empty string")
    return value.strip()


def _parameter_overrides(
    graph: RuntimeGraph,
    env: str,
    env_config: Mapping[str, Any] | None,
) -> Mapping[str, Any]:
    env_spec = graph.envs.get(env)
    if not isinstance(env_spec, Mapping):
        raise ValueError(f"unknown Runtime Graph env: {env}")
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    value = implementation.get(
        "parameter_overrides",
        env_spec.get("parameter_overrides", {}),
    )
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"Env {env!r} parameter_overrides must be a mapping")
    return value


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, set | frozenset):
        return tuple(sorted(str(item) for item in value))
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    return ()
