"""Compile one resolved LingTu product into Module and process plans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from runtime.blueprint import Blueprint
from runtime.graph import RuntimePlan, build_runtime_plan, load_runtime_graph
from runtime.graph.loader import RuntimeGraph
from runtime.profiles.resolver import canonical_profile_name


@dataclass(frozen=True)
class Product:
    """Side-effect-free result of compiling one profile and endpoint."""

    profile: str
    endpoint: str | None
    process_control: str
    config: Mapping[str, Any]
    blueprint: Blueprint
    plan: RuntimePlan | None
    required_topics: tuple[str, ...]
    route_contract: str | None
    module_transport: str

    @property
    def modules(self) -> tuple[str, ...]:
        """Return stable Module aliases without instantiating the graph."""

        return self.blueprint.module_names

    def build(self) -> Any:
        """Materialize only the Blueprint-owned application Module graph."""

        transport = module_transport_for_resolved_config(self.config)
        if transport is None:
            return self.blueprint.build()
        return self.blueprint.build(transport=transport)

    def as_dict(self) -> dict[str, Any]:
        """Return the product contract used by CLI, diagnostics, and Launcher."""

        return {
            "schema_version": "lingtu.product.v1",
            "profile": self.profile,
            "endpoint": self.endpoint,
            "process_control": self.process_control,
            "modules": list(self.modules),
            "runtime_plan": self.plan.as_dict() if self.plan is not None else None,
            "required_topics": list(self.required_topics),
            "route_contract": self.route_contract,
            "module_transport": self.module_transport,
        }


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

    validate_route_contract_for_resolved_config(resolved_config)
    route_contract = route_contract_name_for_resolved_config(resolved_config)
    blueprint = blueprint_for_resolved_profile(canonical_profile, resolved_config)
    if route_contract:
        blueprint.route_contract(route_contract)

    plan: RuntimePlan | None = None
    product_spec = graph.products.get(canonical_profile)
    if product_spec is not None and endpoint_name and process_control == "runtime_plan":
        plan = build_runtime_plan(canonical_profile, endpoint_name, graph=graph)

    required_topics = _string_tuple(product_spec.get("required_topics")) if product_spec else ()
    if product_spec is not None:
        from lingtu.assembly.validation import validate_profile

        issues = validate_profile(
            canonical_profile,
            graph,
            config=resolved_config,
            module_names=blueprint.module_names,
        )
        if issues:
            detail = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
            raise ValueError(f"product validation failed: {detail}")

    return Product(
        profile=canonical_profile,
        endpoint=endpoint_name,
        process_control=process_control,
        config=resolved_config,
        blueprint=blueprint,
        plan=plan,
        required_topics=required_topics,
        route_contract=route_contract,
        module_transport=module_transport_name(resolved_config),
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
        detail = "; ".join(f"{issue.code}:{issue.topic}" for issue in issues)
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


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set):
        return tuple(str(item) for item in value)
    return ()
