# ruff: noqa: S101

from __future__ import annotations

from copy import deepcopy

from lingtu.assembly.graph import graph_for_product
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import blueprint_from_run_plan, compile_run_plan
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import RuntimeGraph, load_runtime_graph, validate_runtime_graph
from runtime.runtime_interface import TOPICS

OPERATOR_MOTION_TOPICS = frozenset(
    {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }
)
OPERATOR_MOTION_CAPABILITIES = frozenset(
    {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
    }
)
OPERATOR_MOTION_CRITICAL_MODULES = frozenset({"nav.commands", "GatewayModule"})
PYTHON_MOTION_AUTHORITY_MODULES = frozenset({"TeleopModule", "nav.velocity_mux"})
PRODUCTS_REQUIRING_OPERATOR_MOTION = frozenset(
    {
        "inspection",
        "map",
        "nav",
        "teleop",
        "teleop_avoid",
        "tracking",
    }
)


def _with_products(graph: RuntimeGraph, products: dict) -> RuntimeGraph:
    return RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )


def _contracts(name: str, product: dict):
    return resolve_product_spec_contracts(name, product)


def test_operator_motion_python_module_is_not_mounted_by_field_products() -> None:
    for mode in ("static", "runtime"):
        assisted = graph_for_product(
            "teleop_avoid",
            env="real",
            mode=mode,
            run_startup_checks=False,
        )
        assert "operator.motion" not in assisted.modules

    for profile in ("teleop", "map", "nav"):
        assert "operator.motion" not in graph_for_product(profile, env="real").modules


def test_teleop_avoid_compilation_uses_native_operator_motion_without_python_authority() -> None:
    resolved = resolve_product_host_runtime("teleop_avoid", "real")

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )
    blueprint = blueprint_from_run_plan(plan)

    assert plan.critical_modules == (
        "host.bus",
        "SlamAdapterModule",
        "nav.commands",
        "GatewayModule",
    )
    assert blueprint.required_module_names == plan.critical_modules
    assert "operator.motion" not in plan.modules
    assert "maps.service" not in plan.modules
    assert "nav.commands" in plan.modules
    assert plan.process_control == "systemd"
    assert plan.has_process("maps")
    assert plan.has_process("nav")
    assert plan.has_process("driver")
    assert OPERATOR_MOTION_TOPICS <= set(plan.required_topics)
    assert OPERATOR_MOTION_CAPABILITIES <= set(plan.required_capabilities)


def test_teleop_avoid_declares_explicit_native_motion_settings() -> None:
    product = load_runtime_graph().products["teleop_avoid"]
    contract = _contracts("teleop_avoid", product)

    assert product["native_nav"] == {
        "control_mode": "teleop_avoid",
        "publish_cmd_vel": True,
        "check_obstacle": True,
        "use_traversability_cost": True,
        "allow_teleop_takeover": False,
        "teleop_local_planner": True,
    }
    assert OPERATOR_MOTION_CAPABILITIES <= set(contract.capabilities)
    assert OPERATOR_MOTION_TOPICS <= set(contract.topics)
    assert "required_capabilities" not in product
    assert "required_topics" not in product


def test_operator_motion_requirement_is_derived_from_native_policy_for_every_product() -> None:
    products = load_runtime_graph().products

    assert set(products) == PRODUCTS_REQUIRING_OPERATOR_MOTION | {"explore"}
    for name, product in products.items():
        native_nav = product.get("native_nav") or {}
        control_mode = native_nav.get("control_mode")
        derived_requirement = control_mode in {"teleop", "teleop_avoid"} or (
            control_mode == "autonomy" and native_nav.get("allow_teleop_takeover") is True
        )

        assert derived_requirement is (name in PRODUCTS_REQUIRING_OPERATOR_MOTION)
        contract = _contracts(name, product)
        topics = set(contract.topics)
        capabilities = set(contract.capabilities)
        if derived_requirement:
            assert OPERATOR_MOTION_TOPICS <= topics
            assert OPERATOR_MOTION_CAPABILITIES <= capabilities
            assert OPERATOR_MOTION_CRITICAL_MODULES <= set(product.get("critical_modules", ()))
        else:
            assert OPERATOR_MOTION_TOPICS.isdisjoint(topics)
            assert OPERATOR_MOTION_CAPABILITIES.isdisjoint(capabilities)


def test_compiled_operator_motion_products_make_host_adapters_startup_critical() -> None:
    for profile in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        resolved = resolve_product_host_runtime(profile, "real")
        manifest = compile_run_plan(
            resolved.product,
            resolved.env,
            resolved.config,
        )
        blueprint = blueprint_from_run_plan(manifest)

        assert OPERATOR_MOTION_CRITICAL_MODULES <= set(manifest.modules)
        assert OPERATOR_MOTION_CRITICAL_MODULES <= set(manifest.critical_modules)
        assert blueprint.required_module_names == manifest.critical_modules

    explore = load_runtime_graph().products["explore"]
    assert "operator.motion" not in explore.get("critical_modules", ())


def test_explore_map_variant_declares_operator_motion_takeover_contract() -> None:
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
    )

    assert plan.product == "explore"
    assert plan.product_variant == "map"
    assert OPERATOR_MOTION_TOPICS <= set(plan.required_topics)
    assert OPERATOR_MOTION_CAPABILITIES <= set(plan.required_capabilities)
    assert OPERATOR_MOTION_CRITICAL_MODULES <= set(plan.critical_modules)


def test_validator_rejects_contract_without_operator_motion_topics_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["contracts"] = ["lingtu.product.explore.v1"]

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_boundary_incomplete" and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(topic in matching[0].message for topic in OPERATOR_MOTION_TOPICS)


def test_validator_rejects_contract_without_operator_motion_capabilities_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["contracts"] = ["lingtu.product.explore.v1"]

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_capability_missing" and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(capability in matching[0].message for capability in OPERATOR_MOTION_CAPABILITIES)


def test_validator_rejects_missing_critical_host_adapters_for_every_required_product() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        products[product_name]["critical_modules"] = []

    issues = validate_runtime_graph(_with_products(graph, products))

    for product_name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_operator_motion_critical_adapter_missing"
            and issue.scope == f"product:{product_name}"
        ]
        assert len(matching) == 1
        assert all(module in matching[0].message for module in OPERATOR_MOTION_CRITICAL_MODULES)


def test_native_final_writer_products_forbid_python_motion_authorities() -> None:
    products = load_runtime_graph().products

    for name, product in products.items():
        capabilities = set(_contracts(name, product).capabilities)
        if "final_cmd_vel_single_writer" not in capabilities:
            continue
        forbidden = set(product.get("forbidden_modules", ()))
        assert PYTHON_MOTION_AUTHORITY_MODULES <= forbidden, name


def test_validator_rejects_python_motion_authority_in_native_final_writer_products() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    affected = []
    for name, product in products.items():
        capabilities = set(_contracts(name, product).capabilities)
        if "final_cmd_vel_single_writer" not in capabilities:
            continue
        affected.append(name)
        product["forbidden_modules"] = sorted(
            set(product.get("forbidden_modules", ())) - PYTHON_MOTION_AUTHORITY_MODULES
        )

    issues = validate_runtime_graph(_with_products(graph, products))

    for name in affected:
        matching = [
            issue
            for issue in issues
            if issue.code == "product_python_motion_authority_not_forbidden"
            and issue.scope == f"product:{name}"
        ]
        assert len(matching) == 1
        assert all(module in matching[0].message for module in PYTHON_MOTION_AUTHORITY_MODULES)


def test_native_policy_bootstraps_operator_motion_requirement() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["native_nav"]["allow_teleop_takeover"] = True

    issues = validate_runtime_graph(_with_products(graph, products))

    codes = {
        issue.code
        for issue in issues
        if issue.scope == "product:explore"
    }
    assert "product_operator_motion_boundary_incomplete" in codes
    assert "product_operator_motion_capability_missing" in codes


def test_product_contracts_use_typed_operator_motion_topics_without_yaml_chains() -> None:
    products = load_runtime_graph().products

    for name in PRODUCTS_REQUIRING_OPERATOR_MOTION:
        contract = _contracts(name, products[name])
        assert OPERATOR_MOTION_TOPICS <= set(contract.topics)
        assert "target_chain" not in products[name]
