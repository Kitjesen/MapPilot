from __future__ import annotations

from lingtu.assembly.profile_builder import compile_product
from runtime.blueprint import Blueprint
from runtime.profiles.resolver import resolve_runtime_config


def test_field_product_compiles_module_graph_and_runtime_plan_together() -> None:
    resolved = resolve_runtime_config("nav")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.profile == "nav"
    assert product.endpoint == "thunder_field"
    assert product.process_control == "runtime_plan"
    assert product.plan is not None
    assert product.plan.has_process("slam")
    assert product.plan.has_process("nav")
    assert product.plan.has_process("driver")
    assert "/slam/odometry" in product.required_topics
    assert "/nav/cmd_vel" in product.required_topics
    assert "nav.mission" in product.modules
    assert "nav.local_planner" not in product.modules
    assert "nav.path_follower" not in product.modules


def test_local_profile_compiles_without_native_process_plan() -> None:
    resolved = resolve_runtime_config("sim_nav")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.profile == "sim_nav"
    assert product.plan is None
    assert product.process_control == "module"
    assert "nav.mission" in product.modules


def test_product_contract_is_serializable_without_starting_runtime() -> None:
    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    payload = product.as_dict()

    assert payload["schema_version"] == "lingtu.product.v1"
    assert payload["runtime_plan"]["endpoint"] == "thunder_field"
    assert payload["route_contract"] == "robot"
    assert payload["module_transport"] == "local"


def test_product_compile_defers_startup_preflight(monkeypatch) -> None:
    import lingtu.assembly.products.thunder as thunder

    calls: list[str] = []
    monkeypatch.setattr(
        thunder,
        "_run_startup_preflight",
        lambda **_: calls.append("preflight"),
    )
    resolved = resolve_runtime_config("nav")

    compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert calls == []


def test_blueprint_runs_deferred_checks_only_when_building() -> None:
    calls: list[str] = []
    blueprint = Blueprint().before_build(lambda: calls.append("preflight"))

    assert calls == []
    blueprint.build()

    assert calls == ["preflight"]
