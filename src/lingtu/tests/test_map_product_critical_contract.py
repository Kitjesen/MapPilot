from __future__ import annotations

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import blueprint_from_run_plan, compile_run_plan


def test_real_map_requires_map_service_in_host_blueprint() -> None:
    resolved = resolve_product_host_runtime("map", "real")
    plan = compile_run_plan(resolved.product, resolved.env, resolved.config)
    blueprint = blueprint_from_run_plan(plan)

    assert "maps.service" in plan.modules
    assert "maps.service" in plan.critical_modules
    assert blueprint.required_module_names == plan.critical_modules
