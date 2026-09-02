from __future__ import annotations

from lingtu.assembly.compiler import blueprint_from_run_plan, compile_run_plan
from lingtu.assembly.products import resolve_product_host_runtime


def test_real_map_uses_native_mapd_without_python_map_module() -> None:
    resolved = resolve_product_host_runtime("map", "real", robot="unitree/go2")
    plan = compile_run_plan(resolved.product, resolved.env, robot="unitree/go2")
    blueprint = blueprint_from_run_plan(plan)

    assert plan.has_process("maps")
    assert "maps.service" not in plan.modules
    assert "maps.service" not in plan.critical_modules
    assert blueprint.required_module_names == plan.critical_modules
