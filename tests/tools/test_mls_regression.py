import runpy
from pathlib import Path

assess = runpy.run_path(str(Path(__file__).parents[2] / "tools/bench/mls_regression.py"))["assess"]


def test_only_checked_repeatable_rust_routes_pass():
    plan = {"ok": True, "reached_goal": True, "returncode": 0,
            "benchmark": {"same_result": True},
            "diagnostics": {"used_rust_graph": True, "rust_fallback": False}}
    geometry = {"route_valid": True, "sweep_valid": True, "valid": True, "returncode": 0}
    assert assess(plan, geometry) == "rust_pass"
    geometry["returncode"] = 1
    assert assess(plan, geometry) == "geometry_failed"
    geometry["returncode"] = 0
    del plan["diagnostics"]["rust_fallback"]
    assert assess(plan, geometry) == "missing_diagnostics"
    plan["diagnostics"]["rust_fallback"] = True
    assert assess(plan, geometry) == "fallback_pass"
    geometry["sweep_valid"] = False
    assert assess(plan, geometry) == "geometry_failed"
    geometry["sweep_valid"] = True
    plan["benchmark"]["same_result"] = False
    assert assess(plan, geometry) == "repeat_changed"
    plan["ok"] = False
    assert assess(plan, geometry) == "planning_failed"
