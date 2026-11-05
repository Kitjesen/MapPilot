"""Only a resolved real DOSO RunPlan supplies the Brainstem observation source."""

import importlib
from types import SimpleNamespace

import pytest

gateway_stack = importlib.import_module("lingtu.assembly.host.stacks.gateway")


@pytest.mark.parametrize("env,backend,enabled", [
    ("real", "doso", True), ("real", "go2", False), ("sim", "doso", False),
])
def test_source_follows_resolved_driver(monkeypatch, env, backend, enabled):
    monkeypatch.setattr(gateway_stack, "stack_module", lambda *args, **kwargs: type("Gateway", (), {}))
    monkeypatch.setattr(gateway_stack, "optional_stack_module", lambda *args, **kwargs: None)
    environment = {"LINGTU_DRIVER_BACKEND": backend, "LINGTU_DRIVER_TARGET": "127.0.0.1:13145"}
    plan = SimpleNamespace(env=env, process_environment=environment)
    builder = gateway_stack.gateway(run_plan=plan)
    config = builder._entries[0].config
    assert ("observations" in config) is enabled
    if enabled:
        assert config["observations"]._environment == environment


def test_compiled_doso_plan_injects_the_exact_driver_configuration():
    from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan

    plan = compile_run_plan("teleop", "real", robot="doso/thunder_v4")
    builder = host_builder_from_run_plan(plan)
    config = next(entry.config for entry in builder._entries if entry.name == "GatewayModule")
    source_environment = config["observations"]._environment
    assert source_environment == plan.process_environment
    assert source_environment["LINGTU_DRIVER_BACKEND"] == "doso"
    assert source_environment["LINGTU_DRIVER_TARGET"]
    assert all(f"LINGTU_DRIVER_TLS_{name}_FILE" in source_environment for name in ("CA", "CERT", "KEY"))
