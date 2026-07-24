from __future__ import annotations

from lingtu.assembly.graph import graph_for_profile
from lingtu.assembly.profile_builder import compile_product
from lingtu.assembly.validation import validate_profile
from runtime.graph import load_runtime_graph
from runtime.profiles.resolver import resolve_runtime_config


def test_operator_motion_is_mounted_only_for_teleop_avoid_product() -> None:
    for mode in ("static", "runtime"):
        assisted = graph_for_profile(
            "teleop_avoid",
            mode=mode,
            run_startup_checks=False,
            manage_external_services=False,
        )
        assert "operator.motion" in assisted.modules

    for profile in ("teleop", "map", "nav"):
        assert "operator.motion" not in graph_for_profile(profile).modules


def test_teleop_avoid_compilation_requires_operator_motion() -> None:
    resolved = resolve_runtime_config("teleop_avoid")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    critical_modules = tuple(load_runtime_graph().products["teleop_avoid"].get("critical_modules") or ())
    assert critical_modules == (
        "SlamAdapterModule",
        "maps.service",
        "nav.commands",
        "operator.motion",
        "GatewayModule",
    )
    assert "operator.motion" in product.modules


def test_teleop_avoid_validation_fails_when_operator_motion_is_missing() -> None:
    resolved = resolve_runtime_config("teleop_avoid")
    modules = tuple(
        module
        for module in graph_for_profile("teleop_avoid", mode="runtime").modules
        if module != "operator.motion"
    )

    issues = validate_profile(
        "teleop_avoid",
        config=resolved.config,
        module_names=modules,
    )

    missing = [issue for issue in issues if issue.code == "real_profile_critical_module_missing"]
    assert len(missing) == 1
    assert "operator.motion" in missing[0].message
