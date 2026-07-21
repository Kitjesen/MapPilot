"""Minimal no-hardware product assembly used by tests."""

from __future__ import annotations

from typing import Any

from drivers.sim.stub import StubDogModule
from runtime.blueprint import Blueprint
from runtime.plugin_resolution import stack_module


def stub_blueprint(**config: Any) -> Blueprint:
    """Assemble the stub driver, navigation mission, and safety ring."""

    navigation_cls = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.navigation.Navigation",
    )
    safety_cls = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.services.safety.safety_ring.SafetyRing",
    )

    blueprint = Blueprint(name="stub")
    blueprint.add(StubDogModule)
    blueprint.add(navigation_cls, planner=config.get("planner_backend", "direct"))
    blueprint.add(safety_cls)
    blueprint.wire("nav.safety", "stop_cmd", "StubDogModule", "stop_signal")
    return blueprint.auto_wire()
