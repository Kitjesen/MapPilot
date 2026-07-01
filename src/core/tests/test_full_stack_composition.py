from __future__ import annotations

from core.blueprints.full_stack import full_stack_blueprint
from core.blueprints.stacks.composition import compose_full_stack_modules


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }


def test_compose_full_stack_modules_builds_minimal_stub_graph() -> None:
    bp = compose_full_stack_modules(
        robot="stub",
        driver_module="StubDogModule",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        tomogram="",
        gateway_port=5050,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        config={},
    )
    names = _entry_names(bp)

    assert "StubDogModule" in names
    assert "NavigationModule" in names
    assert "SafetyRingModule" in names
    assert "CmdVelMux" in names
    assert "ExternalServiceManagerModule" not in names
    assert "PerceptionModule" not in names
    assert "GatewayModule" not in names


def test_full_stack_blueprint_keeps_wiring_outside_stack_composition() -> None:
    bp = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        manage_external_services=False,
        run_startup_checks=False,
    )
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert "StubDogModule" in names
    assert "NavigationModule" in names
    assert "SafetyRingModule" in names
    assert "SafetyRingModule.stop_cmd->StubDogModule.stop_signal" in wires
    assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in wires
