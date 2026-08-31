"""Apply the cross-stack wires for a complete Host graph."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.introspection.module_graph import ModuleGraph
from runtime.wiring import WireSpec, wire_key

from .context import build_wiring_context
from .gateway import gateway_status_specs, teleop_media_specs
from .navigation import (
    navigation_support_specs,
)
from .semantic import (
    recorder_specs,
    semantic_camera_specs,
    semantic_command_specs,
    semantic_scene_specs,
    visual_servo_specs,
    vla_specs,
)
from .slam import (
    localization_specs,
    map_specs,
    odometry_fanout_specs,
    scan_view_specs,
)


def _apply_wire(
    bp: Blueprint,
    graph: ModuleGraph,
    spec: WireSpec,
) -> None:
    issues: list[str] = []
    if not _declares_port(graph, spec.out_module, spec.out_port):
        issues.append(f"missing source port {spec.out_module}.{spec.out_port}")

    if not _declares_port(graph, spec.in_module, spec.in_port):
        issues.append(f"missing destination port {spec.in_module}.{spec.in_port}")
    if issues:
        raise ValueError(f"invalid wire {spec.label()}: {'; '.join(issues)}")
    spec.apply(bp)


def _declares_port(graph: ModuleGraph, module_name: str, port_name: str) -> bool:
    module = graph.module_spec(module_name)
    return module.declares_port(port_name) if module is not None else False


def full_stack_wire_specs(
    module_names: set[str] | frozenset[str],
    *,
    driver_module: str,
    slam_profile: str,
    enable_semantic: bool = True,
) -> tuple[WireSpec, ...]:
    """Return full-stack wires whose source and destination Modules exist."""

    names = set(module_names)
    ctx = build_wiring_context(
        names,
        driver_module=driver_module,
        slam_profile=slam_profile,
    )

    specs: list[WireSpec] = []
    specs.extend(map_specs(ctx))
    specs.extend(scan_view_specs(ctx))

    if enable_semantic:
        specs.extend(semantic_camera_specs(ctx))

    specs.extend(localization_specs(ctx))
    specs.extend(semantic_command_specs(ctx))
    specs.extend(odometry_fanout_specs(ctx))
    specs.extend(semantic_scene_specs())
    specs.extend(recorder_specs(ctx))
    specs.extend(gateway_status_specs(ctx))
    specs.extend(navigation_support_specs())
    specs.extend(visual_servo_specs(ctx))
    specs.extend(vla_specs(ctx))
    specs.extend(teleop_media_specs(ctx))

    return tuple(spec for spec in specs if spec.out_module in names and spec.in_module in names)


def apply_full_stack_wires(
    bp: Blueprint,
    *,
    driver_module: str,
    slam_profile: str,
    enable_semantic: bool = True,
) -> Blueprint:
    """Apply explicit cross-stack wires to a composed full-stack Blueprint."""

    graph = bp.export_graph()
    names = set(graph.module_names)
    seen = {
        (wire.out_module, wire.out_port, wire.in_module, wire.in_port)
        for wire in graph.explicit_wires
    }
    for spec in full_stack_wire_specs(
        names,
        driver_module=driver_module,
        slam_profile=slam_profile,
        enable_semantic=enable_semantic,
    ):
        key = wire_key(spec)
        if key in seen:
            continue
        seen.add(key)
        _apply_wire(
            bp,
            graph,
            spec,
        )

    return bp
