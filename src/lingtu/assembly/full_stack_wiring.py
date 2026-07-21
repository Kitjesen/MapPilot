"""Explicit cross-stack wiring aggregator for the full LingTu profile.

Subsystem wire bundles live under :mod:`lingtu.assembly.wires`. This module is
kept as the stable public entry point used by runtime builds and profile graph
tests.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.introspection.module_graph import ModuleGraph

from .wires.context import build_wiring_context
from .wires.gateway import gateway_command_specs, gateway_status_specs, teleop_media_specs
from .wires.mapping import map_output_specs, traversability_specs
from .wires.navigation import (
    exploration_specs,
    navigation_execution_specs,
    navigation_input_specs,
    navigation_service_specs,
    navigation_support_specs,
)
from .wires.safety import (
    cmd_vel_collision_monitor_specs,
    cmd_vel_mux_specs,
    required_safety_stop_specs,
    safety_status_specs,
)
from .wires.semantic import (
    recorder_specs,
    semantic_camera_specs,
    semantic_command_specs,
    semantic_scene_specs,
    visual_servo_specs,
    vla_specs,
)
from .wires.slam import (
    gnss_feed_specs,
    localization_specs,
    map_cloud_specs,
    odometry_fanout_specs,
    scan_view_specs,
    sensor_feed_specs,
)
from .wires.types import WireSpec, wire_key

logger = logging.getLogger(__name__)


def _wire_if_present(bp: Blueprint, graph: ModuleGraph, names: set[str], spec: WireSpec) -> None:
    if (
        spec.out_module in names
        and spec.in_module in names
        and _declares_port(graph, spec.out_module, spec.out_port)
        and _declares_port(graph, spec.in_module, spec.in_port)
    ):
        spec.apply(bp)


def _wire_contract_issues(graph: ModuleGraph, names: set[str], spec: WireSpec) -> list[str]:
    issues: list[str] = []
    if spec.out_module not in names:
        issues.append(f"missing source module {spec.out_module}")
    elif not _declares_port(graph, spec.out_module, spec.out_port):
        issues.append(f"missing source port {spec.out_module}.{spec.out_port}")

    if spec.in_module not in names:
        issues.append(f"missing destination module {spec.in_module}")
    elif not _declares_port(graph, spec.in_module, spec.in_port):
        issues.append(f"missing destination port {spec.in_module}.{spec.in_port}")
    return issues


def _require_wire(bp: Blueprint, graph: ModuleGraph, names: set[str], spec: WireSpec) -> None:
    issues = _wire_contract_issues(graph, names, spec)
    if issues:
        raise ValueError(f"Required full-stack wire unavailable: {spec.label()} ({'; '.join(issues)})")
    spec.apply(bp)


def _declares_port(graph: ModuleGraph, module_name: str, port_name: str) -> bool:
    module = graph.module_spec(module_name)
    return module.declares_port(port_name) if module is not None else False


def full_stack_wire_specs(
    module_names: set[str] | frozenset[str],
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
    safety_stop_wiring: bool = True,
    cmd_vel_mux_collision_monitor: bool = False,
    legacy_driver_sensor_fallback: bool = False,
    nav_plan_transport: str | None = None,
) -> tuple[WireSpec, ...]:
    """Return module-name-filtered full-stack wire specs.

    Static profile graph snapshots and runtime Blueprint application share this
    source of truth. Runtime application still validates concrete port
    declarations and skips optional wires whose modules do not expose a port.
    """

    names = set(module_names)
    ctx = build_wiring_context(
        names,
        robot=robot,
        driver_module=driver_module,
        slam_profile=slam_profile,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
        legacy_driver_sensor_fallback=legacy_driver_sensor_fallback,
    )

    specs: list[WireSpec] = []
    specs.extend(map_cloud_specs(ctx))
    specs.extend(scan_view_specs(ctx))
    specs.extend(sensor_feed_specs(ctx))
    specs.extend(gnss_feed_specs(ctx))

    if enable_semantic:
        specs.extend(semantic_camera_specs(ctx))

    if safety_stop_wiring:
        specs.extend(required_safety_stop_specs(names, driver_module=ctx.driver_module))

    specs.extend(gateway_command_specs(ctx))
    specs.extend(localization_specs(ctx))
    specs.extend(semantic_command_specs(ctx))
    specs.extend(exploration_specs(ctx))
    specs.extend(navigation_service_specs())
    specs.extend(navigation_input_specs(ctx))
    specs.extend(odometry_fanout_specs(ctx))
    specs.extend(traversability_specs())
    specs.extend(map_output_specs())
    specs.extend(semantic_scene_specs())
    specs.extend(recorder_specs(ctx))
    specs.extend(safety_status_specs())
    specs.extend(gateway_status_specs())
    specs.extend(navigation_execution_specs())
    specs.extend(navigation_support_specs())
    specs.extend(visual_servo_specs())
    specs.extend(vla_specs(ctx))
    specs.extend(teleop_media_specs(ctx))
    if cmd_vel_mux_collision_monitor:
        specs.extend(cmd_vel_collision_monitor_specs(ctx))
    specs.extend(cmd_vel_mux_specs(ctx))

    return tuple(spec for spec in specs if spec.out_module in names and spec.in_module in names)


def apply_full_stack_wires(
    bp: Blueprint,
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
    safety_stop_wiring: bool = True,
    cmd_vel_mux_collision_monitor: bool = False,
    legacy_driver_sensor_fallback: bool = False,
    nav_plan_transport: str | None = None,
) -> Blueprint:
    """Apply explicit cross-stack wires to a composed full-stack Blueprint."""

    graph = bp.export_graph()
    names = set(graph.module_names)
    required_wire_keys: set[tuple[str, str, str, str]] = set()
    if safety_stop_wiring:
        required_wire_keys = {
            wire_key(spec)
            for spec in required_safety_stop_specs(
                names,
                driver_module=driver_module,
            )
        }

    seen: set[tuple[str, str, str, str]] = {
        (wire.out_module, wire.out_port, wire.in_module, wire.in_port) for wire in bp._wires
    }
    for spec in full_stack_wire_specs(
        names,
        robot=robot,
        driver_module=driver_module,
        slam_profile=slam_profile,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
        safety_stop_wiring=safety_stop_wiring,
        cmd_vel_mux_collision_monitor=cmd_vel_mux_collision_monitor,
        legacy_driver_sensor_fallback=legacy_driver_sensor_fallback,
        nav_plan_transport=nav_plan_transport,
    ):
        key = wire_key(spec)
        if key in seen:
            continue
        seen.add(key)
        if key in required_wire_keys:
            _require_wire(bp, graph, names, spec)
        else:
            _wire_if_present(bp, graph, names, spec)

    # Runtime-only driver output: some sim drivers expose goal_pose directly,
    # but checking that requires registry/class introspection. Keep it out of
    # full_stack_wire_specs() so static graph compilation stays dependency-light.
    try:
        from runtime.registry import get as get_plugin

        if hasattr(get_plugin("driver", robot), "goal_pose"):
            _wire_if_present(
                bp,
                graph,
                names,
                WireSpec(driver_module, "goal_pose", "nav.mission", "goal_pose"),
            )
    except Exception:
        logger.debug("apply_full_stack_wires: driver goal_pose wire skipped", exc_info=True)

    return bp
