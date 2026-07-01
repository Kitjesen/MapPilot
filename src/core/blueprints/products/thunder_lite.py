"""Thunder Lite product composition.

This module is intentionally independent from the full-stack composition and
wire aggregators. Thunder Lite is the local, lightweight product surface: it
keeps the robot driver, Python navigation chain, safety ring, and velocity
mux, without SLAM, maps, semantic modules, Gateway, or ROS compatibility.
"""

from __future__ import annotations

from typing import Any

from core.blueprint import Blueprint, autoconnect
from core.blueprints.stacks.driver import driver
from core.blueprints.stacks.navigation import navigation
from core.blueprints.stacks.safety import safety
from core.blueprints.stacks.stack_config import driver_stack_config


def compose_thunder_lite_modules(
    *,
    robot: str,
    driver_module: str,
    planner_backend: str,
    tomogram: str,
    enable_native: bool,
    config: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose the Thunder Lite runtime graph before explicit wires."""

    from lingtu_runtime.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()

    cfg = dict(config or {})
    enable_robot_driver = bool(cfg.get("enable_robot_driver", True))
    driver_config = driver_stack_config(
        cfg,
        slam_profile="none",
        driver_module=driver_module,
        enable_semantic=False,
    )

    return autoconnect(
        driver(robot, **driver_config) if enable_robot_driver else Blueprint(),
        navigation(planner_backend, tomogram, enable_native, **cfg),
        safety(cmd_vel_mux_source_timeout=cfg.get("cmd_vel_mux_source_timeout")),
    )


def apply_thunder_lite_wires(
    bp: Blueprint,
    *,
    driver_module: str,
    safety_stop_wiring: bool = True,
) -> Blueprint:
    """Apply the explicit control and safety wires for Thunder Lite."""

    names = {entry.name for entry in bp._entries}

    def wire_if_present(
        out_module: str,
        out_port: str,
        in_module: str,
        in_port: str,
    ) -> None:
        if out_module in names and in_module in names:
            bp.wire(out_module, out_port, in_module, in_port)

    if safety_stop_wiring:
        wire_if_present("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")
        wire_if_present("SafetyRingModule", "stop_cmd", driver_module, "stop_signal")
        wire_if_present("GeofenceManagerModule", "stop_cmd", "NavigationModule", "stop_signal")
        wire_if_present("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal")

    for consumer in (
        "NavigationModule",
        "TerrainModule",
        "LocalPlannerModule",
        "PathFollowerModule",
        "SafetyRingModule",
        "GeofenceManagerModule",
    ):
        wire_if_present(driver_module, "odometry", consumer, "odometry")

    wire_if_present("NavigationModule", "global_path", "LocalPlannerModule", "global_path")
    wire_if_present("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
    wire_if_present("NavigationModule", "clear_path", "LocalPlannerModule", "clear_path")
    wire_if_present("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
    wire_if_present("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
    wire_if_present("LocalPlannerModule", "local_path", "SafetyRingModule", "path")
    wire_if_present("LocalPlannerModule", "control_hint", "PathFollowerModule", "control_hint")

    wire_if_present("NavigationModule", "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
    wire_if_present("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel")
    wire_if_present("CmdVelMux", "driver_cmd_vel", driver_module, "cmd_vel")
    wire_if_present("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel")

    return bp
