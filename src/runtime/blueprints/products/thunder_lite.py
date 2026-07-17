"""Thunder Lite product composition.

This module is intentionally independent from the full-stack composition and
wire aggregators. Thunder Lite is the local, lightweight product surface: it
keeps the robot driver, Python navigation chain, safety ring, and velocity
mux, without SLAM, maps, semantic modules, Gateway, or ROS compatibility.
"""

from __future__ import annotations

from typing import Any

from runtime.blueprint import Blueprint, autoconnect
from runtime.blueprints.stacks.driver import driver
from runtime.blueprints.stacks.navigation import navigation
from runtime.blueprints.stacks.safety import safety
from runtime.blueprints.stacks.stack_config import driver_stack_config


def compose_thunder_lite_modules(
    *,
    robot: str,
    driver_module: str,
    planner_backend: str,
    map_path: str,
    enable_native: bool,
    config: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose the Thunder Lite runtime graph before explicit wires."""

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
        navigation(planner_backend, map_path, enable_native, **cfg),
        safety(cmd_vel_mux_source_timeout=cfg.get("cmd_vel_mux_source_timeout")),
    )


def apply_thunder_lite_wires(
    bp: Blueprint,
    *,
    driver_module: str,
    safety_stop_wiring: bool = True,
) -> Blueprint:
    """Apply the explicit control and safety wires for Thunder Lite."""

    names = set(bp.export_graph().module_names)

    def wire_if_present(
        out_module: str,
        out_port: str,
        in_module: str,
        in_port: str,
    ) -> None:
        if out_module in names and in_module in names:
            bp.wire(
                out_module,
                out_port,
                in_module,
                in_port,
            )

    if safety_stop_wiring:
        wire_if_present("nav.safety", "stop_cmd", "nav.mission", "stop_signal")
        wire_if_present("nav.safety", "stop_cmd", driver_module, "stop_signal")
        wire_if_present("GeofenceManagerModule", "stop_cmd", "nav.mission", "stop_signal")
        wire_if_present("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal")

    for consumer in (
        "nav.mission",
        "nav.terrain",
        "nav.local_planner",
        "nav.path_follower",
        "nav.safety",
        "GeofenceManagerModule",
    ):
        wire_if_present(driver_module, "odometry", consumer, "odometry")

    for spec in (
        ("nav.mission", "global_path", "nav.local_planner", "global_path"),
        ("nav.mission", "waypoint", "nav.local_planner", "waypoint"),
        ("nav.mission", "clear_path", "nav.local_planner", "clear_path"),
        ("nav.terrain", "terrain_map", "nav.local_planner", "terrain_map"),
        ("nav.terrain", "terrain_map_ext", "nav.local_planner", "terrain_map_ext"),
        ("nav.terrain", "traversability", "nav.local_planner", "traversability"),
        ("nav.local_planner", "local_path", "nav.path_follower", "local_path"),
        ("nav.local_planner", "local_path", "nav.safety", "path"),
        ("nav.local_planner", "control_hint", "nav.path_follower", "control_hint"),
    ):
        wire_if_present(*spec)

    wire_if_present("nav.mission", "recovery_cmd_vel", "nav.velocity_mux", "recovery_cmd_vel")
    wire_if_present("nav.path_follower", "cmd_vel", "nav.velocity_mux", "path_follower_cmd_vel")
    wire_if_present("nav.velocity_mux", "driver_cmd_vel", driver_module, "cmd_vel")
    wire_if_present("nav.velocity_mux", "driver_cmd_vel", "nav.safety", "cmd_vel")

    return bp
