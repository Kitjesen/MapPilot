"""Composition helpers for product-scale module stacks."""

from __future__ import annotations

import os
from typing import Any

from runtime.blueprint import Blueprint, autoconnect
from runtime.profiles.binding_policy import (
    endpoint_contract_for_config,
    localization_adapter_for_config,
)

from .driver import driver
from .exploration import exploration
from .gateway import gateway
from .lidar import lidar
from .maps import maps
from .memory import DEFAULT_SEMANTIC_DIR, memory
from .navigation import navigation
from .perception import perception
from .planner import planner as planner_stack
from .safety import safety
from .services import services
from .sim_lidar import sim_lidar
from .slam import slam
from .stack_config import (
    driver_stack_config,
    exploration_stack_config,
    needs_lidar_for_slam,
    perception_stack_config,
)
from .system import device_manager, external_services, gnss


def compose_full_stack_modules(
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    detector: str,
    encoder: str,
    llm: str,
    planner_backend: str,
    tomogram: str,
    gateway_port: int,
    teleop_port: int = 5050,
    enable_native: bool = False,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
    enable_navigation: bool = True,
    enable_rerun: bool = False,
    scene_xml: str = "",
    manage_external_services: bool = True,
    semantic_save_dir: str = DEFAULT_SEMANTIC_DIR,
    config: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose the standard LingTu/Thunder module graph before explicit wires.

    This helper keeps stack selection separate from the legacy full_stack entry
    point. It is intentionally side-effect free at Blueprint construction time;
    external services are represented by ExternalServiceManagerModule and run
    during system startup.
    """

    config = dict(config or {})
    enable_robot_driver = bool(config.get("enable_robot_driver", True))
    driver_config = driver_stack_config(
        config,
        slam_profile=slam_profile,
        driver_module=driver_module,
        enable_semantic=enable_semantic,
    )
    perception_config = perception_stack_config(config, driver_module=driver_module)
    exploration_config = exploration_stack_config(config)
    enable_gnss = _optional_bool(config.get("enable_gnss"))
    enable_device_manager = bool(config.get("enable_device_manager", True))
    enable_lidar = _optional_bool(config.get("enable_lidar"))
    if enable_lidar is None:
        enable_lidar = needs_lidar_for_slam(slam_profile)
    lidar_start_driver = bool(config.get("lidar_start_driver", False))
    services_config = dict(config)
    enable_goals = bool(services_config.pop("enable_goals", True))
    enable_patrol_routes = bool(services_config.pop("enable_patrol_routes", True))
    enable_scheduler = bool(services_config.pop("enable_scheduler", False))
    manage_session_services = _optional_bool(config.get("manage_session_services"))
    if manage_session_services is None:
        manage_session_services = _optional_bool(
            os.environ.get("LINGTU_MANAGE_SESSION_SERVICES")
        )
    if manage_session_services is None:
        manage_session_services = True

    return autoconnect(
        external_services(
            enabled=manage_external_services and enable_robot_driver,
            driver_module=driver_module,
            slam_profile=slam_profile,
            enable_semantic=enable_semantic,
            config=config,
        ),
        device_manager() if enable_device_manager else Blueprint(),
        driver(robot, **driver_config) if enable_robot_driver else Blueprint(),
        lidar(
            ip=config.get("lidar_ip"),
            enabled=enable_lidar,
            start_driver=lidar_start_driver,
        ),
        sim_lidar(scene_xml=scene_xml),
        slam(
            slam_profile,
            enable_visual_backup=bool(config.get("enable_visual_backup", True)),
            manage_services=False,
            localization_adapter=localization_adapter_for_config(config) or None,
            endpoint_contract=endpoint_contract_for_config(config) or None,
        ),
        gnss(enabled=enable_gnss),
        maps(**config) if enable_map_modules else Blueprint(),
        perception(
            detector,
            encoder,
            manage_services=False,
            **perception_config,
        )
        if enable_semantic
        else Blueprint(),
        memory(semantic_save_dir) if enable_semantic else Blueprint(),
        planner_stack(llm, semantic_save_dir) if enable_semantic else Blueprint(),
        services(
            enable_goals=enable_goals,
            enable_patrol_routes=enable_patrol_routes,
            enable_scheduler=enable_scheduler,
            enable_navigation=enable_navigation,
            **services_config,
        ),
        navigation(planner_backend, tomogram, enable_native, **config)
        if enable_navigation
        else Blueprint(),
        exploration(**exploration_config),
        safety(
            cmd_vel_mux_source_timeout=config.get("cmd_vel_mux_source_timeout"),
            enable_collision_monitor=bool(
                config.get("cmd_vel_mux_collision_monitor", False)
            ),
            collision_monitor_timeout_s=config.get(
                "cmd_vel_mux_collision_monitor_timeout_s"
            ),
            collision_monitor_horizon_s=config.get(
                "cmd_vel_mux_collision_monitor_horizon_s"
            ),
            collision_monitor_step_s=config.get(
                "cmd_vel_mux_collision_monitor_step_s"
            ),
            collision_monitor_stop_cost=config.get(
                "cmd_vel_mux_collision_monitor_stop_cost"
            ),
            collision_monitor_slow_cost=config.get(
                "cmd_vel_mux_collision_monitor_slow_cost"
            ),
            collision_monitor_slowdown_scale=config.get(
                "cmd_vel_mux_collision_monitor_slowdown_scale"
            ),
        ),
        gateway(
            gateway_port,
            mcp_port=config.get("mcp_port", 8090),
            teleop_port=teleop_port,
            enable_teleop=enable_teleop,
            enable_rerun=enable_rerun,
            enable_ros2_rerun_bridge=bool(
                config.get("enable_ros2_rerun_bridge", False)
            ),
            manage_session_services=manage_session_services,
        )
        if enable_gateway
        else Blueprint(),
    )


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off", ""}
    return bool(value)
