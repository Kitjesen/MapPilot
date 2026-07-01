"""Composition helpers for product-scale module stacks."""

from __future__ import annotations

from typing import Any

from core.blueprint import Blueprint, autoconnect

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
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
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

    return autoconnect(
        external_services(
            enabled=manage_external_services and enable_robot_driver,
            driver_module=driver_module,
            slam_profile=slam_profile,
            enable_semantic=enable_semantic,
            config=config,
        ),
        device_manager(),
        driver(robot, **driver_config) if enable_robot_driver else Blueprint(),
        lidar(ip=config.get("lidar_ip"), enabled=needs_lidar_for_slam(slam_profile)),
        sim_lidar(scene_xml=scene_xml),
        slam(
            slam_profile,
            manage_services=False,
            localization_adapter=_localization_adapter_for_config(config),
            endpoint_contract=_endpoint_contract_for_config(config),
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
        navigation(planner_backend, tomogram, enable_native, **config),
        exploration(**exploration_config),
        safety(cmd_vel_mux_source_timeout=config.get("cmd_vel_mux_source_timeout")),
        gateway(
            gateway_port,
            mcp_port=config.get("mcp_port", 8090),
            teleop_port=teleop_port,
            enable_teleop=enable_teleop,
            enable_rerun=enable_rerun,
        )
        if enable_gateway
        else Blueprint(),
    )


def _localization_adapter_for_config(config: dict[str, Any]) -> str | None:
    explicit = config.get("localization_adapter") or config.get("_localization_adapter")
    if explicit:
        return str(explicit)
    endpoint_transport = str(config.get("endpoint_transport") or config.get("_endpoint_transport") or "")
    endpoint_contract = _endpoint_contract_for_config(config)
    if endpoint_transport.lower() == "lcm" and endpoint_contract:
        return "lcm_endpoint"
    return None


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off", ""}
    return bool(value)


def _endpoint_contract_for_config(config: dict[str, Any]) -> str | None:
    value = config.get("endpoint_contract") or config.get("_endpoint_contract")
    return str(value).strip() if value else None
