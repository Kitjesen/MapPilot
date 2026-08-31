"""Composition helpers for product-scale module stacks."""

from __future__ import annotations

from typing import Any

from lingtu.assembly.binding_policy import (
    endpoint_contract_for_config,
    localization_adapter_for_config,
)
from runtime.blueprint import Blueprint, autoconnect

from .driver import driver
from .exploration import exploration
from .gateway import gateway
from .lidar import lidar
from .memory import DEFAULT_SEMANTIC_DIR, memory
from .navigation import navigation
from .perception import camera as camera_stack
from .perception import perception
from .planner import planner as planner_stack
from .services import services
from .slam import slam
from .stack_config import (
    driver_stack_config,
    exploration_stack_config,
    needs_lidar_for_slam,
    perception_stack_config,
)


def compose_full_stack_modules(
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    detector: str,
    encoder: str,
    llm: str,
    gateway_port: int,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_navigation: bool = True,
    semantic_save_dir: str = DEFAULT_SEMANTIC_DIR,
    config: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose the standard LingTu/Thunder module graph before explicit wires.

    This helper is side-effect free. It constructs only the Python Host graph;
    ProductControl owns field process lifecycle through its internal
    SystemdRunner.
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
    enable_lidar = _optional_bool(config.get("enable_lidar"))
    if enable_lidar is None:
        enable_lidar = needs_lidar_for_slam(slam_profile)
    lidar_backend = str(
        config.get(
            "lidar_backend",
            "mujoco" if driver_module == "MujocoDriverModule" else "mid360",
        )
    )
    services_config = dict(config)
    enable_goals = bool(services_config.pop("enable_goals", True))
    enable_semantic_planning = bool(
        config.get("enable_semantic_planning", enable_semantic)
    )
    return autoconnect(
        driver(robot, **driver_config) if enable_robot_driver else Blueprint(),
        lidar(
            ip=config.get("lidar_ip"),
            enabled=enable_lidar,
            backend=lidar_backend,
        ),
        slam(
            slam_profile,
            localization_adapter=localization_adapter_for_config(config) or None,
            endpoint_contract=endpoint_contract_for_config(config) or None,
        ),
        camera_stack(**perception_config)
        if not enable_semantic and bool(perception_config.get("enable_camera", False))
        else Blueprint(),
        perception(
            detector,
            encoder,
            **perception_config,
        )
        if enable_semantic
        else Blueprint(),
        memory(semantic_save_dir) if enable_semantic_planning else Blueprint(),
        planner_stack(
            llm,
            semantic_save_dir,
            enable_semantic_planning=enable_semantic_planning,
        )
        if enable_semantic
        else Blueprint(),
        services(
            enable_goals=enable_goals,
            **services_config,
        ),
        navigation() if enable_navigation else Blueprint(),
        exploration(**exploration_config),
        gateway(
            gateway_port,
            mcp_port=config.get("mcp_port", 8090),
            enable_teleop=enable_teleop,
            enable_camera=bool(perception_config.get("enable_camera", False)),
            command_output_mode=config.get("command_output_mode"),
            hardware_control_boundary=config.get("hardware_control_boundary"),
            product=config.get("_product"),
            run_plan=config.get("_run_plan"),
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
