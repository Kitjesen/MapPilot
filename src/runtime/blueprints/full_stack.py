"""Full-stack navigation blueprint - composable factory pattern.

Usage::

    # dimos-style one-liner per stack
    system = full_stack_blueprint(robot="thunder", slam_profile="localizer").build()
    system.start()

    # Or compose directly
    from runtime.blueprints.stacks import *
    system = autoconnect(
        driver("thunder", host="192.168.66.190"),
        slam("localizer"),
        maps(),
        perception("bpu"),
        memory(),
        planner("kimi"),
        navigation("octoplanner3d"),
        safety(),
        gateway(5050),
    ).build()
"""

from __future__ import annotations

from typing import Any

from runtime.blueprint import Blueprint

from .full_stack_wiring import apply_full_stack_wires
from .stacks.composition import compose_full_stack_modules
from .stacks.driver import driver_name
from .stacks.memory import DEFAULT_SEMANTIC_DIR
from .stacks.slam import normalize_slam_profile
from .stacks.system import (
    external_services as _external_services_blueprint,
    run_startup_preflight as _run_startup_preflight,
)


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner_backend: str = "octoplanner3d",
    tomogram: str = "",
    gateway_port: int = 5050,
    teleop_port: int = 5050,  # teleop is now on /ws/teleop of the main gateway port
    enable_native: bool = False,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
    enable_navigation: bool = True,
    enable_rerun: bool = False,
    enable_swap: bool = False,
    swap_mux_name: str = "nav.velocity_mux",
    swap_nav_name: str = "nav.mission",
    scene_xml: str = "",
    run_startup_checks: bool = True,
    manage_external_services: bool = True,
    namespace: str | None = None,
    # Legacy alias
    planner: str = "",
    **config: Any,
) -> Blueprint:
    """Build the complete LingTu navigation stack from composable factories.

    Each stack is a factory function returning a Blueprint.
    autoconnect() merges them and auto-wires by (port_name, msg_type).
    """
    planner_backend = planner or planner_backend
    slam_profile = normalize_slam_profile(slam_profile)
    semantic_save_dir = config.get("semantic_save_dir", DEFAULT_SEMANTIC_DIR)
    drv = driver_name(robot)

    if run_startup_checks:
        _run_startup_preflight(
            enable_semantic=enable_semantic,
            slam_profile=slam_profile,
        )

    bp = compose_full_stack_modules(
        robot=robot,
        driver_module=drv,
        slam_profile=slam_profile,
        detector=detector,
        encoder=encoder,
        llm=llm,
        planner_backend=planner_backend,
        tomogram=tomogram,
        gateway_port=gateway_port,
        teleop_port=teleop_port,
        enable_native=enable_native,
        enable_semantic=enable_semantic,
        enable_gateway=enable_gateway,
        enable_teleop=enable_teleop,
        enable_map_modules=enable_map_modules,
        enable_navigation=enable_navigation,
        enable_rerun=enable_rerun,
        scene_xml=scene_xml,
        manage_external_services=manage_external_services,
        semantic_save_dir=semantic_save_dir,
        config=config,
    )

    bp = apply_full_stack_wires(
        bp,
        robot=robot,
        driver_module=drv,
        slam_profile=slam_profile,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
        safety_stop_wiring=bool(config.get("safety_stop_wiring", True)),
        nav_plan_transport=(
            config.get("nav_plan_transport")
            if "nav_plan_transport" in config
            else config.get("local_planner_transport")
        ),
    )

    # Opt-in swap manager: enables hot-swap of navigation backends at runtime.
    # Stored on the Blueprint so build() can create the SwapManager automatically
    # without changing the caller's .build() step.
    if enable_swap:
        bp._swap_config = {
            "mux_name": swap_mux_name,
            "nav_name": swap_nav_name,
            "driver_name": drv,
        }

    if namespace:
        bp.namespace(namespace)

    return bp
