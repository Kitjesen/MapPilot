"""Thunder product blueprints.

These functions give the real quadruped stack a product-level entry point
without naming it after the board. Existing CLI profiles can keep their
compatibility names while new architecture work targets Thunder directly.
"""

from __future__ import annotations

import os
from typing import Any, Mapping

from core.blueprint import Blueprint
from core.blueprints.catalog.runtime_paths import RUNTIME_MAP_FRAME_ID, _resolve_tomogram
from core.runtime_policy import normalize_slam_profile

DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")

_THUNDER_BASE_CONFIG: dict[str, Any] = {
    "robot": "thunder",
    "slam_profile": "localizer",
    "detector": "bpu",
    "encoder": "mobileclip",
    "dog_host": "127.0.0.1",
    "dog_port": 13145,
    "auto_enable": False,
    "auto_standup": False,
    "planning_frame_id": RUNTIME_MAP_FRAME_ID,
    "gateway_port": 5050,
}


def _with_overrides(config: dict[str, Any], **overrides: Any) -> dict[str, Any]:
    resolved = dict(config)
    resolved.update({k: v for k, v in overrides.items() if v is not None})
    return resolved


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() not in {"", "0", "false", "no", "off"}
    return bool(value)


def _driver_name(robot: str) -> str:
    from lingtu_runtime.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()

    from core.blueprints.stacks.driver import driver_name

    return driver_name(robot)


def _run_startup_preflight(*, enable_semantic: bool, slam_profile: str) -> None:
    from core.blueprints.stacks.system import run_startup_preflight

    run_startup_preflight(
        enable_semantic=enable_semantic,
        slam_profile=slam_profile,
    )


def _is_lite_runtime(
    *,
    slam_profile: str,
    enable_semantic: bool,
    enable_gateway: bool,
    enable_teleop: bool,
    enable_map_modules: bool,
    enable_rerun: bool,
    manage_external_services: bool,
    scene_xml: str,
    config: Mapping[str, Any],
) -> bool:
    if slam_profile not in {"", "none"}:
        return False
    if any(
        (
            enable_semantic,
            enable_gateway,
            enable_teleop,
            enable_map_modules,
            enable_rerun,
            manage_external_services,
            bool(scene_xml),
        )
    ):
        return False
    if _optional_bool(config.get("enable_gnss")) is True:
        return False
    if str(config.get("exploration_backend", "none") or "none") != "none":
        return False
    return not any(
        bool(config.get(key))
        for key in (
            "enable_frontier",
            "enable_traversable_frontier",
            "enable_endpoint_command_bridge",
            "enable_endpoint_waypoint_bridge",
            "enable_endpoint_path_bridge",
            "enable_endpoint_grid_bridge",
            "enable_ros2_command_bridge",
            "enable_ros2_bridge",
            "enable_ros2_path_bridge",
            "enable_ros2_grid_bridge",
        )
    )


def _blueprint(config: dict[str, Any]) -> Blueprint:
    cfg = dict(config)
    robot = str(cfg.pop("robot", "thunder"))
    slam_profile = normalize_slam_profile(cfg.pop("slam_profile", "fastlio2"))
    detector = str(cfg.pop("detector", "bpu"))
    encoder = str(cfg.pop("encoder", "mobileclip"))
    llm = str(cfg.pop("llm", "qwen"))
    planner_backend_default = cfg.pop("planner_backend", "pct")
    planner_backend = str(cfg.pop("planner", "") or planner_backend_default)
    tomogram = str(cfg.pop("tomogram", ""))
    gateway_port = int(cfg.pop("gateway_port", 5050))
    teleop_port = int(cfg.pop("teleop_port", gateway_port))
    enable_native = bool(cfg.pop("enable_native", False))
    enable_semantic = bool(cfg.pop("enable_semantic", True))
    enable_gateway = bool(cfg.pop("enable_gateway", True))
    enable_teleop = bool(cfg.pop("enable_teleop", True))
    enable_map_modules = bool(cfg.pop("enable_map_modules", True))
    enable_rerun = bool(cfg.pop("enable_rerun", False))
    enable_swap = bool(cfg.pop("enable_swap", False))
    swap_mux_name = str(cfg.pop("swap_mux_name", "CmdVelMux"))
    swap_nav_name = str(cfg.pop("swap_nav_name", "NavigationModule"))
    scene_xml = str(cfg.pop("scene_xml", ""))
    run_startup_checks = bool(cfg.pop("run_startup_checks", True))
    manage_external_services = bool(cfg.pop("manage_external_services", True))
    namespace = cfg.pop("namespace", None)
    semantic_save_dir = str(cfg.get("semantic_save_dir", DEFAULT_SEMANTIC_DIR))

    driver_module = _driver_name(robot)
    if run_startup_checks:
        _run_startup_preflight(
            enable_semantic=enable_semantic,
            slam_profile=slam_profile,
        )

    if _is_lite_runtime(
        slam_profile=slam_profile,
        enable_semantic=enable_semantic,
        enable_gateway=enable_gateway,
        enable_teleop=enable_teleop,
        enable_map_modules=enable_map_modules,
        enable_rerun=enable_rerun,
        manage_external_services=manage_external_services,
        scene_xml=scene_xml,
        config=cfg,
    ):
        from core.blueprints.products.thunder_lite import (
            apply_thunder_lite_wires,
            compose_thunder_lite_modules,
        )

        bp = compose_thunder_lite_modules(
            robot=robot,
            driver_module=driver_module,
            planner_backend=planner_backend,
            tomogram=tomogram,
            enable_native=enable_native,
            config=cfg,
        )
        bp = apply_thunder_lite_wires(
            bp,
            driver_module=driver_module,
            safety_stop_wiring=bool(cfg.get("safety_stop_wiring", True)),
        )
    else:
        from core.blueprints.full_stack_wiring import apply_full_stack_wires
        from core.blueprints.stacks.composition import compose_full_stack_modules

        bp = compose_full_stack_modules(
            robot=robot,
            driver_module=driver_module,
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
            enable_rerun=enable_rerun,
            scene_xml=scene_xml,
            manage_external_services=manage_external_services,
            semantic_save_dir=semantic_save_dir,
            config=cfg,
        )
        bp = apply_full_stack_wires(
            bp,
            robot=robot,
            driver_module=driver_module,
            slam_profile=slam_profile,
            scene_xml=scene_xml,
            enable_semantic=enable_semantic,
            safety_stop_wiring=bool(cfg.get("safety_stop_wiring", True)),
        )

    if enable_swap:
        bp._swap_config = {
            "mux_name": swap_mux_name,
            "nav_name": swap_nav_name,
            "driver_name": driver_module,
        }

    if namespace:
        bp.namespace(str(namespace))

    return bp


def thunder_blueprint(
    config: Mapping[str, Any] | None = None,
    **overrides: Any,
) -> Blueprint:
    """Build a Thunder product blueprint from an already resolved config."""

    resolved = dict(config or {})
    resolved.update({k: v for k, v in overrides.items() if v is not None})
    return _blueprint(resolved)


def thunder_basic_config(**overrides: Any) -> dict[str, Any]:
    """Return the minimal Thunder driver/navigation product config."""

    return _with_overrides(
        {
            **_THUNDER_BASE_CONFIG,
            "slam_profile": "none",
            "llm": "mock",
            "planner": "astar",
            "enable_native": False,
            "python_autonomy_backend": "simple",
            "python_path_follower_backend": "pid",
            "enable_semantic": False,
            "enable_gateway": False,
            "enable_teleop": False,
            "enable_map_modules": False,
            "enable_gnss": False,
            "manage_external_services": False,
            "run_startup_checks": False,
        },
        **overrides,
    )


def thunder_lite_config(**overrides: Any) -> dict[str, Any]:
    """Return the lightweight Thunder local product config."""

    return thunder_basic_config(**overrides)


def thunder_map_config(**overrides: Any) -> dict[str, Any]:
    """Return the Thunder mapping product config."""

    return _with_overrides(
        {
            **_THUNDER_BASE_CONFIG,
            "slam_profile": "fastlio2",
            "llm": "mock",
            "planner": "astar",
            "enable_native": False,
            "enable_semantic": False,
            "enable_gateway": True,
            "enable_map_modules": True,
        },
        **overrides,
    )


def thunder_nav_config(**overrides: Any) -> dict[str, Any]:
    """Return the Thunder saved-map navigation product config."""

    return _with_overrides(
        {
            **_THUNDER_BASE_CONFIG,
            "llm": "qwen",
            "planner": "pct",
            "tomogram": _resolve_tomogram(),
            "plan_safety_policy": "fallback_astar",
            "fallback_planner_name": "astar",
            "enable_native": False,
            "enable_semantic": True,
            "enable_gateway": True,
        },
        **overrides,
    )


def thunder_explore_config(**overrides: Any) -> dict[str, Any]:
    """Return the Thunder frontier-exploration product config."""

    return _with_overrides(
        {
            **_THUNDER_BASE_CONFIG,
            "slam_profile": "fastlio2",
            "llm": "qwen",
            "planner": "pct",
            "plan_safety_policy": "fallback_astar",
            "fallback_planner_name": "astar",
            "enable_native": False,
            "enable_semantic": True,
            "enable_gateway": True,
            "enable_frontier": True,
            "enable_traversable_frontier": True,
            "exploration_backend": "none",
        },
        **overrides,
    )


def thunder_map_blueprint(**overrides: Any) -> Blueprint:
    return _blueprint(thunder_map_config(**overrides))


def thunder_basic_blueprint(**overrides: Any) -> Blueprint:
    return _blueprint(thunder_basic_config(**overrides))


def thunder_lite_blueprint(**overrides: Any) -> Blueprint:
    return _blueprint(thunder_lite_config(**overrides))


def thunder_nav_blueprint(**overrides: Any) -> Blueprint:
    return _blueprint(thunder_nav_config(**overrides))


def thunder_explore_blueprint(**overrides: Any) -> Blueprint:
    return _blueprint(thunder_explore_config(**overrides))
