"""Thunder product blueprints.

These functions give the real quadruped stack a product-level entry point
without naming it after the board. Existing CLI profiles can keep their
compatibility names while new architecture work targets Thunder directly.
"""

from __future__ import annotations

import os
from typing import Any, Mapping

from runtime.blueprint import Blueprint
from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.profiles.binding_policy import (
    LEGACY_NAV_IN_ENABLE_KEYS,
    LEGACY_NAV_OUT_ENABLE_KEYS,
    LEGACY_MAP_OUT_ENABLE_KEYS,
    MAP_OUT_ENABLE_KEYS,
    NAV_IN_ENABLE_KEYS,
    NAV_OUT_ENABLE_KEYS,
)
from runtime.runtime_policy import normalize_slam_profile

DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")

_THUNDER_BASE_CONFIG: dict[str, Any] = {
    "robot": "thunder",
    "slam_profile": "localizer",
    "localization_adapter": "cpp_slam_status",
    "detector": "bpu",
    "encoder": "mobileclip",
    "dog_host": "127.0.0.1",
    "dog_port": 13145,
    "auto_enable": False,
    "auto_standup": False,
    "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
    "gateway_port": DEFAULT_GATEWAY_PORT,
    "preview_timeout": 30.0,
}
_THUNDER_OCTOPLANNER3D_CONSTRAINTS: dict[str, Any] = {
    "octoplanner3d_timeout_s": 30.0,
    "octoplanner3d_robot_radius": 0.25,
    "octoplanner3d_max_iterations": 500000,
    "octoplanner3d_snap_search_radius_cells": 12,
    "octoplanner3d_require_ground_support": True,
    "octoplanner3d_strict_direct_ground_support": False,
    "octoplanner3d_ground_support_xy_radius_cells": 1,
    "octoplanner3d_ground_support_depth_cells": 1,
    "octoplanner3d_enable_preblocked_costmap": True,
    "octoplanner3d_preblocked_costmap_radius_cells": 3,
    "octoplanner3d_preblocked_costmap_weight": 2.5,
    "octoplanner3d_lowest_traversable_only": False,
    "octoplanner3d_floor_change_penalty": 6.0,
    "octoplanner3d_max_step_height": 0.45,
    "octoplanner3d_max_slope": 0.0,
    "octoplanner3d_same_floor_preference": True,
    "octoplanner3d_same_floor_z_tolerance": 0.75,
    "octoplanner3d_max_same_floor_z_excursion": 2.0,
    "octoplanner3d_obstacle_clearance_radius_cells": 2,
    "octoplanner3d_obstacle_clearance_weight": 1.5,
}
_LITE_IGNORED_GRAPH_KEYS = (
    "enable_frontier",
    "enable_traversable_frontier",
    *NAV_IN_ENABLE_KEYS,
    *NAV_OUT_ENABLE_KEYS,
    *LEGACY_NAV_IN_ENABLE_KEYS,
    *LEGACY_NAV_OUT_ENABLE_KEYS,
    *MAP_OUT_ENABLE_KEYS,
    *LEGACY_MAP_OUT_ENABLE_KEYS,
    "enable_ros2_camera_bridge",
    "enable_ros2_rerun_bridge",
    "exploration_backend",
)


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


def _lite_graph_config(config: Mapping[str, Any]) -> dict[str, Any]:
    lite_config = dict(config)
    for key in _LITE_IGNORED_GRAPH_KEYS:
        lite_config.pop(key, None)
    return lite_config


def _driver_name(robot: str) -> str:
    from runtime.blueprints.stacks.driver import driver_name

    return driver_name(robot)


def _run_startup_preflight(*, enable_semantic: bool, slam_profile: str) -> None:
    from runtime.blueprints.stacks.system import run_startup_preflight

    run_startup_preflight(
        enable_semantic=enable_semantic,
        slam_profile=slam_profile,
    )


def _is_lite_runtime(
    *,
    runtime_mode: str,
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
    normalized_mode = runtime_mode.strip().lower()
    if normalized_mode == "lite":
        return True
    if normalized_mode and normalized_mode not in {"auto", "legacy"}:
        return False

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
            *NAV_IN_ENABLE_KEYS,
            *NAV_OUT_ENABLE_KEYS,
            *LEGACY_NAV_IN_ENABLE_KEYS,
            *LEGACY_NAV_OUT_ENABLE_KEYS,
            *MAP_OUT_ENABLE_KEYS,
            *LEGACY_MAP_OUT_ENABLE_KEYS,
            "enable_ros2_camera_bridge",
            "enable_ros2_rerun_bridge",
        )
    )


def _lite_runtime_incompatibilities(
    *,
    slam_profile: str,
    enable_semantic: bool,
    enable_gateway: bool,
    enable_teleop: bool,
    enable_map_modules: bool,
    enable_navigation: bool,
    enable_rerun: bool,
    manage_external_services: bool,
    run_startup_checks: bool,
    scene_xml: str,
    config: Mapping[str, Any],
) -> tuple[str, ...]:
    problems: list[str] = []
    if slam_profile not in {"", "none"}:
        problems.append(f"slam_profile={slam_profile!r}")
    if enable_semantic:
        problems.append("enable_semantic=True")
    if enable_gateway:
        problems.append("enable_gateway=True")
    if enable_teleop:
        problems.append("enable_teleop=True")
    if enable_map_modules:
        problems.append("enable_map_modules=True")
    # enable_navigation is intentionally not checked here: Thunder Lite keeps
    # the local nav.mission/local_planner/path_follower autonomy chain (see
    # test_resolved_thunder_lite_profile_builds_only_local_lite_control_graph);
    # it only strips the field/endpoint and semantic layers. _is_lite_runtime's
    # auto-detection heuristic mirrors this by omitting enable_navigation too.
    if enable_rerun:
        problems.append("enable_rerun=True")
    if manage_external_services:
        problems.append("manage_external_services=True")
    if run_startup_checks:
        problems.append("run_startup_checks=True")
    if scene_xml:
        problems.append("scene_xml is set")
    if _optional_bool(config.get("enable_gnss")) is True:
        problems.append("enable_gnss=True")
    if str(config.get("exploration_backend", "none") or "none") != "none":
        problems.append(f"exploration_backend={config.get('exploration_backend')!r}")
    for key in (
        "enable_frontier",
        "enable_traversable_frontier",
        *NAV_IN_ENABLE_KEYS,
        *NAV_OUT_ENABLE_KEYS,
        *LEGACY_NAV_IN_ENABLE_KEYS,
        *LEGACY_NAV_OUT_ENABLE_KEYS,
        *MAP_OUT_ENABLE_KEYS,
        *LEGACY_MAP_OUT_ENABLE_KEYS,
        "enable_ros2_camera_bridge",
        "enable_ros2_rerun_bridge",
    ):
        if _optional_bool(config.get(key)) is True:
            problems.append(f"{key}=True")
    return tuple(problems)


def _blueprint(config: dict[str, Any]) -> Blueprint:
    cfg = dict(config)
    robot = str(cfg.pop("robot", "thunder"))
    slam_profile = normalize_slam_profile(cfg.pop("slam_profile", "fastlio2"))
    detector = str(cfg.pop("detector", "bpu"))
    encoder = str(cfg.pop("encoder", "mobileclip"))
    llm = str(cfg.pop("llm", "qwen"))
    planner_backend_default = cfg.pop("planner_backend", "octoplanner3d")
    planner_backend = str(cfg.pop("planner", "") or planner_backend_default)
    tomogram = str(cfg.pop("tomogram", ""))
    gateway_port = int(cfg.pop("gateway_port", DEFAULT_GATEWAY_PORT))
    teleop_port = int(cfg.pop("teleop_port", gateway_port))
    enable_native = bool(cfg.pop("enable_native", False))
    enable_semantic = bool(cfg.pop("enable_semantic", True))
    enable_gateway = bool(cfg.pop("enable_gateway", True))
    enable_teleop = bool(cfg.pop("enable_teleop", True))
    enable_map_modules = bool(cfg.pop("enable_map_modules", True))
    enable_navigation = bool(cfg.pop("enable_navigation", True))
    enable_rerun = bool(cfg.pop("enable_rerun", False))
    enable_swap = bool(cfg.pop("enable_swap", False))
    swap_mux_name = str(cfg.pop("swap_mux_name", "nav.velocity_mux"))
    swap_nav_name = str(cfg.pop("swap_nav_name", "nav.mission"))
    scene_xml = str(cfg.pop("scene_xml", ""))
    runtime_mode = str(cfg.pop("runtime_mode", "") or cfg.pop("_runtime_mode", ""))
    run_startup_checks = bool(cfg.pop("run_startup_checks", True))
    manage_external_services = bool(cfg.pop("manage_external_services", True))
    namespace = cfg.pop("namespace", None)
    semantic_save_dir = str(cfg.get("semantic_save_dir", DEFAULT_SEMANTIC_DIR))
    normalized_runtime_mode = runtime_mode.strip().lower()

    if normalized_runtime_mode == "lite":
        lite_incompatibilities = _lite_runtime_incompatibilities(
            slam_profile=slam_profile,
            enable_semantic=enable_semantic,
            enable_gateway=enable_gateway,
            enable_teleop=enable_teleop,
            enable_map_modules=enable_map_modules,
            enable_navigation=enable_navigation,
            enable_rerun=enable_rerun,
            manage_external_services=manage_external_services,
            run_startup_checks=run_startup_checks,
            scene_xml=scene_xml,
            config=cfg,
        )
        if lite_incompatibilities:
            joined = ", ".join(lite_incompatibilities)
            raise ValueError(
                "Thunder Lite runtime cannot enable full-stack capabilities: "
                f"{joined}"
            )

    driver_module = _driver_name(robot)
    if run_startup_checks:
        _run_startup_preflight(
            enable_semantic=enable_semantic,
            slam_profile=slam_profile,
        )

    if _is_lite_runtime(
        runtime_mode=runtime_mode,
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
        lite_config = _lite_graph_config(cfg)
        from runtime.blueprints.products.thunder_lite import (
            apply_thunder_lite_wires,
            compose_thunder_lite_modules,
        )

        bp = compose_thunder_lite_modules(
            robot=robot,
            driver_module=driver_module,
            planner_backend=planner_backend,
            tomogram=tomogram,
            enable_native=enable_native,
            config=lite_config,
        )
        bp = apply_thunder_lite_wires(
            bp,
            driver_module=driver_module,
            safety_stop_wiring=bool(cfg.get("safety_stop_wiring", True)),
        )
    else:
        from runtime.blueprints.full_stack_wiring import apply_full_stack_wires
        from runtime.blueprints.stacks.composition import compose_full_stack_modules

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
            enable_navigation=enable_navigation,
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
            cmd_vel_mux_collision_monitor=bool(
                cfg.get("cmd_vel_mux_collision_monitor", False)
            ),
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
            "runtime_mode": "lite",
            "slam_profile": "none",
            "llm": "mock",
            "planner": "direct",
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
            "planner": "octoplanner3d",
            "tomogram": _resolve_octoplanner3d_map(),
            "plan_safety_policy": "reject",
            "fallback_planner_name": "",
            **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
            "enable_native": True,
            "terrain_backend": "nanobind",
            "terrain_strict_native": True,
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
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
            "planner": "octoplanner3d",
            "tomogram": _resolve_octoplanner3d_map(),
            "plan_safety_policy": "reject",
            "fallback_planner_name": "",
            "waypoint_threshold": 0.20,
            "final_waypoint_threshold": 0.10,
            "local_planner_allow_direct_track_fallback": True,
            "local_planner_direct_track_fallback_min_distance_m": 0.05,
            "local_planner_min_trackable_local_path_m": 0.05,
            "path_follower_goal_tolerance": 0.05,
            "path_follower_lookahead": 0.35,
            "path_follower_max_speed": 0.20,
            "path_follower_min_speed": 0.08,
            "path_follower_native_max_accel": 10.0,
            **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
            "enable_native": True,
            "terrain_backend": "nanobind",
            "terrain_strict_native": True,
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
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
            "planner": "octoplanner3d",
            "tomogram": _resolve_octoplanner3d_map(),
            "plan_safety_policy": "reject",
            "fallback_planner_name": "",
            **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
            "enable_semantic": True,
            "enable_gateway": True,
            "enable_frontier": True,
            "enable_traversable_frontier": True,
            "exploration_backend": "none",
            "enable_native": True,
            "terrain_backend": "nanobind",
            "terrain_strict_native": True,
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
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
