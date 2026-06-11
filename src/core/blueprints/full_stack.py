"""Full-stack navigation blueprint - composable factory pattern.

Usage::

    # dimos-style one-liner per stack
    system = full_stack_blueprint(robot="thunder", slam_profile="localizer").build()
    system.start()

    # Or compose directly
    from core.blueprints.stacks import *
    system = autoconnect(
        driver("thunder", host="192.168.66.190"),
        slam("localizer"),
        maps(),
        perception("bpu"),
        memory(),
        planner("kimi"),
        navigation("astar"),
        safety(),
        gateway(5050),
    ).build()
"""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint, autoconnect

from .full_stack_wiring import apply_full_stack_wires
from .stacks import (
    driver,
    exploration,
    gateway,
    lidar,
    maps,
    memory,
    navigation,
    perception,
    safety,
    sim_lidar,
    slam,
)
from .stacks import planner as planner_stack
from .stacks.driver import driver_name
from .stacks.memory import DEFAULT_SEMANTIC_DIR
from .stacks.slam import normalize_slam_profile

logger = logging.getLogger(__name__)

_EXPLORATION_CONFIG_KEYS = (
    "way_point_topic",
    "path_topic",
    "runtime_topic",
    "finish_topic",
    "start_topic",
    "goal_frame_id",
    "way_point_timeout_s",
    "hold_active_goal_until_terminal",
    "max_waypoint_distance_m",
    "waypoint_odometry_timeout_s",
    "prefer_path_strategy",
    "path_goal_min_distance_m",
    "path_goal_spacing_m",
    "path_start_tolerance_m",
    "path_max_goal_count",
    "path_strategy_timeout_s",
    "path_strategy_fallback_to_waypoint",
    "tare_warn_timeout_s",
    "tare_fallback_timeout_s",
    "tare_supervisor_hz",
)


def _run_startup_preflight(
    *,
    enable_semantic: bool,
    slam_profile: str,
) -> None:
    from core.utils.calibration_check import run_calibration_check

    needs_camera = enable_semantic or slam_profile not in ("", "none")
    needs_slam = slam_profile not in ("", "none")
    calib = run_calibration_check(
        require_camera=needs_camera,
        require_slam=needs_slam,
    )
    if not calib.ok:
        raise RuntimeError(
            f"Calibration self-check failed ({len(calib.errors)} error(s)): "
            + "; ".join(calib.errors)
        )


def _driver_stack_config(
    config: dict[str, Any],
    *,
    slam_profile: str,
    driver_module: str,
    enable_semantic: bool,
) -> dict[str, Any]:
    driver_config = dict(config)
    if slam_profile in ("", "none") and driver_module in {"StubDogModule", "MujocoDriverModule"}:
        driver_config.setdefault("odom_frame_id", "map")
    if enable_semantic and driver_module == "MujocoDriverModule":
        driver_config.setdefault("enable_camera", True)
    return driver_config


def _perception_stack_config(config: dict[str, Any], *, driver_module: str) -> dict[str, Any]:
    perception_config = dict(config)
    perception_config["_driver_cls_name"] = driver_module
    return perception_config


def _device_manager_blueprint() -> Blueprint:
    device_bp = Blueprint()
    try:
        import os
        from pathlib import Path

        devices_yaml = Path(__file__).resolve().parents[3] / "config" / "devices.yaml"
        if devices_yaml.exists():
            from core.devices import DeviceManager

            device_bp.add(
                DeviceManager,
                config_path=str(devices_yaml),
                enable_hotplug=os.environ.get("LINGTU_HOTPLUG", "0") == "1",
            )
    except Exception as exc:
        logger.debug("DeviceManager not loaded: %s", exc)
    return device_bp


def _gnss_blueprint() -> Blueprint:
    gnss_bp = Blueprint()
    try:
        from core.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
        if gnss_cfg.get("enabled", False):
            from slam.gnss_bridge import GnssBridgeModule
            from slam.gnss_module import GnssModule

            gnss_bp.add(
                GnssModule,
                device_model=gnss_cfg.get("model", "WTRTK-980"),
                origin_lat=(gnss_cfg.get("origin") or {}).get("lat"),
                origin_lon=(gnss_cfg.get("origin") or {}).get("lon"),
                origin_alt=(gnss_cfg.get("origin") or {}).get("alt"),
                auto_init_origin=(gnss_cfg.get("origin") or {}).get("auto_init", True),
                min_sat_used=(gnss_cfg.get("quality") or {}).get("min_sat_used", 8),
                max_hdop=(gnss_cfg.get("quality") or {}).get("max_hdop", 2.5),
            )
            gnss_bp.add(
                GnssBridgeModule,
                device_id=gnss_cfg.get("device_id", "wtrtk980_main"),
            )
            rtcm_cfg = gnss_cfg.get("rtcm") or {}
            if rtcm_cfg.get("enabled", False):
                from slam.ntrip_client_module import NtripClientModule

                gnss_bp.add(
                    NtripClientModule,
                    enabled=True,
                    host=rtcm_cfg.get("ntrip_host", ""),
                    port=int(rtcm_cfg.get("ntrip_port", 2101)),
                    mount=rtcm_cfg.get("ntrip_mount", ""),
                    user=rtcm_cfg.get("ntrip_user", ""),
                    password=rtcm_cfg.get("ntrip_pass", ""),
                )
    except Exception as exc:
        logger.debug("GNSS not loaded: %s", exc)
    return gnss_bp


def _exploration_stack_config(config: dict[str, Any]) -> dict[str, Any]:
    exploration_config = {
        "backend": config.get("exploration_backend", "none"),
        "tare_scenario": config.get("tare_scenario", "forest"),
        "auto_start": config.get("exploration_auto_start", True),
    }
    for key in _EXPLORATION_CONFIG_KEYS:
        if key in config:
            exploration_config[key] = config[key]
    return exploration_config


def _needs_lidar_for_slam(slam_profile: str) -> bool:
    return slam_profile not in (
        "",
        "none",
        "bridge",
        "super_lio",
        "super_lio_relocation",
    )


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner_backend: str = "astar",
    tomogram: str = "",
    gateway_port: int = 5050,
    teleop_port: int = 5050,  # teleop is now on /ws/teleop of the main gateway port
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
    enable_rerun: bool = False,
    enable_swap: bool = False,
    swap_mux_name: str = "CmdVelMux",
    swap_nav_name: str = "NavigationModule",
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

    driver_config = _driver_stack_config(
        config,
        slam_profile=slam_profile,
        driver_module=drv,
        enable_semantic=enable_semantic,
    )
    perception_config = _perception_stack_config(config, driver_module=drv)
    needs_lidar = _needs_lidar_for_slam(slam_profile)
    lidar_ip = config.get("lidar_ip")
    device_bp = _device_manager_blueprint()
    gnss_bp = _gnss_blueprint()
    exploration_config = _exploration_stack_config(config)

    bp = autoconnect(
        device_bp,
        driver(robot, **driver_config),
        lidar(ip=lidar_ip, enabled=needs_lidar),
        sim_lidar(scene_xml=scene_xml),
        slam(slam_profile, manage_services=manage_external_services),
        gnss_bp,
        maps(**config) if enable_map_modules else Blueprint(),
        perception(
            detector,
            encoder,
            manage_services=manage_external_services,
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
            teleop_port=teleop_port,
            enable_teleop=enable_teleop,
            enable_rerun=enable_rerun,
        )
        if enable_gateway
        else Blueprint(),
    )

    bp = apply_full_stack_wires(
        bp,
        robot=robot,
        driver_module=drv,
        slam_profile=slam_profile,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
        safety_stop_wiring=bool(config.get("safety_stop_wiring", True)),
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
