"""Pure config adapters used by stack composition."""

from __future__ import annotations

from typing import Any

from runtime.contracts import CAMERA_BACKEND_SIM

EXPLORATION_CONFIG_KEYS = (
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
    "transport_mode",
    "policy_rate_hz",
)


def driver_stack_config(
    config: dict[str, Any],
    *,
    slam_profile: str,
    driver_module: str,
    enable_semantic: bool,
) -> dict[str, Any]:
    driver_config = dict(config)
    if slam_profile in ("", "none") and driver_module == "StubDogModule":
        driver_config.setdefault("odom_frame_id", "map")
    if slam_profile in ("", "none") and driver_module == "MujocoDriverModule":
        frame_id = driver_config.setdefault("odom_frame_id", "map")
        driver_config.setdefault("map_cloud_frame_id", frame_id)
    if enable_semantic and driver_module == "MujocoDriverModule":
        driver_config.setdefault("enable_camera", True)
        driver_config.setdefault(
            "publish_camera",
            bool(driver_config.get("use_driver_camera", False)),
        )
    if driver_module == "MujocoDriverModule":
        driver_config.setdefault(
            "publish_lidar",
            bool(driver_config.get("use_driver_lidar", False)),
        )
    return driver_config


def perception_stack_config(config: dict[str, Any], *, driver_module: str) -> dict[str, Any]:
    perception_config = dict(config)
    perception_config["_driver_cls_name"] = driver_module
    if driver_module == "MujocoDriverModule":
        perception_config.setdefault("enable_camera", True)
        perception_config.setdefault("camera_backend", CAMERA_BACKEND_SIM)
        perception_config.setdefault("use_driver_camera", False)
    return perception_config


def exploration_owner(config: dict[str, Any]) -> str:
    """Return the single runtime owner for exploration goal generation."""
    backend = str(config.get("exploration_backend", "none") or "none")
    if backend == "tare" and config.get("native_navigation_endpoint"):
        return "native"
    return "module"


def exploration_stack_config(config: dict[str, Any]) -> dict[str, Any]:
    exploration_config = {
        "backend": config.get("exploration_backend", "none"),
        "owner": exploration_owner(config),
        "tare_scenario": config.get("tare_scenario", "forest"),
        "auto_start": config.get("exploration_auto_start", True),
    }
    for key in EXPLORATION_CONFIG_KEYS:
        if key in config:
            exploration_config[key] = config[key]
    return exploration_config


def needs_lidar_for_slam(slam_profile: str) -> bool:
    return slam_profile == "native_dds"
