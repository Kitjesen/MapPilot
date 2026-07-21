"""Runtime endpoint catalog.

Profiles define what Thunder should do. Endpoints define where data and
commands come from: real robot, MuJoCo, Gazebo, or CMU Unity.
"""

from __future__ import annotations

import sys
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from runtime.profiles.catalog.endpoint_adapter_configs import (
    CMU_UNITY_CONFIG,
    GAZEBO_CONFIG,
    MUJOCO_LIVE_CONFIG,
)
from runtime.profiles.catalog.runtime_paths import (
    RUNTIME_ODOM_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
)

PRODUCT_PROFILE_ENDPOINTS: dict[str, tuple[str, ...]] = {
    "lite": ("thunder_lite",),
    "teleop": ("thunder_field",),
    "teleop_avoid": ("thunder_field",),
    "map": ("thunder_field", "mujoco_live"),
    "tracking": ("thunder_field",),
    "nav": ("thunder_field",),
    "inspection": ("thunder_field",),
    "explore": ("thunder_field", "mujoco_live", "gazebo"),
    "tare_explore": ("thunder_field", "mujoco_live", "cmu_unity"),
    "super_lio": ("thunder_field",),
    "super_lio_relocation": ("thunder_field",),
}


class RuntimeEndpointError(ValueError):
    """Raised when a profile cannot run on a selected runtime endpoint."""


@dataclass(frozen=True)
class RuntimeEndpointSpec:
    name: str
    description: str
    data_source: str
    robot_preset: str
    supported_profiles: tuple[str, ...]
    simulation_only: bool
    module_transport: str = "local"
    endpoint_transport: str = "local"
    endpoint_contract: str | None = None
    external_launcher: str | None = None
    runtime_contract: str | None = None
    config_overrides: Mapping[str, Any] = field(default_factory=dict)
    profile_overrides: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    env_overrides: Mapping[str, str] = field(default_factory=dict)
    default_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    record_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    def require_profile(self, profile: str) -> None:
        if profile not in self.supported_profiles:
            supported = ", ".join(self.supported_profiles)
            raise RuntimeEndpointError(
                f"endpoint '{self.name}' does not support profile '{profile}' (supported: {supported})"
            )

    def config_for_profile(self, profile: str) -> dict[str, Any]:
        from runtime.profiles.endpoint_config import endpoint_config_for_profile

        return endpoint_config_for_profile(self, profile)


@dataclass(frozen=True)
class RuntimeRunSpec:
    profile: str
    endpoint: str | None
    data_source: str
    runtime_contract: str | None
    robot_preset: str | None
    module_transport: str
    endpoint_transport: str
    endpoint_contract: str | None
    route_contract: str | None
    simulation_only: bool
    command_sink: str
    slam_source: str
    localization_source: str
    mapping_source: str
    lidar_extrinsic_profile: str | None
    frames: Mapping[str, Any]
    frame_links: Mapping[str, Mapping[str, Any]]
    topic_allowed_frame_ids: Mapping[str, tuple[str, ...]]
    topic_default_frame_ids: Mapping[str, str]
    resolved_runtime_data_flow: tuple[Mapping[str, Any], ...]
    runtime_data_flow_stage_algorithm_interfaces: Mapping[str, tuple[str, ...]]
    launcher: str | None
    launcher_args: tuple[str, ...]
    env: Mapping[str, str]
    product_semantic_overrides: tuple[Mapping[str, Any], ...] = ()
    localization_adapter: str | None = None
    global_planner: str | None = None
    fallback_global_planners: tuple[str, ...] = ()
    planner_latency_budget_ms: int | None = None
    plan_safety_policy: str | None = None
    autonomy_backends: Mapping[str, str] = field(default_factory=dict)

    def as_command(self) -> list[str]:
        if not self.launcher:
            return []
        if Path(self.launcher).suffix == ".py":
            return [sys.executable, self.launcher, *self.launcher_args]
        return ["bash", self.launcher, *self.launcher_args]


RUNTIME_ENDPOINTS: dict[str, RuntimeEndpointSpec] = {
    "thunder_lite": RuntimeEndpointSpec(
        name="thunder_lite",
        description="Physical Thunder local lightweight endpoint without ROS, SLAM, maps, or gateway.",
        data_source="thunder_lite_local",
        robot_preset="thunder",
        supported_profiles=("lite",),
        simulation_only=False,
        runtime_contract="thunder_lite_local",
        config_overrides={
            "enable_hw": False,
        },
    ),
    "thunder_field": RuntimeEndpointSpec(
        name="thunder_field",
        description="Physical Thunder field robot endpoint.",
        data_source=THUNDER_FIELD_RUNTIME_CONTRACT,
        robot_preset="thunder",
        supported_profiles=(
            "teleop",
            "teleop_avoid",
            "map",
            "tracking",
            "nav",
            "inspection",
            "explore",
            "tare_explore",
            "super_lio",
            "super_lio_relocation",
        ),
        simulation_only=False,
        # Canonical field endpoint: typed DDS into LingTu-owned wire contracts.
        # Literal contract name avoids an L1 profiles -> L3 adapters import.
        endpoint_transport="dds",
        endpoint_contract="thunder_field_dds_v1",
        runtime_contract=THUNDER_FIELD_RUNTIME_CONTRACT,
        config_overrides={
            "enable_hw": False,
            "enable_robot_driver": False,
            "enable_lidar": False,
            "enable_imu": False,
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
            "localization_adapter": "cpp_slam_status",
            "native_navigation_endpoint": "lingtu-nav-dds",
            "manage_session_services": False,
            # Field navigation DDS is owned exclusively by C++ services.
            # Python navigation adapters have been removed.
            "enable_map_out": False,
            # Field camera hardware is owned by lingtu-camera-dds.service.
            # The runtime camera role consumes DDS and must not open Orbbec
            # directly, otherwise two processes compete for the same device.
            "enable_camera": True,
            "camera_backend": "dds",
        },
        profile_overrides={},
    ),
    "mujoco_live": RuntimeEndpointSpec(
        name="mujoco_live",
        description="MuJoCo raw MID-360 + IMU endpoint feeding Fast-LIO.",
        data_source="mujoco_fastlio2_live",
        robot_preset="sim_endpoint",
        supported_profiles=(
            "map",
            "explore",
            "tare_explore",
            "sim_mujoco_live",
            "sim_mujoco_octo_live",
        ),
        simulation_only=True,
        external_launcher="sim/scripts/mujoco/launch_fastlio2_live.sh",
        runtime_contract="mujoco_fastlio2_live",
        config_overrides=MUJOCO_LIVE_CONFIG,
        profile_overrides={
            "map": {
                "enable_frontier": False,
                "exploration_backend": "none",
            },
            "explore": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
            "tare_explore": {
                "planner": "octoplanner3d",
                "map_path": _resolve_octoplanner3d_map(),
                "map_artifact_gate_required": True,
                "plan_safety_policy": "reject",
                "fallback_planner_name": "",
                "enable_frontier": False,
                "exploration_backend": "tare",
                "exploration_auto_start": False,
                "goal_frame_id": RUNTIME_ODOM_FRAME_ID,
                "hold_active_goal_until_terminal": True,
                "max_waypoint_distance_m": 6.0,
                "waypoint_odometry_timeout_s": 5.0,
                "defer_empty_path_planning_failure": True,
                "empty_path_retry_interval_s": 2.0,
                "empty_path_retry_timeout_s": 90.0,
                "planning_frame_id": RUNTIME_ODOM_FRAME_ID,
                "occupancy_frame_id": RUNTIME_ODOM_FRAME_ID,
                "enable_map_out": False,
            },
            "sim_mujoco_live": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
            "sim_mujoco_octo_live": {
                "planner": "octoplanner3d",
                "planner_backend": "octoplanner3d",
                "map_path": _resolve_octoplanner3d_map(),
                "plan_safety_policy": "reject",
                "fallback_planner_name": "",
                "enable_frontier": False,
                "enable_traversable_frontier": False,
                "exploration_backend": "none",
                "local_planner_allow_direct_track_fallback": False,
                "local_planner_ignore_near_field_stop": False,
            },
        },
        default_actions={
            "map": ("gate",),
            "explore": ("explore",),
            "tare_explore": ("tare",),
            "sim_mujoco_live": ("gate",),
            "sim_mujoco_octo_live": ("octo-moving-obstacle-video",),
        },
        record_actions={
            "map": ("video",),
            "explore": ("video",),
            "tare_explore": ("tare-video",),
            "sim_mujoco_live": ("video",),
            "sim_mujoco_octo_live": ("octo-moving-obstacle-video",),
        },
    ),
    "gazebo": RuntimeEndpointSpec(
        name="gazebo",
        description="Gazebo/GZ industrial delivery simulation endpoint.",
        data_source="gazebo_industrial",
        robot_preset="sim_endpoint",
        supported_profiles=("explore", "sim_gazebo", "sim_industrial"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_lingtu_gazebo_industrial_demo.sh",
        runtime_contract="gazebo_industrial",
        config_overrides=GAZEBO_CONFIG,
        default_actions={
            "explore": ("start", "--gate"),
            "sim_gazebo": ("start", "--gate"),
            "sim_industrial": ("start", "--gate"),
        },
        record_actions={
            "explore": ("start", "--gate", "--rviz"),
            "sim_gazebo": ("start", "--gate", "--rviz"),
            "sim_industrial": ("start", "--gate", "--rviz"),
        },
    ),
    "cmu_unity": RuntimeEndpointSpec(
        name="cmu_unity",
        description="CMU Unity + external TARE/FAR benchmark endpoint.",
        data_source="cmu_unity_external",
        robot_preset="sim_endpoint",
        supported_profiles=("tare_explore", "sim_cmu_tare"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        runtime_contract="cmu_unity_external",
        config_overrides=CMU_UNITY_CONFIG,
        default_actions={
            "tare_explore": ("gate",),
            "sim_cmu_tare": ("gate",),
        },
        record_actions={
            "tare_explore": ("start", "--gate", "--rviz"),
            "sim_cmu_tare": ("start", "--gate", "--rviz"),
        },
    ),
}

PRODUCT_RUNTIME_ENDPOINT_ALIASES: dict[str, str] = {
    # Product-facing endpoint aliases for CLI/docs/operator workflows.
    "field": "thunder_field",
    "thunder": "thunder_field",
    "thunder-field": "thunder_field",
    "thunder-lite": "thunder_lite",
    "thunder-basic": "thunder_lite",
}

COMPAT_RUNTIME_ENDPOINT_ALIASES: dict[str, str] = {
    # Compatibility aliases for older CLI/docs/scripts.
    "real_s100p": "thunder_field",
    "s100p": "thunder_field",
}

RUNTIME_ENDPOINT_ALIASES: dict[str, str] = {
    **PRODUCT_RUNTIME_ENDPOINT_ALIASES,
    **COMPAT_RUNTIME_ENDPOINT_ALIASES,
}


__all__ = [
    "COMPAT_RUNTIME_ENDPOINT_ALIASES",
    "PRODUCT_PROFILE_ENDPOINTS",
    "PRODUCT_RUNTIME_ENDPOINT_ALIASES",
    "RUNTIME_ENDPOINTS",
    "RUNTIME_ENDPOINT_ALIASES",
    "RuntimeEndpointError",
    "RuntimeEndpointSpec",
    "RuntimeRunSpec",
]
