"""Runtime endpoint catalog.

Profiles define what Thunder should do. Endpoints define where data and
commands come from: real robot, MuJoCo, Gazebo, replay, or CMU Unity.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from core.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    map_frame_id,
    odom_frame_id,
)


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
    default_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    record_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    def require_profile(self, profile: str) -> None:
        if profile not in self.supported_profiles:
            supported = ", ".join(self.supported_profiles)
            raise RuntimeEndpointError(
                f"endpoint '{self.name}' does not support profile '{profile}' "
                f"(supported: {supported})"
            )

    def config_for_profile(self, profile: str) -> dict[str, Any]:
        self.require_profile(profile)
        merged = dict(self.config_overrides)
        merged.update(self.profile_overrides.get(profile, {}))
        merged["_runtime_endpoint"] = self.name
        merged["_endpoint_data_source"] = self.data_source
        merged["_module_transport"] = self.module_transport
        merged["_endpoint_transport"] = self.endpoint_transport
        if self.endpoint_contract:
            merged["_endpoint_contract"] = self.endpoint_contract
        if self.external_launcher:
            merged["_external_launcher"] = self.external_launcher
        if self.runtime_contract:
            merged["_runtime_contract"] = self.runtime_contract
        if self.default_actions:
            if profile not in self.default_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{self.name}' default_actions missing profile "
                    f"'{profile}'"
                )
            merged["_external_default_args"] = self.default_actions[profile]
        if self.record_actions:
            if profile not in self.record_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{self.name}' record_actions missing profile "
                    f"'{profile}'"
                )
            merged["_external_record_args"] = self.record_actions[profile]
        return merged


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

    def as_command(self) -> list[str]:
        if not self.launcher:
            return []
        if Path(self.launcher).suffix == ".py":
            return [sys.executable, self.launcher, *self.launcher_args]
        return ["bash", self.launcher, *self.launcher_args]


RUNTIME_MAP_FRAME_ID = map_frame_id()
RUNTIME_ODOM_FRAME_ID = odom_frame_id()

_MUJOCO_LIVE_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "astar",
    "tomogram": "",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_endpoint_grid_bridge": True,
    "enable_endpoint_path_bridge": True,
    "enable_camera": False,
    "use_driver_camera": False,
    "cloud_topic": TOPICS.map_cloud,
    "planning_frame_id": RUNTIME_MAP_FRAME_ID,
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_core",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}

_GAZEBO_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "astar",
    "tomogram": "src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "enable_semantic": True,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_camera": True,
    "use_driver_camera": True,
    "cloud_topic": TOPICS.map_cloud,
    "planning_frame_id": RUNTIME_MAP_FRAME_ID,
    "enable_frontier": True,
    "exploration_backend": "none",
    "frontier_safe_distance": 0.80,
    "frontier_max_dist": 20.0,
    "frontier_rate": 2.0,
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_core",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}

_CMU_UNITY_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "pct",
    "tomogram": "",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "safe_goal_tolerance": 0.4,
    "waypoint_threshold": 0.45,
    "final_waypoint_threshold": 0.35,
    "stuck_timeout": 25.0,
    "stuck_dist_thre": 0.08,
    "downsample_dist": 0.6,
    "path_follower_goal_tolerance": 0.35,
    "local_planner_allow_direct_track_fallback": True,
    "local_planner_ignore_near_field_stop": True,
    "local_planner_direct_track_fallback_min_distance_m": 0.3,
    "enable_native": False,
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_endpoint_waypoint_bridge": True,
    "enable_endpoint_path_bridge": True,
    "exploration_backend": "tare_external",
    "exploration_auto_start": True,
    "prefer_path_strategy": True,
    "path_start_tolerance_m": 1.5,
    "path_goal_min_distance_m": 1.0,
    "path_goal_spacing_m": 0.75,
    "tare_fallback_timeout_s": 180.0,
    "allow_direct_goal_fallback": True,
    "direct_goal_fallback_on_planner_failure": True,
    "accept_partial_goal_progress": True,
    "partial_goal_repeat_ignore_window_s": 5.0,
    "external_strategy_path_control": False,
    "external_strategy_start_tolerance_m": 1.5,
    "planning_frame_id": RUNTIME_MAP_FRAME_ID,
    "latch_stop_signal": False,
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}


RUNTIME_ENDPOINTS: dict[str, RuntimeEndpointSpec] = {
    "thunder_lite": RuntimeEndpointSpec(
        name="thunder_lite",
        description="Physical Thunder local lightweight endpoint without ROS, SLAM, maps, or gateway.",
        data_source="thunder_lite_local",
        robot_preset="thunder",
        supported_profiles=("lite",),
        simulation_only=False,
        runtime_contract="thunder_lite_local",
    ),
    "thunder_field": RuntimeEndpointSpec(
        name="thunder_field",
        description="Physical Thunder field robot endpoint.",
        data_source=THUNDER_FIELD_RUNTIME_CONTRACT,
        robot_preset="thunder",
        supported_profiles=(
            "map",
            "nav",
            "explore",
            "tare_explore",
            "super_lio",
            "super_lio_relocation",
        ),
        simulation_only=False,
        endpoint_transport="lcm",
        endpoint_contract="thunder_field_lcm_v1",
        runtime_contract=THUNDER_FIELD_RUNTIME_CONTRACT,
        config_overrides={
            "enable_robot_driver": False,
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "lcm_endpoint_source",
            "localization_adapter": "lcm_endpoint",
            "endpoint_ingress_adapter": "lcm_endpoint",
            "endpoint_egress_adapter": "lcm_endpoint",
            "enable_endpoint_command_bridge": True,
            "enable_endpoint_path_bridge": True,
            "enable_camera": False,
        },
        profile_overrides={
            "nav": {
                # The field computer runs SLAM/localization as supervised
                # services that own the Livox device; the product graph bridges
                # their normalized outputs instead of spawning another localizer.
                "slam_profile": "bridge",
            },
        },
    ),
    "mujoco_live": RuntimeEndpointSpec(
        name="mujoco_live",
        description="MuJoCo raw MID-360 + IMU endpoint feeding Fast-LIO.",
        data_source="mujoco_fastlio2_live",
        robot_preset="sim_gazebo",
        supported_profiles=(
            "map",
            "explore",
            "tare_explore",
            "sim_mujoco_live",
            "sim_mujoco_pct_live",
        ),
        simulation_only=True,
        external_launcher="sim/scripts/launch_mujoco_fastlio2_live.sh",
        runtime_contract="mujoco_fastlio2_live",
        config_overrides=_MUJOCO_LIVE_CONFIG,
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
                "planner": "astar",
                "planner_backend": "astar",
                "enable_frontier": False,
                "exploration_backend": "tare",
                "exploration_auto_start": False,
                "tare_scenario": "indoor",
                "goal_frame_id": RUNTIME_ODOM_FRAME_ID,
                "hold_active_goal_until_terminal": True,
                "max_waypoint_distance_m": 6.0,
                "waypoint_odometry_timeout_s": 5.0,
                "defer_empty_path_planning_failure": True,
                "empty_path_retry_interval_s": 2.0,
                "empty_path_retry_timeout_s": 90.0,
                "planning_frame_id": RUNTIME_ODOM_FRAME_ID,
                "occupancy_frame_id": RUNTIME_ODOM_FRAME_ID,
                "enable_endpoint_grid_bridge": True,
                "enable_endpoint_path_bridge": True,
            },
            "sim_mujoco_live": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
            "sim_mujoco_pct_live": {
                "planner": "pct",
                "planner_backend": "pct",
                "tomogram": "src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
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
            "sim_mujoco_pct_live": ("pct-moving-obstacle-video",),
        },
        record_actions={
            "map": ("video",),
            "explore": ("video",),
            "tare_explore": ("tare-video",),
            "sim_mujoco_live": ("video",),
            "sim_mujoco_pct_live": ("pct-moving-obstacle-video",),
        },
    ),
    "replay": RuntimeEndpointSpec(
        name="replay",
        description="No-actuation rosbag/log replay endpoint.",
        data_source="rosbag_fastlio2_replay",
        robot_preset="sim_gazebo",
        supported_profiles=("map", "nav", "explore", "tare_explore"),
        simulation_only=True,
        external_launcher="sim/scripts/fastlio2_rosbag_replay_gate.py",
        runtime_contract="rosbag_fastlio2_replay",
        config_overrides=_MUJOCO_LIVE_CONFIG,
        default_actions={
            "map": ("gate",),
            "nav": ("gate",),
            "explore": ("gate",),
            "tare_explore": ("gate",),
        },
        record_actions={
            "map": ("report",),
            "nav": ("report",),
            "explore": ("report",),
            "tare_explore": ("report",),
        },
    ),
    "gazebo": RuntimeEndpointSpec(
        name="gazebo",
        description="Gazebo/GZ industrial delivery simulation endpoint.",
        data_source="gazebo_industrial",
        robot_preset="sim_gazebo",
        supported_profiles=("explore", "sim_gazebo", "sim_industrial"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_lingtu_gazebo_industrial_demo.sh",
        runtime_contract="gazebo_industrial",
        config_overrides=_GAZEBO_CONFIG,
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
        robot_preset="sim_gazebo",
        supported_profiles=("tare_explore", "sim_cmu_tare"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        runtime_contract="cmu_unity_external",
        config_overrides=_CMU_UNITY_CONFIG,
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
    "PRODUCT_RUNTIME_ENDPOINT_ALIASES",
    "RUNTIME_ENDPOINT_ALIASES",
    "RUNTIME_ENDPOINTS",
    "RuntimeEndpointError",
    "RuntimeEndpointSpec",
    "RuntimeRunSpec",
]
