"""Runtime endpoint definitions for Dimos-style task/connection split.

Profiles describe what LingTu should do: map, navigate, explore, or run TARE.
Runtime endpoints describe where sensor/state data and command sinks come from:
real robot, MuJoCo, Gazebo, or CMU Unity. Keeping this split explicit lets the
same high-level LingTu stack run against different connection layers.
"""

from __future__ import annotations

import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping

from core.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    FRAMES,
    FRAME_LINKS,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    TOPICS,
    map_frame_id,
    odom_frame_id,
    profile_data_source,
    resolved_runtime_data_flow,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_ids,
)


class RuntimeEndpointError(ValueError):
    """Raised when a profile cannot run on a selected runtime endpoint."""


PRODUCT_SEMANTIC_CONFIG_KEYS: tuple[str, ...] = (
    "slam_profile",
    "planner",
    "planner_backend",
    "llm",
    "tomogram",
    "plan_safety_policy",
    "fallback_planner_name",
    "enable_semantic",
    "enable_map_modules",
    "enable_frontier",
    "exploration_backend",
    "frontier_safe_distance",
    "frontier_max_dist",
    "frontier_rate",
    "tare_scenario",
    "exploration_auto_start",
    "safe_goal_tolerance",
    "waypoint_threshold",
    "final_waypoint_threshold",
    "stuck_timeout",
    "stuck_dist_thre",
    "downsample_dist",
    "path_follower_goal_tolerance",
    "local_planner_allow_direct_track_fallback",
    "local_planner_ignore_near_field_stop",
    "local_planner_direct_track_fallback_min_distance_m",
    "allow_direct_goal_fallback",
    "direct_goal_fallback_on_planner_failure",
    "accept_partial_goal_progress",
    "prefer_path_strategy",
    "path_start_tolerance_m",
    "path_goal_min_distance_m",
    "path_goal_spacing_m",
    "external_strategy_path_control",
    "external_strategy_start_tolerance_m",
    "hold_active_goal_until_terminal",
    "max_waypoint_distance_m",
    "waypoint_odometry_timeout_s",
    "defer_empty_path_planning_failure",
    "empty_path_retry_interval_s",
    "empty_path_retry_timeout_s",
    "planning_frame_id",
    "goal_frame_id",
    "occupancy_frame_id",
)

_UNSET = object()


@dataclass(frozen=True)
class RuntimeEndpointSpec:
    name: str
    description: str
    data_source: str
    robot_preset: str
    supported_profiles: tuple[str, ...]
    simulation_only: bool
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
    "enable_ros2_grid_bridge": True,
    "enable_ros2_path_bridge": True,
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
    "enable_ros2_bridge": True,
    "enable_ros2_path_bridge": True,
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
    "real_s100p": RuntimeEndpointSpec(
        name="real_s100p",
        description="Physical S100P/Thunder robot endpoint.",
        data_source="real_s100p",
        robot_preset="s100p",
        supported_profiles=(
            "map",
            "nav",
            "explore",
            "tare_explore",
            "super_lio",
            "super_lio_relocation",
        ),
        simulation_only=False,
        runtime_contract="real_s100p",
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
                "enable_ros2_grid_bridge": True,
                "enable_ros2_path_bridge": True,
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


def runtime_endpoint(name: str) -> RuntimeEndpointSpec:
    try:
        return RUNTIME_ENDPOINTS[name]
    except KeyError as exc:
        choices = ", ".join(runtime_endpoint_names())
        raise RuntimeEndpointError(f"unknown runtime endpoint '{name}' (choices: {choices})") from exc


def runtime_endpoint_names() -> tuple[str, ...]:
    return tuple(RUNTIME_ENDPOINTS.keys())


def runtime_endpoint_robot_preset(profile: str, endpoint_name: str) -> str:
    endpoint = runtime_endpoint(endpoint_name)
    endpoint.require_profile(profile)
    return endpoint.robot_preset


def apply_runtime_endpoint_config(
    profile: str,
    config: Mapping[str, Any],
    endpoint_name: str,
) -> dict[str, Any]:
    endpoint = runtime_endpoint(endpoint_name)
    merged = dict(config)
    endpoint_config = endpoint.config_for_profile(profile)
    merged.update(endpoint_config)
    merged["_product_semantic_overrides"] = _product_semantic_overrides(
        config,
        merged,
    )
    return merged


def _launcher_args_for_config(
    config: Mapping[str, Any],
    *,
    profile: str,
    endpoint_name: str | None,
    launcher: Any,
    record: bool,
    extra_args: tuple[str, ...],
) -> tuple[str, ...]:
    if extra_args:
        return extra_args
    explicit_default_args = config.get("_external_default_args")
    explicit_record_args = config.get("_external_record_args")
    if record:
        if explicit_record_args:
            return tuple(explicit_record_args)
        endpoint = _endpoint_for_launcher_args(config, endpoint_name, launcher)
        if endpoint is not None and endpoint.record_actions:
            if profile not in endpoint.record_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{endpoint.name}' record_actions missing profile "
                    f"'{profile}'"
                )
            return endpoint.record_actions[profile]
        if explicit_default_args:
            return tuple(explicit_default_args)
        if endpoint is not None and endpoint.default_actions:
            if profile not in endpoint.default_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{endpoint.name}' default_actions missing profile "
                    f"'{profile}'"
                )
            return endpoint.default_actions[profile]
        raise RuntimeEndpointError(
            f"external launcher args missing for profile '{profile}'"
        )

    if explicit_default_args:
        return tuple(explicit_default_args)
    endpoint = _endpoint_for_launcher_args(config, endpoint_name, launcher)
    if endpoint is not None and endpoint.default_actions:
        if profile not in endpoint.default_actions:
            raise RuntimeEndpointError(
                f"endpoint '{endpoint.name}' default_actions missing profile "
                f"'{profile}'"
            )
        return endpoint.default_actions[profile]
    raise RuntimeEndpointError(
        f"external launcher args missing for profile '{profile}'"
    )


def _endpoint_for_launcher_args(
    config: Mapping[str, Any],
    endpoint_name: str | None,
    launcher: Any,
) -> RuntimeEndpointSpec | None:
    if endpoint_name not in RUNTIME_ENDPOINTS:
        return None
    endpoint = RUNTIME_ENDPOINTS[str(endpoint_name)]
    if not endpoint.external_launcher:
        return None
    if config.get("_runtime_endpoint") == endpoint.name:
        return endpoint
    if str(launcher or "") == endpoint.external_launcher:
        return endpoint
    return None


def _endpoint_name_for_config(
    config: Mapping[str, Any],
    data_source_name: str,
) -> str | None:
    endpoint_name = config.get("_runtime_endpoint")
    if endpoint_name:
        return str(endpoint_name)

    runtime_contract = config.get("_runtime_contract")
    launcher = config.get("_external_launcher")
    for name, endpoint in RUNTIME_ENDPOINTS.items():
        if runtime_contract and endpoint.runtime_contract == runtime_contract:
            return name
        if launcher and endpoint.external_launcher == launcher:
            return name
    for name, endpoint in RUNTIME_ENDPOINTS.items():
        if endpoint.data_source == data_source_name:
            return name
    return None


def resolve_runtime_run_spec(
    profile: str,
    config: Mapping[str, Any],
    *,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
) -> RuntimeRunSpec:
    data_source_name = str(
        config.get("_endpoint_data_source")
        or config.get("_runtime_contract")
        or profile_data_source(profile).data_source
    )
    source = DATA_SOURCE_CONTRACTS[data_source_name]
    endpoint_name = _endpoint_name_for_config(config, data_source_name)
    runtime_contract = config.get("_runtime_contract")
    if not runtime_contract and endpoint_name in RUNTIME_ENDPOINTS:
        runtime_contract = RUNTIME_ENDPOINTS[endpoint_name].runtime_contract
    launcher = config.get("_external_launcher")
    robot_preset = (
        RUNTIME_ENDPOINTS[endpoint_name].robot_preset
        if endpoint_name in RUNTIME_ENDPOINTS
        else str(config.get("robot") or "")
    )

    env = {
        "LINGTU_PROFILE": profile,
        "LINGTU_DATA_SOURCE": data_source_name,
        "LINGTU_SIMULATION_ONLY": "1" if source.provider != "hardware" else "0",
    }
    if endpoint_name:
        env["LINGTU_ENDPOINT"] = str(endpoint_name)
    if runtime_contract:
        env["LINGTU_RUNTIME_CONTRACT"] = str(runtime_contract)
    env["LINGTU_COMMAND_SINK"] = source.command_sink

    return RuntimeRunSpec(
        profile=profile,
        endpoint=str(endpoint_name) if endpoint_name else None,
        data_source=data_source_name,
        runtime_contract=str(runtime_contract) if runtime_contract else None,
        robot_preset=robot_preset or None,
        simulation_only=source.provider != "hardware",
        command_sink=source.command_sink,
        slam_source=source.slam_source,
        localization_source=source.localization_source,
        mapping_source=source.mapping_source,
        lidar_extrinsic_profile=source.lidar_extrinsic_profile,
        frames=asdict(FRAMES),
        frame_links={
            name: asdict(link)
            for name, link in FRAME_LINKS.items()
        },
        topic_allowed_frame_ids=runtime_topic_allowed_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        topic_default_frame_ids=runtime_topic_default_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        resolved_runtime_data_flow=tuple(
            asdict(stage)
            for stage in resolved_runtime_data_flow(data_source_name)
        ),
        runtime_data_flow_stage_algorithm_interfaces={
            name: tuple(interfaces)
            for name, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
        },
        launcher=str(launcher) if launcher else None,
        launcher_args=(
            _launcher_args_for_config(
                config,
                profile=profile,
                endpoint_name=endpoint_name,
                launcher=launcher,
                record=record,
                extra_args=extra_args,
            )
            if launcher
            else ()
        ),
        env=env,
        product_semantic_overrides=_normalized_product_semantic_overrides(
            config.get("_product_semantic_overrides")
        ),
    )


def _product_semantic_overrides(
    product_config: Mapping[str, Any],
    endpoint_config: Mapping[str, Any],
) -> tuple[dict[str, object], ...]:
    overrides: list[dict[str, object]] = []
    for key in PRODUCT_SEMANTIC_CONFIG_KEYS:
        product_value = product_config.get(key, _UNSET)
        endpoint_value = endpoint_config.get(key, _UNSET)
        if product_value is _UNSET and endpoint_value is _UNSET:
            continue
        if product_value == endpoint_value:
            continue
        overrides.append(
            {
                "field": key,
                "override_scope": "compatibility_override",
                "product_value": _json_config_value(product_value),
                "endpoint_value": _json_config_value(endpoint_value),
            }
        )
    return tuple(overrides)


def _normalized_product_semantic_overrides(
    value: Any,
) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, (list, tuple)):
        return ()
    normalized: list[Mapping[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping):
            continue
        field = item.get("field")
        if not field:
            continue
        normalized.append(
            {
                "field": str(field),
                "override_scope": str(
                    item.get("override_scope") or "compatibility_override"
                ),
                "product_value": _json_config_value(item.get("product_value")),
                "endpoint_value": _json_config_value(item.get("endpoint_value")),
            }
        )
    return tuple(normalized)


def _json_config_value(value: Any) -> object:
    if value is _UNSET:
        return "<unset>"
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [_json_config_value(item) for item in value]
    if isinstance(value, list):
        return [_json_config_value(item) for item in value]
    if isinstance(value, Mapping):
        return {str(key): _json_config_value(item) for key, item in value.items()}
    return value
