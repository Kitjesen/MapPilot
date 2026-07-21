"""LingTu module stack construction for MuJoCo live simulation endpoints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from runtime.profiles.catalog.runtime_paths import (
    _resolve_octoplanner3d_map,
)
from runtime.runtime_interface import TOPICS, topic_default_frame_id

# Fast-LIO live simulation owns a continuous local world frame named odom.
# Saved-map relocalization validates map-frame navigation separately.
MUJOCO_LIVE_PLANNING_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_LIVE_OCCUPANCY_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_LIVE_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_LIVE_SAVED_MAP_FRAME_ID = topic_default_frame_id(TOPICS.saved_map_cloud)


def _planner_map_path(planner_backend: str, explicit_path: str = "") -> str:
    if explicit_path:
        return str(explicit_path)
    planner = str(planner_backend or "").strip().lower()
    if planner == "octoplanner3d":
        return _resolve_octoplanner3d_map()
    return ""


@dataclass(frozen=True)
class MuJoCoLingTuStack:
    """Resolved LingTu modules used by the MuJoCo live endpoint."""

    system: Any
    driver: Any | None = None
    frontier: Any | None = None
    tare: Any | None = None
    occupancy_grid: Any | None = None
    navigation: Any | None = None
    local_planner: Any | None = None
    path_follower: Any | None = None
    cmd_vel_mux: Any | None = None


def _optional_module(system: Any, name: str) -> Any | None:
    try:
        return system.get_module(name)
    except (KeyError, AttributeError):
        return None


def _build_system_from_profile(profile: str, overrides: dict[str, Any]) -> Any:
    """Build a simulation system through the shared product/compat selector."""

    from lingtu.assembly.profile_builder import build_system_for_profile

    return build_system_for_profile(profile, overrides)


def build_fastlio2_frontier_stack(
    *,
    cloud_topic: str = TOPICS.registered_cloud,
    cmd_vel_topic: str = TOPICS.cmd_vel,
    frontier_goal_timeout: float = 120.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
    cmd_vel_mux_source_timeout: float = 0.5,
) -> MuJoCoLingTuStack:
    """Build the canonical LingTu frontier stack for MuJoCo + Fast-LIO live runs.

    The simulator owns raw MID-360/IMU rendering and command application.
    LingTu owns occupancy mapping, frontier selection, global planning, local
    planning, path following, cmd_vel muxing, and canonical /nav/* output.
    """

    system = _build_system_from_profile(
        "sim_mujoco_live",
        dict(
            robot="sim_endpoint",
            slam_profile="none",
            detector="sim_scene",
            llm="mock",
            planner_backend="octoplanner3d",
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            enable_map_modules=True,
            enable_frontier=True,
            enable_nav_out=False,
            enable_map_out=False,
            cloud_topic=cloud_topic,
            cmd_vel_topic=cmd_vel_topic,
            planning_frame_id=MUJOCO_LIVE_PLANNING_FRAME_ID,
            occupancy_frame_id=MUJOCO_LIVE_OCCUPANCY_FRAME_ID,
            occupancy_raycast_free_space=True,
            occupancy_unknown_as_obstacle_for_costmap=True,
            occupancy_raycast_max_rays=1600,
            occupancy_raycast_free_inflation_radius=0.4,
            grid_radius=12.0,
            grid_resolution=0.2,
            inflation_radius=0.25,
            frontier_min_size=3,
            frontier_safe_distance=0.35,
            frontier_lookahead=8.0,
            frontier_max_dist=8.0,
            frontier_info_gain=0.0,
            frontier_goal_timeout=max(1.0, float(frontier_goal_timeout)),
            frontier_rate=1.0,
            frontier_approach_standoff_m=0.8,
            frontier_approach_max_target_distance_m=6.0,
            frontier_approach_goal_max_distance_m=2.5,
            frontier_reachable_goal_radius=3.0,
            waypoint_threshold=0.45,
            final_waypoint_threshold=0.45,
            # This live endpoint can run slower than real time because MuJoCo, ROS,
            # Fast-LIO, map accumulation, and evidence capture share one process.
            # Keep these validation-profile tracker thresholds tolerant; production
            # defaults remain in Navigation and profile configs.
            stuck_timeout=max(45.0, float(frontier_goal_timeout) * 0.5),
            stuck_dist_thre=0.05,
            max_replan_count=6,
            mission_timeout=max(300.0, float(frontier_goal_timeout) * 3.0),
            downsample_dist=0.35,
            safe_goal_tolerance=6.0,
            plan_safety_policy="reject",
            python_autonomy_backend="nanobind",
            python_path_follower_backend="nav_kernel",
            local_planner_allow_direct_track_fallback=True,
            local_planner_ignore_near_field_stop=True,
            local_planner_direct_track_fallback_min_distance_m=0.3,
            local_planner_min_trackable_local_path_m=0.3,
            # Local planner outputs short rolling path segments. Keep the follower
            # stop band below the direct fallback segment length.
            path_follower_goal_tolerance=0.12,
            path_follower_max_speed=max(0.02, float(nav_max_linear_speed)),
            path_follower_max_yaw_rate=max(0.02, float(nav_max_angular_z)),
            path_follower_turn_speed_yaw_rate_start=max(
                0.0,
                float(nav_turn_speed_yaw_rate_start),
            ),
            path_follower_turn_speed_min_scale=max(
                0.0,
                min(1.0, float(nav_turn_speed_min_scale)),
            ),
            cmd_vel_mux_source_timeout=max(0.02, float(cmd_vel_mux_source_timeout)),
            path_follower_two_way_drive=False,
            latch_stop_signal=False,
            safety_stop_wiring=False,
            run_startup_checks=False,
        ),
    )

    return MuJoCoLingTuStack(
        system=system,
        driver=_optional_module(system, "SimEndpointDriverModule"),
        frontier=system.get_module("WavefrontFrontierExplorer"),
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("nav.mission"),
        local_planner=system.get_module("nav.local_planner"),
        path_follower=_optional_module(system, "nav.path_follower"),
        cmd_vel_mux=_optional_module(system, "nav.velocity_mux"),
    )


def build_fastlio2_inspection_stack(
    *,
    cloud_topic: str = TOPICS.registered_cloud,
    cmd_vel_topic: str = TOPICS.cmd_vel,
    planner_backend: str = "octoplanner3d",
    map_path: str = "",
    replan_on_costmap_update: bool | None = None,
    inspection_goal_timeout: float = 90.0,
    downsample_dist: float = 0.35,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
    cmd_vel_mux_source_timeout: float = 0.5,
    waypoint_threshold: float = 0.50,
    final_waypoint_threshold: float = 0.50,
    complete_path_on_goal_proximity: bool = False,
    goal_proximity_completion_threshold: float | None = None,
    path_follower_goal_tolerance: float = 0.12,
    path_follower_lookahead: float = 1.5,
    path_follower_min_speed: float = 0.15,
    path_follower_yaw_rate_gain: float = 7.5,
    path_follower_stop_yaw_rate_gain: float = 7.5,
    path_follower_dir_diff_thre: float = 0.1,
) -> MuJoCoLingTuStack:
    """Build the LingTu patrol/inspection stack for MuJoCo + Fast-LIO live runs.

    The inspection gate uses scripted patrol goals, but localization and local
    map inputs remain live Fast-LIO outputs routed through the canonical /nav/*
    contract. This keeps the scenario close to a real inspection mission: map,
    plan, track, and replan while the robot is moving.
    """

    planner_backend = str(planner_backend or "octoplanner3d").strip().lower()
    map_path = _planner_map_path(planner_backend, map_path)
    if replan_on_costmap_update is None:
        replan_on_costmap_update = True

    system = _build_system_from_profile(
        "sim_mujoco_live",
        dict(
            robot="sim_endpoint",
            slam_profile="none",
            detector="sim_scene",
            llm="mock",
            planner_backend=planner_backend,
            map_path=map_path,
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            enable_map_modules=True,
            enable_frontier=False,
            exploration_backend="none",
            enable_nav_out=False,
            enable_map_out=False,
            cloud_topic=cloud_topic,
            cmd_vel_topic=cmd_vel_topic,
            planning_frame_id=MUJOCO_LIVE_PLANNING_FRAME_ID,
            expected_saved_map_frame_id=MUJOCO_LIVE_SAVED_MAP_FRAME_ID,
            occupancy_frame_id=MUJOCO_LIVE_OCCUPANCY_FRAME_ID,
            goal_frame_id=MUJOCO_LIVE_GOAL_FRAME_ID,
            occupancy_raycast_free_space=True,
            occupancy_unknown_as_obstacle_for_costmap=True,
            occupancy_raycast_max_rays=1600,
            occupancy_raycast_free_inflation_radius=0.4,
            grid_radius=12.0,
            grid_resolution=0.2,
            inflation_radius=0.25,
            waypoint_threshold=max(0.05, float(waypoint_threshold)),
            final_waypoint_threshold=max(0.05, float(final_waypoint_threshold)),
            complete_path_on_goal_proximity=bool(complete_path_on_goal_proximity),
            goal_proximity_completion_threshold=(
                None
                if goal_proximity_completion_threshold is None
                else max(0.05, float(goal_proximity_completion_threshold))
            ),
            stuck_timeout=max(45.0, float(inspection_goal_timeout) * 0.5),
            stuck_dist_thre=0.05,
            max_replan_count=6,
            mission_timeout=max(180.0, float(inspection_goal_timeout) * 3.0),
            defer_empty_path_planning_failure=True,
            empty_path_retry_interval_s=2.0,
            empty_path_retry_timeout_s=min(45.0, max(12.0, float(inspection_goal_timeout) * 0.5)),
            downsample_dist=max(0.05, float(downsample_dist)),
            safe_goal_tolerance=6.0,
            plan_safety_policy="reject",
            replan_on_costmap_update=bool(replan_on_costmap_update),
            allow_path_start_insert=True,
            python_autonomy_backend="nanobind",
            python_path_follower_backend="nav_kernel",
            local_planner_allow_direct_track_fallback=True,
            local_planner_ignore_near_field_stop=True,
            local_planner_direct_track_fallback_min_distance_m=0.3,
            local_planner_min_trackable_local_path_m=0.3,
            path_follower_goal_tolerance=max(0.05, float(path_follower_goal_tolerance)),
            path_follower_lookahead=max(0.2, float(path_follower_lookahead)),
            path_follower_min_speed=max(0.0, float(path_follower_min_speed)),
            path_follower_yaw_rate_gain=max(0.0, float(path_follower_yaw_rate_gain)),
            path_follower_stop_yaw_rate_gain=max(
                0.0,
                float(path_follower_stop_yaw_rate_gain),
            ),
            path_follower_dir_diff_thre=max(0.0, float(path_follower_dir_diff_thre)),
            path_follower_max_speed=max(0.02, float(nav_max_linear_speed)),
            path_follower_max_yaw_rate=max(0.02, float(nav_max_angular_z)),
            path_follower_turn_speed_yaw_rate_start=max(
                0.0,
                float(nav_turn_speed_yaw_rate_start),
            ),
            path_follower_turn_speed_min_scale=max(
                0.0,
                min(1.0, float(nav_turn_speed_min_scale)),
            ),
            cmd_vel_mux_source_timeout=max(0.02, float(cmd_vel_mux_source_timeout)),
            path_follower_two_way_drive=False,
            latch_stop_signal=False,
            safety_stop_wiring=False,
            run_startup_checks=False,
        ),
    )

    return MuJoCoLingTuStack(
        system=system,
        driver=_optional_module(system, "SimEndpointDriverModule"),
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("nav.mission"),
        local_planner=system.get_module("nav.local_planner"),
        path_follower=_optional_module(system, "nav.path_follower"),
        cmd_vel_mux=_optional_module(system, "nav.velocity_mux"),
    )


def build_fastlio2_tare_stack(
    *,
    cloud_topic: str = TOPICS.registered_cloud,
    cmd_vel_topic: str = TOPICS.cmd_vel,
    tare_scenario: str = "indoor",
    tare_goal_timeout: float = 180.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
    cmd_vel_mux_source_timeout: float = 0.5,
) -> MuJoCoLingTuStack:
    """Build the canonical LingTu TARE stack for MuJoCo + Fast-LIO live runs.

    MuJoCo owns raw MID-360/IMU rendering, Fast-LIO owns live localization and
    map cloud generation, and TARE owns exploration target selection. LingTu
    keeps navigation, local planning, path following, cmd_vel muxing, and the
    canonical /nav/* output contract.
    """

    system = _build_system_from_profile(
        "sim_mujoco_live",
        dict(
            robot="sim_endpoint",
            slam_profile="none",
            detector="sim_scene",
            llm="mock",
            planner_backend="octoplanner3d",
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            enable_map_modules=True,
            enable_frontier=False,
            exploration_backend="tare",
            exploration_auto_start=False,
            tare_scenario=tare_scenario,
            goal_frame_id=MUJOCO_LIVE_GOAL_FRAME_ID,
            way_point_timeout_s=max(10.0, float(tare_goal_timeout) * 0.5),
            hold_active_goal_until_terminal=True,
            max_waypoint_distance_m=6.0,
            waypoint_odometry_timeout_s=5.0,
            prefer_path_strategy=False,
            enable_nav_out=False,
            enable_map_out=False,
            cloud_topic=cloud_topic,
            cmd_vel_topic=cmd_vel_topic,
            planning_frame_id=MUJOCO_LIVE_PLANNING_FRAME_ID,
            occupancy_frame_id=MUJOCO_LIVE_OCCUPANCY_FRAME_ID,
            occupancy_raycast_free_space=True,
            occupancy_unknown_as_obstacle_for_costmap=True,
            occupancy_raycast_max_rays=1600,
            occupancy_raycast_free_inflation_radius=0.4,
            grid_radius=15.0,
            grid_resolution=0.2,
            inflation_radius=0.25,
            waypoint_threshold=0.55,
            final_waypoint_threshold=0.55,
            stuck_timeout=max(60.0, float(tare_goal_timeout) * 0.5),
            stuck_dist_thre=0.05,
            max_replan_count=6,
            mission_timeout=max(300.0, float(tare_goal_timeout) * 3.0),
            defer_empty_path_planning_failure=True,
            empty_path_retry_interval_s=2.0,
            empty_path_retry_timeout_s=min(90.0, max(20.0, float(tare_goal_timeout) * 0.5)),
            downsample_dist=0.35,
            safe_goal_tolerance=6.0,
            plan_safety_policy="reject",
            python_autonomy_backend="nanobind",
            python_path_follower_backend="nav_kernel",
            local_planner_allow_direct_track_fallback=True,
            local_planner_ignore_near_field_stop=True,
            local_planner_direct_track_fallback_min_distance_m=0.3,
            local_planner_min_trackable_local_path_m=0.3,
            path_follower_goal_tolerance=0.12,
            path_follower_max_speed=max(0.02, float(nav_max_linear_speed)),
            path_follower_max_yaw_rate=max(0.02, float(nav_max_angular_z)),
            path_follower_turn_speed_yaw_rate_start=max(
                0.0,
                float(nav_turn_speed_yaw_rate_start),
            ),
            path_follower_turn_speed_min_scale=max(
                0.0,
                min(1.0, float(nav_turn_speed_min_scale)),
            ),
            cmd_vel_mux_source_timeout=max(0.02, float(cmd_vel_mux_source_timeout)),
            path_follower_two_way_drive=False,
            latch_stop_signal=False,
            safety_stop_wiring=False,
            run_startup_checks=False,
        ),
    )

    return MuJoCoLingTuStack(
        system=system,
        driver=_optional_module(system, "SimEndpointDriverModule"),
        tare=system.get_module("TAREExplorerModule"),
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("nav.mission"),
        local_planner=system.get_module("nav.local_planner"),
        path_follower=_optional_module(system, "nav.path_follower"),
        cmd_vel_mux=_optional_module(system, "nav.velocity_mux"),
    )
