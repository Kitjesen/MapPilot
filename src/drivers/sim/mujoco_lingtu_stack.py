"""LingTu module stack construction for MuJoCo live simulation endpoints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.runtime_interface import TOPICS, topic_default_frame_id


# Fast-LIO live simulation owns a continuous local world frame named odom.
# Saved-map relocalization validates map-frame navigation separately.
MUJOCO_LIVE_PLANNING_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_LIVE_OCCUPANCY_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_LIVE_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.odometry)


@dataclass(frozen=True)
class MuJoCoLingTuStack:
    """Resolved LingTu modules used by the MuJoCo live endpoint."""

    system: Any
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


def build_fastlio2_frontier_stack(
    *,
    cloud_topic: str = TOPICS.registered_cloud,
    cmd_vel_topic: str = TOPICS.cmd_vel,
    frontier_goal_timeout: float = 120.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
) -> MuJoCoLingTuStack:
    """Build the canonical LingTu frontier stack for MuJoCo + Fast-LIO live runs.

    The simulator owns raw MID-360/IMU rendering and command application.
    LingTu owns occupancy mapping, frontier selection, global planning, local
    planning, path following, cmd_vel muxing, and canonical /nav/* output.
    """

    from core.blueprints.full_stack import full_stack_blueprint

    system = full_stack_blueprint(
        robot="sim_ros2",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=True,
        enable_frontier=True,
        enable_ros2_path_bridge=True,
        enable_ros2_grid_bridge=True,
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
        # defaults remain in NavigationModule and profile configs.
        stuck_timeout=max(45.0, float(frontier_goal_timeout) * 0.5),
        stuck_dist_thre=0.05,
        max_replan_count=6,
        mission_timeout=max(300.0, float(frontier_goal_timeout) * 3.0),
        downsample_dist=0.35,
        safe_goal_tolerance=6.0,
        plan_safety_policy="reject",
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
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
        path_follower_two_way_drive=False,
        latch_stop_signal=False,
        safety_stop_wiring=False,
        run_startup_checks=False,
    ).build()

    return MuJoCoLingTuStack(
        system=system,
        frontier=system.get_module("WavefrontFrontierExplorer"),
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("NavigationModule"),
        local_planner=system.get_module("LocalPlannerModule"),
        path_follower=_optional_module(system, "PathFollowerModule"),
        cmd_vel_mux=_optional_module(system, "CmdVelMux"),
    )


def build_fastlio2_inspection_stack(
    *,
    cloud_topic: str = TOPICS.registered_cloud,
    cmd_vel_topic: str = TOPICS.cmd_vel,
    planner_backend: str = "astar",
    tomogram: str = "",
    replan_on_costmap_update: bool | None = None,
    inspection_goal_timeout: float = 90.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
) -> MuJoCoLingTuStack:
    """Build the LingTu patrol/inspection stack for MuJoCo + Fast-LIO live runs.

    The inspection gate uses scripted patrol goals, but localization and local
    map inputs remain live Fast-LIO outputs routed through the canonical /nav/*
    contract. This keeps the scenario close to a real inspection mission: map,
    plan, track, and replan while the robot is moving.
    """

    from core.blueprints.full_stack import full_stack_blueprint

    planner_backend = str(planner_backend or "astar").strip().lower()
    if replan_on_costmap_update is None:
        replan_on_costmap_update = planner_backend != "pct"

    system = full_stack_blueprint(
        robot="sim_ros2",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend=planner_backend,
        tomogram=tomogram,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=True,
        enable_frontier=False,
        exploration_backend="none",
        enable_ros2_path_bridge=True,
        enable_ros2_grid_bridge=True,
        cloud_topic=cloud_topic,
        cmd_vel_topic=cmd_vel_topic,
        planning_frame_id=MUJOCO_LIVE_PLANNING_FRAME_ID,
        occupancy_frame_id=MUJOCO_LIVE_OCCUPANCY_FRAME_ID,
        goal_frame_id=MUJOCO_LIVE_GOAL_FRAME_ID,
        occupancy_raycast_free_space=True,
        occupancy_unknown_as_obstacle_for_costmap=True,
        occupancy_raycast_max_rays=1600,
        occupancy_raycast_free_inflation_radius=0.4,
        grid_radius=12.0,
        grid_resolution=0.2,
        inflation_radius=0.25,
        waypoint_threshold=0.50,
        final_waypoint_threshold=0.50,
        stuck_timeout=max(45.0, float(inspection_goal_timeout) * 0.5),
        stuck_dist_thre=0.05,
        max_replan_count=6,
        mission_timeout=max(180.0, float(inspection_goal_timeout) * 3.0),
        defer_empty_path_planning_failure=True,
        empty_path_retry_interval_s=2.0,
        empty_path_retry_timeout_s=min(45.0, max(12.0, float(inspection_goal_timeout) * 0.5)),
        downsample_dist=0.35,
        safe_goal_tolerance=6.0,
        plan_safety_policy="reject",
        replan_on_costmap_update=bool(replan_on_costmap_update),
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
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
        path_follower_two_way_drive=False,
        latch_stop_signal=False,
        safety_stop_wiring=False,
        run_startup_checks=False,
    ).build()

    return MuJoCoLingTuStack(
        system=system,
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("NavigationModule"),
        local_planner=system.get_module("LocalPlannerModule"),
        path_follower=_optional_module(system, "PathFollowerModule"),
        cmd_vel_mux=_optional_module(system, "CmdVelMux"),
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
) -> MuJoCoLingTuStack:
    """Build the canonical LingTu TARE stack for MuJoCo + Fast-LIO live runs.

    MuJoCo owns raw MID-360/IMU rendering, Fast-LIO owns live localization and
    map cloud generation, and TARE owns exploration target selection. LingTu
    keeps navigation, local planning, path following, cmd_vel muxing, and the
    canonical /nav/* output contract.
    """

    from core.blueprints.full_stack import full_stack_blueprint

    system = full_stack_blueprint(
        robot="sim_ros2",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        planner_backend="astar",
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
        enable_ros2_path_bridge=True,
        enable_ros2_grid_bridge=True,
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
        python_path_follower_backend="nav_core",
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
        path_follower_two_way_drive=False,
        latch_stop_signal=False,
        safety_stop_wiring=False,
        run_startup_checks=False,
    ).build()

    return MuJoCoLingTuStack(
        system=system,
        tare=system.get_module("TAREExplorerModule"),
        occupancy_grid=system.get_module("OccupancyGridModule"),
        navigation=system.get_module("NavigationModule"),
        local_planner=system.get_module("LocalPlannerModule"),
        path_follower=_optional_module(system, "PathFollowerModule"),
        cmd_vel_mux=_optional_module(system, "CmdVelMux"),
    )
