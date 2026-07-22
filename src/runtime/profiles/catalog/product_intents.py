"""Thunder product intent profiles.

This module owns what each Thunder product mode is meant to do. It does not
describe where data comes from or which compatibility adapter is used; runtime
endpoints layer those details on top.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.profiles.product_mode_contracts import PRODUCT_CONTRACTS, PRODUCT_MODE_CONTRACTS

_ACTIVE_OCTOPLANNER3D_MAP = _resolve_octoplanner3d_map()
THUNDER_MAP_ARTIFACT_CONFIG = dict(
    # Keep the physical 0.25 m body radius. The denser OctoMap removes the
    # coarse 0.2 m voxel quantization that otherwise snaps a valid start pose
    # almost a metre away from the robot.
    octomap_resolution=0.1,
    # Preserve the same 0.6 m free-space envelope used by the former
    # 0.2 m / three-layer artifact.
    octomap_free_layers_above=6,
    octomap_free_dilation_cells=1,
)
THUNDER_OCTO_CONFIG = dict(
    **THUNDER_MAP_ARTIFACT_CONFIG,
    preview_timeout=30.0,
    octoplanner3d_timeout_s=30.0,
    octoplanner3d_robot_radius=0.25,
    octoplanner3d_max_iterations=500000,
    octoplanner3d_snap_search_radius_cells=24,
    octoplanner3d_require_ground_support=True,
    octoplanner3d_strict_direct_ground_support=False,
    octoplanner3d_ground_support_xy_radius_cells=2,
    octoplanner3d_ground_support_depth_cells=2,
    octoplanner3d_enable_preblocked_costmap=True,
    octoplanner3d_preblocked_costmap_radius_cells=3,
    octoplanner3d_preblocked_costmap_weight=2.5,
    octoplanner3d_lowest_traversable_only=False,
    octoplanner3d_floor_change_penalty=6.0,
    octoplanner3d_max_step_height=0.45,
    octoplanner3d_max_slope=0.0,
    octoplanner3d_same_floor_preference=True,
    octoplanner3d_same_floor_z_tolerance=0.75,
    octoplanner3d_max_same_floor_z_excursion=2.0,
    octoplanner3d_obstacle_clearance_radius_cells=2,
    octoplanner3d_obstacle_clearance_weight=1.5,
)
PRODUCT_PROFILE_CONFIGS: dict[str, dict[str, Any]] = {
    "teleop": dict(
        _desc="Remote-control mode: operator command through Gateway/Teleop, no navigation decisions",
        slam_profile="none",
        llm="mock",
        planner="direct",
        enable_navigation=False,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=True,
        enable_map_modules=False,
        enable_goals=False,
        enable_patrol_routes=False,
        enable_scheduler=False,
        enable_gnss=False,
        manage_external_services=False,
        run_startup_checks=False,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "teleop_avoid": dict(
        _desc="Native assisted teleoperation with live SLAM, local obstacle detours, and final command safety",
        slam_profile="fastlio2",
        llm="mock",
        planner="direct",
        enable_navigation=False,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=True,
        enable_map_modules=True,
        cmd_vel_mux_collision_monitor=True,
        enable_goals=False,
        enable_patrol_routes=False,
        enable_scheduler=False,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        teleop_planner_horizon_m=2.0,
        teleop_planner_max_deviation_deg=55.0,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "lite": dict(
        _desc="Minimal Thunder runtime: driver, safety, pure-Python navigation shell, no ROS/SLAM/semantic stack",
        product_mode="lite",
        runtime_mode="lite",
        slam_profile="none",
        llm="mock",
        planner="direct",
        enable_native=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        enable_semantic=False,
        enable_gateway=False,
        enable_teleop=False,
        enable_map_modules=False,
        enable_gnss=False,
        manage_external_services=False,
        run_startup_checks=False,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "map": dict(
        _desc="Build a new map of the environment",
        slam_profile="fastlio2",
        llm="mock",
        planner="direct",
        enable_navigation=False,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=True,
        enable_map_modules=True,
        enable_goals=False,
        enable_patrol_routes=False,
        enable_scheduler=False,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "tracking": dict(
        _desc="Follow explicit map-frame goals with native planning and safety, without semantic goal selection",
        slam_profile="localizer",
        llm="mock",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "nav": dict(
        _desc="Navigate using a saved map",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        # OctoPlanner3D is the product 3D global planner. Do not silently
        # downgrade to A*; fail the plan when the native backend or safety gate
        # rejects the route.
        plan_safety_policy="reject",
        fallback_planner_name="",
        waypoint_threshold=0.20,
        final_waypoint_threshold=0.10,
        native_corridor_lookahead_m=3.0,
        local_planner_direct_track_fallback_min_distance_m=0.05,
        local_planner_min_trackable_local_path_m=0.05,
        path_follower_goal_tolerance=0.05,
        path_follower_lookahead=0.35,
        path_follower_max_speed=0.20,
        path_follower_max_accel=1.0,
        path_follower_min_speed=0.08,
        **THUNDER_OCTO_CONFIG,
        # This historical flag only disables legacy Python NativeModule
        # backends. The thunder_field endpoint still owns navigation in C++.
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        # App/Web goals, semantic targets, saved locations, maps, and global paths
        # are all contracted in map frame. The selected runtime endpoint chooses
        # whether localization arrives from a managed localizer or compatibility
        # bridge before the Module graph sees it.
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "super_lio": dict(
        _desc="Evaluate Super-LIO as the external LIO backend",
        product_mode="super_lio_eval",
        slam_profile="super_lio",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **THUNDER_OCTO_CONFIG,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "super_lio_relocation": dict(
        _desc="Evaluate Super-LIO relocation against the active saved map",
        product_mode="super_lio_relocation_eval",
        slam_profile="super_lio_relocation",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **THUNDER_OCTO_CONFIG,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "explore": dict(
        _desc="Explore unknown area (wavefront frontier under navigation stack)",
        slam_profile="fastlio2",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **THUNDER_OCTO_CONFIG,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        # WavefrontFrontierExplorer is provided by navigation() stack (via
        # enable_frontier=True), not by exploration() stack; the latter is
        # TARE-only since 1c457f3. Keep exploration_backend="none" so we
        # don't try to spawn a TARE NativeModule on top of wavefront.
        enable_frontier=True,
        enable_traversable_frontier=True,
        exploration_backend="none",
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "tare_explore": dict(
        _desc="Explore via LingTu-owned TARE-style frontier/viewpoint selection",
        # The current native global planner consumes a validated saved OctoMap.
        # Live ExplorationGrid snapshots select coverage goals but do not yet replace it.
        slam_profile="localizer",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        map_artifact_gate_required=True,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **THUNDER_OCTO_CONFIG,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        enable_frontier=False,
        enable_traversable_frontier=False,
        exploration_backend="tare",
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "inspection": dict(
        _desc="Inspection mode: native C++ multi-point routes over saved-map navigation",
        slam_profile="localizer",
        llm="qwen",
        planner="octoplanner3d",
        map_path=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **THUNDER_OCTO_CONFIG,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_goals=True,
        enable_inspection_evidence=True,
        enable_patrol_routes=False,
        enable_scheduler=False,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
}

for _profile_name, _contract in PRODUCT_CONTRACTS.items():
    if _profile_name not in PRODUCT_PROFILE_CONFIGS:
        continue
    PRODUCT_PROFILE_CONFIGS[_profile_name]["product_mode"] = _contract.product_mode

PRODUCT_INTENT_PROFILES = PRODUCT_PROFILE_CONFIGS

PRODUCT_PROFILES = tuple(PRODUCT_PROFILE_CONFIGS)
PRODUCT_MODE_PROFILES = tuple(PRODUCT_MODE_CONTRACTS)

LIGHTWEIGHT_PRODUCT_PROFILES = ("lite",)
OPTIONAL_NATIVE_PRODUCT_PROFILES: tuple[str, ...] = ()


def product_intent_profile(name: str) -> dict[str, Any]:
    """Return a copy of a Thunder product intent profile."""

    return dict(PRODUCT_PROFILE_CONFIGS[name])
