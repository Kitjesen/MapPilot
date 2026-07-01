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
from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS

_ACTIVE_OCTOPLANNER3D_MAP = _resolve_octoplanner3d_map()
_THUNDER_OCTOPLANNER3D_CONSTRAINTS = dict(
    octoplanner3d_robot_radius=0.60,
    octoplanner3d_max_iterations=800000,
    octoplanner3d_snap_search_radius_cells=12,
    octoplanner3d_require_ground_support=True,
    octoplanner3d_strict_direct_ground_support=False,
    octoplanner3d_ground_support_xy_radius_cells=1,
    octoplanner3d_ground_support_depth_cells=1,
    octoplanner3d_enable_preblocked_costmap=True,
    octoplanner3d_preblocked_costmap_radius_cells=3,
    octoplanner3d_preblocked_costmap_weight=2.5,
    octoplanner3d_lowest_traversable_only=False,
)
PRODUCT_PROFILE_CONFIGS: dict[str, dict[str, Any]] = {
    "teleop": dict(
        _desc="Remote-control mode: operator command through Gateway/Teleop, no navigation decisions",
        product_mode="teleop",
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
        _desc="Remote-control obstacle-avoidance mode: operator drives, SLAM/maps/safety may veto unsafe motion",
        product_mode="teleop_avoid",
        slam_profile="localizer",
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
        product_mode="mapping",
        slam_profile="fastlio2",
        llm="mock",
        planner="direct",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
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
        _desc="Tracking mode: follow supplied path/waypoint with local planning and safety, no semantic decisions",
        product_mode="tracking",
        slam_profile="localizer",
        llm="mock",
        planner="direct",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
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
        product_mode="navigation",
        llm="qwen",
        planner="octoplanner3d",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        # OctoPlanner3D is the product 3D global planner. Do not silently
        # downgrade to A*; fail the plan when the native backend or safety gate
        # rejects the route.
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
        # Keep autonomy inside the Module graph. The default local planner and
        # path follower are ROS-free nanobind/nav_kernel backends; legacy ROS2
        # NativeModule backends stay explicit compatibility choices only.
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
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
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
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "explore": dict(
        _desc="Explore unknown area (wavefront frontier under navigation stack)",
        product_mode="exploration",
        slam_profile="fastlio2",
        llm="qwen",
        planner="octoplanner3d",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
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
        product_mode="tare_exploration",
        slam_profile="fastlio2",
        llm="qwen",
        planner="octoplanner3d",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_frontier=False,
        enable_traversable_frontier=False,
        exploration_backend="tare",
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
    "inspection": dict(
        _desc="Inspection mode: semantic/patrol tasks over saved-map navigation",
        product_mode="inspection",
        slam_profile="localizer",
        llm="qwen",
        planner="octoplanner3d",
        tomogram=_ACTIVE_OCTOPLANNER3D_MAP,
        plan_safety_policy="reject",
        fallback_planner_name="",
        **_THUNDER_OCTOPLANNER3D_CONSTRAINTS,
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_goals=True,
        enable_patrol_routes=True,
        enable_scheduler=True,
        planning_frame_id=DEFAULT_PLANNING_FRAME_ID,
        gateway_port=DEFAULT_GATEWAY_PORT,
    ),
}

PRODUCT_INTENT_PROFILES = PRODUCT_PROFILE_CONFIGS

PRODUCT_PROFILES = (
    "teleop",
    "teleop_avoid",
    "lite",
    "map",
    "tracking",
    "nav",
    "inspection",
    "explore",
    "tare_explore",
    "super_lio",
    "super_lio_relocation",
)

PRODUCT_MODE_PROFILES = (
    "teleop",
    "teleop_avoid",
    "map",
    "tracking",
    "nav",
    "inspection",
)

if set(PRODUCT_MODE_PROFILES) != set(PRODUCT_MODE_CONTRACTS):
    raise RuntimeError("product mode profiles and product mode contracts differ")

LIGHTWEIGHT_PRODUCT_PROFILES = ("lite",)
OPTIONAL_NATIVE_PRODUCT_PROFILES: tuple[str, ...] = ()


def product_intent_profile(name: str) -> dict[str, Any]:
    """Return a copy of a Thunder product intent profile."""

    return dict(PRODUCT_PROFILE_CONFIGS[name])
