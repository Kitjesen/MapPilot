"""Product profile catalog.

This module owns LingTu/Thunder runtime profiles. ``core.runtime_profiles``
keeps the legacy import path for CLI and older tooling.
"""

from __future__ import annotations

import os
from typing import Any

from core.blueprints.catalog.runtime_paths import RUNTIME_MAP_FRAME_ID, _resolve_tomogram
from core.runtime_interface import TOPICS

_ACTIVE_TOMOGRAM = _resolve_tomogram()

PROFILES: dict[str, dict[str, Any]] = {
    "lite": dict(
        _desc="Minimal Thunder runtime: driver, safety, pure-Python navigation shell, no ROS/SLAM/semantic stack",
        _default_robot="thunder",
        slam_profile="none",
        llm="mock",
        planner="astar",
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
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "map": dict(
        _desc="Build a new map of the environment",
        _default_robot="thunder",
        slam_profile="fastlio2",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "nav": dict(
        _desc="Navigate using a saved map",
        _default_robot="thunder",
        llm="qwen",
        planner="pct",
        tomogram=_ACTIVE_TOMOGRAM,
        # PCT remains the production planner, but unsafe global plans are not
        # allowed to pass through just because the native backend produced
        # waypoints. Fall back to A* on the same active map if the path safety
        # gate finds blocked samples.
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        # enable_native=False: C++ local_planner requires the 'local_planner'
        # ROS2 package installed via colcon. Use Python autonomy chain instead.
        # Switch to True only after running: make build && ros2 pkg list | grep local_planner
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        # App/Web goals, semantic targets, saved locations, maps, and PCT paths
        # are all contracted in map frame. The selected runtime endpoint chooses
        # whether localization arrives from a managed localizer or compatibility
        # bridge before the Module graph sees it.
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "super_lio": dict(
        _desc="Evaluate Super-LIO as the external LIO backend",
        _default_robot="thunder",
        slam_profile="super_lio",
        llm="qwen",
        planner="pct",
        tomogram=_ACTIVE_TOMOGRAM,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "super_lio_relocation": dict(
        _desc="Evaluate Super-LIO relocation against the active saved map",
        _default_robot="thunder",
        slam_profile="super_lio_relocation",
        llm="qwen",
        planner="pct",
        tomogram=_ACTIVE_TOMOGRAM,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "explore": dict(
        _desc="Explore unknown area (wavefront frontier under navigation stack)",
        _default_robot="thunder",
        slam_profile="fastlio2",
        llm="qwen",
        planner="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
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
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "tare_explore": dict(
        _desc="Explore via CMU TARE hierarchical planner (needs tare_planner submodule built)",
        _default_robot="thunder",
        slam_profile="fastlio2",
        llm="qwen",
        planner="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        # Don't add wavefront explorer; TARE stack handles goal generation.
        enable_frontier=False,
        exploration_backend="tare",
        tare_scenario="forest",
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        gateway_port=5050,
    ),
    "sim": dict(
        _desc="MuJoCo simulation",
        _default_robot="sim",
        llm="mock",
        planner="astar",
        tomogram="src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
        # Simulation can safely exercise fallback behavior without changing
        # the hardware navigation profile.
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        # Keep profile execution inside the Module graph. The ROS2 native
        # localPlanner publishes /nav/local_path, but this profile consumes
        # LocalPlannerModule.local_path, so use the in-process planner here.
        enable_native=False,
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "sim_mujoco_live": dict(
        _desc="MuJoCo raw MID-360 + Fast-LIO live simulation",
        _default_robot="sim_gazebo",
        _external_launcher="sim/scripts/launch_mujoco_fastlio2_live.sh",
        _runtime_contract="mujoco_fastlio2_live",
        slam_profile="none",
        llm="mock",
        planner="astar",
        tomogram="",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_camera=False,
        use_driver_camera=False,
        cloud_topic=TOPICS.map_cloud,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        enable_frontier=True,
        enable_traversable_frontier=True,
        exploration_backend="none",
        frontier_safe_distance=0.80,
        frontier_max_dist=25.0,
        frontier_rate=2.0,
        enable_native=False,
        local_planner_allow_direct_track_fallback=True,
        local_planner_ignore_near_field_stop=True,
        local_planner_direct_track_fallback_min_distance_m=0.3,
        path_follower_goal_tolerance=0.12,
        latch_stop_signal=False,
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
        run_startup_checks=False,
        manage_external_services=False,
        gateway_port=5050,
    ),
    "sim_mujoco_pct_live": dict(
        _desc="MuJoCo Fast-LIO + PCT closed-loop simulation",
        _default_robot="sim_gazebo",
        _external_launcher="sim/scripts/launch_mujoco_fastlio2_live.sh",
        _runtime_contract="mujoco_fastlio2_live",
        slam_profile="none",
        llm="mock",
        planner="pct",
        planner_backend="pct",
        tomogram="src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
        plan_safety_policy="reject",
        fallback_planner_name="",
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_camera=False,
        use_driver_camera=False,
        cloud_topic=TOPICS.map_cloud,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        enable_frontier=False,
        enable_traversable_frontier=False,
        exploration_backend="none",
        enable_native=False,
        local_planner_allow_direct_track_fallback=False,
        local_planner_ignore_near_field_stop=False,
        path_follower_goal_tolerance=0.12,
        latch_stop_signal=False,
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
        run_startup_checks=False,
        manage_external_services=False,
        gateway_port=5050,
    ),
    "sim_gazebo": dict(
        _desc="Gazebo/GZ ROS-native simulation",
        _default_robot="sim_gazebo",
        slam_profile="none",
        llm="mock",
        planner="astar",
        tomogram="src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        enable_camera=True,
        use_driver_camera=True,
        cloud_topic=TOPICS.map_cloud,
        # Gazebo raw topics enter through ROS2SimDriverModule, but LingTu
        # planning and App/Web goal contracts stay in map frame.
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        # The Gazebo profile is ROS-native at the simulator boundary, while
        # planning/tracking stay in the Module graph so /nav/local_path cannot
        # disappear into an unbridged external ROS2 process.
        enable_native=False,
        latch_stop_signal=False,
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
        gateway_port=5050,
    ),
    "sim_industrial": dict(
        _desc="Engineering Gazebo industrial-yard simulation",
        _default_robot="sim_gazebo",
        slam_profile="none",
        llm="mock",
        planner="astar",
        tomogram="src/global_planning/pct_planner/rsc/tomogram/building2_9.pickle",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_semantic=True,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_camera=True,
        use_driver_camera=True,
        cloud_topic=TOPICS.map_cloud,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        enable_frontier=True,
        exploration_backend="none",
        frontier_safe_distance=0.80,
        frontier_max_dist=20.0,
        frontier_rate=2.0,
        enable_native=False,
        latch_stop_signal=False,
        python_autonomy_backend="nanobind",
        python_path_follower_backend="nav_core",
        run_startup_checks=False,
        manage_external_services=False,
        gateway_port=5050,
    ),
    "sim_cmu_tare": dict(
        _desc="CMU Unity + external TARE simulation",
        _default_robot="sim_gazebo",
        _external_launcher="sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        _runtime_contract="cmu_unity_external",
        slam_profile="none",
        llm="mock",
        planner="pct",
        tomogram=os.environ.get("LINGTU_CMU_TOMOGRAM", ""),
        plan_safety_policy=os.environ.get("LINGTU_CMU_PLAN_SAFETY_POLICY", "fallback_astar"),
        fallback_planner_name="astar",
        safe_goal_tolerance=0.4,
        waypoint_threshold=0.45,
        final_waypoint_threshold=0.35,
        stuck_timeout=25.0,
        stuck_dist_thre=0.08,
        downsample_dist=0.6,
        path_follower_goal_tolerance=0.35,
        local_planner_allow_direct_track_fallback=True,
        local_planner_ignore_near_field_stop=True,
        local_planner_direct_track_fallback_min_distance_m=0.3,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_teleop=False,
        enable_map_modules=True,
        enable_endpoint_waypoint_bridge=True,
        enable_endpoint_path_bridge=True,
        exploration_backend="tare_external",
        exploration_auto_start=True,
        prefer_path_strategy=True,
        path_start_tolerance_m=1.5,
        path_goal_min_distance_m=1.0,
        path_goal_spacing_m=0.75,
        tare_fallback_timeout_s=180.0,
        allow_direct_goal_fallback=True,
        direct_goal_fallback_on_planner_failure=True,
        accept_partial_goal_progress=True,
        partial_goal_repeat_ignore_window_s=5.0,
        external_strategy_path_control=False,
        external_strategy_start_tolerance_m=1.5,
        planning_frame_id=RUNTIME_MAP_FRAME_ID,
        latch_stop_signal=False,
        run_startup_checks=False,
        manage_external_services=False,
        gateway_port=5050,
    ),
    "sim_nav": dict(
        _desc="Pure-Python navigation sim (no ROS2/C++)",
        _default_robot="stub",
        slam_profile="none",
        llm="mock",
        planner="astar",
        scene_xml="sim/worlds/mujoco/building_scene.xml",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=True,
        gateway_port=5050,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
    ),
    "dev": dict(
        _desc="Test perception & planning without a robot",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "stub": dict(
        _desc="Framework testing only",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        gateway_port=5050,
    ),
}


SIMULATION_PROFILES = (
    "stub",
    "dev",
    "sim",
    "sim_mujoco_live",
    "sim_mujoco_pct_live",
    "sim_gazebo",
    "sim_industrial",
    "sim_cmu_tare",
    "sim_nav",
)

PRODUCT_PROFILES = (
    "lite",
    "map",
    "nav",
    "explore",
    "tare_explore",
    "super_lio",
    "super_lio_relocation",
)

LIGHTWEIGHT_PRODUCT_PROFILES = ("lite",)
OPTIONAL_NATIVE_PRODUCT_PROFILES = ("tare_explore",)

PROFILE_SNAPSHOT_TARGETS = (
    *SIMULATION_PROFILES,
    *(
        profile
        for profile in PRODUCT_PROFILES
        if profile not in OPTIONAL_NATIVE_PRODUCT_PROFILES
        and profile not in LIGHTWEIGHT_PRODUCT_PROFILES
    ),
)


def product_profile(name: str) -> dict[str, Any]:
    """Return a copy of a named product or simulation profile."""

    return dict(PROFILES[name])
