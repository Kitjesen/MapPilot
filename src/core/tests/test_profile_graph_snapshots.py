import sys

import pytest

pytestmark = [pytest.mark.sim]

from core.blueprints.profile_graph import (
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    PRODUCT_PROFILES,
    SIMULATION_PROFILES,
    graph_for_profile,
    resolve_profile_config,
)
from core.blueprints.runtime_endpoint import RuntimeEndpointError, runtime_endpoint
from core.blueprints.simulation_contract import (
    CANONICAL_NAV_TOPICS,
    SIMULATION_RUNTIME_CONTRACTS,
    runtime_contracts_for_profile,
    simulation_runtime_contract,
)
from core.blueprints.stacks.navigation import navigation
from core.runtime_interface import DATA_SOURCE_CONTRACTS, TOPICS, profile_data_source
from cli.profiles_data import PROFILES


REAL_LOCALIZATION_PROFILES = (
    "map",
    "nav",
    "explore",
    "super_lio",
    "super_lio_relocation",
)


def test_top_level_blueprint_api_exposes_all_stack_factories():
    import core.blueprints as blueprints

    for name in (
        "driver",
        "exploration",
        "gateway",
        "lidar",
        "maps",
        "memory",
        "navigation",
        "perception",
        "planner",
        "safety",
        "sim_lidar",
        "slam",
    ):
        assert callable(getattr(blueprints, name))
        assert name in blueprints.__all__


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def test_profile_graphs_compile_for_primary_profiles():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)

        assert "NavigationModule" in graph.modules
        assert "SafetyRingModule" in graph.modules
        assert "GatewayModule" in graph.modules
        assert "MCPServerModule" in graph.modules
        assert not graph.dangling_wires(), profile


def test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        driver = next(
            module
            for module in modules
            if module.endswith("Driver")
            or module.endswith("DriverModule")
            or module.endswith("DogModule")
        )

        assert f"SafetyRingModule.stop_cmd->{driver}.stop_signal" in wires
        assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert f"GatewayModule.stop_cmd->{driver}.stop_signal" in wires
        assert "GatewayModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert f"MCPServerModule.stop_cmd->{driver}.stop_signal" in wires
        assert "MCPServerModule.stop_cmd->NavigationModule.stop_signal" in wires
        if "GeofenceManagerModule" in modules:
            assert f"GeofenceManagerModule.stop_cmd->{driver}.stop_signal" in wires
            assert "GeofenceManagerModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert "GatewayModule.cmd_vel->CmdVelMux.teleop_cmd_vel" in wires
        assert "MCPServerModule.cmd_vel->CmdVelMux.teleop_cmd_vel" in wires
        assert "NavigationModule.mission_status->GatewayModule.mission_status" in wires
        assert "NavigationModule.mission_status->MCPServerModule.mission_status" in wires
        assert "NavigationModule.clear_path->LocalPlannerModule.clear_path" in wires
        assert "NavigationModule.global_path->LocalPlannerModule.global_path" in wires
        assert "LocalPlannerModule.control_hint->PathFollowerModule.control_hint" in wires
        assert "GatewayModule.cancel->NavigationModule.cancel" in wires
        assert f"CmdVelMux.driver_cmd_vel->{driver}.cmd_vel" in wires


def test_profile_graph_snapshot_locks_mapping_and_algorithm_edges():
    for profile in (
        "stub",
        "dev",
        "sim",
        "sim_mujoco_live",
        "sim_gazebo",
        "sim_industrial",
        "sim_cmu_tare",
        "map",
        "nav",
        "explore",
    ):
        wires = _wire_set(graph_for_profile(profile))

        assert "OccupancyGridModule.costmap->TraversabilityCostModule.costmap" in wires
        assert "ESDFModule.esdf->TraversabilityCostModule.esdf" in wires
        assert "TraversabilityCostModule.fused_cost->NavigationModule.costmap" in wires
        assert "TraversabilityCostModule.fused_cost->GatewayModule.costmap" in wires


# Profiles that assemble the full map + autonomy chain (occupancy/ESDF ->
# traversability -> nav, plus terrain -> local planner -> path follower).
_NAV_CONTRACT_PROFILES = (
    "stub",
    "dev",
    "sim",
    "sim_gazebo",
    "sim_industrial",
    "map",
    "nav",
    "explore",
)


def test_navigation_compute_contract_layer_main_edges():
    """NAV COMPUTE CONTRACT: the four-layer main chain must be wired.

    docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md §2/§7.
      L5 global -> L2 gating -> L2 local -> L2 control.
    """
    for profile in _NAV_CONTRACT_PROFILES:
        wires = _wire_set(graph_for_profile(profile))

        # L2 safety gating: fused_cost (risk grid) -> global gate only.
        assert (
            "TraversabilityCostModule.fused_cost->NavigationModule.costmap" in wires
        ), profile
        # L2 local planning: terrain_map (local geometry) is the main local input.
        assert (
            "TerrainModule.terrain_map->LocalPlannerModule.terrain_map" in wires
        ), profile
        # L5 -> L2 command dispatch (global_path + waypoint as staged goals).
        assert (
            "NavigationModule.global_path->LocalPlannerModule.global_path" in wires
        ), profile
        assert (
            "NavigationModule.waypoint->LocalPlannerModule.waypoint" in wires
        ), profile
        # L2 control tracking: local_path -> path follower -> cmd_vel.
        assert (
            "LocalPlannerModule.local_path->PathFollowerModule.local_path" in wires
        ), profile


def test_navigation_compute_contract_forbids_role_drift_edges():
    """NAV COMPUTE CONTRACT §3/§7: forbidden edges must never appear.

    - fused_cost/costmap must NOT feed the local planner's primary scoring.
    - terrain_map (local geometry) must NOT feed global strategy.
    """
    for profile in PROFILE_SNAPSHOT_TARGETS:
        wires = _wire_set(graph_for_profile(profile))
        for wire in wires:
            assert not wire.startswith(
                "TraversabilityCostModule.fused_cost->LocalPlannerModule"
            ), f"{profile}: costmap must not be a local-planner input ({wire})"
            assert not wire.startswith(
                "TerrainModule.terrain_map->NavigationModule"
            ), f"{profile}: terrain_map must not drive global planning ({wire})"


def test_nav_profile_uses_slam_bridge_localization_health_edges():
    wires = _wire_set(graph_for_profile("nav"))

    assert "SlamBridgeModule.localization_status->SafetyRingModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->NavigationModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->DepthVisualOdomModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->GatewayModule.localization_status" in wires
    assert "SlamBridgeModule.map_frame_jump_event->NavigationModule.map_frame_jump_event" in wires
    assert "SlamBridgeModule.map_frame_jump_event->LocalPlannerModule.map_frame_jump_event" in wires
    assert "SlamBridgeModule.map_frame_jump_event->PathFollowerModule.map_frame_jump_event" in wires


def test_s100p_profiles_drive_real_brainstem_driver_by_default():
    for profile in ("map", "nav", "explore"):
        graph = graph_for_profile(profile)

        assert "ThunderDriver" in graph.modules
        assert "ROS2SimDriverModule" not in graph.modules


    @pytest.mark.sim
def test_sim_gazebo_profile_uses_ros2_driver_and_map_planning_frame():
    graph = graph_for_profile("sim_gazebo")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_gazebo")

    assert config["robot"] == "sim_ros2"
    assert config["slam_profile"] == "none"
    assert config["planning_frame_id"] == "map"
    assert config["enable_native"] is False
    assert config["latch_stop_signal"] is False
    assert config["python_autonomy_backend"] == "nanobind"
    assert config["python_path_follower_backend"] == "nav_core"
    assert config["enable_camera"] is True
    assert config["use_driver_camera"] is True
    assert config["cloud_topic"] == "/nav/map_cloud"
    assert "ROS2SimDriverModule" in graph.modules
    assert "CameraBridgeModule" not in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "ROS2SimDriverModule.map_cloud->TerrainModule.map_cloud" in wires
    assert "ROS2SimDriverModule.odometry->NavigationModule.odometry" in wires
    assert "SlamBridgeModule" not in graph.modules
    assert not graph.dangling_wires()


    @pytest.mark.sim
def test_sim_industrial_profile_is_lingtu_owned_exploration_profile():
    graph = graph_for_profile("sim_industrial")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_industrial")

    assert config["robot"] == "sim_ros2"
    assert config["slam_profile"] == "none"
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["latch_stop_signal"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["cloud_topic"] == "/nav/map_cloud"
    assert "ROS2SimDriverModule" in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "WavefrontFrontierExplorer.exploration_goal->NavigationModule.goal_pose" in wires
    assert "NavigationModule.mission_status->WavefrontFrontierExplorer.navigation_status" in wires
    assert "ROS2SimDriverModule.odometry->WavefrontFrontierExplorer.odometry" in wires
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "NavigationModule.global_path->LocalPlannerModule.global_path" in wires
    assert "LocalPlannerModule.local_path->PathFollowerModule.local_path" in wires
    assert not graph.dangling_wires()


    @pytest.mark.sim
def test_sim_mujoco_live_profile_is_raw_fastlio_simulation_entry():
    graph = graph_for_profile("sim_mujoco_live")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_mujoco_live")

    assert config["robot"] == "sim_ros2"
    assert config["slam_profile"] == "none"
    assert PROFILES["sim_mujoco_live"]["_external_launcher"] == (
        "sim/scripts/launch_mujoco_fastlio2_live.sh"
    )
    assert PROFILES["sim_mujoco_live"]["_runtime_contract"] == "mujoco_fastlio2_live"
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["cloud_topic"] == "/nav/map_cloud"
    assert "ROS2SimDriverModule" in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "ROS2SimDriverModule.odometry->NavigationModule.odometry" in wires
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "ROS2SimDriverModule.map_cloud->TerrainModule.map_cloud" in wires
    assert "WavefrontFrontierExplorer.exploration_goal->NavigationModule.goal_pose" in wires
    assert "NavigationModule.mission_status->WavefrontFrontierExplorer.navigation_status" in wires
    assert "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers" in wires
    assert "TraversableFrontierModule.frontier_candidate->GatewayModule.frontier_candidate" in wires
    assert "TraversableFrontierModule.frontier_candidate->NavigationModule.goal_pose" not in wires
    assert not graph.dangling_wires()


def test_product_explore_can_run_on_mujoco_live_endpoint():
    endpoint = runtime_endpoint("mujoco_live")
    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    graph = graph_for_profile("explore", runtime_endpoint="mujoco_live")
    wires = _wire_set(graph)

    assert endpoint.data_source == "mujoco_fastlio2_live"
    assert endpoint.robot_preset == "sim_gazebo"
    assert "explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_ros2"
    assert config["_runtime_endpoint"] == "mujoco_live"
    assert config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert config["_external_launcher"] == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert config["_external_default_args"] == ("explore",)
    assert config["_external_record_args"] == ("video",)
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["cloud_topic"] == "/nav/map_cloud"
    assert "ROS2SimDriverModule" in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "ROS2SimDriverModule.odometry->NavigationModule.odometry" in wires
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "WavefrontFrontierExplorer.exploration_goal->NavigationModule.goal_pose" in wires
    assert "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers" in wires
    assert "TraversableFrontierModule.frontier_candidate->GatewayModule.frontier_candidate" in wires
    assert "TraversableFrontierModule.frontier_candidate->NavigationModule.goal_pose" not in wires
    assert not graph.dangling_wires()


def test_runtime_run_spec_carries_endpoint_command_and_safety_boundary():
    from core.blueprints.runtime_endpoint import resolve_runtime_run_spec

    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    spec = resolve_runtime_run_spec(
        "explore",
        config,
        record=True,
        extra_args=(),
    )

    assert spec.profile == "explore"
    assert spec.endpoint == "mujoco_live"
    assert spec.data_source == "mujoco_fastlio2_live"
    assert spec.runtime_contract == "mujoco_fastlio2_live"
    assert spec.simulation_only is True
    assert spec.command_sink == "mujoco_velocity_adapter"
    assert spec.slam_source == "lingtu_fastlio2"
    assert spec.localization_source == "fastlio2_odometry"
    assert spec.mapping_source == "fastlio2_map_cloud"
    assert spec.lidar_extrinsic_profile == "mujoco_thunder_v3"
    assert spec.launcher == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert spec.launcher_args == ("video",)
    assert spec.env["LINGTU_PROFILE"] == "explore"
    assert spec.env["LINGTU_ENDPOINT"] == "mujoco_live"
    assert spec.env["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert spec.env["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert spec.env["LINGTU_COMMAND_SINK"] == "mujoco_velocity_adapter"
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"
    assert spec.as_command() == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "video",
    ]


def test_replay_endpoint_is_no_actuation_runtime():
    from core.blueprints.runtime_endpoint import runtime_endpoint, resolve_runtime_run_spec

    endpoint = runtime_endpoint("replay")
    config = resolve_profile_config("nav", runtime_endpoint="replay")
    spec = resolve_runtime_run_spec("nav", config, extra_args=("status",))

    assert endpoint.simulation_only is True
    assert endpoint.data_source == "rosbag_fastlio2_replay"
    assert DATA_SOURCE_CONTRACTS["rosbag_fastlio2_replay"].command_sink == (
        "no_actuation_replay_sink"
    )
    assert spec.command_sink == "no_actuation_replay_sink"
    assert spec.simulation_only is True
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"
    assert spec.as_command() == [
        sys.executable,
        "sim/scripts/fastlio2_rosbag_replay_gate.py",
        "status",
    ]


def test_real_runtime_run_spec_carries_hardware_contract_boundary():
    from core.blueprints.runtime_endpoint import resolve_runtime_run_spec

    config = resolve_profile_config("nav")
    spec = resolve_runtime_run_spec("nav", config)

    assert spec.endpoint == "real_s100p"
    assert spec.data_source == "real_s100p"
    assert spec.runtime_contract == "real_s100p"
    assert spec.simulation_only is False
    assert spec.command_sink == "hardware_driver_after_cmd_vel_mux"
    assert spec.slam_source == "lingtu_fastlio_or_external_robot_slam"
    assert spec.localization_source == "slam_localizer"
    assert spec.mapping_source == "slam_map_cloud"
    assert spec.lidar_extrinsic_profile == "real_mid360"
    assert spec.launcher is None
    assert spec.launcher_args == ()
    assert spec.env["LINGTU_ENDPOINT"] == "real_s100p"
    assert spec.env["LINGTU_DATA_SOURCE"] == "real_s100p"
    assert spec.env["LINGTU_RUNTIME_CONTRACT"] == "real_s100p"
    assert spec.env["LINGTU_COMMAND_SINK"] == "hardware_driver_after_cmd_vel_mux"
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "0"


def test_product_tare_can_run_on_mujoco_live_endpoint():
    endpoint = runtime_endpoint("mujoco_live")
    config = resolve_profile_config("tare_explore", runtime_endpoint="mujoco_live")

    assert endpoint.data_source == "mujoco_fastlio2_live"
    assert endpoint.robot_preset == "sim_gazebo"
    assert "tare_explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_ros2"
    assert config["_runtime_endpoint"] == "mujoco_live"
    assert config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert config["_external_launcher"] == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert config["_external_default_args"] == ("tare",)
    assert config["_external_record_args"] == ("tare-video",)
    assert config["planner"] == "astar"
    assert config["planner_backend"] == "astar"
    assert config["planning_frame_id"] == "odom"
    assert config["goal_frame_id"] == "odom"
    assert config["exploration_backend"] == "tare"
    assert config["enable_frontier"] is False
    assert config["hold_active_goal_until_terminal"] is True


def test_product_tare_can_run_on_cmu_unity_endpoint():
    endpoint = runtime_endpoint("cmu_unity")
    config = resolve_profile_config("tare_explore", runtime_endpoint="cmu_unity")
    graph = graph_for_profile("tare_explore", runtime_endpoint="cmu_unity")
    wires = _wire_set(graph)

    assert endpoint.data_source == "cmu_unity_external"
    assert endpoint.robot_preset == "sim_gazebo"
    assert "tare_explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_ros2"
    assert config["_runtime_endpoint"] == "cmu_unity"
    assert config["_endpoint_data_source"] == "cmu_unity_external"
    assert config["_external_launcher"] == "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    assert config["_external_default_args"] == ("gate",)
    assert config["_external_record_args"] == ("start", "--gate", "--rviz")
    assert config["planner"] == "pct"
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_ros2_bridge"] is True
    assert config["enable_ros2_path_bridge"] is True
    assert "ROS2SimDriverModule" in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "ROS2PathBridgeModule" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->NavigationModule.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->NavigationModule.patrol_goals" in wires
    assert "NavigationModule.global_path->ROS2PathBridgeModule.global_path" in wires
    assert not graph.dangling_wires()


def test_endpoint_rejects_unsupported_task_pairings():
    with pytest.raises(RuntimeEndpointError):
        resolve_profile_config("nav", runtime_endpoint="cmu_unity")


def test_sim_cmu_tare_profile_is_external_tare_simulation_entry():
    graph = graph_for_profile("sim_cmu_tare")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_cmu_tare")

    assert config["robot"] == "sim_ros2"
    assert config["slam_profile"] == "none"
    assert PROFILES["sim_cmu_tare"]["_external_launcher"] == (
        "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    )
    assert PROFILES["sim_cmu_tare"]["_runtime_contract"] == "cmu_unity_external"
    assert config["planner"] == "pct"
    assert config["planning_frame_id"] == "map"
    assert config["enable_ros2_bridge"] is True
    assert config["enable_ros2_path_bridge"] is True
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["local_planner_allow_direct_track_fallback"] is True
    assert config["local_planner_ignore_near_field_stop"] is True
    assert config["local_planner_direct_track_fallback_min_distance_m"] == pytest.approx(0.3)
    assert config["prefer_path_strategy"] is True
    assert config["external_strategy_path_control"] is False
    assert config["partial_goal_repeat_ignore_window_s"] == pytest.approx(5.0)
    assert "ROS2SimDriverModule" in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules
    assert "ROS2PathBridgeModule" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SlamBridgeModule" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->NavigationModule.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->NavigationModule.patrol_goals" in wires
    assert "ROS2SimDriverModule.odometry->TAREExplorerModule.odometry" in wires
    assert "NavigationModule.mission_status->TAREExplorerModule.navigation_status" in wires
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "NavigationModule.global_path->ROS2PathBridgeModule.global_path" in wires
    assert "LocalPlannerModule.local_path->ROS2PathBridgeModule.local_path" in wires
    assert not graph.dangling_wires()


def test_sim_profiles_keep_autonomy_inside_module_graph():
    for profile in ("sim", "sim_gazebo", "sim_industrial", "sim_cmu_tare"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)

        bp = navigation(planner, tomogram, enable_native, **nav_config)
        entries = {entry.name: entry for entry in bp._entries}

        assert enable_native is False
        assert entries["TerrainModule"].config["backend"] == "nanobind"
        assert entries["LocalPlannerModule"].config["backend"] == "nanobind"
        assert entries["PathFollowerModule"].config["backend"] == "nav_core"
        if profile == "sim_cmu_tare":
            assert (
                entries["LocalPlannerModule"]
                .config["allow_direct_track_fallback"]
                is True
            )
            assert (
                entries["LocalPlannerModule"]
                .config["ignore_near_field_stop"]
                is True
            )


def test_real_robot_profiles_do_not_auto_actuate_on_startup():
    for profile in ("map", "nav", "super_lio", "super_lio_relocation", "explore", "tare_explore"):
        config = resolve_profile_config(profile)

        assert config["robot"] == "thunder"
        assert config["auto_enable"] is False
        assert config["auto_standup"] is False


def test_navigation_profiles_use_localization_odometry_for_runtime_consumers():
    for profile in REAL_LOCALIZATION_PROFILES:
        source = "SlamBridgeModule"
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert source in graph.modules
        assert f"{source}.odometry->NavigationModule.odometry" in wires
        assert f"{source}.odometry->GatewayModule.odometry" in wires
        assert f"{source}.odometry->SafetyRingModule.odometry" in wires
        assert f"{source}.odometry->PathFollowerModule.odometry" in wires
        assert f"{source}.odometry->LocalPlannerModule.odometry" in wires
        assert f"{source}.map_cloud->OccupancyGridModule.map_cloud" in wires
        assert f"{source}.map_cloud->VoxelGridModule.map_cloud" in wires
        assert f"{source}.map_cloud->ElevationMapModule.map_cloud" in wires
        assert f"{source}.map_cloud->TerrainModule.map_cloud" in wires
        assert f"{source}.map_cloud->GatewayModule.map_cloud" in wires
        assert f"{source}.localization_status->NavigationModule.localization_status" in wires
        assert f"{source}.localization_status->SafetyRingModule.localization_status" in wires
        assert f"{source}.localization_status->GatewayModule.localization_status" in wires
        if "DepthVisualOdomModule" in graph.modules:
            assert f"{source}.localization_status->DepthVisualOdomModule.localization_status" in wires


def test_super_lio_profiles_wire_bridge_localization_status_to_gateway():
    for profile in ("super_lio", "super_lio_relocation"):
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert "SlamBridgeModule" in graph.modules
        assert "SlamBridgeModule.localization_status->GatewayModule.localization_status" in wires
        assert "SlamBridgeModule.map_frame_jump_event->NavigationModule.map_frame_jump_event" in wires
        assert "SlamBridgeModule.map_frame_jump_event->LocalPlannerModule.map_frame_jump_event" in wires
        assert "SlamBridgeModule.map_frame_jump_event->PathFollowerModule.map_frame_jump_event" in wires
        assert not graph.dangling_wires(), profile


def test_profile_groups_make_simulation_boundary_explicit():
    assert set(PROFILE_SNAPSHOT_TARGETS) == (
        (set(PRODUCT_PROFILES) - set(OPTIONAL_NATIVE_PRODUCT_PROFILES))
        | set(SIMULATION_PROFILES)
    )
    assert not set(PRODUCT_PROFILES) & set(SIMULATION_PROFILES)
    assert "tare_explore" in PRODUCT_PROFILES
    assert "tare_explore" in OPTIONAL_NATIVE_PRODUCT_PROFILES
    assert "sim_mujoco_live" in SIMULATION_PROFILES
    assert "sim_gazebo" in SIMULATION_PROFILES
    assert "sim_industrial" in SIMULATION_PROFILES
    assert "sim_cmu_tare" in SIMULATION_PROFILES


def test_only_sanctioned_external_simulator_profiles_are_first_class():
    external_profiles = {
        profile
        for profile, config in PROFILES.items()
        if config.get("_external_launcher")
    }

    assert external_profiles == {"sim_cmu_tare", "sim_mujoco_live"}


def test_product_profiles_do_not_enable_simulation_bypass_flags():
    forbidden_keys = {
        "allow_direct_goal_fallback",
        "direct_goal_fallback_on_planner_failure",
        "local_planner_allow_direct_track_fallback",
        "local_planner_ignore_near_field_stop",
        "safety_stop_wiring",
        "safety_cmd_vel_timeout_ms",
        "latch_stop_signal",
    }

    for profile in PRODUCT_PROFILES:
        config = resolve_profile_config(profile)
        assert forbidden_keys.isdisjoint(config), profile


def test_simulation_runtime_contracts_lock_simulator_boundary():
    gazebo = simulation_runtime_contract("gazebo_industrial")
    cmu_baseline = simulation_runtime_contract("cmu_unity_baseline")
    cmu = simulation_runtime_contract("cmu_unity_external")
    fastlio = simulation_runtime_contract("mujoco_fastlio2_live")

    assert gazebo.profile == "sim_industrial"
    assert gazebo.world == "sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf"
    assert gazebo.launch_script == "sim/scripts/launch_lingtu_gazebo_industrial_demo.sh"
    assert gazebo.rviz_config == "tests/planning/lingtu_industrial_demo.rviz"
    assert gazebo.adapter_script == "sim/engine/bridge/gazebo_runtime_adapter.py"
    assert gazebo.data_source_contract == "gazebo_industrial"
    assert gazebo.command_topic == DATA_SOURCE_CONTRACTS["gazebo_industrial"].command_sink
    assert gazebo.canonical_topics == CANONICAL_NAV_TOPICS
    assert gazebo.native_topics == DATA_SOURCE_CONTRACTS["gazebo_industrial"].source_outputs
    assert set(DATA_SOURCE_CONTRACTS["gazebo_industrial"].algorithm_entry_outputs) <= set(
        gazebo.required_runtime_topics
    )
    assert gazebo.required_path_topics == (TOPICS.global_path, TOPICS.local_path)
    assert gazebo.required_map_growth_topics == (TOPICS.exploration_grid,)
    assert gazebo.required_scan_topics == (TOPICS.registered_cloud,)
    assert gazebo.simulation_only is True
    assert gazebo.isolated_domain_required is True
    assert "global_planning" in gazebo.lingtu_owns
    assert "local_planning" in gazebo.lingtu_owns
    assert "path_following" in gazebo.lingtu_owns
    assert "sensor_rendering" in gazebo.simulator_owns
    assert "global_planning" not in gazebo.simulator_owns
    assert gazebo.contract_role == "lingtu_full_stack_delivery_demo"
    assert gazebo.runtime_stage == "no_saved_map_live_mapping_smoke"
    assert gazebo.map_dependency == "gazebo_live_lidar_occupancy"
    assert gazebo.world_sensor_owner == "gazebo"
    assert gazebo.slam_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].slam_source
    assert gazebo.localization_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].localization_source
    assert gazebo.mapping_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].mapping_source
    assert gazebo.slam_validated is False
    assert gazebo.requires_live_slam is False
    assert gazebo.requires_saved_map is False
    assert gazebo.requires_tomogram is False
    assert gazebo.required_slam_topics == ()
    assert gazebo.cmd_vel_owner == "lingtu_cmd_vel_mux_to_gazebo_adapter"
    assert "real_robot_readiness" in gazebo.forbidden_claims
    assert "lingtu_fastlio_mapping_validated" in gazebo.forbidden_claims

    assert cmu_baseline.profile is None
    assert cmu_baseline.contract_role == "baseline_reference"
    assert cmu_baseline.runtime_stage == "external_baseline_no_lingtu_claim"
    assert cmu_baseline.map_dependency == "cmu_unity_live_registered_scan"
    assert cmu_baseline.launch_script.startswith("external:")
    assert cmu_baseline.adapter_script is None
    assert cmu_baseline.data_source_contract == "cmu_unity_external"
    assert cmu_baseline.command_topic == "/cmd_vel"
    assert cmu_baseline.exploration_owner == "cmu_tare_far"
    assert cmu_baseline.local_planning_owner == "cmu_local_planner"
    assert cmu_baseline.path_following_owner == "cmu_path_follower"
    assert cmu_baseline.cmd_vel_owner == "cmu_path_follower_to_vehicle_simulator"
    assert cmu_baseline.slam_source == "none"
    assert cmu_baseline.localization_source == "cmu_unity_state_estimation"
    assert cmu_baseline.mapping_source == "cmu_unity_registered_scan_and_terrain_map_ext"
    assert cmu_baseline.slam_validated is False
    assert cmu_baseline.requires_saved_map is False
    assert "/path" in cmu_baseline.required_path_topics
    assert "cmu_local_planning" in cmu_baseline.simulator_owns
    assert "lingtu_planning_validated" in cmu_baseline.forbidden_claims
    assert "lingtu_slam_localization_validated" in cmu_baseline.forbidden_claims

    assert cmu.profile == "sim_cmu_tare"
    assert cmu.adapter_script == "sim/engine/bridge/cmu_unity_lingtu_adapter.py"
    assert cmu.rviz_config == "tests/planning/cmu_unity_lingtu_runtime.rviz"
    assert cmu.data_source_contract == "cmu_unity_external"
    assert cmu.command_topic == DATA_SOURCE_CONTRACTS["cmu_unity_external"].command_sink
    assert set(DATA_SOURCE_CONTRACTS["cmu_unity_external"].source_outputs) <= set(cmu.native_topics)
    assert TOPICS.exploration_way_point in cmu.required_runtime_topics
    assert TOPICS.map_cloud in cmu.required_runtime_topics
    assert set(DATA_SOURCE_CONTRACTS["cmu_unity_external"].algorithm_entry_outputs) <= set(
        cmu.required_runtime_topics
    )
    assert cmu.required_path_topics == (TOPICS.global_path, TOPICS.local_path)
    assert cmu.required_map_growth_topics == (TOPICS.map_cloud, TOPICS.terrain_map_ext)
    assert cmu.required_scan_topics == ("/registered_scan", TOPICS.registered_cloud)
    assert "external_tare_runtime" in cmu.simulator_owns
    assert "tare_waypoint_supervision" in cmu.lingtu_owns
    assert cmu.contract_role == "lingtu_tare_adapter_execution_gate"
    assert cmu.runtime_stage == "external_live_map_execution"
    assert cmu.map_dependency == "cmu_unity_live_registered_scan_or_same_source_tomogram"
    assert cmu.world_sensor_owner == "cmu_unity"
    assert cmu.slam_source == DATA_SOURCE_CONTRACTS["cmu_unity_external"].slam_source
    assert cmu.localization_source == DATA_SOURCE_CONTRACTS["cmu_unity_external"].localization_source
    assert cmu.mapping_source == DATA_SOURCE_CONTRACTS["cmu_unity_external"].mapping_source
    assert cmu.slam_validated is False
    assert cmu.requires_live_slam is False
    assert cmu.requires_saved_map is False
    assert cmu.required_slam_topics == ()
    assert cmu.exploration_owner == "cmu_tare_external"
    assert cmu.global_planning_owner == "lingtu_navigation_optional_pct"
    assert cmu.local_planning_owner == "lingtu_navigation"
    assert cmu.path_following_owner == "lingtu_path_follower"
    assert cmu.cmd_vel_owner == "lingtu_adapter_relay_to_cmu_vehicle_simulator"
    assert "external_tare_waypoints_ingested" in cmu.validated_claims
    assert "pct_is_tare_executor" in cmu.forbidden_claims
    assert "pure_lingtu_exploration" in cmu.forbidden_claims
    assert "true_lingtu_mapping_closure" in cmu.forbidden_claims

    assert fastlio.provider == "mujoco"
    assert fastlio.profile == "sim_mujoco_live"
    assert fastlio.world == "sim/worlds/mujoco/industrial_park_scene.xml"
    assert fastlio.launch_script == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert fastlio.data_source_contract == "mujoco_fastlio2_live"
    assert set(fastlio.canonical_topics) == set(
        DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].source_outputs
    ) | set(DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].algorithm_entry_outputs)
    assert "/Odometry" in fastlio.native_topics
    assert "/cloud_map" in fastlio.native_topics
    assert "/Odometry" not in fastlio.canonical_topics
    assert "/cloud_map" not in fastlio.canonical_topics
    assert fastlio.runtime_stage == "no_saved_map_live_slam_optional_exploration"
    assert fastlio.map_dependency == "none_raw_lidar_imu"
    assert fastlio.required_slam_topics == (
        *DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].source_outputs,
        "/Odometry",
        "/cloud_map",
    )
    assert fastlio.slam_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].slam_source
    assert fastlio.localization_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].localization_source
    assert fastlio.mapping_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].mapping_source
    assert fastlio.slam_validated is True
    assert fastlio.requires_live_slam is True
    assert fastlio.requires_saved_map is False
    assert fastlio.requires_tomogram is False
    assert fastlio.exploration_owner == "lingtu_frontier_optional"
    assert fastlio.cmd_vel_owner == "mujoco_velocity_adapter_or_fixed_gate_motion"
    assert "fastlio2_lidar_imu_mapping_localization" in fastlio.validated_claims
    assert "canonical_nav_output_relay" in fastlio.validated_claims
    assert "lingtu_navigation_validated" not in fastlio.forbidden_claims

    assert runtime_contracts_for_profile("sim_industrial") == (gazebo,)
    assert runtime_contracts_for_profile("sim_cmu_tare") == (cmu,)
    assert runtime_contracts_for_profile("sim_mujoco_live") == (fastlio,)
    assert "gazebo_industrial" in SIMULATION_RUNTIME_CONTRACTS
    assert "cmu_unity_baseline" in SIMULATION_RUNTIME_CONTRACTS
    assert "mujoco_fastlio2_live" in SIMULATION_RUNTIME_CONTRACTS
    assert "rosbag_fastlio2_replay" in SIMULATION_RUNTIME_CONTRACTS

    for profile in (*SIMULATION_PROFILES, *PRODUCT_PROFILES):
        binding = profile_data_source(profile)
        assert binding.profile == profile
        assert binding.data_source in DATA_SOURCE_CONTRACTS

    assert profile_data_source("sim_industrial").data_source == "gazebo_industrial"
    assert profile_data_source("sim_cmu_tare").data_source == "cmu_unity_external"
    assert profile_data_source("sim_mujoco_live").data_source == "mujoco_fastlio2_live"
    assert profile_data_source("sim").data_source == "mujoco_module_graph"
    assert profile_data_source("map").data_source == "real_s100p"

    replay = simulation_runtime_contract("rosbag_fastlio2_replay")
    assert replay.command_topic == "no_actuation_replay_sink"
    assert replay.data_source_contract == "rosbag_fastlio2_replay"
    assert replay.simulation_only is True
    assert replay.cmd_vel_owner == "lingtu_cmd_vel_mux_to_no_actuation_sink"


def test_lingtu_simulation_runtime_contracts_mirror_data_source_boundaries():
    for contract in SIMULATION_RUNTIME_CONTRACTS.values():
        if contract.contract_role == "baseline_reference":
            continue
        source = DATA_SOURCE_CONTRACTS[contract.data_source_contract]
        required_runtime = set(contract.required_runtime_topics)

        assert set(source.algorithm_entry_outputs) <= required_runtime, contract.name
        assert set(source.algorithm_context_outputs) <= required_runtime, contract.name
        assert set(source.source_outputs) <= set(contract.native_topics), contract.name
        assert contract.slam_source == source.slam_source, contract.name
        assert contract.localization_source == source.localization_source, contract.name
        assert contract.mapping_source == source.mapping_source, contract.name
        if source.command_sink.startswith("/"):
            assert contract.command_topic == source.command_sink, contract.name


def test_real_robot_profiles_plan_in_client_map_frame():
    profiles = (
        "map",
        "nav",
        "super_lio",
        "super_lio_relocation",
        "explore",
        "tare_explore",
    )
    for profile in profiles:
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)
        bp = navigation(
            planner,
            tomogram,
            enable_native,
            **nav_config,
        )
        nav_entry = next(entry for entry in bp._entries if entry.name == "NavigationModule")

        assert nav_entry.config["planning_frame_id"] == "map"


def test_navigation_plan_safety_policy_is_profile_visible():
    sim_config = resolve_profile_config("sim")
    sim_nav_config = dict(sim_config)
    sim_planner = sim_nav_config.pop("planner", "astar")
    sim_tomogram = sim_nav_config.pop("tomogram", "")
    sim_enable_native = sim_nav_config.pop("enable_native", False)
    sim_bp = navigation(sim_planner, sim_tomogram, sim_enable_native, **sim_nav_config)
    sim_entry = next(entry for entry in sim_bp._entries if entry.name == "NavigationModule")

    assert sim_entry.config["plan_safety_policy"] == "fallback_astar"
    assert sim_entry.config["fallback_planner_name"] == "astar"

    for profile in ("nav", "explore", "tare_explore", "super_lio", "super_lio_relocation"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)
        bp = navigation(planner, tomogram, enable_native, **nav_config)
        nav_entry = next(entry for entry in bp._entries if entry.name == "NavigationModule")

        assert nav_entry.config["plan_safety_policy"] == "fallback_astar"
        assert nav_entry.config["fallback_planner_name"] == "astar"


def test_navigation_forwards_frontier_reachability_tuning():
    bp = navigation(
        "astar",
        "",
        False,
        enable_frontier=True,
        frontier_approach_max_target_distance_m=2.4,
        frontier_approach_goal_max_distance_m=6.0,
        frontier_reachable_goal_radius=1.6,
    )

    frontier_entry = next(
        entry for entry in bp._entries if entry.name == "WavefrontFrontierExplorer"
    )

    assert frontier_entry.config["approach_max_target_distance_m"] == 2.4
    assert frontier_entry.config["approach_goal_max_distance_m"] == 6.0
    assert frontier_entry.config["reachable_goal_radius"] == 1.6


def test_navigation_forwards_path_follower_drive_mode():
    bp = navigation(
        "astar",
        "",
        False,
        path_follower_two_way_drive=False,
    )

    follower_entry = next(
        entry for entry in bp._entries if entry.name == "PathFollowerModule"
    )

    assert follower_entry.config["two_way_drive"] is False


def test_navigation_forwards_independent_autonomy_backends():
    bp = navigation(
        "astar",
        "",
        False,
        terrain_backend="simple",
        local_planner_backend="cmu_py",
        path_follower_backend="pid",
    )

    terrain_entry = next(entry for entry in bp._entries if entry.name == "TerrainModule")
    local_planner_entry = next(
        entry for entry in bp._entries if entry.name == "LocalPlannerModule"
    )
    follower_entry = next(
        entry for entry in bp._entries if entry.name == "PathFollowerModule"
    )

    assert terrain_entry.config["backend"] == "simple"
    assert local_planner_entry.config["backend"] == "cmu_py"
    assert follower_entry.config["backend"] == "pid"
