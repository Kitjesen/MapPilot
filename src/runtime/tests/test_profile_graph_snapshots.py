import builtins
import sys
import types

import pytest

pytestmark = [pytest.mark.sim]

from runtime.blueprints.profile_graph import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    PRODUCT_PROFILES,
    SIMULATION_PROFILES,
    blueprint_for_profile,
    graph_for_profile,
    resolve_profile_config,
)
from runtime.profiles.endpoints import (
    RuntimeEndpointError,
    runtime_endpoint,
    runtime_endpoint_names,
    resolve_runtime_run_spec,
)
from runtime.profiles.binding_policy import resolved_autonomy_backend_selection
from runtime.blueprints.simulation_contract import (
    CANONICAL_NAV_TOPICS,
    SIMULATION_RUNTIME_CONTRACTS,
    runtime_contracts_for_profile,
    simulation_runtime_contract,
)
from runtime.blueprints.stacks.navigation import (
    autonomy_stack_config,
    frontier_module_config,
    navigation_config,
)
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    RUNTIME_DATA_FLOW,
    TOPICS,
    profile_data_source,
)
from runtime.profiles.product_mode_contracts import (
    PRODUCT_MODE_CONTRACTS,
    product_mode_switch_plan,
)
from runtime.runtime_profiles import PROFILES, ROBOT_PRESETS


REAL_LOCALIZATION_PROFILES = (
    "teleop_avoid",
    "map",
    "tracking",
    "nav",
    "inspection",
    "explore",
    "super_lio",
    "super_lio_relocation",
)

ENDPOINT_SLAM_MODULE = "SlamAdapterModule"

SIMULATION_PROFILE_RUNTIME_MATRIX = {
    "stub": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "dev": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_nav": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim": {
        "data_source": "mujoco_module_graph",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "portable_mujoco": {
        "data_source": "mujoco_module_graph",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_mujoco_live": {
        "data_source": "mujoco_fastlio2_live",
        "profile_contracts": ("mujoco_fastlio2_live",),
        "external_launcher": "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "runtime_contract": "mujoco_fastlio2_live",
    },
    "sim_mujoco_octo_live": {
        "data_source": "mujoco_fastlio2_live",
        "profile_contracts": ("mujoco_fastlio2_live",),
        "external_launcher": "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "runtime_contract": "mujoco_fastlio2_live",
    },
    "sim_mujoco_pct_live": {
        "data_source": "mujoco_fastlio2_live",
        "profile_contracts": ("mujoco_fastlio2_live",),
        "external_launcher": "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "runtime_contract": "mujoco_fastlio2_live",
    },
    "sim_gazebo": {
        "data_source": "gazebo_industrial",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_industrial": {
        "data_source": "gazebo_industrial",
        "profile_contracts": ("gazebo_industrial",),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_cmu_tare": {
        "data_source": "cmu_unity_external",
        "profile_contracts": ("cmu_unity_external",),
        "external_launcher": "sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        "runtime_contract": "cmu_unity_external",
    },
}


def test_six_product_modes_are_locked_to_profiles():
    expected = {
        "teleop": "teleop",
        "teleop_avoid": "teleop_avoid",
        "map": "mapping",
        "tracking": "tracking",
        "nav": "navigation",
        "inspection": "inspection",
    }

    for profile, mode in expected.items():
        config = resolve_profile_config(profile)
        assert config["product_mode"] == mode
        assert profile_data_source(profile).data_source in DATA_SOURCE_CONTRACTS


def test_runtime_data_flow_stages_define_operator_relevant_contract():
    for stage in RUNTIME_DATA_FLOW:
        assert stage.producer, stage.name
        assert stage.consumers, stage.name
        assert stage.frequency, stage.name
        assert stage.transport_policy, stage.name


def test_product_modes_start_and_forbid_expected_module_groups():
    for profile, contract in PRODUCT_MODE_CONTRACTS.items():
        modules = set(graph_for_profile(profile).modules)
        assert contract.required_modules <= modules, profile
        assert contract.forbidden_modules.isdisjoint(modules), profile


def test_product_modes_required_wires_are_contract_locked():
    for profile, contract in PRODUCT_MODE_CONTRACTS.items():
        wires = _wire_set(graph_for_profile(profile))
        assert contract.required_wires <= wires, profile


def test_product_mode_switch_contracts_do_not_claim_online_hot_switch():
    plan = product_mode_switch_plan("tracking", "inspection")
    assert plan["same_graph_candidate"] is True
    assert plan["online_hot_switch_supported"] is False
    assert plan["required_lifecycle"] == "cold_restart"

    teleop_to_nav = product_mode_switch_plan("teleop", "nav")
    assert teleop_to_nav["same_graph_candidate"] is False
    assert teleop_to_nav["required_lifecycle"] == "cold_restart"


def test_top_level_blueprint_api_exposes_all_stack_factories():
    import runtime.blueprints as blueprints

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


def test_static_profile_graph_does_not_import_runtime_message_stack(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "numpy" or name.startswith("runtime.msgs"):
            raise AssertionError(f"static graph imported runtime message stack: {name}")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)

    graph = graph_for_profile("explore")

    assert "nav.mission" in graph.modules
    assert "nav.out" in graph.modules
    assert "ThunderDriver" not in graph.modules


def test_profile_graphs_compile_for_primary_profiles():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)
        config = resolve_profile_config(profile)

        contract = PRODUCT_MODE_CONTRACTS.get(profile)
        if contract is not None and "nav.mission" in contract.forbidden_modules:
            # teleop / teleop_avoid / map contracts forbid the autonomy stack.
            assert "nav.mission" not in graph.modules
        else:
            assert "nav.mission" in graph.modules
        assert "nav.safety" in graph.modules
        if config.get("enable_gateway", True):
            assert "GatewayModule" in graph.modules
            assert "MCPServerModule" in graph.modules
        else:
            assert "GatewayModule" not in graph.modules
            assert "MCPServerModule" not in graph.modules
        assert not graph.dangling_wires(), profile


def test_runtime_product_profile_uses_product_blueprint_entrypoint(monkeypatch):
    fake_full_stack = types.ModuleType("runtime.blueprints.full_stack")

    def fail_full_stack_blueprint(**kwargs):
        raise AssertionError("product profile should not use full_stack entrypoint")

    fake_full_stack.full_stack_blueprint = fail_full_stack_blueprint
    monkeypatch.setitem(sys.modules, "runtime.blueprints.full_stack", fake_full_stack)

    bp = blueprint_for_profile(
        "nav",
        run_startup_checks=False,
        manage_external_services=False,
    )
    modules = {entry.name for entry in bp._entries}
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert "nav.out" in modules
    assert "ThunderDriver" not in modules
    assert ENDPOINT_SLAM_MODULE in modules
    assert "SlamBridgeModule" not in modules
    assert "nav.safety.stop_cmd->nav.mission.stop_signal" in wires
    assert "nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel" in wires
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires


def test_simulation_profiles_match_runtime_data_source_matrix():
    assert set(SIMULATION_PROFILE_RUNTIME_MATRIX) == set(SIMULATION_PROFILES)

    for profile, expected in SIMULATION_PROFILE_RUNTIME_MATRIX.items():
        config = resolve_profile_config(profile)
        binding = profile_data_source(profile)
        source = DATA_SOURCE_CONTRACTS[binding.data_source]
        contracts = runtime_contracts_for_profile(profile)

        assert binding.data_source == expected["data_source"], profile
        assert source.provider != "hardware", profile
        assert tuple(contract.name for contract in contracts) == expected[
            "profile_contracts"
        ], profile
        assert PROFILES[profile].get("_external_launcher") == expected[
            "external_launcher"
        ], profile
        assert PROFILES[profile].get("_runtime_contract") == expected[
            "runtime_contract"
        ], profile
        assert "_external_launcher" not in config, profile
        assert "_runtime_contract" not in config, profile
        for contract in contracts:
            assert contract.data_source_contract == binding.data_source, profile
            assert contract.simulation_only is True, profile


def test_simulation_endpoints_generate_coherent_runtime_run_specs():
    for endpoint_name in (
        "mujoco_live",
        "replay",
        "gazebo",
        "cmu_unity",
    ):
        endpoint = runtime_endpoint(endpoint_name)
        source = DATA_SOURCE_CONTRACTS[endpoint.data_source]

        assert endpoint.simulation_only is True
        assert source.provider != "hardware"
        assert endpoint.runtime_contract

        for profile in endpoint.supported_profiles:
            config = resolve_profile_config(profile, runtime_endpoint=endpoint_name)
            spec = resolve_runtime_run_spec(profile, config)

            assert spec.endpoint == endpoint_name, profile
            assert spec.data_source == endpoint.data_source, profile
            assert spec.runtime_contract == endpoint.runtime_contract, profile
            assert spec.simulation_only is True, profile
            assert spec.command_sink == source.command_sink, profile
            if endpoint.external_launcher:
                assert spec.launcher == endpoint.external_launcher, profile
                assert spec.launcher_args == endpoint.default_actions[profile], profile
            else:
                assert spec.launcher is None, profile
                assert spec.launcher_args == (), profile
                assert spec.as_command() == [], profile
            assert spec.env["LINGTU_PROFILE"] == profile
            assert spec.env["LINGTU_ENDPOINT"] == endpoint_name
            assert spec.env["LINGTU_DATA_SOURCE"] == endpoint.data_source
            assert spec.env["LINGTU_MODULE_TRANSPORT"] == endpoint.module_transport
            assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == endpoint.endpoint_transport
            assert spec.env["LINGTU_RUNTIME_CONTRACT"] == endpoint.runtime_contract
            assert spec.env["LINGTU_COMMAND_SINK"] == source.command_sink
            assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"


def test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        driver = next(
            (
                module
                for module in modules
                if module.endswith("Driver")
                or module.endswith("DriverModule")
                or module.endswith("DogModule")
            ),
            None,
        )

        if "nav.mission" in modules:
            assert "nav.safety.stop_cmd->nav.mission.stop_signal" in wires
        if "GatewayModule" in modules:
            if "nav.mission" in modules:
                assert "GatewayModule.stop_cmd->nav.mission.stop_signal" in wires
            assert "GatewayModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" in wires
            if "nav.mission" in modules:
                assert "nav.mission.mission_status->GatewayModule.mission_status" in wires
            if "nav.goals" in modules and "nav.mission" in modules:
                assert (
                    "GatewayModule.goal_pose->nav.goals.goal_request"
                    in wires
                )
                assert (
                    "GatewayModule.cancel->nav.goals.cancel_request"
                    in wires
                )
                assert (
                    "nav.goals.goal_pose->nav.mission.goal_pose"
                    in wires
                )
                assert (
                    "nav.goals.cancel->nav.mission.cancel"
                    in wires
                )
            elif "nav.mission" in modules:
                assert "GatewayModule.cancel->nav.mission.cancel" in wires
        if "MCPServerModule" in modules:
            if "nav.mission" in modules:
                assert "MCPServerModule.stop_cmd->nav.mission.stop_signal" in wires
            assert "MCPServerModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" in wires
            if "nav.mission" in modules:
                assert "nav.mission.mission_status->MCPServerModule.mission_status" in wires
            if "nav.goals" in modules and "nav.mission" in modules:
                assert (
                    "MCPServerModule.goal_pose->nav.goals.goal_request"
                    in wires
                )
        if "GeofenceManagerModule" in modules and "nav.mission" in modules:
            assert "GeofenceManagerModule.stop_cmd->nav.mission.stop_signal" in wires
        if driver is not None:
            assert f"nav.safety.stop_cmd->{driver}.stop_signal" in wires
            if "GatewayModule" in modules:
                assert f"GatewayModule.stop_cmd->{driver}.stop_signal" in wires
            if "MCPServerModule" in modules:
                assert f"MCPServerModule.stop_cmd->{driver}.stop_signal" in wires
            if "GeofenceManagerModule" in modules:
                assert f"GeofenceManagerModule.stop_cmd->{driver}.stop_signal" in wires
            assert f"nav.velocity_mux.driver_cmd_vel->{driver}.cmd_vel" in wires
        else:
            assert "nav.out" in modules
            assert (
                f"nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel@{TOPICS.cmd_vel}"
                in wires
            )
        if {"nav.mission", "nav.local_planner"} <= modules:
            assert "nav.mission.clear_path->nav.local_planner.clear_path" in wires
            assert "nav.mission.global_path->nav.local_planner.global_path" in wires
        if {"nav.local_planner", "nav.path_follower"} <= modules:
            assert "nav.local_planner.control_hint->nav.path_follower.control_hint" in wires


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
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        if {"OccupancyGridModule", "TraversabilityCostModule"} <= modules:
            assert "OccupancyGridModule.costmap->TraversabilityCostModule.costmap" in wires
        if {"ESDFModule", "TraversabilityCostModule"} <= modules:
            assert "ESDFModule.esdf->TraversabilityCostModule.esdf" in wires
        if {"TraversabilityCostModule", "nav.mission"} <= modules:
            assert "TraversabilityCostModule.fused_cost->nav.mission.costmap" in wires
        if {"nav.terrain", "nav.mission"} <= modules:
            assert "nav.terrain.traversability->nav.mission.traversability" in wires
        if {"TraversabilityCostModule", "GatewayModule"} <= modules:
            assert "TraversabilityCostModule.fused_cost->GatewayModule.costmap" in wires
        if {"SemanticMapperModule", "SemanticPlannerModule"} <= modules:
            assert (
                "SemanticMapperModule.topo_summary->SemanticPlannerModule.topo_summary"
                in wires
            )
            assert "SemanticMapperModule.room_graph->SemanticPlannerModule.room_graph" in wires


# Profiles that assemble the full map + autonomy chain (occupancy/ESDF ->
# traversability -> nav, plus terrain -> local planner -> path follower).
_NAV_CONTRACT_PROFILES = (
    "stub",
    "dev",
    "sim",
    "portable_mujoco",
    "sim_gazebo",
    "sim_industrial",
    "tracking",
    "nav",
    "inspection",
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
            "TraversabilityCostModule.fused_cost->nav.mission.costmap" in wires
        ), profile
        # L2 local planning: terrain_map (local geometry) is the main local input.
        assert (
            "nav.terrain.terrain_map->nav.local_planner.terrain_map" in wires
        ), profile
        assert (
            "nav.terrain.traversability->nav.local_planner.traversability" in wires
        ), profile
        # L5 -> L2 command dispatch (global_path + waypoint as staged goals).
        assert (
            "nav.mission.global_path->nav.local_planner.global_path" in wires
        ), profile
        assert (
            "nav.mission.waypoint->nav.local_planner.waypoint" in wires
        ), profile
        # L2 control tracking: local_path -> path follower -> cmd_vel.
        assert (
            "nav.local_planner.local_path->nav.path_follower.local_path" in wires
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
                "TraversabilityCostModule.fused_cost->LocalPlanner"
            ), f"{profile}: costmap must not be a local-planner input ({wire})"
            assert not wire.startswith(
                "nav.terrain.terrain_map->Navigation"
            ), f"{profile}: terrain_map must not drive global planning ({wire})"


def test_nav_profile_uses_endpoint_slam_localization_health_edges():
    wires = _wire_set(graph_for_profile("nav"))

    health = TOPICS.localization_health
    quality = TOPICS.localization_quality

    assert f"{ENDPOINT_SLAM_MODULE}.localization_status->nav.safety.localization_status@{health}" in wires
    assert f"{ENDPOINT_SLAM_MODULE}.localization_status->nav.mission.localization_status@{health}" in wires
    assert (
        f"{ENDPOINT_SLAM_MODULE}.localization_status->DepthVisualOdomModule.localization_status"
        f"@{health}"
        in wires
    )
    assert f"{ENDPOINT_SLAM_MODULE}.localization_status->GatewayModule.localization_status@{health}" in wires
    assert f"{ENDPOINT_SLAM_MODULE}.localization_quality->GatewayModule.localization_quality@{quality}" in wires
    assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.mission.map_frame_jump_event" in wires
    assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.local_planner.map_frame_jump_event" in wires
    assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.path_follower.map_frame_jump_event" in wires


def test_thunder_field_profiles_use_endpoint_only_command_boundary_by_default():
    for profile in (
        "map",
        "nav",
        "explore",
        "tare_explore",
        "super_lio",
        "super_lio_relocation",
    ):
        config = resolve_profile_config(profile)
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert config["_runtime_endpoint"] == "thunder_field"
        assert config["_endpoint_transport"] == "dds"
        assert config["localization_adapter"] == "dds_endpoint"
        assert config["enable_device_manager"] is False
        assert config["enable_robot_driver"] is False
        assert config["enable_lidar"] is False
        assert "nav.out" in graph.modules
        assert "DeviceManager" not in graph.modules
        assert "ThunderDriver" not in graph.modules
        assert "LidarModule" not in graph.modules
        assert "GnssBridgeModule" not in graph.modules
        assert "ROS2SimDriverModule" not in graph.modules
        assert ENDPOINT_SLAM_MODULE in graph.modules
        assert "SlamBridgeModule" not in graph.modules
        assert (
            f"nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel@{TOPICS.cmd_vel}"
            in wires
        )
        assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires

        has_mission_stack = "nav.mission" in graph.modules
        if has_mission_stack:
            assert (
                f"nav.mission.global_path->nav.out.global_path@{TOPICS.global_path}"
                in wires
            )
            assert (
                f"nav.local_planner.local_path->nav.out.local_path@{TOPICS.local_path}"
                in wires
            )
            assert (
                f"nav.mission.waypoint->nav.out.waypoint@{TOPICS.nav_way_point}"
                in wires
            )
            assert f"{ENDPOINT_SLAM_MODULE}.odometry->nav.mission.odometry@{TOPICS.odometry}" in wires
            assert (
                f"{ENDPOINT_SLAM_MODULE}.localization_status->nav.mission.localization_status"
                f"@{TOPICS.localization_health}"
                in wires
            )
        else:
            # "map" runs mapping-only: its contract forbids nav.mission /
            # nav.local_planner, so there is no global_path/waypoint/local_path
            # to forward through nav.out.
            assert "nav.local_planner" not in graph.modules


def test_thunder_field_semantic_profiles_use_orbbec_camera_bridge():
    for profile in (
        "nav",
        "explore",
        "tare_explore",
        "super_lio",
        "super_lio_relocation",
    ):
        config = resolve_profile_config(profile)
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert config["_runtime_endpoint"] == "thunder_field"
        assert config["camera_backend"] == "orbbec_native"
        assert config["enable_camera"] is True
        assert "CameraBridgeModule" in graph.modules
        assert "CameraBridgeModule.color_image->PerceptionModule.color_image" in wires
        assert "CameraBridgeModule.depth_image->PerceptionModule.depth_image" in wires
        assert "CameraBridgeModule.camera_info->PerceptionModule.camera_info" in wires


@pytest.mark.sim
def test_sim_gazebo_profile_uses_endpoint_driver_and_map_planning_frame():
    graph = graph_for_profile("sim_gazebo")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_gazebo")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert config["planning_frame_id"] == "map"
    assert config["enable_native"] is False
    assert config["latch_stop_signal"] is False
    assert config["python_autonomy_backend"] == "nanobind"
    assert config["python_path_follower_backend"] == "nav_kernel"
    assert config["enable_camera"] is True
    assert config["use_driver_camera"] is True
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "CameraBridgeModule" not in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "SimEndpointDriverModule.map_cloud->nav.terrain.map_cloud" in wires
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert ENDPOINT_SLAM_MODULE not in graph.modules
    assert "SlamBridgeModule" not in graph.modules
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_sim_industrial_profile_is_lingtu_owned_exploration_profile():
    graph = graph_for_profile("sim_industrial")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_industrial")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["latch_stop_signal"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "WavefrontFrontierExplorer.exploration_goal->nav.mission.goal_pose" in wires
    assert "nav.mission.mission_status->WavefrontFrontierExplorer.navigation_status" in wires
    assert "SimEndpointDriverModule.odometry->WavefrontFrontierExplorer.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "nav.mission.global_path->nav.local_planner.global_path" in wires
    assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_portable_mujoco_profile_is_no_ros_planning_sensor_entry():
    graph = graph_for_profile("portable_mujoco")
    wires = _wire_set(graph)
    config = resolve_profile_config("portable_mujoco")
    source = DATA_SOURCE_CONTRACTS[profile_data_source("portable_mujoco").data_source]

    assert config["robot"] == "sim_mujoco"
    assert config["world"] == "open_field"
    assert config["drive_mode"] == "kinematic"
    assert config["slam_profile"] == "none"
    assert config["planner"] == "octoplanner3d"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_gateway"] is False
    assert config["enable_camera"] is True
    assert config["use_driver_camera"] is True
    assert config["planner_profile"]["latency_budget_ms"] == 250
    assert source.name == "mujoco_module_graph"
    assert source.provider == "mujoco"
    assert TOPICS.camera_color in source.normalized_outputs
    assert TOPICS.camera_depth in source.normalized_outputs
    assert TOPICS.camera_info in source.normalized_outputs
    assert TOPICS.height_rays in source.normalized_outputs
    assert source.algorithm_context_outputs == (TOPICS.height_rays,)
    assert "fixed_terrain_height_rays" in source.owns
    assert "MujocoDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "GatewayModule" not in graph.modules
    assert "MCPServerModule" not in graph.modules
    assert "CameraBridgeModule" not in graph.modules
    assert "MujocoDriverModule.odometry->nav.mission.odometry" in wires
    assert "MujocoDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "nav.mission.global_path->nav.local_planner.global_path" in wires
    assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires
    assert "nav.velocity_mux.driver_cmd_vel->MujocoDriverModule.cmd_vel" in wires
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_sim_mujoco_live_profile_is_raw_fastlio_simulation_entry():
    graph = graph_for_profile("sim_mujoco_live")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_mujoco_live")

    assert config["robot"] == "sim_endpoint"
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
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "SimEndpointDriverModule.map_cloud->nav.terrain.map_cloud" in wires
    assert "WavefrontFrontierExplorer.exploration_goal->nav.mission.goal_pose" in wires
    assert "nav.mission.mission_status->WavefrontFrontierExplorer.navigation_status" in wires
    assert "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers" in wires
    assert "TraversableFrontierModule.frontier_candidate->GatewayModule.frontier_candidate" in wires
    assert "TraversableFrontierModule.frontier_candidate->nav.mission.goal_pose" not in wires
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_sim_mujoco_octo_live_profile_is_octoplanner3d_closed_loop_entry():
    graph = graph_for_profile("sim_mujoco_octo_live")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_mujoco_octo_live")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert PROFILES["sim_mujoco_octo_live"]["_external_launcher"] == (
        "sim/scripts/launch_mujoco_fastlio2_live.sh"
    )
    assert PROFILES["sim_mujoco_octo_live"]["_runtime_contract"] == "mujoco_fastlio2_live"
    assert config["planner"] == "octoplanner3d"
    assert config["planner_backend"] == "octoplanner3d"
    assert config["tomogram"].endswith(".bt")
    assert config["plan_safety_policy"] == "reject"
    assert config["fallback_planner_name"] == ""
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is False
    assert config["enable_traversable_frontier"] is False
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["local_planner_allow_direct_track_fallback"] is False
    assert config["local_planner_ignore_near_field_stop"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "SimEndpointDriverModule.map_cloud->nav.terrain.map_cloud" in wires
    assert "nav.mission.global_path->nav.local_planner.global_path" in wires
    assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires
    assert not graph.dangling_wires()


def test_product_explore_can_run_on_mujoco_live_endpoint():
    endpoint = runtime_endpoint("mujoco_live")
    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")
    graph = graph_for_profile("explore", runtime_endpoint="mujoco_live")
    wires = _wire_set(graph)

    assert endpoint.data_source == "mujoco_fastlio2_live"
    assert endpoint.robot_preset == "sim_endpoint"
    assert "explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_endpoint"
    assert config["_runtime_endpoint"] == "mujoco_live"
    assert config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert config["_external_launcher"] == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert config["_external_default_args"] == ("explore",)
    assert config["_external_record_args"] == ("video",)
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "WavefrontFrontierExplorer.exploration_goal->nav.mission.goal_pose" in wires
    assert "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers" in wires
    assert "TraversableFrontierModule.frontier_candidate->GatewayModule.frontier_candidate" in wires
    assert "TraversableFrontierModule.frontier_candidate->nav.mission.goal_pose" not in wires
    assert not graph.dangling_wires()


def test_runtime_run_spec_carries_endpoint_command_and_safety_boundary():
    from runtime.profiles.endpoints import resolve_runtime_run_spec

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
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert spec.env["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert spec.env["LINGTU_COMMAND_SINK"] == "mujoco_velocity_adapter"
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"
    assert spec.as_command() == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "video",
    ]


def test_replay_endpoint_is_no_actuation_runtime():
    from runtime.profiles.endpoints import runtime_endpoint, resolve_runtime_run_spec

    endpoint = runtime_endpoint("replay")
    config = resolve_profile_config("nav", runtime_endpoint="replay")
    spec = resolve_runtime_run_spec("nav", config, extra_args=("status",))

    assert endpoint.simulation_only is True
    assert endpoint.data_source == "rosbag_fastlio2_replay"
    assert endpoint.robot_preset == "sim_endpoint"
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
    from runtime.profiles.endpoints import resolve_runtime_run_spec

    config = resolve_profile_config("nav")
    spec = resolve_runtime_run_spec("nav", config)

    assert spec.endpoint == "thunder_field"
    assert spec.data_source == "thunder_field"
    assert spec.runtime_contract == "thunder_field"
    assert spec.simulation_only is False
    assert spec.command_sink == "hardware_driver_after_cmd_vel_mux"
    assert spec.slam_source == "lingtu_fastlio_or_external_robot_slam"
    assert spec.localization_source == "slam_localizer"
    assert spec.mapping_source == "slam_map_cloud"
    assert spec.lidar_extrinsic_profile == "real_mid360"
    assert spec.launcher is None
    assert spec.launcher_args == ()
    assert spec.env["LINGTU_ENDPOINT"] == "thunder_field"
    assert spec.env["LINGTU_DATA_SOURCE"] == "thunder_field"
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "dds"
    assert spec.env["LINGTU_RUNTIME_CONTRACT"] == "thunder_field"
    assert spec.env["LINGTU_COMMAND_SINK"] == "hardware_driver_after_cmd_vel_mux"
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "0"


def test_product_tare_can_run_on_mujoco_live_endpoint():
    endpoint = runtime_endpoint("mujoco_live")
    config = resolve_profile_config("tare_explore", runtime_endpoint="mujoco_live")

    assert endpoint.data_source == "mujoco_fastlio2_live"
    assert endpoint.robot_preset == "sim_endpoint"
    assert "tare_explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_endpoint"
    assert config["_runtime_endpoint"] == "mujoco_live"
    assert config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert config["_external_launcher"] == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert config["_external_default_args"] == ("tare",)
    assert config["_external_record_args"] == ("tare-video",)
    assert config["planner"] == "octoplanner3d"
    assert "planner_backend" not in config
    assert config["planning_frame_id"] == "odom"
    assert config["goal_frame_id"] == "odom"
    assert config["exploration_backend"] == "tare"
    assert config["enable_map_out"] is False
    assert config["enable_nav_out"] is False
    assert config["enable_frontier"] is False
    assert config["hold_active_goal_until_terminal"] is True


def test_product_tare_can_run_on_cmu_unity_endpoint():
    endpoint = runtime_endpoint("cmu_unity")
    config = resolve_profile_config("tare_explore", runtime_endpoint="cmu_unity")
    graph = graph_for_profile("tare_explore", runtime_endpoint="cmu_unity")
    wires = _wire_set(graph)

    assert endpoint.data_source == "cmu_unity_external"
    assert endpoint.robot_preset == "sim_endpoint"
    assert "tare_explore" in endpoint.supported_profiles
    assert config["robot"] == "sim_endpoint"
    assert config["_runtime_endpoint"] == "cmu_unity"
    assert config["_endpoint_data_source"] == "cmu_unity_external"
    assert config["_external_launcher"] == "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    assert config["_external_default_args"] == ("gate",)
    assert config["_external_record_args"] == ("start", "--gate", "--rviz")
    assert config["planner"] == "octoplanner3d"
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_nav_out"] is False
    assert "enable_ros2_bridge" not in config
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "nav.out" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->nav.mission.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->nav.mission.patrol_goals" in wires
    assert "SimEndpointDriverModule.odometry->TAREExplorerModule.odometry" in wires
    assert not graph.dangling_wires()


def test_endpoint_rejects_unsupported_task_pairings():
    with pytest.raises(RuntimeEndpointError):
        resolve_profile_config("nav", runtime_endpoint="cmu_unity")


def test_sim_cmu_tare_profile_is_external_tare_simulation_entry():
    graph = graph_for_profile("sim_cmu_tare")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_cmu_tare")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert PROFILES["sim_cmu_tare"]["_external_launcher"] == (
        "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    )
    assert PROFILES["sim_cmu_tare"]["_runtime_contract"] == "cmu_unity_external"
    assert config["planner"] == "octoplanner3d"
    assert config["planning_frame_id"] == "map"
    assert config["enable_nav_out"] is False
    assert "enable_ros2_bridge" not in config
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["run_startup_checks"] is False
    assert config["manage_external_services"] is False
    assert config["python_autonomy_backend"] == "nanobind"
    assert config["python_path_follower_backend"] == "nav_kernel"
    assert config["local_planner_allow_direct_track_fallback"] is True
    assert config["local_planner_ignore_near_field_stop"] is True
    assert config["local_planner_direct_track_fallback_min_distance_m"] == pytest.approx(0.3)
    assert config["prefer_path_strategy"] is True
    assert config["external_strategy_path_control"] is False
    assert config["partial_goal_repeat_ignore_window_s"] == pytest.approx(5.0)
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules
    assert "nav.out" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert ENDPOINT_SLAM_MODULE not in graph.modules
    assert "SlamBridgeModule" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->nav.mission.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->nav.mission.patrol_goals" in wires
    assert "SimEndpointDriverModule.odometry->TAREExplorerModule.odometry" in wires
    assert "nav.mission.mission_status->TAREExplorerModule.navigation_status" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert not graph.dangling_wires()


def test_sim_profiles_keep_autonomy_inside_module_graph():
    for profile in ("sim", "sim_nav", "sim_gazebo", "sim_industrial", "sim_cmu_tare"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        enable_native = nav_config.pop("enable_native", False)
        nav_config.pop("planner", "octoplanner3d")
        nav_config.pop("tomogram", "")

        autonomy_config = autonomy_stack_config(enable_native, **nav_config)

        assert enable_native is False
        assert config["python_autonomy_backend"] == "nanobind"
        assert config["python_path_follower_backend"] == "nav_kernel"
        assert (autonomy_config["terrain_backend"] or autonomy_config["backend"]) == "nanobind"
        assert autonomy_config["backend"] == "nanobind"
        assert autonomy_config["path_follower_backend"] == "nav_kernel"
        if profile == "sim_cmu_tare":
            assert autonomy_config["local_planner_config"][
                "allow_direct_track_fallback"
            ] is True
            assert autonomy_config["local_planner_config"][
                "ignore_near_field_stop"
            ] is True


def test_real_robot_profiles_do_not_auto_actuate_on_startup():
    for profile in (
        "lite",
        "map",
        "nav",
        "super_lio",
        "super_lio_relocation",
        "explore",
        "tare_explore",
    ):
        config = resolve_profile_config(profile)

        assert config["robot"] == "thunder"
        assert config["auto_enable"] is False
        assert config["auto_standup"] is False


def test_lite_profile_graph_stays_lightweight_and_python_only():
    config = resolve_profile_config("lite")
    graph = graph_for_profile(
        "lite",
        mode="runtime",
        run_startup_checks=False,
        manage_external_services=False,
    )
    modules = set(graph.modules)

    assert {
        "ThunderDriver",
        "nav.mission",
        "nav.terrain",
        "nav.local_planner",
        "nav.path_follower",
        "nav.safety",
        "nav.velocity_mux",
    } <= modules
    assert {
        "DeviceManager",
        "LidarModule",
        "SLAMModule",
        "SlamAdapterModule",
        "SlamBridgeModule",
        "DepthVisualOdomModule",
        "OccupancyGridModule",
        "VoxelGridModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "nav.maps",
        "CameraBridgeModule",
        "PerceptionModule",
        "ReconstructionModule",
        "SemanticPlannerModule",
        "LLMModule",
        "GatewayModule",
        "MCPServerModule",
        "TeleopModule",
        "RerunBridgeModule",
        "map.out",
        "nav.out",
        "ExternalServiceManagerModule",
    }.isdisjoint(modules)
    assert config["enable_device_manager"] is False
    assert not graph.dangling_wires()

    bp = blueprint_for_profile(
        "lite",
        run_startup_checks=False,
        manage_external_services=False,
    )
    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["nav.mission"]["planner"] == "direct"
    assert configs["nav.mission"].get("tomogram", "") == ""
    assert configs["nav.terrain"]["backend"] == "simple"
    assert configs["nav.local_planner"]["backend"] == "simple"
    assert configs["nav.path_follower"]["backend"] == "pid"


def test_navigation_profiles_use_localization_odometry_for_runtime_consumers():
    for profile in REAL_LOCALIZATION_PROFILES:
        source = ENDPOINT_SLAM_MODULE
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)
        slam_topic = source in {"SlamModule", "SlamAdapterModule", "SlamBridgeModule"}
        odom_suffix = f"@{TOPICS.odometry}" if slam_topic else ""
        map_suffix = f"@{TOPICS.map_cloud}" if slam_topic else ""
        health_suffix = f"@{TOPICS.localization_health}" if slam_topic else ""
        quality_suffix = f"@{TOPICS.localization_quality}" if slam_topic else ""
        has_mission_stack = "nav.mission" in graph.modules

        assert source in graph.modules
        assert f"{source}.odometry->GatewayModule.odometry{odom_suffix}" in wires
        assert f"{source}.odometry->nav.safety.odometry{odom_suffix}" in wires
        assert f"{source}.map_cloud->OccupancyGridModule.map_cloud{map_suffix}" in wires
        assert f"{source}.map_cloud->VoxelGridModule.map_cloud{map_suffix}" in wires
        assert f"{source}.map_cloud->ElevationMapModule.map_cloud{map_suffix}" in wires
        assert f"{source}.map_cloud->GatewayModule.map_cloud{map_suffix}" in wires
        assert f"{source}.localization_status->nav.safety.localization_status{health_suffix}" in wires
        assert f"{source}.localization_status->GatewayModule.localization_status{health_suffix}" in wires
        assert f"{source}.localization_quality->GatewayModule.localization_quality{quality_suffix}" in wires

        if has_mission_stack:
            assert f"{source}.odometry->nav.mission.odometry{odom_suffix}" in wires
            assert f"{source}.odometry->nav.path_follower.odometry{odom_suffix}" in wires
            assert f"{source}.odometry->nav.local_planner.odometry{odom_suffix}" in wires
            assert f"{source}.map_cloud->nav.terrain.map_cloud{map_suffix}" in wires
            assert f"{source}.localization_status->nav.mission.localization_status{health_suffix}" in wires
        else:
            # teleop_avoid / map contracts forbid nav.mission / local_planner /
            # path_follower; map_cloud feeds nav.maps directly instead of
            # going through the (absent) nav.terrain local-autonomy stage.
            assert f"{source}.map_cloud->nav.maps.map_cloud{map_suffix}" in wires

        if "DepthVisualOdomModule" in graph.modules:
            assert (
                f"{source}.localization_status->DepthVisualOdomModule.localization_status"
                f"{health_suffix}"
                in wires
            )


def test_super_lio_profiles_wire_endpoint_localization_status_to_gateway():
    for profile in ("super_lio", "super_lio_relocation"):
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert ENDPOINT_SLAM_MODULE in graph.modules
        assert "SlamBridgeModule" not in graph.modules
        assert (
            f"{ENDPOINT_SLAM_MODULE}.localization_status->GatewayModule.localization_status"
            f"@{TOPICS.localization_health}"
            in wires
        )
        assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.mission.map_frame_jump_event" in wires
        assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.local_planner.map_frame_jump_event" in wires
        assert f"{ENDPOINT_SLAM_MODULE}.map_frame_jump_event->nav.path_follower.map_frame_jump_event" in wires
        assert not graph.dangling_wires(), profile


def test_profile_groups_make_simulation_boundary_explicit():
    assert set(PROFILE_SNAPSHOT_TARGETS) == (
        (
            set(PRODUCT_PROFILES)
            - set(OPTIONAL_NATIVE_PRODUCT_PROFILES)
            - set(LIGHTWEIGHT_PRODUCT_PROFILES)
        )
        | set(SIMULATION_PROFILES)
    )
    assert not set(PRODUCT_PROFILES) & set(SIMULATION_PROFILES)
    assert "lite" in PRODUCT_PROFILES
    assert "lite" in LIGHTWEIGHT_PRODUCT_PROFILES
    assert "lite" not in PROFILE_SNAPSHOT_TARGETS
    assert "tare_explore" in PRODUCT_PROFILES
    assert OPTIONAL_NATIVE_PRODUCT_PROFILES == ()
    assert "portable_mujoco" in SIMULATION_PROFILES
    assert "sim_mujoco_live" in SIMULATION_PROFILES
    assert "sim_mujoco_octo_live" in SIMULATION_PROFILES
    assert "sim_gazebo" in SIMULATION_PROFILES
    assert "sim_industrial" in SIMULATION_PROFILES
    assert "sim_cmu_tare" in SIMULATION_PROFILES


def test_non_optional_product_graphs_do_not_include_native_modules():
    checked = set(PRODUCT_PROFILES) - set(OPTIONAL_NATIVE_PRODUCT_PROFILES)

    for profile in sorted(checked):
        graph = graph_for_profile(profile)
        native_modules = [
            module_name
            for module_name in graph.modules
            if module_name.endswith("NativeModule")
        ]
        assert native_modules == [], profile


def test_tare_explore_product_graph_uses_lingtu_tare_policy():
    graph = graph_for_profile("tare_explore")
    wires = _wire_set(graph)

    assert {
        module_name
        for module_name in graph.modules
        if module_name.endswith("NativeModule")
    } == set()
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "TAREExplorerModule.exploration_goal->nav.mission.goal_pose" in wires
    assert "OccupancyGridModule.exploration_grid->TAREExplorerModule.exploration_grid" in wires


def test_only_sanctioned_external_simulator_profiles_are_first_class():
    external_profiles = {
        profile
        for profile, config in PROFILES.items()
        if config.get("_external_launcher")
    }

    assert external_profiles == {
        "sim_cmu_tare",
        "sim_mujoco_live",
        "sim_mujoco_octo_live",
        "sim_mujoco_pct_live",
    }


def test_profile_robot_presets_resolve_through_driver_stack():
    from runtime.blueprints.stacks.driver import RobotProfile

    profile_presets = {
        config.get("_default_robot", "stub")
        for config in PROFILES.values()
    }
    endpoint_presets = {
        runtime_endpoint(endpoint_name).robot_preset
        for endpoint_name in runtime_endpoint_names()
    }
    used_presets = profile_presets | endpoint_presets

    assert used_presets <= set(ROBOT_PRESETS)
    assert set(ROBOT_PRESETS) == set(RobotProfile.known_presets())


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
    assert gazebo.rviz_config == "sim/planning/lingtu_industrial_demo.rviz"
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
    assert cmu.rviz_config == "sim/planning/cmu_unity_lingtu_runtime.rviz"
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
    assert runtime_contracts_for_profile("sim_mujoco_octo_live") == (fastlio,)
    assert "gazebo_industrial" in SIMULATION_RUNTIME_CONTRACTS
    assert "cmu_unity_baseline" in SIMULATION_RUNTIME_CONTRACTS
    assert "mujoco_fastlio2_live" in SIMULATION_RUNTIME_CONTRACTS
    assert "rosbag_fastlio2_replay" in SIMULATION_RUNTIME_CONTRACTS
    assert "portable_fastlio2_replay" not in SIMULATION_RUNTIME_CONTRACTS

    for profile in (*SIMULATION_PROFILES, *PRODUCT_PROFILES):
        binding = profile_data_source(profile)
        assert binding.profile == profile
        assert binding.data_source in DATA_SOURCE_CONTRACTS

    assert profile_data_source("sim_industrial").data_source == "gazebo_industrial"
    assert profile_data_source("sim_cmu_tare").data_source == "cmu_unity_external"
    assert profile_data_source("sim_mujoco_live").data_source == "mujoco_fastlio2_live"
    assert profile_data_source("sim_mujoco_octo_live").data_source == "mujoco_fastlio2_live"
    assert profile_data_source("sim").data_source == "mujoco_module_graph"
    assert profile_data_source("lite").data_source == "thunder_lite_local"
    assert profile_data_source("map").data_source == "thunder_field"

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
        planner = nav_config.pop("planner", "octoplanner3d")
        tomogram = nav_config.pop("tomogram", "")
        nav_config.pop("enable_native", False)
        nav_entry_config = navigation_config(
            planner,
            tomogram,
            **nav_config,
        )

        assert nav_entry_config["planning_frame_id"] == "map"


def test_navigation_plan_safety_policy_is_profile_visible():
    sim_config = resolve_profile_config("sim")
    sim_nav_config = dict(sim_config)
    sim_planner = sim_nav_config.pop("planner", "octoplanner3d")
    sim_tomogram = sim_nav_config.pop("tomogram", "")
    sim_nav_config.pop("planner_backend", None)
    sim_nav_config.pop("enable_native", False)
    sim_entry_config = navigation_config(
        sim_planner,
        sim_tomogram,
        **sim_nav_config,
    )

    assert sim_entry_config["plan_safety_policy"] == "reject"
    assert sim_entry_config["fallback_planner_name"] == ""

    for profile in ("nav", "explore", "tare_explore", "super_lio", "super_lio_relocation"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "octoplanner3d")
        tomogram = nav_config.pop("tomogram", "")
        nav_config.pop("planner_backend", None)
        nav_config.pop("enable_native", False)
        nav_entry_config = navigation_config(planner, tomogram, **nav_config)

        assert nav_entry_config["plan_safety_policy"] == "reject"
        assert nav_entry_config["fallback_planner_name"] == ""


def test_navigation_forwards_frontier_reachability_tuning():
    frontier_config = frontier_module_config(
        enable_frontier=True,
        frontier_approach_max_target_distance_m=2.4,
        frontier_approach_goal_max_distance_m=6.0,
        frontier_reachable_goal_radius=1.6,
    )

    assert frontier_config["approach_max_target_distance_m"] == 2.4
    assert frontier_config["approach_goal_max_distance_m"] == 6.0
    assert frontier_config["reachable_goal_radius"] == 1.6


def test_navigation_forwards_path_follower_drive_mode():
    config = autonomy_stack_config(
        False,
        path_follower_two_way_drive=False,
    )

    assert config["path_follower_config"]["two_way_drive"] is False


def test_navigation_forwards_path_follower_safety_guard_config():
    config = autonomy_stack_config(
        False,
        path_follower_use_incl_rate_to_slow=True,
        path_follower_incl_rate_thre=90.0,
        path_follower_slow_rate_1=0.2,
        path_follower_slow_rate_2=0.4,
        path_follower_slow_rate_3=0.7,
        path_follower_slow_time_1=1.5,
        path_follower_slow_time_2=2.5,
        path_follower_use_incl_to_stop=True,
        path_follower_incl_thre=35.0,
        path_follower_stop_time=4.0,
    )

    assert config["path_follower_config"] == {
        "use_incl_rate_to_slow": True,
        "incl_rate_thre": 90.0,
        "slow_rate_1": 0.2,
        "slow_rate_2": 0.4,
        "slow_rate_3": 0.7,
        "slow_time_1": 1.5,
        "slow_time_2": 2.5,
        "use_incl_to_stop": True,
        "incl_thre": 35.0,
        "stop_time": 4.0,
    }


def test_navigation_forwards_independent_autonomy_backends():
    config = autonomy_stack_config(
        False,
        terrain_backend="simple",
        local_planner_backend="cmu_py",
        path_follower_backend="pid",
    )

    assert config["terrain_backend"] == "simple"
    assert config["backend"] == "cmu_py"
    assert config["path_follower_backend"] == "pid"


def test_autonomy_stack_config_matches_runtime_backend_policy_selection():
    runtime_config = {
        "terrain_backend": "simple",
        "python_autonomy_backend": "simple",
        "python_path_follower_backend": "pid",
        "path_follower_two_way_drive": False,
    }

    stack_config = autonomy_stack_config(False, **runtime_config)
    backend_selection = resolved_autonomy_backend_selection(
        runtime_config,
        enable_native=False,
    )

    assert stack_config["terrain_backend"] == backend_selection["terrain_backend"]
    assert stack_config["backend"] == backend_selection["local_planner_backend"]
    assert (
        stack_config["path_follower_backend"]
        == backend_selection["path_follower_backend"]
    )
    assert stack_config["path_follower_config"]["two_way_drive"] is False

