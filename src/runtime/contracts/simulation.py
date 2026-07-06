"""Simulation runtime contracts for LingTu-owned autonomy runs.

Profiles describe the LingTu module graph. Runtime contracts describe how an
external simulator is allowed to feed that graph and what evidence a run must
publish before we call it product-relevant simulation.
"""

from __future__ import annotations

from dataclasses import dataclass

from runtime.runtime_interface import CANONICAL_NAV_TOPICS, DATA_SOURCE_CONTRACTS, TOPICS


def _data_source(name: str):
    return DATA_SOURCE_CONTRACTS[name]


def _source_outputs(name: str) -> tuple[str, ...]:
    return _data_source(name).source_outputs


def _algorithm_entry_outputs(name: str) -> tuple[str, ...]:
    return _data_source(name).algorithm_entry_outputs


def _algorithm_context_outputs(name: str) -> tuple[str, ...]:
    return _data_source(name).algorithm_context_outputs


def _slam_source(name: str) -> str:
    return _data_source(name).slam_source


def _localization_source(name: str) -> str:
    return _data_source(name).localization_source


def _mapping_source(name: str) -> str:
    return _data_source(name).mapping_source


def _runtime_topics_for(name: str, *extra: str) -> tuple[str, ...]:
    return tuple(
        dict.fromkeys(
            (*_algorithm_entry_outputs(name), *_algorithm_context_outputs(name), *extra)
        )
    )


@dataclass(frozen=True)
class SimulationRuntimeContract:
    name: str
    provider: str
    profile: str | None
    world: str | None
    launch_script: str
    rviz_config: str | None
    adapter_script: str | None
    data_source_contract: str
    command_topic: str
    canonical_topics: tuple[str, ...]
    lingtu_owns: tuple[str, ...]
    simulator_owns: tuple[str, ...]
    native_topics: tuple[str, ...] = ()
    required_runtime_topics: tuple[str, ...] = ()
    required_path_topics: tuple[str, ...] = ()
    required_map_growth_topics: tuple[str, ...] = ()
    required_scan_topics: tuple[str, ...] = ()
    required_slam_topics: tuple[str, ...] = ()
    isolated_domain_required: bool = True
    simulation_only: bool = True
    contract_role: str = "lingtu_validation"
    runtime_stage: str = "not_declared"
    map_dependency: str = "not_declared"
    world_sensor_owner: str = "simulator"
    slam_source: str = "not_declared"
    localization_source: str = "not_declared"
    mapping_source: str = "not_declared"
    slam_validated: bool = False
    requires_live_slam: bool = False
    requires_saved_map: bool = False
    requires_tomogram: bool = False
    exploration_owner: str = "lingtu"
    global_planning_owner: str = "lingtu"
    local_planning_owner: str = "lingtu"
    path_following_owner: str = "lingtu"
    cmd_vel_owner: str = "lingtu"
    validated_claims: tuple[str, ...] = ()
    forbidden_claims: tuple[str, ...] = ()

    def as_report(self) -> dict[str, object]:
        return {
            "name": self.name,
            "provider": self.provider,
            "profile": self.profile,
            "world": self.world,
            "launch_script": self.launch_script,
            "rviz_config": self.rviz_config,
            "adapter_script": self.adapter_script,
            "data_source_contract": self.data_source_contract,
            "command_topic": self.command_topic,
            "canonical_topics": list(self.canonical_topics),
            "native_topics": list(self.native_topics),
            "lingtu_owns": list(self.lingtu_owns),
            "simulator_owns": list(self.simulator_owns),
            "required_runtime_topics": list(self.required_runtime_topics),
            "required_path_topics": list(self.required_path_topics),
            "required_map_growth_topics": list(self.required_map_growth_topics),
            "required_scan_topics": list(self.required_scan_topics),
            "required_slam_topics": list(self.required_slam_topics),
            "isolated_domain_required": self.isolated_domain_required,
            "simulation_only": self.simulation_only,
            "contract_role": self.contract_role,
            "runtime_stage": self.runtime_stage,
            "map_dependency": self.map_dependency,
            "world_sensor_owner": self.world_sensor_owner,
            "slam_source": self.slam_source,
            "localization_source": self.localization_source,
            "mapping_source": self.mapping_source,
            "slam_validated": self.slam_validated,
            "requires_live_slam": self.requires_live_slam,
            "requires_saved_map": self.requires_saved_map,
            "requires_tomogram": self.requires_tomogram,
            "exploration_owner": self.exploration_owner,
            "global_planning_owner": self.global_planning_owner,
            "local_planning_owner": self.local_planning_owner,
            "path_following_owner": self.path_following_owner,
            "cmd_vel_owner": self.cmd_vel_owner,
            "validated_claims": list(self.validated_claims),
            "forbidden_claims": list(self.forbidden_claims),
        }


SIMULATION_RUNTIME_CONTRACTS = {
    "gazebo_industrial": SimulationRuntimeContract(
        name="gazebo_industrial",
        provider="gazebo",
        profile="sim_industrial",
        world="sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf",
        launch_script="sim/scripts/launch_lingtu_gazebo_industrial_demo.sh",
        rviz_config="sim/planning/lingtu_industrial_demo.rviz",
        adapter_script="sim/engine/bridge/gazebo_runtime_adapter.py",
        data_source_contract="gazebo_industrial",
        command_topic=_data_source("gazebo_industrial").command_sink,
        canonical_topics=CANONICAL_NAV_TOPICS,
        native_topics=_source_outputs("gazebo_industrial"),
        lingtu_owns=(
            "occupancy_mapping",
            "frontier_or_tare_goal_selection",
            "global_planning",
            "local_planning",
            "path_following",
            "cmd_vel_mux",
            "safety",
        ),
        simulator_owns=(
            "world_geometry",
            "physics",
            "sensor_rendering",
            "simulation_actuation",
        ),
        required_runtime_topics=_runtime_topics_for(
            "gazebo_industrial",
            TOPICS.global_path,
            TOPICS.local_path,
            TOPICS.cmd_vel,
        ),
        required_path_topics=(TOPICS.global_path, TOPICS.local_path),
        required_map_growth_topics=(TOPICS.exploration_grid,),
        required_scan_topics=(TOPICS.registered_cloud,),
        contract_role="lingtu_full_stack_delivery_demo",
        runtime_stage="no_saved_map_live_mapping_smoke",
        map_dependency="gazebo_live_lidar_occupancy",
        world_sensor_owner="gazebo",
        slam_source=_slam_source("gazebo_industrial"),
        localization_source=_localization_source("gazebo_industrial"),
        mapping_source=_mapping_source("gazebo_industrial"),
        slam_validated=False,
        exploration_owner="lingtu_frontier_or_tare",
        global_planning_owner="lingtu_navigation",
        local_planning_owner="lingtu_navigation",
        path_following_owner="lingtu_navigation",
        cmd_vel_owner="lingtu_cmd_vel_mux_to_gazebo_adapter",
        validated_claims=(
            "lingtu_closed_loop_navigation",
            "lidar_derived_map_growth",
            "simulation_only_command_relay",
        ),
        forbidden_claims=(
            "real_robot_readiness",
            "lingtu_fastlio_mapping_validated",
            "lingtu_slam_localization_validated",
            "product_slam_quality_from_cumulative_debug_cloud",
        ),
    ),
    "cmu_unity_baseline": SimulationRuntimeContract(
        name="cmu_unity_baseline",
        provider="cmu_unity",
        profile=None,
        world=None,
        launch_script=(
            "external:autonomy_stack_mecanum_wheel_platform/"
            "system_simulation_with_exploration_planner.sh"
        ),
        rviz_config=(
            "external:autonomy_stack_mecanum_wheel_platform/rviz/"
            "vehicle_simulator.rviz"
        ),
        adapter_script=None,
        data_source_contract="cmu_unity_external",
        command_topic="/cmd_vel",
        canonical_topics=(
            "/state_estimation",
            "/registered_scan",
            "/terrain_map",
            "/terrain_map_ext",
            "/way_point",
            "/path",
            "/cmd_vel",
        ),
        lingtu_owns=(),
        simulator_owns=(
            "world_geometry",
            "physics",
            "sensor_rendering",
            "external_tare_runtime",
            "cmu_terrain_analysis",
            "cmu_local_planning",
            "cmu_path_following",
            "simulation_actuation",
        ),
        required_runtime_topics=(
            "/state_estimation",
            "/registered_scan",
            "/terrain_map_ext",
            "/way_point",
            "/path",
            "/cmd_vel",
        ),
        required_path_topics=("/path",),
        required_map_growth_topics=("/terrain_map_ext",),
        required_scan_topics=("/registered_scan",),
        contract_role="baseline_reference",
        runtime_stage="external_baseline_no_lingtu_claim",
        map_dependency="cmu_unity_live_registered_scan",
        world_sensor_owner="cmu_unity",
        slam_source="none",
        localization_source="cmu_unity_state_estimation",
        mapping_source="cmu_unity_registered_scan_and_terrain_map_ext",
        slam_validated=False,
        exploration_owner="cmu_tare_far",
        global_planning_owner="cmu_exploration_stack",
        local_planning_owner="cmu_local_planner",
        path_following_owner="cmu_path_follower",
        cmd_vel_owner="cmu_path_follower_to_vehicle_simulator",
        validated_claims=(
            "cmu_reference_simulation_effect",
            "cmu_native_exploration_navigation_loop",
        ),
        forbidden_claims=(
            "lingtu_planning_validated",
            "lingtu_cmd_vel_owner",
            "lingtu_product_profile",
            "lingtu_fastlio_mapping_validated",
            "lingtu_slam_localization_validated",
            "real_robot_readiness",
        ),
    ),
    "cmu_unity_external": SimulationRuntimeContract(
        name="cmu_unity_external",
        provider="cmu_unity",
        profile="sim_cmu_tare",
        world=None,
        launch_script="sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        rviz_config="sim/planning/cmu_unity_lingtu_runtime.rviz",
        adapter_script="sim/engine/bridge/cmu_unity_lingtu_adapter.py",
        data_source_contract="cmu_unity_external",
        command_topic=_data_source("cmu_unity_external").command_sink,
        canonical_topics=CANONICAL_NAV_TOPICS,
        native_topics=(*_source_outputs("cmu_unity_external"), "/cmd_vel"),
        lingtu_owns=(
            "map_ingestion",
            "tare_waypoint_supervision",
            "global_planning",
            "local_planning",
            "path_following",
            "cmd_vel_mux",
            "safety",
        ),
        simulator_owns=(
            "world_geometry",
            "physics",
            "sensor_rendering",
            "external_tare_runtime",
            "simulation_actuation",
        ),
        required_runtime_topics=_runtime_topics_for(
            "cmu_unity_external",
            TOPICS.global_path,
            TOPICS.local_path,
            TOPICS.cmd_vel,
        ),
        required_path_topics=(TOPICS.global_path, TOPICS.local_path),
        required_map_growth_topics=(TOPICS.map_cloud, TOPICS.terrain_map_ext),
        required_scan_topics=("/registered_scan", TOPICS.registered_cloud),
        contract_role="lingtu_tare_adapter_execution_gate",
        runtime_stage="external_live_map_execution",
        map_dependency="cmu_unity_live_registered_scan_or_same_source_tomogram",
        world_sensor_owner="cmu_unity",
        slam_source=_slam_source("cmu_unity_external"),
        localization_source=_localization_source("cmu_unity_external"),
        mapping_source=_mapping_source("cmu_unity_external"),
        slam_validated=False,
        exploration_owner="cmu_tare_external",
        global_planning_owner="lingtu_navigation_optional_pct",
        local_planning_owner="lingtu_navigation",
        path_following_owner="lingtu_path_follower",
        cmd_vel_owner="lingtu_adapter_relay_to_cmu_vehicle_simulator",
        validated_claims=(
            "external_tare_waypoints_ingested",
            "lingtu_execution_chain_closed_loop",
            "simulation_only_cmd_vel_relay",
        ),
        forbidden_claims=(
            "cmu_baseline_equivalence",
            "pure_lingtu_exploration",
            "pct_is_tare_executor",
            "lingtu_fastlio_mapping_validated",
            "lingtu_slam_localization_validated",
            "true_lingtu_mapping_closure",
            "real_robot_readiness",
        ),
    ),
    "mujoco_fastlio2_live": SimulationRuntimeContract(
        name="mujoco_fastlio2_live",
        provider="mujoco",
        profile="sim_mujoco_live",
        world="sim/worlds/mujoco/industrial_park_scene.xml",
        launch_script="sim/scripts/mujoco/launch_fastlio2_live.sh",
        rviz_config=None,
        adapter_script=None,
        data_source_contract="mujoco_fastlio2_live",
        command_topic="/cmd_vel",
        canonical_topics=_runtime_topics_for(
            "mujoco_fastlio2_live",
            TOPICS.raw_lidar_points,
            TOPICS.raw_imu,
        ),
        native_topics=(
            *_source_outputs("mujoco_fastlio2_live"),
            "/Odometry",
            "/cloud_registered",
            "/cloud_map",
        ),
        lingtu_owns=(
            "fastlio2_slam_gate",
            "raw_lidar_imu_stream",
            "slam_output_validation",
            "optional_frontier_navigation_gate",
        ),
        simulator_owns=(
            "world_geometry",
            "physics",
            "raw_lidar_imu_simulation",
            "simulation_actuation",
        ),
        required_runtime_topics=_runtime_topics_for(
            "mujoco_fastlio2_live",
            TOPICS.raw_lidar_points,
            TOPICS.raw_imu,
        ),
        required_slam_topics=(
            *_source_outputs("mujoco_fastlio2_live"),
            "/Odometry",
            "/cloud_map",
        ),
        contract_role="mujoco_raw_mid360_fastlio_live_gate",
        runtime_stage="no_saved_map_live_slam_optional_exploration",
        map_dependency="none_raw_lidar_imu",
        world_sensor_owner="mujoco",
        slam_source=_slam_source("mujoco_fastlio2_live"),
        localization_source=_localization_source("mujoco_fastlio2_live"),
        mapping_source=_mapping_source("mujoco_fastlio2_live"),
        slam_validated=True,
        requires_live_slam=True,
        exploration_owner="lingtu_frontier_optional",
        global_planning_owner="lingtu_navigation_optional",
        local_planning_owner="lingtu_navigation_optional",
        path_following_owner="lingtu_path_follower_optional",
        cmd_vel_owner="mujoco_velocity_adapter_or_fixed_gate_motion",
        validated_claims=(
            "fastlio2_lidar_imu_mapping_localization",
            "true_mapping_input_path",
            "canonical_nav_output_relay",
        ),
        forbidden_claims=(
            "cmu_unity_exploration_validated",
            "real_robot_readiness",
        ),
    ),
    "rosbag_fastlio2_replay": SimulationRuntimeContract(
        name="rosbag_fastlio2_replay",
        provider="replay",
        profile=None,
        world=None,
        launch_script="sim/scripts/fastlio2_rosbag_replay_gate.py",
        rviz_config=None,
        adapter_script="sim/scripts/rosbag_slam_bridge_replay.py",
        data_source_contract="rosbag_fastlio2_replay",
        command_topic="no_actuation_replay_sink",
        canonical_topics=CANONICAL_NAV_TOPICS,
        native_topics=_source_outputs("rosbag_fastlio2_replay"),
        lingtu_owns=(
            "recorded_input_adapter",
            "navigation_graph_replay",
            "local_planning",
            "path_following",
            "cmd_vel_mux_to_no_actuation_sink",
        ),
        simulator_owns=(
            "recorded_sensor_log",
            "recorded_slam_outputs",
            "no_actuation",
        ),
        required_runtime_topics=_runtime_topics_for(
            "rosbag_fastlio2_replay",
            TOPICS.global_path,
            TOPICS.local_path,
            TOPICS.cmd_vel,
        ),
        required_path_topics=(TOPICS.global_path, TOPICS.local_path),
        required_slam_topics=(
            *_source_outputs("rosbag_fastlio2_replay"),
            "/Odometry",
            "/cloud_map",
        ),
        simulation_only=True,
        contract_role="no_actuation_replay_validation",
        runtime_stage="recorded_sensor_or_slam_replay",
        map_dependency="replayed_same_source_map",
        world_sensor_owner="recorded_log",
        slam_source=_slam_source("rosbag_fastlio2_replay"),
        localization_source=_localization_source("rosbag_fastlio2_replay"),
        mapping_source=_mapping_source("rosbag_fastlio2_replay"),
        slam_validated=True,
        requires_live_slam=False,
        requires_saved_map=False,
        cmd_vel_owner="lingtu_cmd_vel_mux_to_no_actuation_sink",
        validated_claims=("same_graph_no_actuation_replay",),
        forbidden_claims=("real_robot_motion", "hardware_command_output"),
    ),
}


def simulation_runtime_contract(name: str) -> SimulationRuntimeContract:
    try:
        return SIMULATION_RUNTIME_CONTRACTS[name]
    except KeyError as exc:
        raise KeyError(f"unknown simulation runtime contract: {name}") from exc


def runtime_contracts_for_profile(profile: str) -> tuple[SimulationRuntimeContract, ...]:
    from runtime.runtime_profiles import PROFILES

    declared_contract = str(PROFILES.get(profile, {}).get("_runtime_contract") or "")
    return tuple(
        contract
        for contract in SIMULATION_RUNTIME_CONTRACTS.values()
        if contract.profile == profile
        or (declared_contract and contract.name == declared_contract)
    )
