from __future__ import annotations

from runtime.blueprints.profile_graph import graph_for_profile, resolve_profile_config


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def test_explore_profile_owns_wavefront_frontier_through_navigation_stack():
    config = resolve_profile_config("explore")
    graph = graph_for_profile("explore")
    wires = _wire_set(graph)

    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "TAREExplorerModule" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "ExplorationSupervisorModule" not in graph.modules
    assert "WavefrontFrontierExplorer.exploration_goal->nav.mission.goal_pose" in wires
    assert (
        "nav.mission.mission_status->WavefrontFrontierExplorer.navigation_status"
        in wires
    )
    assert (
        "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers"
        in wires
    )
    assert (
        "TraversableFrontierModule.frontier_candidate->nav.mission.goal_pose"
        not in wires
    )
    assert not graph.dangling_wires()


def test_tare_explore_profile_uses_lingtu_tare_without_native_module():
    config = resolve_profile_config("tare_explore")
    graph = graph_for_profile("tare_explore")
    wires = _wire_set(graph)

    assert config["enable_frontier"] is False
    assert config["enable_traversable_frontier"] is False
    assert config["exploration_backend"] == "tare"
    assert config["robot"] == "thunder"
    assert config["slam_profile"] == "fastlio2"
    assert config["planner"] == "octoplanner3d"
    assert config["enable_native"] is False
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules
    assert "TAREExplorerModule.exploration_goal->nav.mission.goal_pose" in wires
    assert "OccupancyGridModule.exploration_grid->TAREExplorerModule.exploration_grid" in wires
    assert "nav.mission.mission_status->TAREExplorerModule.navigation_status" in wires
    assert not graph.dangling_wires()


def test_tare_explore_cmu_unity_endpoint_uses_external_tare_bridge():
    config = resolve_profile_config("tare_explore", runtime_endpoint="cmu_unity")
    graph = graph_for_profile("tare_explore", runtime_endpoint="cmu_unity")
    wires = _wire_set(graph)

    assert config["enable_frontier"] is False
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_nav_out"] is False
    assert "enable_ros2_bridge" not in config
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules
    assert "nav.out" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->nav.mission.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->nav.mission.patrol_goals" in wires
    assert "nav.mission.mission_status->TAREExplorerModule.navigation_status" in wires
    assert "SimEndpointDriverModule.odometry->TAREExplorerModule.odometry" in wires
    assert not graph.dangling_wires()
