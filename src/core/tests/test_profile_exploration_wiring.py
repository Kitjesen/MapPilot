from __future__ import annotations

from core.blueprints.profile_graph import graph_for_profile, resolve_profile_config


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
    assert "WavefrontFrontierExplorer.exploration_goal->NavigationModule.goal_pose" in wires
    assert (
        "NavigationModule.mission_status->WavefrontFrontierExplorer.navigation_status"
        in wires
    )
    assert (
        "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers"
        in wires
    )
    assert (
        "TraversableFrontierModule.frontier_candidate->NavigationModule.goal_pose"
        not in wires
    )
    assert not graph.dangling_wires()


def test_tare_explore_profile_keeps_tare_separate_from_wavefront():
    config = resolve_profile_config("tare_explore")

    assert config["enable_frontier"] is False
    assert config["exploration_backend"] == "tare"
    assert config["robot"] == "thunder"
    assert config["slam_profile"] == "fastlio2"
    assert config["planner"] == "pct"
    assert config["enable_native"] is False
    assert config["tare_scenario"] == "forest"


def test_tare_explore_cmu_unity_endpoint_uses_external_tare_bridge():
    config = resolve_profile_config("tare_explore", runtime_endpoint="cmu_unity")
    graph = graph_for_profile("tare_explore", runtime_endpoint="cmu_unity")
    wires = _wire_set(graph)

    assert config["enable_frontier"] is False
    assert config["exploration_backend"] == "tare_external"
    assert config["enable_endpoint_path_bridge"] is True
    assert "enable_ros2_path_bridge" not in config
    assert "TAREExplorerModule" in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TAREExplorerModule.exploration_goal->NavigationModule.goal_pose" in wires
    assert "TAREExplorerModule.exploration_path->NavigationModule.patrol_goals" in wires
    assert "NavigationModule.mission_status->TAREExplorerModule.navigation_status" in wires
    assert "NavigationModule.global_path->EndpointPathBridgeModule.global_path" in wires
    assert not graph.dangling_wires()
