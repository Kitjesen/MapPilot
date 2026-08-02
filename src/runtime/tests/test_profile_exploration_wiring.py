from __future__ import annotations

import pytest

from lingtu.assembly.graph import graph_for_product
from lingtu.assembly.products import resolve_product_host_config


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def test_exploration_package_has_one_canonical_import_surface():
    from explore.frontier import WavefrontFrontierExplorer
    from explore.tare.module import TAREExplorerModule
    from explore.traversable_frontier import TraversableFrontierModule

    assert WavefrontFrontierExplorer.__module__ == "explore.frontier"
    assert TAREExplorerModule.__module__ == "explore.tare.module"
    assert TraversableFrontierModule.__module__ == "explore.traversable_frontier"


def test_explore_product_delegates_frontier_execution_to_native_navd():
    config = resolve_product_host_config("explore", "real")
    graph = graph_for_product("explore", env="real")
    wires = _wire_set(graph)

    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert "host.bus" in graph.modules
    assert "nav.mission" not in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "TAREExplorerModule" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "ExplorationSupervisorModule" not in graph.modules
    assert not any("WavefrontFrontierExplorer" in wire for wire in wires)
    assert not any("TraversableFrontierModule" in wire for wire in wires)
    assert not any("nav.mission" in wire for wire in wires)
    assert not graph.dangling_wires()


def test_explore_map_variant_delegates_saved_map_coverage_to_native_endpoint():
    config = resolve_product_host_config(
        "explore",
        "real",
        product_variant="map",
    )
    graph = graph_for_product(
        "explore",
        env="real",
        product_variant="map",
    )

    assert config["enable_frontier"] is False
    assert config["enable_traversable_frontier"] is False
    assert config["exploration_backend"] == "tare"
    assert config["robot"] == "thunder"
    assert config["slam_profile"] == "localizer"
    assert config["planner"] == "octoplanner3d"
    assert config["map_path"].endswith((".ot", ".bt"))
    assert config["map_artifact_gate_required"] is True
    assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert config["command_output_mode"] == "endpoint_only"
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "TAREExplorerModule" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "ExplorationSupervisorModule" not in graph.modules
    assert not graph.dangling_wires()

def test_explore_map_variant_rejects_retired_cmu_unity_product_backend():
    with pytest.raises(ValueError, match="unsupported sim backend 'cmu_unity'"):
        resolve_product_host_config(
            "explore",
            "sim",
            product_variant="map",
            env_config={"backend": "cmu_unity"},
        )
