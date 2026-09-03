from __future__ import annotations

import pytest

from lingtu.assembly.compiler import blueprint_from_run_plan, compile_run_plan
from lingtu.assembly.products import (
    resolve_product_host_config,
    resolve_product_host_runtime,
)

pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def _product_graph(
    product: str,
    *,
    env: str = "real",
    robot: str = "unitree/go2",
    product_variant: str | None = None,
):
    resolved = resolve_product_host_runtime(
        product,
        env,
        robot=robot,
        product_variant=product_variant,
    )
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        robot=robot,
        product_variant=resolved.product_variant,
    )
    return blueprint_from_run_plan(plan).export_graph(profile=product)


def test_exploration_package_has_one_canonical_import_surface():
    from explore.tare.module import TAREExplorerModule

    assert TAREExplorerModule.__module__ == "explore.tare.module"


def test_explore_product_delegates_frontier_execution_to_native_navd():
    config = resolve_product_host_config(
        "explore",
        "real",
        robot="unitree/go2",
    )
    graph = _product_graph("explore")
    modules = set(graph.module_names)

    assert config["exploration_backend"] == "none"
    assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert "host.bus" in modules
    assert "TAREExplorerModule" not in modules
    assert "TAREPlannerNativeModule" not in modules
    assert "ExplorationSupervisorModule" not in modules
    assert not graph.dangling_wires()


def test_explore_map_variant_delegates_saved_map_coverage_to_native_endpoint():
    config = resolve_product_host_config(
        "explore",
        "real",
        robot="unitree/go2",
        product_variant="map",
    )
    graph = _product_graph("explore", product_variant="map")
    modules = set(graph.module_names)

    assert config["exploration_backend"] == "tare"
    assert "robot" not in config
    assert config["slam_profile"] == "native_dds"
    assert config["slam_mode"] == "localization"
    assert config["map_artifact_gate_required"] is True
    assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert config["command_output_mode"] == "endpoint_only"
    assert "TAREExplorerModule" not in modules
    assert "TAREPlannerNativeModule" not in modules
    assert "ExplorationSupervisorModule" not in modules
    assert not graph.dangling_wires()


def test_sim_explore_product_uses_native_endpoint_without_python_explorer():
    graph = _product_graph("explore", env="sim", robot="doso/thunder_v4")

    assert "TAREExplorerModule" not in set(graph.module_names)
    assert not graph.dangling_wires()


def test_explore_map_variant_rejects_retired_cmu_unity_product_backend():
    with pytest.raises(ValueError, match="unsupported sim backend 'cmu_unity'"):
        resolve_product_host_config(
            "explore",
            "sim",
            robot="doso/thunder_v4",
            product_variant="map",
            env_config={"backend": "cmu_unity"},
        )
