from __future__ import annotations

import re
from pathlib import Path
from typing import get_args

from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.native_nav import compile_native_nav_config
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.products.host_defaults import FIELD_PRODUCT_NAMES
from lingtu.products import (
    OPERATOR_PRODUCT_LIFECYCLES,
    ProductName,
    product_lifecycle,
)
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph.loader import load_runtime_graph, resolve_product_variant_spec
from runtime.runtime_interface import TOPICS

ROOT = Path(__file__).resolve().parents[3]


def test_product_name_type_covers_every_runtime_graph_product() -> None:
    assert set(get_args(ProductName)) == {
        "teleop",
        "teleop_avoid",
        "map",
        "explore",
        "nav",
        "tracking",
        "inspection",
    }


def test_explore_product_declares_live_and_map_variants() -> None:
    products = load_runtime_graph().products

    assert "tare_explore" not in products
    explore = products["explore"]
    assert explore["default_variant"] == "live"
    assert set(explore["variants"]) == {"live", "map"}

    live = resolve_product_variant_spec("explore", explore)
    mapped = resolve_product_variant_spec(
        "explore",
        explore,
        product_variant="map",
    )

    assert live["product_variant"] == "live"
    assert live["slam_mode"] == "mapping"
    assert live["requires_map"] is False
    assert mapped["product_variant"] == "map"
    assert mapped["slam_mode"] == "localization"
    assert mapped["requires_map"] is True
    assert live["processes"] == mapped["processes"]
    assert live["switch_policy"] == mapped["switch_policy"] == "cold_restart"


def test_runtime_graph_is_the_operator_product_source_of_truth() -> None:
    products = load_runtime_graph().products
    switchable = {name for name, product in products.items() if product.get("operator_switchable") is True}

    assert set(FIELD_PRODUCT_NAMES) == set(products)
    assert switchable == set(OPERATOR_PRODUCT_LIFECYCLES)
    assert switchable == set(products)

    defaults = {
        contract.session_mode: profile
        for profile, contract in OPERATOR_PRODUCT_LIFECYCLES.items()
        if contract.default_for_session_mode
    }
    assert defaults == {
        "mapping": "map",
        "navigating": "nav",
        "exploring": "explore",
    }


def test_explore_lifecycle_resolves_the_requested_internal_variant() -> None:
    live = product_lifecycle("explore", product_variant="live")
    mapped = product_lifecycle("explore", product_variant="map")

    assert live.product == mapped.product == "explore"
    assert live.product_variant == "live"
    assert live.slam_mode == "mapping"
    assert live.requires_map is False
    assert mapped.product_variant == "map"
    assert mapped.slam_mode == "localization"
    assert mapped.requires_map is True


def test_field_mapd_modes_do_not_mount_python_live_map_layers() -> None:
    products = load_runtime_graph().products
    python_layers = {
        "OccupancyGridModule",
        "VoxelGridModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
    }

    for profile, product in products.items():
        if "maps" not in product.get("processes", []):
            continue
        resolved = resolve_product_host_runtime(profile, "real", robot="unitree/go2")
        modules = set(
            compile_run_plan(
                resolved.product,
                resolved.env,
                robot="unitree/go2",
                product_variant=resolved.product_variant,
            ).modules
        )
        assert not (python_layers & modules), profile


def test_field_traversability_has_one_native_control_writer() -> None:
    topics = load_runtime_graph().topic_contracts
    nav_contract = topics[TOPICS.traversability]
    mapd_dds = (ROOT / "src" / "maps" / "cpp" / "mapd" / "dds.cpp").read_text(encoding="utf-8")
    traversability_dds = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "traversability" / "traversability_dds.cpp"
    ).read_text(encoding="utf-8")

    assert nav_contract["field_producer"] == "native_traversability_endpoint"
    assert nav_contract["single_writer_per_product"] is True
    assert "/maps/traversability" not in topics
    assert "kMapsTraversability" not in mapd_dds
    assert "kNavTraversability" not in mapd_dds
    assert "kNavTraversability" in traversability_dds
    assert "kMapsTraversability" not in traversability_dds


def test_native_control_modes_match_the_cpp_endpoint_enum() -> None:
    source = (
        ROOT / "src" / "nav" / "cpp" / "endpoint" / "config" / "build.cpp"
    ).read_text(encoding="utf-8")
    block = source.split("controlModeName(ControlMode mode)", 1)[1].split(
        "globalPlannerBackendName",
        1,
    )[0]
    cpp_modes = set(re.findall(r'return "([^"]+)";', block)) - {"unknown"}
    products = load_runtime_graph().products
    contract_modes = {str(product["native_control_mode"]) for product in products.values()}

    assert cpp_modes == contract_modes


def test_product_native_nav_contract_matches_operator_mode() -> None:
    for profile, product in load_runtime_graph().products.items():
        native_nav = compile_native_nav_config(profile, product).native_nav
        native_control_mode = product["native_control_mode"]

        assert native_nav["publish_cmd_vel"] is True
        if native_control_mode == "teleop":
            assert native_nav["check_obstacle"] is False
            assert native_nav["use_traversability_cost"] is False
            assert native_nav["teleop_local_planner"] is False
        if native_control_mode == "teleop_avoid":
            assert native_nav["check_obstacle"] is True
            assert native_nav["use_traversability_cost"] is False
            assert native_nav["teleop_local_planner"] is True


def test_operator_mode_switches_match_the_cold_restart_executor() -> None:
    for contract in OPERATOR_PRODUCT_LIFECYCLES.values():
        assert contract.switch_policy == "cold_restart", contract.product
        assert contract.online_hot_switch_supported is False, contract.product


def test_teleop_avoid_is_map_free_native_assisted_teleop() -> None:
    lifecycle = OPERATOR_PRODUCT_LIFECYCLES["teleop_avoid"]
    product = load_runtime_graph().products["teleop_avoid"]
    contract = resolve_product_spec_contracts("teleop_avoid", product)

    assert lifecycle.requires_map is False
    assert lifecycle.native_control_mode == "teleop_avoid"
    assert lifecycle.slam_mode == "mapping"
    assert "/nav/traversability" in contract.topics
    assert "operator_assisted_local_planner_control" in contract.capabilities


def test_product_claims_match_enabled_features() -> None:
    products = load_runtime_graph().products
    explore = products["explore"]
    mapped_explore = resolve_product_variant_spec(
        "explore",
        explore,
        product_variant="map",
    )
    inspection = products["inspection"]
    tracking = products["tracking"]
    explore_contract = resolve_product_spec_contracts("explore", explore)
    mapped_explore_contract = resolve_product_spec_contracts(
        "explore",
        explore,
        product_variant="map",
    )
    inspection_contract = resolve_product_spec_contracts("inspection", inspection)
    tracking_contract = resolve_product_spec_contracts("tracking", tracking)

    assert "semantic_or_scheduled_goal_source" not in inspection_contract.capabilities
    assert "target_chain" not in inspection
    assert "target_chain" not in tracking
    assert tracking_contract.contract_ids == ("lingtu.product.tracking.v1",)
    assert explore["requires_map"] is False
    assert explore["slam_mode"] == "mapping"
    assert "rolling_map_segment_execution" in explore_contract.capabilities
    assert "octoplanner3d_global_planning" not in explore_contract.capabilities
    assert mapped_explore["requires_map"] is True
    assert mapped_explore["slam_mode"] == "localization"
    assert "octoplanner3d_global_planning" in mapped_explore_contract.capabilities
    assert "rolling_map_segment_execution" in mapped_explore_contract.capabilities
    assert TOPICS.exploration_snapshot in mapped_explore_contract.topics
