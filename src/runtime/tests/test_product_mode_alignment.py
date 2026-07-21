from __future__ import annotations

import re
from pathlib import Path

from runtime.graph.loader import load_runtime_graph
from lingtu.assembly.graph import graph_for_profile
from runtime.profiles.catalog.product_intents import PRODUCT_MODE_PROFILES
from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS
from runtime.runtime_interface import TOPICS

ROOT = Path(__file__).resolve().parents[3]


def test_runtime_graph_is_the_product_mode_source_of_truth() -> None:
    products = load_runtime_graph().products
    switchable = {name for name, product in products.items() if product.get("operator_switchable") is True}

    assert switchable == set(PRODUCT_MODE_PROFILES)
    assert switchable == set(PRODUCT_MODE_CONTRACTS)

    defaults = {
        contract.session_mode: profile
        for profile, contract in PRODUCT_MODE_CONTRACTS.items()
        if contract.default_for_session_mode
    }
    assert defaults == {
        "mapping": "map",
        "navigating": "nav",
        "exploring": "tare_explore",
    }


def test_field_modes_do_not_mount_python_motion_safety() -> None:
    for profile in PRODUCT_MODE_PROFILES:
        modules = set(graph_for_profile(profile, runtime_endpoint="thunder_field").modules)

        assert "nav.safety" not in modules, profile
        assert "GeofenceManagerModule" not in modules, profile
        assert "nav.velocity_mux" not in modules, profile


def test_native_control_modes_match_the_cpp_endpoint_enum() -> None:
    source = (ROOT / "src" / "nav" / "cpp" / "endpoint" / "nav_endpoint_config.cpp").read_text(encoding="utf-8")
    block = source.split("const char* controlModeName", 1)[1].split("const char* globalPlannerBackendName", 1)[0]
    cpp_modes = set(re.findall(r'return "([^"]+)";', block)) - {"unknown"}
    contract_modes = {contract.native_control_mode for contract in PRODUCT_MODE_CONTRACTS.values()}

    assert cpp_modes == contract_modes


def test_operator_mode_switches_match_the_cold_restart_executor() -> None:
    for contract in PRODUCT_MODE_CONTRACTS.values():
        assert contract.switch_policy == "cold_restart", contract.profile
        assert contract.online_hot_switch_supported is False, contract.profile
        assert not contract.hot_switch_candidates, contract.profile

    script = (ROOT / "scripts" / "lingtu").read_text(encoding="utf-8")
    assert 'if [ "$lifecycle" != "cold_restart" ]' in script

    gateway_switch = (ROOT / "src" / "gateway" / "services" / "runtime_switch_execute.py").read_text(encoding="utf-8")
    assert "if contract.requires_map" in gateway_switch
    assert '"teleop_avoid",\n        "tracking"' not in gateway_switch


def test_teleop_avoid_is_map_free_native_assisted_teleop() -> None:
    contract = PRODUCT_MODE_CONTRACTS["teleop_avoid"]

    assert contract.requires_map is False
    assert contract.native_control_mode == "teleop_avoid"
    assert contract.slam_mode == "mapping"
    assert "/nav/traversability" in contract.required_topics
    assert "operator_assisted_local_planner_control" in contract.required_capabilities

    script = (ROOT / "scripts" / "lingtu").read_text(encoding="utf-8")
    assert 'if [ "$MODE_TARGET_REQUIRES_MAP" = "true" ]' in script
    assert 'case "$target" in\n                tracking|nav|inspection)' not in script


def test_product_claims_match_enabled_features() -> None:
    products = load_runtime_graph().products
    inspection = products["inspection"]
    tracking = products["tracking"]
    tare = products["tare_explore"]

    assert "semantic_or_scheduled_goal_source" not in inspection.get("required_capabilities", [])
    assert all("TaskScheduler" not in step for step in inspection.get("target_chain", []))
    assert all("externally computed tracking path" not in step for step in tracking.get("target_chain", []))
    assert tare["requires_map"] is True
    assert tare["slam_mode"] == "localization"
    assert TOPICS.exploration_snapshot in tare.get("required_topics", [])
