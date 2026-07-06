from __future__ import annotations

from copy import deepcopy

import pytest

from runtime.graph import (
    RuntimeGraph,
    load_runtime_graph,
    render_endpoint_mermaid,
    render_product_markdown,
    validate_profile_against_runtime_graph,
    validate_runtime_graph,
)
from runtime.profiles.catalog.products import PROFILES
from runtime.runtime_interface import TOPICS


REAL_PRODUCT_PROFILES = ("map", "nav", "explore", "teleop_avoid")
LEGACY_MODULE_SIM_PROFILES = ("sim_mujoco_live", "sim_mujoco_octo_live")


def test_runtime_graph_contracts_are_valid() -> None:
    graph = load_runtime_graph()

    assert validate_runtime_graph(graph) == []
    assert "mujoco_native_dds" in graph.endpoints
    assert "thunder_field" in graph.endpoints
    assert graph.endpoints["mujoco_native_dds"]["real_equivalent"] is True
    assert graph.endpoints["sim_mujoco_live"]["real_equivalent"] is False


def test_runtime_graph_native_dds_endpoints_share_core_contract() -> None:
    graph = load_runtime_graph()
    required = set(graph.native_contract_topics)

    for endpoint_name in ("thunder_field", "mujoco_native_dds"):
        endpoint = graph.endpoints[endpoint_name]
        endpoint_topics = set(endpoint["source_topics"]) | set(endpoint["exposed_topics"])
        assert required <= endpoint_topics
        assert endpoint["data_plane"] == "native_dds"
        assert endpoint["real_equivalent"] is True


def test_runtime_graph_rejects_missing_localization_health() -> None:
    graph = load_runtime_graph()
    topics = deepcopy(graph.topics)
    topics["topics"].pop(TOPICS.localization_health)
    broken = RuntimeGraph(
        root=graph.root,
        topics=topics,
        products=graph.products,
        endpoints=graph.endpoints,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "native_topic_missing" for issue in issues)
    assert any(issue.code == "product_required_topic_missing" for issue in issues)


def test_runtime_graph_rejects_nav_without_odom_or_map_cloud() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["required_topics"] = [TOPICS.localization_health, TOPICS.cmd_vel]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        endpoints=graph.endpoints,
    )

    issues = validate_runtime_graph(broken)

    messages = [issue.message for issue in issues if issue.code == "nav_core_topic_missing"]
    assert any(TOPICS.odometry in message for message in messages)
    assert any(TOPICS.map_cloud in message for message in messages)


@pytest.mark.parametrize("profile", REAL_PRODUCT_PROFILES)
def test_real_product_profiles_match_runtime_graph(profile: str) -> None:
    assert validate_profile_against_runtime_graph(profile) == []


@pytest.mark.parametrize("profile", LEGACY_MODULE_SIM_PROFILES)
def test_legacy_mujoco_profiles_are_module_harnesses(profile: str) -> None:
    config = PROFILES[profile]

    assert config["_runtime_graph_role"] == "module_sim_harness"
    assert config["_real_equivalent"] is False
    assert validate_profile_against_runtime_graph(profile) == []


def test_legacy_module_sim_profile_cannot_claim_real_equivalence(monkeypatch) -> None:
    config = dict(PROFILES["sim_mujoco_live"])
    config["_real_equivalent"] = True
    monkeypatch.setitem(PROFILES, "sim_mujoco_live", config)

    issues = validate_profile_against_runtime_graph("sim_mujoco_live")

    assert [issue.code for issue in issues] == ["legacy_module_sim_real_equivalent_flag"]


def test_runtime_graph_renderers_emit_human_readable_contracts() -> None:
    mermaid = render_endpoint_mermaid("mujoco_native_dds")
    markdown = render_product_markdown("nav")

    assert mermaid.startswith("flowchart LR")
    assert "mujoco_native_dds" in mermaid
    assert TOPICS.raw_lidar_points in mermaid
    assert "# nav" in markdown
    assert f"`{TOPICS.odometry}`" in markdown
    assert "`SlamBridgeModule`" in markdown
