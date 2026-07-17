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

REAL_PRODUCT_PROFILES = (
    "teleop",
    "teleop_avoid",
    "map",
    "tracking",
    "nav",
    "inspection",
    "explore",
    "tare_explore",
)
LEGACY_MODULE_SIM_PROFILES = ("sim_mujoco_live", "sim_mujoco_octo_live")
INSPECTION_DDS_TOPICS = (
    TOPICS.inspection_command,
    TOPICS.inspection_ack,
    TOPICS.inspection_status,
    TOPICS.inspection_evidence_request,
    TOPICS.inspection_evidence_result,
)


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


def test_inspection_dds_topics_are_a_native_runtime_contract() -> None:
    graph = load_runtime_graph()

    assert set(INSPECTION_DDS_TOPICS) <= set(graph.native_contract_topics)
    assert graph.topic_contracts[TOPICS.inspection_command] == {
        "role": "native_inspection_command",
        "frame": "map",
        "schema": "inspection_command_request",
        "producer": "persistent_cpp_navigation_client",
        "consumers": ["native_nav_runtime"],
        "qos": "reliable_volatile_keep_last_32",
        "semantics": "route_start_pause_resume_or_cancel",
        "port_bindings": [
            {
                "owner": "persistent_cpp_navigation_client",
                "port": "inspection_command",
                "direction": "out",
                "boundary": "native",
            },
            {
                "owner": "native_nav_runtime",
                "port": "inspection_command",
                "direction": "in",
                "boundary": "endpoint",
            },
        ],
    }
    assert "persistent_cpp_navigation_client" in graph.topic_contracts[
        TOPICS.inspection_ack
    ]["consumers"]
    assert graph.topic_contracts[TOPICS.inspection_status]["qos"] == (
        "reliable_transient_local_keep_last_1"
    )


def test_native_endpoints_close_the_inspection_command_ack_status_chain() -> None:
    graph = load_runtime_graph()

    for endpoint_name in ("thunder_field", "mujoco_native_dds"):
        endpoint = graph.endpoints[endpoint_name]
        assert TOPICS.inspection_command in endpoint["source_topics"]
        assert TOPICS.inspection_ack in endpoint["exposed_topics"]
        assert TOPICS.inspection_status in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_request in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_result in endpoint["source_topics"]
        assert endpoint["inspection_command_boundary"] == {
            "request": TOPICS.inspection_command,
            "ack": TOPICS.inspection_ack,
            "status": TOPICS.inspection_status,
            "client_completion": "business_ack_required",
        }
        assert endpoint["inspection_evidence_boundary"] == {
            "request": TOPICS.inspection_evidence_request,
            "result": TOPICS.inspection_evidence_result,
            "client_completion": "matching_persisted_result_required",
        }


def test_inspection_product_requires_its_native_control_and_status_topics() -> None:
    graph = load_runtime_graph()
    inspection = graph.products["inspection"]

    assert set(INSPECTION_DDS_TOPICS) <= set(inspection["required_topics"])
    assert inspection["inspection_command_boundary"] == {
        "request": TOPICS.inspection_command,
        "ack": TOPICS.inspection_ack,
        "status": TOPICS.inspection_status,
        "client_completion": "business_ack_required",
    }
    assert inspection["inspection_evidence_boundary"] == {
        "request": TOPICS.inspection_evidence_request,
        "result": TOPICS.inspection_evidence_result,
        "client_completion": "matching_persisted_result_required",
    }


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


def test_real_profile_rejects_noncanonical_or_duplicate_driver(monkeypatch) -> None:
    from runtime.profiles.resolver import resolve_profile_config

    broken = dict(resolve_profile_config("nav"))
    broken["hardware_control_boundary"] = "dds_endpoint_source"
    broken["enable_robot_driver"] = True
    monkeypatch.setattr(
        "runtime.profiles.resolver.resolve_profile_config",
        lambda _profile: broken,
    )

    issues = validate_profile_against_runtime_graph("nav")
    codes = {issue.code for issue in issues}

    assert "real_profile_driver_boundary_drift" in codes
    assert "real_profile_duplicate_driver" in codes


def test_runtime_graph_renderers_emit_human_readable_contracts() -> None:
    mermaid = render_endpoint_mermaid("mujoco_native_dds")
    markdown = render_product_markdown("nav")

    assert mermaid.startswith("flowchart LR")
    assert "mujoco_native_dds" in mermaid
    assert TOPICS.raw_lidar_points in mermaid
    assert "# nav" in markdown
    assert f"`{TOPICS.odometry}`" in markdown
    assert "native localization" in markdown.lower()
    assert "## Forbidden Modules" in markdown
    assert "`SlamBridgeModule`" not in markdown
