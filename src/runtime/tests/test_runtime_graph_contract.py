from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from lingtu.assembly.validation import validate_product, validate_profile
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import (
    RuntimeGraph,
    load_runtime_graph,
    render_env_mermaid,
    render_product_markdown,
    resolve_env_implementation,
    resolve_processes,
    validate_runtime_graph,
)
from runtime.profiles.catalog.host_defaults import HOST_PROFILE_DEFAULTS
from runtime.runtime_interface import TOPICS

REPO_ROOT = Path(__file__).resolve().parents[3]

REAL_PRODUCTS = tuple(sorted(load_runtime_graph().products))
HOST_SIMULATION_PROFILES = ("sim_mujoco_live", "sim_mujoco_octo_live")
INSPECTION_DDS_TOPICS = (
    TOPICS.inspection_task_request,
    TOPICS.inspection_task_ack,
    TOPICS.inspection_status,
    TOPICS.inspection_task_event,
    TOPICS.inspection_evidence_request,
    TOPICS.inspection_evidence_result,
)


def _endpoint_contract(
    graph: RuntimeGraph,
    env: str,
    *,
    backend: str | None = None,
) -> dict[str, object]:
    env_config = {"backend": backend} if backend else None
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    return dict(implementation["endpoints"]["contract"])


def _native_endpoint_contracts(graph: RuntimeGraph) -> tuple[dict[str, object], ...]:
    return (
        _endpoint_contract(graph, "real"),
        _endpoint_contract(graph, "sim", backend="mujoco_native"),
    )


def test_runtime_graph_contracts_are_valid() -> None:
    graph = load_runtime_graph()

    assert set(graph.envs) == {"real", "sim"}
    assert {
        env["schema_version"] for env in graph.envs.values()
    } == {"lingtu.runtime_graph.env.v1"}
    assert {
        product["schema_version"] for product in graph.products.values()
    } == {"lingtu.runtime_graph.product.v1"}
    assert graph.envs["real"]["robot_config_ref"] == "config/robot_config.yaml"
    assert set(graph.envs["sim"]["backends"]) == {
        "mujoco_native",
        "mujoco_host",
        "gazebo",
    }
    assert graph.envs["sim"]["backends"]["gazebo"][
        "supported_product_variants"
    ] == {"explore": ["live"]}
    assert "default_backend" not in graph.envs["sim"]
    assert validate_runtime_graph(graph) == []


def test_runtime_graph_rejects_an_unknown_product_schema() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["schema_version"] = "lingtu.runtime_graph.product.v0"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    assert "product_schema_invalid" in {
        issue.code for issue in validate_runtime_graph(broken)
    }


def test_final_velocity_topic_is_identity_bound_and_single_writer() -> None:
    topic = load_runtime_graph().topics["topics"][TOPICS.cmd_vel]

    assert topic["role"] == "final_velocity_command"
    assert topic["schema"] == "final_velocity_command"
    assert topic["single_writer_per_product"] is True
    assert (
        topic["semantics"]
        == "identity_bound_freshness_bounded_final_velocity_envelope"
    )


def test_real_env_resolves_each_product_role_to_one_process_owner() -> None:
    graph = load_runtime_graph()
    available = resolve_processes("nav", "real", graph=graph)[1]

    assert {process.name for process in available} == set(
        graph.envs["real"]["provided_roles"]
    )
    assert len({process.target for process in available}) == len(available)
    for product_name, product in graph.products.items():
        selected, _available, _conflicts = resolve_processes(
            product_name,
            "real",
            graph=graph,
        )
        assert {process.name for process in selected} == set(product["processes"])

    driver = next(process for process in available if process.name == "driver")
    assert driver.lifecycle == "persistent"


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    (
        ("drop_role", "env_role_ownership_mismatch"),
        ("share_owner", "env_process_owner_duplicate"),
    ),
)
def test_real_env_rejects_incomplete_or_non_unique_role_ownership(
    mutation: str,
    expected_code: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    if mutation == "drop_role":
        envs["real"]["provided_roles"].remove("maps")
    else:
        envs["real"]["processes"]["maps"]["target"] = (
            envs["real"]["processes"]["slam"]["target"]
        )
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == expected_code for issue in issues)


def test_process_resolution_does_not_infer_processes_from_topics() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["required_topics"] = []
    isolated = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    selected, _available, _conflicts = resolve_processes(
        "nav", "real", graph=isolated
    )

    assert any(process.name == "nav" for process in selected)
    assert any(process.name == "traversability" for process in selected)


def test_process_resolution_rejects_unmapped_product_process() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["processes"].append("missing")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    with pytest.raises(ValueError, match="missing"):
        resolve_processes("nav", "real", graph=broken)


def test_sim_process_resolution_requires_an_explicit_supported_backend() -> None:
    with pytest.raises(ValueError, match=r"env_config\.backend"):
        resolve_processes("nav", "sim")

    selected, available, conflicts = resolve_processes(
        "nav",
        "sim",
        env_config={"backend": "mujoco_native"},
    )

    assert selected == available == conflicts == ()
    with pytest.raises(ValueError, match="does not support Product"):
        resolve_processes(
            "map",
            "sim",
            env_config={"backend": "mujoco_native"},
        )
    with pytest.raises(ValueError, match="unknown backend 'cmu_unity'"):
        resolve_processes(
            "explore",
            "sim",
            env_config={"backend": "cmu_unity"},
        )

@pytest.mark.parametrize(
    ("backend", "topic_field"),
    (
        ("mujoco_native", "exposed_topics"),
        ("gazebo", "provided_topics"),
    ),
)
def test_sim_backend_rejects_missing_supported_product_topic(
    backend: str,
    topic_field: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    contract = envs["sim"]["backends"][backend]["endpoints"]["contract"]
    contract[topic_field].remove(TOPICS.nav_goal_status)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_product_topic_missing" for issue in issues)


def test_sim_backend_without_variant_limit_must_support_every_variant() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    del envs["sim"]["backends"]["gazebo"]["supported_product_variants"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "env_product_topic_missing"
        and "explore variant 'map'" in issue.message
        for issue in issues
    )


def test_sim_backend_rejects_unknown_product_variant_limit() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["sim"]["backends"]["gazebo"]["supported_product_variants"] = {
        "explore": ["live", "missing"]
    }
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "env_supported_product_variant_unknown"
        and "missing" in issue.message
        for issue in issues
    )


def test_env_resolution_rejects_legacy_deployment_selector_names() -> None:
    for legacy_name in ("legacy_real_target", "mujoco_native_dds", "sim_mujoco_live"):
        with pytest.raises(ValueError, match="unknown Runtime Graph env"):
            resolve_env_implementation(legacy_name)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("target", "--no-block.service", "invalid target"),
        ("order", "10", "needs integer order"),
        ("timeout_s", 0, "invalid order or timeout"),
    ),
)
def test_process_resolution_rejects_unsafe_process_specs(
    field: str,
    value: object,
    message: str,
) -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["processes"]["nav"][field] = value
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match=message):
        resolve_processes("nav", "real", graph=broken)


def test_runtime_graph_rejects_process_contract_drift() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["processes"].remove("traversability")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_process_missing" and "traversability" in issue.message for issue in issues)


def test_runtime_graph_requires_native_explorer_for_explore_capability() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["processes"].remove("explore")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_process_missing" and "process explore" in issue.message
        for issue in issues
    )


def test_runtime_graph_requires_native_exploration_topics() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    resolved = resolve_product_spec_contracts("explore", products["explore"])
    products["explore"]["required_topics"] = list(resolved.topics)
    products["explore"]["required_topics"].remove(TOPICS.exploration_command)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_contract_invalid"
        and "required_topics mirror disagrees" in issue.message
        for issue in issues
    )


def test_runtime_graph_requires_host_process_not_legacy_runtime_name() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["teleop_avoid"]["processes"].remove("host")
    products["teleop_avoid"]["processes"].append("runtime")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_process_missing"
        and "process host" in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_legacy_service_lists() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["native_services"] = ["legacy.service"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "env_legacy_service_list" for issue in issues)


def test_runtime_graph_native_dds_endpoints_share_core_contract() -> None:
    graph = load_runtime_graph()
    required = set(graph.native_contract_topics)

    for endpoint in _native_endpoint_contracts(graph):
        endpoint_topics = set(endpoint["source_topics"]) | set(endpoint["exposed_topics"])
        assert required <= endpoint_topics
        assert endpoint["data_plane"] == "native_dds"
        assert endpoint["real_equivalent"] is True


def test_operator_motion_topics_are_native_endpoint_contract() -> None:
    graph = load_runtime_graph()
    operator_topics = {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }

    assert operator_topics <= set(graph.native_contract_topics)
    assert graph.topic_contracts[TOPICS.operator_motion_control]["frame"] == "none"
    assert graph.topic_contracts[TOPICS.operator_motion_control]["qos"] == "reliable_volatile_keep_last_32"
    assert graph.topic_contracts[TOPICS.operator_motion_sample]["frame"] == "body"
    assert graph.topic_contracts[TOPICS.operator_motion_sample]["qos"] == "best_effort_volatile_keep_last_1"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["frame"] == "none"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["qos"] == "reliable_transient_local_keep_last_64"
    assert graph.topic_contracts[TOPICS.operator_motion_ack]["consumers"] == [
        "operator_motion_adapter"
    ]
    assert (
        graph.topic_contracts[TOPICS.operator_motion_ack]["semantics"]
        == "native_business_ack_for_claim_hold_release_only"
    )
    assert graph.topic_contracts[TOPICS.operator_motion_status]["frame"] == "map"
    assert graph.topic_contracts[TOPICS.operator_motion_status]["qos"] == "reliable_transient_local_keep_last_1"
    assert graph.topic_contracts[TOPICS.operator_motion_status]["consumers"] == []
    assert graph.topic_contracts[TOPICS.operator_motion_status]["external_diagnostics_subscribable"] is True
    assert (
        graph.topic_contracts[TOPICS.operator_motion_status]["semantics"]
        == "active_source_epoch_last_sample_admission_and_final_output_evidence"
    )

    for endpoint in _native_endpoint_contracts(graph):
        assert TOPICS.operator_motion_control in endpoint["source_topics"]
        assert TOPICS.operator_motion_sample in endpoint["source_topics"]
        assert TOPICS.operator_motion_ack in endpoint["exposed_topics"]
        assert TOPICS.operator_motion_status in endpoint["exposed_topics"]


def test_operator_motion_observability_consumers_are_truthful() -> None:
    graph = load_runtime_graph()
    ack = graph.topic_contracts[TOPICS.operator_motion_ack]
    status = graph.topic_contracts[TOPICS.operator_motion_status]
    client_cpp = (REPO_ROOT / "src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")

    assert ack["consumers"] == ["operator_motion_adapter"]
    assert {
        (binding["owner"], binding["direction"], binding["port"])
        for binding in ack["port_bindings"]
    } == {
        ("native_nav_runtime", "out", "operator_motion_ack"),
        ("operator_motion_adapter", "in", "operator_motion_ack"),
    }
    assert "operator_motion_ack_reader = createReader" in client_cpp
    assert "lingtu::message::kOperatorMotionAck" in client_cpp

    assert status["consumers"] == []
    assert status["external_diagnostics_subscribable"] is True
    assert {
        (binding["owner"], binding["direction"], binding["port"])
        for binding in status["port_bindings"]
    } == {("native_nav_runtime", "out", "operator_motion_status")}


def test_exploration_dds_consumers_do_not_claim_a_gateway_reader() -> None:
    graph = load_runtime_graph()
    grid = graph.topic_contracts[TOPICS.exploration_grid]
    snapshot = graph.topic_contracts[TOPICS.exploration_snapshot]

    assert "gateway" not in grid["consumers"]
    assert "gateway" not in snapshot["consumers"]
    assert set(grid["consumers"]) == {
        "WavefrontFrontierExplorer",
        "TraversableFrontierModule",
    }
    assert snapshot["consumers"] == ["native_explore_endpoint"]


def test_inspection_dds_topics_are_a_native_runtime_contract() -> None:
    graph = load_runtime_graph()

    assert set(INSPECTION_DDS_TOPICS) <= set(graph.native_contract_topics)
    assert "/nav/inspection/command" not in graph.native_contract_topics
    assert "/nav/inspection/ack" not in graph.native_contract_topics
    assert graph.topic_contracts[TOPICS.inspection_task_request] == {
        "role": "native_inspection_task_request",
        "frame": "map",
        "schema": "inspection_task_request",
        "producer": "persistent_cpp_navigation_client",
        "consumers": ["native_nav_runtime"],
        "qos": "reliable_volatile_keep_last_32",
        "semantics": "caller_task_id_and_retryable_request_id",
        "port_bindings": [
            {
                "owner": "persistent_cpp_navigation_client",
                "port": "inspection_task_request",
                "direction": "out",
                "boundary": "native",
            },
            {
                "owner": "native_nav_runtime",
                "port": "inspection_task_request",
                "direction": "in",
                "boundary": "endpoint",
            },
        ],
    }
    assert "persistent_cpp_navigation_client" in graph.topic_contracts[TOPICS.inspection_task_ack]["consumers"]
    assert graph.topic_contracts[TOPICS.inspection_status]["qos"] == ("reliable_transient_local_keep_last_1")


def test_native_ack_and_status_topics_do_not_claim_nonexistent_gateway_readers() -> None:
    graph = load_runtime_graph()

    for topic in (
        TOPICS.exploration_ack,
        TOPICS.inspection_task_ack,
        TOPICS.nav_command_ack,
    ):
        assert graph.topic_contracts[topic]["consumers"] == [
            "persistent_cpp_navigation_client"
        ]

    inspection_status = graph.topic_contracts[TOPICS.inspection_status]
    assert inspection_status["consumers"] == []
    assert inspection_status["external_diagnostics_subscribable"] is True
    assert graph.topic_contracts[TOPICS.inspection_evidence_result]["consumers"] == [
        "native_nav_runtime"
    ]


def test_native_endpoints_close_the_inspection_task_ack_status_chain() -> None:
    graph = load_runtime_graph()

    for endpoint in _native_endpoint_contracts(graph):
        assert TOPICS.inspection_task_request in endpoint["source_topics"]
        assert TOPICS.inspection_task_ack in endpoint["exposed_topics"]
        assert TOPICS.inspection_status in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_request in endpoint["exposed_topics"]
        assert TOPICS.inspection_evidence_result in endpoint["source_topics"]
        assert endpoint["inspection_task_boundary"] == {
            "request": TOPICS.inspection_task_request,
            "ack": TOPICS.inspection_task_ack,
            "status": TOPICS.inspection_status,
            "client_completion": "business_ack_required",
            "request_identity_fields": ["task_id", "request_id"],
            "response_identity_fields": ["task_id", "request_id"],
        }
        assert endpoint["inspection_evidence_boundary"] == {
            "request": TOPICS.inspection_evidence_request,
            "result": TOPICS.inspection_evidence_result,
            "client_completion": "matching_persisted_result_required",
        }


def test_tare_goal_status_is_a_field_endpoint_lifecycle_contract() -> None:
    graph = load_runtime_graph()
    status = graph.topic_contracts[TOPICS.nav_goal_status]

    assert status["role"] == "native_navigation_goal_status"
    assert status["frame"] == "map"
    assert status["schema"] == "navigation_goal_status"
    assert status["producer"] == "native_nav_runtime"
    assert status["consumers"] == ["native_explore_runtime", "host_bus"]
    assert status["qos"] == "reliable_transient_local_keep_last_64"
    assert status["semantics"] == "request_correlated_goal_lifecycle"
    assert {
        (binding["owner"], binding["port"], binding["direction"], binding["boundary"])
        for binding in status["port_bindings"]
    } == {
        ("native_nav_runtime", "goal_status", "out", "endpoint"),
        ("native_explore_runtime", "goal_status", "in", "endpoint"),
        ("host_bus", "navigation_goal_status", "in", "native"),
    }
    assert TOPICS.nav_goal_status in _endpoint_contract(graph, "real")["exposed_topics"]
    for product_name, product_variant in (
        ("explore", "live"),
        ("explore", "map"),
        ("nav", None),
        ("inspection", None),
        ("tracking", None),
    ):
        contract = resolve_product_spec_contracts(
            product_name,
            graph.products[product_name],
            product_variant=product_variant,
        )
        assert TOPICS.nav_goal_status in contract.topics
    assert TOPICS.nav_goal_status not in graph.native_contract_topics


def test_directed_exploration_intent_reuses_the_existing_command_ack_boundary() -> None:
    graph = load_runtime_graph()
    command = graph.topic_contracts[TOPICS.exploration_command]
    ack = graph.topic_contracts[TOPICS.exploration_ack]

    assert command["qos"] == "reliable_volatile_keep_last_32"
    assert command["semantics"] == "exploration_lifecycle_or_directed_target_set_clear"
    assert ack["qos"] == "reliable_transient_local_keep_last_64"
    assert ack["semantics"] == "exploration_business_ack_with_intent_revision"

    directed_target = {
        "set_kind": 5,
        "clear_kind": 6,
        "request_fields": [
            "has_directed_target",
            "directed_target_x",
            "directed_target_y",
            "directed_target_ttl_s",
        ],
        "ack_field": "intent_revision",
        "policy": "soft_tare_candidate_preference",
        "grants_static_boundary_path_authorization": False,
    }
    for endpoint in _native_endpoint_contracts(graph):
        boundary = endpoint["exploration_command_boundary"]
        assert boundary["request"] == TOPICS.exploration_command
        assert boundary["ack"] == TOPICS.exploration_ack
        assert boundary["directed_target"] == directed_target



def test_inspection_product_requires_its_native_task_and_status_topics() -> None:
    graph = load_runtime_graph()
    inspection = graph.products["inspection"]
    contract = resolve_product_spec_contracts("inspection", inspection)

    assert set(INSPECTION_DDS_TOPICS) <= set(contract.topics)
    assert inspection["inspection_task_boundary"] == {
        "request": TOPICS.inspection_task_request,
        "ack": TOPICS.inspection_task_ack,
        "status": TOPICS.inspection_status,
        "client_completion": "business_ack_required",
        "request_identity_fields": ["task_id", "request_id"],
        "response_identity_fields": ["task_id", "request_id"],
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
        envs=graph.envs,
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
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    messages = [issue.message for issue in issues if issue.code == "nav_core_topic_missing"]
    assert any(TOPICS.odometry in message for message in messages)
    assert any(TOPICS.map_cloud in message for message in messages)


def test_runtime_graph_rejects_incomplete_operator_mode_contract() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["teleop"].pop("native_control_mode")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_mode_field_missing" and "native_control_mode" in issue.message for issue in issues
    )


def test_runtime_graph_rejects_field_product_topic_without_endpoint_provider() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["endpoints"]["contract"]["exposed_topics"].remove(
        TOPICS.exploration_snapshot
    )
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_topic_missing"
        and "explore" in issue.message
        and TOPICS.exploration_snapshot in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_product_command_boundary_drift() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    contract = resolve_product_spec_contracts("map", products["map"])
    products["map"]["required_topics"] = list(contract.topics)
    products["map"]["required_topics"].remove(TOPICS.cmd_vel)
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_contract_invalid"
        and issue.scope == "product:map"
        and "required_topics mirror disagrees" in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_map_and_slam_mode_conflict() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["nav"]["requires_map"] = False
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_map_slam_conflict" and "nav" in issue.message for issue in issues)


def test_runtime_graph_rejects_map_free_exploration_without_live_route() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["contracts"] = [
        "lingtu.product.teleop_avoid.v1"
    ]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "explore_live_missing" for issue in issues)


def test_runtime_graph_rejects_static_planner_as_map_free_exploration_requirement() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["contracts"] = [
        "lingtu.product.nav.v1"
    ]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "explore_map_conflict" for issue in issues)


def test_runtime_graph_validates_non_default_explore_variant_contract() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["contracts"] = [
        "lingtu.product.missing.v1"
    ]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_contract_invalid"
        and issue.scope == "product:explore:variant:map"
        and "variant 'map'" in issue.message
        for issue in issues
    )


def test_runtime_graph_validates_non_default_explore_variant_lifecycle() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["slam_mode"] = "mapping"
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_map_slam_conflict"
        and issue.scope == "product:explore:variant:map"
        and "variant 'map'" in issue.message
        for issue in issues
    )


def test_runtime_graph_validates_non_default_explore_variant_processes() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["map"]["processes"] = [
        *products["explore"]["processes"],
        "unmapped_variant_process",
    ]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_process_unmapped"
        and issue.scope == "product:explore:variant:map"
        and "unmapped_variant_process" in issue.message
        for issue in issues
    )


def test_runtime_graph_checks_env_topic_closure_for_non_default_variant() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["endpoints"]["contract"]["exposed_topics"].remove(
        TOPICS.operator_motion_status
    )
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "field_product_topic_missing"
        and issue.scope == "product:explore:variant:map"
        and TOPICS.operator_motion_status in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_python_layers_beside_native_maps_process() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["teleop_avoid"]["forbidden_modules"].remove("VoxelGridModule")
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.code == "product_python_realtime_map_not_forbidden"
        and "teleop_avoid" in issue.message
        and "VoxelGridModule" in issue.message
        for issue in issues
    )


def test_runtime_graph_rejects_duplicate_session_defaults() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["tracking"]["default_for_session_mode"] = True
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(issue.code == "product_session_default_invalid" and "navigating" in issue.message for issue in issues)


@pytest.mark.parametrize("product", REAL_PRODUCTS)
def test_real_products_match_runtime_graph(product: str) -> None:
    assert validate_product(product) == []


@pytest.mark.parametrize("profile", HOST_SIMULATION_PROFILES)
def test_mujoco_host_simulation_profiles_are_not_field_equivalent(profile: str) -> None:
    config = HOST_PROFILE_DEFAULTS[profile]

    assert config["_runtime_graph_role"] == "host_simulation"
    assert config["_real_equivalent"] is False
    assert validate_profile(profile) == []


def test_host_simulation_profile_cannot_claim_real_equivalence(monkeypatch) -> None:
    config = dict(HOST_PROFILE_DEFAULTS["sim_mujoco_live"])
    config["_real_equivalent"] = True
    monkeypatch.setitem(HOST_PROFILE_DEFAULTS, "sim_mujoco_live", config)

    issues = validate_profile("sim_mujoco_live")

    assert [issue.code for issue in issues] == ["host_simulation_real_equivalent_flag"]


def test_real_product_rejects_noncanonical_or_duplicate_driver(monkeypatch) -> None:
    from lingtu.assembly.products import resolve_product_host_config

    broken = dict(resolve_product_host_config("nav", "real"))
    broken["hardware_control_boundary"] = "dds_endpoint_source"
    broken["enable_robot_driver"] = True
    monkeypatch.setattr(
        "lingtu.assembly.products.resolve_product_host_config",
        lambda _product, _env, **_kwargs: broken,
    )

    issues = validate_product("nav", env_name="real", module_names=())
    codes = {issue.code for issue in issues}

    assert "real_product_driver_boundary_drift" in codes
    assert "real_product_duplicate_driver" in codes
    assert {issue.scope for issue in issues} == {"product:nav"}


def test_profile_validation_rejects_product_names() -> None:
    with pytest.raises(ValueError, match="use validate_product"):
        validate_profile("nav")


def test_product_validation_rejects_profile_names() -> None:
    with pytest.raises(ValueError, match="use validate_profile"):
        validate_product("sim_nav")


def test_runtime_graph_renderers_emit_human_readable_contracts() -> None:
    mermaid = render_env_mermaid(
        "sim",
        env_config={"backend": "mujoco_native"},
    )
    markdown = render_product_markdown("nav")

    assert mermaid.startswith("flowchart LR")
    assert 'env["sim"]' in mermaid
    assert 'backend["mujoco_native"]' in mermaid
    assert TOPICS.raw_lidar_points in mermaid
    assert "# nav" in markdown
    assert f"`{TOPICS.odometry}`" in markdown
    assert "native localization" in markdown.lower()
    assert "## Forbidden Modules" in markdown
    assert "`SlamBridgeModule`" not in markdown


def test_env_renderer_rejects_missing_endpoint_contract() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    del envs["sim"]["backends"]["mujoco_native"]["endpoints"]["contract"]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    with pytest.raises(ValueError, match=r"endpoints\.contract"):
        render_env_mermaid(
            "sim",
            graph=broken,
            env_config={"backend": "mujoco_native"},
        )
