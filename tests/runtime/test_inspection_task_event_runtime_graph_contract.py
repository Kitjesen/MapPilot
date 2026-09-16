from __future__ import annotations

from lingtu.assembly.graph import load_runtime_graph, resolve_env_implementation
from lingtu.assembly.graph.loader import resolve_product_variant_spec
from message.topics import TOPICS, topic_spec
from runtime.route_contract.routes import robot


def _native_endpoint_contracts() -> tuple[dict[str, object], ...]:
    graph = load_runtime_graph()
    real = resolve_env_implementation("real", graph=graph)
    sim = resolve_env_implementation(
        "sim",
        graph=graph,
        env_config={"backend": "mujoco"},
    )
    return (
        dict(real["endpoints"]["contract"]),
        dict(sim["endpoints"]["contract"]),
    )


def test_inspection_task_ingress_binds_caller_task_identity_end_to_end() -> None:
    graph = load_runtime_graph()
    request_topic = TOPICS.inspection_task_request
    ack_topic = TOPICS.inspection_task_ack
    assert graph.topic_contracts[request_topic]["message_type"] == (
        "lingtu.dds.InspectionTaskRequest"
    )
    assert graph.topic_contracts[request_topic]["semantics"] == (
        "caller_task_id_and_retryable_request_id"
    )
    assert graph.topic_contracts[ack_topic]["message_type"] == (
        "lingtu.dds.InspectionTaskAck"
    )
    assert graph.topic_contracts[ack_topic]["semantics"] == "task_id_preserving_business_ack"

    for endpoint in _native_endpoint_contracts():
        assert endpoint["inspection_task_boundary"] == {
            "request": request_topic,
            "ack": ack_topic,
            "status": TOPICS.inspection_status,
            "client_completion": "business_ack_required",
            "request_identity_fields": ["task_id", "request_id"],
            "response_identity_fields": ["task_id", "request_id"],
        }

    assert topic_spec(request_topic).message_type == "lingtu.dds.InspectionTaskRequest"
    assert topic_spec(ack_topic).message_type == "lingtu.dds.InspectionTaskAck"
    assert robot().binding_for("dds", request_topic) == {}
    assert robot().binding_for("dds", ack_topic) == {}


def test_inspection_task_event_is_an_honest_native_product_fact_stream() -> None:
    graph = load_runtime_graph()
    topic = TOPICS.inspection_task_event

    assert topic in graph.native_contract_topics
    contract = graph.topic_contracts[topic]
    assert contract["message_type"] == "lingtu.dds.InspectionTaskEvent"
    assert contract["qos_profile"] == "TaskEvent"
    assert contract["producer"] == "native_nav_runtime"
    assert contract["consumers"] == ["host_bus"]
    assert contract["semantics"] == "ordered_task_facts_with_boot_id_and_event_sequence"

    for endpoint in _native_endpoint_contracts():
        assert topic in endpoint["exposed_topics"]
        assert endpoint["inspection_task_event_stream"] == {
            "topic": topic,
            "ordering_cursor": ["boot_id", "event_sequence"],
            "terminal_truth": "native_stop_evidence_before_terminal_state",
        }

    inspection = graph.products["inspection"]
    assert topic in resolve_product_variant_spec("inspection", inspection)["topics"]

    assert topic_spec(topic).message_type == "lingtu.dds.InspectionTaskEvent"
    assert contract["frame"] == "map"
    assert robot().binding_for("dds", topic) == {}
