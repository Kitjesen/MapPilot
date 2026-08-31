from __future__ import annotations

from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.endpoints.dds.contracts import FIELD_DDS_CONTRACT, binding_for_topic
from runtime.graph import load_runtime_graph, resolve_env_implementation
from runtime.route_contract.routes import robot
from runtime.runtime_interface import TOPICS


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
    assert graph.topic_contracts[request_topic]["schema"] == "inspection_task_request"
    assert graph.topic_contracts[request_topic]["semantics"] == (
        "caller_task_id_and_retryable_request_id"
    )
    assert graph.topic_contracts[ack_topic]["schema"] == "inspection_task_ack"
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

    request_binding = binding_for_topic(FIELD_DDS_CONTRACT.name, request_topic)
    ack_binding = binding_for_topic(FIELD_DDS_CONTRACT.name, ack_topic)
    assert request_binding.direction == "lingtu_to_endpoint"
    assert request_binding.idl_type == "lingtu.dds.InspectionTaskRequest"
    assert ack_binding.direction == "endpoint_to_lingtu"
    assert ack_binding.idl_type == "lingtu.dds.InspectionTaskAck"
    assert robot().binding_for("dds", request_topic) == {"qos": "command"}
    assert robot().binding_for("dds", ack_topic) == {"qos": "event"}


def test_inspection_task_event_is_an_honest_native_product_fact_stream() -> None:
    graph = load_runtime_graph()
    topic = TOPICS.inspection_task_event

    assert topic in graph.native_contract_topics
    assert graph.topic_contracts[topic] == {
        "role": "native_inspection_task_event",
        "frame": "map",
        "schema": "inspection_task_event",
        "producer": "native_nav_runtime",
        "consumers": ["host_bus"],
        "external_diagnostics_subscribable": True,
        "qos": "reliable_transient_local_keep_last_512",
        "semantics": "ordered_task_facts_with_boot_id_and_event_sequence",
        "port_bindings": [
            {
                "owner": "native_nav_runtime",
                "port": "inspection_task_event",
                "direction": "out",
                "boundary": "endpoint",
            },
            {
                "owner": "host_bus",
                "port": "inspection_task_event",
                "direction": "in",
                "boundary": "native",
            }
        ],
    }

    for endpoint in _native_endpoint_contracts():
        assert topic in endpoint["exposed_topics"]
        assert endpoint["inspection_task_event_stream"] == {
            "topic": topic,
            "ordering_cursor": ["boot_id", "event_sequence"],
            "terminal_truth": "native_stop_evidence_before_terminal_state",
        }

    inspection = graph.products["inspection"]
    assert topic in resolve_product_spec_contracts("inspection", inspection).topics

    binding = binding_for_topic(FIELD_DDS_CONTRACT.name, topic)
    assert binding.direction == "endpoint_to_lingtu"
    assert binding.idl_type == "lingtu.dds.InspectionTaskEvent"
    assert binding.frame_ids == ("map",)
    assert binding.required is True
    assert robot().binding_for("dds", topic) == {"qos": "event"}
