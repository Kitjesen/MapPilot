from __future__ import annotations

from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.endpoints.dds.contracts import THUNDER_DDS_CONTRACT, binding_for_topic
from runtime.graph import load_runtime_graph, resolve_env_implementation
from runtime.route_contract.routes import robot
from runtime.runtime_interface import TOPICS


def _native_endpoint_contracts() -> tuple[dict[str, object], ...]:
    graph = load_runtime_graph()
    real = resolve_env_implementation("real", graph=graph)
    sim = resolve_env_implementation(
        "sim",
        graph=graph,
        env_config={"backend": "mujoco_native"},
    )
    return (
        dict(real["endpoints"]["contract"]),
        dict(sim["endpoints"]["contract"]),
    )


def test_taskless_inspection_dds_types_are_not_public_contracts() -> None:
    import message.dds_types as dds_types
    import message.dds_types.nav as nav_types
    import message.dds_types_generated as generated_types

    for type_name in (
        "InspectionCommandRequest",
        "InspectionCommandAck",
        "DDS_InspectionCommandRequest",
        "DDS_InspectionCommandAck",
    ):
        assert not hasattr(dds_types, type_name)
        assert not hasattr(nav_types, type_name)
        if not type_name.startswith("DDS_"):
            assert not hasattr(generated_types, type_name)


def test_inspection_task_event_is_available_from_the_public_dds_type_namespace() -> None:
    from message.dds_types import DDS_InspectionTaskEvent, InspectionTaskEvent

    assert DDS_InspectionTaskEvent is InspectionTaskEvent


def test_inspection_task_ingress_binds_caller_task_identity_end_to_end() -> None:
    from message.dds_types import (
        DDS_InspectionTaskAck,
        DDS_InspectionTaskRequest,
        InspectionTaskAck,
        InspectionTaskRequest,
    )

    assert DDS_InspectionTaskRequest is InspectionTaskRequest
    assert DDS_InspectionTaskAck is InspectionTaskAck

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

    product = graph.products["inspection"]
    assert product["inspection_task_boundary"] == {
        "request": request_topic,
        "ack": ack_topic,
        "status": TOPICS.inspection_status,
        "client_completion": "business_ack_required",
        "request_identity_fields": ["task_id", "request_id"],
        "response_identity_fields": ["task_id", "request_id"],
    }

    request_binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, request_topic)
    ack_binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, ack_topic)
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
    assert inspection["inspection_task_event_stream"] == {
        "topic": topic,
        "ordering_cursor": ["boot_id", "event_sequence"],
        "consumer_status": "host_bus_gateway_projection_enabled",
        "retention": "durable_gateway_projection",
        "task_journal_status": (
            "gateway_atomic_projection_enabled_native_restart_reconciliation_pending"
        ),
        "recording_status": (
            "native_mcap_task_verification_enabled_native_task_journal_pending"
        ),
    }
    assert inspection["recording_contract"] == {
        "activation": "explicit_product_operation",
        "native_preset": "inspection-evidence-v1",
        "identity": "task_id",
        "completion": (
            "unique_native_terminal_after_boot_matched_zero_output_and_driver_confirmation"
        ),
        "output": "atomic_mcap_plus_session_manifest",
        "replay_policy": (
            "sensors_only_recorded_control_and_task_facts_are_never_republished"
        ),
        "offline_verification": "player_dry_run_same_task_timeline_and_stop_evidence",
        "lifecycle_binding": "pending",
    }
    assert inspection["inspection_report_contract"] == {
        "path": "/api/v1/inspection/tasks/{task_id}/report",
        "source": (
            "native_task_events_plus_task_route_snapshot_plus_verified_evidence"
        ),
        "route_requirements": "minimal_snapshot_persisted_with_task_journal",
        "route_snapshot_required": True,
        "request_id_binding": "durable_before_native_command",
        "native_identity_conflict": "reject_event_and_require_review",
        "execution_truth": "native_task_event_only",
        "acceptance": "separate_from_execution_terminal",
        "incomplete_history": "unknown_review_required",
        "diagnostic_recording": "optional_not_an_acceptance_source",
    }

    binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, topic)
    assert binding.direction == "endpoint_to_lingtu"
    assert binding.idl_type == "lingtu.dds.InspectionTaskEvent"
    assert binding.frame_ids == ("map",)
    assert binding.required is True
    assert robot().binding_for("dds", topic) == {"qos": "event"}
