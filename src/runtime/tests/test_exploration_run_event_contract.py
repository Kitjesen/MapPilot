from __future__ import annotations

import pytest

from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.endpoints.dds.contracts import THUNDER_DDS_CONTRACT, binding_for_topic
from runtime.graph import load_runtime_graph, resolve_env_implementation
from runtime.msgs import (
    ExplorationRunEvent,
    ExplorationRunEventKind,
    ExplorationRunState,
)
from runtime.route_contract.routes import robot
from runtime.runtime_interface import MESSAGE_FORMATS, TOPICS

RUN_ID = "01K1M9S4FX27T8XMY6QJNBAV3W"


def _event(**overrides: object) -> ExplorationRunEvent:
    values: dict[str, object] = {
        "ts": 42.5,
        "frame_id": "map",
        "boot_id": "explore-boot-7",
        "event_sequence": 3,
        "kind": int(ExplorationRunEventKind.STATE_CHANGED),
        "exploration_run_id": RUN_ID,
        "start_request_id": "01K1M9S4FX27T8XMY6QJNBAV3X",
        "command_request_id": "01K1M9S4FX27T8XMY6QJNBAV3Y",
        "product_session_id": "product-session-7",
        "state": int(ExplorationRunState.RUNNING),
        "route": "map",
        "map_id": "yard-a",
        "map_version": 4,
        "artifact_hash": "a" * 64,
        "reason": "goal_active",
        "motion_stop_confirmed": False,
        "motion_stop_reason": "",
    }
    values.update(overrides)
    return ExplorationRunEvent(**values)


def test_exploration_run_event_preserves_distinct_lifecycle_identities() -> None:
    event = _event()

    assert event.exploration_run_id != event.start_request_id
    assert event.terminal is False
    assert event.kind_name == "STATE_CHANGED"
    assert event.state_name == "RUNNING"
    assert event.to_dict() == {
        "timestamp_s": 42.5,
        "frame_id": "map",
        "boot_id": "explore-boot-7",
        "event_sequence": 3,
        "kind": 2,
        "kind_name": "STATE_CHANGED",
        "exploration_run_id": RUN_ID,
        "start_request_id": "01K1M9S4FX27T8XMY6QJNBAV3X",
        "command_request_id": "01K1M9S4FX27T8XMY6QJNBAV3Y",
        "product_session_id": "product-session-7",
        "state": 2,
        "state_name": "RUNNING",
        "terminal": False,
        "route": "map",
        "map_id": "yard-a",
        "map_version": 4,
        "artifact_hash": "a" * 64,
        "reason": "goal_active",
        "motion_stop_confirmed": False,
        "motion_stop_reason": "",
    }


def test_exploration_run_terminal_requires_native_motion_stop_confirmation() -> None:
    with pytest.raises(ValueError, match="motion_stop_confirmed"):
        _event(state=int(ExplorationRunState.COMPLETED))

    event = _event(
        state=int(ExplorationRunState.COMPLETED),
        motion_stop_confirmed=True,
        motion_stop_reason="driver_ack_and_odometry_still",
    )
    assert event.terminal is True


def test_exploration_run_event_rejects_invalid_identity_route_and_stop_failure() -> None:
    with pytest.raises(ValueError, match="must be distinct"):
        _event(exploration_run_id="01K1M9S4FX27T8XMY6QJNBAV3X")
    with pytest.raises(ValueError, match="route"):
        _event(route="legacy_tare")
    with pytest.raises(ValueError, match="map identity"):
        _event(map_id="", map_version=0, artifact_hash="")
    with pytest.raises(ValueError, match="STOP_CONFIRMATION_FAILED"):
        _event(
            kind=int(ExplorationRunEventKind.STOP_CONFIRMATION_FAILED),
            motion_stop_confirmed=True,
            motion_stop_reason="unexpected",
        )
    with pytest.raises(ValueError, match="ADMITTED kind and state"):
        _event(kind=int(ExplorationRunEventKind.ADMITTED))
    with pytest.raises(ValueError, match="cannot claim confirmed parking"):
        _event(
            state=int(ExplorationRunState.PAUSING),
            motion_stop_confirmed=True,
            motion_stop_reason="premature",
        )
    with pytest.raises(ValueError, match="unconfirmed motion stop"):
        _event(motion_stop_reason="stale_stop_reason")


def test_exploration_run_event_is_required_by_both_explore_variants() -> None:
    graph = load_runtime_graph()
    topic = TOPICS.exploration_run_event

    for variant in ("live", "map"):
        contract = resolve_product_spec_contracts(
            "explore",
            graph.products["explore"],
            product_variant=variant,
        )
        assert topic in contract.topics

    assert graph.products["explore"]["exploration_run_event_stream"] == {
        "topic": topic,
        "ordering_cursor": ["boot_id", "event_sequence"],
        "terminal_truth": "native_motion_stop_confirmation_before_terminal",
    }


def test_exploration_run_event_is_one_native_writer_to_host_bus_stream() -> None:
    graph = load_runtime_graph()
    topic = TOPICS.exploration_run_event

    assert topic in graph.native_contract_topics
    assert graph.topic_contracts[topic] == {
        "role": "native_exploration_run_event",
        "frame": "map",
        "schema": "exploration_run_event",
        "producer": "native_explore_runtime",
        "consumers": ["host_bus"],
        "external_diagnostics_subscribable": True,
        "qos": "reliable_transient_local_keep_last_512",
        "semantics": "ordered_exploration_run_facts_with_native_stop_evidence",
        "port_bindings": [
            {
                "owner": "native_explore_runtime",
                "port": "exploration_run_event",
                "direction": "out",
                "boundary": "endpoint",
            },
            {
                "owner": "host_bus",
                "port": "exploration_run_event",
                "direction": "in",
                "boundary": "native",
            },
        ],
    }

    real = resolve_env_implementation("real", graph=graph)
    sim = resolve_env_implementation(
        "sim",
        graph=graph,
        env_config={"backend": "mujoco_native"},
    )
    for implementation in (real, sim):
        endpoint = implementation["endpoints"]["contract"]
        assert topic in endpoint["exposed_topics"]
        assert endpoint["exploration_run_event_stream"] == {
            "topic": topic,
            "ordering_cursor": ["boot_id", "event_sequence"],
            "terminal_truth": "native_motion_stop_confirmation_before_terminal",
        }

    binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, topic)
    assert binding.direction == "endpoint_to_lingtu"
    assert binding.idl_type == "lingtu.dds.ExplorationRunEvent"
    assert binding.frame_ids == ("map",)
    assert binding.required is True
    assert robot().binding_for("dds", topic) == {"qos": "event"}


def test_exploration_run_event_runtime_format_is_complete() -> None:
    assert MESSAGE_FORMATS["exploration_run_event"].required_fields == (
        "timestamp_s",
        "frame_id",
        "boot_id",
        "event_sequence",
        "kind",
        "exploration_run_id",
        "start_request_id",
        "command_request_id",
        "product_session_id",
        "state",
        "route",
        "map_id",
        "map_version",
        "artifact_hash",
        "reason",
        "motion_stop_confirmed",
        "motion_stop_reason",
    )


def test_exploration_run_event_has_a_resolvable_public_dds_type() -> None:
    from message.dds import TOPIC_SPECS
    from message.dds_types import DDS_ExplorationRunEvent

    spec = TOPIC_SPECS[TOPICS.exploration_run_event]
    assert spec.dds_topic == "rt/nav/exploration/run/event"
    assert spec.idl_type == "lingtu.dds.ExplorationRunEvent"
    assert spec.cpp_type == "lingtu::dds::ExplorationRunEvent"
    assert spec.dds_type() is DDS_ExplorationRunEvent
