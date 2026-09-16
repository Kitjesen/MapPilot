"""Regression checks for the canonical runtime topic contract."""

from tools.validate.validate_topics import validate_topic_format_contract

from diagnostics.runtime_contract import runtime_contract_manifest
from lingtu.assembly.graph.loader import load_runtime_graph
from runtime.adapters.topics import ADAPTER_TOPIC_ALIASES


def test_runtime_topic_payload_formats_are_complete() -> None:
    assert validate_topic_format_contract(runtime_contract_manifest()) == []


def test_operator_motion_control_actions_are_generated_from_idl() -> None:
    from message.generated.enums import OperatorMotionAction
    from message.generated.schema import MESSAGE_FIELDS

    assert {action.name: action.value for action in OperatorMotionAction} == {
        "CLAIM": 1, "RELEASE": 2, "HOLD": 3,
    }
    assert "action" in MESSAGE_FIELDS["lingtu.dds.OperatorMotionControl"]
    assert "accepted" in MESSAGE_FIELDS["lingtu.dds.OperatorMotionAck"]


def test_runtime_graph_owns_traversability_endpoint_metadata() -> None:
    runtime_topics = load_runtime_graph().topic_contracts

    assert "/maps/traversability" not in runtime_topics
    traversability = runtime_topics["/nav/traversability"]
    assert traversability["producer"] == "traversability_runtime"
    assert traversability["frame"] == "map"
    assert traversability["frame"] == "map"
    assert runtime_topics["/nav/local_traversability"]["frame"] == "odom"


def test_public_runtime_contract_has_one_explore_product() -> None:
    bindings = runtime_contract_manifest()["product_data_sources"]
    assert "explore" in bindings
    assert "tare_explore" not in bindings
    assert bindings["explore"]["mode"] == load_runtime_graph().products["explore"]["session_mode"]


def test_retired_slam_backends_have_no_adapter_alias_contract() -> None:
    for backend in ("pgo", "localizer", "pointlio"):
        assert backend not in ADAPTER_TOPIC_ALIASES
