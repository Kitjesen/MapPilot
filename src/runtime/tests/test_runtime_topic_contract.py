"""Regression checks for the canonical runtime topic contract."""

from pathlib import Path

import yaml
from tools.validate.validate_topics import validate_topic_format_contract

from runtime.runtime_interface import (
    ADAPTER_TOPIC_ALIASES,
    MESSAGE_FORMATS,
    PRODUCT_DATA_SOURCE_BINDINGS,
    runtime_contract_manifest,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME_TOPICS = REPO_ROOT / "config" / "runtime_graph" / "topics.yaml"


def test_runtime_topic_payload_formats_are_complete() -> None:
    assert validate_topic_format_contract(runtime_contract_manifest()) == []


def test_operator_motion_notes_describe_the_native_contract() -> None:
    ack_note = MESSAGE_FORMATS["operator_motion_ack"].note
    status_note = MESSAGE_FORMATS["operator_motion_status"].note

    assert "Claim, Hold, or Release only" in ack_note
    assert "sample admission" not in ack_note.lower()
    assert "Sample is best-effort intent" in ack_note
    assert "native JSON status mirror" in status_note
    assert "external diagnostics may subscribe directly" in status_note


def test_runtime_graph_owns_traversability_endpoint_metadata() -> None:
    runtime_topics = yaml.safe_load(RUNTIME_TOPICS.read_text(encoding="utf-8"))

    assert "/maps/traversability" not in runtime_topics["topics"]
    traversability = runtime_topics["topics"]["/nav/traversability"]
    assert traversability["producer"] == "traversability_runtime"
    assert traversability["frame"] == "map"
    assert MESSAGE_FORMATS["traversability"].frame_role == "map"
    assert MESSAGE_FORMATS["local_traversability"].frame_role == "odom"


def test_public_runtime_contract_has_one_explore_product() -> None:
    assert "explore" in PRODUCT_DATA_SOURCE_BINDINGS
    assert "tare_explore" not in PRODUCT_DATA_SOURCE_BINDINGS
    assert PRODUCT_DATA_SOURCE_BINDINGS["explore"].mode == "real_robot_exploration"


def test_retired_slam_backends_have_no_adapter_alias_contract() -> None:
    for backend in ("pgo", "localizer", "pointlio"):
        assert backend not in ADAPTER_TOPIC_ALIASES
