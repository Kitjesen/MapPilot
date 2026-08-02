"""Regression checks for the YAML mirror of the runtime topic contract."""

from __future__ import annotations

from pathlib import Path

import yaml
from tools.validate.validate_topics import validate_topic_format_contract

from runtime.runtime_interface import MESSAGE_FORMATS, PRODUCT_DATA_SOURCE_BINDINGS

REPO_ROOT = Path(__file__).resolve().parents[3]
TOPIC_CONTRACT = REPO_ROOT / "config" / "topic_contract.yaml"
RUNTIME_TOPICS = REPO_ROOT / "config" / "runtime_graph" / "topics.yaml"


def _load_topic_contract() -> dict:
    return yaml.safe_load(TOPIC_CONTRACT.read_text(encoding="utf-8"))


def test_topic_contract_payload_formats_are_complete() -> None:
    contract = _load_topic_contract()

    assert validate_topic_format_contract(contract) == []


def test_yaml_data_format_catalog_mirrors_runtime_format_registry() -> None:
    contract = _load_topic_contract()
    yaml_formats = contract["data_formats"]

    assert set(yaml_formats) == set(MESSAGE_FORMATS)
    for name, runtime_format in MESSAGE_FORMATS.items():
        yaml_format = yaml_formats[name]
        assert yaml_format["ros_type"] == runtime_format.ros_type
        assert tuple(yaml_format.get("required_fields") or ()) == runtime_format.required_fields


def test_operator_motion_ack_is_not_sample_admission_contract() -> None:
    contract = _load_topic_contract()

    yaml_note = contract["data_formats"]["operator_motion_ack"]["note"]
    runtime_note = MESSAGE_FORMATS["operator_motion_ack"].note

    assert yaml_note == runtime_note
    assert "Claim, Hold, or Release only" in runtime_note
    assert "sample admission" not in runtime_note.lower()
    assert "Sample is best-effort intent" in runtime_note


def test_operator_motion_status_note_does_not_claim_python_dds_reader() -> None:
    contract = _load_topic_contract()

    yaml_note = contract["data_formats"]["operator_motion_status"]["note"]
    runtime_note = MESSAGE_FORMATS["operator_motion_status"].note

    assert yaml_note == runtime_note
    assert "No in-repo Python DDS reader is declared today" in runtime_note
    assert "native JSON status mirror" in runtime_note
    assert "external diagnostics may subscribe directly" in runtime_note


def test_live_maps_traversability_is_removed_without_touching_nav_authority() -> None:
    topic_contract = _load_topic_contract()
    runtime_topics = yaml.safe_load(RUNTIME_TOPICS.read_text(encoding="utf-8"))

    serialized_topic_contract = str(topic_contract)
    assert "/maps/traversability" not in serialized_topic_contract
    assert "/maps/traversability" not in runtime_topics["topics"]
    assert "/nav/traversability" in runtime_topics["topics"]
    assert (
        runtime_topics["topics"]["/nav/traversability"]["producer"]
        == "traversability_runtime"
    )


def test_public_runtime_contract_has_one_explore_product() -> None:
    contract = _load_topic_contract()

    assert "explore" in PRODUCT_DATA_SOURCE_BINDINGS
    assert "tare_explore" not in PRODUCT_DATA_SOURCE_BINDINGS
    assert "explore" in contract["product_data_sources"]
    assert "tare_explore" not in contract["product_data_sources"]
    assert PRODUCT_DATA_SOURCE_BINDINGS["explore"].mode == "real_robot_exploration"
    assert contract["product_data_sources"]["explore"]["mode"] == "real_robot_exploration"
