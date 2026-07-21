"""Regression checks for the YAML mirror of the runtime topic contract."""

from __future__ import annotations

from pathlib import Path

import yaml
from tools.validate.validate_topics import validate_topic_format_contract

from runtime.runtime_interface import MESSAGE_FORMATS

REPO_ROOT = Path(__file__).resolve().parents[3]
TOPIC_CONTRACT = REPO_ROOT / "config" / "topic_contract.yaml"


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
