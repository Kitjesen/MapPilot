from __future__ import annotations

from pathlib import Path

from message.topics import TOPIC_SPECS, dds_topic_name, topic_spec
from runtime.runtime_interface import TOPICS

ROOT = Path(__file__).resolve().parents[3]


def test_message_package_has_no_python_dds_payload_types() -> None:
    message_dir = ROOT / "src/message"
    assert list((message_dir / "dds_types").glob("*.py")) == []
    assert not (message_dir / "dds.py").exists()
    assert "cyclonedds" not in (message_dir / "topics.py").read_text(encoding="utf-8")


def test_topic_metadata_matches_native_catalogue() -> None:
    header = (ROOT / "src/message/cpp/topics.hpp").read_text(encoding="utf-8")
    for spec in TOPIC_SPECS.values():
        assert f'"{spec.topic}"' in header
        assert f'"{dds_topic_name(spec.topic)}"' in header
        assert f'"{spec.idl_type}"' in header


def test_local_collision_topic_is_not_missing_from_host_metadata() -> None:
    spec = topic_spec(TOPICS.maps_local_collision)
    assert spec is not None
    assert spec.type_name == "MapCollisionLayer"
    assert spec.dds_topic == "rt/maps/local_collision"
