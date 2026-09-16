"""Public contract checks for the unified environment-map product surface."""

from __future__ import annotations

from pathlib import Path

from lingtu.assembly.graph.loader import load_runtime_graph

REPO_ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def test_legacy_native_traversability_is_map_frame_only() -> None:
    """The current native writer and nav reader both operate in the map frame."""

    assert load_runtime_graph().topic_contracts["/nav/traversability"]["frame"] == "map"


def test_odom_local_risk_is_a_native_volatile_planning_input() -> None:
    """Keep the local control window separate from map authority and Host UI."""

    graph = load_runtime_graph()
    local = graph.topic_contracts["/nav/local_traversability"]

    assert "/nav/local_traversability" in graph.native_contract_topics
    assert local["message_type"] == "lingtu.dds.OccupancyGrid"
    assert local["qos_profile"] == "LocalRiskGrid"
    assert local["frame"] == "odom"
    assert local["producer"] == "traversability_runtime"
    assert local["consumers"] == ["native_nav_runtime"]
    assert local["single_writer_per_product"] is True


def test_native_odom_local_risk_wiring_preserves_map_safety_authority() -> None:
    """Odom risk may guide local planning but must not replace map safety input."""

    topics = _read("src/message/generated/topics.hpp")
    qos = _read("src/transport/dds/qos.hpp")
    producer = _read("src/nav/cpp/endpoint/traversability/main.cpp")
    projector = _read("src/nav/cpp/endpoint/nav/input/map.cpp")
    dds = _read("src/nav/cpp/endpoint/nav/dds/runtime.cpp")
    loop = _read("src/nav/cpp/endpoint/nav/runtime/loop.cpp")
    navd = _read("src/nav/cpp/endpoint/nav/main.cpp")
    decoder = _read("src/nav/cpp/endpoint/nav/dds/codec.cpp")

    assert '"/nav/local_traversability", "rt/nav/local_traversability"' in topics
    assert "QosProfile::LocalRiskGrid" in qos
    assert "DDS_DURABILITY_VOLATILE" in qos
    assert "DDS_MSECS(500)" in qos
    assert 'toOccupancyMessage(grid, source_stamp_s, "odom")' in producer
    assert "projectRollingRiskGridToOdom" in producer
    assert "projectLocalTraversability" in projector
    assert 'copyGridSample(message, "odom")' in dds
    assert "inputs.apply(dds.takeSensors" in loop
    assert "state.odom_requires_tf" in navd
    assert "ExecutionMode::Route" in navd
    assert "local_traversability" in navd
    assert "kLocalTraversabilityMaxAgeS = 0.5" in navd
    assert "std::min(cfg.traversability_max_age_s, kLocalTraversabilityMaxAgeS)" in navd
    assert "decodeGrid" in decoder and 'if (frame != "map")' in decoder
    assert "decodeLocalRiskGrid" in decoder and 'if (frame != "odom")' in decoder
