"""Public contract checks for the unified environment-map product surface."""

from __future__ import annotations

from pathlib import Path

import yaml

from runtime.runtime_interface import MESSAGE_FORMATS

REPO_ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def test_legacy_native_traversability_is_map_frame_only() -> None:
    """The current native writer and nav reader both operate in the map frame."""

    assert MESSAGE_FORMATS["traversability"].frame_role == "map"


def test_odom_local_risk_is_a_native_volatile_planning_input() -> None:
    """Keep the local control window separate from map authority and Host UI."""

    runtime_topics = yaml.safe_load(
        (REPO_ROOT / "config" / "runtime_graph" / "topics.yaml").read_text(encoding="utf-8")
    )
    local = runtime_topics["topics"]["/nav/local_traversability"]

    assert "/nav/local_traversability" in runtime_topics["native_contract_topics"]
    assert local == {
        "role": "rolling_local_traversability_grid",
        "frame": "odom",
        "schema": "occupancy_grid",
        "producer": "traversability_runtime",
        "field_producer": "native_traversability_endpoint",
        "single_writer_per_product": True,
        "qos": "reliable_volatile_keep_last_1_lifespan_500ms",
        "semantics": (
            "latest_only_odom_rolling_control_risk_for_native_local_planning_"
            "not_map_persistence_or_gateway_projection"
        ),
        "consumers": ["native_nav_runtime"],
        "port_bindings": [
            {
                "owner": "native_traversability_endpoint",
                "port": "local_traversability",
                "direction": "out",
                "boundary": "endpoint",
            },
            {
                "owner": "endpoint_supervisor",
                "port": "local_traversability",
                "direction": "in",
                "boundary": "endpoint",
            },
        ],
    }


def test_native_odom_local_risk_wiring_preserves_map_safety_authority() -> None:
    """Odom risk may guide local planning but must not replace map safety input."""

    topics = _read("src/message/cpp/topics.hpp")
    qos = _read("src/message/cpp/qos.hpp")
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
