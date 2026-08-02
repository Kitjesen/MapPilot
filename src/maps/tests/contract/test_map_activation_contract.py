"""Contracts for the native exact saved-map activation transaction."""

from __future__ import annotations

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]


def _read(relative: str) -> str:
    return (REPO / relative).read_text(encoding="utf-8")


def test_map_activation_idl_is_typed_and_identity_bearing() -> None:
    idl = _read("src/message/idl/lingtu_slam.idl")
    for operation in (
        "MAP_ACTIVATION_STAGE",
        "MAP_ACTIVATION_RESTORE",
        "MAP_ACTIVATION_VERIFY",
    ):
        assert operation in idl
    for type_name in (
        "struct MapArtifactIdentity",
        "struct MapIdentity",
        "struct MapActivationRequest",
        "struct MapActivationAck",
    ):
        assert type_name in idl
    for field in (
        "string version_id;",
        "string frame_id;",
        "string map_dir;",
        "sequence<MapArtifactIdentity> artifacts;",
        "MapIdentity target;",
        "MapIdentity previous;",
        "MapIdentity active;",
    ):
        assert field in idl
    assert "string map_id;\n  boolean strict;" not in idl


def test_map_activation_topics_and_qos_are_canonical() -> None:
    topics = yaml.safe_load(_read("config/runtime_graph/topics.yaml"))
    request = topics["topics"]["/maps/activation/request"]
    ack = topics["topics"]["/maps/activation/ack"]
    assert request["qos"] == "reliable_volatile_keep_last_32"
    assert request["producer"] == "native_map_control_client"
    assert request["consumers"] == ["native_maps_runtime"]
    assert ack["qos"] == "reliable_transient_local_keep_last_64"
    assert ack["producer"] == "native_maps_runtime"
    assert ack["consumers"] == ["native_map_control_client"]
    assert "/maps/activation/request" in topics["native_contract_topics"]
    assert "/maps/activation/ack" in topics["native_contract_topics"]

    cpp_topics = _read("src/message/cpp/dds_topics.hpp")
    assert '"/maps/activation/request", "rt/maps/activation/request"' in cpp_topics
    assert '"/maps/activation/ack", "rt/maps/activation/ack"' in cpp_topics
    assert "/slam/map_activation" not in cpp_topics

    qos = yaml.safe_load(_read("config/qos_profiles.yaml"))["profiles"]
    assert qos["map_activation_request"] == {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 32,
        "topics": [
            "/maps/activation/request",
            "rt/maps/activation/request",
        ],
    }
    assert qos["map_activation_ack"] == {
        "reliability": "reliable",
        "durability": "transient_local",
        "history": "keep_last",
        "depth": 64,
        "topics": [
            "/maps/activation/ack",
            "rt/maps/activation/ack",
        ],
    }


def test_mapd_owns_exact_stage_restore_verify_and_stale_rollback() -> None:
    activation = _read("src/maps/cpp/mapd/activation.cpp")
    assert "SetActiveMapWhileLocked" in activation
    assert "ValidateArtifacts" in activation
    assert 'Reject(request, "target_identity_mismatch")' in activation
    assert 'Reject(request, "previous_identity_mismatch")' in activation
    assert 'Reject(request, "stale_rollback")' in activation
    assert 'Reject(request, "active_identity_mismatch")' in activation

    main = _read("src/maps/cpp/mapd/main.cpp")
    assert "ActivationCoordinator activation(map_store);" in main
    assert "activation.Execute(request)" in main
    assert '"request_id_conflict"' in main
    assert "SetActiveMap(request->map_id" not in main


def test_native_mapctl_is_built_and_uses_dds_only_for_mutation() -> None:
    source = _read("src/maps/cpp/tools/mapctl.cpp")
    cmake = _read("src/maps/CMakeLists.txt")
    build = _read("scripts/build/build_mapd.sh")
    assert "CallMapd(" in source
    assert "dds_write(writer, &message)" in source
    assert "lingtu.map_activation.v2" in source
    assert "EncodeActivationToken" in source
    assert "lingtu-mapctl verify ACTIVATION_TOKEN" in source
    assert "activation_token" in source
    assert "restore_token" not in source
    assert "lingtu-mapctl verify MAP_ID" not in source
    assert "result.operation != request.operation" in source
    assert "result.target != request.target" in source
    assert "result.previous != request.previous" in source
    assert "SetActiveMap(" not in source
    assert "ClearActiveMap(" not in source
    assert "add_executable(lingtu_mapctl" in cmake
    assert 'OUTPUT_NAME "lingtu-mapctl"' in cmake
    assert "lingtu_mapctl" in build
    assert 'MAPCTL="${BUILD_DIR}/lingtu-mapctl"' in build


def test_native_mapctl_is_required_in_field_releases() -> None:
    cut_release = _read("scripts/deploy/cut_release.sh")
    package_release = _read("scripts/deploy/package_native_release.sh")
    install_release = _read("scripts/deploy/install_native_release.sh")
    artifact = "build/maps/lingtu-mapctl"
    assert artifact in cut_release
    assert artifact in package_release
    assert f'${{PACKAGE_DIR}}/{artifact}' in install_release
    mapd_unit = _read("scripts/deploy/thunder/mapd.service")
    assert "LINGTU_MAPD_MAX_STRING_BYTES=4096" in mapd_unit


def test_saved_map_products_declare_activation_boundary() -> None:
    real_env = yaml.safe_load(_read("config/runtime_graph/envs/real.yaml"))
    assert real_env["endpoints"]["contract"]["map_activation_boundary"] == {
        "request": "/maps/activation/request",
        "ack": "/maps/activation/ack",
        "client_completion": "matching_business_ack_required",
        "operations": ["STAGE", "RESTORE", "VERIFY"],
        "request_identity_fields": ["target", "previous"],
        "stale_rollback_policy": "reject_unless_active_exactly_matches_target",
    }
    for product_name in ("nav", "inspection", "tracking"):
        product = yaml.safe_load(
            _read(f"config/runtime_graph/products/{product_name}.yaml")
        )
        assert product["requires_map"] is True

    explore = yaml.safe_load(_read("config/runtime_graph/products/explore.yaml"))
    assert explore["variants"]["live"]["requires_map"] is False
    assert explore["variants"]["map"]["requires_map"] is True
