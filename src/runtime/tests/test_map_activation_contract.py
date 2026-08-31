"""Contracts for the native exact saved-map activation transaction."""

from __future__ import annotations

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]


def _read(relative: str) -> str:
    return (REPO / relative).read_text(encoding="utf-8")


def test_map_activation_idl_is_typed_and_identity_bearing() -> None:
    idl = _read("src/message/idl/messages.idl")
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
        "long long content_epoch;",
        "string frame_id;",
        "sequence<MapArtifactIdentity> artifacts;",
        "MapIdentity target;",
        "MapIdentity previous;",
        "MapIdentity active;",
    ):
        assert field in idl
    assert "string map_dir;" not in idl
    assert "version_id" not in idl
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

    cpp_topics = _read("src/message/cpp/topics.hpp")
    assert '"/maps/activation/request", "rt/maps/activation/request"' in cpp_topics
    assert '"/maps/activation/ack", "rt/maps/activation/ack"' in cpp_topics
    assert "/slam/map_activation" not in cpp_topics

def test_mapd_owns_exact_stage_restore_verify_and_stale_rollback() -> None:
    activation = _read("src/maps/cpp/mapd/activation.cpp")
    assert "SetActiveMapWhileLocked" in activation
    assert activation.count("CheckMapActivationWhileLocked") == 1
    assert activation.index("CheckMapActivationWhileLocked") < activation.index(
        "SetActiveMapWhileLocked"
    )
    assert "request.target.map_id, false, *map_lock, expected_active" in activation
    assert "ValidateArtifacts" not in activation
    assert 'Reject(request, "target_identity_mismatch")' in activation
    assert 'Reject(request, "previous_identity_mismatch")' in activation
    assert 'Reject(request, "stale_rollback")' in activation
    assert 'Reject(request, "active_identity_mismatch")' in activation

    main = _read("src/maps/cpp/mapd/main.cpp")
    assert "MapsServiceCore maps_service(" in main
    assert "ActivationCoordinator activation(maps_service.Store());" in main
    assert "MapQueryCore query_core(maps_service, &save_coordinator);" in main
    assert "activation.Execute(request)" in main
    assert '"request_id_conflict"' in main
    assert "SetActiveMap(request->map_id" not in main


def test_native_mapctl_is_built_for_map_activation() -> None:
    cmake = _read("src/maps/CMakeLists.txt")
    build = _read("scripts/build/build_mapd.sh")
    mapctl = _read("src/maps/cpp/tools/mapctl.cpp")
    assert "add_executable(lingtu_mapctl" in cmake
    assert 'OUTPUT_NAME "lingtu-mapctl"' in cmake
    assert "lingtu_mapctl" in build
    assert 'MAPCTL="${BUILD_DIR}/lingtu-mapctl"' in build
    assert "--offline" not in mapctl
    assert "identities.Execute(request)" not in mapctl
    assert 'operation == "prepare"' in mapctl
    assert "request = identities.PrepareStage(options.operand)" in mapctl
    assert 'PrintResult(prepared, operation_name)' in mapctl
    assert "CallMapd(options.domain_id, request, options.timeout)" in mapctl


def test_native_mapctl_is_required_in_field_releases() -> None:
    package_release = _read("scripts/deploy/package_native_release.sh")
    install_release = _read("scripts/deploy/install_native_release.sh")
    artifact = "build/maps/lingtu-mapctl"
    assert artifact in package_release
    assert f'${{PACKAGE_DIR}}/{artifact}' in install_release
    mapd_unit = _read("scripts/deploy/thunder/lt-maps.service")
    assert "LINGTU_MAPD_MAX_STRING_BYTES=4096" in mapd_unit


def test_native_save_map_prune_is_required_in_field_releases() -> None:
    build = _read("scripts/build/build_mapd.sh")
    package_release = _read("scripts/deploy/package_native_release.sh")
    mapd_unit = _read("scripts/deploy/thunder/lt-maps.service")
    artifact = "build/prune/prune"

    assert artifact in package_release
    assert "lingtu_maps_mapd_save_coordinator_test" in build
    assert "lingtu_maps_save_map_test" in build
    assert "packager accepted a mapd bundle without prune" in package_release
    assert f"native-release/{artifact}" in package_release
    assert f"LINGTU_PRUNE_BIN=/opt/lingtu/current/{artifact}" in mapd_unit


def test_saved_map_products_declare_activation_boundary() -> None:
    real_env = yaml.safe_load(_read("config/runtime_graph/envs/real.yaml"))
    expected_boundary = {
        "request": "/maps/activation/request",
        "ack": "/maps/activation/ack",
        "client_completion": "matching_business_ack_required",
        "operations": ["stage", "restore", "verify"],
        "request_identity_fields": ["target", "previous"],
        "stale_rollback_policy": "reject_unless_active_exactly_matches_target",
    }
    assert real_env["endpoints"]["contract"]["map_activation_boundary"] == expected_boundary
    sim_env = yaml.safe_load(_read("config/runtime_graph/envs/sim.yaml"))
    assert (
        sim_env["backends"]["mujoco"]["endpoints"]["contract"]["map_activation_boundary"]
        == expected_boundary
    )
    for product_name in ("nav", "inspection", "tracking"):
        product = yaml.safe_load(
            _read(f"config/runtime_graph/products/{product_name}.yaml")
        )
        assert product["requires_map"] is True

    explore = yaml.safe_load(_read("config/runtime_graph/products/explore.yaml"))
    assert explore["variants"]["live"]["requires_map"] is False
    assert explore["variants"]["map"]["requires_map"] is True
