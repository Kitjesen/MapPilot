from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import sim.scripts.mujoco.teleop_avoid_native_acceptance as acceptance
from sim.scripts.mujoco.teleop_avoid_native_acceptance import (
    FIELD_TELEOP_AVOID_PROFILE,
    MAPD_RUNTIME_PROFILE,
    SIMULATION_POSTURE_GATE,
    TERRAIN_SCENE_CONTRACT,
    ResilientTeleopProcess,
    _binary_source_provenance,
    _redact_command_args,
    _requested_scenarios,
    _wait_for_policy_driving,
    _wait_for_runtime_ready,
    assign_motion_phases,
    build_execution_plan,
    build_odom_prior_diagnostic_config,
    build_parser,
    build_scene_variant,
    cleanup_owned_pid_file,
    contains_ordered_transition,
    continuous_teleop_exit_blocker,
    evaluate_case,
    evaluate_operator_motion_lifecycle,
    evaluate_simulation_posture,
    mapd_status_evidence,
    parse_operator_motion_events,
    prepare_runtime,
    project_motion_timestamp,
    read_native_gate_transitions,
    reclaim_prior_case_processes,
    reset_case_artifacts,
    run,
    signal_managed_process,
    terrain_scene_forward_probe_attribution,
    typed_teleop_delivery_blocker,
)


def test_redact_command_args_scrubs_external_arm_token_without_mutating_source() -> None:
    raw_token = "case-secret-token"
    command = [
        "python",
        "native_dds_sensors.py",
        "--external-arm-token",
        raw_token,
        "--domain-id",
        "44",
    ]

    redacted = _redact_command_args(command)

    assert command[3] == raw_token
    assert redacted[3] == "<redacted>"
    assert raw_token not in json.dumps(redacted)


def test_binary_provenance_includes_native_client_shared_library(tmp_path) -> None:
    control = tmp_path / "lingtu_nav_control"
    client_library = tmp_path / "liblingtu_nav_client.so"
    control.write_bytes(b"control")
    client_library.write_bytes(b"client")

    provenance, blockers = _binary_source_provenance({"navigation_control": control})

    assert blockers == []
    dependency = provenance["navigation_control"]["runtime_dependencies"]
    assert dependency["lingtu_nav_client"]["path"] == str(client_library)
    assert len(dependency["lingtu_nav_client"]["sha256"]) == 64


def test_binary_provenance_uses_current_native_source_tree(tmp_path: Path) -> None:
    navigation = tmp_path / "navd"
    navigation.write_bytes(b"navigation")

    provenance, blockers = _binary_source_provenance({"navigation": navigation})

    item = provenance["navigation"]
    normalized_specs = [Path(spec).as_posix() for spec in item["source_specs"]]
    assert any(spec.endswith("src/nav/cpp/endpoint") for spec in normalized_specs)
    assert any(
        spec.endswith("src/nav/cpp/cmake/NavCoreTargets.cmake")
        for spec in normalized_specs
    )
    assert any(
        spec.endswith("src/nav/inspection/CMakeLists.txt")
        for spec in normalized_specs
    )
    assert any(spec.endswith("src/maps/CMakeLists.txt") for spec in normalized_specs)
    retired_endpoint_root = "src/nav/services/" + "endpoint/cpp"
    assert all(retired_endpoint_root not in spec for spec in normalized_specs)
    assert "missing_source_specs" not in item
    assert not any(blocker.startswith("native_source_spec_missing:navigation:") for blocker in blockers)


def test_binary_provenance_includes_slam_build_contract(tmp_path: Path) -> None:
    slam = tmp_path / "slamd"
    slam.write_bytes(b"slam")

    provenance, _blockers = _binary_source_provenance({"slam": slam})

    normalized_specs = [
        Path(spec).as_posix() for spec in provenance["slam"]["source_specs"]
    ]
    assert any(
        spec.endswith("src/localization/slam/cpp/CMakeLists.txt")
        for spec in normalized_specs
    )
    assert any(
        spec.endswith("third_party/research_localization/small_gicp/include/small_gicp")
        for spec in normalized_specs
    )


def test_binary_provenance_tracks_vendored_small_gicp_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    source_files = (
        fake_root / "src/localization/slam/cpp/slam.cpp",
        fake_root / "src/localization/slam/cpp/CMakeLists.txt",
        fake_root / "src/localization/fastlio2/src/fastlio.cpp",
        fake_root / "src/localization/localizer/src/localizers/icp_localizer.cpp",
        fake_root / "src/maps/include/lingtu/maps/c_api/semantic_occupancy.h",
        fake_root / "src/message/cpp/snapshot_file.hpp",
        fake_root / "src/message/idl/lingtu_slam.idl",
        fake_root / "src/message/cpp/CMakeLists.txt",
        fake_root / "src/message/cpp/dds_topics.hpp",
        fake_root / "src/message/cpp/dds_qos_profiles.hpp",
    )
    for source in source_files:
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("source", encoding="utf-8")
    small_gicp_header = (
        fake_root
        / "third_party/research_localization/small_gicp/include/small_gicp/pcl/pcl_registration.hpp"
    )
    small_gicp_header.parent.mkdir(parents=True)
    small_gicp_header.write_text("header", encoding="utf-8")
    future_ns = time.time_ns() + 1_000_000_000
    os.utime(small_gicp_header, ns=(future_ns, future_ns))
    slam = tmp_path / "slamd"
    slam.write_bytes(b"slam")
    monkeypatch.setattr(acceptance, "ROOT", fake_root)

    provenance, blockers = acceptance._binary_source_provenance({"slam": slam})

    assert provenance["slam"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:slam" in blockers


def test_binary_provenance_covers_transitive_native_build_contracts(
    tmp_path: Path,
) -> None:
    binaries = {
        "mapd": tmp_path / "mapd",
        "traversability": tmp_path / "lingtu_traversability_dds",
        "navigation_control": tmp_path / "lingtu_nav_control",
        "sensor_publisher": tmp_path / "livox_sdk2_stream",
    }
    for binary in binaries.values():
        binary.write_bytes(binary.name.encode("utf-8"))
    (tmp_path / "liblingtu_nav_client.so").write_bytes(b"nav-client")

    provenance, blockers = _binary_source_provenance(binaries)

    expected_suffixes = {
        "mapd": (
            "src/maps/CMakeLists.txt",
            "src/maps/cpp",
            "src/maps/include/lingtu/maps",
        ),
        "traversability": (
            "src/maps/CMakeLists.txt",
            "src/nav/cpp/cmake/NavCoreTargets.cmake",
            "src/message/cpp/snapshot_file.hpp",
        ),
        "navigation_control": (
            "src/message/cpp/exploration_command.hpp",
            "src/message/cpp/operator_motion.hpp",
        ),
        "sensor_publisher": (
            "src/drivers/real/lidar/sdk2_stream/CMakeLists.txt",
            "src/drivers/real/lidar/native",
        ),
    }
    for artifact, suffixes in expected_suffixes.items():
        normalized_specs = [
            Path(spec).as_posix() for spec in provenance[artifact]["source_specs"]
        ]
        assert all(
            any(spec.endswith(suffix) for spec in normalized_specs)
            for suffix in suffixes
        )
        assert "missing_source_specs" not in provenance[artifact]
        assert not any(
            blocker.startswith(f"native_source_spec_missing:{artifact}:")
            for blocker in blockers
        )


def test_default_manifest_matches_canonical_map_free_product() -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    manifest = native._load_manifest(acceptance.DEFAULT_MANIFEST)
    evidence = acceptance._teleop_product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["canonical"] == {
        "product": "teleop_avoid",
        "source": "config/runtime_graph/products/teleop_avoid.yaml",
        "native_control_mode": "teleop_avoid",
        "slam_mode": "mapping",
        "requires_map": False,
    }
    assert manifest["slam_runtime"]["mode"] == "mapping"
    assert manifest["asset_builder"]["kind"] == "scene_only"
    assert "extends" not in manifest
    assert "map_dir" not in manifest
    assert "map_files" not in manifest
    assert "goal" not in manifest


def test_scene_only_preparation_generates_no_saved_map(tmp_path: Path) -> None:
    manifest = {
        "world": "",
        "asset_builder": {
            "kind": "scene_only",
            "scene_preset": "corridor",
            "length": 3.0,
            "width": 1.8,
        },
    }

    preparation = acceptance._prepare_teleop_scene_asset(manifest, tmp_path)

    assert preparation["ok"] is True
    assert preparation["reason"] == "teleop_scene_prepared"
    scene = Path(preparation["scene_xml"])
    assert scene.is_file()
    assert manifest["world"] == str(scene)
    assert not (tmp_path / "prepared_assets" / "same_source_map").exists()
    assert "map_dir" not in manifest
    assert "map_files" not in manifest


def test_product_contract_rejects_localization_and_saved_map_inputs() -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    manifest = native._load_manifest(acceptance.DEFAULT_MANIFEST)
    manifest["slam_runtime"]["mode"] = "localization"
    manifest["map_dir"] = "maps/legacy"
    manifest["map_files"] = {"slam": "map.pcd"}

    evidence = acceptance._teleop_product_contract_evidence(manifest)

    assert evidence["ok"] is False
    assert "teleop_slam_runtime_mode_not_mapping" in evidence["blockers"]
    assert "teleop_saved_map_contract_forbidden" in evidence["blockers"]


def test_generic_navigation_preflight_stays_map_required() -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    _binaries, map_paths, blockers, _provenance = native._preflight({})
    _free_binaries, free_paths, free_blockers, free_provenance = (
        native._preflight_map_free({})
    )

    assert any(blocker.startswith("map_artifact_missing:") for blocker in blockers)
    assert {"map_dir", "slam", "planner", "metadata"} <= set(map_paths)
    assert not any(
        blocker.startswith("map_artifact_missing:") for blocker in free_blockers
    )
    assert not {"map_dir", "slam", "planner", "metadata"} & set(free_paths)
    assert free_provenance["map_contract"]["required"] is False
def test_binary_provenance_fails_closed_when_source_specs_move(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    missing_root = tmp_path / "missing-source-root"
    missing_root.mkdir()
    navigation = tmp_path / "navd"
    navigation.write_bytes(b"navigation")
    monkeypatch.setattr(acceptance, "ROOT", missing_root)

    provenance, blockers = acceptance._binary_source_provenance(
        {"navigation": navigation}
    )

    item = provenance["navigation"]
    assert item["missing_source_specs"]
    assert "newer_than_sources" not in item
    assert any(
        blocker.startswith("native_source_spec_missing:navigation:")
        for blocker in blockers
    )




def test_binary_provenance_tracks_c_abi_header_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    nav_cpp = fake_root / "src" / "nav" / "cpp"
    client_dir = nav_cpp / "client"
    endpoint_dir = nav_cpp / "endpoint"
    common_files = (
        fake_root / "src" / "message" / "idl" / "lingtu_slam.idl",
        fake_root / "src" / "message" / "cpp" / "CMakeLists.txt",
        fake_root / "src" / "message" / "cpp" / "dds_topics.hpp",
        fake_root / "src" / "message" / "cpp" / "dds_qos_profiles.hpp",
        fake_root / "src" / "message" / "cpp" / "exploration_command.hpp",
        fake_root / "src" / "message" / "cpp" / "inspection_command.hpp",
        fake_root / "src" / "message" / "cpp" / "navigation_command.hpp",
        fake_root / "src" / "message" / "cpp" / "operator_motion.hpp",
    )
    sources = (
        endpoint_dir / "motion" / "nav_control.cpp",
        nav_cpp / "CMakeLists.txt",
        endpoint_dir / "CMakeLists.txt",
        *common_files,
    )
    client_dir.mkdir(parents=True)
    for source in sources:
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("source", encoding="utf-8")

    control = tmp_path / "lingtu_nav_control"
    client_library = tmp_path / "liblingtu_nav_client.so"
    control.write_bytes(b"control")
    client_library.write_bytes(b"client")
    c_header = client_dir / "client_c.h"
    c_header.write_text("abi", encoding="utf-8")
    future_ns = time.time_ns() + 1_000_000_000
    os.utime(c_header, ns=(future_ns, future_ns))
    monkeypatch.setattr(acceptance, "ROOT", fake_root)

    provenance, blockers = acceptance._binary_source_provenance(
        {"navigation_control": control}
    )

    assert provenance["navigation_control"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:navigation_control" in blockers
    assert "native_runtime_dependency_stale:navigation_control" in blockers


def test_binary_provenance_tracks_sensor_publisher_idl_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    publisher_source = (
        fake_root / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "main.cpp"
    )
    publisher_cmake = publisher_source.with_name("CMakeLists.txt")
    common_files = (
        fake_root / "src" / "message" / "idl" / "lingtu_slam.idl",
        fake_root / "src" / "message" / "cpp" / "CMakeLists.txt",
        fake_root / "src" / "message" / "cpp" / "dds_topics.hpp",
        fake_root / "src" / "message" / "cpp" / "dds_qos_profiles.hpp",
        fake_root / "src" / "drivers" / "real" / "lidar" / "native" / "dds_module.cpp",
    )
    for source in (publisher_source, publisher_cmake, *common_files):
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("source", encoding="utf-8")

    publisher = tmp_path / "livox_sdk2_stream"
    publisher.write_bytes(b"publisher")
    future_ns = time.time_ns() + 1_000_000_000
    os.utime(common_files[0], ns=(future_ns, future_ns))
    monkeypatch.setattr(acceptance, "ROOT", fake_root)

    provenance, blockers = acceptance._binary_source_provenance(
        {"sensor_publisher": publisher}
    )

    assert provenance["sensor_publisher"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:sensor_publisher" in blockers


def test_binary_provenance_tracks_cmd_vel_tap_idl_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    sources = (
        fake_root / "sim" / "native_dds" / "cmd_vel_tap.cpp",
        fake_root / "sim" / "native_dds" / "CMakeLists.txt",
        fake_root / "src" / "message" / "idl" / "lingtu_slam.idl",
        fake_root / "src" / "message" / "cpp" / "CMakeLists.txt",
        fake_root / "src" / "message" / "cpp" / "dds_topics.hpp",
        fake_root / "src" / "message" / "cpp" / "dds_qos_profiles.hpp",
    )
    for source in sources:
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("source", encoding="utf-8")

    tap = tmp_path / "lingtu_mujoco_cmd_vel_tap"
    tap.write_bytes(b"tap")
    future_ns = time.time_ns() + 1_000_000_000
    os.utime(sources[2], ns=(future_ns, future_ns))
    monkeypatch.setattr(acceptance, "ROOT", fake_root)

    provenance, blockers = acceptance._binary_source_provenance(
        {"cmd_vel_tap": tap}
    )

    assert provenance["cmd_vel_tap"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:cmd_vel_tap" in blockers


def test_harness_field_profile_matches_deployed_service_contract() -> None:
    nav_service = Path("scripts/deploy/thunder/lingtu-nav-dds.service").read_text(encoding="utf-8")
    terrain_service = Path("scripts/deploy/thunder/lingtu-traversability-dds.service").read_text(encoding="utf-8")
    mapd_service = Path("scripts/deploy/thunder/mapd.service").read_text(encoding="utf-8")
    expected_nav = {
        "LINGTU_NAV_ODOM_MAX_AGE_S": "odom_max_age_s",
        "LINGTU_NAV_TF_MAX_AGE_S": "tf_max_age_s",
        "LINGTU_NAV_CLOUD_MAX_AGE_S": "cloud_max_age_s",
        "LINGTU_NAV_CLOUD_POSE_MAX_GAP_S": "cloud_pose_max_gap_s",
        "LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S": "localization_health_max_age_s",
        "LINGTU_NAV_INPUT_RECOVERY_FRAMES": "input_recovery_frames",
        "LINGTU_NAV_STOP_CONFIRMATION_TIMEOUT_S": "stop_confirmation_timeout_s",
    }
    for environment_name, profile_name in expected_nav.items():
        match = re.search(rf"Environment={environment_name}=([^\r\n]+)", nav_service)
        assert match is not None
        assert float(match.group(1)) == pytest.approx(float(FIELD_TELEOP_AVOID_PROFILE[profile_name]))
    expected_terrain = {
        "LINGTU_TRAVERSABILITY_PUBLISH_HZ": "traversability_publish_hz",
        "LINGTU_TRAVERSABILITY_TERRAIN_MAP_HZ": "traversability_slow_hz",
        "LINGTU_TRAVERSABILITY_TICK_HZ": "traversability_tick_hz",
        "LINGTU_TRAVERSABILITY_CLOUD_POSE_MAX_GAP_S": ("traversability_cloud_pose_max_gap_s"),
        "LINGTU_TRAVERSABILITY_OBSERVED_FREE_TTL_S": "observed_free_ttl_s",
    }
    for environment_name, profile_name in expected_terrain.items():
        match = re.search(rf"Environment={environment_name}=([^\r\n]+)", terrain_service)
        assert match is not None
        assert float(match.group(1)) == pytest.approx(float(FIELD_TELEOP_AVOID_PROFILE[profile_name]))
    expected_mapd = {
        "LINGTU_MAPD_MAX_CLOUD_BYTES": "max_cloud_bytes",
        "LINGTU_MAPD_MAX_POINT_FIELDS": "max_fields",
        "LINGTU_MAPD_MAX_POINT_STEP": "max_point_step",
        "LINGTU_MAPD_MAX_STRING_BYTES": "max_string_bytes",
        "LINGTU_MAPD_MAX_SCENE_BYTES": "max_scene_bytes",
        "LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS": "max_voxel_snapshot_points",
        "LINGTU_MAPD_VOXEL_SNAPSHOT_RADIUS_M": "voxel_snapshot_radius_m",
        "LINGTU_MAPD_MAX_VOXELS": "max_voxels",
        "LINGTU_MAPD_MAX_ACCUMULATED_CELLS": "max_accumulated_cells",
        "LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS": "max_accumulated_blocks",
        "LINGTU_MAPD_CARVE_MIN_Z_M": "carve_min_z_m",
        "LINGTU_MAPD_CARVE_MAX_Z_M": "carve_max_z_m",
    }
    for environment_name, profile_name in expected_mapd.items():
        match = re.search(rf"Environment={environment_name}=([^\r\n]+)", mapd_service)
        assert match is not None
        assert float(match.group(1)) == pytest.approx(float(MAPD_RUNTIME_PROFILE[profile_name]))


def _mapd_status(**overrides: object) -> dict:
    status = {
        "schema_version": "lingtu.maps.runtime.v1",
        "process": "mapd",
        "status": "ready",
        "ready": True,
        "running": True,
        "live": True,
        "accepted_observations": 4,
        "processed_observations": 3,
        "generation": 3,
        "dds_received": 4,
        "dds_decoded": 4,
        "dds_rejected": 0,
        "dds_write_attempts": 16,
        "dds_write_failures": 0,
        "dds_serialization_rejections": 0,
        "dds_scene_oversize_rejections": 0,
        "dds_unhealthy_writers": 0,
        "required_publications_ready": True,
        "current_generation_published": True,
        "state_published_generation": 3,
        "realtime_clouds_published_generation": 3,
        "map_layers_published_generation": 3,
        "scene_published_generation": 3,
        "capacity_limited": False,
        "voxel_capacity_rejections": 0,
        "accumulated_capacity_rejections": 0,
        "producer_boot_id": "must-not-leak-into-bounded-evidence",
    }
    status.update(overrides)
    return status


def test_mapd_status_evidence_accepts_only_fresh_complete_native_status(tmp_path: Path) -> None:
    status_path = tmp_path / "mapd_status.json"
    status_path.write_text(json.dumps(_mapd_status()), encoding="utf-8")

    evidence = mapd_status_evidence(
        status_path,
        required=True,
        not_before_ns=status_path.stat().st_mtime_ns,
        evidence_scope="product_e2e",
        product_gate_eligible=True,
    )

    assert evidence["ok"] is True
    assert evidence["fresh"] is True
    assert evidence["product_evidence"] is True
    assert evidence["source_topic"] == "/slam/map_observation"
    assert evidence["scene_topic"] == "/maps/scene"
    assert evidence["navigation_traversability_topic"] == "/nav/traversability"
    assert evidence["status"]["status"] == "ready"
    assert evidence["status"]["required_publications_ready"] is True
    assert evidence["status"]["current_generation_published"] is True
    assert evidence["status"]["capacity_limited"] is False
    assert {
        evidence["status"][field] for field in evidence["status"] if field.endswith("published_generation")
    } == {evidence["status"]["generation"]}
    assert "producer_boot_id" not in evidence["status"]


def test_mapd_status_evidence_rejects_stale_or_degraded_status(tmp_path: Path) -> None:
    status_path = tmp_path / "mapd_status.json"
    status_path.write_text(
        json.dumps(
            _mapd_status(
                schema_version="wrong",
                process="not-mapd",
                status="degraded",
                ready=False,
                running=False,
                live=False,
                accepted_observations=0,
                processed_observations=0,
                generation=0,
                dds_received=0,
                dds_decoded=0,
                dds_rejected=1,
                dds_write_attempts=0,
                dds_write_failures=1,
                dds_serialization_rejections=1,
                dds_scene_oversize_rejections=1,
                dds_unhealthy_writers=1,
                required_publications_ready=False,
                current_generation_published=False,
                state_published_generation=2,
                realtime_clouds_published_generation=2,
                map_layers_published_generation=2,
                scene_published_generation=2,
                capacity_limited=True,
                voxel_capacity_rejections=1,
                accumulated_capacity_rejections=1,
            )
        ),
        encoding="utf-8",
    )

    evidence = mapd_status_evidence(
        status_path,
        required=True,
        not_before_ns=status_path.stat().st_mtime_ns + 1,
    )

    assert evidence["ok"] is False
    assert set(evidence["blockers"]) >= {
        "mapd_status_stale",
        "mapd_status_schema_invalid",
        "mapd_status_process_invalid",
        "mapd_status_status_not_ready",
        "mapd_status_ready_false",
        "mapd_status_running_false",
        "mapd_status_live_false",
        "mapd_status_required_publications_ready_false",
        "mapd_status_current_generation_published_false",
        "mapd_status_accepted_observations_not_positive",
        "mapd_status_processed_observations_not_positive",
        "mapd_status_generation_not_positive",
        "mapd_status_dds_received_not_positive",
        "mapd_status_dds_decoded_not_positive",
        "mapd_status_dds_write_attempts_not_positive",
        "mapd_status_dds_rejected_nonzero",
        "mapd_status_dds_write_failures_nonzero",
        "mapd_status_dds_serialization_rejections_nonzero",
        "mapd_status_dds_scene_oversize_rejections_nonzero",
        "mapd_status_dds_unhealthy_writers_nonzero",
        "mapd_status_state_published_generation_mismatch",
        "mapd_status_realtime_clouds_published_generation_mismatch",
        "mapd_status_map_layers_published_generation_mismatch",
        "mapd_status_scene_published_generation_mismatch",
        "mapd_status_capacity_limited_not_false",
        "mapd_status_voxel_capacity_rejections_nonzero",
        "mapd_status_accumulated_capacity_rejections_nonzero",
    }


def test_runtime_startup_gate_requires_healthy_mapd_evidence(tmp_path: Path) -> None:
    class RunningSensor:
        @staticmethod
        def poll() -> None:
            return None

    nav_status = tmp_path / "nav_status.json"
    slam_status = tmp_path / "slam_status.json"
    traversability_status = tmp_path / "traversability_status.json"
    mapd_status = tmp_path / "mapd_status.json"
    nav_status.write_text(
        json.dumps(
            {
                "stamp_s": 1.0,
                "has_odom": True,
                "control_mode": "teleop_avoid",
                "input_gate": {"ready": True},
            }
        ),
        encoding="utf-8",
    )
    slam_status.write_text(json.dumps({"state": "TRACKING"}), encoding="utf-8")
    traversability_status.write_text(
        json.dumps({"counters": {"published": 1}}),
        encoding="utf-8",
    )
    mapd_status.write_text(json.dumps(_mapd_status()), encoding="utf-8")
    common = {
        "sensor": RunningSensor(),
        "nav_status": nav_status,
        "slam_status": slam_status,
        "mapd_status": mapd_status,
        "traversability_status": traversability_status,
        "timeline": [],
        "state": {},
        "state_provider": "fastlio2",
        "mapd_required": True,
        "mapd_not_before_ns": 0,
        "evidence_scope": "product_e2e",
        "product_gate_eligible": True,
        "timeout_s": 0.1,
    }

    ready, reason, evidence = _wait_for_runtime_ready(**common)

    assert (ready, reason) == (True, "ready")
    assert evidence["product_evidence"] is True

    mapd_status.write_text(
        json.dumps(_mapd_status(dds_rejected=1)),
        encoding="utf-8",
    )
    common.update({"timeline": [], "state": {}})
    ready, reason, evidence = _wait_for_runtime_ready(**common)

    assert ready is False
    assert reason == "mapd_runtime_startup_gate_failed"
    assert "mapd_status_dds_rejected_nonzero" in evidence["blockers"]


def test_manifest_keeps_mapd_scene_separate_from_navigation_safety_authority() -> None:
    manifest = json.loads(
        Path("config/runtime_graph/acceptance/mujoco_native_navigation_acceptance.json").read_text(
            encoding="utf-8"
        )
    )
    contracts = manifest["contracts"]

    assert manifest["binaries"]["mapd"]["env"] == "LINGTU_MUJOCO_MAPD_BIN"
    assert "build/maps/mapd" in manifest["binaries"]["mapd"]["candidates"]
    assert "native maps (mapd)" in manifest["description"]
    assert contracts["mapd_input"] == "rt/slam/map_observation"
    assert "rt/maps/state" in contracts["mapd_outputs"]
    assert "rt/maps/scene" in contracts["mapd_outputs"]
    assert contracts["terrain_output"] == "rt/nav/traversability"
    assert contracts["navigation_traversability_authority"] == "rt/nav/traversability"
    assert "rt/nav/traversability" in contracts["navigation_inputs"]


def test_fixture_mapd_omission_is_explicitly_non_product_and_not_covered(tmp_path: Path) -> None:
    evidence = mapd_status_evidence(
        tmp_path / "missing.json",
        required=False,
        evidence_scope="local_planner_simulation_fixture",
        product_gate_eligible=False,
        omission_reason="state_provider_cannot_publish_slam_map_observation",
    )

    assert evidence["evaluated"] is False
    assert evidence["ok"] is False
    assert evidence["product_evidence"] is False
    assert evidence["coverage"] == "not_covered"
    assert evidence["omission_allowed"] is True
    assert evidence["omission_reason"] == "state_provider_cannot_publish_slam_map_observation"


def _nav_sample(
    *,
    phase: str,
    reason: str,
    output_vx: float,
    gate_ready: bool = True,
    gate_reason: str = "ready",
    command_count: int = 1,
) -> dict:
    return {
        "phase": phase,
        "wall_s": float(command_count),
        "nav": {
            "control_mode": "teleop_avoid",
            "input_gate": {
                "ready": gate_ready,
                "recovering": gate_reason == "recovering",
                "reason": gate_reason,
            },
            "teleop": {
                "seen": True,
                "reason": reason,
                "request": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                "output": {"vx": output_vx, "vy": 0.0, "wz": 0.0},
            },
            "final_cmd_vel": {"vx": output_vx, "vy": 0.0, "wz": 0.0},
            "counters": {"teleop_cmd": command_count},
        },
    }


def _motion_posture_sample(
    *,
    z: float = 0.48,
    roll_rad: float = 0.0,
    pitch_rad: float = 0.0,
    phase: str = "steady",
) -> dict:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    # Quaternion for yaw=0 using MuJoCo qpos order qw, qx, qy, qz.
    quaternion = [cr * cp, sr * cp, cr * sp, -sr * sp]
    return {
        "phase": phase,
        "driving": True,
        "qpos": [0.0, 0.0, z, *quaternion],
        "cmd": [0.0, 0.0, 0.0],
        "x": 0.0,
        "y": 0.0,
    }


def test_simulation_posture_gate_uses_motion_qpos() -> None:
    samples = [_motion_posture_sample(z=0.46, roll_rad=0.10, pitch_rad=-0.12) for _ in range(4)]

    result = evaluate_simulation_posture(samples)

    assert result["ok"] is True
    assert result["evaluated"] is True
    assert result["evidence_origin"] == "motion"
    assert result["sources"] == ["motion.qpos"]
    assert result["min_base_z_m"] == pytest.approx(0.46)
    assert result["thresholds"] == SIMULATION_POSTURE_GATE


def test_simulation_posture_gate_falls_back_to_timeline_posture() -> None:
    timeline = [
        {
            "phase": "steady",
            "nav": {
                "simulation_posture": {
                    "base_z_m": 0.48,
                    "roll_rad": 0.0,
                    "pitch_rad": 0.90,
                }
            },
        }
        for _ in range(3)
    ]

    result = evaluate_simulation_posture([], timeline)

    assert result["ok"] is False
    assert result["reason"] == "simulation_posture_invalid"
    assert result["evidence_origin"] == "timeline"
    assert result["longest_invalid_run"] == 3
    assert result["violations"] == ["pitch_exceeds_limit"]


def test_invalid_simulation_posture_preempts_terrain_attribution() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    motion_samples = [
        _motion_posture_sample(),
        *[_motion_posture_sample(z=0.22) for _ in range(3)],
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert result["blockers"] == ["simulation_posture_invalid"]
    assert result["failure"]["reason"] == "simulation_posture_invalid"
    assert result["policy_attribution"] == {
        "evaluated": False,
        "reason": "simulation_posture_invalid",
    }
    assert result["metrics"]["teleop_reasons"] == []
    assert result["simulation_posture"]["fall_detected"] is True
    assert "base_z_below_min" in result["simulation_posture"]["violations"]


def test_free_case_proves_ready_native_teleop_and_policy_motion() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="accepted",
            output_vx=0.18,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.18, 0.0, 0.0],
            "x": index * 0.04,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["metrics"]["policy_motion_xy_m"] >= 0.20
    assert result["metrics"]["steady_nonzero_cmd_samples"] == 6


def test_free_case_accepts_native_local_planner_control_ready_reason() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="teleop_assist_control_ready",
            output_vx=0.18,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.18, 0.0, 0.0],
            "x": index * 0.04,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["accepted_ratio"] == 1.0


def test_cleanup_stop_trigger_uses_absolute_simulation_horizon() -> None:
    trigger = acceptance._cleanup_stop_trigger_sim_s(
        arm_observed_sim_time_s=6.345,
        total_duration_s=25.0,
        stop_margin_s=5.0,
    )

    assert trigger == 20.0
    assert trigger < 25.0


def test_free_case_rejects_pulsed_motion_and_any_false_hazard_reason() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="accepted" if index < 2 else "terrain_stop",
            output_vx=0.18 if index < 2 else 0.0,
            command_count=index,
        )
        for index in range(1, 7)
    ]
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.18 if index < 2 else 0.0, 0.0, 0.0],
            "x": index * 0.03,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert "free_hazard_decision_observed" in result["blockers"]
    assert "free_accepted_ratio_too_low" in result["blockers"]
    assert "free_output_scale_out_of_range" in result["blockers"]
    assert "free_nonzero_command_ratio_too_low" in result["blockers"]


def test_mode_evidence_is_independent_from_input_gate_readiness() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=index,
        )
        for index in range(1, 4)
    ]
    motion_samples = [_motion_posture_sample() for _ in range(3)]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert "input_gate_ready_missing" in result["blockers"]
    assert "teleop_avoid_mode_missing" not in result["blockers"]


def test_obstacle_slow_requires_auditable_scale_on_final_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="obstacle_slow",
            output_vx=0.063,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["slowed"] = True
        sample["nav"]["teleop"]["obstacle_distance_m"] = 0.90
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.063, 0.0, 0.0],
            "x": index * 0.02,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "obstacle_slow",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["median_policy_cmd_vx"] == 0.063
    assert result["metrics"]["median_output_scale"] == 0.35
    assert result["metrics"]["obstacle_distance_m"] == 0.90


def test_obstacle_stop_proves_zero_at_policy_while_teleop_requests_continue() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="obstacle_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 7)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["stopped"] = True
        sample["nav"]["teleop"]["obstacle_distance_m"] = 0.42
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.0, 0.0, 0.0],
            "x": 0.002,
            "y": -0.001,
        }
        for _ in range(8)
    ]

    result = evaluate_case(
        "obstacle_stop",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["steady_zero_cmd_samples"] == 8
    assert result["metrics"]["teleop_command_count_delta"] == 5
    assert result["metrics"]["policy_motion_xy_m"] == 0.0


def test_terrain_soft_is_labeled_as_injected_dds_consumer_evidence() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_slow",
            output_vx=0.063,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["slowed"] = True
        sample["nav"]["teleop"]["traversability_cost"] = 50.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.063, 0.0, 0.0],
            "x": index * 0.02,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "terrain_soft",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is True
    assert result["evidence_scope"] == "dds_consumer_contract_injected"
    assert result["producer_e2e"] is False
    assert result["metrics"]["traversability_cost"] == 50.0


def test_terrain_hard_injected_contract_requires_zero_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["stopped"] = True
        sample["nav"]["teleop"]["traversability_cost"] = 100.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.0, 0.0, 0.0],
            "x": 0.0,
            "y": 0.0,
        }
        for _ in range(6)
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is True
    assert result["evidence_scope"] == "dds_consumer_contract_injected"
    assert result["metrics"]["traversability_cost"] == 100.0
    assert result["metrics"]["steady_zero_cmd_samples"] == 6


def test_terrain_hard_rejects_a_nonzero_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 5)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["traversability_cost"] = 100.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.10, 0.0, 0.0],
            "x": index * 0.03,
            "y": 0.0,
        }
        for index in range(5)
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is False
    assert "terrain_hard_zero_command_missing" in result["blockers"]


def test_traversability_dropout_recovers_only_after_zero_and_recovery_hysteresis() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=2,
        ),
        _nav_sample(
            phase="dropout_grace",
            reason="accepted",
            output_vx=0.18,
            command_count=3,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=4,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="recovering",
            command_count=7,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=9,
        ),
    ]
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.00, "y": 0.0},
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "dropout_grace", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.07, "y": 0.0},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.08, "y": 0.0},
    ]

    result = evaluate_case(
        "traversability_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["dropout_reason"] == "traversability_stale"
    assert result["metrics"]["dropout_zero_cmd_samples"] == 2
    assert result["metrics"]["recovery_sequence"] == ["recovering", "ready"]
    assert result["metrics"]["teleop_command_count_delta"] == 8
    assert result["metrics"]["dropout_teleop_command_count_delta"] == 2


def test_correlated_slam_dropout_proves_all_streams_stale_and_recovers() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=2,
        ),
        _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=4,
        ),
        _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="recovering",
            command_count=7,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=9,
        ),
    ]
    for sample in nav_samples:
        sample["nav"]["counters"].update({"odom": 20, "tf": 21, "registered_clouds": 19})
    for sample in nav_samples[2:4]:
        sample["nav"]["input_gate"].update(
            {
                "odom_age_s": 0.8,
                "odom_max_age_s": 0.25,
                "tf_age_s": 0.8,
                "tf_max_age_s": 0.25,
                "cloud_age_s": 0.8,
                "cloud_max_age_s": 0.35,
                "localization_health_age_s": 0.8,
                "localization_health_max_age_s": 0.5,
            }
        )
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0]},
    ]

    result = evaluate_case(
        "slam_inputs_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["dropout_reason"] == "odom_stale"
    assert result["metrics"]["correlated_slam_streams_stale"] == [
        "odom",
        "tf",
        "registered_cloud",
        "localization_health",
    ]
    assert result["metrics"]["correlated_slam_counter_deltas"] == {
        "odom": 0,
        "tf": 0,
        "registered_cloud": 0,
    }
    assert result["metrics"]["input_gate_generation_recovery_proven"] is True


def test_correlated_slam_dropout_rejects_an_advancing_stream() -> None:
    def sample(command_count: int, odom_count: int) -> dict:
        value = _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=command_count,
        )
        value["nav"]["counters"].update({"odom": odom_count, "tf": 4, "registered_clouds": 5})
        value["nav"]["input_gate"].update(
            {
                "odom_age_s": 0.8,
                "odom_max_age_s": 0.25,
                "tf_age_s": 0.8,
                "tf_max_age_s": 0.25,
                "cloud_age_s": 0.8,
                "cloud_max_age_s": 0.35,
                "localization_health_age_s": 0.8,
                "localization_health_max_age_s": 0.5,
            }
        )
        return value

    nav_samples = [sample(2, 7), sample(4, 8)]
    result = evaluate_case(
        "slam_inputs_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=[
            {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
            {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        ],
        command_vx=0.18,
    )

    assert "slam_dropout_stream_advanced:odom" in result["blockers"]


def test_dropout_rejects_nonzero_final_command_during_recovery_hysteresis() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=3,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=5,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.05,
            gate_ready=False,
            gate_reason="recovering",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=8,
        ),
    ]
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0]},
    ]

    result = evaluate_case(
        "traversability_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert "dropout_recovery_nonzero_before_ready" in result["blockers"]


def test_native_gate_log_proves_dropout_recovery_hysteresis(tmp_path: Path) -> None:
    log = tmp_path / "navigation.log"
    log.write_text(
        "tick gate=ready\n"
        "tick gate=traversability_stale\n"
        "tick gate=traversability_stale\n"
        "tick gate=recovering\n"
        "tick gate=recovering\n"
        "tick gate=ready\n",
        encoding="utf-8",
    )

    transitions = read_native_gate_transitions(log)
    assert transitions == ["ready", "traversability_stale", "recovering", "ready"]
    assert contains_ordered_transition(transitions, ("traversability_stale", "recovering", "ready"))


def test_native_gate_log_scope_excludes_unrelated_recovery_transition(tmp_path: Path) -> None:
    log = tmp_path / "navigation.log"
    prefix = (
        "startup gate=traversability_stale\n"
        "startup gate=recovering\n"
        "startup gate=ready\n"
    )
    fault_window = "fault gate=traversability_stale\nfault gate=ready\n"
    log.write_bytes((prefix + fault_window).encode("utf-8"))

    transitions = read_native_gate_transitions(
        log,
        start_offset=len(prefix.encode("utf-8")),
        end_offset=len((prefix + fault_window).encode("utf-8")),
    )

    assert transitions == ["traversability_stale", "ready"]
    assert not contains_ordered_transition(transitions, ("traversability_stale", "recovering", "ready"))


def test_terrain_producer_scene_variants_use_declared_continuous_slopes(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        '<mujoco model="base"><worldbody><geom name="floor" type="plane"/></worldbody></mujoco>',
        encoding="utf-8",
    )

    soft = build_scene_variant(base, tmp_path / "soft.xml", "terrain_soft")
    hard = build_scene_variant(base, tmp_path / "hard.xml", "terrain_hard")
    obstacle_slow = build_scene_variant(base, tmp_path / "obstacle_slow.xml", "obstacle_slow")

    soft_root = ET.parse(soft).getroot()
    hard_root = ET.parse(hard).getroot()
    slow_root = ET.parse(obstacle_slow).getroot()
    soft_geom = soft_root.find("./worldbody/geom[@name='acceptance_terrain_soft']")
    hard_geom = hard_root.find("./worldbody/geom[@name='acceptance_terrain_hard']")
    slow_geom = slow_root.find("./worldbody/geom[@name='acceptance_obstacle_slow']")
    soft_backstop = soft_root.find("./worldbody/geom[@name='acceptance_observed_free_backstop']")
    slow_backstop = slow_root.find("./worldbody/geom[@name='acceptance_observed_free_backstop']")

    assert soft_geom is not None
    assert soft_geom.attrib["type"] == "plane"
    assert soft_geom.attrib["pos"] == "3.80 0 0"
    assert soft_geom.attrib["size"] == "10 10 0.10"
    assert soft_geom.attrib["euler"] == "0 -12.5 0"
    assert soft_geom.attrib["contype"] == "1"
    assert soft_geom.attrib["conaffinity"] == "15"
    assert hard_geom is not None
    assert hard_geom.attrib["type"] == "plane"
    assert hard_geom.attrib["pos"] == "3.80 0 0"
    assert hard_geom.attrib["size"] == "10 10 0.10"
    assert hard_geom.attrib["euler"] == "0 -28 0"
    assert hard_geom.attrib["contype"] == "1"
    assert hard_geom.attrib["conaffinity"] == "15"
    assert slow_geom is not None
    assert slow_geom.attrib["pos"] == "0.94 0.35 0.40"
    assert slow_geom.attrib["size"] == "0.04 0.08 0.40"
    assert soft_backstop is None
    assert slow_backstop is not None
    assert slow_backstop.attrib["contype"] == "0"


def test_terrain_scene_forward_probe_attribution_requires_target_region_and_cost_band() -> None:
    soft_status = {
        "forward_probe": {
            "terrain_generation": 7,
            "samples": [
                {
                    "used_by_teleop": True,
                    "in_bounds": True,
                    "observed_before_overlays": True,
                    "map_x": 4.10,
                    "map_y": 0.0,
                    "occupancy_cost": 0.0,
                    "surface_risk_cost": 55.0,
                    "fused_cost": 55.0,
                },
                {
                    "used_by_teleop": True,
                    "in_bounds": True,
                    "observed_before_overlays": True,
                    "map_x": 7.95,
                    "map_y": 0.0,
                    "occupancy_cost": 0.0,
                    "surface_risk_cost": 100.0,
                    "fused_cost": 100.0,
                },
            ],
        },
    }

    soft = terrain_scene_forward_probe_attribution("terrain_soft", soft_status)

    assert soft["ok"] is True
    assert soft["component"] == "surface_risk_cost"
    assert len(soft["candidate_samples"]) == 1
    assert soft["matching_samples"][0]["map_x"] == 4.10

    hard = terrain_scene_forward_probe_attribution(
        "terrain_hard",
        {
            "forward_probe": {
                "terrain_generation": 8,
                "samples": [
                    {
                        "used_by_teleop": True,
                        "in_bounds": True,
                        "observed_before_overlays": True,
                        "map_x": 4.10,
                        "map_y": 0.0,
                        "occupancy_cost": 0.0,
                        "surface_risk_cost": 100.0,
                        "fused_cost": 100.0,
                    }
                ],
            }
        },
    )
    outside_only = terrain_scene_forward_probe_attribution(
        "terrain_hard",
        {
            "forward_probe": {
                "terrain_generation": 8,
                "samples": [
                    {
                        "used_by_teleop": True,
                        "in_bounds": True,
                        "observed_before_overlays": True,
                        "map_x": 7.95,
                        "map_y": 0.0,
                        "occupancy_cost": 0.0,
                        "surface_risk_cost": 100.0,
                        "fused_cost": 100.0,
                    }
                ],
            }
        },
    )
    mixed_obstacle = terrain_scene_forward_probe_attribution(
        "terrain_hard",
        {
            "forward_probe": {
                "terrain_generation": 8,
                "samples": [
                    {
                        "used_by_teleop": True,
                        "in_bounds": True,
                        "observed_before_overlays": True,
                        "map_x": 4.10,
                        "map_y": 0.0,
                        "occupancy_cost": 100.0,
                        "surface_risk_cost": 100.0,
                        "fused_cost": 100.0,
                    }
                ],
            }
        },
    )
    off_center = terrain_scene_forward_probe_attribution(
        "terrain_hard",
        {
            "forward_probe": {
                "terrain_generation": 8,
                "samples": [
                    {
                        "used_by_teleop": True,
                        "in_bounds": True,
                        "observed_before_overlays": True,
                        "map_x": 4.10,
                        "map_y": 0.50,
                        "occupancy_cost": 0.0,
                        "surface_risk_cost": 100.0,
                        "fused_cost": 100.0,
                    }
                ],
            }
        },
    )


    assert hard["ok"] is True
    assert hard["matching_samples"][0]["surface_risk_cost"] == 100.0
    assert outside_only["ok"] is False
    assert mixed_obstacle["ok"] is False
    assert off_center["ok"] is False


def test_terrain_scene_extends_generated_corridor_for_mid360_lookahead(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        """<mujoco model="base"><worldbody>
        <geom name="floor" type="box" pos="1.500 0 -0.025" size="2.500 0.900 0.025"/>
        <geom name="left_rail" type="box" pos="1.500 0.900 0.4" size="2.500 0.05 0.4"/>
        <geom name="right_rail" type="box" pos="1.500 -0.900 0.4" size="2.500 0.05 0.4"/>
        </worldbody></mujoco>""",
        encoding="utf-8",
    )

    scene = build_scene_variant(base, tmp_path / "terrain.xml", "terrain_soft")
    root = ET.parse(scene).getroot()
    for name in ("floor", "left_rail", "right_rail"):
        geom = root.find(f"./worldbody/geom[@name='{name}']")
        assert geom is not None
        assert geom.attrib["pos"].split()[0] == "3.500"
        assert geom.attrib["size"].split()[0] == "4.500"


def test_interactive_obstacle_stop_scene_has_demo_clearance_without_leaving_path(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        '<mujoco model="base"><worldbody><geom name="floor" type="plane"/></worldbody></mujoco>',
        encoding="utf-8",
    )

    scene = build_scene_variant(base, tmp_path / "demo_stop.xml", "obstacle_stop_demo")
    obstacle = ET.parse(scene).getroot().find("./worldbody/geom[@name='acceptance_obstacle_stop_demo']")

    assert obstacle is not None
    pos = [float(value) for value in obstacle.attrib["pos"].split()]
    size = [float(value) for value in obstacle.attrib["size"].split()]
    conservative_robot_front_x_m = 0.50
    minimum_demo_clearance_m = 1.10
    obstacle_near_face_x_m = pos[0] - size[0]

    assert obstacle_near_face_x_m - conservative_robot_front_x_m >= minimum_demo_clearance_m
    assert abs(pos[1]) <= size[1]


def test_odom_prior_diagnostic_derives_config_without_mutating_product_default(
    tmp_path: Path,
) -> None:
    base = tmp_path / "slam.yaml"
    base.write_text(
        "backend: fastlio2\nodom_prior_enabled: false\n",
        encoding="utf-8",
    )

    derived = build_odom_prior_diagnostic_config(
        base,
        tmp_path / "diagnostic" / "slam.yaml",
    )

    assert "odom_prior_enabled: false" in base.read_text(encoding="utf-8")
    assert "odom_prior_enabled: true" in derived.read_text(encoding="utf-8")


def test_manifest_only_declares_runtime_tolerances_consumed_by_the_harness() -> None:
    manifest = json.loads(acceptance.DEFAULT_MANIFEST.read_text(encoding="utf-8"))

    assert set(manifest["runtime_tolerances"]) == {
        "sim_hardware_realtime_factor",
        "sim_hardware_catch_up_yield_steps",
        "input_future_tolerance_s",
        "sensor_publisher_write_mode",
        "sensor_publisher_async_max_bytes",
        "sensor_publisher_async_max_records",
        "sensor_publisher_async_max_batches",
        "sensor_publisher_async_oldest_s",
        "sensor_publisher_async_shutdown_s",
    }
    assert manifest["runtime_tolerances"]["sim_hardware_catch_up_yield_steps"] >= 40

def test_external_arm_trigger_and_status_are_case_scoped_and_fail_closed(
    tmp_path: Path,
) -> None:
    token = "a" * 32
    arm_path = tmp_path / "sensor_arm.json"
    trigger = acceptance.trigger_external_arm(
        arm_path,
        token=token,
        domain_id=226,
        scenario="free",
    )

    payload = json.loads(arm_path.read_text(encoding="utf-8"))
    assert payload == {
        "schema": acceptance.EXTERNAL_ARM_SCHEMA,
        "arm": True,
        "token": token,
        "domain_id": 226,
        "scenario": "free",
    }
    assert not arm_path.with_name(f".{arm_path.name}.tmp").exists()
    assert token not in json.dumps(trigger)

    status_path = tmp_path / "sensor_arm_status.json"
    status_path.write_text(
        json.dumps(
            {
                "schema": acceptance.EXTERNAL_ARM_STATUS_SCHEMA,
                "enabled": True,
                "state": "armed",
                "acknowledged": True,
                "domain_id": 226,
                "scenario": "free",
                "token_sha256_12": acceptance._external_arm_token_digest(token),
                "arm_observed_sim_time_s": 12.5,
                "wait_elapsed_wall_s": 0.1,
                "last_error": "",
            }
        ),
        encoding="utf-8",
    )
    evidence = acceptance.external_arm_status_evidence(
        status_path,
        token=token,
        domain_id=226,
        scenario="free",
        not_before_ns=int(trigger["not_before_ns"]),
    )
    assert evidence["acknowledged"] is True
    mismatch = acceptance.external_arm_status_evidence(
        status_path,
        token=token,
        domain_id=227,
        scenario="free",
        not_before_ns=int(trigger["not_before_ns"]),
    )
    assert mismatch["acknowledged"] is False
    assert "external_arm_domain_mismatch" in mismatch["blockers"]


def test_continuous_teleop_must_be_admitted_before_external_arm(tmp_path: Path) -> None:
    class Running:
        @staticmethod
        def poll() -> None:
            return None

    nav_status = tmp_path / "nav_status.json"
    nav_status.write_text(
        json.dumps(
            {
                "stamp_s": 1.0,
                "operator_motion": {
                    "status": {
                        "active_source_id": "mujoco-teleop-avoid-226",
                        "active_source_epoch": 7,
                        "has_active_authority": True,
                        "admitted_sequence": 9,
                        "final_output_sequence": 12,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    admitted, reason, evidence = acceptance._wait_for_continuous_teleop_admission(
        sensor=Running(),
        teleop=Running(),
        nav_status=nav_status,
        timeline=[],
        state={},
        source_id="mujoco-teleop-avoid-226",
        timeout_s=0.1,
    )

    assert admitted is True
    assert reason == "continuous_teleop_admitted"
    assert evidence["admitted_sequence"] == 9


def test_execution_plan_is_the_real_native_teleop_avoid_policy_chain(tmp_path: Path) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam": tmp_path / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "path_library": tmp_path / "paths",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_execution_plan(
        scenario="free",
        domain_id=226,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )

    by_name = {item["name"]: item for item in plan["processes"]}
    assert list(by_name) == ["slam", "mapd", "traversability", "navigation", "sensor"]
    mapd_command = by_name["mapd"]["command"]
    assert mapd_command[mapd_command.index("--domain-id") + 1] == "226"
    assert mapd_command[mapd_command.index("--status-file") + 1] == native._linux_arg(
        Path(plan["artifacts"]["mapd_status"])
    )
    for option, expected in {
        "--max-cloud-bytes": "16777216",
        "--max-fields": "16",
        "--max-point-step": "64",
        "--max-string-bytes": "4096",
        "--max-scene-bytes": "33554432",
        "--max-voxel-snapshot-points": "200000",
        "--voxel-snapshot-radius": "30",
        "--max-voxels": "500000",
        "--max-accumulated-cells": "2000000",
        "--max-accumulated-blocks": "4096",
        "--carve-min-z": "-0.7",
        "--carve-max-z": "1.8",
    }.items():
        assert mapd_command[mapd_command.index(option) + 1] == expected
    assert by_name["mapd"]["log"].endswith("mapd.log")
    assert "--control-mode" in by_name["navigation"]["command"]
    assert "teleop_avoid" in by_name["navigation"]["command"]
    assisted_planner_index = by_name["navigation"]["command"].index("--teleop-local-planner")
    assert by_name["navigation"]["command"][assisted_planner_index + 1] == "true"
    path_library_index = by_name["navigation"]["command"].index("--path-library")
    assert by_name["navigation"]["command"][path_library_index + 1] == native._linux_arg(paths["path_library"])
    assert "--command-source" in by_name["sensor"]["command"]
    assert "dds" in by_name["sensor"]["command"]
    assert "--drive-mode" in by_name["sensor"]["command"]
    assert "policy" in by_name["sensor"]["command"]
    sensor_command = by_name["sensor"]["command"]
    assert "--sim-hardware-catch-up-yield-steps" not in sensor_command
    assert plan["artifacts"]["parent_sensor_diagnostics"].endswith(
        "parent_sensor_diagnostics.json"
    )
    assert sensor_command[sensor_command.index("--parent-diagnostics-json") + 1] == plan[
        "artifacts"
    ]["parent_sensor_diagnostics"]
    assert sensor_command[sensor_command.index("--parent-diagnostics-period-s") + 1] == "0.5"
    arm_contract = plan["external_arm"]
    assert arm_contract["required"] is True
    assert arm_contract["schema"] == acceptance.EXTERNAL_ARM_SCHEMA
    assert arm_contract["status_schema"] == acceptance.EXTERNAL_ARM_STATUS_SCHEMA
    assert len(arm_contract["token"]) == 32
    for option, expected in {
        "--external-arm-file": plan["artifacts"]["sensor_arm"],
        "--external-arm-token": arm_contract["token"],
        "--external-arm-timeout-s": "120.0",
        "--external-arm-status-json": plan["artifacts"]["sensor_arm_status"],
        "--external-arm-scenario": "free",
    }.items():
        assert sensor_command[sensor_command.index(option) + 1] == expected
    assert "--terrain-soft-height-m" in by_name["traversability"]["command"]
    assert "0.08" in by_name["traversability"]["command"]
    assert "--terrain-hard-height-m" in by_name["traversability"]["command"]
    assert "0.20" in by_name["traversability"]["command"]
    slow_index = by_name["traversability"]["command"].index("--slow-hz")
    assert by_name["traversability"]["command"][slow_index + 1] == "5"
    terrain_tick_index = by_name["traversability"]["command"].index("--tick-hz")
    assert by_name["traversability"]["command"][terrain_tick_index + 1] == "50"
    slam_command = by_name["slam"]["command"]
    assert slam_command[slam_command.index("--mode") + 1] == "mapping"
    assert "--map" not in slam_command
    assert "--track-against-map-period-s" not in slam_command
    assert "--track-against-map-initial-pose" not in slam_command
    assert plan["product_contract"]["product"] == "teleop_avoid"
    assert plan["product_contract"]["slam_mode"] == "mapping"
    assert plan["product_contract"]["requires_map"] is False
    assert paths["slam"] not in [Path(value) for value in slam_command]
    nav_command = by_name["navigation"]["command"]
    for option, expected in {
        "--odom-max-age-s": "0.25",
        "--tf-max-age-s": "0.25",
        "--cloud-max-age-s": "0.35",
        "--cloud-pose-max-gap-s": "0.1",
        "--localization-health-max-age-s": "0.5",
        "--input-recovery-frames": "3",
        "--stop-confirmation-timeout-s": "4.0",
    }.items():
        option_index = nav_command.index(option)
        assert nav_command[option_index + 1] == expected
    assert "operator-motion" in plan["teleop_command"]
    assert "teleop" not in plan["teleop_command"]
    assert "--duration-s" in plan["teleop_command"]
    teleop_timeout_index = plan["teleop_command"].index("--timeout-ms")
    assert plan["teleop_command"][teleop_timeout_index + 1] == "3000"
    teleop_duration_index = plan["teleop_command"].index("--duration-s")
    assert float(plan["teleop_command"][teleop_duration_index + 1]) >= 60.0
    source_index = plan["teleop_command"].index("--source-id")
    assert plan["teleop_command"][source_index + 1] == "mujoco-teleop-avoid-226"
    assert plan["teleop_command"][plan["teleop_command"].index("--lease-ttl-ms") + 1] == "2000"
    assert plan["teleop_command"][plan["teleop_command"].index("--freshness-budget-ms") + 1] == "350"
    assert plan["artifacts"]["motion_log"].endswith("motion.jsonl")
    assert plan["terrain_producer_contract"]["soft_cost"] == 40.0
    assert plan["terrain_producer_contract"]["hard_cost"] == 100.0
    assert plan["functional_scope"]["traversability_cost_in_decision"] is True
    assert plan["functional_scope"]["mapd_process"] is True
    assert plan["functional_scope"]["mapd_data_contract"] == {
        "input": "/slam/map_observation",
        "scene": "/maps/scene",
        "navigation_traversability": "/nav/traversability",
        "navigation_traversability_role": "standalone_safety_authority",
    }
    assert plan["terrain_producer_contract"]["scenario_geometry"] == {}
    assert "scenario_step_height_m" not in plan["terrain_producer_contract"]
    for scenario in ("terrain_soft", "terrain_hard"):
        terrain_plan = build_execution_plan(
            scenario=scenario,
            domain_id=226,
            binaries=binaries,
            paths=paths,
            case_dir=tmp_path / f"{scenario}_case",
            duration_s=8.0,
            warmup_s=4.0,
            command_vx=0.18,
            manifest={},
        )
        contract = terrain_plan["terrain_producer_contract"]
        assert contract["scenario_geometry"] == TERRAIN_SCENE_CONTRACT[scenario]
        assert "scenario_step_height_m" not in contract
        terrain_scope = terrain_plan["functional_scope"]
        assert terrain_scope["live_obstacle_layer"] is False
        assert terrain_scope["terrain_surface_isolation"] is True
        assert terrain_scope["isolation"] == "terrain_surface_decision_with_obstacle_layer_excluded"
        terrain_parameters = terrain_scope["traversability_parameters"]
        assert terrain_parameters["obstacle_min_z_m"] == 2.0
        assert terrain_parameters["obstacle_max_z_m"] == 3.0
        assert terrain_parameters["terrain_soft_slope_deg"] == 12.0

    fixture_plan = build_execution_plan(
        scenario="free",
        domain_id=229,
        binaries={
            name: path
            for name, path in binaries.items()
            if name not in {"slam", "mapd"}
        },
        paths={name: path for name, path in paths.items() if name != "slam_config"},
        case_dir=tmp_path / "fixture_case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={
            "slam_runtime": {"provider": "mujoco_navigation_fixture"},
            "sensor_runtime": {
                "publish_odom_prior": True,
                "scan_time_profile": "instantaneous",
            },
        },
    )
    fixture_by_name = {item["name"]: item for item in fixture_plan["processes"]}
    assert list(fixture_by_name) == ["traversability", "navigation", "sensor"]
    assert fixture_plan["functional_scope"]["mapd_process"] is False
    fixture_sensor = fixture_by_name["sensor"]["command"]
    assert "--navigation-fixture" in fixture_sensor
    assert "--publish-odom-prior" in fixture_sensor
    assert "--require-slam-output" not in fixture_sensor
    assert "--slam-status-json" not in fixture_sensor
    assert "--navigation-fixture-cloud-points" in fixture_sensor
    fixture_cloud_index = fixture_sensor.index("--navigation-fixture-cloud-points")
    assert fixture_sensor[fixture_cloud_index + 1] == "15000"

    obstacle_plan = build_execution_plan(
        scenario="obstacle_slow",
        domain_id=227,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "obstacle_case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )
    obstacle_nav = next(item for item in obstacle_plan["processes"] if item["name"] == "navigation")["command"]
    use_cost_index = obstacle_nav.index("--use-traversability-cost")
    assert obstacle_nav[use_cost_index + 1] == "true"
    obstacle_traversability = next(item for item in obstacle_plan["processes"] if item["name"] == "traversability")[
        "command"
    ]
    obstacle_max_z_index = obstacle_traversability.index("--z-max")
    obstacle_min_z_index = obstacle_traversability.index("--obstacle-min-z")
    soft_height_index = obstacle_traversability.index("--terrain-soft-height-m")
    soft_slope_index = obstacle_traversability.index("--terrain-soft-slope-deg")
    assert obstacle_traversability[obstacle_max_z_index + 1] == "3.00"
    assert obstacle_traversability[obstacle_min_z_index + 1] == "2.00"
    assert obstacle_plan["functional_scope"]["traversability_parameters"]["obstacle_max_z_m"] == 3.0
    assert obstacle_traversability[soft_height_index + 1] == "2.00"
    assert obstacle_traversability[soft_slope_index + 1] == "100.0"
    assert obstacle_plan["functional_scope"]["isolation"] == (
        "live_obstacle_decision_with_free_cost_producer_thresholds"
    )

    diagnostic_plan = build_execution_plan(
        scenario="free",
        domain_id=228,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "diagnostic_case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={
            "_odom_prior_diagnostic": True,
            "sensor_runtime": {"publish_odom_prior": True},
            "runtime_tolerances": {"parent_diagnostics_period_s": 0.75},
        },
    )
    diagnostic_by_name = {
        item["name"]: item for item in diagnostic_plan["processes"]
    }
    assert "mapd" in diagnostic_by_name
    diagnostic_sensor = diagnostic_by_name["sensor"]["command"]
    assert "--publish-odom-prior" in diagnostic_sensor
    assert "--allow-kinematic-fastlio-acceptance" in diagnostic_sensor
    assert diagnostic_sensor[
        diagnostic_sensor.index("--parent-diagnostics-period-s") + 1
    ] == "0.75"
    assert "--publisher-write-mode" not in diagnostic_sensor
    assert "--async-publisher-max-bytes" not in diagnostic_sensor


def test_execution_plan_applies_canonical_sensor_runtime_and_requires_slam_output(
    tmp_path: Path,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    manifest = native._load_manifest(acceptance.DEFAULT_MANIFEST)
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "path_library": tmp_path / "paths",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
    }

    plan = build_execution_plan(
        scenario="free",
        domain_id=226,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest=manifest,
    )

    sensor_command = next(
        item["command"] for item in plan["processes"] if item["name"] == "sensor"
    )
    for option, expected in {
        "--publish-hz": "10.0",
        "--imu-hz": "200.0",
        "--mid360-samples-per-frame": "15000",
        "--scan-time-profile": "physical_rolling",
        "--physical-rolling-sample-mode": "subscan",
        "--sim-hardware-catch-up-yield-steps": "40",
        "--publisher-write-mode": "async_fifo",
        "--async-publisher-max-bytes": "1048576",
        "--async-publisher-max-records": "512",
        "--async-publisher-max-batches": "256",
        "--async-publisher-oldest-s": "0.5",
        "--async-publisher-shutdown-s": "2.0",
    }.items():
        assert sensor_command[sensor_command.index(option) + 1] == expected
    assert "--publish-odom-prior" not in sensor_command
    assert "--require-slam-output" in sensor_command


def test_execution_plan_rejects_localization_mode_for_product_acceptance(
    tmp_path: Path,
) -> None:
    with pytest.raises(
        ValueError,
        match="teleop_avoid product acceptance requires slam_runtime.mode=mapping",
    ):
        build_execution_plan(
            scenario="free",
            domain_id=226,
            binaries={},
            paths={},
            case_dir=tmp_path / "case",
            duration_s=8.0,
            warmup_s=4.0,
            command_vx=0.18,
            manifest={"slam_runtime": {"mode": "localization"}},
        )


def test_execution_plan_rejects_fixture_localization_mode_for_product_acceptance(
    tmp_path: Path,
) -> None:
    with pytest.raises(
        ValueError,
        match="teleop_avoid product acceptance requires slam_runtime.mode=mapping",
    ):
        build_execution_plan(
            scenario="free",
            domain_id=226,
            binaries={},
            paths={},
            case_dir=tmp_path / "fixture_case",
            duration_s=8.0,
            warmup_s=4.0,
            command_vx=0.18,
            manifest={
                "slam_runtime": {
                    "provider": "mujoco_navigation_fixture",
                    "mode": "localization",
                }
            },
        )


def test_execution_plan_cannot_fallback_to_legacy_teleop_when_legacy_inputs_are_disabled(
    tmp_path: Path,
) -> None:
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam": tmp_path / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "path_library": tmp_path / "paths",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_execution_plan(
        scenario="free",
        domain_id=226,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )

    navigation_command = next(
        item["command"] for item in plan["processes"] if item["name"] == "navigation"
    )
    legacy_index = navigation_command.index("--allow-legacy-motion-inputs")
    assert navigation_command[legacy_index + 1] == "false"
    assert "teleop" not in plan["teleop_command"]
    assert "teleop-stream" not in plan["teleop_command"]
    assert "operator-motion" in plan["teleop_command"]


def test_typed_operator_motion_cli_owns_fail_closed_lifecycle() -> None:
    source = Path("src/nav/cpp/endpoint/motion/nav_control.cpp").read_text(encoding="utf-8")
    start = source.index("int runOperatorMotion(")
    end = source.index("}  // namespace", start)
    implementation = source[start:end]

    assert "client.operatorMotion().claim(" in implementation
    assert "client.operatorMotion().sample(" in implementation
    assert "client.operatorMotion().hold(" in implementation
    assert "client.operatorMotion().release(" in implementation
    assert 'finishAuthority("operator_motion_error", true);' in implementation
    assert "requestOperatorMotionStop" in implementation
    assert "client.navigation().sendTeleop" not in implementation


def test_operator_motion_lifecycle_binds_cli_ack_to_native_status_sequences() -> None:
    output = "\n".join(
        (
            "LT_OPERATOR_MOTION_EVENT_V1 action=claim accepted=true source_id=case-226 "
            "source_epoch=77 source_sequence=1 sample_count=0",
            "LT_OPERATOR_MOTION_EVENT_V1 action=sample accepted=true source_id=case-226 "
            "source_epoch=77 source_sequence=2 sample_count=1",
            "LT_OPERATOR_MOTION_EVENT_V1 action=hold accepted=true source_id=case-226 "
            "source_epoch=77 source_sequence=3 sample_count=1",
            "LT_OPERATOR_MOTION_EVENT_V1 action=release accepted=true source_id=case-226 "
            "source_epoch=77 source_sequence=4 sample_count=1",
        )
    )
    events = parse_operator_motion_events(output)
    timeline = [
        {
            "phase": "steady",
            "nav": {
                "operator_motion": {
                    "status": {
                        "active_source_id": "case-226",
                        "active_source_epoch": 77,
                        "admitted_sequence": 2,
                        "final_output_sequence": 41,
                        "has_active_authority": True,
                        "holding": False,
                        "final_cmd_vel": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                    }
                }
            },
        },
        {
            "phase": "cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case-226",
                        "source_epoch": 77,
                        "source_sequence": 3,
                        "action": 3,
                        "accepted": True,
                        "final_output_sequence": 42,
                    },
                    "status": {
                        "has_active_authority": True,
                        "holding": True,
                        "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    },
                }
            },
        },
        {
            "phase": "cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case-226",
                        "source_epoch": 77,
                        "source_sequence": 4,
                        "action": 2,
                        "accepted": True,
                        "final_output_sequence": 43,
                    },
                    "status": {
                        "has_active_authority": False,
                        "holding": False,
                        "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    },
                }
            },
        },
    ]

    evidence = evaluate_operator_motion_lifecycle(timeline, events)

    assert evidence["ok"] is True
    assert evidence["source_id"] == "case-226"
    assert evidence["source_epoch"] == 77
    assert evidence["max_admitted_sequence"] == 2
    assert evidence["max_mapped_final_output_sequence"] == 41
    assert evidence["hold_zero_barrier_samples"] == 1
    assert evidence["release_zero_barrier_samples"] == 1


def test_operator_motion_lifecycle_uses_final_log_events_with_cleanup_status(
    tmp_path: Path,
) -> None:
    attempt_events = parse_operator_motion_events(
        "\n".join(
            (
                "LT_OPERATOR_MOTION_EVENT_V1 action=sample accepted=true source_id=case-226 "
                "source_epoch=77 source_sequence=2 sample_count=1",
                "LT_OPERATOR_MOTION_EVENT_V1 action=hold accepted=true source_id=case-226 "
                "source_epoch=77 source_sequence=3 sample_count=1",
                "LT_OPERATOR_MOTION_EVENT_V1 action=release accepted=true source_id=case-226 "
                "source_epoch=77 source_sequence=4 sample_count=1",
            )
        )
    )
    durable_log = tmp_path / "teleop_command.log"
    durable_log.write_text(
        "\x00WSL banner noise before event "
        "LT_OPERATOR_MOTION_EVENT_V1 action=claim accepted=true source_id=case-226 "
        "source_epoch=77 source_sequence=1 sample_count=0\n",
        encoding="utf-8",
    )
    events = acceptance._dedupe_operator_motion_events(
        [*attempt_events, *acceptance._operator_motion_events_from_log(durable_log)]
    )
    timeline = [
        {
            "phase": "pre_cleanup",
            "nav": {
                "operator_motion": {
                    "status": {
                        "active_source_id": "case-226",
                        "active_source_epoch": 77,
                        "admitted_sequence": 2,
                        "final_output_sequence": 41,
                        "has_active_authority": True,
                        "holding": False,
                        "final_cmd_vel": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                    }
                }
            },
        },
        {
            "phase": "cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case-226",
                        "source_epoch": 77,
                        "source_sequence": 3,
                        "action": 3,
                        "accepted": True,
                        "final_output_sequence": 42,
                    },
                    "status": {
                        "has_active_authority": True,
                        "holding": True,
                        "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    },
                }
            },
        },
        {
            "phase": "final_cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case-226",
                        "source_epoch": 77,
                        "source_sequence": 4,
                        "action": 2,
                        "accepted": True,
                        "final_output_sequence": 43,
                    },
                    "status": {
                        "has_active_authority": False,
                        "holding": False,
                        "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    },
                }
            },
        },
    ]

    evidence = evaluate_operator_motion_lifecycle(timeline, events)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert {event["action"] for event in evidence["events"]} == {
        "claim",
        "sample",
        "hold",
        "release",
    }
    assert evidence["hold_zero_barrier_samples"] == 1
    assert evidence["release_zero_barrier_samples"] == 1


def test_operator_motion_lifecycle_fails_closed_without_release_barrier() -> None:
    events = parse_operator_motion_events(
        "\n".join(
            (
                "LT_OPERATOR_MOTION_EVENT_V1 action=claim accepted=true source_id=case "
                "source_epoch=9 source_sequence=1 sample_count=0",
                "LT_OPERATOR_MOTION_EVENT_V1 action=sample accepted=true source_id=case "
                "source_epoch=9 source_sequence=2 sample_count=1",
                "LT_OPERATOR_MOTION_EVENT_V1 action=hold accepted=true source_id=case "
                "source_epoch=9 source_sequence=3 sample_count=1",
            )
        )
    )

    evidence = evaluate_operator_motion_lifecycle([], events)

    assert evidence["ok"] is False
    assert "operator_motion_release_zero_barrier_missing" in evidence["blockers"]


@pytest.mark.parametrize(
    ("final_cmd_vel", "expected_blockers"),
    (
        (
            None,
            {
                "operator_motion_hold_zero_barrier_missing",
                "operator_motion_release_zero_barrier_missing",
            },
        ),
        (
            {"vx": 0.0, "vy": 0.0},
            {
                "operator_motion_hold_zero_barrier_missing",
                "operator_motion_release_zero_barrier_missing",
            },
        ),
        (
            {"vx": "nan", "vy": 0.0, "wz": "not-a-number"},
            {
                "operator_motion_hold_zero_barrier_missing",
                "operator_motion_release_zero_barrier_missing",
            },
        ),
        ({"vx": 0.0, "vy": 0.0, "wz": 0.0}, set()),
    ),
)
def test_operator_motion_zero_barriers_require_complete_finite_final_cmd(
    final_cmd_vel: object,
    expected_blockers: set[str],
) -> None:
    events = parse_operator_motion_events(
        "\n".join(
            (
                "LT_OPERATOR_MOTION_EVENT_V1 action=claim accepted=true source_id=case "
                "source_epoch=9 source_sequence=1 sample_count=0",
                "LT_OPERATOR_MOTION_EVENT_V1 action=sample accepted=true source_id=case "
                "source_epoch=9 source_sequence=2 sample_count=1",
                "LT_OPERATOR_MOTION_EVENT_V1 action=hold accepted=true source_id=case "
                "source_epoch=9 source_sequence=3 sample_count=1",
                "LT_OPERATOR_MOTION_EVENT_V1 action=release accepted=true source_id=case "
                "source_epoch=9 source_sequence=4 sample_count=1",
            )
        )
    )

    def status(*, holding: bool, active: bool) -> dict[str, object]:
        value: dict[str, object] = {
            "has_active_authority": active,
            "holding": holding,
        }
        if final_cmd_vel is not None:
            value["final_cmd_vel"] = final_cmd_vel
        return value

    timeline = [
        {
            "phase": "steady",
            "nav": {
                "operator_motion": {
                    "status": {
                        "active_source_id": "case",
                        "active_source_epoch": 9,
                        "admitted_sequence": 2,
                        "final_output_sequence": 41,
                        "has_active_authority": True,
                        "holding": False,
                        "final_cmd_vel": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                    }
                }
            },
        },
        {
            "phase": "cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case",
                        "source_epoch": 9,
                        "source_sequence": 3,
                        "action": 3,
                        "accepted": True,
                        "final_output_sequence": 42,
                    },
                    "status": status(holding=True, active=True),
                }
            },
        },
        {
            "phase": "cleanup",
            "nav": {
                "operator_motion": {
                    "last_ack": {
                        "source_id": "case",
                        "source_epoch": 9,
                        "source_sequence": 4,
                        "action": 2,
                        "accepted": True,
                        "final_output_sequence": 43,
                    },
                    "status": status(holding=False, active=False),
                }
            },
        },
    ]

    evidence = evaluate_operator_motion_lifecycle(timeline, events)

    assert set(evidence["blockers"]) == expected_blockers
    assert evidence["ok"] is (not expected_blockers)
    assert evidence["hold_zero_barrier_samples"] == (0 if expected_blockers else 1)
    assert evidence["release_zero_barrier_samples"] == (0 if expected_blockers else 1)


def test_post_stop_zero_output_evidence_accepts_bounded_zero_status_samples() -> None:
    timeline = [
        {
            "wall_s": 10.1,
            "nav": {
                "stamp_s": 20.1,
                "operator_motion": {
                    "status": {"final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}
                }
            },
        },
        {
            "wall_s": 10.2,
            "nav": {"stamp_s": 20.2, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
        },
        {
            "wall_s": 10.3,
            "nav": {
                "stamp_s": 20.3,
                "operator_motion": {
                    "status": {"final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}
                }
            },
        },
    ]

    evidence = acceptance._post_stop_zero_output_evidence(
        timeline,
        stop_ack_wall_s=10.0,
        pre_stop_status_stamp_s=20.0,
        window_start_wall_s=10.05,
        window_end_wall_s=10.35,
    )

    assert evidence["zero_output_observed"] is True
    assert evidence["status_samples"] == 3
    assert evidence["final_cmd_samples"] == 3
    assert evidence["nonzero_final_cmd_samples"] == 0


def test_post_stop_zero_output_evidence_fails_closed_on_nonzero_or_out_of_window() -> None:
    timeline = [
        {"wall_s": 10.1, "nav": {"stamp_s": 20.1, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}},
        {"wall_s": 10.2, "nav": {"stamp_s": 20.2, "final_cmd_vel": {"vx": 0.2, "vy": 0.0, "wz": 0.0}}},
        {"wall_s": 10.3, "nav": {"stamp_s": 20.3, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}},
        {"wall_s": 12.0, "nav": {"stamp_s": 22.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}},
    ]

    evidence = acceptance._post_stop_zero_output_evidence(
        timeline,
        stop_ack_wall_s=10.0,
        pre_stop_status_stamp_s=20.0,
        window_start_wall_s=10.05,
        window_end_wall_s=10.35,
    )

    assert evidence["zero_output_observed"] is False
    assert evidence["status_samples"] == 3
    assert evidence["nonzero_final_cmd_samples"] == 1


def test_post_stop_zero_output_evidence_rejects_stale_or_invalid_status_samples() -> None:
    timeline = [
        {"wall_s": 10.1, "nav": {"stamp_s": 19.9, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}},
        {"wall_s": 10.2, "nav": {"stamp_s": 20.2, "final_cmd_vel": {"vx": 0.0, "vy": 0.0}}},
        {"wall_s": True, "nav": {"stamp_s": 20.3, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}},
    ]

    evidence = acceptance._post_stop_zero_output_evidence(
        timeline,
        stop_ack_wall_s=10.0,
        pre_stop_status_stamp_s=20.0,
        window_start_wall_s=10.05,
        window_end_wall_s=10.35,
        required_status_samples=1,
    )

    assert evidence["zero_output_observed"] is False
    assert evidence["status_samples"] == 1
    assert evidence["invalid_or_stale_status_samples"] == 1
    assert evidence["invalid_final_cmd_samples"] == 1


def test_native_stop_ack_annotation_requires_accepted_stop_stdout() -> None:
    accepted = {"returncode": 0, "stdout": "accepted stop: teleop_avoid_acceptance_cleanup"}
    rejected = {"returncode": 0, "stdout": "cleanup completed"}

    acceptance._annotate_native_stop_ack(accepted, ack_wall_s=42.0)
    acceptance._annotate_native_stop_ack(rejected, ack_wall_s=43.0)

    assert accepted["accepted"] is True
    assert accepted["acked"] is True
    assert accepted["ack_wall_s"] == 42.0
    assert rejected["accepted"] is False
    assert rejected["acked"] is False
    assert "ack_wall_s" not in rejected


def test_native_cleanup_stop_blockers_make_product_source_fail_closed() -> None:
    blockers = acceptance._native_cleanup_stop_blockers(
        product_gate_eligible=True,
        stop_result={"returncode": 0, "accepted": False, "acked": False},
        post_stop_zero_output={"zero_output_observed": False},
    )

    assert blockers == [
        "native_cleanup_stop_ack_missing",
        "native_cleanup_post_stop_zero_unproven",
    ]
    assert (
        acceptance._native_cleanup_stop_blockers(
            product_gate_eligible=True,
            stop_result={"returncode": 0, "accepted": True, "acked": True},
            post_stop_zero_output={"zero_output_observed": True},
        )
        == []
    )


def test_preflight_only_never_starts_a_scenario(tmp_path: Path) -> None:
    args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--artifact-dir",
            str(tmp_path),
            "--preflight-only",
        ]
    )

    def prepare(_args: object) -> dict:
        return {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {"source": "test"},
        }

    def execute(**_kwargs: object) -> dict:
        raise AssertionError("preflight-only must not execute a scenario")

    report = run(args, prepare_runtime_fn=prepare, execute_case_fn=execute)

    assert report["ok"] is True
    assert report["preflight"]["ok"] is True
    assert report["cases"] == []
    assert report["acceptance_evaluated"] is False
    assert report["product_acceptance_passed"] is False
    assert report["requested_case_count"] == 1
    assert report["executed_case_count"] == 0
    assert report["all_requested_cases_executed"] is False
    assert report["mapd_evidence"]["coverage"] == "preflight_only_not_evaluated"


def test_policy_runtime_preflight_fails_before_launch_when_onnxruntime_is_missing(
    monkeypatch,
) -> None:
    real_find_spec = acceptance.importlib.util.find_spec

    def find_spec(name: str):
        if name == "onnxruntime":
            return None
        return real_find_spec(name)

    monkeypatch.setattr(acceptance.importlib.util, "find_spec", find_spec)

    evidence = acceptance._policy_runtime_evidence(required=True)

    assert evidence["ok"] is False
    assert evidence["modules"]["onnxruntime"] is False
    assert evidence["blockers"] == [
        "python_runtime_dependency_missing:onnxruntime"
    ]


def test_teleop_preflight_requires_local_path_library(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    world = tmp_path / "scene.xml"
    world.write_text("<mujoco><worldbody/></mujoco>", encoding="utf-8")
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text("{}", encoding="utf-8")
    manifest = {
        "world": str(world),
        "product_contract": {
            "product": "teleop_avoid",
            "source": "config/runtime_graph/products/teleop_avoid.yaml",
            "native_control_mode": "teleop_avoid",
            "slam_mode": "mapping",
            "requires_map": False,
        },
        "slam_runtime": {"provider": "fastlio2", "mode": "mapping"},
        "asset_builder": {"kind": "scene_only"},
        "binaries": {},
    }
    binaries = {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "traversability",
            "navigation",
            "navigation_control",
            "cmd_vel_tap",
        )
    }
    paths = {
        "world": world,
        "sensor_runner": tmp_path / "sensor.py",
        "policy": tmp_path / "policy.onnx",
    }
    monkeypatch.setattr(native, "_load_manifest", lambda _path: dict(manifest))
    monkeypatch.setattr(
        native,
        "_preflight_map_free",
        lambda _manifest: (
            binaries,
            paths,
            [
                "runtime_path_missing:path_library:/unused/paths",
                "native_binary_missing:autonomy_only_tool",
            ],
            {"map_contract": {"required": False}},
        ),
    )
    monkeypatch.setattr(
        acceptance,
        "_binary_source_provenance",
        lambda _binaries: ({}, []),
    )

    args = build_parser().parse_args(
        [
            "--manifest",
            str(manifest_path),
            "--state-provider",
            "mujoco_navigation_fixture",
            "--artifact-dir",
            str(tmp_path / "artifacts"),
            "--preflight-only",
        ]
    )
    prepared = prepare_runtime(args)

    assert prepared["ok"] is False
    assert prepared["blockers"] == [
        "runtime_path_missing:path_library:/unused/paths"
    ]
    assert prepared["details"]["out_of_scope_preflight_findings"] == [
        "native_binary_missing:autonomy_only_tool"
    ]


def test_product_preflight_requires_native_mapd_binary(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    world = tmp_path / "scene.xml"
    world.write_text("<mujoco><worldbody/></mujoco>", encoding="utf-8")
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text("{}", encoding="utf-8")
    manifest = {
        "world": str(world),
        "product_contract": {
            "product": "teleop_avoid",
            "source": "config/runtime_graph/products/teleop_avoid.yaml",
            "native_control_mode": "teleop_avoid",
            "slam_mode": "mapping",
            "requires_map": False,
        },
        "slam_runtime": {"provider": "fastlio2", "mode": "mapping"},
        "asset_builder": {"kind": "scene_only"},
        "binaries": {},
    }
    binaries = {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "slam",
            "traversability",
            "navigation",
            "navigation_control",
            "cmd_vel_tap",
        )
    }
    paths = {
        "world": world,
        "slam_config": tmp_path / "slam.yaml",
        "path_library": tmp_path / "paths",
        "sensor_runner": tmp_path / "sensor.py",
        "policy": tmp_path / "policy.onnx",
    }
    monkeypatch.setattr(native, "_load_manifest", lambda _path: dict(manifest))
    monkeypatch.setattr(
        native,
        "_preflight_map_free",
        lambda _manifest: (
            binaries,
            paths,
            [],
            {"map_contract": {"required": False}},
        ),
    )
    monkeypatch.setattr(
        acceptance,
        "_binary_source_provenance",
        lambda _binaries: ({}, []),
    )

    args = build_parser().parse_args(
        [
            "--manifest",
            str(manifest_path),
            "--artifact-dir",
            str(tmp_path / "artifacts"),
            "--preflight-only",
        ]
    )
    prepared = prepare_runtime(args)

    assert prepared["ok"] is False
    assert prepared["blockers"] == ["native_binary_missing:mapd"]
def test_motion_samples_are_correlated_with_fault_phase_events() -> None:
    samples = [
        {"t": 99.0, "cmd": [0.0, 0.0, 0.0]},
        {"t": 100.5, "cmd": [0.18, 0.0, 0.0]},
        {"t": 102.5, "cmd": [0.0, 0.0, 0.0]},
        {"t": 104.5, "cmd": [0.18, 0.0, 0.0]},
    ]
    events = [
        {"phase": "baseline", "wall_s": 100.0},
        {"phase": "dropout", "wall_s": 102.0},
        {"phase": "recovery", "wall_s": 104.0},
    ]

    assigned = assign_motion_phases(samples, events)

    assert [sample["phase"] for sample in assigned] == [
        "warmup",
        "baseline",
        "dropout",
        "recovery",
    ]


def test_motion_phase_projection_uses_sim_hardware_rate_not_wall_clock() -> None:
    projected = project_motion_timestamp(
        last_sensor_timestamp_s=100.0,
        last_write_wall_s=200.0,
        event_wall_s=202.0,
        realtime_factor=0.5,
    )
    assigned = assign_motion_phases(
        [
            {"t": 100.9, "cmd": [0.18, 0.0, 0.0]},
            {"t": 101.1, "cmd": [0.0, 0.0, 0.0]},
        ],
        [{"phase": "dropout", "wall_s": 202.0, "motion_s": projected}],
    )

    assert projected == 101.0
    assert [sample["phase"] for sample in assigned] == ["warmup", "dropout"]


def test_managed_process_signal_supports_stop_and_cleanup_cont(monkeypatch) -> None:
    sent: list[int] = []
    monkeypatch.setattr(signal, "SIGSTOP", 19, raising=False)
    monkeypatch.setattr(signal, "SIGCONT", 18, raising=False)

    class Child:
        def poll(self) -> None:
            return None

        def send_signal(self, signum: int) -> None:
            sent.append(signum)

    class Managed:
        linux_pid = None
        pid_path = None
        process = Child()

    assert signal_managed_process(Managed(), "STOP") is True
    assert signal_managed_process(Managed(), "CONT") is True
    assert sent == [signal.SIGSTOP, signal.SIGCONT]


def test_fault_phase_waits_until_policy_driving_and_teleop_is_alive(
    tmp_path: Path,
) -> None:
    motion_log = tmp_path / "motion.jsonl"
    motion_log.write_text(
        '{"t":1.0,"driving":false}\n{"t":2.0,"driving":true}\n',
        encoding="utf-8",
    )

    class Alive:
        def poll(self) -> None:
            return None

    ready, reason = _wait_for_policy_driving(
        sensor=Alive(),
        teleop=Alive(),
        motion_log=motion_log,
        timeout_s=0.1,
    )

    assert (ready, reason) == (True, "driving")


def test_continuous_teleop_early_exit_is_a_hard_blocker() -> None:
    assert continuous_teleop_exit_blocker(None) == ""
    assert continuous_teleop_exit_blocker(1) == "continuous_teleop_exited_early:1"


def test_product_gate_rejects_retried_teleop_but_diagnostic_reports_it() -> None:
    delivery = {
        "retry_count": 1,
        "failure_reason_counts": {"teleop_source_stamp_stale": 1},
    }

    assert (
        typed_teleop_delivery_blocker(
            delivery,
            product_gate_eligible=True,
        )
        == "typed_teleop_delivery_unstable"
    )
    assert (
        typed_teleop_delivery_blocker(
            delivery,
            product_gate_eligible=False,
        )
        == ""
    )


def test_resilient_teleop_restarts_after_ack_timeout_and_stays_live(
    tmp_path: Path,
) -> None:
    created: list[object] = []

    class Attempt:
        def __init__(self, returncode: int | None, output: str) -> None:
            self.returncode = returncode
            self.output = output
            self.cleanup: dict = {}

        def start(self) -> None:
            return None

        def poll(self) -> int | None:
            return self.returncode

        def stop(self) -> None:
            self.cleanup = {"clean": True}

        def tail(self) -> str:
            return self.output

    def factory(_name: str, _command: list[str], _log_path: Path) -> Attempt:
        attempt = Attempt(1, "dds_wait_for_acks(nav_command_request): Timeout") if not created else Attempt(None, "")
        created.append(attempt)
        return attempt

    publisher = ResilientTeleopProcess(
        ["lingtu_nav_control", "operator-motion", "0.18", "0", "0"],
        tmp_path / "teleop.log",
        managed_process_factory=factory,
        retry_delay_s=0.0,
    )
    publisher.start()
    deadline = time.monotonic() + 1.0
    while len(created) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)

    assert len(created) >= 2
    assert publisher.poll() is None
    publisher.request_stop()
    publisher.stop()

    snapshot = publisher.snapshot()
    assert snapshot["attempt_count"] == 2
    assert snapshot["ack_timeout_count"] == 1
    assert snapshot["retry_count"] == 1
    assert publisher.cleanup["clean"] is True


def test_case_artifact_reset_removes_stale_slam_snapshots(tmp_path: Path) -> None:
    snapshots = tmp_path / "slam_clouds"
    nested = snapshots / "old"
    nested.mkdir(parents=True)
    (nested / "stale.pcd").write_text("stale", encoding="utf-8")
    status = tmp_path / "status.json"
    status.write_text("stale", encoding="utf-8")

    reset_case_artifacts(
        {
            "slam_cloud_dir": snapshots,
            "status": status,
        }
    )

    assert snapshots.is_dir()
    assert list(snapshots.iterdir()) == []
    assert not status.exists()


def test_run_assigns_isolated_domains_and_aggregates_case_failures(tmp_path: Path) -> None:
    args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--scenario",
            "obstacle_slow",
            "--domain-base",
            "226",
            "--artifact-dir",
            str(tmp_path),
        ]
    )

    def prepare(_args: object) -> dict:
        return {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {},
        }

    observed: list[tuple[str, int]] = []

    def execute(*, scenario: str, domain_id: int, **_kwargs: object) -> dict:
        observed.append((scenario, domain_id))
        blockers = ["slow_not_observed"] if scenario == "obstacle_slow" else []
        return {"scenario": scenario, "ok": not blockers, "blockers": blockers}

    report = run(args, prepare_runtime_fn=prepare, execute_case_fn=execute)

    assert observed == [("free", 226), ("obstacle_slow", 227)]
    assert report["ok"] is False
    assert report["blockers"] == ["slow_not_observed"]
    assert report["terrain_producer_contract"]["soft_height_m"] == 0.08


def test_fixture_and_injected_reports_are_labeled_non_product_evidence(tmp_path: Path) -> None:
    def prepare_fixture(_args: object) -> dict:
        return {
            "ok": True,
            "blockers": [],
            "manifest": {"slam_runtime": {"provider": "mujoco_navigation_fixture"}},
            "binaries": {},
            "paths": {},
            "details": {},
        }

    fixture_args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--artifact-dir",
            str(tmp_path / "fixture"),
            "--preflight-only",
        ]
    )
    fixture_report = run(fixture_args, prepare_runtime_fn=prepare_fixture)

    assert fixture_report["product_gate_eligible"] is False
    assert fixture_report["evidence_class"] == "non_product_diagnostic"
    assert fixture_report["evidence_scope"] == "local_planner_simulation_fixture"
    assert fixture_report["mapd_evidence"]["required_for_executed_cases"] is False
    assert fixture_report["mapd_evidence"]["coverage"] == (
        "not_covered_state_provider_cannot_publish_slam_map_observation"
    )

    injected_args = build_parser().parse_args(
        [
            "--scenario",
            "terrain_soft_injected",
            "--artifact-dir",
            str(tmp_path / "injected"),
            "--preflight-only",
        ]
    )
    injected_report = run(
        injected_args,
        prepare_runtime_fn=lambda _args: {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {},
        },
    )

    assert injected_report["product_gate_eligible"] is False
    assert injected_report["evidence_class"] == "non_product_diagnostic"
    assert injected_report["evidence_scope"] == "dds_consumer_contract_injected"
    assert injected_report["mapd_evidence"]["required_for_executed_cases"] is True


def test_report_rejects_product_case_without_mapd_evidence(tmp_path: Path) -> None:
    args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--artifact-dir",
            str(tmp_path),
        ]
    )

    report = run(
        args,
        prepare_runtime_fn=lambda _args: {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {},
        },
        execute_case_fn=lambda **_kwargs: {
            "scenario": "free",
            "ok": True,
            "product_gate_eligible": True,
            "blockers": [],
        },
    )

    assert report["ok"] is False
    assert "mapd_product_evidence_missing_or_degraded:free" in report["blockers"]


def test_slam_inputs_dropout_is_selectable_but_not_in_default_matrix() -> None:
    selected = build_parser().parse_args(["--scenario", "slam_inputs_dropout_recovery"])

    assert selected.scenario == ["slam_inputs_dropout_recovery"]
    assert "slam_inputs_dropout_recovery" not in _requested_scenarios(None)


def test_cli_entrypoint_loads_the_evaluator_before_main_runs() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "sim/scripts/mujoco/teleop_avoid_native_acceptance.py",
            "--help",
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "--preflight-only" in completed.stdout


def test_direct_script_contract_adds_repo_root_before_runtime_import(
    tmp_path: Path,
) -> None:
    script = Path(__file__).resolve().parents[1] / "scripts" / "mujoco" / "teleop_avoid_native_acceptance.py"
    code = (
        "import importlib, runpy; "
        f"runpy.run_path({str(script)!r}, run_name='teleop_acceptance_import'); "
        "importlib.import_module('sim.scripts.mujoco.native_navigation_acceptance')"
    )

    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


def test_owned_pid_cleanup_waits_for_late_pidfile_and_audits_term(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    pid_path = tmp_path / "late-child.pid"
    calls: list[tuple] = []

    def read_pid(path: Path, timeout_s: float = 0.0) -> int:
        calls.append(("read", path, timeout_s))
        return 495

    alive_answers = iter((True, False))

    def alive(pid: int | None) -> bool:
        calls.append(("alive", pid))
        return next(alive_answers)

    def signal(pid: int | None, signal_name: str) -> bool:
        calls.append(("signal", pid, signal_name))
        return True

    def wait(pid: int | None, timeout_s: float) -> bool:
        calls.append(("wait", pid, timeout_s))
        return True

    monkeypatch.setattr(native, "_read_linux_pid", read_pid)
    monkeypatch.setattr(native, "_wsl_pid_alive", alive)
    monkeypatch.setattr(native, "_signal_wsl_pid", signal)
    monkeypatch.setattr(native, "_wait_wsl_pid_exit", wait)

    result = cleanup_owned_pid_file("late_child", pid_path, pid_wait_s=1.5)

    assert calls[0] == ("read", pid_path, 1.5)
    assert ("signal", 495, "TERM") in calls
    assert result == {
        "name": "late_child",
        "linux_pid": 495,
        "pid_file": str(pid_path),
        "pid_wait_s": 1.5,
        "alive_before_cleanup": True,
        "term_sent": True,
        "kill_sent": False,
        "alive_after_cleanup": False,
        "clean": True,
    }


def test_owned_pid_cleanup_uses_posix_kill_outside_wsl(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    pid_path = tmp_path / "native-child.pid"
    pid_path.write_text("812\n", encoding="ascii")
    alive = True
    calls: list[tuple[int, int]] = []

    def kill(pid: int, signum: int) -> None:
        nonlocal alive
        calls.append((pid, signum))
        if signum == 0 and not alive:
            raise ProcessLookupError(pid)
        if signum == signal.SIGTERM:
            alive = False

    monkeypatch.setattr(acceptance.os, "name", "posix")
    monkeypatch.setattr(acceptance.os, "kill", kill)
    monkeypatch.setattr(native, "_read_linux_pid", lambda *_args, **_kwargs: 812)

    result = cleanup_owned_pid_file("native_child", pid_path)

    assert (812, signal.SIGTERM) in calls
    assert result["alive_before_cleanup"] is True
    assert result["term_sent"] is True
    assert result["clean"] is True


def test_prior_owned_pidfiles_are_reclaimed_before_artifact_reset(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    case_dir = tmp_path / "case"
    case_dir.mkdir()
    mapd_pid = case_dir / "mapd.pid"
    traversability_pid = case_dir / "traversability.pid"
    tap_pid = case_dir / "cmd_vel_tap.pid"
    mapd_pid.write_text("103\n", encoding="ascii")
    traversability_pid.write_text("101\n", encoding="ascii")
    tap_pid.write_text("102\n", encoding="ascii")
    observed: list[tuple[str, Path, float]] = []

    def cleanup(name: str, path: Path, *, pid_wait_s: float = 2.0, **_kwargs) -> dict:
        observed.append((name, path, pid_wait_s))
        return {"name": name, "pid_file": str(path), "clean": True}

    monkeypatch.setattr(acceptance, "cleanup_owned_pid_file", cleanup)
    monkeypatch.setattr(
        acceptance,
        "_inspect_pid_command",
        lambda pid: {
            "alive": True,
            "readable": True,
            "command": {
                101: "lingtu_traversability_dds --domain-id 226",
                102: "lingtu_mujoco_cmd_vel_tap --domain-id 226",
                103: "mapd --domain-id 226",
            }[pid],
        },
    )

    result = reclaim_prior_case_processes(
        case_dir,
        {
            "sensor_publisher_pid": case_dir / "sensor_publisher.pid",
            "cmd_vel_tap_pid": tap_pid,
        },
        {
            "prior_mapd": [
                "mapd",
                "--domain-id",
                "226",
            ],
            "prior_traversability": [
                "lingtu_traversability_dds",
                "--domain-id",
                "226",
            ],
            "prior_cmd_vel_tap": [
                "lingtu_mujoco_cmd_vel_tap",
                "--domain-id",
                "226",
            ],
        },
    )

    assert observed == [
        ("prior_mapd", mapd_pid, 0.1),
        ("prior_traversability", traversability_pid, 0.1),
        ("prior_cmd_vel_tap", tap_pid, 0.1),
    ]
    assert all(item["ownership_handle_reclaimed"] is True for item in result)


def test_prior_reclaim_never_signals_a_reused_unrelated_pid(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    case_dir = tmp_path / "case"
    case_dir.mkdir()
    pid_path = case_dir / "navigation.pid"
    pid_path.write_text("777\n", encoding="ascii")
    monkeypatch.setattr(native, "_read_linux_pid", lambda *_args, **_kwargs: 777)
    monkeypatch.setattr(
        acceptance,
        "_inspect_pid_command",
        lambda _pid: {
            "alive": True,
            "readable": True,
            "command": "unrelated_database_worker --domain-id 226",
        },
    )
    monkeypatch.setattr(
        acceptance,
        "cleanup_owned_pid_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("a reused PID must never be signalled")),
    )

    result = reclaim_prior_case_processes(
        case_dir,
        {
            "sensor_publisher_pid": case_dir / "sensor_publisher.pid",
            "cmd_vel_tap_pid": case_dir / "cmd_vel_tap.pid",
        },
        {
            "prior_navigation": [
                "navd",
                "--domain-id",
                "226",
            ]
        },
    )

    assert result[0]["action"] == "stale_reused_pid_handle_removed"
    assert result[0]["clean"] is True
    assert not pid_path.exists()
