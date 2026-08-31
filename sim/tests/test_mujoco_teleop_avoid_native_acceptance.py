from __future__ import annotations

import inspect
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

import sim.scripts.mujoco.teleop_avoid_native_acceptance as acceptance
from sim.scripts.mujoco.teleop_avoid_native_acceptance import (
    _binary_source_provenance,
    _path_lateral_offset_m,
    _with_native_env,
    build_parser,
    prepare_runtime,
)


def test_manifest_uses_scan_with_live_mapd_collision() -> None:
    manifest = json.loads(
        Path("config/runtime_graph/acceptance/mujoco_teleop_avoid_native_acceptance.json").read_text(
            encoding="utf-8"
        )
    )

    assert manifest["navigation_runtime"]["local_planner"] == "scan"
    assert "path_library" not in manifest["paths"]
    assert "rt/maps/local_collision" in manifest["contracts"]["navigation_inputs"]


def test_local_detour_is_measured_from_command_corridor_not_path_chord() -> None:
    diagonal_detour = [
        [0.0, 0.0, 0.0],
        [0.4, -0.4, 0.0],
        [0.8, -0.8, 0.0],
    ]

    assert _path_lateral_offset_m(diagonal_detour, 0.0) == pytest.approx(0.8)
    assert _path_lateral_offset_m(diagonal_detour, -math.pi / 4.0) == pytest.approx(0.0)


def test_attached_case_uses_split_product_processes_without_starting_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    readiness = SimpleNamespace(kind="file", target="nav.status.json")
    plan = SimpleNamespace(
        product="teleop_avoid",
        native_process_environment={"LINGTU_HOST_BOOT_ID": "boot"},
        processes=tuple(
            SimpleNamespace(
                name=name,
                command=SimpleNamespace(readiness=readiness),
            )
            for name in (
                "lidar_publisher",
                "imu_publisher",
                "camera_publisher",
                "nav_runtime",
            )
        ),
    )
    calls: list[tuple[str, ...]] = []

    def control(_binary, arguments, **_kwargs):
        calls.append(tuple(arguments))
        if arguments[0] == "stop":
            return {"returncode": 0, "stdout": "accepted stop: exact"}
        return {"returncode": 0, "stdout": "motion events"}

    monkeypatch.setattr(acceptance, "_run_control", control)
    monkeypatch.setattr(
        acceptance,
        "parse_operator_motion_events",
        lambda _output: [{"action": action, "accepted": True} for action in ("claim", "sample", "hold", "release")],
    )
    monkeypatch.setattr(
        acceptance,
        "_samples",
        lambda *_args, **_kwargs: [
            {
                "wall_s": float(index),
                "nav": {
                    "stamp_s": float(index),
                    "control_mode": "teleop_avoid",
                    "input_gate": {"ready": True},
                    "teleop": {
                        "reason": reason,
                        "obstacle_distance_m": -1.0,
                        "output": {"vx": 0.2, "vy": 0.1, "wz": 0.0},
                    },
                    "local_path_points": 4,
                    "local_path": [
                        [0.0, 0.0, 0.0],
                        [0.7, 1.0, 0.0],
                        [2.5, 1.0, 0.0],
                        [3.5, 0.0, 0.0],
                    ],
                    "control_authority": {"resume_required": False},
                },
            }
            for index, reason in enumerate(
                ("terrain_slow", "accepted", "accepted"),
                start=1,
            )
        ],
    )
    monkeypatch.setattr(acceptance, "_status", lambda _path: {"stamp_s": 10.0})
    monkeypatch.setattr(
        acceptance,
        "_fresh_zero",
        lambda *_args, **_kwargs: [
            {
                "stamp_s": 11.0 + index,
                "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            }
            for index in range(acceptance.POST_STOP_REQUIRED_STATUS_SAMPLES)
        ],
    )
    args = SimpleNamespace(domain_base=225, command_vx=0.18, duration_s=None)

    result = acceptance.run_attached(
        plan=plan,
        run_plan_path=tmp_path / "run-plan.json",
        product_session_id="s" * 32,
        prepared={
            "binaries": {"navigation_control": tmp_path / "navctl.exe"},
            "manifest": {
                "teleop_command": {"duration_s": 1.0},
                "robot_geometry": {
                    "vehicle_length_m": 1.0,
                    "vehicle_width_m": 0.6,
                }
            },
        },
        args=args,
    )

    assert result["ok"] is True
    assert result["sensor_processes"] == [
        "lidar_publisher",
        "imu_publisher",
        "camera_publisher",
    ]
    assert result["robot_geometry"] == {
        "vehicle_length_m": 1.0,
        "vehicle_width_m": 0.6,
    }
    assert [call[0] for call in calls] == ["operator-motion", "stop"]
    assert calls[0][calls[0].index("--duration-s") + 1] == "1.0"
    source = inspect.getsource(acceptance.run_attached)
    assert "Popen" not in source
    assert "ManagedProcess" not in source
    assert "execute_case" not in source


def test_binary_provenance_includes_native_client_shared_library(tmp_path) -> None:
    control = tmp_path / "lingtu_nav_control"
    client_library = tmp_path / "liblingtu_nav_client.so"
    control.write_bytes(b"control")
    client_library.write_bytes(b"client")

    provenance, blockers = _binary_source_provenance({"navigation_control": control})

    assert blockers == []
    dependency = provenance["navigation_control"]["runtime_dependencies"]
    assert dependency["lingtu_nav_client"]["path"] == str(client_library)
    assert dependency["lingtu_nav_client"]["size_bytes"] == len(b"client")
    dependency_specs = dependency["lingtu_nav_client"]["source_specs"]
    assert all(not Path(spec).as_posix().endswith("endpoint/tools/navctl.cpp") for spec in dependency_specs)


def test_binary_provenance_uses_windows_nav_client_dll_for_exe(tmp_path) -> None:
    control = tmp_path / "lingtu_nav_control.exe"
    client_library = tmp_path / "lingtu_nav_client.dll"
    control.write_bytes(b"control")
    client_library.write_bytes(b"client")

    provenance, blockers = _binary_source_provenance({"navigation_control": control})

    assert "native_runtime_dependency_missing:navigation_control" not in blockers
    dependency = provenance["navigation_control"]["runtime_dependencies"]
    assert dependency["lingtu_nav_client"]["path"] == str(client_library)
    assert dependency["lingtu_nav_client"]["size_bytes"] == len(b"client")


def test_binary_provenance_uses_current_native_source_tree(tmp_path: Path) -> None:
    navigation = tmp_path / "navd"
    navigation.write_bytes(b"navigation")

    provenance, blockers = _binary_source_provenance({"navigation": navigation})

    item = provenance["navigation"]
    normalized_specs = [Path(spec).as_posix() for spec in item["source_specs"]]
    assert any(spec.endswith("src/nav/cpp/endpoint") for spec in normalized_specs)
    assert any(spec.endswith("src/nav/cpp/cmake/NavCoreTargets.cmake") for spec in normalized_specs)
    assert any(spec.endswith("src/nav/inspection/CMakeLists.txt") for spec in normalized_specs)
    assert any(spec.endswith("src/maps/CMakeLists.txt") for spec in normalized_specs)
    retired_endpoint_root = "src/nav/services/" + "endpoint/cpp"
    assert all(retired_endpoint_root not in spec for spec in normalized_specs)
    excluded_specs = [Path(spec).as_posix() for spec in item["excluded_source_specs"]]
    assert any(spec.endswith("src/nav/cpp/endpoint/tools/navctl.cpp") for spec in excluded_specs)
    assert "missing_source_specs" not in item
    assert not any(blocker.startswith("native_source_spec_missing:navigation:") for blocker in blockers)


def test_binary_provenance_includes_slam_build_contract(tmp_path: Path) -> None:
    slam = tmp_path / "slamd"
    slam.write_bytes(b"slam")

    provenance, _blockers = _binary_source_provenance({"slam": slam})

    normalized_specs = [Path(spec).as_posix() for spec in provenance["slam"]["source_specs"]]
    assert any(spec.endswith("src/localization/slam/cpp/CMakeLists.txt") for spec in normalized_specs)
    assert any(
        spec.endswith("third_party/research_localization/small_gicp/include/small_gicp") for spec in normalized_specs
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
        fake_root / "src/maps/cpp/semantic_map_persistence.cpp",
        fake_root / "src/maps/include/lingtu/maps/semantic_map_persistence.hpp",
        fake_root / "src/native/snapshot_file.hpp",
        fake_root / "src/message/idl/messages.idl",
        fake_root / "src/message/cpp/CMakeLists.txt",
        fake_root / "src/message/cpp/topics.hpp",
        fake_root / "src/message/cpp/qos.hpp",
    )
    for source in source_files:
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("source", encoding="utf-8")
    small_gicp_header = (
        fake_root / "third_party/research_localization/small_gicp/include/small_gicp/pcl/pcl_registration.hpp"
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
            "src/native/snapshot_file.hpp",
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
        normalized_specs = [Path(spec).as_posix() for spec in provenance[artifact]["source_specs"]]
        assert all(any(spec.endswith(suffix) for spec in normalized_specs) for suffix in suffixes)
        assert "missing_source_specs" not in provenance[artifact]
        assert not any(blocker.startswith(f"native_source_spec_missing:{artifact}:") for blocker in blockers)


def test_formal_command_defaults_to_half_meter_per_second_with_override() -> None:
    defaults = build_parser().parse_args([])
    assert defaults.duration_s is None
    assert (defaults.command_vx, defaults.command_vy, defaults.command_wz) == (
        0.5,
        0.0,
        0.0,
    )
    overridden = build_parser().parse_args(
        [
            "--command-vx",
            "0.31",
            "--command-vy",
            "-0.2",
            "--command-wz",
            "0.7",
            "--duration-s",
            "12",
            "--start-yaw-deg",
            "90",
        ]
    )
    assert (
        overridden.command_vx,
        overridden.command_vy,
        overridden.command_wz,
        overridden.duration_s,
        overridden.start_yaw_deg,
    ) == (0.31, -0.2, 0.7, 12.0, 90.0)


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
    _free_binaries, free_paths, free_blockers, free_provenance = native._preflight_map_free({})

    assert any(blocker.startswith("map_artifact_missing:") for blocker in blockers)
    assert {"map_dir", "slam", "planner", "metadata"} <= set(map_paths)
    assert not any(blocker.startswith("map_artifact_missing:") for blocker in free_blockers)
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

    provenance, blockers = acceptance._binary_source_provenance({"navigation": navigation})

    item = provenance["navigation"]
    assert item["missing_source_specs"]
    assert "newer_than_sources" not in item
    assert any(blocker.startswith("native_source_spec_missing:navigation:") for blocker in blockers)


def test_binary_provenance_tracks_c_abi_header_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    nav_cpp = fake_root / "src" / "nav" / "cpp"
    client_dir = nav_cpp / "client"
    endpoint_dir = nav_cpp / "endpoint"
    common_files = (
        fake_root / "src" / "message" / "idl" / "messages.idl",
        fake_root / "src" / "message" / "cpp" / "CMakeLists.txt",
        fake_root / "src" / "message" / "cpp" / "topics.hpp",
        fake_root / "src" / "message" / "cpp" / "qos.hpp",
        fake_root / "src" / "message" / "cpp" / "exploration_command.hpp",
        fake_root / "src" / "message" / "cpp" / "inspection_command.hpp",
        fake_root / "src" / "message" / "cpp" / "navigation_command.hpp",
        fake_root / "src" / "message" / "cpp" / "operator_motion.hpp",
    )
    sources = (
        endpoint_dir / "tools" / "navctl.cpp",
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

    provenance, blockers = acceptance._binary_source_provenance({"navigation_control": control})

    assert provenance["navigation_control"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:navigation_control" in blockers
    assert "native_runtime_dependency_stale:navigation_control" in blockers


def test_binary_provenance_tracks_sensor_publisher_idl_changes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_root = tmp_path / "source-root"
    publisher_source = fake_root / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "main.cpp"
    publisher_cmake = publisher_source.with_name("CMakeLists.txt")
    common_files = (
        fake_root / "src" / "message" / "idl" / "messages.idl",
        fake_root / "src" / "message" / "cpp" / "CMakeLists.txt",
        fake_root / "src" / "message" / "cpp" / "topics.hpp",
        fake_root / "src" / "message" / "cpp" / "qos.hpp",
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

    provenance, blockers = acceptance._binary_source_provenance({"sensor_publisher": publisher})

    assert provenance["sensor_publisher"]["source_latest_mtime_ns"] == future_ns
    assert "native_binary_stale:sensor_publisher" in blockers


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


def test_manifest_keeps_mapd_scene_separate_from_navigation_safety_authority() -> None:
    manifest = json.loads(
        Path("config/runtime_graph/acceptance/mujoco_native_navigation_acceptance.json").read_text(encoding="utf-8")
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
                "fresh": True,
                "age_s": 0.01,
                "reason": reason,
                "request": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                "output": {"vx": output_vx, "vy": 0.0, "wz": 0.0},
            },
            "operator_motion": {
                "status": {
                    "observed": True,
                    "published": True,
                    "active_source_id": "acceptance-teleop",
                    "active_source_epoch": 1,
                    "has_active_authority": True,
                    "holding": False,
                    "has_active_sample": True,
                    "last_sample_sequence": command_count,
                    "admitted_sequence": command_count,
                    "final_output_sequence": command_count,
                }
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
    t: float = 0.0,
    x: float = 0.0,
    y: float = 0.0,
    cmd_vx: float = 0.0,
    lidar_world: list[list[float]] | None = None,
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
        "t": t,
        "qpos": [x, y, z, *quaternion],
        "cmd": [cmd_vx, 0.0, 0.0],
        "x": x,
        "y": y,
        "yaw": 0.0,
        "lidar_world": list(lidar_world or []),
    }


def _dynamic_scene_sample(wall_s: float, count: int) -> dict:
    return {
        "type": "scene",
        "wall_s": wall_s,
        "producer_boot_id": "mapd-boot",
        "reset_epoch": 1,
        "generation": int(wall_s * 10),
        "roi_counts": {
            "live": count,
            "voxel": count,
            "accumulated": count,
        },
    }


def test_native_stop_budget_exceeds_endpoint_confirmation_window() -> None:
    confirmation_ms = int(float(acceptance.FIELD_TELEOP_AVOID_PROFILE["stop_confirmation_timeout_s"]) * 1000)

    assert acceptance.NATIVE_STOP_CLIENT_TIMEOUT_MS > confirmation_ms
    assert acceptance.NATIVE_STOP_PROCESS_TIMEOUT_S > acceptance.NATIVE_STOP_CLIENT_TIMEOUT_MS / 1000.0
    assert acceptance.NATIVE_STOP_MARGIN_SIM_S > confirmation_ms / 1000.0
    assert acceptance._native_stop_arguments("cleanup") == ["stop", "cleanup", "--timeout-ms", "7000"]


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


def test_manifest_declares_windows_native_candidates_for_sensor_and_nav_bins() -> None:
    manifest = json.loads(acceptance.DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    assert manifest["binaries"]["sensor_publisher"]["candidates"][0] == (
        "build/windows-native-dds-adapter/Release/lingtu_mujoco_sensor_publisher.exe"
    )
    assert manifest["binaries"]["driver_bridge"]["env"] == ("LINGTU_MUJOCO_DRIVER_BRIDGE_BIN")
    assert manifest["binaries"]["driver_bridge"]["candidates"][0] == (
        "build/windows-native-dds-adapter/Release/lingtu_mujoco_driver_bridge.exe"
    )
    assert "cmd_vel_tap" not in manifest["binaries"]
    assert manifest["binaries"]["navigation"]["candidates"][0] == (
        "build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe"
    )
    assert manifest["binaries"]["traversability"]["candidates"][0] == (
        "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_traversability_dds.exe"
    )
    assert manifest["binaries"]["navigation_control"]["candidates"][0] == (
        "build/nav-cpp/windows-x64-nav-endpoint/Release/lingtu_nav_control.exe"
    )


def test_native_teleop_with_native_windows_exe_uses_popen_env_not_prefix(monkeypatch):
    monkeypatch.setattr(acceptance.os, "name", "nt")

    command, env = _with_native_env(
        ["C:\\build\\Release\\lingtu_traversability_dds.exe", "--domain-id", "226"],
        LINGTU_EXPLORE_ROUTE="live",
    )

    assert command == ["C:\\build\\Release\\lingtu_traversability_dds.exe", "--domain-id", "226"]
    assert env == {"LINGTU_EXPLORE_ROUTE": "live"}


def test_native_teleop_wsl_command_keeps_env_wrapper_for_linux_child(monkeypatch):
    monkeypatch.setattr(acceptance.os, "name", "nt")

    command, env = _with_native_env(
        ["C:\\Windows\\System32\\wsl.exe", "-e", "/home/sunrise/navd", "--domain-id", "226"],
        LINGTU_EXPLORE_ROUTE="live",
    )

    assert command == [
        "C:\\Windows\\System32\\wsl.exe",
        "-e",
        "env",
        "LINGTU_EXPLORE_ROUTE=live",
        "/home/sunrise/navd",
        "--domain-id",
        "226",
    ]
    assert env == {}


def test_prepare_runtime_keeps_mapd_but_omits_slam_for_truth_fixture(
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
        "slam_runtime": {"provider": "mujoco_navigation_fixture", "mode": "mapping"},
        "asset_builder": {"kind": "scene_only"},
        "binaries": {},
    }
    binaries = {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "driver_bridge",
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
            "--state-provider",
            "mujoco_navigation_fixture",
            "--artifact-dir",
            str(tmp_path / "artifacts"),
            "--preflight-only",
        ]
    )
    prepared = prepare_runtime(args)

    assert all(
        blocker not in prepared["blockers"]
        for blocker in ("native_binary_missing:mapd", "native_binary_missing:slam")
    )
    assert prepared["details"]["binaries"] == {key: str(path) for key, path in binaries.items()}


def test_preflight_rejects_mixed_native_boot_clock_platforms(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if os.name != "nt":
        pytest.skip("Windows/WSL split clock contract applies on Windows hosts")
    from sim.scripts.mujoco import native_navigation_acceptance as native

    binaries = {
        "sensor_publisher": tmp_path / "sensor_publisher.exe",
        "traversability": tmp_path / "traversability.exe",
        "navigation": tmp_path / "navd.exe",
        "navigation_control": tmp_path / "navigation_control.exe",
        "driver_bridge": tmp_path / "lingtu_mujoco_driver_bridge",
    }
    manifest = {
        "slam_runtime": {"provider": "mujoco_navigation_fixture", "mode": "mapping"},
    }
    monkeypatch.setattr(native, "_load_manifest", lambda _path: dict(manifest))
    monkeypatch.setattr(
        native,
        "_preflight_map_free",
        lambda _manifest: (binaries, {}, [], {}),
    )
    monkeypatch.setattr(
        acceptance,
        "_teleop_product_contract_evidence",
        lambda _manifest: {"blockers": []},
    )
    monkeypatch.setattr(
        acceptance,
        "_prepare_teleop_scene_asset",
        lambda _manifest, _artifact_dir: {"ok": True},
    )
    monkeypatch.setattr(
        acceptance,
        "_binary_source_provenance",
        lambda _binaries: ({}, []),
    )
    args = build_parser().parse_args(
        [
            "--artifact-dir",
            str(tmp_path / "artifacts"),
            "--state-provider",
            "mujoco_navigation_fixture",
            "--preflight-only",
        ]
    )

    prepared = prepare_runtime(args)

    assert prepared["ok"] is False
    assert "native_driver_clock_platform_mismatch" in prepared["blockers"]


def test_typed_operator_motion_cli_owns_fail_closed_lifecycle() -> None:
    source = Path("src/nav/cpp/endpoint/tools/navctl.cpp").read_text(encoding="utf-8")
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
    assert evidence["blockers"] == ["python_runtime_dependency_missing:onnxruntime"]


def test_scan_teleop_preflight_does_not_require_cmu_path_library(
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
        "navigation_runtime": {"local_planner": "scan"},
        "asset_builder": {"kind": "scene_only"},
        "binaries": {},
    }
    binaries = {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "mapd",
            "traversability",
            "navigation",
            "navigation_control",
            "driver_bridge",
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
            ["native_binary_missing:autonomy_only_tool"],
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

    assert prepared["ok"] is True
    assert "path_library" not in prepared["paths"]
    assert prepared["details"]["out_of_scope_preflight_findings"] == ["native_binary_missing:autonomy_only_tool"]


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
        "navigation_runtime": {"local_planner": "scan"},
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
            "driver_bridge",
        )
    }
    paths = {
        "world": world,
        "slam_config": tmp_path / "slam.yaml",
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


def test_native_sensor_start_yaw_and_report_tail_remain_in_run_scope() -> None:
    from sim.scripts.mujoco import native_dds_sensors as sensors

    args = sensors._build_parser().parse_args(["--start-yaw-deg", "-90"])
    run_source = inspect.getsource(sensors.run)

    assert args.start_yaw_deg == -90.0
    assert 'report["cmd_vel"] = driver_bridge_stats' in run_source
    assert run_source.rstrip().endswith("return report")
