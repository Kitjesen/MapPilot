from __future__ import annotations

import ast
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
_THUNDER_SERVICE_DIR = ROOT / "scripts" / "deploy" / "thunder"

# Bash-only constructs that systemd mangles before handing ExecStart to the shell.
# Field evidence (2026-07-04): ${VAR:-default} and ${args[@]} produced an empty
# positional arg and crash-looped lingtu-thunder-dds-endpoint for 29h.
_BASH_ONLY_EXECSTART_PATTERNS = (
    re.compile(r"\$\{[^}]*:-"),  # ${VAR:-default} / ${VAR:-${OTHER:-}}
    re.compile(r"\$\{args\[@\]\}"),
    re.compile(r"args=\("),
)


def _read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8-sig")


def _release_run_plan_loader_source() -> str:
    text = _read("scripts/deploy/cut_release.sh")
    loader = text.split("load_committed_run_plan() {", 1)[1]
    return loader.split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0]


def _service_environment_float(unit_text: str, name: str) -> float:
    prefix = f"Environment={name}="
    line = next(line for line in unit_text.splitlines() if line.startswith(prefix))
    return float(line.removeprefix(prefix))


def _authoritative_body_to_lidar_translation() -> tuple[float, float, float]:
    robot = yaml.safe_load((ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8"))
    lidar = robot["lidar"]
    return lidar["offset_x"], lidar["offset_y"], lidar["offset_z"]


@pytest.mark.parametrize(
    ("service", "environment_prefix"),
    [
        ("scripts/deploy/thunder/lingtu-nav-dds.service", "LINGTU_NAV_SENSOR_OFFSET"),
        (
            "scripts/deploy/thunder/lingtu-traversability-dds.service",
            "LINGTU_TRAVERSABILITY_SENSOR_OFFSET",
        ),
    ],
)
def test_thunder_native_services_use_authoritative_body_to_lidar_translation(
    service: str, environment_prefix: str
) -> None:
    text = _read(service)
    configured_sensor_origin = tuple(
        _service_environment_float(text, f"{environment_prefix}_{axis}_M") for axis in ("X", "Y", "Z")
    )
    assert configured_sensor_origin == pytest.approx(_authoritative_body_to_lidar_translation())


def _exec_start_lines(unit_text: str) -> list[str]:
    return [
        line
        for line in unit_text.splitlines()
        if line.startswith("ExecStart=") or line.startswith("ExecStartPre=") or line.startswith("ExecStartPost=")
    ]


def test_canonical_thunder_deploy_script_deploys_then_optionally_activates_product() -> None:
    text = _read("scripts/deploy/deploy_thunder.sh")

    assert "LINGTU_DEPLOY_PRODUCT" in text
    assert "LINGTU_DEPLOY_PROFILE" not in text
    assert "is_field_product()" in text
    assert "teleop|teleop_avoid|map|explore|nav|tracking|inspection" in text
    assert "tare_explore" not in text
    assert 'bash "${REPO}/scripts/lingtu" --env real mode switch "${PRODUCT}"' in text
    assert "LINGTU_DEPLOY_MAP" in text
    assert "LINGTU_DEPLOY_INSTALL_SERVICES" in text
    assert 'bash "${REPO}/scripts/deploy/thunder/install_services.sh"' in text
    assert "ProductControl" in text
    assert "git reset --hard" not in text
    assert "git pull --ff-only" in text
    assert "LINGTU_PROFILE" not in text
    assert "LINGTU_DATA_SOURCE:=" not in text
    assert "LINGTU_RUNTIME_CONTRACT:=" not in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=" not in text
    assert "LINGTU_ENDPOINT_CONTRACT:=thunder_lcm_v1" not in text
    assert '[ -f "/opt/ros/humble/setup.bash" ]' not in text
    assert "SOURCE_ROS2" not in text
    assert "ros2|sim_ros2|*-ros2|ros-compat|legacy" not in text
    assert "lite|thunder-lite|basic|thunder-basic" not in text
    assert "nohup" not in text
    assert "--daemon" not in text
    assert ".lingtu/run.json" not in text
    assert ".lingtu/run.pid" not in text
    assert "Building production navigation, driver, camera, and maps" in text
    assert 'bash "${REPO}/scripts/build/build_maps.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_nav_kernel.sh" --clean' in text
    assert 'bash "${REPO}/scripts/build/build_nav_endpoint.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_orbbec_native.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_camera_dds.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_driver.sh"' in text
    assert 'require_nav_kernel(context="Thunder deployment")' in text


def test_legacy_s100p_deploy_alias_is_removed() -> None:
    assert not (ROOT / "scripts" / "deploy" / "deploy_s100p.sh").exists()


def test_thunder_service_installer_defaults_to_native_field_cpp_stack() -> None:
    text = _read("scripts/deploy/thunder/install_services.sh")

    assert 'MODE="${1:-field-cpp}"' in text
    assert "runtime.service_catalogs.thunder install-services" in text
    assert "runtime.service_catalogs.thunder install-modes" in text
    assert 'catalog_services_for_mode "${MODE}"' in text
    assert 'run_catalog_services "${service_text}"' in text
    assert 'catalog_installer "${service}"' in text
    assert 'bash "${SCRIPT_DIR}/${installer}"' in text
    assert "install-plan" not in text
    assert "PYTHONPATH=" in text
    assert "camera-dds|native-camera|camera" not in text
    assert "slam-dds|cpp-slam" not in text
    assert "traversability-dds|terrain-dds" not in text
    assert "field|nav|thunder-nav|field-cpp|dds-cpp" not in text
    assert "lidar-dds|livox|livox-dds" not in text
    assert "nav-dds|cpp-nav" not in text
    assert "lingtu|app|runtime" not in text
    assert "install_" + ("lc" + "m") + "_endpoint_service.sh" not in text
    assert "install_lite_service.sh" not in text
    assert "Unknown Thunder service install mode" in text
    assert "Usage: $0 [<catalog-mode>]" in text
    assert "Catalog modes:" in text
    assert "Compatibility modes:" not in text
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" not in text
    assert "LEGACY_INSTALLER" not in text
    assert "ros-compat|legacy)" not in text
    assert "Refusing to install legacy ROS compatibility services by default" not in text
    assert "lite|thunder-lite|basic|thunder-basic" not in text


def test_thunder_ros2_env_helper_is_the_deployment_compat_boundary() -> None:
    text = _read("scripts/deploy/thunder/ros2-env.sh")

    assert "/opt/ros/${LINGTU_ROS_DISTRO}/setup.bash" in text
    assert "LINGTU_ROS_OVERLAY_SETUP" in text
    assert "RMW_IMPLEMENTATION" in text
    assert "Product entrypoints should use LingTu profiles" in text


def test_thunder_runtime_env_has_no_hidden_profile_or_deployment_identity() -> None:
    text = _read("scripts/deploy/thunder/runtime-env.sh")

    assert "LINGTU_ENV:=real" in text
    assert "LINGTU_PROFILE:=" not in text
    assert "LINGTU_MODULE_TRANSPORT:=local" in text
    assert "LINGTU_PROFILE_ADAPTER:=" not in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=local" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=" in text
    assert "LINGTU_SIMULATION_ONLY:=0" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER:=1" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE:=local_driver" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY:=module_graph_driver" in text
    assert (
        "LINGTU_MAP_ARTIFACT_CONVERTER:=${LINGTU_REPO}/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap"
    ) in text
    assert "LINGTU_MAPS_LIB:=${LINGTU_REPO}/build/maps/liblingtu_maps.so" in text
    assert "export LINGTU_MAPS_LIB" in text
    assert "/opt/ros" not in text
    assert "ROS_DOMAIN_ID" not in text
    assert "RMW_IMPLEMENTATION" not in text


def test_thunder_lite_has_no_independent_systemd_lifecycle() -> None:
    installer = _read("scripts/deploy/thunder/install_services.sh")

    assert not (ROOT / "scripts/deploy/thunder/lingtu-thunder-lite.service").exists()
    assert not (ROOT / "scripts/deploy/thunder/install_lite_service.sh").exists()
    assert "install_lite_service.sh" not in installer
    assert "lite|thunder-lite|basic|thunder-basic" not in installer


def test_thunder_main_lingtu_service_consumes_product_control_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu.service")
    wants = next(line for line in text.splitlines() if line.startswith("Wants="))
    after = next(line for line in text.splitlines() if line.startswith("After="))

    assert "Description=LingTu Python application host" in text
    assert wants == "Wants=network-online.target"
    assert after == "After=network-online.target"
    assert "lingtu-driver.service" not in wants + after
    assert "lingtu-slam-dds.service" not in wants + after
    assert "lingtu-nav-dds.service" not in wants + after
    assert "lingtu-thunder-dds-endpoint.service" not in text
    assert "LINGTU_ENV=real" in text
    assert "LINGTU_MODULE_TRANSPORT=local" in text
    assert "LINGTU_PROFILE=" not in text
    assert "LINGTU_PROFILE_ADAPTER=" not in text
    assert "LINGTU_PRODUCT=" not in text
    assert "LINGTU_SERVICE_DDS_CHECK=1" in text
    assert "LINGTU_DDS_PROBE_SCRIPT=/opt/lingtu/current/scripts/diagnostics/dds_probe.py" in text
    assert "LINGTU_DDS_PROBE_BIN=/opt/lingtu/current/build/dds_probe/lingtu_dds_probe" in text
    assert "LINGTU_DDS_PROBE_ALLOW_BUILD=0" in text
    assert "LINGTU_SERVICE_READINESS_JSON=/tmp/lingtu_service_readiness.json" in text
    assert "LINGTU_INSPECTION_LIBRARY=/opt/lingtu/current/build/nav_endpoint/inspection/liblingtu_inspection.so" in text
    assert (
        "LINGTU_INSPECTION_EVIDENCE_BRIDGE_LIBRARY="
        "/opt/lingtu/current/build/nav_endpoint/liblingtu_inspection_evidence_bridge.so" in text
    )
    assert "LINGTU_INSPECTION_EVIDENCE_DIR=/home/sunrise/data/lingtu/inspection_evidence" in text
    assert "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE=/dev/shm/lingtu/inspection_evidence_status.json" in text
    assert "LINGTU_CLOUD_VIEWER_MAX_HZ=2" in text
    assert "LINGTU_SCAN_VIEWER_MAX_HZ=10" in text
    assert "lingtu-teleop-dds.service" not in text
    assert "run_http_watchdog.sh" not in text
    assert "--readiness-url http://127.0.0.1:5050/ready" not in text
    assert "EnvironmentFile=/run/lingtu/session.env" in text
    assert "ExecStartPre=/bin/bash /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh host" in text
    assert 'lingtu.py "${LINGTU_PRODUCT}" --no-repl' in text
    assert "ros2-env.sh" not in text


@pytest.mark.parametrize(
    "rel_path",
    (
        "scripts/deploy/thunder/install_dds_endpoint_service.sh",
        "scripts/deploy/thunder/lingtu-thunder-dds-endpoint.service",
        "scripts/deploy/thunder/run_dds_endpoint_service.py",
    ),
)
def test_retired_python_dds_field_deployment_files_are_absent(rel_path: str) -> None:
    assert not (ROOT / rel_path).exists()


def test_thunder_driver_service_is_native_product_entrypoint() -> None:
    unit = _read("scripts/deploy/thunder/lingtu-driver.service")
    runner = _read("scripts/deploy/thunder/run_driver.sh")
    cmake = _read("src/drivers/real/thunder/native/CMakeLists.txt")
    source = _read("src/drivers/real/thunder/native/main.cpp")
    dds = _read("src/drivers/real/thunder/native/dds.cpp")
    status = _read("src/drivers/real/thunder/native/status.cpp")
    writer_gate = _read("src/drivers/real/thunder/native/cmd_vel_writer_gate.cpp")
    brainstem = _read("src/drivers/real/thunder/native/brainstem.cpp")
    core = _read("src/drivers/real/thunder/native/core.cpp")

    assert "Description=LingTu native Thunder driver" in unit
    assert "lingtu-nav-dds.service" not in unit
    assert "lingtu-thunder-lite.service" not in unit
    assert "LINGTU_DRIVER_BIN=/opt/lingtu/current/build/driver/lingtu_driver" in unit
    assert "LINGTU_DRIVER_CMD_TIMEOUT_MS=200" in unit
    assert "LINGTU_DRIVER_STATUS_FILE=/dev/shm/lingtu/driver_status.json" in unit
    assert "Conflicts=lingtu-thunder-dds-endpoint.service" in unit
    assert "run_driver.sh" in unit
    assert "python" not in unit.lower()
    assert "/opt/ros" not in unit
    assert "build_driver.sh" in runner
    assert 'exec "${BIN}"' in runner
    assert "find_package(CycloneDDS REQUIRED)" in cmake
    assert "find_package(gRPC CONFIG QUIET)" in cmake
    assert "pkg_check_modules(GRPCPP REQUIRED" in cmake
    assert "test_driver_core" in cmake
    assert "test_driver_io" in cmake
    assert "refresh_control(now)" in source
    assert "WalkChecked" in brainstem
    assert "stub_->Walk(" not in brainstem
    assert "best_effort_stop(ActionReason::Fault)" in source
    assert "best_effort_stop(ActionReason::Shutdown)" in source
    assert "dropped_disconnected" in source
    assert "core.poll(now)" in source
    assert "cmd_vel_writer_gate.update(dds.matchedCommandWriters())" in source
    assert "if (!writer_decision.ready)" in source
    assert "brainstem.release()" in source
    assert "dds_get_matched_publications(impl_->reader, nullptr, 0)" in dds
    assert 'return "missing_cmd_vel_writer"' in writer_gate
    assert 'return "ambiguous_cmd_vel_writers"' in writer_gate
    assert "matched_cmd_vel_writers" in status
    assert "cmd_vel_writer_ready" in status
    assert 'frame == "body" || frame == "base_link"' in core

    installer = _read("scripts/deploy/thunder/install_driver_service.sh")
    assert 'install_catalog_service.sh" driver' in installer
    assert "systemctl disable --now" in installer
    assert "lingtu-thunder-dds-endpoint.service" in installer


def test_thunder_service_units_avoid_bash_only_execstart_constructs() -> None:
    """systemd expands ${VAR} before bash; bash-only syntax crash-loops the unit."""
    unit_paths = sorted(_THUNDER_SERVICE_DIR.glob("*.service"))
    assert unit_paths, "expected thunder service unit files"

    offenders: list[str] = []
    for path in unit_paths:
        text = path.read_text(encoding="utf-8-sig")
        for line in _exec_start_lines(text):
            for pattern in _BASH_ONLY_EXECSTART_PATTERNS:
                if pattern.search(line):
                    offenders.append(f"{path.name}: {pattern.pattern}")

    assert not offenders, (
        "ExecStart uses bash-only constructs that systemd mangles "
        f"(use direct flags from Environment= instead): {offenders}"
    )

    # Guard: the old buggy dds-endpoint form must stay rejected.
    buggy = (
        'args=(--contract "${LINGTU_ENDPOINT_CONTRACT}"); '
        'sources="${LINGTU_ENDPOINT_SOURCES:-${LINGTU_ENDPOINT_SOURCE:-}}"; '
        'if [ -n "${sources}" ]; then args+=(--source "${sources}"); fi; '
        'exec "${LINGTU_PYTHON}" scripts/deploy/thunder/run_dds_endpoint_service.py '
        '"${args[@]}"'
    )
    for pattern in _BASH_ONLY_EXECSTART_PATTERNS:
        assert pattern.search(buggy), pattern.pattern


def test_thunder_slam_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu-slam-dds.service")
    runner = _read("scripts/deploy/thunder/run_slam_dds.sh")

    assert "Description=LingTu C++ CycloneDDS SLAM runtime" in text
    assert "After=network-online.target" in text
    assert "Wants=network-online.target" in text
    assert "After=network-online.target lingtu-livox-dds.service" not in text
    assert "Wants=network-online.target lingtu-livox-dds.service" not in text
    assert "systemd-time-wait-sync.service" not in text
    assert "wait_for_time_sync.sh" not in text
    assert "time-sync.target" not in text
    assert "lingtu-livox-dds.service" not in text
    assert "robot-lidar.service" not in text
    assert "LINGTU_SLAM_BIN=/opt/lingtu/current/build/slam_core/slamd" in text
    assert "LINGTU_SLAM_BACKEND=fastlio2" in text
    assert "LINGTU_SLAM_MODE=localization" in text
    assert "LINGTU_SLAM_MAP=/home/sunrise/data/nova/maps/active/map.pcd" in text
    assert "LINGTU_SLAM_TRACK_SEED_FILE=/home/sunrise/data/nova/maps/active/track_seed.json" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_X=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_Y=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_Z=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_YAW=" not in text
    assert "LINGTU_SLAM_CONFIG=/opt/lingtu/current/src/localization/fastlio2/config/mid360_s100p.yaml" in text
    assert "LINGTU_DDS_DOMAIN_ID=0" in text
    assert "LINGTU_SLAM_STATUS_JSON=/tmp/lingtu_slam_status.json" in text
    assert "LINGTU_SLAM_LIDAR_SCAN_SNAPSHOT_HZ=10" in text
    assert "run_slam_dds.sh" in text
    assert "native SLAM DDS runtime is missing or not executable" in runner
    assert "build_slam_core.sh" in runner
    assert "--domain-id" in runner
    assert "--mode" in runner
    assert '--map "$LINGTU_SLAM_MAP"' in runner
    assert "--track-against-map-seed-file" in runner
    assert 'track_seed_file="$(dirname "$LINGTU_SLAM_MAP")/track_seed.json"' in runner
    assert "--track-against-map-initial-pose" in runner
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" not in text
    assert "source /opt/lingtu/config/ros2-env.sh" not in text
    assert "python" not in text.lower()
    assert "ros2 run fastlio2" not in text
    assert "rclcpp" not in text


def test_slam_runtime_uses_persisted_track_seed_before_fallback_seed() -> None:
    text = _read("src/localization/slam/cpp/cyclone_runtime.cpp")

    assert "loadTrackSeed(cli.track_against_map_seed_file, cli.map_path)" in text
    assert "saveTrackSeed(cli.track_against_map_seed_file, cli.map_path" in text
    assert "track_against_map_initial_pose" in text
    assert "startup_track_seed.has_value()" in text
    assert "std::optional<Pose3d> track_against_map_seed = startup_track_seed" in text


def test_slam_track_against_map_waits_for_inputs_without_disabling() -> None:
    text = _read("src/localization/slam/cpp/cyclone_runtime.cpp")

    assert "bool isTrackAgainstMapInputWait" in text
    assert 'note_track_wait("registered_cloud_unavailable")' in text
    assert 'note_track_wait("registered_cloud_stale")' in text
    assert "isTrackAgainstMapInputWait(start_status.message)" in text
    assert "note_track_failure(start_status.message)" in text
    assert "backend->pollRelocalizeAsync()" in text
    assert "ProjectMapTrackingHealth" in text
    assert 'note_track_failure("registered_cloud_unavailable")' not in text
    assert 'note_track_failure("registered_cloud_stale")' not in text


def test_lidar_network_service_waits_for_managed_eth1_before_secondary_address() -> None:
    text = _read("scripts/deploy/setup_network.sh")

    assert "NetworkManager-wait-online.service" in text
    assert "ip addr replace ${HOST_IP}/${SUBNET_MASK} dev \\$IFACE" in text
    assert "ip route replace ${LIDAR_SUBNET} dev \\$IFACE src ${HOST_IP}" in text
    assert "nmcli device set \\$IFACE managed no" not in text
    assert "ip link set \\$IFACE down" not in text


def test_native_mapd_service_is_packaged_as_a_strict_cpp_boundary() -> None:
    unit = _read("scripts/deploy/thunder/mapd.service")
    installer = _read("scripts/deploy/thunder/install_mapd_service.sh")
    build = _read("scripts/build/build_mapd.sh")
    cmake = _read("src/maps/CMakeLists.txt")
    topics = _read("src/message/cpp/dds_topics.hpp")

    assert "Description=Native live maps and map service runtime" in unit
    assert "After=network-online.target" in unit
    assert "lingtu-slam-dds.service" not in unit
    assert "LINGTU_MAPD_BIN=/opt/lingtu/current/build/maps/mapd" in unit
    assert "LINGTU_MAPD_STATUS_FILE=/dev/shm/lingtu/mapd_status.json" in unit
    assert "LINGTU_MAPD_QUERY_SOCKET=/run/lingtu-mapd/mapd.sock" in unit
    assert ' --query-socket "${LINGTU_MAPD_QUERY_SOCKET}"' in unit
    assert "Group=sunrise" in unit
    assert "User=" not in unit
    assert "\nRuntimeDirectory=lingtu-mapd\n" in unit
    assert "RuntimeDirectoryMode=0770" in unit
    assert "UMask=0007" in unit
    assert "LINGTU_MAPD_STATE_HZ=2" in unit
    assert "LINGTU_MAPD_CLOUD_HZ=10" in unit
    assert "LINGTU_MAPD_MAP_HZ=2" in unit
    assert "LINGTU_MAPD_SCENE_HZ=2" in unit
    assert "LINGTU_MAPD_MAX_POINTS=300000" in unit
    assert "LINGTU_MAPD_MAX_SCENE_BYTES=33554432" in unit
    assert "LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS=200000" in unit
    assert "LINGTU_MAPD_MAX_VOXELS=500000" in unit
    assert "LINGTU_MAPD_MAX_ACCUMULATED_CELLS=2000000" in unit
    assert "LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS=4096" in unit
    assert "LINGTU_MAPD_CARVE_MIN_Z_M=-0.7" in unit
    assert "ExecStartPre=/bin/rm -f /dev/shm/lingtu/mapd_status.json" in unit
    assert "native maps runtime is missing or not executable" in unit
    assert "python" not in unit.lower()

    assert 'install_catalog_service.sh" maps' in installer
    assert "-DLINGTU_MAPS_BUILD_MAPD=ON" in build
    for target in (
        "mapd",
        "lingtu_maps_c_api",
        "lingtu_maps_mapd_engine_test",
        "lingtu_maps_mapd_dds_test",
        "lingtu_maps_mapd_uds_test",
        "lingtu_maps_service_c_api_test",
    ):
        assert target in build
    assert "ctest --test-dir" in build
    assert "add_executable(mapd" in cmake
    for topic in (
        "kMapsState",
        "kMapsLiveCloud",
        "kMapsVoxelCloud",
        "kMapsAccumulatedCloud",
        "kMapsOccupancy",
        "kMapsElevation",
        "kMapsEsdf",
        "kMapsScene",
    ):
        assert topic in topics


def test_thunder_traversability_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu-traversability-dds.service")
    installer = _read("scripts/deploy/thunder/install_traversability_dds_service.sh")
    source = _read("src/nav/cpp/endpoint/traversability/traversability_dds.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    topics = _read("src/message/cpp/dds_topics.hpp")
    idl = _read("src/message/idl/lingtu_slam.idl")

    assert "Description=LingTu native traversability DDS producer" in text
    assert "lingtu-slam-dds.service" not in text
    assert "require_product_session.sh traversability" in text
    assert "LINGTU_TRAVERSABILITY_DDS_BIN=/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds" in text
    assert "LINGTU_TRAVERSABILITY_PUBLISH_HZ=10" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MAP_HZ=5" in text
    assert '--slow-hz "${LINGTU_TRAVERSABILITY_TERRAIN_MAP_HZ}"' in text
    assert "LINGTU_TRAVERSABILITY_TICK_HZ=50" in text
    assert "LINGTU_TRAVERSABILITY_STATUS_FILE=/dev/shm/lingtu/traversability_status.json" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_DECAY_S=2.0" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_BLOCK_POINTS=10" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_QUANTILE=0.25" in text
    assert "LINGTU_TRAVERSABILITY_RADIUS=6" in text
    assert "LINGTU_TRAVERSABILITY_MAX_POINTS=5000" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_CACHE_MAX_POINTS=20000" in text
    assert "LINGTU_TRAVERSABILITY_ROBOT_RADIUS_M=0.45" in text
    assert '--robot-radius "${LINGTU_TRAVERSABILITY_ROBOT_RADIUS_M}"' in text
    assert "LINGTU_TRAVERSABILITY_OBSERVED_FREE_TTL_S=0.60" in text
    assert '--observed-free-ttl-s "${LINGTU_TRAVERSABILITY_OBSERVED_FREE_TTL_S}"' in text
    assert "LINGTU_TRAVERSABILITY_CLOUD_POSE_MAX_GAP_S=0.10" in text
    assert '--cloud-pose-max-gap-s "${LINGTU_TRAVERSABILITY_CLOUD_POSE_MAX_GAP_S}"' in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_SOFT_HEIGHT_M=0.08" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_HARD_HEIGHT_M=0.20" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_SOFT_SLOPE_DEG=12" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_HARD_SLOPE_DEG=28" in text
    assert '--terrain-soft-height-m "${LINGTU_TRAVERSABILITY_TERRAIN_SOFT_HEIGHT_M}"' in text
    assert '--terrain-hard-height-m "${LINGTU_TRAVERSABILITY_TERRAIN_HARD_HEIGHT_M}"' in text
    assert '--terrain-soft-slope-deg "${LINGTU_TRAVERSABILITY_TERRAIN_SOFT_SLOPE_DEG}"' in text
    assert '--terrain-hard-slope-deg "${LINGTU_TRAVERSABILITY_TERRAIN_HARD_SLOPE_DEG}"' in text
    configured_sensor_origin = tuple(
        _service_environment_float(text, f"LINGTU_TRAVERSABILITY_SENSOR_OFFSET_{axis}_M") for axis in ("X", "Y", "Z")
    )
    assert configured_sensor_origin == pytest.approx(_authoritative_body_to_lidar_translation())
    assert '--sensor-offset-x-m "${LINGTU_TRAVERSABILITY_SENSOR_OFFSET_X_M}"' in text
    assert '--sensor-offset-y-m "${LINGTU_TRAVERSABILITY_SENSOR_OFFSET_Y_M}"' in text
    assert '--sensor-offset-z-m "${LINGTU_TRAVERSABILITY_SENSOR_OFFSET_Z_M}"' in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS=1" in text
    assert '--terrain-clear-dy-obs "${LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS}"' in text
    assert "$${LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS}" not in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_DY_OBS_DIS=0.30" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_DY_OBS_ANGLE=0.0" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_DY_OBS_REL_Z=-0.5" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_ABS_DY_OBS_REL_Z_THRE=0.2" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_DY_OBS_VFOV=-16.0" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MAX_DY_OBS_VFOV=16.0" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_MIN_DY_OBS_POINT_NUM=1" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_NO_DATA_OBSTACLE=0" in text
    assert "LINGTU_TRAVERSABILITY_TERRAIN_NO_DATA_BLOCK_SKIP_NUM=0" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR=1" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_VOXEL_SIZE=0.20" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_WEAK_TTL_S=0.80" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_STATIC_TTL_S=3.0" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_STATIC_MIN_HITS=3" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_STATIC_MIN_FRAMES=2" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST=1" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST_MIN_FRAMES=2" in text
    assert "LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST_MAX_RANGE=6.0" in text
    assert "--publish-hz" in text
    assert "--tick-hz" in text
    assert "--resolution" in text
    assert "--radius" in text
    assert "--max-points" in text
    assert "--terrain-decay-s" in text
    assert "--terrain-min-block-points" in text
    assert "--terrain-quantile" in text
    assert "--terrain-cache-max-points" in text
    assert "--terrain-clear-dy-obs" in text
    assert "--terrain-min-dy-obs-dis" in text
    assert "--terrain-min-dy-obs-angle" in text
    assert "--terrain-min-dy-obs-rel-z" in text
    assert "--terrain-abs-dy-obs-rel-z-thre" in text
    assert "--terrain-min-dy-obs-vfov" in text
    assert "--terrain-max-dy-obs-vfov" in text
    assert "--terrain-min-dy-obs-point-num" in text
    assert "--terrain-no-data-obstacle" in text
    assert "--terrain-no-data-block-skip-num" in text
    assert "--dynamic-clear" in text
    assert "--dynamic-clear-voxel-size" in text
    assert "--dynamic-clear-weak-ttl-s" in text
    assert "--dynamic-clear-static-ttl-s" in text
    assert "--dynamic-clear-static-min-hits" in text
    assert "--dynamic-clear-static-min-frames" in text
    assert "--dynamic-clear-raycast" in text
    assert "--dynamic-clear-raycast-min-frames" in text
    assert "--dynamic-clear-raycast-max-range" in text
    assert "native traversability DDS producer is missing or not executable" in text
    assert "build_nav_endpoint.sh" in text
    assert "ros2-env.sh" not in text
    assert "python" not in text.lower()
    assert "install_catalog_service.sh" in installer
    assert " traversability" in installer
    assert "LINGTU_ENABLE_SERVICE_DEFAULT" not in installer
    assert "install-enable-default" in _read("scripts/deploy/thunder/install_catalog_service.sh")
    assert "double publish_hz{10.0}" in source
    assert "double slow_hz{1.0}" in source
    assert "double tick_hz{50.0}" in source
    assert "slow_tolerance_s" in source
    assert "next_slow_publish += slow_period_s" in source
    assert "next_slow_publish = now + 1.0 / cfg.slow_hz" not in source
    assert "kNavTerrainMap" in source
    assert "kNavTerrainMapExt" in source
    assert "kNavMapClearing" in source
    assert "kNavCloudClearing" in source
    assert '#include "message/cpp/dds_qos_profiles.hpp"' in source
    assert "qos_for_topic(topic_name)" in source
    assert "dds_qset_reliability" not in source
    assert "writeTerrainMap" in source
    assert "writeTerrainMapExt" in source
    assert "drainMapClearing" in source
    assert "drainCloudClearing" in source
    assert "TerrainAnalysisCore terrain_core" in source
    assert "DynamicClearCore dynamic_clear" in source
    assert "terrainParamsFromConfig" in source
    assert "dynamicClearParamsFromConfig" in source
    assert "params.maxPointsPerVoxel" in source
    assert "params.maxRayCount" in source
    assert "dynamic_clear_raycast{true}" in source
    assert "dynamic_clear_raycast_min_frames{2}" in source
    assert "terrain_core.process" in source
    assert "return nav_endpoint::rigidTransformFromOdometry(msg);" in source
    assert "traversabilityPose(*cloud_map_body)" in source
    assert "pose.roll" in source
    assert "pose.pitch" in source
    assert "cloud_sensor_origin->x" in source
    assert "makeUnknownSafetyGrid" in source
    assert "ObservedFreeCache" in source
    assert "observed_free.observeRay" in source
    assert "observed_free.apply" in source
    assert "observed_free.clear()" in source
    assert "terrain_core.clear()" in source
    assert "dynamic_clear.reset()" in source
    assert "DynamicClearOrigin clear_origin" in source
    assert "dynamic_clear.filter(" in source
    assert "cloud_stamp_s);" in source
    assert "TransformBuffer map_body_buffer" in source
    assert "map_body_buffer.sample(cloud_stamp_s, cfg.cloud_pose_max_gap_s)" in source
    assert "xyzToXyzi" in source
    assert "limitXyzi(clear_result.kept_xyzi, cfg.max_points)" in source
    assert "combineTerrainExt" in source
    assert "buildTerrainMapCloud" not in source
    assert "terrain_min_block_points{10}" in source
    assert "terrain_clear_dy_obs{false}" in source
    assert "terrain_no_data_obstacle{false}" in source
    assert "dynamic_clear{true}" in source
    assert "terrain_decay_s{2.0}" in source
    assert '\\"terrain_clear_dy_obs\\"' in source
    assert '\\"terrain_no_data_obstacle\\"' in source
    assert '\\"dynamic_clear\\"' in source
    assert "terrain_map_ext_points" in source
    assert "dynamic_clear_points" in source
    assert "dynamic_clear_ray_points" in source
    assert "dynamic_clear_evidence_voxels" in source
    assert "dynamic_clear_free_voxels" in source
    assert '\\"terrain_core\\"' in source
    assert '\\"dynamic_clear\\"' in source
    assert '\\"terrain_pack\\"' in source
    assert "terrain_core.storedPointCount()" in source
    assert "updateTerrainCache" not in source
    assert '"map_clearing"' in source
    assert '"cloud_clearing"' in source
    assert "find_package(OpenMP QUIET)" in cmake
    assert "lingtu_dds_qos_profiles" in cmake
    assert "target_link_libraries(lingtu_traversability_dds PRIVATE OpenMP::OpenMP_CXX)" in cmake
    assert '"/nav/map_clearing", "rt/nav/map_clearing"' in topics
    assert '"/nav/cloud_clearing", "rt/nav/cloud_clearing"' in topics
    assert "struct Bool" in idl


def test_thunder_livox_dds_service_publishes_native_sdk2_stream() -> None:
    text = _read("scripts/deploy/thunder/lingtu-livox-dds.service")
    runner = _read("scripts/deploy/thunder/run_livox_dds.sh")
    installer = _read("scripts/deploy/thunder/install_livox_dds_service.sh")
    time_sync_waiter = _read("scripts/deploy/thunder/wait_for_time_sync.sh")

    assert "Description=LingTu native Livox SDK2 DDS publisher" in text
    assert "After=network-online.target" in text
    assert "Wants=network-online.target" in text
    assert "systemd-time-wait-sync.service" not in text
    assert "time-sync.target" not in text
    assert "Environment=LINGTU_TIME_SYNC_WAIT_SECONDS=8" in text
    assert ("ExecStartPre=-/bin/bash /opt/lingtu/current/scripts/deploy/thunder/wait_for_time_sync.sh") in text
    assert "LINGTU_TIME_SYNC_WAIT_MAX_SECONDS=15" in time_sync_waiter
    assert "LINGTU_TIME_SYNC_WAIT_MARKER:-/run/lingtu/time-sync-wait.done" in time_sync_waiter
    assert "trap mark_wait_complete EXIT" in time_sync_waiter
    assert 'timeout --signal=TERM --kill-after=1s "${wait_seconds}s"' in time_sync_waiter
    assert "NTPSynchronized" in time_sync_waiter
    assert "continuing without synchronized wall clock" in time_sync_waiter
    assert "systemctl" not in time_sync_waiter
    assert time_sync_waiter.rstrip().endswith("exit 0")
    assert "LINGTU_LIVOX_BIN=/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream" in text
    assert "LINGTU_LIVOX_CONFIG_DIR=/opt/lingtu/config/livox" in text
    assert "LINGTU_LIVOX_NET_IFACE=eth1" in text
    assert "LINGTU_LIVOX_LIDAR_FRAME=lidar_link" in text
    assert "run_livox_dds.sh" in text
    assert "native Livox DDS publisher is missing or not executable" in runner
    assert "build_livox_sdk2_stream.sh" in runner
    assert "Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json" not in text
    assert "ensure_mid360_config_file" in runner
    assert "select_livox_host_ip" in runner
    assert "LINGTU_LIVOX_HOST_IP" in runner
    assert "LINGTU_LIVOX_LIDAR_FRAME:=lidar_link" in runner
    assert "LINGTU_LIVOX_IMU_HZ" in runner
    assert "head -n 1" not in runner
    assert "--dds" in runner
    assert "--domain-id" in runner
    assert "--publish-freq" in runner
    assert "--imu-publish-freq" in runner
    assert "--lidar-frame" in runner
    assert "--imu-frame" in runner
    assert "source /opt/lingtu/config/thunder-runtime-env.sh" in runner
    assert "ros2-env.sh" not in text
    assert "ros2-env.sh" not in runner
    assert "livox_ros_driver2" not in text
    assert "livox_ros_driver2" not in runner
    assert "install_catalog_service.sh" in installer
    assert "exec bash" in installer
    assert " lidar" in installer
    assert "lingtu-thunder-dds-endpoint.service" not in installer
    assert "LINGTU_LIVOX_DDS_SERVICE_NAME" in installer


@pytest.mark.skipif(os.name == "nt", reason="Runtime contract requires native Bash")
def test_time_sync_waiter_runtime_contract() -> None:
    completed = subprocess.run(
        [
            "bash",
            str(_THUNDER_SERVICE_DIR / "tests" / "test_wait_for_time_sync.sh"),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert "wait_for_time_sync tests passed" in completed.stdout


def test_thunder_camera_dds_service_is_optional_and_fails_without_native_publisher() -> None:
    text = _read("scripts/deploy/thunder/lingtu-camera-dds.service")
    runner = _read("scripts/deploy/thunder/run_camera_dds.sh")
    installer = _read("scripts/deploy/thunder/install_camera_dds_service.sh")
    build = _read("scripts/build/build_camera_dds.sh")
    cmake = _read("src/drivers/real/camera/native/CMakeLists.txt")
    nav_cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    source = _read("src/drivers/real/camera/native/camera_dds.cpp")
    topics = _read("src/message/cpp/dds_topics.hpp")
    idl = _read("src/message/idl/lingtu_slam.idl")

    assert "Description=LingTu native camera DDS publisher" in text
    assert "LINGTU_CAMERA_DDS_BIN=/opt/lingtu/current/build/camera_dds/lingtu_camera_dds" in text
    assert "LINGTU_ORBBEC_CAPTURE_BIN=/opt/lingtu/current/build/orbbec_native/orbbec_capture" in text
    assert "LINGTU_ORBBEC_PRODUCT_ID=0x0800" in text
    assert "LINGTU_ORBBEC_DEVICE_INDEX=0" in text
    assert "LINGTU_ORBBEC_CONNECT_TIMEOUT_MS=10000" in text
    assert "LINGTU_CAMERA_COLOR_WIDTH=640" in text
    assert "LINGTU_CAMERA_DEPTH_WIDTH=640" in text
    assert "run_camera_dds.sh" in text
    assert "native camera DDS publisher is missing or not executable" in runner
    assert "build_camera_dds.sh" in runner
    assert "build_orbbec_native.sh" in runner
    assert "--domain-id" in runner
    assert "--capture-bin" in runner
    assert "--frame-id" in runner
    assert "--color-topic" in runner
    assert "--depth-topic" in runner
    assert "--info-topic" in runner
    assert "--status-file" in runner
    assert "capture_args=(" in runner
    assert "--color-width" in runner
    assert "--color-height" in runner
    assert "--color-fps" in runner
    assert "--depth-width" in runner
    assert "--depth-height" in runner
    assert "--depth-fps" in runner
    assert "--connect-timeout-ms" in runner
    assert "--timeout-ms" in runner
    assert "--serial-number" in runner
    assert "--uid" in runner
    assert "--product-id" in runner
    assert "--device-index" in runner
    assert "--sdk-config" in runner
    assert "--enable-frame-sync" in runner
    assert '-- "${capture_args[@]}"' in runner
    assert "${LINGTU_CAMERA_FRAME_ID}" in runner
    assert "${LINGTU_CAMERA_STATUS_FILE}" in runner
    assert "${LINGTU_CAMERA_COLOR_TOPIC}" in runner
    assert "${LINGTU_CAMERA_DEPTH_TOPIC}" in runner
    assert "${LINGTU_CAMERA_INFO_TOPIC}" in runner
    assert "${LINGTU_CAMERA_COLOR_WIDTH}" in runner
    assert "${LINGTU_CAMERA_DEPTH_WIDTH}" in runner
    assert "${LINGTU_ORBBEC_CONNECT_TIMEOUT_MS}" in runner
    assert "ros2-env.sh" not in text
    assert "OrbbecSDK_ROS2" not in text
    assert "install_catalog_service.sh" in installer
    assert "exec bash" in installer
    assert " camera" in installer
    assert "LINGTU_ENABLE_SERVICE_DEFAULT" not in installer
    helper = _read("scripts/deploy/thunder/install_catalog_service.sh")
    assert "REQUESTED_ENABLE=\"${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}\"" in helper
    assert "--target lingtu_camera_dds" in build
    assert "LINGTU_CYCLONEDDS_PREFIX" in build
    assert "CMAKE_PREFIX_PATH" in build
    assert "include/${multiarch}" in build
    assert "--parallel" in build
    assert "native camera DDS publisher is missing" in build
    assert "add_executable(lingtu_camera_dds" in cmake
    assert "camera_dds.cpp" in cmake
    assert "src/drivers/real/camera/native" not in nav_cmake
    assert "lingtu_dds_Image_desc" in source
    assert "lingtu_dds_CameraInfo_desc" in source
    assert "execv(cfg.capture_bin.c_str()" in source
    assert "RecordHeader" in source
    assert "header.dist_k1" in source
    assert "header.dist_k2" in source
    assert "header.dist_p1" in source
    assert "header.dist_p2" in source
    assert "header.dist_k3" in source
    assert "d.assign(5, 0.0)" not in source
    assert "dds_write(camera_color)" in source
    assert "dds_write(camera_depth)" in source
    assert "dds_write(camera_info)" in source
    assert '"/camera/color/image_raw", "rt/camera/color"' in topics
    assert '"/camera/depth/image_raw", "rt/camera/depth"' in topics
    assert '"/camera/color/camera_info", "rt/camera/info"' in topics
    assert "struct Image" in idl
    assert "struct CameraInfo" in idl


def test_thunder_nav_dds_service_enables_bounded_local_planner_diagnostics() -> None:
    text = _read("scripts/deploy/thunder/lingtu-nav-dds.service")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    assert "LINGTU_NAV_STATUS_S=0.2" in text
    assert "LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES=18" in text
    assert "LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS=640" in text
    assert '--local-planner-debug-candidates "${LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES}"' in runner
    assert '--local-map-debug-points "${LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS}"' in runner


def test_thunder_nav_dds_service_diagnoses_missing_endpoint_binary() -> None:
    text = _read("scripts/deploy/thunder/lingtu-nav-dds.service")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")
    endpoint_bootstrap = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    endpoint_loop = _read("src/nav/cpp/endpoint/endpoint_loop.cpp")
    input_projector = _read("src/nav/cpp/endpoint/input/nav_input_state_projector.cpp")
    source = "\n".join(
        [
            endpoint_bootstrap,
            input_projector,
            _read("src/nav/cpp/endpoint/motion/autonomy_tick_controller.cpp"),
            _read("src/nav/cpp/endpoint/motion/teleop_safety.cpp"),
            _read("src/nav/cpp/endpoint/plan/planner_inputs.hpp"),
            _read("src/nav/cpp/endpoint/endpoint_time.hpp"),
            _read("src/nav/cpp/endpoint/endpoint_config.hpp"),
        ]
    )
    config_source = _read("src/nav/cpp/endpoint/nav_endpoint_config.cpp")

    assert "Description=LingTu native navigation DDS endpoint" in text
    assert "Wants=network-online.target lingtu-traversability-dds.service" not in text
    assert "LINGTU_NAV_DDS_BIN=/opt/lingtu/current/build/nav_endpoint/navd" in text
    assert "LINGTU_LOCAL_PLANNER_PATHS=/opt/lingtu/current/src/nav/local/paths" in text
    assert "LINGTU_NAV_GLOBAL_PLANNER=octoplanner3d" in text
    assert "LINGTU_ACTIVE_OCTOMAP=" in text
    assert "LINGTU_ACTIVE_OCCUPANCY=" in text
    assert "EnvironmentFile=-/etc/lingtu/nav.env" in text
    assert "run_nav_dds.sh" in text
    assert "LINGTU_NAV_CONTROL_MODE=autonomy" in text
    assert "LINGTU_NAV_MAX_SPEED_MPS=0.4" in text
    assert "LINGTU_NAV_MAX_ACCEL_MPS2=1.0" in text
    assert "LINGTU_NAV_PUBLISH_CMD_VEL=1" in text
    assert "LINGTU_NAV_CHECK_OBSTACLE=1" in text
    assert "LINGTU_NAV_USE_TRAVERSABILITY_COST=1" in text
    assert "LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S=1.5" in text
    assert "LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S=0.5" in text
    assert "LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S=1.5" in text
    assert "LINGTU_NAV_ODOM_MAX_AGE_S=0.25" in text
    assert "LINGTU_NAV_TF_MAX_AGE_S=0.25" in text
    assert "LINGTU_NAV_CLOUD_MAX_AGE_S=0.35" in text
    assert "LINGTU_NAV_CLOUD_POSE_MAX_GAP_S=0.10" in text
    assert "LINGTU_NAV_INPUT_RECOVERY_FRAMES=3" in text
    assert "LINGTU_NAV_OBSTACLE_TERRAIN_EXT_SHARE=0.0" in text
    assert "LINGTU_NAV_STATUS_FILE=/dev/shm/lingtu/nav_endpoint_status.json" in text
    assert "StateDirectory=lingtu" in text
    assert "LINGTU_NAV_ESTOP_LATCH_FILE=/var/lib/lingtu/nav_estop_latched" in text
    assert "LINGTU_NAV_STATUS_S=0.2" in text
    assert "--path-library" in runner
    assert "--control-mode" in runner
    assert "--global-planner" in runner
    assert "LINGTU_ACTIVE_PLANNER_MAP" in runner
    assert "--max-speed-mps" in config_source
    assert "--max-accel-mps2" in config_source
    assert "--max-obstacle-points" in runner
    assert "--publish-cmd-vel" in runner
    assert "--check-obstacle" in runner
    assert "--use-traversability-cost" in runner
    assert "--traversability-max-age-s" in runner
    assert "--localization-health-max-age-s" in runner
    assert "--terrain-map-max-age-s" in runner
    assert "--odom-max-age-s" in runner
    assert "--tf-max-age-s" in runner
    assert "--cloud-max-age-s" in runner
    assert "--cloud-pose-max-gap-s" in runner
    assert "--input-recovery-frames" in runner
    assert "--status-file" in runner
    assert "--estop-latch-file" in runner
    assert '--status-s "${LINGTU_NAV_STATUS_S}"' in runner
    assert "--gateway-host" not in runner
    assert "--gateway-port" not in runner
    assert "native navigation DDS endpoint is missing or not executable" in runner
    assert "build_nav_endpoint.sh" in runner
    assert "ros2-env.sh" not in text
    assert "ros2-env.sh" not in runner
    assert "LINGTU_NAV_CHECK_OBSTACLE" in config_source
    assert "LINGTU_NAV_MAX_SPEED_MPS" in config_source
    assert "LINGTU_NAV_MAX_ACCEL_MPS2" in config_source
    assert "LINGTU_NAV_TRAVERSABILITY_HARD_COST" in config_source
    assert "LINGTU_NAV_VEHICLE_LENGTH_M" in config_source
    assert "LINGTU_NAV_VEHICLE_WIDTH_M" in config_source
    configured_sensor_origin = tuple(
        _service_environment_float(text, f"LINGTU_NAV_SENSOR_OFFSET_{axis}_M") for axis in ("X", "Y", "Z")
    )
    assert configured_sensor_origin == pytest.approx(_authoritative_body_to_lidar_translation())
    assert "LINGTU_TELEOP_MIN_MOTION_SPEED_MPS=0.03" in text
    assert "LINGTU_NAV_SENSOR_OFFSET_Z_M" in config_source
    assert "cfg.check_obstacle = parseBool" in config_source
    assert "cfg.check_obstacle && cfg.use_traversability_cost" in source
    assert "buildPlannerObstacleCloud(" in source
    assert "sensorOriginFromBody(" in source
    assert "obstacle_xyzh," in source
    assert "live_obstacles.stats()" in source
    assert "constexpr double kLayerInflationM = 0.0;" in source
    assert "buildNavLoopConfig(cfg, safety_config.obstacle_margin_m)" in source
    assert "out.local_planner.footprintPadding = obstacle_margin_m;" in source
    assert "cfg.teleop_obstacle_margin_m + cfg.live_obstacle_inflation_radius_m" in config_source
    assert "out.path_follower.nominalDt = 1.0 / cfg.tick_hz;" in source
    assert "dynamicClusters(32, stamp_s)" in source
    assert "dynamicClusters(32, wall_now_s)" in source
    assert "const auto dynamic_clusters = live_obstacles.dynamicClusters(32, now);" not in source
    assert "live_obstacles_.snapshot(config_.max_obstacle_points, state_.last_cloud_s)" in source
    assert "if (xyzh.empty())" in source
    assert "obstacle_snapshot_dirty" in source
    assert "timing.obstacle_snapshot_last_ms" in source
    assert "timing.motion_update_last_ms" in source
    assert "InputGate input_gate" in endpoint_bootstrap
    assert "state_.input_gate_state = input_gate_.evaluate(snapshot);" in input_projector
    assert "path_active_for_tick && !input_gate_state.ready" in endpoint_loop
    navigation_state_publish = endpoint_loop.split(
        "(void)dds.writeNavigationState(navigation_state.sample(NavigationStateContext{", 1
    )[1].split("}));", 1)[0]
    assert "input_gate_state.ready" in navigation_state_publish
    assert "input_gate_state.reason" in navigation_state_publish
    assert "TransformBuffer pose_buffer" in source
    assert "pose_buffer_.sample(stamp_s, config_.cloud_pose_max_gap_s)" in source
    assert "headerStampSeconds(msg.header)" in source
    assert "cloud_sync.pose_rejected" in source
    status = _read("src/nav/cpp/endpoint/status/nav_status_writer.cpp")
    assert "motion_layer" in status
    assert "last_sensor_origin" in status
    assert "dynamic_objects" in status
    assert "local_planner_footprint" in status
    assert "obstacle_snapshot_last" in status
    assert "motion_update_last" in status
    assert "unknown_query_state" in status
    assert '\\"max_accel_mps2\\"' in status
    assert '\\"nominal_dt_s\\"' in status
    assert "input_gate" in status
    assert '\\"control_mode\\"' in status
    assert '\\"require_odom\\"' in status
    assert "final_safety" in status
    assert "evaluateCommandSafety(" in source
    assert 'std::string("final_safety_")' in source
    assert "cloud_pose_rejected" in status
    assert "cloud_stamp_rejected" in status
    assert "last_pose_gap_s" in status
    assert "terrain_map_ext_diagnostics_only" in status
    assert "cloud_stale" in _read("src/nav/cpp/tests/endpoint/test_input_gate.cpp")


def test_native_endpoint_accepts_only_typed_command_and_operator_motion_inputs() -> None:
    loop = _read("src/nav/cpp/endpoint/endpoint_loop.cpp")
    runtime = _read("src/nav/cpp/endpoint/nav_dds_runtime.cpp")
    runtime_h = _read("src/nav/cpp/endpoint/nav_dds_runtime.hpp")
    config = _read("src/nav/cpp/endpoint/nav_endpoint_config.cpp")
    status = _read("src/nav/cpp/endpoint/status/nav_status_writer.cpp")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    combined = "\n".join((loop, runtime, runtime_h))
    assert "drain" + "Legacy" not in combined
    for removed_reader_suffix in (
        "goal_reader_",
        "global_path_reader_",
        "cancel_reader_",
        "teleop_reader_",
    ):
        assert "legacy_" + removed_reader_suffix not in combined
    assert "drainCommandRequests" in combined
    assert "drainOperatorMotionControls" in combined
    assert "drainOperatorMotionSamples" in combined
    for removed_flag in (
        "allow_" + "legacy_motion_inputs",
        "LINGTU_NAV_ALLOW_" + "LEGACY_MOTION_INPUTS",
        "legacy_" + "motion_inputs_enabled",
        "legacy_" + "inputs_enabled",
    ):
        assert removed_flag not in "\n".join((config, status, runner))
    assert "external_global_path" not in loop
    assert "decodePath(msg" not in loop
    assert "nav.setGlobalPath(path);" not in loop
    assert "control_authority.activatePath();" not in loop
    assert "writer(lingtu::message::kNavGlobalPath.dds_topic.data()" in runtime
    assert "dds_write(global_path)" in runtime


def test_native_endpoint_navigation_runtime_orders_ticketed_terminal_lifecycle() -> None:
    bootstrap = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    loop = _read("src/nav/cpp/endpoint/endpoint_loop.cpp")
    controller = _read("src/nav/cpp/endpoint/navigation_runtime_controller.cpp")
    terminal_transaction = _read("src/nav/cpp/endpoint/status/goal_terminal_transaction.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")

    callback = bootstrap.split("goal_plan_actions.publish_status", 1)[1].split(
        "goal_plan_actions.inspection_active", 1
    )[0]
    assert "NavigationGoalStatusOutbox" in bootstrap
    assert "GoalTerminalStatusDelivery" in bootstrap
    assert "GoalReplanRuntimeCoordinator" in bootstrap
    assert "goal_status_outbox.record(status)" in callback
    assert "dds.writeNavigationGoalStatus" not in callback
    assert "return dds.writeNavigationGoalStatus" in bootstrap
    assert "status/navigation_goal_status_outbox.cpp" in cmake
    assert "status/goal_terminal_status_delivery.cpp" in cmake
    assert "navigation_runtime_controller.cpp" in cmake
    assert "NavigationRuntimeController navigation_runtime_controller" in loop

    for old_bypass in (
        "goal_plan.advance(",
        "goal_plan.resumePending(",
        "goal_plan.deferFailure(",
        "goal_plan.deferActiveTerminal(",
    ):
        assert old_bypass not in loop

    for controller_owned_operation in (
        "advancePlanningCycle",
        "handleAutonomyOutcome",
        "drainPendingCycle",
        "decideGoalTerminalScheduling",
        "goal_terminal_transaction.advance",
    ):
        assert controller_owned_operation not in loop

    frame_lifecycle = controller.split("NavigationRuntimeController::advanceFrame", 1)[1].split(
        "bool NavigationRuntimeController::terminalPending", 1
    )[0]
    ordered_frame_steps = (
        "goal_replan_runtime_.advancePlanningCycle(frame)",
        "completeTerminal(result.planning_result)",
        "actions.complete_endpoint_work_before_autonomy(result.planning_result)",
        "goal_plan_.snapshot()",
        "actions.run_autonomy(pre_autonomy_goal_snapshot)",
        "goal_replan_runtime_.handleAutonomyOutcome(",
        "actions.apply_autonomy_outputs(runtime_outcome)",
        "completeTerminal(terminal_candidate)",
        "goal_replan_runtime_.drainPendingCycle(frame)",
    )
    cursor = 0
    for step in ordered_frame_steps:
        position = frame_lifecycle.find(step, cursor)
        assert position >= 0, f"missing ordered navigation runtime step: {step}"
        cursor = position + len(step)
    assert "if (!goal_replan_runtime_.terminalPending())" in frame_lifecycle

    terminal_advance = terminal_transaction.split("GoalTerminalTransaction::advance(", 1)[1].split(
        "GoalTerminalTransaction::stopWhileTerminalPending", 1
    )[0]
    ordered_ticket_steps = (
        "goal_terminal_delivery_.stage(",
        "goal_terminal_delivery_.markCommitted(",
        "goal_terminal_delivery_.flushAndAcknowledge(",
    )
    cursor = 0
    for step in ordered_ticket_steps:
        position = terminal_advance.find(step, cursor)
        assert position >= 0, f"missing ordered goal terminal ticket step: {step}"
        cursor = position + len(step)
    assert (
        "if (!goal_terminal_delivery_.isCommitted(runtime_result.terminal_intent_id))"
        in terminal_advance
    )
    assert "ack.accepted = terminal_delivery_acknowledged" not in loop
    assert "goal_status_outbox.flush" in loop

    inspection_stop = bootstrap.split("inspection_command_actions.stop_and_commit", 1)[1].split(
        "inspection_command_actions.publish_ack", 1
    )[0]
    assert "goal_plan.deferAbort" not in inspection_stop
    assert "GoalReplanRuntimeInterruption::kInspectionPause" in inspection_stop
    assert "GoalReplanRuntimeInterruption::kInspectionCancel" in inspection_stop
    assert "goal_terminal_delivery.stage" in inspection_stop
    assert "goal_terminal_delivery.markCommitted" in inspection_stop
    # The ordinary MotionStop safety seam remains intentionally distinct from
    # inspection task pause/cancel, which is ticketed above.
    assert "motion_stop_actions.defer_goal_abort" in bootstrap


def test_native_endpoint_terminal_barrier_gates_ingress_and_stages_inspection_goals() -> None:
    loop = _read("src/nav/cpp/endpoint/endpoint_loop.cpp")
    terminal_transaction = _read("src/nav/cpp/endpoint/status/goal_terminal_transaction.cpp")

    assert "evaluateGoalTerminalIngress" in loop
    for ingress in (
        "kTypedGoal",
        "kTypedTaskCancel",
        "kTypedTaskPause",
        "kTypedTaskResume",
        "kTypedClearEstop",
        "kTypedResumeAutonomy",
        "kTypedClientClockSync",
        "kTypedStop",
        "kTypedEstop",
        "kOperatorClaim",
        "kOperatorHold",
        "kOperatorRelease",
        "kOperatorMotionSample",
        "kInspectionCommand",
        "kInspectionGoalDispatch",
        "kRollingCommand",
    ):
        assert ingress in loop
    assert "k" + "Legacy" not in loop

    assert "motion_stop_.stopPreservingGoalTerminal" in terminal_transaction
    assert "motion_stop_.estopPreservingGoalTerminal" in terminal_transaction
    assert "motion_stop.stopPreservingGoalTerminal" not in loop
    assert "motion_stop.estopPreservingGoalTerminal" not in loop
    assert "inspection_command_coordinator.reject" in loop
    assert "RollingSegmentIngressRejected" in loop
    assert "goal_replan_runtime.terminalPending()" not in loop

    terminal_barrier = loop.split("auto terminal_ingress", 1)[1].split(
        "GoalTaskCancelRouter", 1
    )[0]
    assert "navigation_runtime_controller.terminalPending()" in terminal_barrier

    inspection_busy = loop.split("inspection_tick_input.goal_plan_busy =", 1)[1].split(
        ";", 1
    )[0]
    assert "goal_plan.snapshot().busy" in inspection_busy
    assert "navigation_runtime_controller.terminalPending()" in inspection_busy
    assert "staged_inspection_goal" in loop
    assert 'completeGoalDispatch(false, "goal_terminal_pending"' not in loop


def test_nav_endpoint_uses_relative_height_when_cloud_has_no_height_field() -> None:
    native = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    projector = _read("src/nav/cpp/endpoint/input/nav_input_state_projector.cpp")
    messages = _read("src/nav/cpp/endpoint/nav_endpoint_messages.cpp")
    runtime = _read("src/nav/cpp/endpoint/nav_dds_runtime.cpp")
    runtime_h = _read("src/nav/cpp/endpoint/nav_dds_runtime.hpp")
    text = "\n".join([native, projector, messages, runtime, runtime_h])

    assert 'name == "intensity"' in text
    assert "const bool has_height = offsets.height >= 0" in text
    assert "offsets.height >= 0 || offsets.intensity >= 0" not in text
    assert "offsets.height >= 0 ? offsets.height : offsets.intensity" not in text
    assert "height = static_cast<float>(world_z - map_body->translation.z)" in text
    assert "offsets.height >= 0 ? readFloat(base + offsets.height) : z" not in text
    assert "kNavTerrainMap" in text
    assert "kNavTerrainMapExt" in text
    assert "kNavMapClearing" in text
    assert "kNavCloudClearing" in text
    assert "drainTerrainMap" in text
    assert "drainTerrainMapExt" in text
    assert "drainMapClearing" in text
    assert "drainCloudClearing" in text
    assert "clearPlannerInputState" in text
    assert "obstacle_xyzh.clear()" in text
    assert "buildPlannerObstacleCloud(" in text
    assert "appendXyzhCloudDedupe(" in messages
    assert "config.registered_share > 0.0 && !registered_xyzh.empty()" in messages
    assert "config.terrain_share > 0.0 && terrain_map_fresh" in messages
    assert "config.terrain_ext_share > 0.0 && terrain_ext_fresh" in messages
    assert "const bool terrain_ext_active" in messages
    assert "registered_budget" in messages
    assert "terrain_budget" in messages
    assert "terrain_ext_budget" in messages
    assert "double terrain_ext_share{0.0};" in _read("src/nav/cpp/endpoint/nav_endpoint_messages.hpp")
    assert "double obstacle_terrain_ext_share{0.0};" in _read("src/nav/cpp/endpoint/nav_endpoint_config.hpp")
    assert "registered_xyzh," in messages
    assert "terrain_xyzh," in messages
    assert "terrain_map_fresh ? terrain_xyzh : obstacle_xyzh" not in text
    status = _read("src/nav/cpp/endpoint/status/nav_status_writer.cpp")
    assert "has_terrain_map" in status
    assert "has_terrain_map_ext" in status
    assert "terrain_maps" in status
    assert "terrain_map_exts" in status
    assert "map_clearing" in status
    assert "cloud_clearing" in status
    assert "navigation_compute_owner" in status
    assert "navd" in status
    assert "local_path_role" in status
    assert "dds_telemetry_and_preview" in status
    assert "path_follower_role" in status
    assert "embedded_before_cmd_vel_gate" in status
    assert "cmd_vel_role" in status
    assert "final_navigation_command_output_when_enabled" in status


def test_thunder_slam_dds_installer_is_explicit_cpp_slam_boundary() -> None:
    text = _read("scripts/deploy/thunder/install_slam_dds_service.sh")
    helper = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert "install_catalog_service.sh" in text
    assert "exec bash" in text
    assert " slam" in text
    assert "runtime.service_catalogs.thunder" in helper
    assert "install-unit" in helper
    assert "installer" in helper
    assert "lingtu-slam-dds.service" not in text
    assert "lingtu-livox-dds.service" not in text
    assert "thunder-runtime-env.sh" in helper
    assert "ros2-env.sh" not in text
    assert "run_dds_endpoint_service.py" not in text
    assert "../s100p/install_services.sh" not in text


def test_legacy_s100p_ros2_service_templates_are_absent() -> None:
    for rel_path in (
        "scripts/deploy/s100p/lidar.service",
        "scripts/deploy/s100p/slam.service",
        "scripts/deploy/s100p/slam_pgo.service",
        "scripts/deploy/s100p/localizer.service",
        "scripts/deploy/s100p/genz_icp.service",
        "scripts/deploy/s100p/hba.service",
    ):
        assert not (ROOT / rel_path).exists()


def test_super_lio_experiment_stays_out_of_default_field_deployment() -> None:
    for rel_path in (
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        assert not (ROOT / rel_path).exists()

    assert (ROOT / "integrations/super_lio/README.md").is_file()
    assert (ROOT / "integrations/super_lio/systemd/super_lio.service").is_file()
    assert (ROOT / "integrations/super_lio/systemd/super_lio_relocation.service").is_file()

    real_env = _read("config/runtime_graph/envs/real.yaml").lower()
    processes = real_env.split("processes:", 1)[1].split("conflicts:", 1)[0]
    assert "super_lio" not in processes
    assert "super-lio" not in processes

    conflicts = real_env.split("conflicts:", 1)[1].split("endpoints:", 1)[0]
    for tombstone in (
        "robot-super-lio.service",
        "robot-super-lio-relocation.service",
        "super_lio.service",
        "super_lio_relocation.service",
    ):
        assert f"- {tombstone}" in conflicts

    installer = _read("scripts/deploy/thunder/install_services.sh").lower()
    assert "super_lio" not in installer
    assert "super-lio" not in installer


def test_release_script_consumes_the_committed_run_plan_identity() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    assert "LingTu Thunder release" in text
    assert "resolve_current_run_path" in text
    assert "CURRENT_RUN_SCHEMA" in text
    assert "RunPlan.load" in text
    assert 'current.get("run_plan_path")' in text
    assert "current Product does not match RunPlan" in text
    assert "current Env does not match RunPlan" in text
    assert "current RunPlan fingerprint does not match its record" in text
    assert "CURRENT_PRODUCT" in text
    assert "CURRENT_CONTROL_MODE" in text
    assert "CURRENT_DRIVER_UNIT" in text
    assert "CURRENT_MAPD_UNIT" in text
    assert 'SELECTED_PRODUCT="${LINGTU_PRODUCT:-}"' not in text
    assert "LINGTU_RELEASE_CONTROL_MODE" not in text
    assert "case \"$SELECTED_PRODUCT\"" not in text
    assert "printf 'LINGTU_PRODUCT=%s\\n'" not in text
    assert "product_mode_switch" not in text
    assert 'payload.get("processes"' not in text
    for retired in (
        "LINGTU_RELEASE_PRODUCT_" + "PROFILE",
        "LINGTU_PRODUCT_" + "PROFILE",
        "SELECTED_PRODUCT_" + "PROFILE",
        "LINGTU_" + "PROFILE",
    ):
        assert retired not in text
    assert "S100P" not in text
    assert "\u95c1" not in text


def test_release_run_plan_loader_accepts_the_current_schema(tmp_path: Path) -> None:
    from lingtu.control import ProductControl
    from lingtu.run_plan import CURRENT_RUN_SCHEMA

    plan = ProductControl(env="real").resolve("teleop")
    plan_path = plan.write(tmp_path / "plan.json")
    current_path = tmp_path / "current.json"
    current_path.write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "env": plan.env,
                "fingerprint": plan.fingerprint,
                "run_plan_path": str(plan_path.resolve()),
            }
        ),
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(ROOT / "src"), environment.get("PYTHONPATH", ""))
    ).rstrip(os.pathsep)
    environment["LINGTU_CURRENT_FILE"] = str(current_path)

    result = subprocess.run(
        [sys.executable, "-c", _release_run_plan_loader_source()],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr.decode(errors="replace")
    assert result.stdout.split(b"\0") == [
        str(plan_path.resolve()).encode(),
        b"teleop",
        b"real",
        plan.fingerprint.encode(),
        b"teleop",
        b"lingtu-driver.service",
        b"",
        b"",
    ]


def test_release_run_plan_loader_rejects_a_mismatched_current_record(
    tmp_path: Path,
) -> None:
    from lingtu.control import ProductControl
    from lingtu.run_plan import CURRENT_RUN_SCHEMA

    plan = ProductControl(env="real").resolve("teleop")
    plan_path = plan.write(tmp_path / "plan.json")
    current_path = tmp_path / "current.json"
    current_path.write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": "nav",
                "env": plan.env,
                "fingerprint": plan.fingerprint,
                "run_plan_path": str(plan_path.resolve()),
            }
        ),
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(ROOT / "src"), environment.get("PYTHONPATH", ""))
    ).rstrip(os.pathsep)
    environment["LINGTU_CURRENT_FILE"] = str(current_path)

    result = subprocess.run(
        [sys.executable, "-c", _release_run_plan_loader_source()],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "current Product does not match RunPlan" in result.stderr


def test_release_script_does_not_gate_on_legacy_ros2_local_autonomy() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    assert "ensure_nav_endpoint_artifacts" in text
    assert 'bash "$DEV_DIR/scripts/build/build_nav_endpoint.sh"' in text
    assert 'cmake --install "$NAV_ENDPOINT_BUILD_DIR"' in text
    assert "NAV_ENDPOINT_RUNTIME_FILES" in text
    assert "navd" in text
    assert "liblingtu_nav_client.so" in text
    assert "liblingtu_inspection.so" in text
    assert "ensure_nav_kernel_artifact" in text
    assert 'bash "$DEV_DIR/scripts/build/build_nav_kernel.sh" --clean' in text
    assert "LINGTU_RELEASE_REQUIRE_PYTHON_NAV_KERNEL:-0" in text
    assert "bash scripts/build/build_octoplanner3d.sh" in text
    assert "bash scripts/build/build_octoplanner3d.sh --require-pcl" in text
    assert "LINGTU_RELEASE_GLOBAL_PLANNER" in text
    assert "far)" in text
    assert "DEFAULT_REQUIRE_OCTOPLANNER3D=0" in text
    assert "LINGTU_RELEASE_REQUIRE_OCTOPLANNER3D:-$DEFAULT_REQUIRE_OCTOPLANNER3D" in text
    assert "LINGTU_RELEASE_REQUIRE_OCTOMAP_CONVERTER:-$REQUIRE_OCTOPLANNER3D" in text
    assert "nav_status_matches_release" in text
    assert "occupancy.npz" in text
    assert "LINGTU_RELEASE_REQUIRE_ROS2_COMPAT" not in text
    assert "LINGTU_RELEASE_RESTART_ROS2_COMPAT" not in text
    assert "ROS2_COMPAT_PKGS" not in text
    assert "require_ros2_compat_install" not in text
    assert "ROS 2 compatibility package gate skipped" not in text
    assert "Build first: cd $DEV_DIR && colcon build" not in text
    assert "RUNTIME_PKGS=" not in text
    assert "lingtu-livox-driver.service" not in text
    assert "emit_release_services" not in text
    assert "lingtu-thunder-dds-endpoint.service" not in text
    assert "resolve_release_services" not in text
    assert "LINGTU_RELEASE_SERVICES" not in text
    assert "OBSERVED_RELEASE_SERVICES" not in text
    assert "RELEASE_SERVICES=" not in text
    assert "robot-fastlio2.service" not in text
    assert "robot-localizer.service" not in text


def test_release_script_packages_and_activates_all_field_native_runtimes() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    for build_script in (
        "scripts/build/build_livox_sdk2_stream.sh",
        "scripts/build/build_slam_core.sh",
        "scripts/build/build_mapd.sh",
        "scripts/build/build_dds_probe.sh",
        "scripts/build/build_nav_endpoint.sh",
        "scripts/build/build_driver.sh",
    ):
        assert build_script in text

    for release_binary in (
        "build/livox_sdk2_stream/livox_sdk2_stream",
        "build/slam_core/slamd",
        "build/maps/mapd",
        "build/maps/lingtu-mapctl",
        "build/maps/liblingtu_maps.so",
        "build/dds_probe/lingtu_dds_probe",
        "build/nav_endpoint/navd",
        "build/nav_endpoint/lingtu_traversability_dds",
        "build/driver/lingtu_driver",
    ):
        assert release_binary in text

    for unit in (
        "lingtu-livox-dds.service",
        "lingtu-slam-dds.service",
        "mapd.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-driver.service",
    ):
        assert f"$LINGTU_SYSTEMD_DIR/{unit}" in text

    assert "require_persistent_driver_service" in text
    assert 'systemctl is-enabled --quiet "$CURRENT_DRIVER_UNIT"' in text
    assert "verify_release_native_runtime_files" in text
    assert "LINGTU_RELEASE_REUSE_VERIFIED_NATIVE_BUILD:-0" in text
    assert text.count('if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then') == 7
    assert "require_reusable_artifact_fresh" in text
    assert "write_release_native_sha256_manifest" in text
    assert '"${RELEASE_NATIVE_RUNTIME_FILES[@]}"' in text
    assert '"${RELEASE_NATIVE_LIBRARY_FILES[@]}"' in text
    assert "sha256sum -c config/release-native-sha256.txt" in text
    assert "bound into RunPlan compatibility" in text
    assert 'verify_release_native_sha256_manifest "$CURRENT_LINK"' in text
    assert 'verify_release_native_runtime_files "$CURRENT_LINK"' in text
    assert "|| true" not in text.split("verify_driver_uses_current_release()", 1)[1].split("\n}\n", 1)[0]
    assert "verify_driver_uses_current_release" in text
    assert "verify_mapd_uses_current_release" in text
    mapd_check = text.split("verify_mapd_uses_current_release() {", 1)[1].split("\n}", 1)[0]
    assert '"$CURRENT_MAPD_UNIT"' in mapd_check
    assert '"build/maps/mapd"' in mapd_check
    assert 'reapply_committed_run_plan "activation"' in text
    activation_body = text.split('reapply_committed_run_plan "activation"', 1)[1]
    assert "verify_driver_uses_current_release" in activation_body
    assert "verify_mapd_uses_current_release" in activation_body


def test_release_status_gate_supports_map_free_teleop_avoid() -> None:
    text = _read("scripts/deploy/cut_release.sh")
    body = text.split("nav_status_matches_release() {", 1)[1].split("\n}\n", 1)[0]

    assert '"$CURRENT_PRODUCT"' in body
    assert '"$CURRENT_CONTROL_MODE"' in body
    assert 'schema != "lingtu.nav.endpoint.status.v1"' in body
    assert 'endpoint != "navd"' in body
    assert 'if control_mode in {"teleop", "teleop_avoid"}' in body
    assert 'if control_mode == "teleop_avoid"' in body
    assert '"check_obstacle": True' in body
    assert '"use_traversability_cost": True' in body
    assert '"teleop_local_planner": True' in body
    assert 'elif control_mode == "autonomy"' in body
    assert 'raise RuntimeError("planner_map is missing for autonomy")' in body
    assert body.index('if control_mode in {"teleop", "teleop_avoid"}') < body.index(
        'planner_map = str(payload.get("planner_map") or "").strip()'
    )


def test_release_nav_status_program_accepts_map_free_teleop_avoid_and_rejects_map_free_autonomy(
    tmp_path: Path,
) -> None:
    text = _read("scripts/deploy/cut_release.sh")
    status_gate = text.split("nav_status_matches_release() {", 1)[1]
    program = status_gate.split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
    status_path = tmp_path / "nav_endpoint_status.json"
    payload = {
        "schema_version": "lingtu.nav.endpoint.status.v1",
        "endpoint": "navd",
        "control_mode": "teleop_avoid",
        "native_product": {"product": "teleop_avoid", "config_fingerprint": "test"},
        "global_planner": "octoplanner3d",
        "planner_map": "",
        "publish_cmd_vel": True,
        "check_obstacle": True,
        "use_traversability_cost": True,
        "teleop_local_planner": True,
        "operator_motion": {"interface_enabled": True},
    }
    status_path.write_text(json.dumps(payload), encoding="utf-8")

    teleop_avoid = subprocess.run(
        [
            sys.executable,
            "-",
            str(status_path),
            "octoplanner3d",
            "teleop_avoid",
            "teleop_avoid",
        ],
        input=program,
        text=True,
        capture_output=True,
        check=False,
    )
    assert teleop_avoid.returncode == 0, teleop_avoid.stderr

    payload["control_mode"] = "autonomy"
    payload["native_product"] = {"product": "nav", "config_fingerprint": "test"}
    status_path.write_text(json.dumps(payload), encoding="utf-8")
    autonomy = subprocess.run(
        [sys.executable, "-", str(status_path), "octoplanner3d", "nav", "autonomy"],
        input=program,
        text=True,
        capture_output=True,
        check=False,
    )
    assert autonomy.returncode == 1
    assert "planner_map is missing for autonomy" in autonomy.stderr


def test_native_nav_endpoint_has_a_release_install_manifest() -> None:
    endpoint_cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    inspection_cmake = _read("src/nav/inspection/CMakeLists.txt")
    service = _read("scripts/deploy/thunder/lingtu-nav-dds.service")

    for target in (
        "navd",
        "lingtu_traversability_dds",
        "lingtu_explore_dds",
        "lingtu_nav_control",
        "lingtu_motion_mock_dds",
    ):
        assert target in endpoint_cmake
    assert "_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS" in endpoint_cmake
    assert "lingtu_nav_client" in endpoint_cmake
    assert "lingtu_inspection_evidence_bridge" in endpoint_cmake
    assert "LIBRARY DESTINATION ." in endpoint_cmake
    assert "LIBRARY DESTINATION inspection" in inspection_cmake
    assert "EnvironmentFile=-/opt/lingtu/current/config/release-runtime.env" in service


def test_native_dds_build_scripts_check_service_binaries() -> None:
    slam = _read("scripts/build/build_slam_core.sh")
    livox = _read("scripts/build/build_livox_sdk2_stream.sh")
    nav = _read("scripts/build/build_nav_endpoint.sh")

    assert "LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME:-ON" in slam
    assert "slamd" in slam
    assert "native SLAM DDS runtime is missing" in slam
    assert "LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME" not in slam

    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS:-ON" in livox
    assert "livox_sdk2_stream" in livox

    assert "navd" in nav
    assert "lingtu_traversability_dds" in nav
    assert "lingtu_nav_control" in nav
    assert "lingtu_motion_mock_dds" in nav
    assert "liblingtu_nav_client.so" in nav
    assert "LINGTU_CYCLONEDDS_PREFIX}/bin" in nav
    assert "include/${multiarch}" in nav
    assert "native navigation DDS endpoint is missing" in nav
    assert "LINGTU_NAV_CPP_BUILD_TESTS:-ON" in nav
    assert "LINGTU_NAV_ENDPOINT_RUN_TESTS:-$BUILD_TESTS" in nav
    assert 'ctest --test-dir "$BUILD_DIR" --output-on-failure' in nav
    assert "required navigation test is missing from CTest" in nav
    assert "test_path_follower_core" in nav
    assert "test_local_planner_core" in nav
    assert "test_nav_client" in nav
    assert "test_teleop_safety" in nav
    assert "test_nav_endpoint_config" in _read("src/nav/cpp/endpoint/CMakeLists.txt")
    endpoint_cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    assert "test_path_follower_core" in endpoint_cmake
    assert "test_local_planner_core" in endpoint_cmake


def test_motion_mock_dds_closes_cmd_vel_to_odom_loop_without_hardware() -> None:
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    source = _read("src/nav/cpp/endpoint/motion/motion_mock_dds.cpp")

    assert "add_executable(lingtu_motion_mock_dds" in cmake
    assert "motion_mock_dds.cpp" in cmake
    assert "kNavCmdVel" in source
    assert "kSlamOdometry" in source
    assert "kTf" in source
    assert "cmd.vx * c - cmd.vy * s" in source
    assert "cmd.vx * s + cmd.vy * c" in source
    assert 'fillHeader(out.header, stamp_s, "odom")' in source
    assert 'out.child_frame_id = const_cast<char *>("body")' in source
    assert 'fillHeader(out.transform.header, stamp_s, "map")' in source
    assert 'out.transform.child_frame_id = const_cast<char *>("odom")' in source
    assert '\\"lingtu.motion_mock.status.v1\\"' in source
    assert "LINGTU_MOTION_MOCK_STATUS_FILE" in source


def test_nav_control_external_path_is_explicit_legacy_smoke_only() -> None:
    text = _read("src/nav/cpp/endpoint/motion/nav_control.cpp")

    assert "path X1 Y1 Z1 X2 Y2 Z2" in text
    assert "kNavGlobalPath" in text
    assert "dds_write(global_path)" in text
    assert 'waitForMatchedReader(writer, "global_path")' in text
    assert 'dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(global_path)"' in text
    assert 'waitForMatchedReader(writer, "goal_pose")' not in text
    assert "waitForMatchedReader(writer, cfg.command.c_str())" in text
    assert 'dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(text)"' in text
    assert "clear <map|cloud|all>" in text
    assert "kNavMapClearing" in text
    assert "kNavCloudClearing" in text
    assert '#include "message/cpp/dds_qos_profiles.hpp"' in text
    assert "qosFor(" in text
    assert "dds_qset_reliability" not in text
    assert "dds_wait_for_acks" in text
    assert "waitForMatchedReader(writer, label)" in text
    assert 'publish_clear(lingtu::message::kNavMapClearing, "map_clearing")' in text
    assert 'publish_clear(lingtu::message::kNavCloudClearing, "cloud_clearing")' in text


def test_nav_control_exposes_typed_exploration_lifecycle() -> None:
    text = _read("src/nav/cpp/endpoint/motion/nav_control.cpp")

    assert "explore start SESSION_ID [REASON]" in text
    assert "explore <pause|resume|stop>" in text
    assert 'arg == "--request-id"' in text
    assert 'cfg.command += "-" + action' in text
    assert "client.exploration().start(" in text
    assert "client.exploration().pause(" in text
    assert "client.exploration().resume(" in text
    assert "client.exploration().stop(" in text
    assert "kNavExplorationCommand" not in text


def test_typed_navigation_client_uses_application_ack_as_authority() -> None:
    source = _read("src/nav/cpp/client/client.cpp")

    assert "active_request_id, pending, timeout_ms" in source
    assert "NavigationCommandAck is already available" in source
    assert "dds_wait_for_acks(nav_command_request)" not in source


def test_native_nav_endpoint_uses_shared_dds_qos_catalog() -> None:
    runtime_source = _read("src/nav/cpp/endpoint/nav_dds_runtime.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")

    assert '#include "message/cpp/dds_qos_profiles.hpp"' in runtime_source
    assert "qos_for_topic(topic_name)" in runtime_source
    assert "dds_qset_reliability" not in runtime_source
    assert "navd" in cmake
    assert "lingtu_dds_qos_profiles" in cmake
    assert "motion_layer.cpp" in cmake
    assert "test_motion_layer" in cmake
    assert "test_nav_endpoint_messages" in cmake
    assert "test_input_gate" in cmake
    assert "test_pose_buffer" in cmake


def test_native_motion_publishers_use_canonical_body_frame() -> None:
    sources = (
        _read("src/nav/cpp/endpoint/nav_dds_runtime.cpp"),
        _read("src/nav/cpp/endpoint/motion/nav_control.cpp"),
    )

    for source in sources:
        assert 'fillHeader(msg.header, nowSeconds(), "base_link")' not in source
    assert 'toDdsPath(path, "map")' in sources[0]
    assert 'toDdsPoseStamped(point, "map")' in sources[0]
    assert 'fillHeader(msg.header, nowSeconds(), "body")' in sources[1]


def test_retired_ota_tree_is_absent_and_native_release_is_canonical() -> None:
    package_script = _read("scripts/deploy/package_native_release.sh")
    installer = _read("scripts/deploy/install_native_release.sh")
    release_guide = _read("docs/04-deployment/OTA_GUIDE.md")
    scripts_index = _read("scripts/README.md")
    build_guide = _read("docs/01-getting-started/BUILD_GUIDE.md")
    combined = "\n".join((package_script, installer, release_guide, build_guide))

    assert not (ROOT / "scripts" / "ota").exists()
    assert 'INSTALLER_SOURCE="${SCRIPT_ROOT}/scripts/deploy/install_native_release.sh"' in package_script
    assert "ProductControl" in installer
    assert "scripts/deploy/package_native_release.sh" in release_guide
    assert "scripts/ota/" not in scripts_index
    for retired in ("build_nav_package.sh", "deploy_to_robot.sh", "generate_manifest.py"):
        assert retired not in scripts_index
        assert retired not in release_guide

    assert "--packages-select fastlio2 local_planner" not in combined
    assert "--packages-select local_planner" not in combined
    assert "--packages-select fastlio2 pct_planner" not in combined
    assert "--packages-select pct_planner" not in combined
    assert "pct_planner pct_adapters" not in combined
    assert "local_planner terrain_analysis terrain_analysis_ext" not in combined
    assert 'grep -E "fastlio2|local_planner|pct_planner' not in combined
    assert "bash scripts/build/build_nav_kernel.sh --clean" in build_guide
    assert "bash scripts/build/build_octoplanner3d.sh" in build_guide
    assert "Product default: native planner kernels, no ROS2" in build_guide
    assert "ROS 2 Humble Desktop is optional" in build_guide
    assert "make build           # source ROS Humble" not in build_guide

def test_deployment_runbooks_prefer_gateway_dataflow_over_ros_topic_defaults() -> None:
    deployment_readme = _read("docs/04-deployment/README.md")
    cli_doc = _read("docs/04-deployment/lingtu_cli.md")
    replacement_map = _read("docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md")

    assert "bash scripts/build/build_nav_kernel.sh --clean" in deployment_readme
    assert "bash scripts/build/build_octoplanner3d.sh" in deployment_readme
    assert "bash scripts/build/build_ros_workspace.sh" in deployment_readme
    assert "lingtu dataflow /nav/lidar_scan" in deployment_readme
    assert "lingtu dataflow /nav/odometry" in deployment_readme
    assert "lingtu doctor --ros2" in deployment_readme
    assert "ros2 topic hz /nav/lidar_scan" not in deployment_readme
    assert "ros2 topic hz /nav/odometry" not in deployment_readme
    assert "source /opt/ros/humble/setup.bash && colcon build" not in deployment_readme

    assert "lingtu doctor                   # read-only service/Gateway/dataflow diagnostics" in cli_doc
    assert "lingtu doctor --ros2" in cli_doc
    assert "Gateway readiness, health, localization, navigation, state, and camera snapshot" in cli_doc
    assert "publisher/subscriber counts for `/nav/lidar_scan`" not in cli_doc
    assert "scripts/deploy/cut_release.sh` is native-first" in replacement_map
    assert "Feishu and Telegram monitor bots collect status through Gateway endpoints" in replacement_map
    assert "obsolete ROS OTA" in replacement_map
    assert "script tree has been physically removed" in replacement_map
    assert "compatibility paths rather than product defaults" in replacement_map


def test_scripts_readme_uses_product_control_as_field_entrypoint() -> None:
    text = _read("scripts/README.md")
    perception_readme = _read("scripts/perception/README.md")
    root_readme = _read("README.md")
    repo_layout = _read("docs/REPO_LAYOUT.md")

    assert "deploy/deploy_thunder.sh" in text
    assert "deploy/cut_release.sh" in text
    assert "scripts/lingtu --env real mode switch nav --map <map>" in text
    assert "scripts/lingtu --env real mode switch explore" in text
    assert "Build and deploy the old ROS OTA package" not in text
    assert "ROS2 compatibility tool index" in text
    assert "python lingtu.py thunder-nav" not in text
    assert "python lingtu.py tare_explore" not in text
    assert "validate_lcm_jsonl_feed.py" not in text
    assert "Gateway camera endpoints" in perception_readme
    assert "S100P" not in text
    assert "Ubuntu 22.04 + ROS2 Humble" not in root_readme
    assert "ROS2 Humble is optional for compatibility services" in root_readme
    assert "colcon build / test / format / lint" not in repo_layout
    assert "native/test wrappers plus ROS workspace compatibility" in repo_layout
    assert "ROS2 compatibility perception demos" in repo_layout


def test_robot_ops_doctor_defaults_to_gateway_first_ros2_explicit() -> None:
    text = _read("scripts/lingtu")
    implementation = _read("src/diagnostics/field/doctor.py")
    doctor_body = text.split("cmd_doctor() {", 1)[1].split("\n}\n\n# -- Subcommand: soak --", 1)[0]

    assert "[--ros2]" in text
    assert "--ros2)" in doctor_body
    assert '[ "$ros2" = "1" ] && source_robot_env' in text
    assert "\n    source_robot_env\n" not in doctor_body
    assert '"$py" -m diagnostics.field.doctor' in doctor_body
    assert '--gateway-url "$GW" --env "$LINGTU_ENV"' in doctor_body
    assert "ros2_enabled = options.ros2" in implementation
    assert "if ros2_enabled:" in implementation
    assert "ROS2 compatibility graph checks skipped; use --ros2" in implementation
    assert "camera.ros2_topics_skipped" in implementation
    assert "camera.gateway_snapshot" in implementation
    for forbidden in ("systemctl", "journalctl", "RunPlan.load", "<<'PY'", "cmd_doctor_json"):
        assert forbidden not in doctor_body


def test_native_endpoint_uses_and_reports_compiled_product_motion_parameters() -> None:
    config = _read("src/nav/cpp/endpoint/nav_endpoint_config.cpp")
    endpoint_config = _read("src/nav/cpp/endpoint/endpoint_config.hpp")
    status = _read("src/nav/cpp/endpoint/status/nav_status_writer.cpp")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    for key in (
        "LINGTU_NAV_CONFIG_FINGERPRINT",
        "LINGTU_NAV_WAYPOINT_REACHED_M",
        "LINGTU_NAV_GOAL_REACHED_M",
        "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M",
        "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS",
        "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS",
        "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2",
        "LINGTU_TELEOP_PLANNER_HORIZON_M",
        "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG",
    ):
        assert key in config
    assert "out.waypoint_reached_m = cfg.waypoint_reached_m" in endpoint_config
    assert "out.goal_reached_m = cfg.goal_reached_m" in endpoint_config
    assert "out.path_follower.minSpeed = cfg.path_follower_min_speed_mps" in endpoint_config
    assert "out.path_follower.baseLookAheadDis = cfg.path_follower_lookahead_m" in endpoint_config
    assert "out.path_follower.stopDisThre = cfg.path_follower_goal_tolerance_m" in endpoint_config
    assert "out.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m" in endpoint_config
    assert "out.teleop_intent_max_deviation_deg =" in endpoint_config
    assert "teleop_planner_horizon_m" in status
    assert "teleop_planner_max_deviation_deg" in status
    assert '--teleop-planner-horizon-m "${LINGTU_TELEOP_PLANNER_HORIZON_M}"' in runner
    assert ('--teleop-planner-max-deviation-deg "${LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG}"') in runner
    assert "native_product" in status
    assert "config_fingerprint" in status
    assert "nav_loop" in status
    assert "--max-speed-mps" not in runner


def test_field_mode_units_require_one_valid_transient_product_session() -> None:
    guard_path = "/opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh"
    expected_roles = {
        "scripts/deploy/thunder/lingtu-livox-dds.service": "lidar",
        "scripts/deploy/thunder/lingtu-slam-dds.service": "slam",
        "scripts/deploy/thunder/mapd.service": "maps",
        "scripts/deploy/thunder/lingtu-traversability-dds.service": "traversability",
        "scripts/deploy/thunder/lingtu-nav-dds.service": "nav",
        "scripts/deploy/thunder/lingtu-explore-dds.service": "explore",
        "scripts/deploy/thunder/lingtu-camera-dds.service": "camera",
        "scripts/deploy/thunder/lingtu.service": "host",
    }
    for relative, role in expected_roles.items():
        unit = _read(relative)
        assert "EnvironmentFile=/run/lingtu/session.env" in unit
        assert f"ExecStartPre=/bin/bash {guard_path} {role}" in unit

    guard = _read("scripts/deploy/thunder/require_product_session.sh")
    for required in (
        "LINGTU_PRODUCT",
        "LINGTU_ENV",
        "LINGTU_PRODUCT_SESSION_ID",
        "LINGTU_RUN_PLAN",
        "LINGTU_RUN_PLAN_FINGERPRINT",
    ):
        assert f"${{{required}:?" in guard
    assert 'expected_plan="/run/lingtu/plan-${LINGTU_RUN_PLAN_FINGERPRINT}.json"' in guard
    assert 'expected_role="${1:-${LINGTU_EXPECTED_ROLE:-}}"' in guard
    assert 'process.get("name") == expected_role' in guard
    assert 'identity.get("fingerprint") != os.environ["LINGTU_RUN_PLAN_FINGERPRINT"]' in guard
    assert "teleop|teleop_avoid|map|explore|nav|tracking|inspection" in guard
    assert "tare_explore" not in guard


def test_robot_ops_delegates_product_switch_to_product_control() -> None:
    text = _read("scripts/lingtu")
    mode_body = text.split("cmd_mode() {", 1)[1].split("\n# ── Subcommand: map ──", 1)[0]

    assert "cmd_mode()" in text
    assert "--env real|sim mode switch <product>" in text
    assert '"$py" -m lingtu.control switch "$target" --env "$LINGTU_ENV" "$@" --json' in mode_body
    assert "ProductControl owns preflight" in text
    assert "mode_switch_preflight" not in text
    assert "mode_stop_motion_and_session" not in text
    assert "MODE_TARGET_" not in text
    assert "mode_profile_dropin" not in text
    assert "mode_nav_endpoint_dropin" not in text
    assert "mode_abort_product_switch" not in text
    assert "mode_restart_product_stack" not in text
    assert "mode)           shift; cmd_mode" in text


def test_product_switch_control_owns_fail_closed_cleanup() -> None:
    source = _read("src/lingtu/product_switch.py")

    assert "class SwitchRequest" in source
    assert "def execute_switch(" in source
    assert "control._apply_plan_for_switch(run_plan_path)" in source
    assert "control.quiesce_plan(plan)" in source
    assert "backend.rollback_session(session_stage)" in source
    assert "disable_boot_ownership" not in source
    assert '"active' + '-product.json"' not in source


def test_product_switch_requires_confirmed_stop_before_runtime_staging() -> None:
    source = _read("src/lingtu/product_switch.py")

    stop = "backend.stop_motion_and_session(current_product)"
    map_stage = "backend.stage_map(map_name)"
    config_stage = "backend.stage_session("
    apply = "control._apply_plan_for_switch(run_plan_path)"
    assert source.index(stop) < source.index(map_stage)
    assert source.index(map_stage) < source.index(config_stage)
    assert source.index(config_stage) < source.index(apply)


def test_robot_ops_service_mutations_delegate_to_the_active_product() -> None:
    text = _read("scripts/lingtu")
    body = text.split("cmd_svc() {", 1)[1].split("\n}\n\n# ── Subcommand: log", 1)[0]

    assert "lingtu_control reapply" in body
    assert "lingtu_control restart --process" in body
    assert "lingtu_control stop" in body
    assert "reapply" + "-active" not in body
    assert "restart" + "-active" not in body
    assert "stop" + "-active" not in body
    assert "systemctl restart" not in body
    assert "systemctl stop" not in body
    assert "svc_force_stop_unit" not in text
    assert "svc_restart_robot_stack" not in text


def test_product_nav_switch_commits_only_after_runtime_readiness() -> None:
    source = _read("src/lingtu/product_switch.py")

    stop = "backend.stop_motion_and_session(current_product)"
    map_stage = "backend.stage_map(map_name)"
    apply = "control._apply_plan_for_switch(run_plan_path)"
    session = "backend.start_session("
    readiness = "backend.wait_navigation("
    commit = "_commit_current_run("

    assert source.index(stop) < source.index(map_stage)
    assert source.index(map_stage) < source.index(apply)
    assert source.index(apply) < source.index(session)
    assert source.index(session) < source.index(readiness)
    assert source.index(readiness) < source.index(commit)


def test_product_switch_uses_gateway_availability_before_session_readiness() -> None:
    runner = _read("src/lingtu/systemd.py")
    manager = _read("src/runtime/service_manager.py")

    assert "thunder_service_spec(service)" in runner
    assert "http_check=True" in runner
    assert '"http://127.0.0.1:5050/api/v1/readiness"' in manager
    assert 'for field in ("data_ready", "non_motion_safe")' in manager
    assert 'if payload.get("data_ready") is False' in manager
    assert 'for field in ("failed_modules", "critical_failed_modules")' in manager


def test_product_nav_switch_failure_never_commits_current_run() -> None:
    source = _read("src/lingtu/product_switch.py")
    transaction = source.split("def execute_switch(", 1)[1].split("\ndef _lifecycle(", 1)[0]
    commit_at = transaction.index("_commit_current_run(")
    failure_at = transaction.index("except Exception as exc:", commit_at)
    failure = transaction[failure_at:]

    assert 'report.status = "rollback_failed"' in failure
    assert 'report.status = "failed_stopped"' in failure
    assert 'report.status = "stop_unconfirmed"' in failure
    assert 'report.status = "failed"' in failure
    assert "control.quiesce_plan(plan)" in failure
    assert "backend.rollback_session(session_stage)" in failure
    assert "_commit_current_run" not in failure


def test_nav_start_delegates_to_compiled_product() -> None:
    text = _read("scripts/lingtu")
    nav_body = text.split("cmd_nav() {", 1)[1].split("\ncmd_loc() {", 1)[0]
    start_body = nav_body.split("start)", 1)[1].split("smoke|motion-smoke", 1)[0]

    assert 'cmd_mode switch nav --map "$map"' in start_body
    assert "--initial-pose" in start_body
    assert "--relocalize" in start_body
    assert "--no-relocalize" in start_body
    assert "slam_dds_set_mode" not in start_body
    assert "/api/v1/session/start" not in start_body
    assert "nav_relocalize_saved_map" not in start_body


def test_motion_smoke_delegates_runtime_setup_to_product_control() -> None:
    text = _read("scripts/lingtu")
    body = text.split("cmd_motion_smoke() {", 1)[1].split(
        "\n}\n\n# -- Subcommand: system-acceptance --",
        1,
    )[0]
    gate = _read("scripts/gates/motion_smoke_gate.py")

    assert body.strip() == 'run_python_gate motion_smoke_gate.py "$@"'
    assert '["switch", "nav", "--env", args.env' in gate
    assert 'stop_args = ["stop-session", "--env", args.env]' in gate
    assert "/api/v1/session/start" not in gate
    assert "nav_relocalize_saved_map" not in text


def test_robot_ops_system_acceptance_gate_matches_that_nav_parity_plan() -> None:
    text = _read("scripts/lingtu")
    body = text.split("cmd_system_acceptance() {", 1)[1].split("\n}\n\n# -- Subcommand: teleop_avoid", 1)[0]
    gate = _read("scripts/gates/system_acceptance_gate.py")
    motion_gate = _read("scripts/gates/motion_smoke_gate.py")
    cli_doc = _read("docs/04-deployment/lingtu_cli.md")

    assert "system-acceptance|that-nav-acceptance|acceptance-gate" in text
    assert body.strip() == 'run_python_gate system_acceptance_gate.py "$@"'
    assert '["switch", "nav", "--env", args.env' in gate
    assert 'stop_args = ["stop-session", "--env", args.env]' in gate
    assert "runtime-audit" in gate
    assert '"doctor", "--non-motion", "--strict", "--json"' in gate
    assert '"soak"' in gate
    assert "saved_map_artifact_gate.py" in gate
    assert '"--require-occupancy"' in gate
    assert '"--expected-data-source", "thunder"' in gate
    assert "validate_saved_map_plan" in gate
    assert "requested_map_validate_plan.json" in gate
    assert 'root / "relocalization"' in gate
    assert '"saved_map_relocalization"' in gate
    assert "--with-relocalization" in gate
    assert "--initial-pose" in gate
    assert "motion-smoke|motioncheck|path-follower-check" in text
    assert "motion_smoke_gate.py" in gate
    assert "--allow-motion" in gate
    assert "motion-smoke requires --allow-motion" in motion_gate
    assert "real_runtime_evidence_collect.py" in _read("scripts/gates/motion_smoke_gate.py")

    assert "That-nav parity gate" in cli_doc
    assert "validates the native/Gateway" in cli_doc
    assert "does not send motion commands by default" in cli_doc


def test_monitor_bots_are_gateway_backed_without_ros2_or_embedded_credentials() -> None:
    bot_paths = [
        "scripts/monitor/telegram_monitor_bot.py",
        "scripts/monitor/feishu_monitor_bot.py",
        "scripts/monitor/gateway_status.py",
    ]
    combined = "\n".join(_read(path) for path in bot_paths)

    for rel_path in bot_paths:
        path = ROOT / rel_path
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        imports: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module)
        assert "rclpy" not in imports
        assert "rclpy.node" not in imports
        assert "std_msgs.msg" not in imports

    assert "collect_gateway_status" in combined
    assert "/api/v1/navigation/status" in combined
    assert "/api/v1/health" in combined
    assert "/api/v1/localization/status" in combined
    assert "LINGTU_GATEWAY_URL" in combined
    assert "TELEGRAM_BOT_TOKEN" in combined
    assert "FEISHU_APP_SECRET" in combined
    assert "STATUS_TOPIC" not in combined
    assert "YOUR_BOT_TOKEN_HERE" not in combined
    assert "F4aHJepltjOioMCyDW0zWfvDwKrpdHeQ" not in combined


def test_rerun_live_uses_compat_ros_context() -> None:
    path = ROOT / "scripts" / "visualization" / "rerun_live.py"
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "runtime.ros2_context" not in imports
    assert "runtime.adapters.ros2.context" not in imports
    assert "runtime.adapters.dds.reader" in imports


def test_rerun_gateway_live_is_ros_free_gateway_viewer() -> None:
    path = ROOT / "scripts" / "visualization" / "rerun_gateway_live.py"
    text = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(text, filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "rclpy" not in imports
    assert "runtime.adapters.ros2.context" not in imports
    assert "/api/v1/runtime/dataflow" in text
    assert "Gateway-backed Rerun live viewer" in text
