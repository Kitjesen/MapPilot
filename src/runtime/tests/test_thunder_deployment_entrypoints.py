from __future__ import annotations

import ast
import os
import re
import subprocess
from pathlib import Path

import pytest

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



def test_traversability_service_uses_product_session_body_to_lidar_translation() -> None:
    text = _read("scripts/deploy/thunder/lt-terrain.service")

    assert "EnvironmentFile=/run/lingtu/session.env" in text
    for axis in ("X", "Y", "Z"):
        assert f"Environment=LINGTU_TRAVERSABILITY_SENSOR_OFFSET_{axis}_M=" not in text
        assert f'--sensor-offset-{axis.lower()}-m "${{LINGTU_NAV_SENSOR_OFFSET_{axis}_M}}"' in text


def test_thunder_nav_motion_geometry_comes_only_from_the_product_session() -> None:
    service = _read("scripts/deploy/thunder/lt-nav.service")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")
    required_session_keys = (
        "LINGTU_NAV_VEHICLE_LENGTH_M",
        "LINGTU_NAV_VEHICLE_WIDTH_M",
        "LINGTU_NAV_SENSOR_OFFSET_X_M",
        "LINGTU_NAV_SENSOR_OFFSET_Y_M",
        "LINGTU_NAV_SENSOR_OFFSET_Z_M",
    )

    assert "EnvironmentFile=/run/lingtu/session.env" in service
    for key in required_session_keys:
        assert f"Environment={key}=" not in service
        assert f"${{{key}:?" in runner


def _exec_start_lines(unit_text: str) -> list[str]:
    return [
        line
        for line in unit_text.splitlines()
        if line.startswith("ExecStart=") or line.startswith("ExecStartPre=") or line.startswith("ExecStartPost=")
    ]


def test_generic_deploy_script_owns_flow() -> None:
    text = _read("scripts/deploy/deploy_robot.sh")
    build_plan = _read("src/lingtu/assembly/deployment.py")

    assert "LINGTU_DEPLOY_PRODUCT" in text
    assert 'PRODUCT="${LINGTU_DEPLOY_PRODUCT:-}"' in text
    assert 'if [ -z "${PRODUCT}" ] && [ $# -gt 0 ]; then' in text
    assert "LINGTU_DEPLOY_PROFILE" not in text
    assert "compile_run_plan" not in text
    assert "product_native_build_scripts" in text
    assert "LINGTU_DEPLOY_PLAN_ONLY" in text
    assert "tare_explore" not in text
    assert 'scripts/lingtu" --robot "${ROBOT}" --env real switch' in text
    assert "LINGTU_DEPLOY_ROBOT" in text
    assert "LINGTU_DEPLOY_MAP" in text
    assert "LINGTU_DEPLOY_INSTALL_SERVICES" in text
    assert "ProductControl" in text
    assert "git reset --hard" not in text
    assert "git pull --ff-only" in text
    assert "LINGTU_PROFILE" not in text
    assert "/opt/ros" not in text
    assert "nohup" not in text
    assert "--daemon" not in text
    assert 'for build_script in "${NATIVE_BUILD_SCRIPTS[@]}"' in text
    for build_script in (
        "scripts/build/build_livox_sdk2_stream.sh",
        "scripts/build/build_slam_core.sh",
        "scripts/build/build_mapd.sh",
        "scripts/build/build_nav_endpoint.sh",
        "scripts/build/build_dds_probe.sh",
        "scripts/build/build_driver.sh",
        "scripts/build/build_orbbec_native.sh",
        "scripts/build/build_camera_dds.sh",
    ):
        assert build_script in build_plan
    assert "scripts/build/build_maps.sh" not in text + build_plan


def test_legacy_s100p_deploy_alias_is_removed() -> None:
    assert not (ROOT / "scripts" / "deploy" / "deploy_s100p.sh").exists()


def test_thunder_service_installer_defaults_to_native_field_cpp_stack() -> None:
    text = _read("scripts/deploy/thunder/install_services.sh")

    assert 'MODE="${1:-field-cpp}"' in text
    assert "runtime.service_catalogs.thunder install-services" in text
    assert "runtime.service_catalogs.thunder install-modes" in text
    assert 'catalog_services_for_mode "${MODE}"' in text
    assert 'run_catalog_services "${service_text}"' in text
    assert 'bash "${SCRIPT_DIR}/install_catalog_service.sh" "${service}"' in text
    assert 'bash "${SCRIPT_DIR}/install_driver_service.sh"' in text
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


def test_thunder_runtime_env_has_no_hidden_profile_or_deployment_identity() -> None:
    text = _read("scripts/deploy/thunder/runtime-env.sh")

    assert "LINGTU_ENV:=real" in text
    assert "LINGTU_PROFILE:=" not in text
    assert "export LINGTU_PROFILE" not in text
    assert "LINGTU_MODULE_TRANSPORT:=local" in text
    assert "LINGTU_PROFILE_ADAPTER:=" not in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=local" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=" in text
    assert "LINGTU_SIMULATION_ONLY:=0" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER:=1" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE:=local_driver" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY:=module_graph_driver" in text
    assert "LINGTU_LIVOX_LIDAR_IP:=" in text
    assert "LINGTU_LIVOX_HOST_IP:=" in text
    assert "LINGTU_LIVOX_NET_IFACE:=" in text
    assert "LINGTU_LIVOX_LIDAR_IP:=192.168" not in text
    assert "LINGTU_LIVOX_HOST_IP:=192.168" not in text
    assert "LINGTU_LIVOX_NET_IFACE:=eth1" not in text
    assert (
        "LINGTU_MAP_ARTIFACT_CONVERTER:=${LINGTU_REPO}/bin/octoplanner3d_pcd_to_octomap"
    ) in text
    assert "LINGTU_MAPS_LIB" not in text
    assert "/opt/ros" not in text
    assert "ROS_DOMAIN_ID" not in text
    assert "RMW_IMPLEMENTATION" not in text


def test_field_python_runtime_is_resolved_once_and_reused() -> None:
    resolver = _read("scripts/deploy/python-runtime.sh")
    deploy = _read("scripts/deploy/deploy_robot.sh")
    installer = _read("scripts/deploy/thunder/install_services.sh")
    catalog_installer = _read("scripts/deploy/thunder/install_catalog_service.sh")
    driver_installer = _read("scripts/deploy/thunder/install_driver_service.sh")
    runtime_env = _read("scripts/deploy/thunder/runtime-env.sh")
    session_guard = _read("scripts/deploy/thunder/require_product_session.sh")
    livox_runner = _read("scripts/deploy/thunder/run_livox_dds.sh")
    host_unit = _read("scripts/deploy/thunder/lt-host.service")
    systemd_runner = _read("src/lingtu/real/systemd.py")
    operator_cli = _read("scripts/lingtu")

    assert "resolve_lingtu_python()" in resolver
    assert "Python 3.10 or newer is required" in resolver
    assert "install_lingtu_python_env()" in resolver
    assert "realpath" not in resolver
    assert "readlink -f" not in resolver
    for script in (deploy, installer, catalog_installer, driver_installer, operator_cli):
        assert "resolve_lingtu_python" in script
        assert "${PYTHON:-python3}" not in script
    assert "install_lingtu_python_env" in catalog_installer
    assert '${LINGTU_CONFIG_DIR}/python.env' in runtime_env
    assert "export LINGTU_PYTHON" in runtime_env
    assert '"${LINGTU_PYTHON}" - "${expected_role}"' in session_guard
    assert ': "${LINGTU_PYTHON:=python3}"' not in livox_runner
    assert "Environment=LINGTU_PYTHON=python3" not in host_unit
    assert 'exec "$${LINGTU_PYTHON}" -m lingtu.real.host' in host_unit
    assert "LINGTU_DDS_PROBE_SCRIPT" not in host_unit + systemd_runner
    assert "LINGTU_DDS_PROBE_BIN" in host_unit + systemd_runner
    assert "LINGTU_DDS_PROBE_ALLOW_BUILD" not in host_unit
    assert "sys.executable" not in systemd_runner


def test_thunder_main_lingtu_service_consumes_product_control_runtime() -> None:
    text = _read("scripts/deploy/thunder/lt-host.service")
    wants = next(line for line in text.splitlines() if line.startswith("Wants="))
    after = next(line for line in text.splitlines() if line.startswith("After="))

    assert "Description=LingTu Python application host" in text
    assert wants == "Wants=network-online.target"
    assert after == "After=network-online.target"
    assert "lt-driver.service" not in wants + after
    assert "lt-slam.service" not in wants + after
    assert "lt-nav.service" not in wants + after
    assert "lingtu-thunder-dds-endpoint.service" not in text
    assert "LINGTU_ENV=real" in text
    assert "LINGTU_MODULE_TRANSPORT=local" in text
    assert "NAV_MAP_DIR=" not in text
    assert "LINGTU_INSPECTION_DIR=/var/lib/lingtu/inspection" in text
    assert "LINGTU_OCTOPLANNER3D_MAP=" not in text
    assert "LINGTU_OCTOPLANNER3D_EXECUTABLE=" not in text
    assert "LINGTU_PROFILE=" not in text
    assert "LINGTU_PROFILE_ADAPTER=" not in text
    assert "LINGTU_PRODUCT=" not in text
    assert "LINGTU_SERVICE_DDS_CHECK=1" in text
    assert "LINGTU_DDS_PROBE_BIN=/opt/lingtu/current/bin/lingtu_dds_probe" in text
    assert "LINGTU_SERVICE_READINESS_JSON=/tmp/lingtu_service_readiness.json" in text
    assert "LINGTU_INSPECTION_LIBRARY=/opt/lingtu/current/lib/liblingtu_inspection.so" in text
    assert (
        "LINGTU_INSPECTION_EVIDENCE_BRIDGE_LIBRARY="
        "/opt/lingtu/current/lib/liblingtu_inspection_evidence_bridge.so" in text
    )
    assert "LINGTU_INSPECTION_EVIDENCE_DIR=/var/lib/lingtu/inspection_evidence" in text
    assert "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE=/dev/shm/lingtu/inspection_evidence_status.json" in text
    assert "LINGTU_CLOUD_VIEWER_MAX_HZ=2" in text
    assert "LINGTU_SCAN_VIEWER_MAX_HZ=10" in text
    assert "lingtu-teleop-dds.service" not in text
    assert "run_http_watchdog.sh" not in text
    assert "--readiness-url http://127.0.0.1:5050/ready" not in text
    assert "EnvironmentFile=/run/lingtu/session.env" in text
    assert "ExecStartPre=/bin/bash /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh host" in text
    assert '"$${LINGTU_PYTHON}" -m lingtu.real.host' in text
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


def test_field_driver_service_is_native_product_entrypoint() -> None:
    unit = _read("scripts/deploy/thunder/lt-driver.service")
    runner = _read("scripts/deploy/thunder/run_driver.sh")
    cmake = _read("src/drivers/real/motion/CMakeLists.txt")
    source = _read("src/drivers/real/motion/main.cpp")
    dds = _read("src/drivers/real/motion/dds.cpp")
    status = _read("src/drivers/real/motion/status.cpp")
    writer_gate = _read("src/drivers/real/motion/cmd_vel_writer_gate.cpp")
    doso = _read("src/drivers/real/motion/robots/doso/doso.cpp")
    core = _read("src/drivers/real/motion/core.cpp")

    assert "Description=LingTu native field driver" in unit
    assert "lt-nav.service" not in unit
    assert "lingtu-thunder-lite.service" not in unit
    assert "Environment=LINGTU_DRIVER_BIN=" not in unit
    assert "EnvironmentFile=/run/lingtu/session.env" in unit
    assert "LINGTU_DRIVER_STATUS_FILE=/dev/shm/lingtu/driver_status.json" in unit
    assert "lingtu-thunder-dds-endpoint.service" in unit
    assert "run_driver.sh" in unit
    assert "python" not in unit.lower()
    assert "/opt/ros" not in unit
    assert "build_driver.sh" in runner
    assert "${LINGTU_REPO}/bin/lingtu_driver" in runner
    assert 'exec "${BIN}"' in runner
    assert "find_package(CycloneDDS REQUIRED)" in cmake
    assert "find_package(BrainstemClient CONFIG REQUIRED)" in cmake
    assert "brainstem::client" in cmake
    assert "test_driver_core" in cmake
    assert "test_driver_io" in cmake
    assert "refresh_control(now)" in source
    assert "brainstem::Client" in doso
    assert "grpc::" not in doso
    assert "best_effort_stop(ActionReason::Fault)" in source
    assert "best_effort_stop(ActionReason::Shutdown)" in source
    assert "dropped_disconnected" in source
    assert "core.poll(now)" in source
    assert "cmd_vel_writer_gate.update(dds.matchedCommandWriters())" in source
    assert "if (!writer_decision.ready)" in source
    assert "body->stop()" in source
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
    text = _read("scripts/deploy/thunder/lt-slam.service")
    runner = _read("scripts/deploy/thunder/run_slam_dds.sh")

    assert "Description=LingTu C++ CycloneDDS SLAM runtime" in text
    assert "After=network-online.target" in text
    assert "Wants=network-online.target" in text
    assert "After=network-online.target lt-lidar.service" not in text
    assert "Wants=network-online.target lt-lidar.service" not in text
    assert "systemd-time-wait-sync.service" not in text
    assert "wait_for_time_sync.sh" not in text
    assert "time-sync.target" not in text
    assert "lt-lidar.service" not in text
    assert "robot-lidar.service" not in text
    assert "Environment=LINGTU_SLAM_BIN=" not in text
    assert "LINGTU_SLAM_BACKEND=fastlio2" in text
    assert "LINGTU_SLAM_MODE=localization" in text
    assert "NAV_MAP_DIR=" not in text
    assert "LINGTU_SLAM_MAP=" not in text
    assert "LINGTU_SLAM_TRACK_SEED_FILE=/var/lib/lingtu/maps/active/track_seed.json" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_X=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_Y=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_Z=" not in text
    assert "LINGTU_SLAM_TRACK_INITIAL_YAW=" not in text
    assert "Environment=LINGTU_SLAM_CONFIG=" not in text
    assert "${LINGTU_SLAM_CONFIG:?" in runner
    assert "config/robots/doso/thunder_v4" not in text + runner
    assert "LINGTU_DDS_DOMAIN_ID=0" in text
    assert "LINGTU_SLAM_STATUS_JSON=/tmp/lingtu_slam_status.json" in text
    assert "LINGTU_SLAM_LIDAR_SCAN_SNAPSHOT_HZ=10" in text
    assert "run_slam_dds.sh" in text
    assert "native SLAM DDS runtime is missing or not executable" in runner
    assert "build_slam_core.sh" in runner
    assert "${LINGTU_REPO}/bin/slamd" in runner
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
    text = _read("tools/robot/setup_network.sh")

    assert "NetworkManager-wait-online.service" in text
    assert "ip addr replace ${HOST_IP}/${SUBNET_MASK} dev \\$IFACE" in text
    assert "ip route replace ${LIDAR_SUBNET} dev \\$IFACE src ${HOST_IP}" in text
    assert "nmcli device set \\$IFACE managed no" not in text
    assert "ip link set \\$IFACE down" not in text


def test_native_mapd_service_is_packaged_as_a_strict_cpp_boundary() -> None:
    unit = _read("scripts/deploy/thunder/lt-maps.service")
    build = _read("scripts/build/build_mapd.sh")
    package_release = _read("scripts/deploy/package_native_release.sh")
    cmake = _read("src/maps/CMakeLists.txt")
    main = _read("src/maps/cpp/mapd/main.cpp")
    dds = _read("src/maps/cpp/mapd/dds.cpp")
    topics = _read("src/message/cpp/topics.hpp")

    assert "Description=Native live maps and map service runtime" in unit
    assert "After=network-online.target" in unit
    assert "lt-slam.service" not in unit
    assert "LINGTU_MAPD_BIN=/opt/lingtu/current/bin/mapd" in unit
    assert "LINGTU_PRUNE_BIN=/opt/lingtu/current/bin/prune" in unit
    assert (
        "LINGTU_MAP_ARTIFACT_CONVERTER=/opt/lingtu/current/bin/"
        "octoplanner3d_pcd_to_octomap"
    ) in unit
    assert "LINGTU_MAPD_STATUS_FILE=/dev/shm/lingtu/mapd_status.json" in unit
    assert "LINGTU_MAPD_QUERY_SOCKET=/run/lingtu-mapd/mapd.sock" in unit
    assert ' --query-socket "${LINGTU_MAPD_QUERY_SOCKET}"' in unit
    assert "Group=lingtu" in unit
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
    assert "native map prune runtime is missing or not executable" in unit
    assert "native OctoMap converter is missing or not executable" in unit
    assert "python" not in unit.lower()
    assert 'PRUNE="${PRUNE_BUILD_DIR}/prune"' in build
    assert 'OCTOMAP_BUILD_DIR="${LINGTU_OCTOPLANNER3D_BUILD_DIR:-' in build
    assert 'export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}' in build
    assert 'LINGTU_PRUNE_ERASOR2=OFF \\' in build
    assert 'CMAKE_BUILD_TYPE="${BUILD_TYPE}" \\' in build
    assert 'bash "${ROOT}/scripts/build/build_prune.sh"' in build
    assert "native map prune runtime is missing after build" in build
    assert 'bash "${ROOT}/scripts/build/build_octoplanner3d.sh" --require-pcl' in build
    assert "native OctoMap converter is missing after build" in build
    assert "lingtu_maps_mapd_save_coordinator_test" in build
    assert "lingtu_maps_save_map_test" in build
    assert "build/prune/prune" in package_release
    assert "build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap" in package_release
    assert "packager accepted a mapd bundle without prune" in package_release
    assert "packager accepted a mapd bundle without the OctoMap converter" in package_release
    assert "native-release/build/prune/prune" in package_release
    assert (
        "native-release/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap"
        in package_release
    )
    assert "live/voxel/local-collision rate limit" in main
    assert "const bool scene_pending = publications.scene.Pending(state);" in main
    assert "if (scene_pending && now >= next_scene)" in main
    for deadline in ("next_cloud", "next_map", "next_scene", "next_state"):
        assert f"{deadline} = now + Period" not in main
        assert f"AdvanceDeadline({deadline}," in main
    realtime_publish = dds.split("bool Dds::PublishRealtimeClouds", 1)[1].split(
        "bool Dds::PublishMapLayers", 1
    )[0]
    map_publish = dds.split("bool Dds::PublishMapLayers", 1)[1].split(
        "bool Dds::PublishScene", 1
    )[0]
    assert '"local_collision"' in realtime_publish
    assert '"local_collision"' not in map_publish
    assert "state.extended_layers_enabled ? snapshot.live_cloud : empty_cloud" in dds

    assert "-DLINGTU_MAPS_BUILD_MAPD=ON" in build
    for target in (
        "mapd",
        "lingtu_maps_mapd_engine_test",
        "lingtu_maps_mapd_service_dispatch_test",
        "lingtu_maps_mapd_dds_test",
        "lingtu_maps_mapd_uds_test",
    ):
        assert target in build
    assert 'cd "${BUILD_DIR}"' in build
    assert "ctest --test-dir" not in build
    assert "add_executable(mapd" in cmake
    for topic in (
        "kMapsState",
        "kMapsLiveCloud",
        "kMapsVoxelCloud",
        "kMapsLocalCollision",
        "kMapsAccumulatedCloud",
        "kMapsOccupancy",
        "kMapsElevation",
        "kMapsEsdf",
        "kMapsScene",
    ):
        assert topic in topics


def test_thunder_traversability_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lt-terrain.service")
    source = _read("src/nav/cpp/endpoint/traversability/main.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    topics = _read("src/message/cpp/topics.hpp")
    idl = _read("src/message/idl/messages.idl")

    assert "Description=LingTu native traversability DDS producer" in text
    assert "lt-slam.service" not in text
    assert "require_product_session.sh traversability" in text
    assert "LINGTU_TRAVERSABILITY_DDS_BIN=/opt/lingtu/current/bin/lingtu_traversability_dds" in text
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
    assert '--sensor-offset-x-m "${LINGTU_NAV_SENSOR_OFFSET_X_M}"' in text
    assert '--sensor-offset-y-m "${LINGTU_NAV_SENSOR_OFFSET_Y_M}"' in text
    assert '--sensor-offset-z-m "${LINGTU_NAV_SENSOR_OFFSET_Z_M}"' in text
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
    assert "install-enable-default" in _read("scripts/deploy/thunder/install_catalog_service.sh")
    assert "double publish_hz{10.0}" in source
    assert "double slow_hz{1.0}" in source
    assert "double tick_hz{50.0}" in source
    assert "bool exploration_enabled{false}" in source
    assert 'cfg.exploration_enabled = cfg.product == "explore";' in source
    assert "std::optional<nav_endpoint::RollingExplorationMap> rolling_exploration" in source
    assert "if (rolling_exploration)" in source
    traversability_sources = cmake.split("add_executable(lingtu_traversability_dds", 1)[1].split(")", 1)[0]
    assert "input/gate.cpp" in traversability_sources
    assert "slow_tolerance_s" in source
    assert "next_slow_publish += slow_period_s" in source
    assert "next_slow_publish = now + 1.0 / cfg.slow_hz" not in source
    assert "kNavTerrainMap" in source
    assert "kNavTerrainMapExt" in source
    assert "kNavMapClearing" in source
    assert "kNavCloudClearing" in source
    assert '#include "message/cpp/qos.hpp"' in source
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
    assert "lingtu_dds_contracts" in cmake
    assert "target_link_libraries(lingtu_traversability_dds PRIVATE OpenMP::OpenMP_CXX)" in cmake
    assert '"/nav/map_clearing", "rt/nav/map_clearing"' in topics
    assert '"/nav/cloud_clearing", "rt/nav/cloud_clearing"' in topics
    assert "struct Bool" in idl


def test_thunder_livox_dds_service_publishes_native_sdk2_stream() -> None:
    text = _read("scripts/deploy/thunder/lt-lidar.service")
    runner = _read("scripts/deploy/thunder/run_livox_dds.sh")
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
    assert "Environment=LINGTU_LIVOX_BIN=" not in text
    assert "LINGTU_LIVOX_CONFIG_DIR=/opt/lingtu/config/livox" in text
    assert "Environment=LINGTU_LIVOX_NET_IFACE=" not in text
    assert "LINGTU_LIVOX_LIDAR_FRAME=lidar_link" in text
    assert "run_livox_dds.sh" in text
    assert "native Livox DDS publisher is missing or not executable" in runner
    assert "build_livox_sdk2_stream.sh" in runner
    assert "${LINGTU_REPO}/bin/livox_sdk2_stream" in runner
    assert "Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json" not in text
    assert "ensure_mid360_config_file" in runner
    assert "select_livox_host_ip" not in runner
    assert "${LINGTU_LIVOX_HOST_IP:?" in runner
    assert "${LINGTU_LIVOX_LIDAR_IP:?" in runner
    assert "${LINGTU_LIVOX_NET_IFACE:?" in runner
    assert "require_product_session.sh lidar" in runner
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


@pytest.mark.skipif(os.name == "nt", reason="Runtime contract requires native Bash")
def test_time_sync_waiter_runtime_contract() -> None:
    completed = subprocess.run(
        [
            "bash",
            str(ROOT / "tests" / "scripts" / "deploy" / "test_wait_for_time_sync.sh"),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert "wait_for_time_sync tests passed" in completed.stdout


@pytest.mark.skipif(os.name == "nt", reason="Runtime contract requires native Bash")
def test_python_runtime_selector_preserves_virtualenv_executable() -> None:
    completed = subprocess.run(
        [
            "bash",
            str(ROOT / "tests" / "scripts" / "deploy" / "test_python_runtime.sh"),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert "python runtime tests passed" in completed.stdout


@pytest.mark.skipif(os.name == "nt", reason="Runtime contract requires native Bash")
def test_product_session_guard_accepts_staged_plan_before_current_commit() -> None:
    completed = subprocess.run(
        [
            "bash",
            str(ROOT / "tests" / "scripts" / "deploy" / "test_require_product_session.sh"),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert completed.returncode == 0, completed.stderr
    assert "product session guard tests passed" in completed.stdout


def test_thunder_camera_dds_service_is_optional_and_fails_without_native_publisher() -> None:
    text = _read("scripts/deploy/thunder/lt-camera.service")
    runner = _read("scripts/deploy/thunder/run_camera_dds.sh")
    build = _read("scripts/build/build_camera_dds.sh")
    cmake = _read("src/drivers/real/camera/native/CMakeLists.txt")
    nav_cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    source = _read("src/drivers/real/camera/native/camera_dds.cpp")
    topics = _read("src/message/cpp/topics.hpp")
    idl = _read("src/message/idl/messages.idl")

    assert "Description=LingTu native camera DDS publisher" in text
    assert "Environment=LINGTU_CAMERA_DDS_BIN=" not in text
    assert "Environment=LINGTU_ORBBEC_CAPTURE_BIN=" not in text
    assert "LINGTU_ORBBEC_PRODUCT_ID=0x0800" in text
    assert "LINGTU_ORBBEC_DEVICE_INDEX=0" in text
    assert "LINGTU_ORBBEC_CONNECT_TIMEOUT_MS=10000" in text
    assert "LINGTU_CAMERA_COLOR_WIDTH=640" in text
    assert "LINGTU_CAMERA_DEPTH_WIDTH=640" in text
    assert "run_camera_dds.sh" in text
    assert "native camera DDS publisher is missing or not executable" in runner
    assert "build_camera_dds.sh" in runner
    assert "build_orbbec_native.sh" in runner
    assert "${LINGTU_REPO}/bin/lingtu_camera_dds" in runner
    assert "${LINGTU_REPO}/bin/orbbec_capture" in runner
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
    helper = _read("scripts/deploy/thunder/install_catalog_service.sh")
    assert 'REQUESTED_ENABLE="${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}"' in helper
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
    text = _read("scripts/deploy/thunder/lt-nav.service")
    config = _read("src/nav/cpp/endpoint/nav/runtime/config/parse.cpp")

    assert "LINGTU_INSPECTION_DIR=/var/lib/lingtu/inspection" in text
    assert "LINGTU_NAV_STATUS_S=0.2" in text
    assert "LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES=18" in text
    assert "LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS=640" in text
    assert 'applyEnvDouble(cfg.status_s, "LINGTU_NAV_STATUS_S")' in config
    assert '"LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES"' in config
    assert '"LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS"' in config


def test_cmu_path_library_comes_from_the_product_session() -> None:
    service = _read("scripts/deploy/thunder/lt-nav.service")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    assert "Environment=LINGTU_LOCAL_PLANNER_PATHS=" not in service
    assert "EnvironmentFile=/run/lingtu/session.env" in service
    assert "CMU Product session is missing LINGTU_LOCAL_PLANNER_PATHS" in runner
    for asset in (
        "startPaths.ply",
        "pathList.ply",
        "paths.ply",
        "correspondences.txt",
        "search_radius.txt",
    ):
        assert asset in runner
    assert "go2|thunder" in runner


def test_nav_runner_uses_product_bound_map_artifacts_without_a_map_root() -> None:
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    assert "NAV_GLOBAL_PLANNER" in runner
    assert "OCTOPLANNER_MAP_PATH" in runner
    assert "FAR_OCCUPANCY_PATH" in runner
    assert "NAV_MAP_DIR" not in runner


def test_thunder_nav_dds_service_diagnoses_missing_endpoint_binary() -> None:
    text = _read("scripts/deploy/thunder/lt-nav.service")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")
    endpoint_bootstrap = _read("src/nav/cpp/endpoint/nav/main.cpp")
    endpoint_loop = _read("src/nav/cpp/endpoint/nav/runtime/loop.cpp")
    input_projector = "\n".join(
        _read(f"src/nav/cpp/endpoint/nav/input/{name}")
        for name in ("pose.cpp", "map.cpp", "health.cpp")
    )
    source = "\n".join(
        [
            endpoint_bootstrap,
            input_projector,
            _read("src/nav/cpp/endpoint/nav/control/autonomy.cpp"),
            _read("src/nav/cpp/endpoint/nav/safety/command.cpp"),
            _read("src/nav/cpp/endpoint/nav/input/planner.hpp"),
            _read("src/nav/cpp/endpoint/nav/input/planner.cpp"),
            _read("src/nav/cpp/endpoint/nav/runtime/time.hpp"),
            _read("src/nav/cpp/endpoint/nav/runtime/config/build.cpp"),
        ]
    )
    config_source = _read("src/nav/cpp/endpoint/nav/runtime/config/parse.cpp")
    dds_codec = _read("src/nav/cpp/endpoint/nav/dds/codec.cpp")

    assert "Description=LingTu native navigation DDS endpoint" in text
    assert "Wants=network-online.target lt-terrain.service" not in text
    assert "Environment=LINGTU_NAV_DDS_BIN=" not in text
    assert [line for line in text.splitlines() if line.startswith("EnvironmentFile=")] == [
        "EnvironmentFile=-/opt/lingtu/current/etc/lingtu/release-runtime.env",
        "EnvironmentFile=-/etc/lingtu/nav.env",
        "EnvironmentFile=/run/lingtu/session.env",
    ]
    assert "run_nav_dds.sh" in text
    assert "NAV_GLOBAL_PLANNER=" not in text
    assert "OCTOPLANNER_MAP_PATH=" not in text
    assert "FAR_OCCUPANCY_PATH=" not in text
    assert "LINGTU_NAV_CONTROL_MODE=" not in text
    assert "LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S=1.5" in text
    assert "LINGTU_NAV_DDS_MAX_OBSTACLE_POINTS=5000" in text
    assert "LINGTU_NAV_STATUS_FILE=/dev/shm/lingtu/nav_endpoint_status.json" in text
    assert "User=lingtu" in text
    assert "Group=lingtu" in text
    assert "StateDirectory=lingtu" in text
    assert "LINGTU_NAV_ESTOP_LATCH_FILE=/var/lib/lingtu/nav_estop_latched" in text
    assert "LINGTU_NAV_GEOFENCE_FILE=/var/lib/lingtu/nav_geofences.dat" in text
    assert "LINGTU_NAV_STATUS_S=0.2" in text
    assert "NAV_GLOBAL_PLANNER" in runner
    assert "OCTOPLANNER_MAP_PATH" in runner
    assert "FAR_OCCUPANCY_PATH" in runner
    assert 'case "${LINGTU_SLAM_MODE}" in' in runner
    assert "mapping|none)" in runner
    assert 'OCTOPLANNER_MAP_PATH=""' in runner
    assert 'FAR_OCCUPANCY_PATH=""' in runner
    assert "LINGTU_ACTIVE_" not in runner
    assert "--max-speed-mps" in config_source
    assert "--max-accel-mps2" in config_source
    assert 'exec "${LINGTU_NAV_DDS_BIN}"' in runner
    assert "${LINGTU_REPO}/bin/navd" in runner
    assert "nav_args" not in runner
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
    assert "LINGTU_NAV_SENSOR_OFFSET_Z_M" in config_source
    assert 'applyEnvInt(cfg.domain_id, "LINGTU_DDS_DOMAIN_ID")' in config_source
    assert 'applyEnvDouble(cfg.tick_hz, "LINGTU_NAV_DDS_TICK_HZ")' in config_source
    assert 'applyEnvDouble(cfg.status_s, "LINGTU_NAV_STATUS_S")' in config_source
    assert "cfg.check_obstacle = parseBool" in config_source
    assert "config.check_obstacle && config.use_traversability" in source
    assert "data.planner_obstacles.assign(" in source
    assert "sensorOriginFromBody(" in source
    assert "obstacle_xyzh," in source
    assert "live_obstacles.stats()" in source
    assert "constexpr double kLayerInflationM = 0.0;" in source
    assert "buildExecutorConfig(cfg)" in source
    assert "buildLocalPlannerParams(cfg)" in source
    assert "cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan" in source
    assert "out.footprintPadding = planner_obstacle_margin_m;" in source
    assert "cfg.teleop_obstacle_margin_m + cfg.live_obstacle_inflation_radius_m" not in source
    assert "params.nominalDt = 1.0 / cfg.tick_hz;" in source
    assert "dynamicClusters(32, stamp_s)" in source
    assert "dynamicClusters(32, wall_now_s)" in source
    assert "const auto dynamic_clusters = live_obstacles.dynamicClusters(32, now);" not in source
    assert (
        "obstacles_.snapshot(state_.obstacle_xyzh, config_.max_obstacle_points, "
        "state_.last_cloud_s)" in source
    )
    assert "if (xyzh.empty())" in source
    assert "obstacle_snapshot_dirty" in source
    assert "timing.obstacle_snapshot_last_ms" in source
    assert "timing.motion_update_last_ms" in source
    assert "InputGate input_gate" in endpoint_bootstrap
    assert "state_.input_gate_state = gate_.evaluate(input);" in input_projector
    assert "path_active_for_tick && !input_gate_state.ready" in endpoint_loop
    navigation_state_publish = endpoint_loop.split(
        "(void)dds.publish(OutputEvent{navigation_state.sample(NavigationStateContext{", 1
    )[1].split("})});", 1)[0]
    assert "input_gate_state.ready" in navigation_state_publish
    assert "input_gate_state.reason" in navigation_state_publish
    assert "TransformBuffer pose_buffer" in source
    assert "pose_buffer_.sample(stamp_s, config_.cloud_pose_max_gap_s)" in source
    assert "headerStampSeconds(msg.header)" in dds_codec
    assert "cloud_sync.pose_rejected" in source
    status = _read("src/nav/cpp/endpoint/nav/status/nav_status_writer.cpp")
    assert "motion_layer" in status
    assert "last_sensor_origin" in status
    assert "dynamic_objects" in status
    assert "active_path_blockage_overlay" in status
    assert "local_planner_footprint" not in status
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
    assert "final_control_.finalize(FinalInput{" in source
    assert "output.reason = final.reason" in source
    assert "cloud_pose_rejected" in status
    assert "cloud_stamp_rejected" in status
    assert "last_pose_gap_s" in status
    assert '\\"terrain_map_ext_role\\": \\"diagnostics\\"' in status
    assert "cloud_stale" in _read("src/nav/cpp/tests/endpoint/test_input_gate.cpp")


def test_native_endpoint_accepts_only_typed_command_and_operator_motion_inputs() -> None:
    loop = _read("src/nav/cpp/endpoint/nav/runtime/loop.cpp")
    runtime = _read("src/nav/cpp/endpoint/nav/dds/runtime.cpp")
    runtime_h = _read("src/nav/cpp/endpoint/nav/dds/runtime.hpp")
    config = _read("src/nav/cpp/endpoint/nav/runtime/config/parse.cpp")
    status = _read("src/nav/cpp/endpoint/nav/status/nav_status_writer.cpp")
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
    bootstrap = _read("src/nav/cpp/endpoint/nav/main.cpp")
    loop = _read("src/nav/cpp/endpoint/nav/runtime/loop.cpp")
    controller = _read("src/nav/cpp/endpoint/nav/runtime/navigation.cpp")
    terminal_transaction = _read("src/nav/cpp/endpoint/nav/status/goal_terminal_transaction.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")

    callback = bootstrap.split("goal_plan_actions.publish_status", 1)[1].split(
        "goal_plan_actions.inspection_active", 1
    )[0]
    assert "NavigationGoalStatusOutbox" in bootstrap
    assert "GoalTerminalStatusDelivery" in bootstrap
    assert "GoalReplanRuntimeCoordinator" in bootstrap
    assert "goal_status_outbox.record(status)" in callback
    assert "dds.writeNavigationGoalStatus" not in callback
    assert "NavigationGoalStatusOutput{" in bootstrap
    assert "dds.writeNavigationGoalStatus" not in bootstrap
    assert "nav/status/navigation_goal_status_outbox.cpp" in cmake
    assert "nav/status/goal_terminal_status_delivery.cpp" in cmake
    assert "nav/runtime/navigation.cpp" in cmake
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
    assert "if (!goal_terminal_delivery_.isCommitted(runtime_result.terminal_intent_id))" in terminal_advance
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
    loop = _read("src/nav/cpp/endpoint/nav/runtime/loop.cpp")
    terminal_transaction = _read("src/nav/cpp/endpoint/nav/status/goal_terminal_transaction.cpp")

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

    terminal_barrier = loop.split("auto terminal_ingress", 1)[1].split("GoalTaskCancelRouter", 1)[0]
    assert "navigation_runtime_controller.terminalPending()" in terminal_barrier

    inspection_busy = loop.split("inspection_tick_input.goal_plan_busy =", 1)[1].split(";", 1)[0]
    assert "goal_plan.snapshot().busy" in inspection_busy
    assert "navigation_runtime_controller.terminalPending()" in inspection_busy
    assert "staged_inspection_goal" in loop
    assert 'completeGoalDispatch(false, "goal_terminal_pending"' not in loop


def test_nav_endpoint_uses_relative_height_when_cloud_has_no_height_field() -> None:
    native = _read("src/nav/cpp/endpoint/nav/main.cpp")
    projector = _read("src/nav/cpp/endpoint/nav/input/map.cpp")
    messages = _read("src/nav/cpp/endpoint/nav/dds/codec.cpp")
    planner = _read("src/nav/cpp/endpoint/nav/input/planner.cpp")
    runtime = _read("src/nav/cpp/endpoint/nav/dds/runtime.cpp")
    runtime_h = _read("src/nav/cpp/endpoint/nav/dds/runtime.hpp")
    text = "\n".join([native, projector, messages, planner, runtime, runtime_h])

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
    assert "clearPlanState" in text
    assert "obstacle_xyzh.clear()" in text
    assert "data.planner_obstacles.assign(" in planner
    assert "const std::vector<float> &measured" in planner
    assert "measured.begin()" in planner
    assert "out.obstacles = &data.planner_obstacles" in planner
    assert "appendPredictedObstacleCloud(" in planner
    assert "registered_share" not in planner
    assert "terrain_share" not in planner
    status = _read("src/nav/cpp/endpoint/nav/status/nav_status_writer.cpp")
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


def test_thunder_catalog_installer_owns_unit_installation() -> None:
    helper = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert "runtime.service_catalogs.thunder" in helper
    assert "install-unit" in helper
    assert "thunder-runtime-env.sh" in helper
    assert "ros2-env.sh" not in helper
    assert "run_dds_endpoint_service.py" not in helper
    assert "../s100p/install_services.sh" not in helper


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


def test_retired_localization_units_are_stop_only_field_tombstones() -> None:
    for rel_path in (
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        assert not (ROOT / rel_path).exists()

    assert not (ROOT / "integrations/super_lio").exists()

    real_env = _read("config/runtime_graph/envs/real.yaml").lower()
    processes = real_env.split("processes:", 1)[1].split("conflicts:", 1)[0]
    assert "super_lio" not in processes
    assert "super-lio" not in processes

    conflicts = real_env.split("conflicts:", 1)[1].split("endpoints:", 1)[0]
    tombstones = {
        "robot-super-lio.service",
        "robot-super-lio-relocation.service",
        "super_lio.service",
        "super_lio_relocation.service",
        "robot-hba.service",
        "robot-genz-icp.service",
        "slam_pgo.service",
    }
    for tombstone in tombstones:
        assert f"- {tombstone}" in conflicts

    installer = _read("scripts/deploy/thunder/install_services.sh").lower()
    assert "super_lio" not in installer
    assert "super-lio" not in installer

    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime

    resolved = resolve_product_host_runtime("nav", "real", robot="unitree/go2")
    plan = compile_run_plan(resolved.product, resolved.env, robot="unitree/go2")
    assert tombstones <= set(plan.stop_before_start)


def test_native_nav_endpoint_has_a_release_install_manifest() -> None:
    endpoint_cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    inspection_cmake = _read("src/nav/inspection/CMakeLists.txt")
    packager = _read("scripts/deploy/package_native_release.sh")
    service = _read("scripts/deploy/thunder/lt-nav.service")

    for target in (
        "navd",
        "lingtu_traversability_dds",
        "lingtu_explore_dds",
        "lingtu_nav_control",
    ):
        assert target in endpoint_cmake
    runtime_targets = endpoint_cmake.split("foreach(_target IN ITEMS", 1)[1].split(")", 1)[0]
    assert "lingtu_motion_mock_dds" not in runtime_targets
    assert "_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS" in endpoint_cmake
    assert "lingtu_nav_client" in endpoint_cmake
    assert "lingtu_inspection_evidence_bridge" in endpoint_cmake
    assert "LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}" in endpoint_cmake
    assert "DESTINATION ${CMAKE_INSTALL_DATADIR}/lingtu/cmu_paths" in endpoint_cmake
    assert 'set(_LINGTU_CMU_PATH_SOURCE "${_NAV_CPP_DIR}/planning/local/cmu/paths")' in endpoint_cmake
    assert '"${_LINGTU_CMU_PATH_SOURCE}/go2"' in endpoint_cmake
    assert '"${_LINGTU_CMU_PATH_SOURCE}/thunder"' in endpoint_cmake
    assert "CMU_PATH_PROFILES=(go2 thunder)" in packager
    assert "Native navigation install is missing CMU" in packager
    assert "LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}" in inspection_cmake
    assert "EnvironmentFile=-/opt/lingtu/current/etc/lingtu/release-runtime.env" in service


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
    assert '(cd "$BUILD_DIR" && ctest --output-on-failure)' in nav
    assert "ctest --test-dir" not in nav
    assert "required navigation test binary is missing" in nav
    assert '[[ ! -x "$BUILD_DIR/$required_test"' in nav
    assert '&& ! -x "$BUILD_DIR/endpoint/$required_test" ]]' in nav
    assert "test_path_follower_core" in nav
    assert "test_local_planner_core" in nav
    assert "test_nav_client" in nav
    assert "test_teleop_safety" in nav
    endpoint_tests = _read("src/nav/cpp/tests/endpoint/CMakeLists.txt")
    assert "test_nav_endpoint_config" in endpoint_tests
    nav_cmake = _read("src/nav/cpp/tests/CMakeLists.txt")
    assert "test_path_follower_core" in nav_cmake
    assert "test_local_planner_core" in nav_cmake


def test_motion_mock_dds_closes_cmd_vel_to_odom_loop_without_hardware() -> None:
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    source = _read("src/nav/cpp/endpoint/tools/mock.cpp")

    assert "add_executable(lingtu_motion_mock_dds" in cmake
    assert "tools/mock.cpp" in cmake
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
    text = _read("src/nav/cpp/endpoint/tools/navctl.cpp")

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
    assert '#include "message/cpp/qos.hpp"' in text
    assert "qosFor(" in text
    assert "dds_qset_reliability" not in text
    assert "dds_wait_for_acks" in text
    assert "waitForMatchedReader(writer, label)" in text
    assert 'publish_clear(lingtu::message::kNavMapClearing, "map_clearing")' in text
    assert 'publish_clear(lingtu::message::kNavCloudClearing, "cloud_clearing")' in text


def test_nav_control_exposes_typed_exploration_lifecycle() -> None:
    text = _read("src/nav/cpp/endpoint/tools/navctl.cpp")

    assert "explore <start|pause|resume|stop> RUN_ID SESSION_ID [REASON]" in text
    assert 'arg == "--request-id"' in text
    assert 'cfg.command += "-" + action' in text
    assert "client.exploration().start(" in text
    assert "client.exploration().pause(" in text
    assert "client.exploration().resume(" in text
    assert "client.exploration().stop(" in text
    assert "kNavExplorationCommand" not in text


def test_typed_navigation_client_uses_application_ack_as_authority() -> None:
    source = _read("src/nav/cpp/client/client.cpp")

    assert "active_request_id, pending, wait_deadline, final_ack_attempt" in source
    assert "NavigationCommandAck is already available" in source
    assert "dds_wait_for_acks(nav_command_request)" not in source


def test_native_nav_endpoint_uses_shared_dds_qos_catalog() -> None:
    runtime_source = _read("src/nav/cpp/endpoint/nav/dds/runtime.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    tests = _read("src/nav/cpp/tests/endpoint/CMakeLists.txt")

    assert '#include "message/cpp/qos.hpp"' in runtime_source
    assert "qos_for_topic(topic_name)" in runtime_source
    assert "dds_qset_reliability" not in runtime_source
    assert "navd" in cmake
    assert "lingtu_dds_contracts" in cmake
    assert "nav/input/obstacle.cpp" in cmake
    assert "test_motion_layer" in tests
    assert "test_nav_endpoint_messages" in tests
    assert "test_input_gate" in tests
    assert "test_pose_buffer" in tests


def test_native_motion_publishers_use_canonical_body_frame() -> None:
    sources = (
        _read("src/nav/cpp/endpoint/nav/dds/runtime.cpp"),
        _read("src/nav/cpp/endpoint/tools/navctl.cpp"),
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
    assert "python3 -m lingtu.control" in installer
    assert "scripts/deploy/package_native_release.sh" in release_guide
    assert "scripts/ota/" not in scripts_index
    for retired in ("build_nav_package.sh", "deploy_to_robot.sh", "generate_manifest.py"):
        assert retired not in scripts_index
        assert retired not in release_guide

    assert "--packages-select fastlio2 local_planner" not in combined
    assert "--packages-select local_planner" not in combined
    assert "local_planner terrain_analysis terrain_analysis_ext" not in combined
    assert "bash scripts/build/build_octoplanner3d.sh" in build_guide
    assert "Product default: native planner kernels, no ROS2" in build_guide
    assert "ROS 2 Humble Desktop is optional" in build_guide
    assert "make build           # source ROS Humble" not in build_guide


def test_scripts_readme_uses_product_control_as_field_entrypoint() -> None:
    text = _read("scripts/README.md")

    assert "deploy/deploy_robot.sh" in text
    assert "scripts/lingtu switch teleop --robot unitree/go2 --env real" in text
    assert "scripts/lingtu status --robot unitree/go2 --env real" in text
    assert "scripts/lingtu stop --robot unitree/go2 --env real" in text
    assert "only executes `python -m lingtu.control`" in text
    assert "Build and deploy the old ROS OTA package" not in text
    assert "validate_lcm_jsonl_feed.py" not in text


def test_robot_ops_is_a_thin_product_control_adapter() -> None:
    text = _read("scripts/lingtu")

    assert "LINGTU_GATEWAY_ENV_FILE:-/etc/lingtu/gateway.env" in text
    assert 'source "${gateway_env}"' in text
    assert 'exec "$LINGTU_PYTHON" -m lingtu.control "$@"' in text
    assert "cmd_" not in text
    assert "systemctl" not in text


def test_native_endpoint_uses_and_reports_compiled_product_motion_parameters() -> None:
    config = _read("src/nav/cpp/endpoint/nav/runtime/config/parse.cpp")
    endpoint_config = _read("src/nav/cpp/endpoint/nav/runtime/config/build.cpp")
    status = _read("src/nav/cpp/endpoint/nav/status/nav_status_writer.cpp")

    for key in (
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
    assert "params.minSpeed = cfg.path_follower_min_speed_mps" in endpoint_config
    assert "params.baseLookAheadDis = cfg.path_follower_lookahead_m" in endpoint_config
    assert "params.stopDisThre = cfg.path_follower_goal_tolerance_m" in endpoint_config
    assert "out.follower = followerParams(cfg)" in endpoint_config
    assert "out.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m" in endpoint_config
    assert "out.teleop_intent_max_deviation_deg =" in endpoint_config
    assert "teleop_planner_horizon_m" in status
    assert "teleop_planner_max_deviation_deg" in status
    assert "native_product" in status
    assert "nav_loop" in status


def test_field_mode_units_require_one_valid_transient_product_session() -> None:
    guard_path = "/opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh"
    expected_roles = {
        "scripts/deploy/thunder/lt-lidar.service": "lidar",
        "scripts/deploy/thunder/lt-slam.service": "slam",
        "scripts/deploy/thunder/lt-maps.service": "maps",
        "scripts/deploy/thunder/lt-terrain.service": "traversability",
        "scripts/deploy/thunder/lt-nav.service": "nav",
        "scripts/deploy/thunder/lt-explore.service": "explore",
        "scripts/deploy/thunder/lt-camera.service": "camera",
        "scripts/deploy/thunder/lt-host.service": "host",
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
    ):
        assert f"${{{required}:?" in guard
    assert 'expected_plan="${session_root}/plan-${LINGTU_PRODUCT_SESSION_ID}.json"' in guard
    assert 'expected_role="${1:-${LINGTU_EXPECTED_ROLE:-}}"' in guard
    assert 'process.get("name") == expected_role' in guard
    assert "current.json" not in guard
    assert "current.get(" not in guard
    assert "teleop|teleop_avoid|map|explore|nav|tracking|inspection" in guard
    assert "tare_explore" not in guard


def test_product_switch_control_owns_fail_closed_cleanup() -> None:
    source = _read("src/lingtu/real/switch.py")

    assert "SwitchRequest," in source
    assert "def execute_switch(" in source
    assert "control._apply_plan_for_switch(" in source
    assert "control._quiesce_plan_for_switch(plan)" in source
    assert "backend.rollback_session(session_stage)" in source
    assert "disable_boot_ownership" not in source
    assert '"active' + '-product.json"' not in source


def test_product_switch_requires_confirmed_stop_before_runtime_staging() -> None:
    source = _read("src/lingtu/real/switch.py")

    stop = "backend.stop_motion(current_product)"
    map_stage = "backend.stage_map(map_name)"
    config_stage = "backend.stage_session("
    apply = "control._apply_plan_for_switch("
    assert source.index(stop) < source.index(map_stage)
    assert source.index(map_stage) < source.index(config_stage)
    assert source.index(config_stage) < source.index(apply)


def test_product_nav_switch_commits_only_after_runtime_readiness() -> None:
    source = _read("src/lingtu/real/switch.py")

    stop = "backend.stop_motion(current_product)"
    map_stage = "backend.stage_map(map_name)"
    apply = "control._apply_plan_for_switch("
    localization = "backend.prepare_localization("
    readiness = "backend.wait_navigation("
    commit = "_commit_current_run("

    assert source.index(stop) < source.index(map_stage)
    assert source.index(map_stage) < source.index(apply)
    assert source.index(apply) < source.index(localization)
    assert source.index(localization) < source.index(readiness)
    assert source.index(readiness) < source.index(commit)


def test_product_switch_uses_gateway_availability_before_runtime_readiness() -> None:
    runner = _read("src/lingtu/real/systemd.py")

    assert "thunder_service_spec(service)" in runner
    assert "http_check=True" in runner
    assert '"http://127.0.0.1:5050/ready"' in runner
    assert 'for field in ("data_ready", "non_motion_safe")' in runner
    assert 'for field in ("failed_modules", "critical_failed_modules")' in runner


def test_product_nav_switch_failure_never_commits_current_run() -> None:
    source = _read("src/lingtu/real/switch.py")
    transaction = source.split("def execute_switch(", 1)[1].split("\ndef _lifecycle(", 1)[0]
    commit_at = transaction.index("_commit_current_run(")
    failure_at = transaction.index("except Exception as exc:", commit_at)
    failure = transaction[failure_at:]

    assert 'report.status = "rollback_failed"' in failure
    assert 'report.status = "failed_stopped"' in failure
    assert 'report.status = "stop_unconfirmed"' in failure
    assert 'report.status = "failed"' in failure
    assert "control._quiesce_plan_for_switch(plan)" in failure
    assert "backend.rollback_session(session_stage)" in failure
    assert "_commit_current_run" not in failure


def test_motion_smoke_gate_delegates_runtime_setup_to_product_control() -> None:
    gate = _read("src/diagnostics/field/motion_smoke.py")

    assert '["switch", "nav", "--env", args.env' in gate
    assert 'stop_args = ["stop", "--env", args.env]' in gate
    assert "/api/v1/session/start" not in gate
    assert "nav_relocalize_saved_map" not in gate


def test_system_acceptance_gate_matches_that_nav_parity_plan() -> None:
    gate = _read("src/diagnostics/field/system_acceptance.py")
    motion_gate = _read("src/diagnostics/field/motion_smoke.py")

    assert '["switch", "nav", "--env", args.env' in gate
    assert 'stop_args = ["stop", "--env", args.env]' in gate
    assert '"diagnostics.field.doctor"' in gate
    assert '"diagnostics.field.soak"' in gate
    assert '"diagnostics.field.map_artifacts"' in gate
    assert '"--require-occupancy"' in gate
    assert '"--expected-data-source", "thunder"' in gate
    assert "validate_saved_map_plan" in gate
    assert "requested_map_validate_plan.json" in gate
    assert 'root / "relocalization"' in gate
    assert '"saved_map_relocalization"' in gate
    assert "--with-relocalization" in gate
    assert "--initial-pose" in gate
    assert '"diagnostics.field.motion_smoke"' in gate
    assert "--allow-motion" in gate
    assert "motion-smoke requires --allow-motion" in motion_gate
    assert '"diagnostics.field.runtime_evidence"' in motion_gate


def test_rerun_gateway_live_is_ros_free_gateway_viewer() -> None:
    path = ROOT / "tools" / "visualization" / "rerun_gateway_live.py"
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
