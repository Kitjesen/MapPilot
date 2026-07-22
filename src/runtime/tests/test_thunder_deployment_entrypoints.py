from __future__ import annotations

import ast
import os
import re
import runpy
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


def _exec_start_lines(unit_text: str) -> list[str]:
    return [
        line
        for line in unit_text.splitlines()
        if line.startswith("ExecStart=") or line.startswith("ExecStartPre=") or line.startswith("ExecStartPost=")
    ]


def test_canonical_thunder_deploy_script_uses_product_profile() -> None:
    text = _read("scripts/deploy/deploy_thunder.sh")

    assert "LINGTU_DEPLOY_PROFILE:-nav" in text
    assert 'lingtu.py "${PROFILE}" --daemon' in text
    assert "git reset --hard" not in text
    assert "git pull --ff-only" in text
    assert "scripts/deploy/thunder/runtime-env.sh" in text
    assert "LINGTU_ENDPOINT:=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT:=dds" in text
    assert "LINGTU_ENDPOINT_CONTRACT:=thunder_field_" + "lcm_v1" not in text
    assert '[ -f "/opt/ros/humble/setup.bash" ]' not in text
    assert "SOURCE_ROS2=0" in text
    assert "ros2|sim_ros2|*-ros2|ros-compat|legacy" in text
    assert "lite|thunder-lite|basic|thunder-basic" in text
    assert "Building production navigation and driver" in text
    assert 'bash "${REPO}/scripts/build/build_maps.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_nav_kernel.sh" --clean' in text
    assert 'bash "${REPO}/scripts/build/build_nav_endpoint.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_orbbec_native.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_camera_dds.sh"' in text
    assert 'bash "${REPO}/scripts/build/build_driver.sh"' in text
    assert 'require_nav_kernel(context="Thunder deployment")' in text


def test_legacy_s100p_deploy_script_is_only_a_wrapper() -> None:
    text = _read("scripts/deploy/deploy_s100p.sh")

    assert "deploy_thunder.sh" in text
    assert "exec " in text
    assert "git reset --hard" not in text
    assert "lingtu.py nav" not in text


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
    assert "install_lite_service.sh" in text
    assert "Unknown Thunder service install mode" in text
    assert "Usage: $0 [<catalog-mode>|lite|ros-compat]" in text
    assert "Catalog modes:" in text
    assert "Compatibility modes: lite thunder-lite basic thunder-basic ros-compat legacy" in text
    assert "../s100p/install_services.sh" in text
    assert "ros2-env.sh" in text
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" in text
    assert "Refusing to install legacy ROS compatibility services by default" in text
    assert 'exec bash "${LEGACY_INSTALLER}" "${SCRIPT_DIR}/../s100p"' in text
    assert "ros-compat|legacy" in text


def test_thunder_ros2_env_helper_is_the_deployment_compat_boundary() -> None:
    text = _read("scripts/deploy/thunder/ros2-env.sh")

    assert "/opt/ros/${LINGTU_ROS_DISTRO}/setup.bash" in text
    assert "LINGTU_ROS_OVERLAY_SETUP" in text
    assert "RMW_IMPLEMENTATION" in text
    assert "Product entrypoints should use LingTu profiles" in text


def test_thunder_lite_runtime_env_has_no_ros_setup() -> None:
    text = _read("scripts/deploy/thunder/runtime-env.sh")

    assert "LINGTU_PROFILE:=thunder-lite" in text
    assert "LINGTU_MODULE_TRANSPORT:=local" in text
    assert "LINGTU_ENDPOINT:=thunder_lite" in text
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


def test_thunder_lite_service_is_standalone_product_entrypoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu-thunder-lite.service")

    assert "Description=LingTu Thunder Lite runtime" in text
    assert "thunder-runtime-env.sh" in text
    assert 'lingtu.py "${LINGTU_PROFILE}" --no-repl' in text
    assert "--daemon" not in text
    assert "ros2-env.sh" not in text
    assert "/opt/ros" not in text


def test_thunder_main_lingtu_service_uses_field_dds_endpoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu.service")
    wants = next(line for line in text.splitlines() if line.startswith("Wants="))

    assert "Description=LingTu Thunder navigation runtime" in text
    assert "lingtu-driver.service" in text
    assert "lingtu-slam-dds.service" in wants
    assert "lingtu-thunder-dds-endpoint.service" not in text
    assert "LINGTU_PROFILE=nav" in text
    assert "LINGTU_MODULE_TRANSPORT=local" in text
    assert "LINGTU_ENDPOINT=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT=dds" in text
    assert "LINGTU_ENDPOINT_CONTRACT=thunder_field_dds_v1" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER=0" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE=endpoint_only" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY=driver" in text
    assert "LINGTU_MANAGE_SESSION_SERVICES=0" in text
    assert "LINGTU_SERVICE_DDS_CHECK=1" in text
    assert "LINGTU_DDS_PROBE_SCRIPT=/opt/lingtu/current/scripts/diagnostics/dds_probe.py" in text
    assert "LINGTU_SERVICE_READINESS_JSON=/tmp/lingtu_service_readiness.json" in text
    assert (
        "LINGTU_INSPECTION_LIBRARY="
        "/opt/lingtu/current/build/nav_endpoint/inspection/liblingtu_inspection.so"
        in text
    )
    assert (
        "LINGTU_INSPECTION_EVIDENCE_BRIDGE_LIBRARY="
        "/opt/lingtu/current/build/nav_endpoint/liblingtu_inspection_evidence_bridge.so"
        in text
    )
    assert "LINGTU_INSPECTION_EVIDENCE_DIR=/home/sunrise/data/lingtu/inspection_evidence" in text
    assert "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE=/dev/shm/lingtu/inspection_evidence_status.json" in text
    assert "LINGTU_CLOUD_VIEWER_MAX_HZ=2" in text
    assert "LINGTU_SCAN_VIEWER_MAX_HZ=10" in text
    assert "lingtu-teleop-dds.service" not in text
    assert 'lingtu.py "${LINGTU_PROFILE}" --no-repl' in text
    assert "ros2-env.sh" not in text


def test_thunder_lite_service_installer_does_not_delegate_to_legacy_ros_services() -> None:
    text = _read("scripts/deploy/thunder/install_lite_service.sh")

    assert "thunder-runtime-env.sh" in text
    assert "lingtu-thunder-lite.service" in text
    assert "systemctl daemon-reload" in text
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text


def test_thunder_dds_endpoint_service_is_standalone_product_entrypoint() -> None:
    text = _read("scripts/deploy/thunder/lingtu-thunder-dds-endpoint.service")

    assert "Description=LingTu Thunder typed DDS endpoint" in text
    assert "thunder-runtime-env.sh" in text
    assert "LINGTU_PROFILE=nav" in text
    assert "LINGTU_ENDPOINT=thunder_field" in text
    assert "LINGTU_ENDPOINT_TRANSPORT=dds" in text
    assert "LINGTU_ENDPOINT_CONTRACT=thunder_field_dds_v1" in text
    assert "LINGTU_ENDPOINT_SOURCES=thunder_field" in text
    assert "LINGTU_ENDPOINT_SOURCE=" in text
    assert "LINGTU_ENDPOINT_JSONL_PATH=" in text
    assert "LINGTU_ENDPOINT_JSONL_COMMAND=" in text
    assert "LINGTU_ENDPOINT_JSONL_RATE_HZ=0" in text
    assert "LINGTU_BRAINSTEM_HOST=127.0.0.1" in text
    assert "LINGTU_BRAINSTEM_PORT=13145" in text
    assert "LINGTU_BRAINSTEM_REQUIRE_SDK=1" in text
    assert "LINGTU_BRAINSTEM_AUTO_ENABLE=0" in text
    assert "LINGTU_BRAINSTEM_AUTO_STANDUP=0" in text
    assert "LINGTU_BRAINSTEM_SAFE_SITDOWN=0" in text
    assert "LINGTU_BRAINSTEM_SAFE_DISABLE=0" in text
    assert "LINGTU_BRAINSTEM_CMD_TIMEOUT_MS=200" in text
    assert "LINGTU_ENABLE_ROBOT_DRIVER=0" in text
    assert "LINGTU_COMMAND_OUTPUT_MODE=endpoint_only" in text
    assert "LINGTU_HARDWARE_CONTROL_BOUNDARY=dds_endpoint_source" in text
    assert "run_dds_endpoint_service.py" in text
    assert '--source "${LINGTU_ENDPOINT_SOURCES}"' in text
    assert '--contract "${LINGTU_ENDPOINT_CONTRACT}"' in text
    assert "--fail-closed-on-missing-python-dds" in text
    assert "bash -c " in text
    assert "bash -lc " not in text
    assert "ros2-env.sh" not in text
    assert "/opt/ros" not in text


def test_thunder_driver_service_is_native_product_entrypoint() -> None:
    unit = _read("scripts/deploy/thunder/lingtu-driver.service")
    runner = _read("scripts/deploy/thunder/run_driver.sh")
    cmake = _read("src/drivers/real/thunder/native/CMakeLists.txt")
    source = _read("src/drivers/real/thunder/native/main.cpp")
    brainstem = _read("src/drivers/real/thunder/native/brainstem.cpp")
    core = _read("src/drivers/real/thunder/native/core.cpp")

    assert "Description=LingTu native Thunder driver" in unit
    assert "lingtu-nav-dds.service" in unit
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
    assert "find_package(gRPC CONFIG REQUIRED)" in cmake
    assert "test_driver_core" in cmake
    assert "test_driver_io" in cmake
    assert "refresh_control(now)" in source
    assert "WalkChecked" in brainstem
    assert "stub_->Walk(" not in brainstem
    assert "best_effort_stop(ActionReason::Fault)" in source
    assert "best_effort_stop(ActionReason::Shutdown)" in source
    assert "dropped_disconnected" in source
    assert "core.poll(now)" in source
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


def test_thunder_dds_endpoint_service_installer_is_no_ros() -> None:
    text = _read("scripts/deploy/thunder/install_dds_endpoint_service.sh")
    helper = _read("scripts/deploy/thunder/install_catalog_service.sh")

    assert "thunder-runtime-env.sh" in helper
    assert "install_catalog_service.sh" in text
    assert " endpoint" in text
    assert "lingtu-thunder-dds-endpoint.service" not in text
    assert "systemctl daemon-reload" in helper
    assert "../s100p/install_services.sh" not in text
    assert "ros2-env.sh" not in text
    assert "lingtu-livox-driver.service" not in text


def test_thunder_slam_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu-slam-dds.service")
    runner = _read("scripts/deploy/thunder/run_slam_dds.sh")

    assert "Description=LingTu C++ CycloneDDS SLAM runtime" in text
    assert "After=network-online.target lingtu-livox-dds.service" in text
    assert "Wants=network-online.target lingtu-livox-dds.service" in text
    assert "systemd-time-wait-sync.service" not in text
    assert "wait_for_time_sync.sh" not in text
    assert "time-sync.target" not in text
    assert "lingtu-livox-dds.service" in text
    assert "robot-lidar.service" not in text
    assert "LINGTU_SLAM_BIN=/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime" in text
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


def test_thunder_traversability_dds_service_runs_cpp_runtime() -> None:
    text = _read("scripts/deploy/thunder/lingtu-traversability-dds.service")
    installer = _read("scripts/deploy/thunder/install_traversability_dds_service.sh")
    source = _read("src/nav/cpp/endpoint/traversability_dds.cpp")
    cmake = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    topics = _read("src/message/cpp/dds_topics.hpp")
    idl = _read("src/message/idl/lingtu_slam.idl")

    assert "Description=LingTu native traversability DDS producer" in text
    assert "lingtu-slam-dds.service" in text
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
    assert "LINGTU_TRAVERSABILITY_SENSOR_OFFSET_X_M=-0.011" in text
    assert "LINGTU_TRAVERSABILITY_SENSOR_OFFSET_Y_M=-0.02329" in text
    assert "LINGTU_TRAVERSABILITY_SENSOR_OFFSET_Z_M=0.04412" in text
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
    assert "LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}" in _read("scripts/deploy/thunder/install_catalog_service.sh")
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
    source = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
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
    assert "LINGTU_NAV_ALLOW_LEGACY_MOTION_INPUTS=0" in text
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
    assert "--allow-legacy-motion-inputs" in runner
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
    assert "LINGTU_NAV_SENSOR_OFFSET_X_M=-0.011" in text
    assert "LINGTU_NAV_SENSOR_OFFSET_Y_M=-0.02329" in text
    assert "LINGTU_NAV_SENSOR_OFFSET_Z_M=0.04412" in text
    assert "LINGTU_TELEOP_MIN_MOTION_SPEED_MPS=0.03" in text
    assert "LINGTU_NAV_SENSOR_OFFSET_Z_M" in config_source
    assert "cfg.check_obstacle = parseBool" in config_source
    assert "cfg.check_obstacle && cfg.use_traversability_cost" in source
    assert "buildPlannerObstacleCloud(" in source
    assert "sensorOriginFromBody(" in source
    assert "obstacle_xyzh," in source
    assert "live_obstacles.stats()" in source
    assert "constexpr double kLayerInflationM = 0.0;" in source
    assert "nav_config.local_planner.footprintPadding = safety_config.obstacle_margin_m;" in source
    assert "cfg.teleop_obstacle_margin_m + cfg.live_obstacle_inflation_radius_m" in config_source
    assert "nav_config.path_follower.nominalDt = 1.0 / cfg.tick_hz;" in source
    assert "latest_dynamic_clusters = live_obstacles.dynamicClusters(32, cloud_stamp_s);" in source
    assert "latest_dynamic_clusters = live_obstacles.dynamicClusters(32, now);" in source
    assert "const auto dynamic_clusters = live_obstacles.dynamicClusters(32, now);" not in source
    assert "live_obstacles.snapshot(cfg.max_obstacle_points, last_cloud_s)" in source
    assert "if (xyzh.empty())" in source
    assert "obstacle_snapshot_dirty" in source
    assert "timing.obstacle_snapshot_last_ms" in source
    assert "timing.motion_update_last_ms" in source
    assert "InputGate input_gate" in source
    assert "input_gate.evaluate(" in source
    assert "!input_gate_state.ready" in source
    assert "TransformBuffer pose_buffer" in source
    assert "pose_buffer.sample(cloud_stamp_s, cfg.cloud_pose_max_gap_s)" in source
    assert "headerStampSeconds(msg.header)" in source
    assert "cloud_sync.pose_rejected" in source
    status = _read("src/nav/cpp/endpoint/nav_status_writer.cpp")
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
    assert "cloud_stale" in _read("src/nav/cpp/endpoint/test_input_gate.cpp")


def test_external_global_path_requires_driver_control_state() -> None:
    source = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    global_path_handler = source.split("dds.drainLegacyGlobalPath", 1)[1].split(
        "dds.drainTraversability", 1
    )[0]

    assert "driver_control_blocker()" in global_path_handler
    assert "path_driver_blocker" in global_path_handler
    assert "frames.path_rejected" in global_path_handler


def test_nav_endpoint_uses_relative_height_when_cloud_has_no_height_field() -> None:
    native = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    messages = _read("src/nav/cpp/endpoint/nav_endpoint_messages.cpp")
    runtime = _read("src/nav/cpp/endpoint/nav_dds_runtime.cpp")
    runtime_h = _read("src/nav/cpp/endpoint/nav_dds_runtime.hpp")
    text = "\n".join([native, messages, runtime, runtime_h])

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
    assert "clear_planner_terrain_inputs" in text
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
    status = _read("src/nav/cpp/endpoint/nav_status_writer.cpp")
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


def test_legacy_service_templates_have_thunder_descriptions() -> None:
    for rel_path in (
        "scripts/deploy/s100p/lidar.service",
        "scripts/deploy/s100p/slam.service",
        "scripts/deploy/s100p/slam_pgo.service",
        "scripts/deploy/s100p/localizer.service",
        "scripts/deploy/s100p/genz_icp.service",
        "scripts/deploy/s100p/hba.service",
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        text = _read(rel_path)

        assert "Description=Thunder " in text
        assert "Description=S100P " not in text


def test_legacy_service_templates_source_deploy_compat_env() -> None:
    for rel_path in (
        "scripts/deploy/s100p/lidar.service",
        "scripts/deploy/s100p/slam.service",
        "scripts/deploy/s100p/slam_pgo.service",
        "scripts/deploy/s100p/localizer.service",
        "scripts/deploy/s100p/genz_icp.service",
        "scripts/deploy/s100p/hba.service",
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ):
        text = _read(rel_path)

        assert "source /opt/lingtu/config/ros2-env.sh" in text
        assert "source /opt/ros/humble/setup.bash" not in text


def test_release_script_uses_thunder_product_wording() -> None:
    text = _read("scripts/deploy/cut_release.sh")

    assert "LingTu Thunder release" in text
    assert "S100P" not in text
    assert "\u95c1" not in text


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
    assert "LINGTU_RELEASE_REQUIRE_ROS2_COMPAT:-0" in text
    assert "ROS 2 compatibility package gate skipped" in text
    assert "Build first: cd $DEV_DIR && colcon build" not in text
    assert "RUNTIME_PKGS=" not in text
    assert "lingtu-livox-driver.service" not in text
    assert "emit_release_services" in text

    compat_line = next(line for line in text.splitlines() if line.startswith("ROS2_COMPAT_PKGS="))

    for legacy_pkg in (
        "local_planner",
        "sensor_scan_generation",
        "terrain_analysis",
        "terrain_analysis_ext",
        "pct_adapters",
        "pct_planner",
        "tare_planner",
    ):
        assert legacy_pkg not in compat_line

    assert "nav_kernel" not in compat_line
    assert "lingtu-thunder-dds-endpoint.service" in text
    assert "lingtu-livox-dds.service" in text
    assert "lingtu-slam-dds.service" in text
    assert "lingtu-nav-dds.service" in text
    assert "robot-fastlio2.service robot-localizer.service" in text


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
    assert "lingtu_slam_cyclone_runtime" in slam
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
    source = _read("src/nav/cpp/endpoint/motion_mock_dds.cpp")

    assert "add_executable(lingtu_motion_mock_dds" in cmake
    assert "motion_mock_dds.cpp" in cmake
    assert "kNavCmdVel" in source
    assert "kSlamOdometry" in source
    assert "kTf" in source
    assert "cmd.vx * c - cmd.vy * s" in source
    assert "cmd.vx * s + cmd.vy * c" in source
    assert 'fillHeader(out.header, stamp_s, "odom")' in source
    assert 'out.child_frame_id = const_cast<char*>("body")' in source
    assert 'fillHeader(out.transform.header, stamp_s, "map")' in source
    assert 'out.transform.child_frame_id = const_cast<char*>("odom")' in source
    assert '\\"lingtu.motion_mock.status.v1\\"' in source
    assert "LINGTU_MOTION_MOCK_STATUS_FILE" in source


def test_nav_control_external_path_is_explicit_legacy_smoke_only() -> None:
    text = _read("src/nav/cpp/endpoint/nav_control.cpp")

    assert "path X1 Y1 Z1 X2 Y2 Z2" in text
    assert "kNavGlobalPath" in text
    assert "dds_write(global_path)" in text
    assert 'waitForMatchedReader(writer, "global_path")' in text
    assert 'dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(global_path)"' in text
    assert "--allow-legacy-motion-inputs true" in text
    assert "product control uses typed Goal" in text
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
    text = _read("src/nav/cpp/endpoint/nav_control.cpp")

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
    source = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
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
        _read("src/nav/cpp/endpoint/nav_control.cpp"),
    )

    for source in sources:
        assert 'fillHeader(msg.header, nowSeconds(), "base_link")' not in source
    assert 'toDdsPath(path, "map")' in sources[0]
    assert 'toDdsPoseStamped(point, "map")' in sources[0]
    assert 'fillHeader(msg.header, nowSeconds(), "body")' in sources[1]


def test_ota_and_build_docs_do_not_recommend_legacy_ros2_planning_or_local_autonomy() -> None:
    push_script = _read("scripts/ota/push_to_robot.sh")
    package_script = _read("scripts/ota/build_nav_package.sh")
    ota_readme = _read("scripts/ota/README.md")
    build_guide = _read("docs/01-getting-started/BUILD_GUIDE.md")

    combined = "\n".join((push_script, package_script, ota_readme, build_guide))

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
    assert "legacy colcon-based OTA helpers" in ota_readme
    assert "Use these OTA scripts only when intentionally" in ota_readme


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
    assert "legacy OTA/perception scripts as explicit" in replacement_map
    assert "compatibility paths rather than product defaults" in replacement_map


def test_scripts_readme_uses_thunder_as_product_entrypoint() -> None:
    text = _read("scripts/README.md")
    perception_readme = _read("scripts/perception/README.md")
    root_readme = _read("README.md")
    repo_layout = _read("docs/REPO_LAYOUT.md")

    assert "deploy/deploy_thunder.sh" in text
    assert "deploy/cut_release.sh" in text
    assert "python lingtu.py thunder-nav" in text
    assert "Legacy ROS OTA package and deploy" in text
    assert "ROS2 compatibility live perception demos" in text
    assert "Gateway camera endpoints" in perception_readme
    assert "S100P" not in text
    assert "Ubuntu 22.04 + ROS2 Humble" not in root_readme
    assert "ROS2 Humble is optional for compatibility services" in root_readme
    assert "colcon build / test / format / lint" not in repo_layout
    assert "native/test wrappers plus ROS workspace compatibility" in repo_layout
    assert "ROS2 compatibility perception demos" in repo_layout


def test_doctor_is_ros_free_and_uses_profile_builder() -> None:
    path = ROOT / "scripts" / "diagnostics" / "doctor.py"
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
    assert "run_gateway_dataflow_checks" in text
    assert "require_nav_kernel" in text
    assert "LingTu native navigation kernel" in text
    assert "LINGTU_DOCTOR_ROS2" not in text
    assert "--skip-gateway" in text
    assert "blueprint_for_resolved_profile" in text
    assert "LINGTU_DOCTOR_AUTOSTART_SLAM" not in text
    assert "_brainstem_endpoint" in text
    assert 'config_dir / "brainstem.env"' in text
    assert 'port_open("127.0.0.1", 13145)' not in text


def test_doctor_resolves_brainstem_endpoint_from_service_environment(tmp_path: Path) -> None:
    env_file = tmp_path / "brainstem.env"
    env_file.write_text(
        "LINGTU_BRAINSTEM_HOST=192.168.114.10\n"
        "LINGTU_BRAINSTEM_PORT=14145\n",
        encoding="utf-8",
    )
    doctor = runpy.run_path(str(ROOT / "scripts" / "diagnostics" / "doctor.py"))
    resolve = doctor["_brainstem_endpoint"]

    assert resolve({}, env_file) == ("192.168.114.10", 14145)
    assert resolve(
        {
            "LINGTU_BRAINSTEM_HOST": "192.168.114.23",
            "LINGTU_BRAINSTEM_PORT": "13145",
        },
        env_file,
    ) == ("192.168.114.23", 13145)


def test_robot_ops_doctor_defaults_to_gateway_first_ros2_explicit() -> None:
    text = _read("scripts/lingtu")
    doctor_body = text.split("cmd_doctor() {", 1)[1].split("\n}\n\n# -- Subcommand: soak --", 1)[0]

    assert "[--ros2]" in text
    assert "--ros2) ros2=1" in text
    assert '[ "$ros2" = "1" ] && source_robot_env' in text
    assert "\n    source_robot_env\n" not in doctor_body
    assert 'cmd_doctor_json "$strict" "$non_motion" "$realtime" "$require_camera" "$ros2"' in text
    assert 'ros2_enabled = sys.argv[6] == "1"' in text
    assert "if ros2_enabled:" in text
    assert "ROS2 compatibility graph checks skipped; use --ros2" in text
    assert "camera.ros2_topics_skipped" in text
    assert "camera.gateway_snapshot" in text
    assert "$GW/api/v1/runtime/dataflow" in text
    assert "$GW/api/v1/camera/snapshot" in text
    assert '[ "$ros2" = "1" ] && command -v ros2' in text


def test_native_endpoint_uses_and_reports_compiled_product_motion_parameters() -> None:
    config = _read("src/nav/cpp/endpoint/nav_endpoint_config.cpp")
    endpoint = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    status = _read("src/nav/cpp/endpoint/nav_status_writer.cpp")
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
    assert "nav_config.waypoint_reached_m = cfg.waypoint_reached_m" in endpoint
    assert "nav_config.goal_reached_m = cfg.goal_reached_m" in endpoint
    assert "nav_config.path_follower.minSpeed = cfg.path_follower_min_speed_mps" in endpoint
    assert "nav_config.path_follower.baseLookAheadDis = cfg.path_follower_lookahead_m" in endpoint
    assert "nav_config.path_follower.stopDisThre = cfg.path_follower_goal_tolerance_m" in endpoint
    assert "nav_config.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m" in endpoint
    assert (
        "nav_config.teleop_intent_max_deviation_deg =" in endpoint
    )
    assert "teleop_planner_horizon_m" in status
    assert "teleop_planner_max_deviation_deg" in status
    assert (
        '--teleop-planner-horizon-m "${LINGTU_TELEOP_PLANNER_HORIZON_M}"'
        in runner
    )
    assert ('--teleop-planner-max-deviation-deg '
            '"${LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG}"') in runner
    assert "native_profile" in status
    assert "config_fingerprint" in status
    assert "nav_loop" in status
    assert "--max-speed-mps" not in runner


def test_robot_ops_has_product_mode_switch_entrypoint() -> None:
    text = _read("scripts/lingtu")
    stop_body = text.split("mode_stop_motion_and_session() {", 1)[1].split("\n}\n\nmode_unit_available()", 1)[0]
    boot_body = text.split("mode_persist_product_boot_ownership() {", 1)[1].split(
        "\n}\n\nmode_wait_nav_control_mode()", 1
    )[0]
    restart_body = text.split("mode_restart_product_stack() {", 1)[1].split("\n}\n\nmode_switch_preflight()", 1)[0]
    preflight_body = text.split("mode_switch_preflight() {", 1)[1].split(
        "\n}\n\nmode_start_session_for_target()", 1
    )[0]

    assert "cmd_mode()" in text
    assert "mode switch <product-profile>" in text
    assert "config/runtime_graph/products/*.yaml" in text
    assert "mode_switch_preflight" in text
    assert 'switch-plan "$current" "$target"' in text
    assert 'mode_profile_dropin "$target" "$endpoint"' in text
    assert 'mode_nav_endpoint_dropin "$MODE_TARGET_NATIVE_CONTROL_MODE"' in text
    assert "LINGTU_NAV_CONTROL_MODE=$control_mode" in text
    assert "MODE_TARGET_NATIVE_CONTROL_MODE=$(pjson" in text
    assert "MODE_TARGET_NATIVE_NAV_ENV=$(pjson" in text
    assert 'n=(d.get("product_mode_switch") or {}).get("native_nav_config") or {}' in text
    assert 'for key, value in sorted((n.get("environment") or {}).items())' in text
    assert 'done <<< "$MODE_TARGET_NATIVE_NAV_ENV"' in text
    assert "LINGTU_NAV_CONFIG_FINGERPRINT" in text
    assert "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS" in text
    assert "LINGTU_NAV_GOAL_REACHED_M" in text
    assert 'case "$profile" in' not in text
    assert "LINGTU_TELEOP_PLANNER_HORIZON_M" in text
    assert "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG" in text
    assert "LINGTU_NAV_CHECK_OBSTACLE=$MODE_TARGET_CHECK_OBSTACLE" in text
    assert "mode_start_planned_process" not in text
    assert "mode_wait_nav_control_mode" in text
    assert '"$py" -m lingtu.launcher apply "$product" --endpoint "$endpoint" --json' in restart_body
    assert 'mode_wait_nav_control_mode "$MODE_TARGET_NATIVE_CONTROL_MODE" 10' in restart_body
    assert "mode_persist_product_boot_ownership" in text
    assert "mode_persist_product_boot_ownership || {" in text
    assert 'mode_fail_closed_product_switch "Product boot ownership update failed"' in text
    assert "mode_plan_target_selected" in boot_body
    assert "MODE_TARGET_KNOWN_TARGETS" in boot_body
    assert "MODE_TARGET_STOP_TARGETS" in boot_body
    assert 'mode_set_unit_boot_enabled "$target" "$enabled" "$enabled"' in boot_body
    assert 'mode_set_unit_boot_enabled "$target" 0 0' in boot_body
    assert 'required_topics' not in preflight_body
    assert 'MODE_TARGET_NEEDS_NAV_ENDPOINT=$(mode_plan_bool nav)' in preflight_body
    assert 'MODE_TARGET_NEEDS_TRAVERSABILITY=$(mode_plan_bool traversability)' in preflight_body
    assert 'MODE_TARGET_NEEDS_INSPECTION_EVIDENCE=$(mode_plan_bool camera)' in preflight_body
    assert 'MODE_TARGET_NEEDS_EXPLORATION_ENDPOINT=$(mode_plan_bool explore)' in preflight_body
    assert 'actual=$(python3 - "$status_file"' in text
    assert "age_s <= 1.0" in text
    assert "__status_fresh_without_control_mode__" in text
    assert "LINGTU_NAV_CONTROL_MODE=$expected" in text
    assert 'mode_wait_nav_control_mode "$MODE_TARGET_NATIVE_CONTROL_MODE" 10' in text
    assert 'rm -f "${LINGTU_NAV_STATUS_FILE:-/dev/shm/lingtu/nav_endpoint_status.json}"' in restart_body
    assert 'svc_force_stop_unit "$target"' not in restart_body
    assert "mode_wait_inspection_evidence_ready()" in text
    assert '"$GW/api/v1/inspection/status"' in text
    assert 'mode_wait_inspection_evidence_ready 45' in text
    assert "inspection evidence never became ready" in text
    assert "/etc/systemd/system/lingtu.service.d" in text
    assert "LINGTU_PROFILE=$profile" in text
    assert "LINGTU_ENDPOINT=$endpoint" in text
    assert "mode_stop_motion_and_session" in text
    assert "$GW/api/v1/navigation/cancel" in stop_body
    assert "$GW/api/v1/stop" not in stop_body
    assert "$GW/api/v1/session/end" in text
    assert 'slam_dds_set_mode mapping ""' in text
    assert 'slam_dds_set_mode localization "$map_pcd"' in text
    assert '"product_session": sys.argv[3]' in text
    assert 'mode_restart_product_stack "$target" "$endpoint"' in text
    assert "mode)           shift; cmd_mode" in text
    assert "Mode $target requires --map NAME" in text
    assert "$GW/api/v1/mode" not in text


def test_failed_product_stack_restart_is_cleaned_up_fail_closed() -> None:
    text = _read("scripts/lingtu")
    switch_body = text.split("cmd_mode() {", 1)[1].split("\ncmd_map() {", 1)[0]
    abort_body = text.split("mode_abort_product_switch() {", 1)[1].split(
        "\n}\n\nmode_restart_product_stack()", 1
    )[0]
    fail_closed_body = text.split("mode_fail_closed_product_switch() {", 1)[1].split(
        "\n}\n\nmode_restart_product_stack()", 1
    )[0]

    assert 'if ! mode_restart_product_stack "$target" "$endpoint"; then' in switch_body
    assert "mode_fail_closed_product_switch" in switch_body
    assert "mode_abort_product_switch" in fail_closed_body
    assert "mode_stop_motion_and_session" in abort_body
    assert 'svc_force_stop_unit "$target"' in abort_body
    assert 'done <<< "$MODE_TARGET_STOP_TARGETS"' in abort_body


def test_product_switch_requires_confirmed_stop_before_mutating_runtime() -> None:
    text = _read("scripts/lingtu")
    stop_body = text.split("mode_stop_motion_and_session() {", 1)[1].split(
        "\n}\n\nmode_activate_saved_map_for_nav()", 1
    )[0]
    switch_body = text.split("cmd_mode() {", 1)[1].split("\ncmd_map() {", 1)[0]

    assert "|| true" not in stop_body
    assert 'cancel_ok=$(pjson "$cancel_raw"' in stop_body
    assert 'd.get("ok") is True' in stop_body
    assert 'command.get("accepted") is True' in stop_body
    assert 'session_ok=$(pjson "$session_raw"' in stop_body
    assert 'd.get("success") is True' in stop_body
    assert "mode_stop_motion_and_session || return 1" in switch_body
    assert switch_body.index("mode_stop_motion_and_session || return 1") < switch_body.index(
        'mode_activate_saved_map_for_nav "$map_name"'
    )


def test_robot_ops_full_stack_restart_restarts_timestamp_consumers_in_order() -> None:
    text = _read("scripts/lingtu")
    body = text.split("svc_restart_robot_stack() {", 1)[1].split("\n}\n\ncmd_svc()", 1)[0]

    stop_livox = body.index("svc_force_stop_unit lingtu-livox-dds.service")
    stop_slam = body.index("svc_force_stop_unit lingtu-slam-dds.service")
    stop_traversability = body.index("svc_force_stop_unit lingtu-traversability-dds.service")
    stop_nav = body.index("svc_force_stop_unit lingtu-nav-dds.service")
    assert stop_nav < stop_traversability < stop_slam < stop_livox

    start_livox = body.index("svc_restart_lidar_chain")
    start_slam = body.index("svc_start_unit lingtu-slam-dds.service")
    start_traversability = body.index("svc_start_unit lingtu-traversability-dds.service")
    start_nav = body.index("svc_start_unit lingtu-nav-dds.service")
    input_gate_ready = body.index("svc_wait_native_input_gate_ready 20")
    assert start_livox < start_slam < start_traversability < start_nav
    assert start_nav < input_gate_ready
def test_product_switch_failures_after_mutation_force_stopped_state() -> None:
    source = _read("scripts/lingtu")

    assert "mode_fail_closed_product_switch()" in source
    abort_start = source.index("mode_abort_product_switch()")
    abort_end = source.index("mode_restart_product_stack()", abort_start)
    abort_body = source[abort_start:abort_end]
    assert 'mode_set_unit_boot_enabled "$target" 0 0' in abort_body
    assert 'svc_force_stop_unit "$target"' in abort_body

    switch_start = source.index(
        'echo -e "${B}=== Product mode switch: $current -> $target ===${N}"'
    )
    switch_end = source.index(
        'echo -e "${G}PASS${N}: product mode is active: $target"', switch_start
    )
    switch_body = source[switch_start:switch_end]
    assert switch_body.count("mode_fail_closed_product_switch") >= 9




def test_product_nav_switch_activates_target_before_native_endpoint_restart() -> None:
    text = _read("scripts/lingtu")
    switch_body = text.split("cmd_mode() {", 1)[1].split("\ncmd_map() {", 1)[0]

    activate = 'mode_activate_saved_map_for_nav "$map_name"'
    preflight = 'mode_switch_preflight "$current" "$target" "$endpoint"'
    stop = "mode_stop_motion_and_session"
    restart = 'mode_restart_product_stack "$target" "$endpoint"'
    start_session = 'mode_start_session_for_target "$target"'
    wait_ready = 'mode_wait_navigation_ready "$map_name" 45'
    success = "PASS${N}: product mode is active: $target"

    assert activate in switch_body
    assert switch_body.index(preflight) < switch_body.index(activate)
    assert switch_body.index(stop) < switch_body.index(activate)
    assert switch_body.index(activate) < switch_body.index(restart)
    assert wait_ready in switch_body
    assert switch_body.index(start_session) < switch_body.index(wait_ready)
    assert switch_body.index(wait_ready) < switch_body.index(success)


def test_product_switch_uses_gateway_availability_before_session_readiness() -> None:
    launcher = _read("src/lingtu/launcher.py")
    manager = _read("src/runtime/service_manager.py")

    assert "thunder_service_spec(service)" in launcher
    assert "http_check=True" in launcher
    assert '"http://127.0.0.1:5050/api/v1/readiness"' in manager
    assert 'for field in ("data_ready", "non_motion_safe")' in manager
    assert 'if payload.get("data_ready") is False' in manager
    assert 'for field in ("failed_modules", "critical_failed_modules")' in manager


def test_product_nav_switch_aborts_session_when_relocalization_or_readiness_fails() -> None:
    text = _read("scripts/lingtu")
    switch_body = text.split("cmd_mode() {", 1)[1].split("\ncmd_map() {", 1)[0]

    start_guard = (
        'if ! mode_start_session_for_target "$target" "$map_name" "$relocalize" '
        '"$initial_explicit" "$initial_x" "$initial_y" "$initial_yaw"; then'
    )
    wait_guard = (
        'mode_wait_navigation_ready "$map_name" 45 '
        '"$MODE_TARGET_NATIVE_CONTROL_MODE" || {'
    )

    assert start_guard in switch_body
    assert wait_guard in switch_body
    assert (
        'mode_fail_closed_product_switch "Target product session failed to start"' in switch_body
    )
    assert 'mode_fail_closed_product_switch "Navigation readiness failed"' in switch_body


def test_nav_start_delegates_to_product_runtime_plan() -> None:
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


def test_robot_ops_system_acceptance_gate_matches_that_nav_parity_plan() -> None:
    text = _read("scripts/lingtu")
    body = text.split("cmd_system_acceptance() {", 1)[1].split("\n}\n\n# -- Subcommand: health --", 1)[0]
    cli_doc = _read("docs/04-deployment/lingtu_cli.md")

    assert "system-acceptance|that-nav-acceptance|acceptance-gate" in text
    assert "slam_dds_set_mode localization" in text
    assert "slam_dds_set_mode mapping" in text
    assert "No ROS2 graph inspection and no motion commands" in text
    assert "--ros2" not in body
    assert "cmd_runtime_audit --json" in body
    assert "cmd_doctor --non-motion --strict --json" in body
    assert "cmd_soak --duration" in body
    assert "--strict --json" in body
    assert "cmd_saved_map_artifact_gate" in body
    assert "--require-occupancy" in body
    assert "--expected-data-source thunder_field" in body
    assert "routecheck_require_no_active_command_source" in body
    assert "saved_map_plan_precheck" in body
    assert "/api/v1/maps/$map_name/validate_plan" in text
    assert "requested_map_validate_plan.json" in body
    assert "nav_relocalize_saved_map" in body
    assert "saved_map_relocalization" in body
    assert "cmd_routecheck --map" not in body
    assert "--with-relocalization" in text
    assert "--initial-pose" in text
    assert "motion-smoke|motioncheck|path-follower-check" in text
    assert "cmd_motion_smoke --map" in body
    assert '--initial-pose "$initial_x" "$initial_y" "$initial_yaw"' in body
    assert "cmd_routecompare --map" not in body
    assert "--allow-motion" in body
    assert "motion-smoke requires --allow-motion" in text
    assert "real-runtime evidence" in text

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
