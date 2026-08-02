#!/bin/bash
# Cut a new LingTu Thunder release on the field robot.
#
# Snapshot the current development checkout into /opt/lingtu/releases/<version>/,
# atomically swap /opt/lingtu/current, then reapply the committed Product
# RunPlan through ProductControl.
#
# Default release gates are native-first:
#   - The tested native navigation endpoint package must be built and installed.
#   - The selected global planner is immutable release metadata.
#   - OctoPlanner3D tools are required only for an OctoPlanner3D release.
#   - The nanobind Python kernel is optional and never substitutes for navd.
#
# Usage on the robot:
#   bash scripts/deploy/cut_release.sh v2.1.1
#
# Rollback:
#   restore the previous activation files and /opt/lingtu/current, then run:
#   bash /opt/lingtu/current/scripts/lingtu --env real svc reapply

set -euo pipefail

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
    echo "Usage: $0 <version>   (example: v2.1.1)"
    exit 1
fi
if [[ ! "$VERSION" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: version must look like v2.1.1 (got: $VERSION)"
    exit 1
fi

DEV_DIR="${LINGTU_DEV_DIR:-$HOME/data/SLAM/navigation}"
PYTHON_BIN="${LINGTU_PYTHON:-python3}"
RELEASES_DIR="${LINGTU_RELEASES_DIR:-/opt/lingtu/releases}"
TARGET_DIR="$RELEASES_DIR/$VERSION"
CURRENT_LINK="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"

load_committed_run_plan() {
    local fields=()

    mapfile -d '' -t fields < <(
        PYTHONPATH="${DEV_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}" \
            "$PYTHON_BIN" - <<'PY'
import json
import os
import sys
from pathlib import Path

from lingtu.product_lock import resolve_current_run_path
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan

current_path = resolve_current_run_path(environment=os.environ)
try:
    current = json.loads(current_path.read_text(encoding="utf-8"))
except FileNotFoundError as exc:
    raise RuntimeError(f"current ProductControl state is missing: {current_path}") from exc
except (OSError, UnicodeError, json.JSONDecodeError) as exc:
    raise RuntimeError(f"current ProductControl state is unreadable: {current_path}: {exc}") from exc
if not isinstance(current, dict):
    raise RuntimeError("current ProductControl state must be a JSON object")
if current.get("schema_version") != CURRENT_RUN_SCHEMA:
    raise RuntimeError("current ProductControl state has unsupported schema")

run_plan_value = current.get("run_plan_path")
if not isinstance(run_plan_value, str) or not run_plan_value.strip():
    raise RuntimeError("current ProductControl state requires run_plan_path")
run_plan_path = Path(run_plan_value).expanduser()
if not run_plan_path.is_absolute():
    raise RuntimeError("current RunPlan path must be absolute")
run_plan_path = run_plan_path.resolve()
plan = RunPlan.load(run_plan_path)
if current.get("fingerprint") != plan.fingerprint:
    raise RuntimeError("current RunPlan fingerprint does not match its record")
if current.get("product") != plan.product:
    raise RuntimeError("current Product does not match RunPlan")
if current.get("env") != plan.env:
    raise RuntimeError("current Env does not match RunPlan")
if plan.process_control != "systemd":
    raise RuntimeError("release activation requires a systemd-controlled RunPlan")

native_environment = plan.native_process_environment
if native_environment.get("LINGTU_PRODUCT") != plan.product:
    raise RuntimeError("RunPlan native Product does not match its identity")
control_mode = str(native_environment.get("LINGTU_NAV_CONTROL_MODE") or "").strip()
if not control_mode:
    raise RuntimeError("RunPlan does not declare its native control mode")

def selected_process(name: str):
    return next((process for process in plan.processes if process.name == name), None)

driver = selected_process("driver")
if driver is None or driver.manager != "systemd" or driver.lifecycle != "persistent":
    raise RuntimeError("current RunPlan requires one persistent systemd driver process")
maps = selected_process("maps")
if maps is not None and maps.manager != "systemd":
    raise RuntimeError("current RunPlan maps process must be systemd-controlled")

values = (
    str(run_plan_path),
    plan.product,
    plan.env,
    plan.fingerprint,
    control_mode,
    driver.target,
    maps.target if maps is not None else "",
)
sys.stdout.write("\0".join(values) + "\0")
PY
    )
    if [ "${#fields[@]}" -ne 7 ]; then
        echo "ERROR: failed to load the committed ProductControl RunPlan" >&2
        exit 1
    fi

    CURRENT_RUN_PLAN_PATH="${fields[0]}"
    CURRENT_PRODUCT="${fields[1]}"
    CURRENT_ENV="${fields[2]}"
    CURRENT_RUN_PLAN_FINGERPRINT="${fields[3]}"
    CURRENT_CONTROL_MODE="${fields[4]}"
    CURRENT_DRIVER_UNIT="${fields[5]}"
    CURRENT_MAPD_UNIT="${fields[6]}"
}

load_committed_run_plan
RELEASE_ENV="$CURRENT_ENV"
LINGTU_CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
LINGTU_SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
GATEWAY_URL="${LINGTU_GATEWAY_URL:-http://localhost:5050}"
NAV_ENDPOINT_BUILD_DIR="${LINGTU_NAV_ENDPOINT_BUILD_DIR:-$DEV_DIR/build/nav_endpoint}"
NAV_STATUS_FILE="${LINGTU_NAV_STATUS_FILE:-/dev/shm/lingtu/nav_endpoint_status.json}"
LIVOX_BUILD_DIR="${LINGTU_LIVOX_SDK2_STREAM_BUILD_DIR:-$DEV_DIR/build/livox_sdk2_stream}"
SLAM_BUILD_DIR="${LINGTU_SLAM_CORE_BUILD_DIR:-$DEV_DIR/build/slam_core}"
MAPS_BUILD_DIR="${LINGTU_MAPD_BUILD_DIR:-$DEV_DIR/build/maps}"
DDS_PROBE_BUILD_DIR="${LINGTU_DDS_PROBE_BUILD_DIR:-$DEV_DIR/build/dds_probe}"
DRIVER_BUILD_DIR="${LINGTU_DRIVER_BUILD_DIR:-$DEV_DIR/build/driver}"
RECORDING_BUILD_DIR="${LINGTU_NATIVE_RECORDING_BUILD_DIR:-$DEV_DIR/build/native-recording}"
LIVOX_RUNTIME_SOURCE="$LIVOX_BUILD_DIR/livox_sdk2_stream"
SLAM_RUNTIME_SOURCE="$SLAM_BUILD_DIR/slamd"
SLAM_CONTROL_SOURCE="$SLAM_BUILD_DIR/slamctl"
MAPD_RUNTIME_SOURCE="$MAPS_BUILD_DIR/mapd"
MAPCTL_RUNTIME_SOURCE="$MAPS_BUILD_DIR/lingtu-mapctl"
MAPS_LIBRARY_SOURCE="$MAPS_BUILD_DIR/liblingtu_maps.so"
DDS_PROBE_RUNTIME_SOURCE="$DDS_PROBE_BUILD_DIR/lingtu_dds_probe"
DRIVER_RUNTIME_SOURCE="$DRIVER_BUILD_DIR/lingtu_driver"
SELECTED_GLOBAL_PLANNER="${LINGTU_RELEASE_GLOBAL_PLANNER:-${LINGTU_NAV_GLOBAL_PLANNER:-octoplanner3d}}"
REUSE_VERIFIED_NATIVE_BUILD="${LINGTU_RELEASE_REUSE_VERIFIED_NATIVE_BUILD:-0}"
SELECTED_GLOBAL_PLANNER="${SELECTED_GLOBAL_PLANNER,,}"
case "$SELECTED_GLOBAL_PLANNER" in
    octo|octplanner|octoplanner3d)
        SELECTED_GLOBAL_PLANNER="octoplanner3d"
        DEFAULT_REQUIRE_OCTOPLANNER3D=1
        ;;
    far)
        DEFAULT_REQUIRE_OCTOPLANNER3D=0
        ;;
    *)
        echo "ERROR: unsupported release global planner: $SELECTED_GLOBAL_PLANNER" >&2
        echo "Supported planners: octoplanner3d, far" >&2
        exit 1
        ;;
esac
REQUIRE_OCTOPLANNER3D="${LINGTU_RELEASE_REQUIRE_OCTOPLANNER3D:-$DEFAULT_REQUIRE_OCTOPLANNER3D}"
REQUIRE_OCTOMAP_CONVERTER="${LINGTU_RELEASE_REQUIRE_OCTOMAP_CONVERTER:-$REQUIRE_OCTOPLANNER3D}"
REQUIRE_PYTHON_NAV_KERNEL="${LINGTU_RELEASE_REQUIRE_PYTHON_NAV_KERNEL:-0}"
REQUIRE_NAV_STATUS="${LINGTU_RELEASE_REQUIRE_NAV_STATUS:-1}"

for release_flag in \
    REQUIRE_OCTOPLANNER3D \
    REQUIRE_OCTOMAP_CONVERTER \
    REQUIRE_PYTHON_NAV_KERNEL \
    REQUIRE_NAV_STATUS \
    REUSE_VERIFIED_NATIVE_BUILD; do
    value="${!release_flag}"
    if [ "$value" != "0" ] && [ "$value" != "1" ]; then
        echo "ERROR: $release_flag must be 0 or 1 (got: $value)" >&2
        exit 1
    fi
done

NAV_ENDPOINT_RUNTIME_FILES=(
    navd
    lingtu_traversability_dds
    lingtu_explore_dds
    lingtu_nav_control
    lingtu_motion_mock_dds
    liblingtu_nav_client.so
    liblingtu_inspection_evidence_bridge.so
    inspection/liblingtu_inspection.so
)
RECORDING_RUNTIME_FILES=(
    lingtu_recorder
    lingtu_dds_recorder
    lingtu_dds_player
    lingtu_camera_recorder
    lingtu_camera_player
)
RELEASE_NATIVE_RUNTIME_FILES=(
    build/livox_sdk2_stream/livox_sdk2_stream
    build/slam_core/slamd
    build/slam_core/slamctl
    build/maps/mapd
    build/maps/lingtu-mapctl
    build/dds_probe/lingtu_dds_probe
    build/nav_endpoint/navd
    build/nav_endpoint/lingtu_traversability_dds
    build/driver/lingtu_driver
    build/native-recording/lingtu_recorder
    build/native-recording/lingtu_dds_recorder
    build/native-recording/lingtu_dds_player
    build/native-recording/lingtu_camera_recorder
    build/native-recording/lingtu_camera_player
)
RELEASE_NATIVE_LIBRARY_FILES=(
    build/maps/liblingtu_maps.so
)


require_reusable_artifact_fresh() {
    local artifact="$1"
    local label="$2"
    shift 2
    local source_root
    local stale_source

    if [ ! -e "$artifact" ]; then
        echo "ERROR: verified native build reuse requested, but $label is missing: $artifact" >&2
        exit 1
    fi
    for source_root in "$@"; do
        if [ ! -e "$source_root" ]; then
            echo "ERROR: cannot verify reused $label; source path is missing: $source_root" >&2
            exit 1
        fi
        stale_source="$(find -L "$source_root" -type f -newer "$artifact" -print -quit)"
        if [ -n "$stale_source" ]; then
            echo "ERROR: refusing stale reused $label: $artifact" >&2
            echo "  newer source: $stale_source" >&2
            echo "Rebuild normally or remove LINGTU_RELEASE_REUSE_VERIFIED_NATIVE_BUILD=1." >&2
            exit 1
        fi
    done
}

nav_endpoint_artifacts_ready() {
    local relative
    for relative in "${NAV_ENDPOINT_RUNTIME_FILES[@]}"; do
        if [ ! -e "$NAV_ENDPOINT_BUILD_DIR/$relative" ]; then
            return 1
        fi
    done
    return 0
}

ensure_nav_endpoint_artifacts() {
    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building and testing the native navigation endpoint package..."
        LINGTU_NAV_ENDPOINT_BUILD_DIR="$NAV_ENDPOINT_BUILD_DIR" \
            bash "$DEV_DIR/scripts/build/build_nav_endpoint.sh"
    fi
    if ! nav_endpoint_artifacts_ready; then
        echo "ERROR: native navigation endpoint package is incomplete: $NAV_ENDPOINT_BUILD_DIR" >&2
        local relative
        for relative in "${NAV_ENDPOINT_RUNTIME_FILES[@]}"; do
            if [ ! -e "$NAV_ENDPOINT_BUILD_DIR/$relative" ]; then
                echo "  missing: $relative" >&2
            fi
        done
        exit 1
    fi
    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "1" ]; then
        local relative
        for relative in "${NAV_ENDPOINT_RUNTIME_FILES[@]}"; do
            require_reusable_artifact_fresh \
                "$NAV_ENDPOINT_BUILD_DIR/$relative" "native nav endpoint $relative" \
                "$DEV_DIR/src/nav/cpp" \
                "$DEV_DIR/src/nav/inspection" \
                "$DEV_DIR/src/message/cpp" \
                "$DEV_DIR/src/message/idl" \
                "$DEV_DIR/scripts/build/build_nav_endpoint.sh"
        done
    fi
}

ensure_native_recording_artifacts() {
    local relative

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building and testing the native recording package..."
        LINGTU_NATIVE_RECORDING_BUILD_DIR="$RECORDING_BUILD_DIR" \
            LINGTU_RECORDING_BUILD_DDS=ON \
            bash "$DEV_DIR/scripts/build/build_native_recording.sh"
    fi
    for relative in "${RECORDING_RUNTIME_FILES[@]}"; do
        if [ ! -x "$RECORDING_BUILD_DIR/$relative" ]; then
            echo "ERROR: native recording executable is missing: $RECORDING_BUILD_DIR/$relative" >&2
            exit 1
        fi
    done
    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "1" ]; then
        for relative in "${RECORDING_RUNTIME_FILES[@]}"; do
            require_reusable_artifact_fresh \
                "$RECORDING_BUILD_DIR/$relative" "native recording $relative" \
                "$DEV_DIR/src/native/recording" \
                "$DEV_DIR/src/message/cpp" \
                "$DEV_DIR/src/message/idl" \
                "$DEV_DIR/scripts/build/build_native_recording.sh"
        done
    fi
}

ensure_field_native_artifacts() {
    ensure_nav_endpoint_artifacts
    ensure_native_recording_artifacts

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building native Livox DDS runtime..."
        LINGTU_LIVOX_SDK2_STREAM_BUILD_DIR="$LIVOX_BUILD_DIR" \
            LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON \
            bash "$DEV_DIR/scripts/build/build_livox_sdk2_stream.sh"
    fi
    if [ ! -x "$LIVOX_RUNTIME_SOURCE" ]; then
        echo "ERROR: native Livox DDS runtime is missing or not executable: $LIVOX_RUNTIME_SOURCE" >&2
        exit 1
    fi

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building native SLAM DDS runtime..."
        LINGTU_SLAM_CORE_BUILD_DIR="$SLAM_BUILD_DIR" \
            LINGTU_SLAM_BUILD_DDS_RUNTIME=ON \
            LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF \
            bash "$DEV_DIR/scripts/build/build_slam_core.sh"
    fi
    if [ ! -x "$SLAM_RUNTIME_SOURCE" ]; then
        echo "ERROR: native SLAM DDS runtime is missing or not executable: $SLAM_RUNTIME_SOURCE" >&2
        exit 1
    fi
    if [ ! -x "$SLAM_CONTROL_SOURCE" ]; then
        echo "ERROR: native SLAM control tool is missing or not executable: $SLAM_CONTROL_SOURCE" >&2
        exit 1
    fi

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building native maps runtime..."
        LINGTU_MAPD_BUILD_DIR="$MAPS_BUILD_DIR" \
            bash "$DEV_DIR/scripts/build/build_mapd.sh"
    fi
    if [ ! -x "$MAPD_RUNTIME_SOURCE" ]; then
        echo "ERROR: native maps runtime is missing or not executable: $MAPD_RUNTIME_SOURCE" >&2
        exit 1
    fi
    if [ ! -x "$MAPCTL_RUNTIME_SOURCE" ]; then
        echo "ERROR: native map control CLI is missing or not executable: $MAPCTL_RUNTIME_SOURCE" >&2
        exit 1
    fi
    if [ ! -f "$MAPS_LIBRARY_SOURCE" ]; then
        echo "ERROR: native maps service library is missing: $MAPS_LIBRARY_SOURCE" >&2
        exit 1
    fi

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building native DDS readiness probe..."
        LINGTU_DDS_PROBE_BUILD_DIR="$DDS_PROBE_BUILD_DIR" \
            bash "$DEV_DIR/scripts/build/build_dds_probe.sh"
    fi
    if [ ! -x "$DDS_PROBE_RUNTIME_SOURCE" ]; then
        echo "ERROR: native DDS readiness probe is missing or not executable: $DDS_PROBE_RUNTIME_SOURCE" >&2
        exit 1
    fi

    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "0" ]; then
        echo ">>> Building native Thunder driver..."
        LINGTU_DRIVER_BUILD_DIR="$DRIVER_BUILD_DIR" \
            bash "$DEV_DIR/scripts/build/build_driver.sh"
    fi
    if [ ! -x "$DRIVER_RUNTIME_SOURCE" ]; then
        echo "ERROR: native Thunder driver is missing or not executable: $DRIVER_RUNTIME_SOURCE" >&2
        exit 1
    fi
    if [ "$REUSE_VERIFIED_NATIVE_BUILD" = "1" ]; then
        require_reusable_artifact_fresh \
            "$LIVOX_RUNTIME_SOURCE" "native Livox DDS runtime" \
            "$DEV_DIR/src/drivers/real/lidar/sdk2_stream" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_livox_sdk2_stream.sh"
        require_reusable_artifact_fresh \
            "$SLAM_RUNTIME_SOURCE" "native SLAM DDS runtime" \
            "$DEV_DIR/src/localization/slam/cpp" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_slam_core.sh"
        require_reusable_artifact_fresh \
            "$SLAM_CONTROL_SOURCE" "native SLAM control tool" \
            "$DEV_DIR/src/localization/slam/cpp" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_slam_core.sh"
        require_reusable_artifact_fresh \
            "$MAPD_RUNTIME_SOURCE" "native maps runtime" \
            "$DEV_DIR/src/maps" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_mapd.sh"
        require_reusable_artifact_fresh \
            "$MAPCTL_RUNTIME_SOURCE" "native map control CLI" \
            "$DEV_DIR/src/maps" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_mapd.sh"
        require_reusable_artifact_fresh \
            "$MAPS_LIBRARY_SOURCE" "native maps service library" \
            "$DEV_DIR/src/maps" \
            "$DEV_DIR/scripts/build/build_mapd.sh"
        require_reusable_artifact_fresh \
            "$DDS_PROBE_RUNTIME_SOURCE" "native DDS readiness probe" \
            "$DEV_DIR/scripts/diagnostics/native" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_dds_probe.sh"
        require_reusable_artifact_fresh \
            "$DRIVER_RUNTIME_SOURCE" "native Thunder driver" \
            "$DEV_DIR/src/drivers/real/thunder/native" \
            "$DEV_DIR/src/message/cpp" \
            "$DEV_DIR/src/message/idl" \
            "$DEV_DIR/scripts/build/build_driver.sh"
    fi
}

find_nav_kernel_artifact() {
    find -L "$DEV_DIR/src" "$DEV_DIR/build/nav_kernel" \
        -maxdepth 1 -type f -name 'lingtu_nav_kernel*.so' -print -quit 2>/dev/null || true
}

ensure_nav_kernel_artifact() {
    NAV_KERNEL_ARTIFACT="$(find_nav_kernel_artifact)"
    if [ -n "$NAV_KERNEL_ARTIFACT" ]; then
        return 0
    fi

    echo ">>> Building LingTu native navigation kernel..."
    bash "$DEV_DIR/scripts/build/build_nav_kernel.sh" --clean
    NAV_KERNEL_ARTIFACT="$(find_nav_kernel_artifact)"
}

find_octoplanner3d_executable() {
    if [ -n "${LINGTU_OCTOPLANNER3D_EXECUTABLE:-}" ]; then
        if [ -f "$LINGTU_OCTOPLANNER3D_EXECUTABLE" ]; then
            printf '%s\n' "$LINGTU_OCTOPLANNER3D_EXECUTABLE"
        fi
        return 0
    fi

    for candidate in \
        "$NAV_ENDPOINT_BUILD_DIR/octoplanner3d_runtime/octoplanner3d_headless" \
        "$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_headless" \
        "$DEV_DIR/build/octoplanner3d_headless/Release/octoplanner3d_headless" \
        "$DEV_DIR/build/octoplanner3d_headless/Debug/octoplanner3d_headless" \
        "$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_headless.exe" \
        "$DEV_DIR/build/octoplanner3d_headless/Release/octoplanner3d_headless.exe" \
        "$DEV_DIR/build/octoplanner3d_headless/Debug/octoplanner3d_headless.exe"; do
        if [ -f "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
}

find_octomap_editor_executable() {
    if [ -n "${LINGTU_OCTOMAP_EDITOR:-}" ]; then
        if [ -f "$LINGTU_OCTOMAP_EDITOR" ]; then
            printf '%s\n' "$LINGTU_OCTOMAP_EDITOR"
        fi
        return 0
    fi

    for candidate in \
        "$NAV_ENDPOINT_BUILD_DIR/octoplanner3d_runtime/octoplanner3d_edit_octomap" \
        "$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_edit_octomap" \
        "$DEV_DIR/build/octoplanner3d_headless/Release/octoplanner3d_edit_octomap" \
        "$DEV_DIR/build/octoplanner3d_headless/Debug/octoplanner3d_edit_octomap" \
        "$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_edit_octomap.exe" \
        "$DEV_DIR/build/octoplanner3d_headless/Release/octoplanner3d_edit_octomap.exe" \
        "$DEV_DIR/build/octoplanner3d_headless/Debug/octoplanner3d_edit_octomap.exe"; do
        if [ -f "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
}

find_octomap_converter_executable() {
    if [ -n "${LINGTU_MAP_ARTIFACT_CONVERTER:-}" ]; then
        if [ -f "$LINGTU_MAP_ARTIFACT_CONVERTER" ]; then
            printf '%s\n' "$LINGTU_MAP_ARTIFACT_CONVERTER"
        fi
        return 0
    fi

    for candidate in \
        "$NAV_ENDPOINT_BUILD_DIR/octoplanner3d_runtime/octoplanner3d_pcd_to_octomap" \
        "$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap"; do
        if [ -f "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
}

require_native_artifacts() {
    ensure_field_native_artifacts

    NAV_KERNEL_ARTIFACT=""
    if [ "$REQUIRE_PYTHON_NAV_KERNEL" = "1" ]; then
        ensure_nav_kernel_artifact
        if [ -z "$NAV_KERNEL_ARTIFACT" ]; then
            echo "ERROR: lingtu_nav_kernel*.so not found after requested Python-kernel build."
            exit 1
        fi
    fi

    OCTO_EXEC_SOURCE="$(find_octoplanner3d_executable)"
    if [ "$REQUIRE_OCTOPLANNER3D" = "1" ] \
        && [ -z "$OCTO_EXEC_SOURCE" ]; then
        echo "ERROR: OctoPlanner3D C++ headless executable not found. Build it first:"
        echo "  cd $DEV_DIR && bash scripts/build/build_octoplanner3d.sh"
        echo "Or select FAR with LINGTU_RELEASE_GLOBAL_PLANNER=far."
        exit 1
    fi

    OCTOMAP_EDITOR_SOURCE="$(find_octomap_editor_executable)"
    if [ "$REQUIRE_OCTOPLANNER3D" = "1" ] \
        && [ -z "$OCTOMAP_EDITOR_SOURCE" ]; then
        echo "ERROR: OctoMap voxel editor not found. Build it first:"
        echo "  cd $DEV_DIR && bash scripts/build/build_octoplanner3d.sh"
        exit 1
    fi
    OCTOMAP_CONVERTER_SOURCE="$(find_octomap_converter_executable)"
    if [ "$REQUIRE_OCTOMAP_CONVERTER" = "1" ] \
        && [ -z "$OCTOMAP_CONVERTER_SOURCE" ]; then
        echo "ERROR: OctoMap PCD converter not found. Build it first:"
        echo "  cd $DEV_DIR && bash scripts/build/build_octoplanner3d.sh --require-pcl"
        echo "Or set LINGTU_RELEASE_REQUIRE_OCTOMAP_CONVERTER=0 for an explicit no-converter release."
        exit 1
    fi
}

copy_octoplanner3d_executable() {
    if [ -z "${OCTO_EXEC_SOURCE:-}" ]; then
        return 0
    fi

    case "$OCTO_EXEC_SOURCE" in
        "$TARGET_DIR"/*)
            return 0
            ;;
    esac

    local dest_dir="$TARGET_DIR/build/octoplanner3d_headless"
    local dest="$dest_dir/$(basename "$OCTO_EXEC_SOURCE")"
    mkdir -p "$dest_dir"
    cp -L "$OCTO_EXEC_SOURCE" "$dest"
    chmod 0755 "$dest"
    echo ">>> Copied OctoPlanner3D headless executable: $dest"
}

copy_octomap_editor_executable() {
    if [ -z "${OCTOMAP_EDITOR_SOURCE:-}" ]; then
        return 0
    fi

    case "$OCTOMAP_EDITOR_SOURCE" in
        "$TARGET_DIR"/*)
            return 0
            ;;
    esac

    local dest_dir="$TARGET_DIR/build/octoplanner3d_headless"
    local dest="$dest_dir/$(basename "$OCTOMAP_EDITOR_SOURCE")"
    mkdir -p "$dest_dir"
    cp -L "$OCTOMAP_EDITOR_SOURCE" "$dest"
    chmod 0755 "$dest"
    echo ">>> Copied OctoMap voxel editor: $dest"
}

copy_octomap_converter_executable() {
    if [ -z "${OCTOMAP_CONVERTER_SOURCE:-}" ]; then
        return 0
    fi

    case "$OCTOMAP_CONVERTER_SOURCE" in
        "$TARGET_DIR"/*)
            return 0
            ;;
    esac

    local dest_dir="$TARGET_DIR/build/octoplanner3d_headless"
    local dest="$dest_dir/$(basename "$OCTOMAP_CONVERTER_SOURCE")"
    mkdir -p "$dest_dir"
    cp -L "$OCTOMAP_CONVERTER_SOURCE" "$dest"
    chmod 0755 "$dest"
    echo ">>> Copied OctoMap PCD converter: $dest"
}

install_nav_endpoint_package() {
    local install_dir="$TARGET_DIR/build/nav_endpoint"
    cmake --install "$NAV_ENDPOINT_BUILD_DIR" --prefix "$install_dir"

    local relative
    for relative in "${NAV_ENDPOINT_RUNTIME_FILES[@]}"; do
        if [ ! -e "$install_dir/$relative" ]; then
            echo "ERROR: native endpoint install manifest omitted: $relative" >&2
            exit 1
        fi
    done
    for relative in \
        navd \
        lingtu_traversability_dds \
        lingtu_explore_dds \
        lingtu_nav_control \
        lingtu_motion_mock_dds; do
        chmod 0755 "$install_dir/$relative"
    done
    echo ">>> Installed tested native endpoint package: $install_dir"
}

install_native_recording_package() {
    local install_dir="$TARGET_DIR/build/native-recording"
    local relative

    cmake --install "$RECORDING_BUILD_DIR" --prefix "$install_dir"
    for relative in "${RECORDING_RUNTIME_FILES[@]}"; do
        if [ ! -x "$install_dir/$relative" ]; then
            echo "ERROR: native recording install manifest omitted: $relative" >&2
            exit 1
        fi
        chmod 0755 "$install_dir/$relative"
    done
    echo ">>> Installed tested native recording package: $install_dir"
}

copy_field_native_runtime_files() {
    local sources=(
        "$LIVOX_RUNTIME_SOURCE"
        "$SLAM_RUNTIME_SOURCE"
        "$SLAM_CONTROL_SOURCE"
        "$MAPD_RUNTIME_SOURCE"
        "$MAPCTL_RUNTIME_SOURCE"
        "$MAPS_LIBRARY_SOURCE"
        "$DDS_PROBE_RUNTIME_SOURCE"
        "$DRIVER_RUNTIME_SOURCE"
    )
    local relatives=(
        build/livox_sdk2_stream/livox_sdk2_stream
        build/slam_core/slamd
        build/slam_core/slamctl
        build/maps/mapd
        build/maps/lingtu-mapctl
        build/maps/liblingtu_maps.so
        build/dds_probe/lingtu_dds_probe
        build/driver/lingtu_driver
    )
    local modes=(
        0755
        0755
        0755
        0755
        0755
        0644
        0755
        0755
    )
    local index
    local destination

    for index in "${!sources[@]}"; do
        destination="$TARGET_DIR/${relatives[$index]}"
        mkdir -p "$(dirname "$destination")"
        cp -L "${sources[$index]}" "$destination"
        chmod "${modes[$index]}" "$destination"
        echo ">>> Copied native runtime: $destination"
    done
}

verify_release_native_runtime_files() {
    local root="${1:-$TARGET_DIR}"
    local relative
    local expected
    local observed

    for relative in "${RELEASE_NATIVE_RUNTIME_FILES[@]}"; do
        if [ ! -x "$root/$relative" ]; then
            echo "ERROR: release native runtime is missing or not executable: $root/$relative" >&2
            return 1
        fi
        expected="$(readlink -f "$TARGET_DIR/$relative")"
        observed="$(readlink -f "$root/$relative")"
        if [ -z "$expected" ] || [ "$observed" != "$expected" ]; then
            echo "ERROR: release runtime does not resolve to the activated target: $root/$relative" >&2
            echo "  expected: ${expected:-missing}" >&2
            echo "  observed: ${observed:-missing}" >&2
            return 1
        fi
    done
    for relative in "${RELEASE_NATIVE_LIBRARY_FILES[@]}"; do
        if [ ! -f "$root/$relative" ]; then
            echo "ERROR: release native library is missing: $root/$relative" >&2
            return 1
        fi
        expected="$(readlink -f "$TARGET_DIR/$relative")"
        observed="$(readlink -f "$root/$relative")"
        if [ -z "$expected" ] || [ "$observed" != "$expected" ]; then
            echo "ERROR: release library does not resolve to the activated target: $root/$relative" >&2
            echo "  expected: ${expected:-missing}" >&2
            echo "  observed: ${observed:-missing}" >&2
            return 1
        fi
    done
}

write_release_native_sha256_manifest() {
    local manifest="$TARGET_DIR/config/release-native-sha256.txt"

    verify_release_native_runtime_files "$TARGET_DIR"
    if ! command -v sha256sum >/dev/null 2>&1; then
        echo "ERROR: sha256sum is required to cut a native field release" >&2
        return 1
    fi
    mkdir -p "$(dirname "$manifest")"
    (
        cd "$TARGET_DIR"
        sha256sum \
            "${RELEASE_NATIVE_RUNTIME_FILES[@]}" \
            "${RELEASE_NATIVE_LIBRARY_FILES[@]}"
    ) >"$manifest"
    chmod 0644 "$manifest"
    echo ">>> Wrote native runtime SHA-256 manifest: $manifest"
}

verify_release_native_sha256_manifest() {
    local root="${1:-$TARGET_DIR}"
    local manifest="$root/config/release-native-sha256.txt"

    if [ ! -s "$manifest" ]; then
        echo "ERROR: release native SHA-256 manifest is missing: $manifest" >&2
        return 1
    fi
    (
        cd "$root"
        sha256sum -c config/release-native-sha256.txt
    )
}

write_release_runtime_contract() {
    local contract="$TARGET_DIR/config/release-runtime.env"
    mkdir -p "$(dirname "$contract")"
    {
        printf '# Generated by scripts/deploy/cut_release.sh for %s\n' "$VERSION"
        printf 'LINGTU_RELEASE_VERSION=%s\n' "$VERSION"
        printf 'LINGTU_NAV_GLOBAL_PLANNER=%s\n' "$SELECTED_GLOBAL_PLANNER"
        printf '# Product identity and native control settings come only from the committed RunPlan.\n'
        printf '# release-native-sha256.txt verifies packaged bytes and is bound into RunPlan compatibility.\n'
    } >"$contract"
    chmod 0644 "$contract"
}

nav_status_matches_release() {
    if [ "$REQUIRE_NAV_STATUS" != "1" ]; then
        return 0
    fi
    python3 - \
        "$NAV_STATUS_FILE" \
        "$SELECTED_GLOBAL_PLANNER" \
        "$CURRENT_PRODUCT" \
        "$CURRENT_CONTROL_MODE" <<'PY'
import json
import os
import sys
import time

path, expected_planner, expected_product, expected_mode = sys.argv[1:5]
try:
    stat = os.stat(path)
    if time.time() - stat.st_mtime > 15.0:
        raise RuntimeError("status snapshot is stale")
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    schema = str(payload.get("schema_version") or "").strip()
    endpoint = str(payload.get("endpoint") or "").strip()
    control_mode = str(payload.get("control_mode") or "").strip().lower()
    native_product = payload.get("native_product") or {}
    reported_product = str(native_product.get("product") or "").strip().lower()
    if schema != "lingtu.nav.endpoint.status.v1":
        raise RuntimeError(f"status schema mismatch: {schema or 'missing'}")
    if endpoint != "navd":
        raise RuntimeError(f"endpoint mismatch: {endpoint or 'missing'}")
    if expected_mode and control_mode != expected_mode:
        raise RuntimeError(
            f"control_mode mismatch: expected={expected_mode} observed={control_mode or 'missing'}"
        )
    if expected_product and reported_product != expected_product:
        raise RuntimeError(
            f"Product mismatch: expected={expected_product} observed={reported_product or 'missing'}"
        )

    planner = str(payload.get("global_planner") or "").strip().lower()
    if planner != expected_planner:
        raise RuntimeError(
            f"planner mismatch: expected={expected_planner} observed={planner or 'missing'}"
        )

    if control_mode in {"teleop", "teleop_avoid"}:
        operator_motion = payload.get("operator_motion") or {}
        if operator_motion.get("interface_enabled") is not True:
            raise RuntimeError("typed operator-motion interface is not enabled")
        if payload.get("publish_cmd_vel") is not True:
            raise RuntimeError("teleop output publication is disabled")
        if control_mode == "teleop_avoid":
            required = {
                "check_obstacle": True,
                "use_traversability_cost": True,
                "teleop_local_planner": True,
            }
            mismatched = [key for key, value in required.items() if payload.get(key) is not value]
            if mismatched:
                raise RuntimeError(
                    "teleop_avoid safety configuration mismatch: " + ",".join(mismatched)
                )
    elif control_mode == "autonomy":
        planner_map = str(payload.get("planner_map") or "").strip()
        if not planner_map:
            raise RuntimeError("planner_map is missing for autonomy")
        basename = os.path.basename(planner_map)
        if expected_planner == "far" and basename != "occupancy.npz":
            raise RuntimeError(f"FAR requires occupancy.npz, observed={planner_map}")
        if expected_planner == "octoplanner3d" and not basename.endswith((".ot", ".bt")):
            raise RuntimeError(
                f"OctoPlanner3D requires an OctoMap artifact, observed={planner_map}"
            )
    else:
        raise RuntimeError(f"unsupported control_mode: {control_mode or 'missing'}")
except Exception as exc:
    print(f"native nav status rejected: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

unit_exists() {
    local unit="$1"
    systemctl list-unit-files "$unit" --no-legend 2>/dev/null | grep -q "$unit"
}

require_persistent_driver_service() {
    if ! unit_exists "$CURRENT_DRIVER_UNIT"; then
        echo "ERROR: RunPlan driver unit is not installed: $CURRENT_DRIVER_UNIT" >&2
        exit 1
    fi
    if ! systemctl is-enabled --quiet "$CURRENT_DRIVER_UNIT"; then
        echo "ERROR: RunPlan driver unit must be enabled before cutting a field release: $CURRENT_DRIVER_UNIT" >&2
        exit 1
    fi
    if [ ! -s "$LINGTU_CONFIG_DIR/brainstem.env" ]; then
        echo "ERROR: persistent driver configuration is missing: $LINGTU_CONFIG_DIR/brainstem.env" >&2
        exit 1
    fi
}

product_control_adapter() {
    if [ -f "$CURRENT_LINK/scripts/lingtu" ]; then
        printf '%s\n' "$CURRENT_LINK/scripts/lingtu"
        return 0
    fi
    if [ -f "$DEV_DIR/scripts/lingtu" ]; then
        printf '%s\n' "$DEV_DIR/scripts/lingtu"
        return 0
    fi
    echo "ERROR: cannot find scripts/lingtu adapter for ProductControl reapply" >&2
    return 1
}

reapply_committed_run_plan() {
    local phase="$1"
    local adapter

    if ! adapter="$(product_control_adapter)"; then
        return 1
    fi
    echo ">>> $phase: reapplying committed Product RunPlan via ProductControl..."
    LINGTU_ENV="$RELEASE_ENV" GW="$GATEWAY_URL" \
        bash "$adapter" --env "$RELEASE_ENV" svc reapply
}

verify_service_uses_current_release() {
    local service="$1"
    local relative="$2"
    local label="$3"
    local expected
    local control_group
    local cgroup_procs
    local pid
    local observed
    local matched=0

    verify_release_native_runtime_files "$CURRENT_LINK"
    expected="$(readlink -f "$CURRENT_LINK/$relative")"
    if [ -z "$expected" ]; then
        echo "ERROR: activated $label binary cannot be resolved through $CURRENT_LINK" >&2
        return 1
    fi
    if ! systemctl is-active --quiet "$service"; then
        echo "ERROR: persistent $service is not active after release activation" >&2
        return 1
    fi

    control_group="$(systemctl show "$service" --property=ControlGroup --value)"
    case "$control_group" in
        /*) ;;
        *)
            echo "ERROR: $service has no usable systemd control group" >&2
            return 1
            ;;
    esac
    cgroup_procs="/sys/fs/cgroup${control_group}/cgroup.procs"
    if [ ! -r "$cgroup_procs" ]; then
        echo "ERROR: cannot inspect $service processes: $cgroup_procs" >&2
        return 1
    fi

    while IFS= read -r pid; do
        if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
            continue
        fi
        if observed="$(readlink -f "/proc/$pid/exe" 2>/dev/null)" \
            && [ "$observed" = "$expected" ]; then
            matched=1
            break
        fi
    done <"$cgroup_procs"

    if [ "$matched" != "1" ]; then
        echo "ERROR: $service is active but no process executes the activated $label: $expected" >&2
        return 1
    fi
    echo ">>> Confirmed $service executes: $expected"
}

verify_driver_uses_current_release() {
    verify_service_uses_current_release \
        "$CURRENT_DRIVER_UNIT" \
        "build/driver/lingtu_driver" \
        "driver"
}

verify_mapd_uses_current_release() {
    if [ -z "$CURRENT_MAPD_UNIT" ]; then
        echo ">>> Current RunPlan does not select maps/mapd; runtime binary check skipped."
        return 0
    fi
    verify_service_uses_current_release \
        "$CURRENT_MAPD_UNIT" \
        "build/maps/mapd" \
        "mapd"
}

validate_current_link() {
    if [ -e "$CURRENT_LINK" ] && [ ! -L "$CURRENT_LINK" ]; then
        echo "ERROR: refusing to activate release: $CURRENT_LINK exists but is not a symlink" >&2
        exit 1
    fi
}

validate_target_dir_for_root_ownership() {
    local releases_real
    local target_parent_real
    local target_name

    if [ ! -d "$TARGET_DIR" ]; then
        echo "ERROR: release target is missing after assembly: $TARGET_DIR" >&2
        exit 1
    fi
    if [ -L "$TARGET_DIR" ]; then
        echo "ERROR: refusing to chown symlink release target: $TARGET_DIR" >&2
        exit 1
    fi

    releases_real="$(readlink -f "$RELEASES_DIR")"
    target_parent_real="$(readlink -f "$(dirname "$TARGET_DIR")")"
    target_name="$(basename "$TARGET_DIR")"
    if [ "$target_parent_real" != "$releases_real" ] || [ "$target_name" != "$VERSION" ]; then
        echo "ERROR: release target is not exactly the requested version child of RELEASES_DIR" >&2
        echo "  RELEASES_DIR: $RELEASES_DIR ($releases_real)" >&2
        echo "  TARGET_DIR:   $TARGET_DIR" >&2
        exit 1
    fi
}

harden_release_ownership() {
    local runtime_user="${LINGTU_RELEASE_RUNTIME_USER:-sunrise}"
    local runtime_group="${LINGTU_RELEASE_RUNTIME_GROUP:-$runtime_user}"

    validate_target_dir_for_root_ownership
    if ! id -u "$runtime_user" >/dev/null 2>&1; then
        echo "ERROR: release runtime user does not exist: $runtime_user" >&2
        exit 1
    fi
    if ! getent group "$runtime_group" >/dev/null 2>&1; then
        echo "ERROR: release runtime group does not exist: $runtime_group" >&2
        exit 1
    fi

    sudo chown -R root:root -- "$TARGET_DIR"
    # Runtime state and logs stay writable by the service user; release code remains root-owned.
    sudo install -d -o "$runtime_user" -g "$runtime_group" -m 0755 -- \
        "$TARGET_DIR/.lingtu" \
        "$TARGET_DIR/logs"
}

# Only these exact activation files are synchronized; /etc/lingtu/*.env and
# *.service.d overrides are intentionally untouched and preserved by systemd layering.
activation_destinations() {
    printf '%s\n' \
        "$LINGTU_CONFIG_DIR/thunder-runtime-env.sh" \
        "$LINGTU_SYSTEMD_DIR/lingtu-livox-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-slam-dds.service" \
        "$LINGTU_SYSTEMD_DIR/mapd.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-traversability-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-nav-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-explore-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-driver.service"
}

release_source_for_destination() {
    local dest="$1"
    case "$(basename "$dest")" in
        thunder-runtime-env.sh)
            printf '%s\n' "$TARGET_DIR/scripts/deploy/thunder/runtime-env.sh"
            ;;
        lingtu-livox-dds.service|lingtu-slam-dds.service|mapd.service|lingtu-traversability-dds.service|lingtu-nav-dds.service|lingtu-explore-dds.service|lingtu-driver.service)
            printf '%s\n' "$TARGET_DIR/scripts/deploy/thunder/$(basename "$dest")"
            ;;
        *)
            echo "ERROR: no release source mapping for destination: $dest" >&2
            exit 1
            ;;
    esac
}

backup_label_for_path() {
    local value="$1"
    value="${value#/}"
    value="${value//\//__}"
    printf '%s\n' "$value"
}

preflight_activation_destinations() {
    local destinations=()
    local dest
    local src
    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        src="$(release_source_for_destination "$dest")"
        if [ ! -f "$src" ]; then
            echo "ERROR: release activation source is missing: $src" >&2
            exit 1
        fi
        if [ -d "$dest" ]; then
            echo "ERROR: refusing to replace directory destination: $dest" >&2
            exit 1
        fi
    done
}

backup_activation_state() {
    BACKUP_DIR="$(mktemp -d /tmp/lingtu-release-rollback.XXXXXX)"
    printf '%s\n' "$PREV_TARGET" >"$BACKUP_DIR/current.target"

    local destinations=()
    local dest
    local label
    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        label="$(backup_label_for_path "$dest")"
        if [ -e "$dest" ] || [ -L "$dest" ]; then
            sudo cp -a -- "$dest" "$BACKUP_DIR/$label"
        else
            : >"$BACKUP_DIR/$label.absent"
        fi
    done
}

install_release_file() {
    local src="$1"
    local dest="$2"
    local dest_dir
    local tmp

    if [ ! -f "$src" ]; then
        echo "ERROR: release activation source is missing: $src" >&2
        exit 1
    fi

    dest_dir="$(dirname "$dest")"
    tmp="$dest.tmp.$$"
    sudo mkdir -p -- "$dest_dir"
    sudo install -o root -g root -m 0644 -- "$src" "$tmp"
    sudo mv -f -- "$tmp" "$dest"
}

install_activation_files() {
    local destinations=()
    local dest
    local src
    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        src="$(release_source_for_destination "$dest")"
        install_release_file "$src" "$dest"
    done
}

verify_activation_files() {
    local destinations=()
    local dest
    local src
    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        src="$(release_source_for_destination "$dest")"
        if ! sudo cmp -s -- "$src" "$dest"; then
            echo "ERROR: installed file differs from release source: $dest" >&2
            exit 1
        fi
    done

    sudo systemd-analyze verify \
        "$LINGTU_SYSTEMD_DIR/lingtu-livox-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-slam-dds.service" \
        "$LINGTU_SYSTEMD_DIR/mapd.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-traversability-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-nav-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-explore-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-driver.service"
}

cleanup_activation_backup() {
    if [ -z "${BACKUP_DIR:-}" ] || [ ! -d "$BACKUP_DIR" ]; then
        return 0
    fi

    local destinations=()
    local dest
    local label
    local tmp
    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        label="$(backup_label_for_path "$dest")"
        tmp="$dest.tmp.$$"
        sudo rm -f -- "$BACKUP_DIR/$label" "$BACKUP_DIR/$label.absent" "$tmp"
    done
    sudo rm -f -- "$BACKUP_DIR/current.target"
    sudo rmdir -- "$BACKUP_DIR" 2>/dev/null || true
}

restore_activation_state() {
    local destinations=()
    local dest
    local label
    local previous_target="none"

    if [ -n "${BACKUP_DIR:-}" ] && [ -f "$BACKUP_DIR/current.target" ]; then
        previous_target="$(cat "$BACKUP_DIR/current.target")"
    fi

    mapfile -t destinations < <(activation_destinations)
    for dest in "${destinations[@]}"; do
        label="$(backup_label_for_path "$dest")"
        if [ -n "${BACKUP_DIR:-}" ] && [ -f "$BACKUP_DIR/$label.absent" ]; then
            sudo rm -f -- "$dest"
        elif [ -n "${BACKUP_DIR:-}" ] && { [ -e "$BACKUP_DIR/$label" ] || [ -L "$BACKUP_DIR/$label" ]; }; then
            sudo rm -f -- "$dest"
            sudo cp -a -- "$BACKUP_DIR/$label" "$dest"
        fi
    done

    if [ "$previous_target" = "none" ]; then
        sudo rm -f -- "$CURRENT_LINK"
    else
        sudo ln -sfn -- "$previous_target" "$CURRENT_LINK"
    fi

    sudo systemctl daemon-reload
    if ! reapply_committed_run_plan "rollback"; then
        echo "ERROR: rollback ProductControl reapply failed; leaving activation restored but not marked live" >&2
        return 1
    fi
}

release_rollback_trap() {
    local status="$1"
    if [ "${ACTIVATION_STARTED:-0}" != "1" ] \
        || [ "${ACTIVATION_COMMITTED:-0}" = "1" ]; then
        return 0
    fi
    if [ "$status" -eq 0 ]; then
        status=1
    fi

    trap - EXIT
    echo "ERROR: release activation failed; restoring previous activation state" >&2
    set +e
    restore_activation_state
    cleanup_activation_backup
    exit "$status"
}

validate_current_link

if [ -e "$TARGET_DIR" ]; then
    echo "ERROR: $TARGET_DIR already exists. Pick a different version."
    exit 1
fi

require_persistent_driver_service
require_native_artifacts

echo "========================================"
echo "  Cutting LingTu Thunder release: $VERSION"
echo "  From: $DEV_DIR"
echo "  To:   $TARGET_DIR"
echo "  Native navigation endpoint: $NAV_ENDPOINT_BUILD_DIR/navd"
echo "  Native maps runtime: $MAPD_RUNTIME_SOURCE"
echo "  Native map control: $MAPCTL_RUNTIME_SOURCE"
echo "  Native DDS readiness probe: $DDS_PROBE_RUNTIME_SOURCE"
echo "  Global planner: $SELECTED_GLOBAL_PLANNER"
if [ -n "${NAV_KERNEL_ARTIFACT:-}" ]; then
    echo "  Optional Python navigation kernel: $NAV_KERNEL_ARTIFACT"
else
    echo "  Optional Python navigation kernel: not requested"
fi
if [ -n "${OCTO_EXEC_SOURCE:-}" ]; then
    echo "  OctoPlanner3D executable: $OCTO_EXEC_SOURCE"
else
    echo "  OctoPlanner3D gate: skipped"
fi
if [ -n "${OCTOMAP_EDITOR_SOURCE:-}" ]; then
    echo "  OctoMap editor: $OCTOMAP_EDITOR_SOURCE"
fi
if [ -n "${OCTOMAP_CONVERTER_SOURCE:-}" ]; then
    echo "  OctoMap PCD converter: $OCTOMAP_CONVERTER_SOURCE"
fi
echo "========================================"

echo ">>> [1/5] Building $TARGET_DIR via rsync..."
sudo mkdir -p "$TARGET_DIR"
sudo chown "$USER:$USER" "$TARGET_DIR"
rsync -aL \
    --exclude=.git \
    --exclude=build \
    --exclude=log \
    --exclude=logs \
    --exclude=.lingtu \
    --exclude=.omc \
    --exclude=.pytest_cache \
    --exclude=__pycache__ \
    --exclude='*.pyc' \
    --exclude=research \
    "$DEV_DIR/" "$TARGET_DIR/"
install_nav_endpoint_package
install_native_recording_package
copy_field_native_runtime_files
copy_octoplanner3d_executable
copy_octomap_editor_executable
copy_octomap_converter_executable
write_release_runtime_contract
verify_release_native_runtime_files "$TARGET_DIR"
write_release_native_sha256_manifest
verify_release_native_sha256_manifest "$TARGET_DIR"
harden_release_ownership

REL_SIZE="$(du -sh "$TARGET_DIR" | awk '{print $1}')"
echo ">>> [1/5] Done; release size: $REL_SIZE"

PREV_TARGET="$(readlink "$CURRENT_LINK" 2>/dev/null || echo none)"
BACKUP_DIR=""
ACTIVATION_STARTED=0
ACTIVATION_COMMITTED=0
trap 'release_rollback_trap "$?"' EXIT

echo ">>> [2/5] Previous current: $PREV_TARGET"
echo ">>> [2/5] Committed Product: $CURRENT_PRODUCT ($CURRENT_RUN_PLAN_FINGERPRINT)"

echo ">>> [3/5] Installing release activation files and swapping $CURRENT_LINK -> $TARGET_DIR ..."
preflight_activation_destinations
backup_activation_state
ACTIVATION_STARTED=1
install_activation_files
sudo ln -sfn -- "$TARGET_DIR" "$CURRENT_LINK"
verify_release_native_runtime_files "$CURRENT_LINK"
verify_release_native_sha256_manifest "$CURRENT_LINK"
sudo systemctl daemon-reload
verify_activation_files
echo ">>> [3/5] Done."

echo ">>> [4/5] Reapplying the committed Product RunPlan..."
if [ "$REQUIRE_NAV_STATUS" = "1" ]; then
    sudo rm -f -- "$NAV_STATUS_FILE"
fi
reapply_committed_run_plan "activation"
verify_driver_uses_current_release
verify_mapd_uses_current_release
echo ">>> [4/5] Done."

echo ">>> [5/5] Committing activation after ProductControl reapply and release artifact checks..."
echo ""
echo "Release $VERSION is live."
echo "Host runtime env and DDS unit base files were updated from the release; /etc/lingtu env and drop-in overrides were left untouched."
echo "Manual rollback note: symlink-only rollback is incomplete because host runtime env and base DDS units remain from this activation."
echo "Re-activate the previous release host files, run sudo systemctl daemon-reload, then reapply the committed RunPlan through ProductControl."
echo "Symlink step for manual rollback:"
if [ "$PREV_TARGET" = "none" ]; then
    echo "  sudo rm -f -- $CURRENT_LINK"
else
    echo "  sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
fi
echo "  bash $CURRENT_LINK/scripts/lingtu --env $RELEASE_ENV svc reapply"
echo "Optional one-shot post-release gate:"
echo "  bash $CURRENT_LINK/scripts/lingtu --env $RELEASE_ENV field-check <map-directory> --acceptance-mode field"
ACTIVATION_COMMITTED=1
echo ">>> [5/5] Done."
trap - EXIT
cleanup_activation_backup
exit 0
