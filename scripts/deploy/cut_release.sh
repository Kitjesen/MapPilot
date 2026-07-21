#!/bin/bash
# Cut a new LingTu Thunder release on the field robot.
#
# Snapshot the current development checkout into /opt/lingtu/releases/<version>/,
# atomically swap /opt/lingtu/current, and restart release-mode services.
#
# Default release gates are native-first:
#   - The tested native navigation endpoint package must be built and installed.
#   - The selected global planner is immutable release metadata.
#   - OctoPlanner3D tools are required only for an OctoPlanner3D release.
#   - The nanobind Python kernel is optional and never substitutes for navd.
#   - ROS 2 compatibility install packages are checked only when
#     LINGTU_RELEASE_REQUIRE_ROS2_COMPAT=1.
#
# Usage on the robot:
#   bash scripts/deploy/cut_release.sh v2.1.1
#
# Rollback:
#   sudo ln -sfn /opt/lingtu/releases/v2.1.0 /opt/lingtu/current
#   sudo systemctl restart lingtu-thunder-dds-endpoint

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
RELEASES_DIR="${LINGTU_RELEASES_DIR:-/opt/lingtu/releases}"
TARGET_DIR="$RELEASES_DIR/$VERSION"
CURRENT_LINK="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"
LINGTU_CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
LINGTU_SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
GATEWAY_URL="${LINGTU_GATEWAY_URL:-http://localhost:5050}"
NAV_ENDPOINT_BUILD_DIR="${LINGTU_NAV_ENDPOINT_BUILD_DIR:-$DEV_DIR/build/nav_endpoint}"
NAV_STATUS_FILE="${LINGTU_NAV_STATUS_FILE:-/dev/shm/lingtu/nav_endpoint_status.json}"
SELECTED_GLOBAL_PLANNER="${LINGTU_RELEASE_GLOBAL_PLANNER:-${LINGTU_NAV_GLOBAL_PLANNER:-octoplanner3d}}"
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
REQUIRE_ROS2_COMPAT="${LINGTU_RELEASE_REQUIRE_ROS2_COMPAT:-0}"
RESTART_ROS2_COMPAT="${LINGTU_RELEASE_RESTART_ROS2_COMPAT:-$REQUIRE_ROS2_COMPAT}"

for release_flag in \
    REQUIRE_OCTOPLANNER3D \
    REQUIRE_OCTOMAP_CONVERTER \
    REQUIRE_PYTHON_NAV_KERNEL \
    REQUIRE_NAV_STATUS \
    REQUIRE_ROS2_COMPAT \
    RESTART_ROS2_COMPAT; do
    value="${!release_flag}"
    if [ "$value" != "0" ] && [ "$value" != "1" ]; then
        echo "ERROR: $release_flag must be 0 or 1 (got: $value)" >&2
        exit 1
    fi
done

ROS2_COMPAT_PKGS="fastlio2 genz_icp hba interface livox_ros_driver2 localizer nav_services pgo pointlio perception decision wtrtk980_reader"

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
    if ! nav_endpoint_artifacts_ready; then
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
    ensure_nav_endpoint_artifacts

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

require_ros2_compat_install() {
    if [ "$REQUIRE_ROS2_COMPAT" != "1" ]; then
        echo ">>> ROS 2 compatibility package gate skipped (set LINGTU_RELEASE_REQUIRE_ROS2_COMPAT=1 to enable)."
        return 0
    fi

    if [ ! -d "$DEV_DIR/install" ]; then
        echo "ERROR: $DEV_DIR/install not found, but ROS 2 compatibility was requested."
        echo "Build compatibility packages first: cd $DEV_DIR && bash scripts/build/build_ros_workspace.sh"
        exit 1
    fi

    for pkg in $ROS2_COMPAT_PKGS; do
        if [ ! -d "$DEV_DIR/install/$pkg" ]; then
            echo "ERROR: install/$pkg missing in dev checkout; refusing to cut ROS 2 compatibility release."
            exit 1
        fi
    done
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

write_release_runtime_contract() {
    local contract="$TARGET_DIR/config/release-runtime.env"
    mkdir -p "$(dirname "$contract")"
    {
        printf '# Generated by scripts/deploy/cut_release.sh for %s\n' "$VERSION"
        printf 'LINGTU_RELEASE_VERSION=%s\n' "$VERSION"
        printf 'LINGTU_NAV_GLOBAL_PLANNER=%s\n' "$SELECTED_GLOBAL_PLANNER"
    } >"$contract"
    chmod 0644 "$contract"
}

nav_status_matches_release() {
    if [ "$REQUIRE_NAV_STATUS" != "1" ]; then
        return 0
    fi
    python3 - "$NAV_STATUS_FILE" "$SELECTED_GLOBAL_PLANNER" <<'PY'
import json
import os
import sys
import time

path, expected = sys.argv[1:3]
try:
    stat = os.stat(path)
    if time.time() - stat.st_mtime > 15.0:
        raise RuntimeError("status snapshot is stale")
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    planner = str(payload.get("global_planner") or "").strip().lower()
    planner_map = str(payload.get("planner_map") or "").strip()
    if planner != expected:
        raise RuntimeError(f"planner mismatch: expected={expected} observed={planner or 'missing'}")
    if not planner_map:
        raise RuntimeError("planner_map is missing")
    basename = os.path.basename(planner_map)
    if expected == "far" and basename != "occupancy.npz":
        raise RuntimeError(f"FAR requires occupancy.npz, observed={planner_map}")
    if expected == "octoplanner3d" and not basename.endswith((".ot", ".bt")):
        raise RuntimeError(f"OctoPlanner3D requires an OctoMap artifact, observed={planner_map}")
except Exception as exc:
    print(f"native nav status rejected: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

unit_exists() {
    local unit="$1"
    systemctl list-unit-files "$unit" --no-legend 2>/dev/null | grep -q "$unit"
}

unit_active_or_enabled() {
    local unit="$1"
    systemctl is-active --quiet "$unit" 2>/dev/null \
        || systemctl is-enabled --quiet "$unit" 2>/dev/null
}

emit_release_services() {
    local primary="$1"
    printf '%s\n' "$primary"
}

resolve_release_services() {
    if [ -n "${LINGTU_RELEASE_SERVICES:-}" ]; then
        printf '%s\n' "$LINGTU_RELEASE_SERVICES"
        return 0
    fi

    local services=()
    local candidate
    for candidate in \
        lingtu-livox-dds.service \
        lingtu-slam-dds.service \
        lingtu-traversability-dds.service \
        lingtu-nav-dds.service; do
        if unit_exists "$candidate"; then
            services+=("$candidate")
        fi
    done

    if unit_exists "lingtu-explore-dds.service" \
        && unit_active_or_enabled "lingtu-explore-dds.service"; then
        services+=("lingtu-explore-dds.service")
    fi

    for candidate in \
        lingtu-thunder-dds-endpoint.service \
        lingtu-thunder-lite.service \
        lingtu.service; do
        if unit_exists "$candidate"; then
            services+=("$candidate")
            break
        fi
    done

    if [ "${#services[@]}" -eq 0 ]; then
        emit_release_services "lingtu-thunder-dds-endpoint.service"
        return 0
    fi

    printf '%s\n' "${services[*]}"
}

restart_service_list() {
    local services="$1"
    local svc
    for svc in $services; do
        echo "  restart: $svc"
        sudo systemctl restart "$svc"
        sleep 2
    done
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
        "$LINGTU_SYSTEMD_DIR/lingtu-traversability-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-nav-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-explore-dds.service"
}

release_source_for_destination() {
    local dest="$1"
    case "$(basename "$dest")" in
        thunder-runtime-env.sh)
            printf '%s\n' "$TARGET_DIR/scripts/deploy/thunder/runtime-env.sh"
            ;;
        lingtu-traversability-dds.service|lingtu-nav-dds.service|lingtu-explore-dds.service)
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
        "$LINGTU_SYSTEMD_DIR/lingtu-traversability-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-nav-dds.service" \
        "$LINGTU_SYSTEMD_DIR/lingtu-explore-dds.service"
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
    if [ -n "${ROLLBACK_SERVICES:-}" ]; then
        restart_service_list "$ROLLBACK_SERVICES"
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

require_native_artifacts
require_ros2_compat_install

echo "========================================"
echo "  Cutting LingTu Thunder release: $VERSION"
echo "  From: $DEV_DIR"
echo "  To:   $TARGET_DIR"
echo "  Native navigation endpoint: $NAV_ENDPOINT_BUILD_DIR/navd"
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
copy_octoplanner3d_executable
copy_octomap_editor_executable
copy_octomap_converter_executable
write_release_runtime_contract
harden_release_ownership

REL_SIZE="$(du -sh "$TARGET_DIR" | awk '{print $1}')"
echo ">>> [1/5] Done; release size: $REL_SIZE"

PREV_TARGET="$(readlink "$CURRENT_LINK" 2>/dev/null || echo none)"
ROLLBACK_SERVICES="$(resolve_release_services)"
if [ "$RESTART_ROS2_COMPAT" = "1" ]; then
    ROLLBACK_SERVICES="robot-fastlio2.service robot-localizer.service $ROLLBACK_SERVICES"
fi
BACKUP_DIR=""
ACTIVATION_STARTED=0
ACTIVATION_COMMITTED=0
trap 'release_rollback_trap "$?"' EXIT

echo ">>> [2/5] Previous current: $PREV_TARGET"
echo ">>> [2/5] Rollback services: $ROLLBACK_SERVICES"

echo ">>> [3/5] Installing release activation files and swapping $CURRENT_LINK -> $TARGET_DIR ..."
preflight_activation_destinations
backup_activation_state
ACTIVATION_STARTED=1
install_activation_files
sudo ln -sfn -- "$TARGET_DIR" "$CURRENT_LINK"
sudo systemctl daemon-reload
verify_activation_files
RELEASE_SERVICES="$(resolve_release_services)"
echo ">>> [3/5] Release services: $RELEASE_SERVICES"
echo ">>> [3/5] Done."

echo ">>> [4/5] Restarting services..."
if [ "$REQUIRE_NAV_STATUS" = "1" ]; then
    sudo rm -f -- "$NAV_STATUS_FILE"
fi
if [ "$RESTART_ROS2_COMPAT" = "1" ]; then
    restart_service_list "robot-fastlio2.service robot-localizer.service"
fi
restart_service_list "$RELEASE_SERVICES"
echo ">>> [4/5] Done."

echo ">>> [5/5] Waiting for /api/v1/health to report a clean module set..."
DEADLINE=$((SECONDS + 60))
while [ $SECONDS -lt $DEADLINE ]; do
    HEALTH="$(curl -sf --max-time 2 "$GATEWAY_URL/api/v1/health" 2>/dev/null || true)"
    if echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); fails=int(d.get("modules_fail", 0) or 0); ok=int(d.get("modules_ok", 0) or 0); status=str(d.get("status", d.get("ready", ""))).lower(); sys.exit(0 if fails == 0 and (ok > 0 or status in {"ok", "healthy", "ready", "true"}) else 1)' 2>/dev/null \
        && nav_status_matches_release; then
        OK="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("modules_ok", "?"))')"
        FSM="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print((d.get("brainstem") or {}).get("fsm", d.get("status", "?")))')"
        echo ">>> [5/5] Healthy: modules=$OK/0, FSM=$FSM"
        echo ""
        echo "Release $VERSION is live."
        echo "Host runtime env and DDS unit base files were updated from the release; /etc/lingtu env and drop-in overrides were left untouched."
        echo "Manual rollback note: symlink-only rollback is incomplete because host runtime env and base DDS units remain from this activation."
        echo "Re-activate the previous release host files, run sudo systemctl daemon-reload, then restart services."
        echo "Symlink step for manual rollback:"
        if [ "$PREV_TARGET" = "none" ]; then
            echo "  sudo rm -f -- $CURRENT_LINK"
        else
            echo "  sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
        fi
        echo "  sudo systemctl restart $RELEASE_SERVICES"
        ACTIVATION_COMMITTED=1
        trap - EXIT
        cleanup_activation_backup
        exit 0
    fi
    sleep 3
done

echo ">>> [5/5] FAIL: health endpoint did not report a clean module set within 60s."
if [ "$REQUIRE_NAV_STATUS" = "1" ]; then
    echo "    Native nav status must be fresh and match planner=$SELECTED_GLOBAL_PLANNER: $NAV_STATUS_FILE"
fi
echo "    Inspect: journalctl -u $RELEASE_SERVICES -n 100"
echo "    Rollback is running automatically via the EXIT trap."
exit 1
