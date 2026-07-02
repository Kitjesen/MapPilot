#!/bin/bash
# Cut a new LingTu Thunder release on the field robot.
#
# Snapshot the current development checkout into /opt/lingtu/releases/<version>/,
# atomically swap /opt/lingtu/current, and restart release-mode services.
#
# Default release gates are native-first:
#   - LingTu native navigation kernel must be built.
#   - OctoPlanner3D C++ headless executable must be available.
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
GATEWAY_URL="${LINGTU_GATEWAY_URL:-http://localhost:5050}"
REQUIRE_OCTOPLANNER3D="${LINGTU_RELEASE_REQUIRE_OCTOPLANNER3D:-1}"
REQUIRE_ROS2_COMPAT="${LINGTU_RELEASE_REQUIRE_ROS2_COMPAT:-0}"
RESTART_ROS2_COMPAT="${LINGTU_RELEASE_RESTART_ROS2_COMPAT:-$REQUIRE_ROS2_COMPAT}"

ROS2_COMPAT_PKGS="fastlio2 genz_icp hba interface livox_ros_driver2 localizer nav_services pgo pointlio perception decision wtrtk980_ros2_reader"

find_nav_kernel_artifact() {
    find -L "$DEV_DIR/src" "$DEV_DIR/src/nav/kernel/build_nb" \
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

    local candidate="$DEV_DIR/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap"
    if [ -f "$candidate" ]; then
        printf '%s\n' "$candidate"
    fi
}

require_native_artifacts() {
    ensure_nav_kernel_artifact
    if [ -z "$NAV_KERNEL_ARTIFACT" ]; then
        echo "ERROR: lingtu_nav_kernel*.so not found after production build."
        exit 1
    fi

    OCTO_EXEC_SOURCE="$(find_octoplanner3d_executable)"
    if [ "$REQUIRE_OCTOPLANNER3D" = "1" ] \
        && [ -z "$OCTO_EXEC_SOURCE" ]; then
        echo "ERROR: OctoPlanner3D C++ headless executable not found. Build it first:"
        echo "  cd $DEV_DIR && bash scripts/build/build_octoplanner3d.sh"
        echo "Or set LINGTU_RELEASE_REQUIRE_OCTOPLANNER3D=0 for an explicit non-OctoPlanner release."
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

unit_exists() {
    local unit="$1"
    systemctl list-unit-files "$unit" --no-legend 2>/dev/null | grep -q "$unit"
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

    for candidate in \
        lingtu-thunder-dds-endpoint.service \
        lingtu-thunder-lite.service \
        lingtu.service; do
        if unit_exists "$candidate"; then
            emit_release_services "$candidate"
            return 0
        fi
    done

    emit_release_services "lingtu-thunder-dds-endpoint.service"
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
echo "  Native local autonomy: $NAV_KERNEL_ARTIFACT"
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
copy_octoplanner3d_executable
copy_octomap_editor_executable
copy_octomap_converter_executable

REL_SIZE="$(du -sh "$TARGET_DIR" | awk '{print $1}')"
echo ">>> [1/5] Done; release size: $REL_SIZE"

PREV_TARGET="$(readlink "$CURRENT_LINK" 2>/dev/null || echo none)"
echo ">>> [2/5] Previous current: $PREV_TARGET"

echo ">>> [3/5] Swapping $CURRENT_LINK -> $TARGET_DIR ..."
sudo ln -sfn "$TARGET_DIR" "$CURRENT_LINK"
echo ">>> [3/5] Done."

echo ">>> [4/5] Restarting services..."
if [ "$RESTART_ROS2_COMPAT" = "1" ]; then
    restart_service_list "robot-fastlio2.service robot-localizer.service"
fi
RELEASE_SERVICES="$(resolve_release_services)"
restart_service_list "$RELEASE_SERVICES"
echo ">>> [4/5] Done."

echo ">>> [5/5] Waiting for /api/v1/health to report a clean module set..."
DEADLINE=$((SECONDS + 60))
while [ $SECONDS -lt $DEADLINE ]; do
    HEALTH="$(curl -sf --max-time 2 "$GATEWAY_URL/api/v1/health" 2>/dev/null || true)"
    if echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); fails=int(d.get("modules_fail", 0) or 0); ok=int(d.get("modules_ok", 0) or 0); status=str(d.get("status", d.get("ready", ""))).lower(); sys.exit(0 if fails == 0 and (ok > 0 or status in {"ok", "healthy", "ready", "true"}) else 1)' 2>/dev/null; then
        OK="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("modules_ok", "?"))')"
        FSM="$(echo "$HEALTH" | python3 -c 'import json,sys; d=json.load(sys.stdin); print((d.get("brainstem") or {}).get("fsm", d.get("status", "?")))')"
        echo ">>> [5/5] Healthy: modules=$OK/0, FSM=$FSM"
        echo ""
        echo "Release $VERSION is live."
        echo "Rollback if needed:"
        echo "  sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
        echo "  sudo systemctl restart $RELEASE_SERVICES"
        exit 0
    fi
    sleep 3
done

echo ">>> [5/5] FAIL: health endpoint did not report a clean module set within 60s."
echo "    Inspect: journalctl -u $RELEASE_SERVICES -n 100"
echo "    Or roll back: sudo ln -sfn $PREV_TARGET $CURRENT_LINK"
exit 1
