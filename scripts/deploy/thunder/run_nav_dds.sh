#!/usr/bin/env bash
set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh

prepend_cyclonedds_libs() {
    local prefix="${LINGTU_CYCLONEDDS_PREFIX:-}"
    local multiarch="${LINGTU_CYCLONEDDS_MULTIARCH:-${DEB_HOST_MULTIARCH:-}}"
    local machine
    local paths=()
    local joined=""
    local path

    if [ -z "${prefix}" ] || [ ! -d "${prefix}/lib" ]; then
        return 0
    fi

    if [ -z "${multiarch}" ] && command -v dpkg-architecture >/dev/null 2>&1; then
        multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
    fi

    if [ -z "${multiarch}" ] && command -v gcc >/dev/null 2>&1; then
        multiarch="$(gcc -print-multiarch 2>/dev/null || true)"
    fi

    if [ -z "${multiarch}" ]; then
        machine="$(uname -m 2>/dev/null || true)"
        case "${machine}" in
            aarch64|arm64)
                multiarch="aarch64-linux-gnu"
                ;;
            x86_64|amd64)
                multiarch="x86_64-linux-gnu"
                ;;
            armv7l|armhf)
                multiarch="arm-linux-gnueabihf"
                ;;
        esac
    fi

    if [ -n "${multiarch}" ]; then
        case "${multiarch}" in
            .*|*..*|*/*|*[!A-Za-z0-9._-]*)
                echo "ERROR: unsafe CycloneDDS multiarch value: ${multiarch}" >&2
                exit 2
                ;;
        esac
        if [ -d "${prefix}/lib/${multiarch}" ]; then
            paths+=("${prefix}/lib/${multiarch}")
        fi
    fi

    paths+=("${prefix}/lib")
    for path in "${paths[@]}"; do
        if [ -z "${joined}" ]; then
            joined="${path}"
        else
            joined="${joined}:${path}"
        fi
    done
    export LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
}

prepend_cyclonedds_libs

if [ ! -x "${LINGTU_NAV_DDS_BIN}" ]; then
    echo "ERROR: native navigation DDS endpoint is missing or not executable: ${LINGTU_NAV_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 2
fi

if [ -z "${NAV_MAP_DIR}" ] || [ -z "${LINGTU_ACTIVE_PLANNER_MAP}" ]; then
    echo "ERROR: native navigation requires NAV_MAP_DIR and LINGTU_ACTIVE_PLANNER_MAP" >&2
    exit 2
fi

exec "${LINGTU_NAV_DDS_BIN}" \
    --control-mode "${LINGTU_NAV_CONTROL_MODE}" \
    --global-planner "${LINGTU_NAV_GLOBAL_PLANNER}" \
    --domain-id "${LINGTU_DDS_DOMAIN_ID}" \
    --path-library "${LINGTU_LOCAL_PLANNER_PATHS}" \
    --map-root "${NAV_MAP_DIR}" \
    --map "${LINGTU_ACTIVE_PLANNER_MAP}" \
    --tick-hz "${LINGTU_NAV_DDS_TICK_HZ}" \
    --max-obstacle-points "${LINGTU_NAV_DDS_MAX_OBSTACLE_POINTS}" \
    --obstacle-voxel-size-m "${LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M}" \
    --obstacle-registered-share "${LINGTU_NAV_OBSTACLE_REGISTERED_SHARE}" \
    --obstacle-terrain-share "${LINGTU_NAV_OBSTACLE_TERRAIN_SHARE}" \
    --obstacle-terrain-ext-share "${LINGTU_NAV_OBSTACLE_TERRAIN_EXT_SHARE}" \
    --live-obstacle-decay-s "${LINGTU_NAV_LIVE_OBSTACLE_DECAY_S}" \
    --live-obstacle-inflation-radius-m "${LINGTU_NAV_LIVE_OBSTACLE_INFLATION_RADIUS_M}" \
    --live-obstacle-ray-clearing "${LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING}" \
    --live-obstacle-ray-clear-max-range-m "${LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEAR_MAX_RANGE_M}" \
    --live-obstacle-ray-clearing-interval-s "${LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING_INTERVAL_S}" \
    --live-obstacle-max-clearing-rays "${LINGTU_NAV_LIVE_OBSTACLE_MAX_CLEARING_RAYS}" \
    --live-obstacle-min-hits "${LINGTU_NAV_LIVE_OBSTACLE_MIN_HITS}" \
    --publish-cmd-vel "${LINGTU_NAV_PUBLISH_CMD_VEL}" \
    --check-obstacle "${LINGTU_NAV_CHECK_OBSTACLE}" \
    --use-traversability-cost "${LINGTU_NAV_USE_TRAVERSABILITY_COST}" \
    --traversability-max-age-s "${LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S}" \
    --localization-health-max-age-s "${LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S}" \
    --allow-teleop-takeover "${LINGTU_NAV_ALLOW_TELEOP_TAKEOVER}" \
    --teleop-local-planner "${LINGTU_TELEOP_LOCAL_PLANNER}" \
    --teleop-planner-horizon-m "${LINGTU_TELEOP_PLANNER_HORIZON_M}" \
    --teleop-planner-max-deviation-deg "${LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG}" \
    --allow-legacy-motion-inputs "${LINGTU_NAV_ALLOW_LEGACY_MOTION_INPUTS}" \
    --terrain-map-max-age-s "${LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S}" \
    --odom-max-age-s "${LINGTU_NAV_ODOM_MAX_AGE_S}" \
    --tf-max-age-s "${LINGTU_NAV_TF_MAX_AGE_S}" \
    --cloud-max-age-s "${LINGTU_NAV_CLOUD_MAX_AGE_S}" \
    --cloud-pose-max-gap-s "${LINGTU_NAV_CLOUD_POSE_MAX_GAP_S}" \
    --input-recovery-frames "${LINGTU_NAV_INPUT_RECOVERY_FRAMES}" \
    --local-planner-debug-candidates "${LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES}" \
    --local-map-debug-points "${LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS}" \
    --status-file "${LINGTU_NAV_STATUS_FILE}" \
    --estop-latch-file "${LINGTU_NAV_ESTOP_LATCH_FILE}" \
    --status-s "${LINGTU_NAV_STATUS_S}"
