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

if [ ! -x "${LINGTU_EXPLORE_DDS_BIN}" ]; then
    echo "ERROR: native exploration endpoint is missing or not executable: ${LINGTU_EXPLORE_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 2
fi

exec "${LINGTU_EXPLORE_DDS_BIN}" \
    --domain "${LINGTU_DDS_DOMAIN_ID}" \
    --tick-hz "${LINGTU_EXPLORE_TICK_HZ}" \
    --command-retry-s "${LINGTU_EXPLORE_COMMAND_RETRY_S}" \
    --command-timeout-ms "${LINGTU_EXPLORE_COMMAND_TIMEOUT_MS}" \
    --arrival-tolerance "${LINGTU_EXPLORE_ARRIVAL_TOLERANCE_M}" \
    --goal-timeout "${LINGTU_EXPLORE_GOAL_TIMEOUT_S}" \
    --status-period "${LINGTU_EXPLORE_STATUS_PERIOD_S}" \
    --status-file "${LINGTU_EXPLORE_STATUS_FILE}" \
    --control-max-age "${LINGTU_EXPLORE_CONTROL_MAX_AGE_S}" \
    --odom-max-age "${LINGTU_EXPLORE_ODOM_MAX_AGE_S}" \
    --snapshot-max-age "${LINGTU_EXPLORE_SNAPSHOT_MAX_AGE_S}" \
    --tf-max-age "${LINGTU_EXPLORE_TF_MAX_AGE_S}" \
    --future-tolerance "${LINGTU_EXPLORE_FUTURE_TOLERANCE_S}" \
    --min-frontier-size "${LINGTU_EXPLORE_MIN_FRONTIER_SIZE}" \
    --sensor-range "${LINGTU_EXPLORE_SENSOR_RANGE_M}" \
    --candidate-radius "${LINGTU_EXPLORE_CANDIDATE_RADIUS_M}" \
    --min-goal-distance "${LINGTU_EXPLORE_MIN_GOAL_DISTANCE_M}" \
    --novelty-radius "${LINGTU_EXPLORE_NOVELTY_RADIUS_M}" \
    --max-candidates "${LINGTU_EXPLORE_MAX_CANDIDATES}" \
    --coverage-resolution "${LINGTU_EXPLORE_COVERAGE_RESOLUTION_M}" \
    --local-route-radius "${LINGTU_EXPLORE_LOCAL_ROUTE_RADIUS_M}" \
    --return-home-distance "${LINGTU_EXPLORE_RETURN_HOME_DISTANCE_M}" \
    --keypose-min-distance "${LINGTU_EXPLORE_KEYPOSE_MIN_DISTANCE_M}" \
    --keypose-connect-distance "${LINGTU_EXPLORE_KEYPOSE_CONNECT_DISTANCE_M}" \
    --gain-weight "${LINGTU_EXPLORE_GAIN_WEIGHT}" \
    --travel-weight "${LINGTU_EXPLORE_TRAVEL_WEIGHT}" \
    --momentum-weight "${LINGTU_EXPLORE_MOMENTUM_WEIGHT}" \
    --revisit-weight "${LINGTU_EXPLORE_REVISIT_WEIGHT}" \
    --max-plan-time-ms "${LINGTU_EXPLORE_MAX_PLAN_TIME_MS}" \
    --route-2opt-iterations "${LINGTU_EXPLORE_ROUTE_2OPT_ITERATIONS}" \
    --max-grid-cells "${LINGTU_EXPLORE_MAX_GRID_CELLS}" \
    --max-frontier-cells "${LINGTU_EXPLORE_MAX_FRONTIER_CELLS}" \
    --max-frontier-clusters "${LINGTU_EXPLORE_MAX_FRONTIER_CLUSTERS}" \
    --max-coverage-cells "${LINGTU_EXPLORE_MAX_COVERAGE_CELLS}" \
    --max-keyposes "${LINGTU_EXPLORE_MAX_KEYPOSES}" \
    --max-keypose-edges "${LINGTU_EXPLORE_MAX_KEYPOSE_EDGES}" \
    --max-keypose-neighbor-links "${LINGTU_EXPLORE_MAX_KEYPOSE_NEIGHBOR_LINKS}" \
    --max-route-targets "${LINGTU_EXPLORE_MAX_ROUTE_TARGETS}" \
    --return-home "${LINGTU_EXPLORE_RETURN_HOME}"
