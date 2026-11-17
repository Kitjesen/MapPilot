#!/usr/bin/env bash
set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh
source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh nav

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_NAV_DDS_BIN:=${LINGTU_REPO}/bin/navd}"
: "${LINGTU_NAV_VEHICLE_LENGTH_M:?Product session is missing LINGTU_NAV_VEHICLE_LENGTH_M}"
: "${LINGTU_NAV_VEHICLE_WIDTH_M:?Product session is missing LINGTU_NAV_VEHICLE_WIDTH_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_X_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_X_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_Y_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_Y_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_Z_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_Z_M}"
: "${LINGTU_NAV_LOCAL_PLANNER_BACKEND:?Product session is missing LINGTU_NAV_LOCAL_PLANNER_BACKEND}"
: "${LINGTU_NAV_CONTROL_MODE:?Product session is missing LINGTU_NAV_CONTROL_MODE}"
: "${LINGTU_TELEOP_LOCAL_PLANNER:?Product session is missing LINGTU_TELEOP_LOCAL_PLANNER}"
if [ "${LINGTU_NAV_LOCAL_PLANNER_BACKEND}" != scan ]; then
    echo "ERROR: local planner must be scan, got ${LINGTU_NAV_LOCAL_PLANNER_BACKEND}" >&2
    exit 2
fi
if [ -n "${LINGTU_LOCAL_PLANNER_PATHS:-}" ]; then
    echo "ERROR: LINGTU_LOCAL_PLANNER_PATHS is removed; SCAN does not use a path library" >&2
    exit 2
fi

prepend_cyclonedds_libs

: "${NAV_GLOBAL_PLANNER:?Product session is missing NAV_GLOBAL_PLANNER}"

if [ ! -x "${LINGTU_NAV_DDS_BIN}" ]; then
    echo "ERROR: native navigation DDS endpoint is missing or not executable: ${LINGTU_NAV_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 2
fi

case "${LINGTU_SLAM_MODE}" in
    localization)
        case "${NAV_GLOBAL_PLANNER}" in
            octoplanner3d)
                : "${OCTOPLANNER_MAP_PATH:?OctoPlanner3D requires OCTOPLANNER_MAP_PATH}"
                ;;
            far)
                : "${FAR_OCCUPANCY_PATH:?FAR requires FAR_OCCUPANCY_PATH}"
                ;;
            *)
                echo "ERROR: unsupported NAV_GLOBAL_PLANNER=${NAV_GLOBAL_PLANNER}" >&2
                exit 2
                ;;
        esac
        ;;
    mapping|none)
        # Persistent unit defaults must never leak a previous saved map into a
        # map-free Product session.
        OCTOPLANNER_MAP_PATH=""
        FAR_OCCUPANCY_PATH=""
        ;;
    *)
        echo "ERROR: unsupported LINGTU_SLAM_MODE=${LINGTU_SLAM_MODE}" >&2
        exit 2
        ;;
esac

exec "${LINGTU_NAV_DDS_BIN}"
