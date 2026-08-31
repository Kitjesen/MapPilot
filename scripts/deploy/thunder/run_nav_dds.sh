#!/usr/bin/env bash
set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh
source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh nav

: "${LINGTU_NAV_VEHICLE_LENGTH_M:?Product session is missing LINGTU_NAV_VEHICLE_LENGTH_M}"
: "${LINGTU_NAV_VEHICLE_WIDTH_M:?Product session is missing LINGTU_NAV_VEHICLE_WIDTH_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_X_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_X_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_Y_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_Y_M}"
: "${LINGTU_NAV_SENSOR_OFFSET_Z_M:?Product session is missing LINGTU_NAV_SENSOR_OFFSET_Z_M}"
: "${LINGTU_TELEOP_SLOW_DISTANCE_M:?Product session is missing LINGTU_TELEOP_SLOW_DISTANCE_M}"
: "${LINGTU_TELEOP_STOP_DISTANCE_M:?Product session is missing LINGTU_TELEOP_STOP_DISTANCE_M}"
: "${LINGTU_NAV_LOCAL_PLANNER_BACKEND:?Product session is missing LINGTU_NAV_LOCAL_PLANNER_BACKEND}"
if [ "${LINGTU_NAV_LOCAL_PLANNER_BACKEND}" = cmu ]; then
    : "${LINGTU_LOCAL_PLANNER_PATHS:?CMU Product session is missing LINGTU_LOCAL_PLANNER_PATHS}"
    case "$(basename -- "${LINGTU_LOCAL_PLANNER_PATHS}")" in
        go2|thunder) ;;
        *)
            echo "ERROR: CMU Product session selected an unknown robot path profile: ${LINGTU_LOCAL_PLANNER_PATHS}" >&2
            exit 2
            ;;
    esac
    for asset in startPaths.ply pathList.ply paths.ply correspondences.txt search_radius.txt; do
        if [ ! -s "${LINGTU_LOCAL_PLANNER_PATHS}/${asset}" ]; then
            echo "ERROR: CMU path library is missing ${asset}: ${LINGTU_LOCAL_PLANNER_PATHS}" >&2
            exit 2
        fi
    done
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
