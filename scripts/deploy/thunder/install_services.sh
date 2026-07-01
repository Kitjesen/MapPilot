#!/usr/bin/env bash
# Canonical Thunder service installer entrypoint.
#
# Default Thunder deployment installs no-ROS product services. Legacy ROS
# compatibility services remain available only through an explicit mode.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-dds-endpoint}"

case "${MODE}" in
    dds|dds-endpoint|field|thunder-nav)
        exec "${SCRIPT_DIR}/install_dds_endpoint_service.sh"
        ;;
    slam-dds|cpp-slam)
        exec "${SCRIPT_DIR}/install_slam_dds_service.sh"
        ;;
    field-cpp|dds-cpp)
        "${SCRIPT_DIR}/install_dds_endpoint_service.sh"
        exec "${SCRIPT_DIR}/install_slam_dds_service.sh"
        ;;
    lcm|lcm-endpoint)
        exec "${SCRIPT_DIR}/install_lcm_endpoint_service.sh"
        ;;
    lite|thunder-lite|basic|thunder-basic)
        exec "${SCRIPT_DIR}/install_lite_service.sh"
        ;;
    ros-compat|legacy)
        shift || true
        LEGACY_INSTALLER="${SCRIPT_DIR}/../s100p/install_services.sh"
        CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
        sudo mkdir -p "${CONFIG_DIR}"
        sudo cp "${SCRIPT_DIR}/ros2-env.sh" "${CONFIG_DIR}/ros2-env.sh"
        sudo chmod 0644 "${CONFIG_DIR}/ros2-env.sh"
        echo "Installed Thunder ROS compatibility environment: ${CONFIG_DIR}/ros2-env.sh"
        if [ "$#" -eq 0 ]; then
            exec "${LEGACY_INSTALLER}" "${SCRIPT_DIR}/../s100p"
        fi
        exec "${LEGACY_INSTALLER}" "$@"
        ;;
    *)
        echo "Usage: $0 [dds-endpoint|slam-dds|field-cpp|lcm-endpoint|lite|ros-compat]" >&2
        exit 2
        ;;
esac
