#!/usr/bin/env bash
# Canonical Thunder service installer entrypoint.
#
# Default Thunder deployment installs no-ROS product services. Legacy ROS
# compatibility services remain available only through an explicit mode.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-field-cpp}"

case "${MODE}" in
    dds|dds-endpoint|endpoint-only)
        exec bash "${SCRIPT_DIR}/install_dds_endpoint_service.sh"
        ;;
    slam-dds|cpp-slam)
        exec bash "${SCRIPT_DIR}/install_slam_dds_service.sh"
        ;;
    nav-dds|cpp-nav)
        exec bash "${SCRIPT_DIR}/install_nav_dds_service.sh"
        ;;
    traversability-dds|terrain-dds)
        exec bash "${SCRIPT_DIR}/install_traversability_dds_service.sh"
        ;;
    lingtu|app|runtime)
        exec bash "${SCRIPT_DIR}/install_lingtu_service.sh"
        ;;
    field|thunder-nav|field-cpp|dds-cpp)
        bash "${SCRIPT_DIR}/install_dds_endpoint_service.sh"
        bash "${SCRIPT_DIR}/install_slam_dds_service.sh"
        bash "${SCRIPT_DIR}/install_traversability_dds_service.sh"
        bash "${SCRIPT_DIR}/install_nav_dds_service.sh"
        exec bash "${SCRIPT_DIR}/install_lingtu_service.sh"
        ;;
    lite|thunder-lite|basic|thunder-basic)
        exec bash "${SCRIPT_DIR}/install_lite_service.sh"
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
            exec bash "${LEGACY_INSTALLER}" "${SCRIPT_DIR}/../s100p"
        fi
        exec bash "${LEGACY_INSTALLER}" "$@"
        ;;
    *)
        echo "Usage: $0 [field-cpp|dds-endpoint|slam-dds|traversability-dds|nav-dds|lingtu|lite|ros-compat]" >&2
        exit 2
        ;;
esac
