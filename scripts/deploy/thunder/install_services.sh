#!/usr/bin/env bash
# Canonical Thunder service installer entrypoint.
#
# Default Thunder deployment installs no-ROS product services. Legacy ROS
# compatibility services remain available only through an explicit mode.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
MODE="${1:-field-cpp}"

catalog_services_for_mode() {
    local mode="$1"
    local python_bin="${PYTHON:-python3}"
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${python_bin}" -m runtime.service_catalogs.thunder install-services "${mode}"
}

catalog_install_modes() {
    local python_bin="${PYTHON:-python3}"
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${python_bin}" -m runtime.service_catalogs.thunder install-modes
}

catalog_installer() {
    local service="$1"
    local python_bin="${PYTHON:-python3}"
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${python_bin}" -m runtime.service_catalogs.thunder installer "${service}"
}

usage() {
    echo "Usage: $0 [<catalog-mode>|lite|ros-compat]" >&2
    echo "Catalog modes:" >&2
    catalog_install_modes >&2 || true
    echo "Compatibility modes: lite thunder-lite basic thunder-basic ros-compat legacy" >&2
}

run_catalog_services() {
    local service_text="$1"
    local services=()
    mapfile -t services <<< "${service_text}"
    if [ "${#services[@]}" -eq 0 ]; then
        echo "Thunder service install plan is empty." >&2
        exit 2
    fi
    local service
    for service in "${services[@]}"; do
        local installer
        installer="$(catalog_installer "${service}")"
        if [ -z "${installer}" ] || [ ! -f "${SCRIPT_DIR}/${installer}" ]; then
            echo "Missing catalog installer for Thunder service ${service}: ${installer:-<empty>}" >&2
            exit 2
        fi
        bash "${SCRIPT_DIR}/${installer}"
    done
}

if service_text="$(catalog_services_for_mode "${MODE}")"; then
    run_catalog_services "${service_text}"
    exit 0
fi

case "${MODE}" in
    lite|thunder-lite|basic|thunder-basic)
        exec bash "${SCRIPT_DIR}/install_lite_service.sh"
        ;;
    ros-compat|legacy)
        if [ "${LINGTU_ENABLE_LEGACY_ROS2_SERVICES:-0}" != "1" ]; then
            echo "Refusing to install legacy ROS compatibility services by default." >&2
            echo "Set LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1 and rerun only for an explicit compatibility test." >&2
            exit 2
        fi
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
        echo "Unknown Thunder service install mode: ${MODE}" >&2
        usage
        exit 2
        ;;
esac
