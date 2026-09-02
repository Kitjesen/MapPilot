#!/usr/bin/env bash
# Canonical Thunder service installer entrypoint.
#
# Supported modes come only from the Thunder product service catalog.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
MODE="${1:-field-cpp}"
source "${SCRIPT_DIR}/../python-runtime.sh"
PYTHON_BIN="$(resolve_lingtu_python)"
export LINGTU_PYTHON="${PYTHON_BIN}"
CURRENT_RELEASE="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"

require_dual_release_layout() {
    if [ ! -e "${CURRENT_RELEASE}" ] && [ ! -L "${CURRENT_RELEASE}" ]; then
        echo "Refusing to install canonical systemd units: ${CURRENT_RELEASE} is missing." >&2
        echo "Install and activate a dual-layout release before switching systemd to canonical paths." >&2
        return 2
    fi

    local relative
    for relative in \
        bin/navd \
        lib/liblingtu_nav_client.so \
        etc/lingtu \
        share/lingtu \
        build/nav_endpoint/navd \
        build/nav_endpoint/liblingtu_nav_client.so; do
        if [ ! -e "${CURRENT_RELEASE}/${relative}" ]; then
            echo "Refusing to install canonical systemd units: ${CURRENT_RELEASE}/${relative} is missing." >&2
            echo "Install and activate a dual-layout release before switching systemd to canonical paths." >&2
            return 2
        fi
    done
}

catalog_services_for_mode() {
    local mode="$1"
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${PYTHON_BIN}" -m runtime.service_catalogs.thunder install-services "${mode}"
}

catalog_install_modes() {
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${PYTHON_BIN}" -m runtime.service_catalogs.thunder install-modes
}

usage() {
    echo "Usage: $0 [<catalog-mode>]" >&2
    echo "Catalog modes:" >&2
    catalog_install_modes >&2 || true
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
        if [ "${service}" = "driver" ]; then
            bash "${SCRIPT_DIR}/install_driver_service.sh"
        else
            bash "${SCRIPT_DIR}/install_catalog_service.sh" "${service}"
        fi
    done
}

main() {
    local service_text
    if service_text="$(catalog_services_for_mode "${MODE}")"; then
        require_dual_release_layout
        run_catalog_services "${service_text}"
        return
    fi

    echo "Unknown Thunder service install mode: ${MODE}" >&2
    usage
    return 2
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
