#!/usr/bin/env bash
# Install one Thunder service from runtime.service_catalogs.thunder.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
SERVICE="${1:?usage: install_catalog_service.sh <service>}"
PYTHON_BIN="${PYTHON:-python3}"

catalog() {
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${PYTHON_BIN}" -m runtime.service_catalogs.thunder "$@"
}

SERVICE_FILE="$(catalog install-unit "${SERVICE}")"
INSTALLER="$(catalog installer "${SERVICE}")"
CATALOG_ENABLE_DEFAULT="$(catalog install-enable-default "${SERVICE}")"
if [ -z "${SERVICE_FILE}" ] || [ -z "${INSTALLER}" ]; then
    echo "Unknown Thunder catalog service: ${SERVICE}" >&2
    exit 2
fi

SERVICE_NAME="${LINGTU_SERVICE_NAME:-${SERVICE_FILE}}"
ENABLE_DEFAULT="${LINGTU_ENABLE_SERVICE_DEFAULT:-${CATALOG_ENABLE_DEFAULT}}"

sudo mkdir -p "${CONFIG_DIR}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"

sudo cp "${SCRIPT_DIR}/${SERVICE_FILE}" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}" = "1" ]; then
    sudo systemctl enable "${SERVICE_NAME}"
fi

echo "Installed Thunder runtime environment: ${CONFIG_DIR}/thunder-runtime-env.sh"
echo "Installed Thunder ${SERVICE} service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Catalog installer: ${INSTALLER}"
echo "Start with: sudo systemctl start ${SERVICE_NAME}"
