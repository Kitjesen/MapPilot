#!/usr/bin/env bash
# Install one Thunder service from runtime.service_catalogs.thunder.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
TMPFILES_DIR="${LINGTU_TMPFILES_DIR:-/etc/tmpfiles.d}"
SERVICE="${1:?usage: install_catalog_service.sh <service>}"
source "${SCRIPT_DIR}/../python-runtime.sh"
PYTHON_BIN="$(resolve_lingtu_python)"
export LINGTU_PYTHON="${PYTHON_BIN}"

catalog() {
    PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
        "${PYTHON_BIN}" -m runtime.service_catalogs.thunder "$@"
}

SERVICE_FILE="$(catalog install-unit "${SERVICE}")"
CATALOG_ENABLE_DEFAULT="$(catalog install-enable-default "${SERVICE}")"
if [ -z "${SERVICE_FILE}" ]; then
    echo "Unknown Thunder catalog service: ${SERVICE}" >&2
    exit 2
fi

SERVICE_NAME="${SERVICE_FILE}"
ENABLE_DEFAULT="${LINGTU_ENABLE_SERVICE_DEFAULT:-${CATALOG_ENABLE_DEFAULT}}"

if ! getent passwd lingtu >/dev/null 2>&1; then
    sudo useradd --system --home-dir /var/lib/lingtu --create-home \
        --shell /usr/sbin/nologin lingtu
fi
sudo install -d -o lingtu -g lingtu -m 0750 /var/lib/lingtu
sudo install -d -o lingtu -g lingtu -m 0750 /var/lib/lingtu/maps
sudo install -d -o lingtu -g lingtu -m 0750 /var/lib/lingtu/task_journal
sudo install -d -o lingtu -g lingtu -m 0750 /var/lib/lingtu/inspection_evidence
sudo chown lingtu:lingtu \
    /var/lib/lingtu \
    /var/lib/lingtu/maps \
    /var/lib/lingtu/task_journal \
    /var/lib/lingtu/inspection_evidence

sudo mkdir -p "${CONFIG_DIR}"
install_lingtu_python_env "${CONFIG_DIR}/python.env" "${PYTHON_BIN}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo mkdir -p "${TMPFILES_DIR}"
sudo cp "${SCRIPT_DIR}/lingtu-runtime.conf" "${TMPFILES_DIR}/lingtu.conf"
sudo chmod 0644 "${TMPFILES_DIR}/lingtu.conf"
sudo systemd-tmpfiles --create "${TMPFILES_DIR}/lingtu.conf"

sudo cp "${SCRIPT_DIR}/${SERVICE_FILE}" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo systemctl daemon-reload

mapfile -t RETIRED_UNITS < <(catalog retired-units "${SERVICE}")
for unit in "${RETIRED_UNITS[@]}"; do
    if systemctl list-unit-files "${unit}" --no-legend 2>/dev/null \
        | grep -Fq "${unit}"; then
        sudo systemctl disable --now "${unit}"
        echo "Disabled retired LingTu service: ${unit}"
    fi
done

REQUESTED_ENABLE="${LINGTU_ENABLE_SERVICE:-${ENABLE_DEFAULT}}"
if [ "${REQUESTED_ENABLE}" = "1" ]; then
    if [ "${CATALOG_ENABLE_DEFAULT}" = "1" ]; then
        sudo systemctl enable "${SERVICE_NAME}"
    else
        echo "Not boot-enabling Thunder ${SERVICE}: ProductControl owns Product role activation."
    fi
fi

echo "Installed Thunder runtime environment: ${CONFIG_DIR}/thunder-runtime-env.sh"
echo "Installed transient runtime directory rule: ${TMPFILES_DIR}/lingtu.conf"
echo "Installed Thunder ${SERVICE} service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Installation does not activate Product processes."
echo "Switch a Product with: bash ${REPO_ROOT}/scripts/lingtu switch <product> --robot <vendor/model> --env real [--map <name>]"
