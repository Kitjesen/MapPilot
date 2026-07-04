#!/usr/bin/env bash
# Install the C++ traversability DDS producer service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
SERVICE_NAME="${LINGTU_TRAVERSABILITY_DDS_SERVICE_NAME:-lingtu-traversability-dds.service}"

sudo mkdir -p "${CONFIG_DIR}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"

sudo cp "${SCRIPT_DIR}/lingtu-traversability-dds.service" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_SERVICE:-0}" = "1" ]; then
    sudo systemctl enable "${SERVICE_NAME}"
fi

echo "Installed LingTu traversability DDS producer service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Start with: sudo systemctl start ${SERVICE_NAME}"
