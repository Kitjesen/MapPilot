#!/usr/bin/env bash
# Install the Thunder navigation runtime service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
SERVICE_NAME="${LINGTU_SERVICE_NAME:-lingtu.service}"

sudo mkdir -p "${CONFIG_DIR}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"

sudo cp "${SCRIPT_DIR}/lingtu.service" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_SERVICE:-0}" = "1" ]; then
    sudo systemctl enable "${SERVICE_NAME}"
fi

echo "Installed LingTu navigation runtime service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Start with: sudo systemctl start ${SERVICE_NAME}"
