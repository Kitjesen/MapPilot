#!/usr/bin/env bash
# Install the Thunder field typed DDS endpoint service without ROS compatibility units.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
SERVICE_NAME="${LINGTU_ENDPOINT_SERVICE_NAME:-lingtu-thunder-dds-endpoint.service}"

sudo mkdir -p "${CONFIG_DIR}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"

sudo cp "${SCRIPT_DIR}/lingtu-thunder-dds-endpoint.service" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_SERVICE:-0}" = "1" ]; then
    sudo systemctl enable "${SERVICE_NAME}"
fi

echo "Installed Thunder runtime environment: ${CONFIG_DIR}/thunder-runtime-env.sh"
echo "Installed Thunder typed DDS endpoint service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Start with: sudo systemctl start ${SERVICE_NAME}"
