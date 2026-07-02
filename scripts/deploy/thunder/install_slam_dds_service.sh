#!/usr/bin/env bash
# Install the C++ LiDAR/IMU -> SLAM CycloneDDS runtime service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${LINGTU_SYSTEMD_DIR:-/etc/systemd/system}"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
SERVICE_NAME="${LINGTU_SLAM_DDS_SERVICE_NAME:-lingtu-slam-dds.service}"
LIVOX_SERVICE_NAME="${LINGTU_LIVOX_DDS_SERVICE_NAME:-lingtu-livox-dds.service}"
NAV_SERVICE_NAME="${LINGTU_NAV_DDS_SERVICE_NAME:-lingtu-nav-dds.service}"

sudo mkdir -p "${CONFIG_DIR}"
sudo cp "${SCRIPT_DIR}/runtime-env.sh" "${CONFIG_DIR}/thunder-runtime-env.sh"
sudo chmod 0644 "${CONFIG_DIR}/thunder-runtime-env.sh"

sudo cp "${SCRIPT_DIR}/lingtu-livox-dds.service" "${SYSTEMD_DIR}/${LIVOX_SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${LIVOX_SERVICE_NAME}"
sudo cp "${SCRIPT_DIR}/lingtu-slam-dds.service" "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${SERVICE_NAME}"
sudo cp "${SCRIPT_DIR}/lingtu-nav-dds.service" "${SYSTEMD_DIR}/${NAV_SERVICE_NAME}"
sudo chmod 0644 "${SYSTEMD_DIR}/${NAV_SERVICE_NAME}"
sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_SERVICE:-0}" = "1" ]; then
    sudo systemctl enable "${LIVOX_SERVICE_NAME}"
    sudo systemctl enable "${SERVICE_NAME}"
    sudo systemctl enable "${NAV_SERVICE_NAME}"
fi

echo "Installed LingTu runtime environment: ${CONFIG_DIR}/thunder-runtime-env.sh"
echo "Installed LingTu native Livox DDS service: ${SYSTEMD_DIR}/${LIVOX_SERVICE_NAME}"
echo "Installed LingTu C++ DDS SLAM service: ${SYSTEMD_DIR}/${SERVICE_NAME}"
echo "Installed LingTu C++ DDS navigation endpoint service: ${SYSTEMD_DIR}/${NAV_SERVICE_NAME}"
echo "Start with: sudo systemctl start ${LIVOX_SERVICE_NAME} ${SERVICE_NAME} ${NAV_SERVICE_NAME}"
