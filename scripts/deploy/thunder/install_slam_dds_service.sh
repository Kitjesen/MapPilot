#!/usr/bin/env bash
# Install the C++ LiDAR/IMU -> SLAM CycloneDDS runtime service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LINGTU_SERVICE_NAME="${LINGTU_SLAM_DDS_SERVICE_NAME:-${LINGTU_SERVICE_NAME:-}}"
exec bash "${SCRIPT_DIR}/install_catalog_service.sh" slam
