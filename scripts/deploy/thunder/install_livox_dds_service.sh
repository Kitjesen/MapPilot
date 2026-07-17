#!/usr/bin/env bash
# Install the native Livox MID-360 DDS publisher service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LINGTU_SERVICE_NAME="${LINGTU_LIVOX_DDS_SERVICE_NAME:-${LINGTU_SERVICE_NAME:-}}"
exec bash "${SCRIPT_DIR}/install_catalog_service.sh" lidar
