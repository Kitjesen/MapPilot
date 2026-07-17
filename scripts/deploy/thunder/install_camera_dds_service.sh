#!/usr/bin/env bash
# Install the optional native camera DDS publisher service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LINGTU_SERVICE_NAME="${LINGTU_CAMERA_DDS_SERVICE_NAME:-${LINGTU_SERVICE_NAME:-}}"
exec bash "${SCRIPT_DIR}/install_catalog_service.sh" camera
