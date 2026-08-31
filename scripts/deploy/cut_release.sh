#!/usr/bin/env bash
# Build/deploy one Product, then package the same checkout as a native release.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAW_VERSION="${1:-}"
OUTPUT_DIR="${LINGTU_RELEASE_OUTPUT_DIR:-dist}"

if [[ ! "${RAW_VERSION}" =~ ^v?[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Usage: $0 <version> <product> [deploy arguments...]" >&2
    exit 64
fi
shift
VERSION="v${RAW_VERSION#v}"

PRODUCT="${LINGTU_DEPLOY_PRODUCT:-}"
if [[ $# -gt 0 && "$1" != -* ]]; then
    PRODUCT="$1"
    shift
fi
if [ -z "${PRODUCT}" ]; then
    echo "Usage: $0 <version> <product> [deploy arguments...]" >&2
    exit 64
fi

bash "${SCRIPT_DIR}/deploy_robot.sh" "${PRODUCT}" "$@"
bash "${SCRIPT_DIR}/package_native_release.sh" "${VERSION}" "${OUTPUT_DIR}"
