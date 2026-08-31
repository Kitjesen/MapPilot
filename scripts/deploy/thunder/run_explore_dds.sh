#!/usr/bin/env bash
set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh
source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh explore

prepend_cyclonedds_libs

if [ ! -x "${LINGTU_EXPLORE_DDS_BIN}" ]; then
    echo "ERROR: native exploration endpoint is missing or not executable: ${LINGTU_EXPLORE_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 2
fi

exec "${LINGTU_EXPLORE_DDS_BIN}" --domain "${LINGTU_DDS_DOMAIN_ID}"
