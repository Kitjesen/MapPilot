#!/usr/bin/env bash
# Canonical Thunder field deployment entrypoint.
#
# Usage on the robot:
#   bash ~/data/inovxio/lingtu/scripts/deploy/deploy_thunder.sh
#   LINGTU_DEPLOY_PRODUCT=nav LINGTU_DEPLOY_MAP=<map> bash ~/data/inovxio/lingtu/scripts/deploy/deploy_thunder.sh
#
# Usage from a workstation:
#   ssh sunrise@192.168.66.13 'bash -s' < scripts/deploy/deploy_thunder.sh

set -euo pipefail

REPO="${LINGTU_REPO:-$HOME/data/inovxio/lingtu}"
PYTHON_BIN="${LINGTU_PYTHON:-python3}"
PRODUCT="${LINGTU_DEPLOY_PRODUCT:-${1:-}}"
if [ $# -gt 0 ]; then
    shift
fi

is_field_product() {
    case "${PRODUCT}" in
        ""|teleop|teleop_avoid|map|explore|nav|tracking|inspection)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

echo "========================================"
echo "  LingTu Thunder deployment"
echo "========================================"
echo "  repo:    ${REPO}"
if [ -n "${PRODUCT}" ]; then
    echo "  product: ${PRODUCT}"
else
    echo "  product: <not activated>"
fi
echo ""

if ! is_field_product; then
    echo "ERROR: LINGTU_DEPLOY_PRODUCT must be a field Product, not a local Profile: ${PRODUCT}" >&2
    exit 2
fi

cd "${REPO}"

if [ "${LINGTU_DEPLOY_UPDATE:-0}" = "1" ]; then
    echo "[1/5] Updating repository with git pull --ff-only..."
    git fetch origin
    git pull --ff-only
    echo "  revision: $(git log --oneline -1)"
else
    echo "[1/5] Repository update skipped (set LINGTU_DEPLOY_UPDATE=1 to enable)."
    echo "  revision: $(git log --oneline -1 2>/dev/null || echo unknown)"
fi

echo "[2/5] Building dashboard when Node.js is available..."
if command -v node >/dev/null 2>&1 && [ -d "${REPO}/web" ]; then
    (
        cd "${REPO}/web"
        npm install --silent
        npm run build
    )
    echo "  dashboard built"
else
    echo "  skipped: Node.js or web/ is not available"
fi

echo "[3/5] Building production navigation, driver, camera, and maps..."
if [ "${LINGTU_DEPLOY_BUILD_NATIVE:-1}" = "1" ]; then
    bash "${REPO}/scripts/build/build_maps.sh"
    bash "${REPO}/scripts/build/build_nav_kernel.sh" --clean
    bash "${REPO}/scripts/build/build_nav_endpoint.sh"
    bash "${REPO}/scripts/build/build_orbbec_native.sh"
    bash "${REPO}/scripts/build/build_camera_dds.sh"
    PYTHONPATH="${REPO}/src:${PYTHONPATH:-}" "${PYTHON_BIN}" - <<'PY'
from nav.kernel import require_nav_kernel
require_nav_kernel(context="Thunder deployment")
print("  nav_kernel OK")
PY
    bash "${REPO}/scripts/build/build_driver.sh"
else
    echo "  skipped: set LINGTU_DEPLOY_BUILD_NATIVE=1 to enable native builds"
fi

echo "[4/5] Installing Thunder field services when requested..."
if [ "${LINGTU_DEPLOY_INSTALL_SERVICES:-0}" = "1" ]; then
    bash "${REPO}/scripts/deploy/thunder/install_services.sh" "${LINGTU_DEPLOY_SERVICE_MODE:-field-cpp}"
else
    echo "  skipped: set LINGTU_DEPLOY_INSTALL_SERVICES=1 to install systemd units"
fi

echo "[5/5] Activation through ProductControl..."
if [ -n "${PRODUCT}" ]; then
    args=()
    if [ -n "${LINGTU_DEPLOY_MAP:-}" ]; then
        args+=(--map "${LINGTU_DEPLOY_MAP}")
    fi
    args+=("$@")
    bash "${REPO}/scripts/lingtu" --env real mode switch "${PRODUCT}" "${args[@]}"
else
    echo "  skipped: set LINGTU_DEPLOY_PRODUCT=<product> or pass <product> to activate"
fi

echo ""
echo "Thunder deployment complete."
