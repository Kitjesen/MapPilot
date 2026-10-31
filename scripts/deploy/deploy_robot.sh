#!/usr/bin/env bash
# Canonical on-robot deployment entrypoint for the real environment.
#
# The workstation SSH target is deliberately outside this script. Example:
#   ssh "${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}" 'bash -s' \
#     < scripts/deploy/deploy_robot.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/python-runtime.sh"

REPO="${LINGTU_REPO:-$HOME/data/inovxio/lingtu}"
PRODUCT="${LINGTU_DEPLOY_PRODUCT:-}"
if [ -z "${PRODUCT}" ] && [ $# -gt 0 ]; then
    case "$1" in
        -*) ;;
        *)
            PRODUCT="$1"
            shift
            ;;
    esac
fi
if [ -z "${PRODUCT}" ]; then
    echo "ERROR: specify the Product as the first argument or LINGTU_DEPLOY_PRODUCT." >&2
    exit 64
fi

PYTHON_BIN="$(resolve_lingtu_python)"
export LINGTU_PYTHON="${PYTHON_BIN}"

cd "${REPO}"
read -r configured_backend configured_camera_driver < <(
    PYTHONPATH="${REPO}/src:${PYTHONPATH:-}" "${PYTHON_BIN}" - <<'PY'
from runtime.config import load_config
config = load_config()
print(config.driver.backend, config.camera.capture_driver)
PY
)
export LINGTU_CAMERA_DRIVER="${configured_camera_driver}"
ROBOT="${LINGTU_DEPLOY_ROBOT:-$(PYTHONPATH="${REPO}/src:${PYTHONPATH:-}" "${PYTHON_BIN}" - <<'PY'
from lingtu.assembly.config.product import resolve_env_spec
print(resolve_env_spec("real").robot)
PY
)}"
BACKEND="${LINGTU_DRIVER_BACKEND:-${configured_backend}}"
if [ "${BACKEND}" != "${configured_backend}" ]; then
    echo "ERROR: LINGTU_DRIVER_BACKEND=${BACKEND} disagrees with RobotConfig (${configured_backend})." >&2
    echo "Change the selected robot's robot.yaml; deployment variables do not own robot identity." >&2
    exit 64
fi

mapfile -t NATIVE_BUILD_SCRIPTS < <(
    PYTHONPATH="${REPO}/src:${PYTHONPATH:-}" "${PYTHON_BIN}" - "${PRODUCT}" "${LINGTU_CAMERA_DRIVER}" "$@" <<'PY'
import argparse
import sys

from lingtu.assembly.build_scripts import product_native_build_scripts

product = sys.argv[1]
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--variant")
args, _ = parser.parse_known_args(sys.argv[3:])
scripts = product_native_build_scripts(product, product_variant=args.variant, camera_driver=sys.argv[2])
sys.stdout.write("\n".join(scripts))
if scripts:
    sys.stdout.write("\n")
PY
)
if [ "${#NATIVE_BUILD_SCRIPTS[@]}" -eq 0 ]; then
    echo "ERROR: native build plan is empty." >&2
    exit 2
fi

echo "========================================"
echo "  LingTu real-environment deployment"
echo "========================================"
echo "  repo:    ${REPO}"
echo "  robot:   ${ROBOT}"
echo "  driver:  ${BACKEND}"
echo "  product: ${PRODUCT}"
echo "  builds:  ${NATIVE_BUILD_SCRIPTS[*]}"
echo ""

if [ "${LINGTU_DEPLOY_PLAN_ONLY:-0}" = "1" ]; then
    echo "Deployment plan resolved; no files, services, or network settings were changed."
    exit 0
fi

if [ "${LINGTU_DEPLOY_UPDATE:-0}" = "1" ]; then
    echo "[1/5] Updating repository with git pull --ff-only..."
    git fetch origin
    git pull --ff-only
    echo "  revision: $(git log --oneline -1)"
else
    echo "[1/5] Repository update skipped (set LINGTU_DEPLOY_UPDATE=1 to enable)."
    echo "  revision: $(git log --oneline -1 2>/dev/null || echo unknown)"
fi

echo "[2/5] Provisioning the robot-facing network..."
if [ "${BACKEND}" = "go2" ]; then
    read -r GO2_INTERFACE GO2_ADDRESS GO2_PROBE_IP < <(
        PYTHONPATH="${REPO}/src:${PYTHONPATH:-}" "${PYTHON_BIN}" - <<'PY'
from runtime.config import load_config

driver = load_config().driver
print(driver.network_interface, driver.network_address, driver.probe_ip)
PY
    )
    bash "${REPO}/scripts/deploy/configure_go2_network.sh" \
        "${GO2_INTERFACE}" "${GO2_ADDRESS}" "${GO2_PROBE_IP}"
else
    echo "  skipped: ${BACKEND} does not use the Go2 wired subnet"
fi

echo "[3/5] Building native runtimes selected by the Product..."
if [ "${LINGTU_DEPLOY_BUILD_NATIVE:-1}" = "1" ]; then
    for build_script in "${NATIVE_BUILD_SCRIPTS[@]}"; do
        echo "  -> ${build_script}"
        if [ "${build_script}" = "scripts/build/build_driver.sh" ]; then
            LINGTU_DRIVER_BACKEND="${BACKEND}" bash "${REPO}/${build_script}"
        else
            bash "${REPO}/${build_script}"
        fi
    done
else
    echo "  skipped: set LINGTU_DEPLOY_BUILD_NATIVE=1 to enable native builds"
fi

echo "[4/5] Installing field services when requested..."
if [ "${LINGTU_DEPLOY_INSTALL_SERVICES:-0}" = "1" ]; then
    LINGTU_DRIVER_BACKEND="${BACKEND}" \
        bash "${REPO}/scripts/deploy/thunder/install_services.sh" \
        "${LINGTU_DEPLOY_SERVICE_MODE:-field-cpp}"
else
    echo "  skipped: set LINGTU_DEPLOY_INSTALL_SERVICES=1 to install systemd units"
fi

echo "[5/5] Activation through ProductControl..."
args=()
if [ -n "${LINGTU_DEPLOY_MAP:-}" ]; then
    args+=(--map "${LINGTU_DEPLOY_MAP}")
fi
args+=("$@")
bash "${REPO}/scripts/lingtu" --robot "${ROBOT}" --env real switch "${PRODUCT}" "${args[@]}"

echo "Deployment complete."
