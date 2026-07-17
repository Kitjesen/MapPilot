#!/usr/bin/env bash
# Canonical Thunder field deployment entrypoint.
#
# Usage on the robot:
#   bash ~/data/inovxio/lingtu/scripts/deploy/deploy_thunder.sh
#
# Usage from a workstation:
#   ssh sunrise@192.168.66.13 'bash -s' < scripts/deploy/deploy_thunder.sh

set -euo pipefail

REPO="${LINGTU_REPO:-$HOME/data/inovxio/lingtu}"
PROFILE="${LINGTU_DEPLOY_PROFILE:-nav}"
PYTHON_BIN="${LINGTU_PYTHON:-python3}"
LOG="${LINGTU_DEPLOY_LOG:-/tmp/deploy_thunder.log}"
STARTUP_TIMEOUT="${LINGTU_DEPLOY_STARTUP_TIMEOUT:-30}"

is_lite_profile() {
    case "${PROFILE}" in
        lite|thunder-lite|basic|thunder-basic)
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
echo "  profile: ${PROFILE}"
echo "  log:     ${LOG}"
echo ""

cd "${REPO}"

if [ "${LINGTU_DEPLOY_UPDATE:-0}" = "1" ]; then
    echo "[1/6] Updating repository with git pull --ff-only..."
    git fetch origin
    git pull --ff-only
    echo "  revision: $(git log --oneline -1)"
else
    echo "[1/6] Repository update skipped (set LINGTU_DEPLOY_UPDATE=1 to enable)."
    echo "  revision: $(git log --oneline -1 2>/dev/null || echo unknown)"
fi

echo "[2/6] Building dashboard when Node.js is available..."
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

echo "[3/6] Building production navigation and driver plus maps..."
if is_lite_profile; then
    echo "  skipped: Lite profile uses the local compatibility driver"
else
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
fi

echo "[4/6] Stopping previous LingTu runtime if present..."
mkdir -p "${REPO}/.lingtu"
OLD_PID=""
if [ -f "${REPO}/.lingtu/run.json" ]; then
    OLD_PID="$("${PYTHON_BIN}" - <<'PY' 2>/dev/null || true
import json
from pathlib import Path
path = Path(".lingtu/run.json")
print(json.loads(path.read_text()).get("pid", ""))
PY
)"
fi
if [ -n "${OLD_PID}" ] && kill -0 "${OLD_PID}" 2>/dev/null; then
    kill "${OLD_PID}" 2>/dev/null || true
    sleep 3
    kill -0 "${OLD_PID}" 2>/dev/null && kill -9 "${OLD_PID}" 2>/dev/null || true
    echo "  stopped previous process: ${OLD_PID}"
else
    echo "  no previous process found"
fi
rm -f "${REPO}/.lingtu/run.json" "${REPO}/.lingtu/run.pid"

echo "[5/6] Starting LingTu profile..."
set +u
[ -f "${HOME}/.bashrc" ] && . "${HOME}/.bashrc" || true
ROS2_ENV="${LINGTU_ROS2_ENV:-${REPO}/scripts/deploy/thunder/ros2-env.sh}"
RUNTIME_ENV="${LINGTU_RUNTIME_ENV:-${REPO}/scripts/deploy/thunder/runtime-env.sh}"
SOURCE_ROS2="${LINGTU_DEPLOY_SOURCE_ROS2:-auto}"
if [ "${SOURCE_ROS2}" = "auto" ]; then
    case "${PROFILE}" in
        ros2|sim_ros2|*-ros2|ros-compat|legacy)
            SOURCE_ROS2=1
            ;;
        *)
            SOURCE_ROS2=0
            ;;
    esac
fi
export LINGTU_PROFILE="${PROFILE}"
case "${PROFILE}" in
    lite|thunder-lite|basic|thunder-basic)
        : "${LINGTU_ENDPOINT:=thunder_lite}"
        : "${LINGTU_ENDPOINT_TRANSPORT:=local}"
        : "${LINGTU_MODULE_TRANSPORT:=local}"
        : "${LINGTU_SIMULATION_ONLY:=1}"
        ;;
    *)
        : "${LINGTU_ENDPOINT:=thunder_field}"
        : "${LINGTU_ENDPOINT_TRANSPORT:=dds}"
        : "${LINGTU_MODULE_TRANSPORT:=local}"
        : "${LINGTU_SIMULATION_ONLY:=0}"
        ;;
esac
[ -f "${RUNTIME_ENV}" ] && . "${RUNTIME_ENV}" || true
if [ "${SOURCE_ROS2}" = "1" ] && [ -f "${ROS2_ENV}" ]; then
    . "${ROS2_ENV}" || true
fi
set -u

nohup "${PYTHON_BIN}" lingtu.py "${PROFILE}" --daemon > "${LOG}" 2>&1 &

echo "  waiting for runtime state..."
for _ in $(seq 1 "${STARTUP_TIMEOUT}"); do
    [ -f "${REPO}/.lingtu/run.json" ] && break
    sleep 1
done

echo "[6/6] Verifying runtime..."
if [ ! -f "${REPO}/.lingtu/run.json" ]; then
    echo "  failed: .lingtu/run.json was not created"
    tail -30 "${LOG}" || true
    exit 1
fi

PID="$("${PYTHON_BIN}" - <<'PY' 2>/dev/null || true
import json
from pathlib import Path
data = json.loads(Path(".lingtu/run.json").read_text())
print(data.get("pid", ""))
PY
)"
RUN_PROFILE="$("${PYTHON_BIN}" - <<'PY' 2>/dev/null || true
import json
from pathlib import Path
data = json.loads(Path(".lingtu/run.json").read_text())
print(data.get("profile", "?"))
PY
)"
RUN_LOG_DIR="$("${PYTHON_BIN}" - <<'PY' 2>/dev/null || true
import json
from pathlib import Path
data = json.loads(Path(".lingtu/run.json").read_text())
print(data.get("log_dir", "?"))
PY
)"

if [ -n "${PID}" ] && kill -0 "${PID}" 2>/dev/null; then
    HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "  running: pid=${PID} profile=${RUN_PROFILE}"
    echo "  gateway: http://${HOST_IP:-127.0.0.1}:5050"
    echo "  logs:    ${RUN_LOG_DIR}"
else
    echo "  failed: process is not running"
    tail -30 "${LOG}" || true
    exit 1
fi

echo ""
echo "Thunder deployment complete."
