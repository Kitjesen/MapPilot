#!/bin/bash
# install.sh — 部署 OTA Daemon 到机器人
# 用法: ./install.sh [--host <robot_ip>]
#
# 本地安装:  sudo ./install.sh
# 远程部署:  ./install.sh --host 192.168.1.100

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DAEMON_DIR="$(dirname "$SCRIPT_DIR")"

# 默认路径
INSTALL_BIN="/opt/robot/bin"
INSTALL_CONFIG="/opt/robot/ota"
INSTALL_SERVICE="/etc/systemd/system"

HOST=""
for arg in "$@"; do
  case "$arg" in
    --host) shift; HOST="${1:-}"; shift || true ;;
    --host=*) HOST="${arg#*=}" ;;
  esac
done

# ──────────────── 本地构建 ────────────────
echo "=== Building ota_daemon ==="
BUILD_DIR="${DAEMON_DIR}/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
echo "Build complete: ${BUILD_DIR}/ota_daemon"

# ──────────────── 部署 ────────────────
INSTALL_SCRIPTS="/opt/robot/scripts"

deploy_local() {
  echo "=== Installing locally ==="

  sudo mkdir -p "$INSTALL_BIN" "$INSTALL_CONFIG" "$INSTALL_SCRIPTS"
  sudo cp "${BUILD_DIR}/ota_daemon" "$INSTALL_BIN/"
  sudo chmod +x "${INSTALL_BIN}/ota_daemon"

  # 配置文件 (不覆盖已有)
  if [ ! -f "${INSTALL_CONFIG}/ota_daemon.yaml" ]; then
    sudo cp "${DAEMON_DIR}/config/ota_daemon.yaml" "$INSTALL_CONFIG/"
    echo "Installed config: ${INSTALL_CONFIG}/ota_daemon.yaml"
  else
    echo "Config exists, skipping: ${INSTALL_CONFIG}/ota_daemon.yaml"
  fi

  # 制品路径映射 (不覆盖已有)
  if [ ! -f "${INSTALL_CONFIG}/artifact_paths.yaml" ]; then
    sudo cp "${DAEMON_DIR}/config/artifact_paths.yaml" "$INSTALL_CONFIG/"
    echo "Installed config: ${INSTALL_CONFIG}/artifact_paths.yaml"
  else
    echo "Config exists, skipping: ${INSTALL_CONFIG}/artifact_paths.yaml"
  fi

  # Dog Board model reload script
  sudo cp "${DAEMON_DIR}/deploy/reload_dog_model.sh" "$INSTALL_SCRIPTS/"
  sudo chmod +x "${INSTALL_SCRIPTS}/reload_dog_model.sh"
  echo "Installed script: ${INSTALL_SCRIPTS}/reload_dog_model.sh"

  # systemd 服务
  sudo cp "${DAEMON_DIR}/deploy/ota_daemon.service" "$INSTALL_SERVICE/"
  sudo systemctl daemon-reload
  sudo systemctl enable ota_daemon.service

  echo ""
  echo "=== OTA Daemon installed ==="
  echo "  Binary:  ${INSTALL_BIN}/ota_daemon"
  echo "  Config:  ${INSTALL_CONFIG}/ota_daemon.yaml"
  echo "  Scripts: ${INSTALL_SCRIPTS}/reload_dog_model.sh"
  echo "  Service: ota_daemon.service"
  echo ""
  echo "Commands:"
  echo "  sudo systemctl start ota_daemon"
  echo "  sudo systemctl status ota_daemon"
  echo "  journalctl -u ota_daemon -f"
}

deploy_remote() {
  echo "=== Deploying to ${HOST} ==="

  ssh "sunrise@${HOST}" "sudo mkdir -p ${INSTALL_BIN} ${INSTALL_CONFIG}"
  scp "${BUILD_DIR}/ota_daemon" "sunrise@${HOST}:/tmp/ota_daemon"
  ssh "sunrise@${HOST}" "sudo mv /tmp/ota_daemon ${INSTALL_BIN}/ && sudo chmod +x ${INSTALL_BIN}/ota_daemon"

  # Config (no overwrite)
  ssh "sunrise@${HOST}" "test -f ${INSTALL_CONFIG}/ota_daemon.yaml" 2>/dev/null || \
    scp "${DAEMON_DIR}/config/ota_daemon.yaml" "sunrise@${HOST}:/tmp/ota_daemon.yaml" && \
    ssh "sunrise@${HOST}" "sudo mv /tmp/ota_daemon.yaml ${INSTALL_CONFIG}/"

  # Systemd
  scp "${DAEMON_DIR}/deploy/ota_daemon.service" "sunrise@${HOST}:/tmp/ota_daemon.service"
  ssh "sunrise@${HOST}" "sudo mv /tmp/ota_daemon.service ${INSTALL_SERVICE}/ && \
    sudo systemctl daemon-reload && \
    sudo systemctl enable ota_daemon.service && \
    sudo systemctl restart ota_daemon.service"

  echo ""
  echo "=== Deployed to ${HOST} ==="
  echo "  Check: ssh sunrise@${HOST} 'systemctl status ota_daemon'"
}

if [ -n "$HOST" ]; then
  deploy_remote
else
  deploy_local
fi
