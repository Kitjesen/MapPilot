#!/bin/bash
# Install legacy ROS2-compatible LingTu service files.
# Run from repo root: bash docs/04-deployment/services/install.sh
set -e

if [ "${LINGTU_ENABLE_LEGACY_ROS2_SERVICES:-0}" != "1" ]; then
  cat >&2 <<'EOF'
ERROR: docs/04-deployment/services/install.sh installs the legacy ROS2 robot-* stack.

The current product service installer is:
  bash scripts/deploy/thunder/install_services.sh field-cpp

To intentionally install these legacy compatibility units, rerun with:
  LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1 bash docs/04-deployment/services/install.sh
EOF
  exit 2
fi

SERVICES_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSTEMD_DIR="/etc/systemd/system"
LINGTU_CONFIG="/opt/lingtu/config"

echo "=== Installing LingTu service files ==="

# 1. Create config dir
sudo mkdir -p "$LINGTU_CONFIG"

# 2. Install ros2-env.sh
sudo cp "$SERVICES_DIR/ros2-env.sh" "$LINGTU_CONFIG/ros2-env.sh"
sudo chmod +x "$LINGTU_CONFIG/ros2-env.sh"
echo "[ok] $LINGTU_CONFIG/ros2-env.sh"

# 3. Install service and target files
for svc in robot-lidar robot-camera robot-brainstem robot-fastlio2 robot-localizer lingtu; do
  sudo cp "$SERVICES_DIR/${svc}.service" "$SYSTEMD_DIR/${svc}.service"
  echo "[ok] $SYSTEMD_DIR/${svc}.service"
done
sudo cp "$SERVICES_DIR/lingtu.target" "$SYSTEMD_DIR/lingtu.target"
echo "[ok] $SYSTEMD_DIR/lingtu.target"

# 4. Reload systemd
sudo systemctl daemon-reload
echo "[ok] systemd daemon-reload"

# 5. Retire legacy camera units. Running both a legacy Orbbec unit and
# robot-camera.service can create duplicate Orbbec node instances or image
# publishers. One /camera/camera plus one /camera/camera_container is normal.
for legacy in camera.service orbbec-camera.service; do
  sudo systemctl stop "$legacy" 2>/dev/null || true
  sudo systemctl disable "$legacy" 2>/dev/null || true
  sudo systemctl mask "$legacy" 2>/dev/null || true
done
echo "[ok] legacy camera units retired"

# 6. Enable services and the umbrella target
sudo systemctl enable robot-lidar robot-camera robot-brainstem robot-fastlio2 robot-localizer lingtu lingtu.target
echo "[ok] services and lingtu.target enabled"

echo ""
echo "=== Done. Verify with: ==="
echo "  systemctl list-unit-files | grep -E 'robot-|lingtu'"
echo ""
echo "=== To start everything now: ==="
echo "  sudo systemctl start lingtu.target"
