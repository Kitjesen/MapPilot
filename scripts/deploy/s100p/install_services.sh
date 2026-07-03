#!/bin/bash
# Install legacy ROS2-compatible Thunder field SLAM systemd services.
#
# This legacy directory name is kept for field compatibility while the
# product-facing deployment entrypoints move to Thunder naming.
# Native DDS services are installed from scripts/deploy/thunder/install_services.sh.
# Usage: scp scripts/deploy/s100p/*.service sunrise@192.168.66.13:/tmp/
#        ssh sunrise@192.168.66.13 'bash /tmp/install_services.sh'
# Or:    ssh sunrise@192.168.66.13 'bash -s' < scripts/deploy/s100p/install_services.sh

set -e

SERVICES=(lidar slam slam_pgo localizer genz_icp hba super_lio super_lio_relocation)
SRC_DIR="${1:-/tmp}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
ROS2_ENV_SRC="${LINGTU_ROS2_ENV_SRC:-}"
if [ -z "${ROS2_ENV_SRC}" ]; then
    if [ -f "${SCRIPT_DIR}/../thunder/ros2-env.sh" ]; then
        ROS2_ENV_SRC="${SCRIPT_DIR}/../thunder/ros2-env.sh"
    elif [ -f "${SRC_DIR}/ros2-env.sh" ]; then
        ROS2_ENV_SRC="${SRC_DIR}/ros2-env.sh"
    fi
fi

echo "=== Installing legacy ROS2-compatible Thunder SLAM services ==="
if [ -n "${ROS2_ENV_SRC}" ] && [ -f "${ROS2_ENV_SRC}" ]; then
    sudo mkdir -p "${CONFIG_DIR}"
    sudo cp "${ROS2_ENV_SRC}" "${CONFIG_DIR}/ros2-env.sh"
    sudo chmod 0644 "${CONFIG_DIR}/ros2-env.sh"
    echo "  Installed: ${CONFIG_DIR}/ros2-env.sh"
fi
for svc in "${SERVICES[@]}"; do
    if [ -f "${SRC_DIR}/${svc}.service" ]; then
        sudo cp "${SRC_DIR}/${svc}.service" /etc/systemd/system/
        echo "  Installed: ${svc}.service"
        if [ "$svc" = "genz_icp" ]; then
            sudo ln -sf /etc/systemd/system/genz_icp.service /etc/systemd/system/robot-genz-icp.service
            echo "  Alias: robot-genz-icp.service -> genz_icp.service"
        elif [ "$svc" = "hba" ]; then
            sudo ln -sf /etc/systemd/system/hba.service /etc/systemd/system/robot-hba.service
            echo "  Alias: robot-hba.service -> hba.service"
        elif [ "$svc" = "super_lio" ]; then
            sudo ln -sf /etc/systemd/system/super_lio.service /etc/systemd/system/robot-super-lio.service
            echo "  Alias: robot-super-lio.service -> super_lio.service"
        elif [ "$svc" = "super_lio_relocation" ]; then
            sudo ln -sf /etc/systemd/system/super_lio_relocation.service /etc/systemd/system/robot-super-lio-relocation.service
            sudo ln -sf /etc/systemd/system/super_lio_relocation.service /etc/systemd/system/robot-super-lio-reloc.service
            echo "  Alias: robot-super-lio-relocation.service -> super_lio_relocation.service"
            echo "  Alias: robot-super-lio-reloc.service -> super_lio_relocation.service"
        fi
    else
        echo "  MISSING: ${SRC_DIR}/${svc}.service"
    fi
done

sudo systemctl daemon-reload

if [ "${LINGTU_ENABLE_LEGACY_ROS2_SERVICES:-0}" = "1" ]; then
    for svc in "${SERVICES[@]}"; do
        sudo systemctl enable "${svc}.service" 2>/dev/null
        echo "  Enabled legacy ROS2 compat: ${svc}.service"
    done
else
    echo "  Legacy ROS2 compat services installed but not enabled."
    echo "  Set LINGTU_ENABLE_LEGACY_ROS2_SERVICES=1 to enable them explicitly."
fi

echo ""
echo "=== Done. Quick start: ==="
echo "  Native default: scripts/deploy/thunder/install_services.sh field-cpp"
echo "  Ops default:    lingtu svc restart slam"
echo "  Legacy mapping: sudo systemctl start lidar slam slam_pgo"
echo "  Legacy nav:     sudo systemctl start lidar slam localizer"
echo "  sudo systemctl start robot-genz-icp         # experimental GenZ-ICP odometry"
echo "  sudo systemctl start robot-hba              # offline HBA map optimization service"
echo "  sudo systemctl start lidar super_lio        # experimental Super-LIO mode"
echo "  sudo systemctl start robot-super-lio        # production alias"
echo "  sudo systemctl start robot-super-lio-relocation  # experimental saved-map relocation"
echo "  sudo systemctl status lingtu-slam-dds       # native status"
echo "  journalctl -u lingtu-slam-dds -f            # native logs"
