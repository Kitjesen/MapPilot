#!/bin/bash
# ============================================================================
# setup_robot.sh — 首次在机器人端配置 OTA 基础设施
#
# 此脚本在机器人上运行一次，创建所需的目录结构和 systemd 服务。
#
# 用法 (在机器人上):
#   sudo bash setup_robot.sh
#
# 或从开发机远程执行:
#   scp scripts/ota/setup_robot.sh sunrise@robot:/tmp/
#   ssh sunrise@robot 'sudo bash /tmp/setup_robot.sh'
# ============================================================================
set -euo pipefail

if [ "${LINGTU_ENABLE_LEGACY_ROS2_OTA:-0}" != "1" ]; then
    cat >&2 <<'EOF'
ERROR: scripts/ota/setup_robot.sh is a legacy ROS2/colcon OTA installer.

The product deployment path is native DDS:
  bash scripts/deploy/thunder/install_services.sh field-cpp
  bash scripts/deploy/cut_release.sh vX.Y.Z

To intentionally install the old ROS2 OTA service, rerun with:
  LINGTU_ENABLE_LEGACY_ROS2_OTA=1 sudo -E bash scripts/ota/setup_robot.sh
EOF
    exit 2
fi

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  导航 OTA 基础设施配置${NC}"
echo -e "${BLUE}============================================${NC}"

# ---- 创建目录结构 ----
echo -e "${GREEN}[1/4] 创建目录结构...${NC}"

mkdir -p /opt/robot/ota/backup
mkdir -p /opt/robot/navigation
mkdir -p /opt/robot/models
mkdir -p /opt/robot/maps
mkdir -p /opt/robot/config

# 初始化 installed_manifest.json
if [ ! -f /opt/robot/ota/installed_manifest.json ]; then
    echo '{"artifacts": {}}' > /opt/robot/ota/installed_manifest.json
fi

# 设置权限 (sunrise 用户可写)
chown -R sunrise:sunrise /opt/robot/ 2>/dev/null || true

echo -e "  /opt/robot/ota/           — OTA 管理目录"
echo -e "  /opt/robot/navigation/    — 导航包安装目录"
echo -e "  /opt/robot/models/        — 模型文件"
echo -e "  /opt/robot/maps/          — 地图文件"
echo -e "  /opt/robot/config/        — 配置文件"

# ---- 安装 systemd 服务 ----
echo -e "${GREEN}[2/4] 安装 systemd 服务...${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 复制服务文件
if [ -f "$SCRIPT_DIR/navigation.service" ]; then
    cp "$SCRIPT_DIR/navigation.service" /etc/systemd/system/navigation.service
elif [ -f "/tmp/navigation.service" ]; then
    cp "/tmp/navigation.service" /etc/systemd/system/navigation.service
else
    cat > /etc/systemd/system/navigation.service << 'SVCEOF'
[Unit]
Description=Navigation System (ROS 2)
After=network-online.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=/opt/robot/navigation/current
ExecStart=/opt/robot/navigation/start_nav.sh
ExecStop=/bin/kill -SIGINT $MAINPID
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=navigation

[Install]
WantedBy=multi-user.target
SVCEOF
fi

# 复制启动脚本
if [ -f "$SCRIPT_DIR/start_nav.sh" ]; then
    cp "$SCRIPT_DIR/start_nav.sh" /opt/robot/navigation/start_nav.sh
fi
chmod +x /opt/robot/navigation/start_nav.sh 2>/dev/null || true

systemctl daemon-reload
systemctl enable navigation.service

echo -e "  服务已注册: navigation.service"
echo -e "  自启动: 已启用"

# ---- 部署 OTA 公钥 (如有) ----
echo -e "${GREEN}[3/4] 检查 OTA 公钥...${NC}"

if [ -f /opt/robot/ota/ota_public.pem ]; then
    echo -e "  公钥已存在: /opt/robot/ota/ota_public.pem"
else
    echo -e "${YELLOW}  公钥未部署 (可选: 未签名的 manifest 仍可使用)${NC}"
    echo -e "  部署方法: scp keys/ota_public.pem sunrise@robot:/opt/robot/ota/"
fi

# ---- 验证 ----
echo -e "${GREEN}[4/4] 验证配置...${NC}"

echo -e "  目录结构:"
ls -la /opt/robot/ 2>/dev/null | while read line; do echo "    $line"; done

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}配置完成!${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo -e "  1. 在开发机上构建: ./scripts/ota/build_nav_package.sh"
echo -e "  2. 推送到机器人:   ./scripts/ota/push_to_robot.sh sunrise@<robot_ip>"
echo -e "  3. 管理服务:"
echo -e "     sudo systemctl start navigation    # 启动"
echo -e "     sudo systemctl stop navigation     # 停止"
echo -e "     sudo systemctl status navigation   # 状态"
echo -e "     journalctl -u navigation -f        # 日志"
