#!/bin/bash
# ══════════════════════════════════════════════════════════
# 导航系统 systemd 服务安装脚本
# 用法: sudo bash scripts/install_services.sh
#
# 工作原理:
#   1. 创建符号链接 /opt/nav → 工作区根目录
#   2. 将 systemd/*.service 直接复制到 /etc/systemd/system/
#      (.service 文件统一使用 /opt/nav/ 路径, 无需替换)
#   3. 配置 sudoers / 环境文件
#
# 换路径? 只需:
#   sudo ln -sfn /new/path /opt/nav
#   sudo systemctl daemon-reload
# ══════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SYSTEMD_DIR="/etc/systemd/system"
SUDOERS_FILE="/etc/sudoers.d/nav-services"
NAV_LINK="/opt/nav"

# 服务运行用户
SERVICE_USER="${SERVICE_USER:-sunrise}"

echo "═══════════════════════════════════════════"
echo "  导航系统服务安装"
echo "═══════════════════════════════════════════"
echo "  工作区:  $WORKSPACE_DIR"
echo "  链接:    $NAV_LINK → $WORKSPACE_DIR"
echo ""

# ── 1. 创建 /opt/nav 符号链接 ──
echo "[1/5] Creating symlink $NAV_LINK → $WORKSPACE_DIR ..."
ln -sfn "$WORKSPACE_DIR" "$NAV_LINK"
echo "  + $NAV_LINK → $(readlink -f $NAV_LINK)"

# ── 2. 设置脚本可执行权限 ──
echo "[2/5] Setting script permissions..."
chmod +x "$WORKSPACE_DIR"/scripts/services/*.sh

# ── 3. 安装 systemd 服务文件 (直接复制, 无需替换) ──
echo "[3/5] Installing systemd service units..."
SERVICES=(
    nav-lidar
    nav-slam
    nav-autonomy
    nav-planning
    nav-grpc
    ota-daemon
)

for svc in "${SERVICES[@]}"; do
    SRC="$WORKSPACE_DIR/systemd/${svc}.service"
    if [ ! -f "$SRC" ]; then
        echo "  ! ${svc}.service not found, skipping"
        continue
    fi
    cp "$SRC" "$SYSTEMD_DIR/${svc}.service"
    echo "  + ${svc}.service"
done

# ── 4. 创建 sudoers 规则 ──
echo "[4/5] Configuring sudoers for service management..."

SUDOERS_CONTENT="# 允许 ${SERVICE_USER} 无密码管理导航服务 (由 install_services.sh 生成)\n"
for svc in "${SERVICES[@]}"; do
    for action in start stop restart reset-failed; do
        SUDOERS_CONTENT+="${SERVICE_USER} ALL=(root) NOPASSWD: /bin/systemctl ${action} ${svc}.service\n"
    done
done

echo -e "$SUDOERS_CONTENT" > "$SUDOERS_FILE"
chmod 440 "$SUDOERS_FILE"
visudo -cf "$SUDOERS_FILE" && echo "  + sudoers validated" || {
    echo "  ! sudoers validation failed, removing..."
    rm -f "$SUDOERS_FILE"
    exit 1
}

# ── 5. 创建规划环境文件 (可选覆盖) ──
echo "[5/5] Creating planning environment config..."
mkdir -p /etc/nav
if [ ! -f /etc/nav/planning.env ]; then
    cat > /etc/nav/planning.env << 'EOF'
# nav-planning.service 运行时参数
# 取消注释并设置所需值
# NAV_MAP_PATH=/opt/nav/maps/site
# NAV_INIT_X=0.0
# NAV_INIT_Y=0.0
# NAV_INIT_Z=0.0
# NAV_INIT_YAW=0.0
EOF
    echo "  + /etc/nav/planning.env (template)"
else
    echo "  = /etc/nav/planning.env (already exists)"
fi

# ── 安装 logrotate 和 cron 配置 ──
DEPLOY_DIR="${WORKSPACE_DIR}/deploy"
if [[ -d "${DEPLOY_DIR}/logrotate.d" ]]; then
  echo "安装 logrotate 配置..."
  cp "${DEPLOY_DIR}/logrotate.d/"* /etc/logrotate.d/ 2>/dev/null || true
fi
if [[ -d "${DEPLOY_DIR}/cron.d" ]]; then
  echo "安装 cron 清理任务..."
  cp "${DEPLOY_DIR}/cron.d/"* /etc/cron.d/ 2>/dev/null || true
fi

# 创建日志和飞行记录目录
mkdir -p /opt/robot/logs /opt/robot/flight_records
chown -R "${SERVICE_USER}:${SERVICE_USER}" /opt/robot/logs /opt/robot/flight_records

# ── 重新加载 systemd ──
systemctl daemon-reload

echo ""
echo "═══════════════════════════════════════════"
echo "  安装完成!"
echo "═══════════════════════════════════════════"
echo ""
echo "可用服务:"
for svc in "${SERVICES[@]}"; do
    status=$(systemctl is-active "${svc}.service" 2>/dev/null || echo "inactive")
    printf "  %-20s %s\n" "${svc}.service" "$status"
done
echo ""
echo "使用示例:"
echo "  sudo systemctl start nav-lidar nav-slam    # 启动建图"
echo "  systemctl status nav-slam                   # 查看状态"
echo "  journalctl -u nav-slam -f                   # 查看日志"
echo ""
echo "换路径:"
echo "  sudo ln -sfn /new/path $NAV_LINK"
echo "  sudo systemctl daemon-reload"
echo ""
