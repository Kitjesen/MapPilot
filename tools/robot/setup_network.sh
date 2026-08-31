#!/bin/bash
#
# 导航系统网络配置脚本
# 功能: 自动配置 LiDAR 以太网接口、Orbbec udev 规则
# 用法:
#   sudo bash tools/robot/setup_network.sh              # 交互式配置（临时生效）
#   sudo bash tools/robot/setup_network.sh --permanent  # 写入 netplan + systemd（重启持久）
#   sudo bash tools/robot/setup_network.sh --status     # 查看当前状态
#   sudo bash tools/robot/setup_network.sh --auto       # 非交互自动配置（临时）
#
# 迁移到新机器时:
#   1. 将 LiDAR 网线插入任意以太网口
#   2. 运行 sudo bash tools/robot/setup_network.sh --permanent
#   3. 完成
#

set -euo pipefail

# ======================== 可修改参数 ========================
LIDAR_IP="192.168.1.178"        # Livox MID360 LiDAR 地址
HOST_IP="192.168.1.5"           # 本机在 LiDAR 子网的地址
SUBNET_MASK="24"                # 子网掩码位数
LIDAR_SUBNET="192.168.1.0/24"  # LiDAR 子网

# Livox 通信端口 (用于防火墙规则)
LIVOX_UDP_PORTS="56000:56501"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORBBEC_UDEV_RULE="${SCRIPT_DIR}/../../scripts/deploy/99-lingtu-orbbec-gemini335.rules"
# ============================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "${CYAN}[STEP]${NC} $*"; }

# ==================== 检测 LiDAR 接口 ====================

detect_lidar_interface() {
    # 策略: 遍历所有以太网接口，找到有物理链路的那个
    local candidates=()
    local link_up_ifaces=()

    for iface in $(ls /sys/class/net/ | grep -E '^eth|^en'); do
        # 跳过虚拟接口
        [[ -d "/sys/class/net/$iface/device" ]] || continue
        candidates+=("$iface")

        # 检查物理链路 (ethtool 更准确)
        if ethtool "$iface" 2>/dev/null | grep -q "Link detected: yes"; then
            link_up_ifaces+=("$iface")
        fi
    done

    if [[ ${#candidates[@]} -eq 0 ]]; then
        log_error "未找到任何以太网接口"
        return 1
    fi

    if [[ ${#link_up_ifaces[@]} -eq 0 ]]; then
        log_warn "没有检测到有物理连接的以太网口"
        log_warn "可用接口: ${candidates[*]}"
        # 如果有配置了 LiDAR 子网 IP 的接口，优先选它
        for iface in "${candidates[@]}"; do
            if ip -4 addr show dev "$iface" 2>/dev/null | grep -q "${HOST_IP}"; then
                echo "$iface"
                return 0
            fi
        done
        # 都没有，返回第一个
        echo "${candidates[0]}"
        return 0
    fi

    if [[ ${#link_up_ifaces[@]} -eq 1 ]]; then
        echo "${link_up_ifaces[0]}"
        return 0
    fi

    # 多个有链路的接口，找已有 LiDAR 子网 IP 的
    for iface in "${link_up_ifaces[@]}"; do
        if ip -4 addr show dev "$iface" 2>/dev/null | grep -q "${HOST_IP}"; then
            echo "$iface"
            return 0
        fi
    done

    # 尝试 ping LiDAR 来确认
    for iface in "${link_up_ifaces[@]}"; do
        # 临时加 IP 测试
        ip addr add "${HOST_IP}/${SUBNET_MASK}" dev "$iface" 2>/dev/null || true
        ip link set "$iface" up 2>/dev/null
        if timeout 2 ping -c 1 -I "$iface" "$LIDAR_IP" &>/dev/null; then
            echo "$iface"
            return 0
        fi
        ip addr del "${HOST_IP}/${SUBNET_MASK}" dev "$iface" 2>/dev/null || true
    done

    # 默认返回第一个有链路的
    echo "${link_up_ifaces[0]}"
    return 0
}

# ==================== 配置接口 ====================

configure_interface() {
    local iface="$1"

    log_step "配置接口 ${iface} ..."

    # 1. 让 NetworkManager 不管理此接口
    if command -v nmcli &>/dev/null; then
        nmcli device set "$iface" managed no 2>/dev/null || true
        log_info "已禁止 NetworkManager 管理 ${iface}"
    fi

    # 2. 清除该接口上已有的 LiDAR 子网 IP (避免重复)
    while ip -4 addr show dev "$iface" 2>/dev/null | grep -q "inet.*${HOST_IP}"; do
        ip addr del "${HOST_IP}/${SUBNET_MASK}" dev "$iface" 2>/dev/null || break
    done

    # 3. 同时清除其他接口上的冲突 IP
    for other_iface in $(ls /sys/class/net/ | grep -E '^eth|^en'); do
        [[ "$other_iface" == "$iface" ]] && continue
        if ip -4 addr show dev "$other_iface" 2>/dev/null | grep -q "inet.*${HOST_IP}"; then
            ip addr del "${HOST_IP}/${SUBNET_MASK}" dev "$other_iface" 2>/dev/null || true
            log_info "从 ${other_iface} 移除冲突 IP ${HOST_IP}"
        fi
    done

    # 4. 添加 IP
    ip addr add "${HOST_IP}/${SUBNET_MASK}" dev "$iface" 2>/dev/null || true

    # 5. 确保接口 UP (down/up 刷新链路状态)
    ip link set "$iface" down
    sleep 1
    ip link set "$iface" up
    sleep 2

    # 6. 验证
    local state
    state=$(cat "/sys/class/net/${iface}/operstate" 2>/dev/null || echo "unknown")
    local has_ip
    has_ip=$(ip -4 addr show dev "$iface" | grep -c "${HOST_IP}" || true)

    if [[ "$state" == "up" ]] && [[ "$has_ip" -gt 0 ]]; then
        log_info "${iface}: UP, IP=${HOST_IP}/${SUBNET_MASK}"
        return 0
    else
        log_warn "${iface}: state=${state}, has_ip=${has_ip}"
        # 链路检测可能延迟，用 ethtool 再查
        if ethtool "$iface" 2>/dev/null | grep -q "Link detected: yes"; then
            log_info "${iface}: ethtool 确认链路正常（内核状态可能延迟更新）"
            return 0
        fi
        log_error "${iface}: 链路异常，请检查网线连接"
        return 1
    fi
}

# ==================== 验证 LiDAR 通信 ====================

verify_lidar() {
    local iface="$1"

    log_step "验证 LiDAR 通信 ..."

    # 用 tcpdump 检测 LiDAR 的 UDP 包 (5秒超时)
    local pkt_count
    pkt_count=$(timeout 5 tcpdump -i "$iface" -c 1 "host ${LIDAR_IP}" 2>/dev/null | grep -c "UDP" || true)

    if [[ "$pkt_count" -gt 0 ]]; then
        log_info "LiDAR (${LIDAR_IP}) 通信正常"
        return 0
    fi

    # 没抓到包，尝试 ARP 检查
    if timeout 3 bash -c "echo > /dev/udp/${LIDAR_IP}/56100" 2>/dev/null; then
        log_info "LiDAR 端口可达"
        return 0
    fi

    log_warn "未检测到 LiDAR 数据包（LiDAR 可能尚未启动或网线未连接到 ${iface}）"
    return 1
}

# ==================== Orbbec udev 规则 ====================

install_orbbec_udev() {
    log_step "安装 Orbbec Gemini 335 udev 规则 ..."
    install -m 0644 "$ORBBEC_UDEV_RULE" /etc/udev/rules.d/99-lingtu-orbbec-gemini335.rules
    udevadm control --reload-rules
    udevadm trigger
    log_info "Orbbec udev 规则安装完成"
}

# ==================== 永久化配置 ====================

make_permanent() {
    local iface="$1"

    log_step "写入永久配置 ..."

    # 1. 修改 netplan: 将 LiDAR 子网 IP 绑定到正确的接口
    local netplan_file
    netplan_file=$(ls /etc/netplan/*.yaml 2>/dev/null | head -1)

    if [[ -n "$netplan_file" ]]; then
        local mac
        mac=$(cat "/sys/class/net/${iface}/address" 2>/dev/null)

        if [[ -n "$mac" ]]; then
            # 备份
            cp "$netplan_file" "${netplan_file}.bak.$(date +%Y%m%d%H%M%S)"
            log_info "已备份 netplan: ${netplan_file}.bak.*"

            # 生成新的 netplan
            cat > "$netplan_file" << NETPLAN_EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    ${iface}:
      addresses:
      - "${HOST_IP}/${SUBNET_MASK}"
      macaddress: "${mac}"
      dhcp4: false
      dhcp6: false
NETPLAN_EOF

            # 保留其他以太网接口（如果有）
            for other_iface in $(ls /sys/class/net/ | grep -E '^eth|^en'); do
                [[ "$other_iface" == "$iface" ]] && continue
                [[ -d "/sys/class/net/${other_iface}/device" ]] || continue
                local other_mac
                other_mac=$(cat "/sys/class/net/${other_iface}/address" 2>/dev/null)
                [[ -z "$other_mac" ]] && continue
                cat >> "$netplan_file" << OTHER_EOF
    ${other_iface}:
      nameservers:
        addresses:
        - 8.8.8.8
        - 8.8.4.4
      dhcp4: true
      dhcp6: true
      macaddress: "${other_mac}"
OTHER_EOF
            done

            log_info "netplan 已更新: ${iface} = ${HOST_IP}/${SUBNET_MASK}"
            netplan generate 2>/dev/null || true
        fi
    fi

    # 2. 创建 systemd 服务确保开机后接口正确初始化
    local service_file="/etc/systemd/system/nav-lidar-network.service"
    local script_path
    script_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

    cat > "$service_file" << SERVICE_EOF
[Unit]
Description=Navigation LiDAR Network Setup
After=NetworkManager.service NetworkManager-wait-online.service network-online.target
Wants=NetworkManager-wait-online.service network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\
  set -e; \
  IFACE="${iface}"; \
  for _ in \$(seq 1 30); do ip link show \$IFACE >/dev/null 2>&1 && break; sleep 1; done; \
  ip link show \$IFACE >/dev/null; \
  for _ in \$(seq 1 30); do ip -4 addr show dev \$IFACE | grep -q "inet " && break; sleep 1; done; \
  ip link set \$IFACE up; \
  ip addr replace ${HOST_IP}/${SUBNET_MASK} dev \$IFACE; \
  ip route replace ${LIDAR_SUBNET} dev \$IFACE src ${HOST_IP}; \
  echo "nav-lidar-network: \$IFACE configured with ${HOST_IP}/${SUBNET_MASK} and ${LIDAR_SUBNET}"'

[Install]
WantedBy=multi-user.target
SERVICE_EOF

    systemctl daemon-reload
    systemctl enable nav-lidar-network.service
    log_info "systemd 服务 nav-lidar-network 已创建并启用"

    # 3. 安装 Orbbec udev 规则
    install_orbbec_udev

    log_info "永久配置完成！重启后自动生效"
}

# ==================== 状态显示 ====================

show_status() {
    echo ""
    echo "======================== 网络状态 ========================"
    echo ""

    for iface in $(ls /sys/class/net/ | grep -E '^eth|^en'); do
        [[ -d "/sys/class/net/${iface}/device" ]] || continue
        local state link ip_info mac
        state=$(cat "/sys/class/net/${iface}/operstate" 2>/dev/null || echo "?")
        link=$(ethtool "$iface" 2>/dev/null | grep "Link detected" | awk '{print $NF}')
        ip_info=$(ip -4 addr show dev "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' || echo "无")
        mac=$(cat "/sys/class/net/${iface}/address" 2>/dev/null || echo "?")
        [[ -z "$ip_info" ]] && ip_info="无"

        local status_icon="❌"
        [[ "$link" == "yes" ]] && status_icon="✅"

        printf "  ${status_icon} %-6s  state=%-6s  link=%-4s  IP=%-18s  MAC=%s\n" \
            "$iface" "$state" "${link:-?}" "$ip_info" "$mac"
    done

    echo ""
    echo "  LiDAR 目标: ${LIDAR_IP}"
    echo "  期望 Host IP: ${HOST_IP}/${SUBNET_MASK}"
    echo ""

    # TF 检查
    if command -v ros2 &>/dev/null; then
        echo "======================== ROS2 状态 ========================"
        echo ""
        local node_count
        node_count=$(ros2 node list 2>/dev/null | wc -l || echo "0")
        echo "  活跃节点数: ${node_count}"

        # 检查关键服务
        for srv in "nav-lidar-network"; do
            if systemctl is-enabled "$srv" 2>/dev/null | grep -q "enabled"; then
                echo "  🔧 systemd ${srv}: 已启用"
            fi
        done
        echo ""
    fi

    # Orbbec udev
    if [[ -f /etc/udev/rules.d/99-lingtu-orbbec-gemini335.rules ]]; then
        echo "  📷 Orbbec udev 规则: 已安装"
    else
        echo "  📷 Orbbec udev 规则: 未安装"
    fi

    echo ""
    echo "==========================================================="
}

# ==================== 主流程 ====================

main() {
    local mode="${1:-}"

    if [[ "$mode" == "--status" ]]; then
        show_status
        exit 0
    fi

    # 需要 root
    if [[ $EUID -ne 0 ]]; then
        log_error "请使用 sudo 运行此脚本"
        exit 1
    fi

    echo ""
    echo "============================================"
    echo "   导航系统网络配置"
    echo "   LiDAR: ${LIDAR_IP}  Host: ${HOST_IP}"
    echo "============================================"
    echo ""

    # 检测接口
    log_step "检测 LiDAR 以太网接口 ..."
    local iface
    iface=$(detect_lidar_interface)
    log_info "选择接口: ${iface}"

    # 非自动模式下确认
    if [[ "$mode" != "--auto" ]] && [[ "$mode" != "--permanent" ]]; then
        echo ""
        read -p "确认使用 ${iface} 作为 LiDAR 接口? [Y/n] " confirm
        if [[ "${confirm,,}" == "n" ]]; then
            echo "可用接口:"
            ls /sys/class/net/ | grep -E '^eth|^en'
            read -p "请输入接口名: " iface
        fi
    fi

    # 配置接口
    configure_interface "$iface"

    # 验证
    verify_lidar "$iface" || true

    # 永久化
    if [[ "$mode" == "--permanent" ]]; then
        make_permanent "$iface"
    else
        echo ""
        log_info "当前为临时配置（重启后失效）"
        log_info "如需永久生效，请运行: sudo bash $0 --permanent"
    fi

    echo ""
    show_status
    log_info "配置完成!"
}

main "$@"
