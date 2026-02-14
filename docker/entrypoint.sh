#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# 3d_NAV Container Entrypoint
#
# 职责:
#   1. Source ROS2 环境
#   2. 配置 LiDAR 网络 (如有权限)
#   3. 创建运行时目录
#   4. 启动 supervisord 或用户指定的命令
# ═══════════════════════════════════════════════════════════════
set -e

# ── 1. Source ROS2 ──
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

# Source 工作区 (如果存在)
if [[ -f /opt/nav/install/setup.bash ]]; then
  source /opt/nav/install/setup.bash
elif [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi

# ── 2. LiDAR 网络配置 (需要 NET_ADMIN 能力) ──
if [[ "${SETUP_LIDAR_NET:-false}" == "true" ]]; then
  LIDAR_IF="${LIDAR_INTERFACE:-eth0}"
  LIDAR_IP="${LIDAR_HOST_IP:-192.168.1.5}"

  echo "[entrypoint] Configuring LiDAR network: ${LIDAR_IF} → ${LIDAR_IP}/24"

  if ip link show "${LIDAR_IF}" &>/dev/null; then
    ip addr add "${LIDAR_IP}/24" dev "${LIDAR_IF}" 2>/dev/null || \
      echo "[entrypoint] WARN: ${LIDAR_IP} already assigned or permission denied"
    ip link set "${LIDAR_IF}" up
  else
    echo "[entrypoint] WARN: Interface ${LIDAR_IF} not found, skipping LiDAR net setup"
  fi
fi

# ── 3. 确保运行时目录存在 ──
mkdir -p /opt/robot/logs \
         /opt/robot/flight_records \
         /opt/robot/ota \
         /opt/robot/bags \
         "${NAV_MAP_DIR:-/opt/nav/maps}" 2>/dev/null || true

# ── 4. 加载 planning 环境变量 (如果存在) ──
if [[ -f /etc/nav/planning.env ]]; then
  echo "[entrypoint] Loading /etc/nav/planning.env"
  set -a
  source /etc/nav/planning.env
  set +a
fi

# ── 5. DDS 配置: 禁用共享内存 (容器环境下不可靠) ──
if [[ ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE:-/dev/null}" ]]; then
  mkdir -p /opt/nav/config
  cat > /opt/nav/config/fastdds_no_shm.xml <<'DDSXML'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>udp_transport</transport_id>
        </userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
DDSXML
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/nav/config/fastdds_no_shm.xml
fi

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}, RMW=${RMW_IMPLEMENTATION}"
echo "[entrypoint] Executing: $@"

# ── 6. 执行传入的命令 ──
exec "$@"
