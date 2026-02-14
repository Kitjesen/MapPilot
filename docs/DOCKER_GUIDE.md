# Docker 容器化部署指南

## 架构概览

```
┌─────────────────────────────────────────────────────────────┐
│  docker-compose.yml                                         │
│                                                             │
│  ┌─────────────── nav-stack ──────────────────────┐         │
│  │  supervisord                                   │         │
│  │  ├── nav-lidar    (Livox MID360)               │         │
│  │  ├── nav-slam     (Fast-LIO2)                  │         │
│  │  ├── nav-autonomy (局部规划 + 地形分析)          │  host   │
│  │  ├── nav-planning (定位 + PCT全局规划)           │  network│
│  │  ├── nav-grpc     (gRPC Gateway :50051)        │  mode   │
│  │  └── ota-daemon   (OTA 服务 :50052)            │         │
│  └────────────────────────────────────────────────┘         │
│                                                             │
│  Volumes: nav-logs, nav-flight-records, nav-ota, maps/      │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 前置条件

- Docker Engine 24+ 
- Docker Compose v2+
- 宿主机已配置 LiDAR 网口 (或在容器内通过 `NET_ADMIN` 配置)

### 1. 生产部署

```bash
# 构建镜像
docker compose build

# 配置环境变量
cp docker/.env.example .env
# 编辑 .env, 设置 LIDAR_INTERFACE, NAV_MAP_PATH 等

# 启动
docker compose up -d

# 查看日志
docker compose logs -f nav-stack

# 查看进程状态
docker exec nav-stack supervisorctl status

# 停止
docker compose down
```

### 2. 建图模式

```bash
# 关闭规划模块, 仅启动 LiDAR + SLAM + gRPC
ENABLE_PLANNING=false ENABLE_AUTONOMY=false docker compose up -d
```

### 3. 仅 gRPC 网关 (模拟/调试)

```bash
ENABLE_LIDAR=false ENABLE_SLAM=false \
ENABLE_PLANNING=false ENABLE_AUTONOMY=false \
docker compose up -d
```

## 开发环境

### 进入开发容器

```bash
# 构建开发镜像 (包含 gdb, valgrind, rviz2 等)
docker compose -f docker-compose.yml -f docker-compose.dev.yml build dev

# 启动交互式开发容器
docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm dev

# 容器内:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
colcon test --packages-select remote_monitoring
source install/setup.bash
ros2 launch launch/subsystems/grpc.launch.py
```

### 特性

| 功能 | 说明 |
|------|------|
| 源码挂载 | 宿主机编辑, 容器内编译 |
| ccache | 增量编译缓存, 首次 ~15min, 后续 ~2min |
| UID 映射 | 文件权限与宿主机一致 |
| gdb | `SYS_PTRACE` 已启用 |
| rviz2 | X11 转发可用 (需 `xhost +local:docker`) |

### 运行测试

```bash
# 方式一: 在开发容器内
colcon test --packages-select remote_monitoring --return-code-on-test-failure
colcon test-result --verbose

# 方式二: 独立测试容器
docker compose -f docker-compose.yml -f docker-compose.dev.yml run --rm test
```

## 镜像体系

| 镜像 | 用途 | 大小 (约) |
|------|------|----------|
| `3dnav:latest` | 生产运行时 | ~1.5 GB |
| `3dnav-dev:latest` | 开发环境 | ~4 GB |
| `3dnav-build:latest` | CI 编译 | ~3 GB |
| `3dnav-test:latest` | CI 测试 | ~3 GB |

## 硬件设备

### LiDAR (Livox MID360)

使用 `host` 网络模式, 容器直接访问宿主机网口:

```yaml
# docker-compose.yml
network_mode: host
environment:
  SETUP_LIDAR_NET: "true"
  LIDAR_INTERFACE: eth0       # 连接 LiDAR 的网口
  LIDAR_HOST_IP: 192.168.1.5  # 宿主机在 LiDAR 子网的 IP
cap_add:
  - NET_ADMIN                  # ip addr add 权限
```

### 串口设备

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0  # 机器人驱动
  - /dev/ttyACM0:/dev/ttyACM0  # pathFollower
```

如果设备号不固定, 建议使用 udev 规则创建稳定的符号链接:

```bash
# /etc/udev/rules.d/99-robot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", SYMLINK+="robot_driver"
```

### USB 摄像头 (Orbbec)

```yaml
devices:
  - /dev/bus/usb:/dev/bus/usb
privileged: true  # Orbbec 需要完整 USB 访问
```

## 数据持久化

| Volume | 宿主机路径 | 用途 |
|--------|-----------|------|
| `nav-logs` | Docker managed | 事件日志、OTA 日志 |
| `nav-flight-records` | Docker managed | FlightRecorder 转储 |
| `nav-ota` | Docker managed | OTA 清单和备份 |
| `nav-bags` | Docker managed | ROS2 bag 录制 |
| `./maps` | 项目目录 | 地图文件 (PCD/PGM) |

```bash
# 查看 volume 位置
docker volume inspect 3d_nav_nav-logs

# 备份日志
docker cp nav-stack:/opt/robot/logs ./backup-logs/

# 导出 FlightRecorder 数据
docker cp nav-stack:/opt/robot/flight_records ./fdr-dumps/
```

## 网络模式

使用 `network_mode: host` 的原因:

1. **LiDAR UDP**: Livox MID360 使用 UDP 56000-56501, 需要直接访问 192.168.1.x 网段
2. **DDS 发现**: ROS2 FastDDS 使用多播 UDP 进行节点发现, host 网络最简单
3. **狗板通信**: 192.168.123.161 是内部网段, 需要宿主机路由

如果不需要硬件 (纯模拟), 可以切换到 bridge 网络:

```yaml
services:
  nav-stack:
    # network_mode: host  # 注释掉
    ports:
      - "50051:50051"
      - "50052:50052"
```

## 进程管理

容器内使用 supervisord 替代 systemd:

```bash
# 查看所有进程状态
docker exec nav-stack supervisorctl status

# 重启单个进程
docker exec nav-stack supervisorctl restart nav-slam

# 停止规划模块
docker exec nav-stack supervisorctl stop navigation:nav-planning

# 查看进程日志
docker exec nav-stack tail -f /opt/robot/logs/nav-grpc.log
```

## CI/CD 集成

```yaml
# .github/workflows/docker-build.yml 示例
- name: Build & Test
  run: |
    docker compose -f docker-compose.yml -f docker-compose.dev.yml \
      run --rm build-only

- name: Build production image
  run: |
    docker compose build nav-stack
    docker tag 3dnav:latest ghcr.io/${{ github.repository }}/3dnav:${{ github.sha }}
    docker push ghcr.io/${{ github.repository }}/3dnav:${{ github.sha }}
```

## 故障排查

```bash
# 容器启动失败
docker compose logs nav-stack | head -50

# 检查 ROS 节点
docker exec nav-stack bash -c \
  "source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash && ros2 node list"

# 检查 DDS 连通性
docker exec nav-stack bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"

# 进入容器调试
docker exec -it nav-stack bash

# 健康检查日志
docker inspect --format='{{json .State.Health}}' nav-stack | python3 -m json.tool
```
