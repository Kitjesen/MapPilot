# 现场部署就绪性验证 Runbook

## 1. 概述

本 runbook 覆盖 camera / Livox LiDAR / IMU / SLAM / native nav endpoint /
driver **native DDS 服务**的现场部署验证流程。
适用于 S100P 机器人（aarch64 / Ubuntu）环境，整合以下四个工作流的机器人侧部署与验证步骤：

| 工作流 | 主题 |
|--------|------|
| #16 | Gateway services/status 端点 |
| #17 | Field readiness collector 采样 |
| #18 | Camera service catalog 安装 |
| #19 | Replay backend round-trip DDS 类型契约 |

---

## 2. Camera Service Catalog 安装（工作流 #18）

通过 Thunder service catalog 统一安装 camera native DDS 服务，替代手工复制 systemd unit。
安装器只安装服务；ProductControl 通过 Product switch 激活进程。

```bash
export LINGTU_TARGET_HOST=ROBOT_IP_OR_HOSTNAME
export LINGTU_TARGET_USER=ROBOT_SSH_USER
ssh "${LINGTU_TARGET_USER}@${LINGTU_TARGET_HOST}"
cd /opt/lingtu/current

# 通过 catalog 安装（替代手工复制 service，不直接激活进程）
sudo bash scripts/deploy/thunder/install_catalog_service.sh camera

# 首次激活：切换到声明 camera 进程的 Product
bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]

# 验证服务状态
systemctl status lt-camera.service

# 确认该 Product 的 RunPlan 已激活 camera
systemctl is-active lt-camera.service
# 期望输出：active
```

### 环境变量覆盖机制

camera 服务支持以下环境变量覆盖（在 systemd unit 的 `[Service]` 段或 override.conf 中设置）：

| 环境变量 | 用途 | 默认值 |
|----------|------|--------|
| `LINGTU_CAMERA_STATUS_FILE` | camera JSON 状态文件路径 | `/dev/shm/lingtu/camera_status.json` |
| `LINGTU_DDS_DOMAIN_ID` | 全部原生 DDS 服务共享的 domain ID | `0` |
| `LINGTU_CAMERA_COLOR_TOPIC` | 彩色图像 topic 名 | `rt/camera/color` |
| `LINGTU_CAMERA_DEPTH_TOPIC` | 深度图像 topic 名 | `rt/camera/depth` |
| `LINGTU_CAMERA_INFO_TOPIC` | 相机参数 topic 名 | `rt/camera/info` |

覆盖示例：

```bash
sudo systemctl edit lt-camera.service
# 在编辑器中添加：
# [Service]
# Environment="LINGTU_CAMERA_STATUS_FILE=/dev/shm/lingtu/camera_status.json"
# Environment="LINGTU_DDS_DOMAIN_ID=0"
```

---

## 3. Gateway services/status 部署与验证（工作流 #16）

部署新 Gateway 代码后，验证 `/api/v1/services/status` 端点不再返回 404。

```bash
# 查询多个服务的状态（逗号分隔）
curl -s "http://${LINGTU_HOST}:5050/api/v1/services/status?names=camera,lidar,slam,nav,driver" | python3 -m json.tool
```

### 期望响应结构

```json
{
  "schema_version": 1,
  "services": {
    "camera": {"status": "running", "systemd_unit": "lt-camera.service"},
    "lidar": {"status": "running", "systemd_unit": "lt-lidar.service"},
    "slam": {"status": "running", "systemd_unit": "lt-slam.service"},
    "nav": {"status": "running", "systemd_unit": "lt-nav.service"},
    "driver": {"status": "running", "systemd_unit": "lt-driver.service"}
  },
  "readiness": {
    "camera": {"ready": true, "blockers": []},
    "lidar": {"ready": true, "blockers": []},
    "slam": {"ready": true, "blockers": []},
    "nav": {"ready": true, "blockers": []},
    "driver": {"ready": true, "blockers": []}
  },
  "field_readiness": {
    "ok": true,
    "missing": [],
    "sampled_at": "2026-07-08T10:00:00Z"
  }
}
```

### 异常排查

- **404**：Gateway 代码未部署或路由未注册，检查 Gateway 版本
- **readiness blockers 非空**：按 blocker 类型逐一排查（systemd unit 缺失 / status file 不存在 / DDS silence）
- **field_readiness.ok = false**：参见第 4 节 field readiness collector 诊断

---

## 4. Field Readiness Collector 采样（工作流 #17）

一次性采样当前 Product 需要的 native DDS Topic，验证数据链路连通性。Topic 必须显式指定，避免检查并未由当前 Product 启动的传感器或算法。

```bash
cd ~/data/SLAM/navigation

# 以包含 camera、LiDAR、SLAM 和 navigation 的 Product 为例
PYTHONPATH=src:. python -m diagnostics.field.dds_readiness \
    --seconds 5 \
    --domain 0 \
    --topics rt/camera/color rt/camera/depth rt/camera/info \
             rt/lidar/raw_frame rt/imu/raw \
             rt/slam/odometry rt/slam/map_cloud rt/slam/localization_health \
             rt/nav/traversability rt/nav/terrain_map \
    --json artifacts/field/readiness.json
```

### 输出解读

- `ok=true`：所有 topic 在采样窗口内收到数据
- `ok=false`：存在 dead topic，查看 `missing[]` 列表

### 期望输出示例

```json
{
  "schema_version": 1,
  "ok": true,
  "duration_sec": 5,
  "domain_id": 0,
  "topics": {
    "rt/camera/color": {"alive": true, "samples": 15},
    "rt/camera/depth": {"alive": true, "samples": 15},
    "rt/camera/info": {"alive": true, "samples": 5},
    "rt/lidar/raw_frame": {"alive": true, "samples": 50},
    "rt/imu/raw": {"alive": true, "samples": 500},
    "rt/slam/odometry": {"alive": true, "samples": 50},
    "rt/slam/map_cloud": {"alive": true, "samples": 2},
    "rt/slam/localization_health": {"alive": true, "samples": 5},
    "rt/nav/traversability": {"alive": true, "samples": 10},
    "rt/nav/terrain_map": {"alive": true, "samples": 5},
    "rt/nav/cmd_vel": {"alive": false, "samples": 0}
  },
  "missing": []
}
```

### 监控的 Topic 清单

| 域 | Topic | 说明 |
|----|-------|------|
| camera | `rt/camera/color` | 彩色图像流 |
| camera | `rt/camera/depth` | 深度图像流 |
| camera | `rt/camera/info` | 相机内参 |
| lidar | `rt/lidar/raw_frame` | Livox 原始点云帧 |
| imu | `rt/imu/raw` | IMU 原始数据 |
| slam | `rt/slam/odometry` | 里程计 |
| slam | `rt/slam/map_cloud` | 建图点云 |
| slam | `rt/slam/localization_health` | 定位健康状态 |
| nav | `rt/nav/traversability` | 可通行性地图 |
| nav | `rt/nav/terrain_map` | 地形地图 |
| nav/driver | `rt/nav/cmd_vel` | 导航端点到 `lingtu-driver` 的速度指令；静止无任务时可能没有非零样本 |

---

## 5. Round-trip 测试（工作流 #19）

### 声明式绑定说明

Replay backend 是**声明式绑定**（declarative binding），非真实 LCM 运行时后端。
其作用是将 LCM channel 声明性地映射到 DDS typed topic，确保：

1. 每个 replay channel 的 message type 都有对应的 DDS typed spec
2. DDS codec 能正确完成 serialize → deserialize round-trip
3. Driver backend 与 replay binding 使用相同的 topic 路由

### DDS Round-trip 覆盖的类型清单

| DDS 类型 | 对应 Topic 示例 |
|----------|-----------------|
| Odometry | `rt/slam/odometry` |
| PointCloud2 | `rt/slam/map_cloud`, `rt/lidar/raw_frame` |
| OccupancyGrid | `rt/nav/traversability` |
| FinalVelocityCommand | `rt/nav/cmd_vel` |
| PoseStamped | 目标位姿 |
| Text (String) | 状态文本 |
| Float32 | 标量传感器值 |
| Imu | `rt/imu/raw` |
| LivoxRawFrame | `rt/lidar/raw_frame`（typed adapter） |
| CameraInfo | `rt/camera/info` |
| Image (color/depth) | `rt/camera/color`, `rt/camera/depth` |
| TFMessage | `/tf` |

---

## 6. 已知缺口 / 后续

| 项目 | 说明 | 优先级 |
|------|------|--------|
| Driver odometry DDS is not a field readiness blocker | Canonical runtime topic is `/driver/odometry`; typed DDS derives to `rt/driver/odometry`. It stays optional until a real driver/base odometry publisher is productized. | Medium |
| Driver motion readiness is status-file based | The product driver is validated through `/dev/shm/lingtu/driver_status.json`, gRPC lease/ACK state, and Gateway/service readiness. Do not require a live nonzero `rt/nav/cmd_vel` sample during a no-motion readiness check. | High |
| LCM/replay backend 非真实运行时 | replay backend 仅为声明式绑定，不作为真实 LCM 运行时后端存在 | 低（设计如此） |
| QoS field evidence | late subscriber / reconnect 场景需在 S100P 实机验证 | 高 |

---

## 7. LiDAR/IMU 收口验证命令

确认 `livox_sdk2_stream` 是唯一的 LiDAR/IMU owner，无遗留 ROS2 驱动进程。

```bash
# 确认 livox_sdk2_stream 是唯一 LiDAR/IMU owner
ps aux | grep -E "livox|lidar" | grep -v grep
# 期望：仅看到 livox_sdk2_stream 进程

# 确认无 livox_ros_driver2 或重复 IMU publisher
systemctl list-units | grep -E "livox|imu"
# 期望：仅 lt-lidar.service（active），无 livox_ros_driver2 相关 unit

# 如发现遗留进程，执行清理
# sudo systemctl stop livox_ros_driver2.service 2>/dev/null
# sudo systemctl disable livox_ros_driver2.service 2>/dev/null
```

### 判定标准

- ✅ 仅 `livox_sdk2_stream` 进程存在
- ✅ 无 `livox_ros_driver2` systemd unit 处于 active 状态
- ✅ IMU 数据仅通过 `rt/imu/raw` 单一 topic 发布
- ❌ 若存在重复 publisher，需先停止遗留服务再重新采样验证

---

## 附录：快速验证清单

部署完成后，按以下顺序逐步验证：

```bash
# 1. 服务安装与启动
# Product mode units are not boot owners; disabled or static is expected.
for unit in lt-camera lt-lidar lt-slam lt-nav; do
  systemctl is-enabled "${unit}.service" 2>/dev/null || true
done
# Only a RunPlan-declared persistent role may be boot-enabled.
systemctl is-enabled lt-driver.service      # → enabled
bash scripts/lingtu status --json | python3 -m json.tool  # active Product and RunPlan

# 2. LiDAR/IMU 收口
ps aux | grep -E "livox|lidar" | grep -v grep  # → 仅 livox_sdk2_stream

# 3. Gateway 端点
curl -sf http://localhost:5050/api/v1/services/status?names=camera,lidar,slam,nav,driver | python3 -m json.tool

# 4. Native endpoint and driver status files
jq . /dev/shm/lingtu/nav_endpoint_status.json
jq . /dev/shm/lingtu/driver_status.json

# 5. Field readiness 采样
PYTHONPATH=src:. python -m diagnostics.field.dds_readiness \
  --seconds 5 --domain 0 \
  --topics rt/lidar/raw_frame rt/imu/raw rt/slam/odometry rt/slam/map_cloud rt/slam/localization_health rt/nav/cmd_vel \
  --json /tmp/readiness.json
cat /tmp/readiness.json | python3 -m json.tool
```

全部通过后，机器人侧 native DDS 服务部署即告完成。
