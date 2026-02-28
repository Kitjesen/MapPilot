# 机器人部署指南 — MapPilot (灵途)

> 本文档覆盖: 硬件清单 → 一次性安装 → 建图 → 导航 → 遥控原理 → 语义导航

---

## 硬件清单

| 组件 | 规格 | 说明 |
|---|---|---|
| **导航计算板** | Jetson Orin NX 16GB | Ubuntu 22.04, ROS2 Humble |
| **LiDAR** | Livox MID-360 | 以太网连接, IP 192.168.1.1xx |
| **四足机器人** | 带 gRPC CMS 接口 | 默认 :13145, 同 LAN |
| **遥控终端** | 手机 / 平板 | 安装 Flutter 客户端 App |
| **可选: RGB-D** | Orbbec 相机 | 语义导航专用 (USB3) |
| **可选: 物理手柄** | PS4 / Xbox / 北通 | 需额外配置 joy_node (见下文) |

---

## 一次性安装

```bash
# 1. 克隆仓库
git clone <repo_url> ~/lingtu && cd ~/lingtu

# 2. 安装系统依赖
./scripts/install_deps.sh

# 3. 配置机器人参数（唯一需要改的文件）
nano config/robot_config.yaml
```

**必改项** (`config/robot_config.yaml`):

```yaml
driver:
  dog_host: "192.168.4.100"   # ← 四足机器人实际 IP
  dog_port: 13145

geometry:
  vehicle_width: 0.6          # ← 机器人实际宽度 (m)
  vehicle_length: 1.0         # ← 机器人实际长度 (m)

speed:
  max_speed: 0.875            # ← 最大速度 (m/s)
  autonomy_speed: 0.875       # ← 自主导航速度 (m/s)
```

```bash
# 4. 编译
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 5. 验证 (12 项健康检查)
make health
```

---

## 遥控原理

### 当前实现：Flutter App 虚拟摇杆

系统**没有物理手柄节点**，遥控完全通过手机 App 的虚拟摇杆实现：

```
Flutter 客户端 (手机/平板)
  ├─ 左摇杆 → vx (前进/后退), vy (横移)
  └─ 右摇杆 → wz (转向)
        ↓ gRPC StreamTeleop (50Hz 持续流)
ControlService (remote_monitoring C++)
        ↓
SafetyGate
  ├─ Deadman 保护: 300ms 未收到指令 → 自动零速
  ├─ 速度限幅: |vx|, |vy| ≤ max_linear, |wz| ≤ max_angular
  ├─ 坡度保护: Roll/Pitch > 30° → 停止
  └─ 近场急停: 障碍物 < 0.8m → 发布 stop=2
        ↓
/nav/cmd_vel (TwistStamped)
        ↓
  建图模式: driver_node.py  → 串口/CAN → 通用底盘
  导航模式: han_dog_bridge  → gRPC :13145 → 四足 CMS
```

### App 安装

```bash
# Android APK (CI 自动构建)
# 在 GitHub Releases 页面下载最新 APK
# 安装后输入机器人 IP:50051 连接

# Windows 开发测试版
# GitHub Releases → Windows_Flutter_Monitor.zip
```

### 如需接入物理手柄（可选）

```bash
# 额外安装
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# 启动手柄节点（单独终端）
ros2 launch teleop_twist_joy teleop-launch.py \
  joy_config:=xbox \
  publish_stamped_twist:=true

# remap 到标准接口
ros2 run topic_tools relay /cmd_vel /nav/cmd_vel
```

---

## 建图流程

### 步骤 1: 启动建图模式

```bash
./mapping.sh
# 或
ros2 launch launch/navigation_bringup.launch.py
```

**启动的节点**:
- `livox_ros_driver2` — LiDAR 驱动
- `fastlio2` — Fast-LIO2 SLAM (点云 + IMU 融合)
- `terrain_analysis` — 地形分析
- `local_planner` — 局部规划 (遥控时作为避障辅助)
- `path_follower` — 路径跟随 (遥控时待机)
- `driver_node` — 底盘驱动 (generic 模式)
- `grpc_gateway` — App 连接

### 步骤 2: 用 App 遥控建图

打开 Flutter App → 连接机器人 → 用虚拟摇杆驱动机器人走完整个场景。

> **建图技巧**: 走 8 字形路线，让 LiDAR 从多角度扫描同一区域，PGO 回环质量更好。

### 步骤 3: 保存地图

```bash
# 新终端（建图节点保持运行）
./save_map.sh
# 选 2 → 保存 PGO 点云 + 自动生成 PCT Tomogram
```

**输出文件**:
```
maps/map_YYYYMMDD_HHMMSS.pcd                           # 原始 SLAM 地图
src/global_planning/PCT_planner/rsc/pcd/map_*.pcd      # PCT 工作目录
src/global_planning/PCT_planner/rsc/tomogram/map_*.pickle  # 规划用地图
```

---

## 导航流程

### 步骤 1: 设置地图路径

```bash
export NAV_MAP_PATH="/root/lingtu/src/global_planning/PCT_planner/rsc/tomogram/map_YYYYMMDD_HHMMSS"
# .pickle 扩展名可省略
```

### 步骤 2: 启动导航

```bash
./planning.sh
# 或
ros2 launch launch/navigation_run.launch.py

# 指定参数
ros2 launch launch/navigation_run.launch.py \
  map_path:=$NAV_MAP_PATH \
  dog_host:=192.168.4.100 \
  maxSpeed:=0.875
```

**启动的节点** (比建图模式多):
- 所有建图节点 (LiDAR / SLAM / terrain / local_planner / path_follower)
- `localizer_icp` — ICP 重定位 (在已知地图中定位)
- `pct_global_planner` — PCT 全局路径规划
- `pct_path_adapter` — 全局路径 → 航点转换
- `han_dog_bridge` — 四足机器人 gRPC 驱动 (替换 driver_node)

### 节点完整链路

```
LiDAR → Fast-LIO2 ─────────────────────────────→ /nav/map_cloud
              │                                         │
              └── ICP Localizer → /nav/odometry    terrain_analysis
                                       │                │
                                 path_follower    local_planner
                                       │                │
                              /nav/cmd_vel ←─────────────┘
                                       │
                                han_dog_bridge
                                       │
                               gRPC :13145
                                       │
                               四足机器人 CMS
```

### 步骤 3: 发送导航目标

```bash
# 方式 A: App 地图界面点击目标点
# 方式 B: 命令行
ros2 topic pub --once /nav/goal_pose geometry_msgs/PoseStamped \
  "{'header': {'frame_id': 'map'},
    'pose': {'position': {'x': 1.5, 'y': 2.0, 'z': 0.0},
             'orientation': {'w': 1.0}}}"

# 方式 C: 语义指令 (需启用语义导航)
ros2 topic pub --once /nav/semantic/instruction std_msgs/String \
  "{'data': '找到餐桌'}"
```

---

## 语义导航

### 硬件要求

- Orbbec RGB-D 相机 (USB3)
- LLM API Key (Kimi / Qwen / GPT 任选其一)

### 配置 LLM

```bash
# 国内优先 (无代理)
export MOONSHOT_API_KEY="sk-..."     # Kimi (默认)
export DASHSCOPE_API_KEY="sk-..."    # Qwen (备用)

# 国际
export OPENAI_API_KEY="sk-..."
export ANTHROPIC_API_KEY="sk-ant-..."
```

`config/semantic_planner.yaml` 中选择 backend:
```yaml
llm:
  backend: kimi          # kimi | openai | claude | qwen
  model: kimi-k2.5
  timeout_sec: 30
```

### 启动方式

**方式 A: 导航模式 + 语义 (已有地图)**

```bash
ros2 launch launch/navigation_run.launch.py \
  enable_semantic:=true \
  map_path:=$NAV_MAP_PATH
```

**方式 B: 探索模式 (无预置地图)**

```bash
# 零样本目标导航 — 边建图边找目标
ros2 launch launch/navigation_explore.launch.py

# 指定目标
ros2 launch launch/navigation_explore.launch.py target:="找到餐桌"
```

### 语义导航数据流

```
Orbbec 相机
  ├─ /camera/color/image_raw
  └─ /camera/depth/image_rect_raw
        ↓
semantic_perception_node
  ├─ YOLO-World 目标检测
  ├─ CLIP 特征提取
  └─ /nav/semantic/scene_graph (JSON 场景图)
        ↓
semantic_planner_node
  ├─ Fast Path: 场景图关键词匹配 (~0.17ms)
  ├─ Slow Path: LLM 推理 (~2s, 仅必要时)
  └─ /nav/goal_pose (解析出的目标坐标)
        ↓
PCT 全局规划器 (正常导航流程)
```

### 语义导航指令示例

```bash
# 自然语言目标
"找到餐桌"
"去厨房"
"找到红色椅子"
"到最近的出口"

# 层次化目标 (OmniNav)
"在客厅里找沙发"   → 先导航到客厅区域, 再找沙发
```

---

## 常用快捷命令

```bash
make build          # 编译
make health         # 12 项系统健康检查
make mapping        # 启动建图
make navigation     # 启动导航
./save_map.sh       # 保存地图 + 生成 PCT Tomogram

# 查看话题频率
ros2 topic hz /nav/cmd_vel
ros2 topic hz /nav/odometry
ros2 topic hz /nav/terrain_map

# 实时查看规划状态
ros2 topic echo /nav/planner_status

# 紧急停车
ros2 topic pub --once /nav/stop std_msgs/Int8 "{'data': 2}"
```

---

## 故障排查

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| App 连不上机器人 | gRPC 端口未开放 | 检查防火墙, 确认 50051 端口 |
| 机器人不动 | Deadman 超时 / App 断连 | 重新连接 App 并保持摇杆活跃 |
| 建图点云漂移 | LiDAR-IMU 时间戳不同步 | 检查 livox 驱动时间戳配置 |
| 导航不走 | safetyStop_ 被触发 | 发布 `stop=0` 清除, 检查障碍物 |
| 全局规划失败 | Tomogram 未加载 | 确认 NAV_MAP_PATH 指向 .pickle |
| 语义导航无响应 | LLM API 超时 | 检查 API Key 和网络, 切换 backend |

---

*最后更新: 2026-02-28 | 版本: 1.5.0*
