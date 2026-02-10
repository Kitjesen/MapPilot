# 3D Navigation System

<h1 align="center">MapPilot - 3D SLAM 导航系统</h1>

<p align="center">
  <strong>野外/越野自主导航系统 | 从感知到控制的完整链路</strong>
</p>

<p align="center">
  <a href="https://github.com/Kitjesen/MapPilot">
    <img src="https://img.shields.io/badge/GitHub-MapPilot-blue?logo=github" alt="GitHub"/>
  </a>
  <img src="https://img.shields.io/badge/ROS2-Humble-green?logo=ros" alt="ROS2 Humble"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow" alt="License"/>
</p>

---

## 📋 目录

- [系统概述](#-系统概述)
- [快速开始](#-快速开始)
- [工作流程](#-工作流程)
- [故障排查](#-故障排查)
- [高级功能](#-高级功能)
- [调试工具](#-调试工具)

---

## 🎯 系统概述

本工作空间包含完整的 **3D SLAM + 自主导航** 系统，基于 ROS 2 Humble：

| 模块 | 功能 | 技术栈 |
|------|------|--------|
| **SLAM** | 实时建图 + 后端优化 | FAST-LIO2 + PGO |
| **定位** | 重定位模块 | ICP 配准 |
| **感知** | 地形分析 | 地面估计、障碍物检测、可穿越性 |
| **全局规划** | 3D 路径规划 | PCT Planner (Point Cloud Tomography) |
| **局部规划** | 避障与跟踪 | base_autonomy |
| **传感器** | 多传感器支持 | Orbbec Gemini 330、Livox LiDAR |

### 系统架构

```
LiDAR → SLAM → 地形分析 → 路径规划 → 底盘控制
                                        ↓
              Flutter App ← gRPC Gateway ← 状态遥测
```

## 系统特性

- **SLAM**: Fast-LIO2 实时里程计 + PGO 回环优化 + 重定位
- **地形分析**: 地面估计 + 障碍物检测 + 可穿越性分析
- **路径规划**: PCT 全局规划 + base_autonomy 局部规划
- **安全架构**: 4 层解耦安全保障（RC > 仲裁器 > 看门狗 > 安全网关）
- **远程监控**: gRPC Gateway + Flutter 客户端（Android/iOS）
- **OTA 更新**: Ed25519 签名 / 安全等级分级 / 原子安装 + 崩溃恢复 / 依赖管理 / 一键回滚
- **双板架构**: Nav Board (导航) + Dog Board (运动控制)

## 快速开始

### 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- Livox MID-360 LiDAR
- 8GB+ RAM, 4核+ CPU

### 编译

```bash
# 1. 克隆仓库
git clone https://github.com/Kitjesen/MapPilot.git
cd MapPilot

# 编译
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 建图模式

```bash
ros2 launch navigation_bringup.launch.py
```

### 运行模式（定位 + 自主导航）

```bash
ros2 launch navigation_run.launch.py
```

详细步骤参见 [BUILD_GUIDE.md](docs/BUILD_GUIDE.md)

## 硬件架构

```
┌─────────────┐  Ethernet/gRPC  ┌──────────────┐
│  Nav Board  ├─────────────────┤  Dog Board   │
│  SLAM+规划  │     :13145      │  RL+电机控制  │
│  :50051     │                 │  :13145      │
└──────┬──────┘                 └──────┬───────┘
       │ WiFi                         │ BLE
  ┌────┴────┐                    ┌────┴────┐
  │ Flutter │                    │ YUNZHUO  │
  │   App   │                    │   RC    │
  └─────────┘                    └─────────┘
```

## 文档索引

| 文档 | 内容 | 面向 |
|------|------|------|
| **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** | 双板架构、数据流、安全体系、模式切换 | 系统设计者 |
| **[BUILD_GUIDE.md](docs/BUILD_GUIDE.md)** | 编译步骤、依赖安装、Docker 配置 | 开发者 |
| **[OTA_GUIDE.md](docs/OTA_GUIDE.md)** | OTA 更新系统、manifest 格式、GitHub 发布流程 | 运维 |
| **[WEBRTC_GUIDE.md](docs/WEBRTC_GUIDE.md)** | WebRTC 实时视频流架构、信令流程、部署配置 | 开发者 |
| **[PARAMETER_TUNING.md](docs/PARAMETER_TUNING.md)** | 规划器/地形分析/路径跟踪参数调优 | 调参工程师 |
| **[SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md)** | Gazebo/Isaac Sim 仿真环境搭建 | 测试工程师 |
| **[TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** | 编译错误、运行时故障、定位丢失排查 | 所有人 |
| **[PRODUCT_REVIEW.md](docs/PRODUCT_REVIEW.md)** | 产品架构评审、进程拆分方案、执行路线图 | 产品/架构师 |
| **[CHANGELOG.md](docs/CHANGELOG.md)** | 版本历史、升级记录、修复日志 | 所有人 |
| **[AGENTS.md](AGENTS.md)** | AI Agent 指引（话题/服务/坐标系速查表） | AI 辅助开发 |

## 项目结构

```
navigation/
├── src/
│   ├── slam/                    # SLAM 模块
│   │   ├── fastlio2/            #   Fast-LIO2 实时里程计
│   │   ├── pgo/                 #   回环检测与位姿图优化
│   │   └── localizer/           #   地图重定位
│   ├── base_autonomy/           # 局部规划
│   │   ├── local_planner/       #   路径选择 + 障碍物避让
│   │   ├── terrain_analysis/    #   地面估计
│   │   └── terrain_analysis_ext/#   可穿越性连通性分析
│   ├── global_planning/         # 全局规划
│   │   └── PCT_planner/         #   层析成像 + A* 规划
│   ├── remote_monitoring/       # gRPC Gateway
│   ├── robot_proto/             # Protobuf 接口定义 (子模块)
│   ├── drivers/                 # 底盘驱动
│   └── utils/                   # 工具包 (Orbbec SDK 等)
├── client/
│   └── flutter_monitor/         # Flutter 监控客户端
├── launch/                      # 顶层启动文件
├── scripts/
│   └── ota/                     # OTA 发布工具
├── docs/                        # 文档
└── AGENTS.md                    # AI Agent 指引
```

## 协议

| 协议 | 端口 | 方向 | 用途 |
|------|------|------|------|
| `robot::v1` gRPC | 50051 | App ↔ Nav Board | 导航、监控、OTA |
| `han_dog` CMS gRPC | 13145 | Nav Board/App → Dog Board | 行走控制 |
| BLE | — | App → Dog Board | 急停、WiFi 配置 |
| SBUS/PPM | — | RC → Dog Board | 硬件级遥控 (最高优先级) |

## 版本

- **v1.0.0** (2026-02-08) — 首个稳定版本

---

## 📍 工作流程

### 1️⃣ 建图 (Mapping)

```bash
./mapping.sh
```

**功能：**
- 启动传感器 (Orbbec/Livox)
- 启动 FAST-LIO2 实时 SLAM
- 启动 PGO 后端优化
- 启动 RViz2 可视化

**操作：**
1. 选择传感器类型
2. 在 RViz2 中观察建图效果
3. 移动机器人扫描环境
4. 完成后运行 `./save_map.sh`

---

### 2️⃣ 保存地图 (Save Map)

```bash
./save_map.sh
```

**保存选项：**

| 选项 | 说明 |
|------|------|
| 1 | 仅保存 PGO 点云 |
| 2 ⭐ | 保存 + 生成 PCT 地图 (推荐) |
| 3 | 仅生成 PCT Tomogram |

**输出文件：**

```
maps/map_YYYYMMDD_HHMMSS.pcd              # 原始点云
src/global_planning/PCT_planner/rsc/
├── tomogram/*.pickle                      # PCT 地图
└── PCD/*.pcd                              # PCD 备份
```

---

### 3️⃣ 规划导航 (Planning)

```bash
./planning.sh
```

**定位模式：**

| 模式 | 说明 | 适用场景 |
|------|------|----------|
| 假定位 ⭐ | RViz 手动设置位置 | 测试/仿真 |
| 真定位 | FAST-LIO2 自动定位 | 实机运行 |
| 跳过 | 使用外部定位 | 已有定位源 |

**规划操作：**
1. 确认机器人位置 (绿色箭头)
2. 使用 **Publish Point** 工具点击目标点
3. 查看生成的 3D 路径 (蓝色线条)

---

## 🔧 故障排查

<details>
<summary><b>❌ 找不到地图文件</b></summary>

```bash
# 检查地图是否存在
ls -lh src/global_planning/PCT_planner/rsc/tomogram/

# 如果没有，运行建图流程
./mapping.sh && ./save_map.sh
```
</details>

<details>
<summary><b>❌ RViz 无法显示点云</b></summary>

1. 检查话题：
```bash
ros2 topic list | grep tomogram
ros2 topic echo /pct_planner/tomogram --once
```

2. 检查 QoS 设置：
   - Reliability: Best Effort
   - Durability: **Transient Local** ⚠️

3. 重启 RViz：
```bash
killall rviz2
rviz2 -d src/global_planning/PCT_planner/rsc/rviz/pct_ros.rviz
```
</details>

<details>
<summary><b>❌ 无法规划路径</b></summary>

1. 检查定位：
```bash
ros2 topic echo /tf --once
# 应该看到 map -> body 的变换
```

2. 检查目标点：
   - 必须在地图范围内
   - 必须在可通行区域

3. 测试规划器：
```bash
python3 src/global_planning/PCT_planner/planner/scripts/test/check_map.py [地图名]
```
</details>

<details>
<summary><b>❌ 传感器无法启动</b></summary>

**Orbbec 相机：**
```bash
sudo bash src/utils/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
ros2 launch orbbec_camera gemini_330_series.launch.py
```

**Livox 激光雷达：**
```bash
ping 192.168.1.1
# 编辑: src/drivers/livox_ros_driver2/config/MID360_config.json
```
</details>

<details>
<summary><b>❌ 编译问题</b></summary>

```bash
# 清理重新编译
rm -rf build/ install/ log/
colcon build --symlink-install

# PCT_planner C++ 库问题
cd src/global_planning/PCT_planner/planner && ./build.sh && cd ../../../..
colcon build --packages-select pct_planner pct_adapters --symlink-install
```
</details>

---

## ⚙️ 高级功能

### 手动启动各模块

```bash
# Terminal 1: 相机
ros2 launch orbbec_camera gemini_330_series.launch.py

# Terminal 2: FAST-LIO2
ros2 launch fastlio2 lio_launch.py

# Terminal 3: PGO
ros2 launch pgo pgo_launch.py

# Terminal 4: PCT Planner
python3 src/global_planning/PCT_planner/planner/scripts/global_planner.py

# Terminal 5: RViz
rviz2 -d src/global_planning/PCT_planner/rsc/rviz/pct_ros.rviz
```

---

## 🔍 调试工具

### 话题监控

```bash
ros2 topic list                           # 所有话题
ros2 topic hz /Odometry                   # 检查频率
ros2 topic hz /pct_planner/tomogram
```

### TF 查看

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 节点信息

```bash
ros2 node list
ros2 node info /global_planner
```

### 地图可视化

```bash
# 可视化 Tomogram
python3 src/global_planning/PCT_planner/tomography/scripts/visualize_tomogram.py --scene [地图名]

# 检查地图信息
python3 src/global_planning/PCT_planner/planner/scripts/test/check_map.py [地图名]
```

---

## 📱 远程监控客户端

Flutter 客户端 (**MapPilot**) 通过 gRPC 实时监控和遥操作机器人，支持 Android / Linux / Web。

| 页面 | 功能 | 说明 |
|------|------|------|
| Status | 实时遥测 | 位姿、速度、姿态、系统资源 (10Hz / 1Hz) |
| Control | 遥操作 | 双摇杆、模式切换、紧急停止、FPV 视频 |
| Map | 轨迹可视化 | 2D/3D 地图、实时路径、点云叠加 |
| Events | 事件时间线 | 严重级别着色 + 确认机制 |
| **Settings** | 设置与管理 | 连接信息、文件管理、云端更新 |

### 文件管理 (OTA 部署)

直接从手机上传训练好的模型、地图、配置文件到机器人：

```
手机 (Settings → 文件管理)
  ↓  选择文件 / 选择分类 (模型·地图·配置·固件)
  ↓  gRPC UploadFile (64KB 分块)
  ↓
机器人 → /home/sunrise/models/yolo_terrain.pt
```

### 云端更新 (Cloud OTA)

App 联网从 GitHub Releases 获取最新版本，一键下载并部署到机器人：

```
GitHub Releases (云端)
  ↓  ① App 查询 API → 显示可用版本
  ↓  ② HTTP 下载资产文件
  ↓  ③ gRPC 上传到机器人
  ↓
机器人文件就位，重启服务即可生效
```

**操作步骤：** Settings → 云端更新 → 检查 → 选择文件 → ☁️ 下载并部署

### 安装 APK

**方式一：GitHub Release (推荐)**

从 [Releases](https://github.com/Kitjesen/MapPilot/releases/latest) 页面下载最新 `MapPilot-*.apk`。

**方式二：本地编译**

```bash
cd client/flutter_monitor
flutter pub get
flutter build apk --release
```

> APK 编译需要 x86_64 环境，ARM64 设备请使用 GitHub Actions 自动构建。

### 发布模型到云端

通过 GitHub Actions 发布模型/固件，App 即可一键拉取部署：

```bash
# 方式一: 手动触发 (GitHub → Actions → Release Models & Firmware)
# 方式二: Git tag 触发
git tag models-v1.0.0
git push origin models-v1.0.0
```

---

## 📚 文档

| 文档 | 说明 |
|------|------|
| [`AGENTS.md`](AGENTS.md) | 系统架构详解 (话题、坐标系、启动流程) |
| [`client/flutter_monitor/README.md`](client/flutter_monitor/README.md) | Flutter 客户端文档 (功能、架构、OTA) |
| [`src/remote_monitoring/README.md`](src/remote_monitoring/README.md) | gRPC 服务端文档 |
| [`src/robot_proto/README.md`](src/robot_proto/README.md) | Protobuf 接口定义 |
| [`src/slam/interface/README.md`](src/slam/interface/README.md) | SLAM 接口定义 |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 系统架构设计 |

---

<p align="center">
  <sub>Made with ❤️ for autonomous navigation</sub>
</p>
