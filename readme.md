<p align="center">
  <img src="FIG/MapPilot.png" alt="MapPilot Logo" width="400"/>
</p>

<h1 align="center">MapPilot - 3D SLAM 导航系统</h1>

<p align="center">
  <strong>野外/越野自主导航系统 | 从感知到控制的完整链路</strong>
</p>

<p align="center">
  <a href="https://github.com/Kitjesen/3d_NAV">
    <img src="https://img.shields.io/badge/GitHub-3d__NAV-blue?logo=github" alt="GitHub"/>
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
传感器 → SLAM (FAST-LIO2) → 地形分析 → 规划 (PCT + Local) → 控制
           ↓
         PGO/Localizer → TF: map → odom → body
```

---

## 🚀 快速开始

### 环境准备

```bash
# 1. 克隆仓库
git clone https://github.com/Kitjesen/3d_NAV.git
cd 3d_NAV

# 2. 编译工作空间
colcon build --symlink-install
source install/setup.bash
```

### 三步启动

| 步骤 | 命令 | 说明 |
|------|------|------|
| **1. 建图** | `./mapping.sh` | 启动传感器 + SLAM + RViz |
| **2. 保存** | `./save_map.sh` | 保存点云 + 生成 PCT 地图 |
| **3. 规划** | `./planning.sh` | 加载地图 + 启动规划器 |

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

Flutter 客户端通过 gRPC 实时监控和遥操作机器人。

| 功能 | 说明 |
|------|------|
| 实时遥测 | 位姿、速度、姿态、系统资源 |
| 遥操作 | 双摇杆、模式切换、紧急停止 |
| 视频流 | WebRTC + JPEG 回退 |
| 3D 可视化 | URDF 机器人模型 |
| 2D 轨迹 | 实时路径 + 全局地图 |

### 安装 APK

**方式一：GitHub Release (推荐)**

从 [Releases](https://github.com/Kitjesen/3d_NAV/releases/latest) 页面下载最新 `MapPilot-*.apk`，传到手机安装。

**方式二：本地编译**

```bash
cd client/flutter_monitor
flutter pub get
flutter build apk --release
# 输出: build/app/outputs/flutter-apk/app-release.apk
```

> 注意: APK 编译需要 x86_64 Linux/Mac/Windows，ARM64 设备请使用 GitHub Actions 自动构建。

### OTA 升级流程

```
开发者推送代码 → GitHub Actions 自动编译 APK → 发布到 Release
                                                    ↓
                        手机浏览器访问 Release 页面 ← 下载安装
```

**步骤：**

1. **开发者**：修改 `client/flutter_monitor/` 代码后推送到 `main` 分支
2. **CI 自动构建**：GitHub Actions 自动编译 APK 并发布到 Release
3. **用户更新**：
   - 手机浏览器访问: `https://github.com/Kitjesen/3d_NAV/releases/latest`
   - 下载最新 `MapPilot-*.apk`
   - 覆盖安装即可（数据保留）

**快捷方式：** 将 Release 页面添加到手机主屏幕书签，随时检查更新。

---

## 📚 文档

- [`AGENTS.md`](AGENTS.md) - 系统架构详解
- [`src/slam/interface/README.md`](src/slam/interface/README.md) - 接口定义
- [`client/flutter_monitor/README.md`](client/flutter_monitor/README.md) - 客户端详细文档
- [`src/remote_monitoring/README.md`](src/remote_monitoring/README.md) - gRPC 服务端文档

---

<p align="center">
  <sub>Made with ❤️ for autonomous navigation</sub>
</p>
