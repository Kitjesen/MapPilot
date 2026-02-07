# 3D Navigation System

野外/越野四足机器人自主导航系统，覆盖从感知到控制的完整链路。

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
# 安装依赖
sudo apt install ros-humble-desktop-full
rosdep install --from-paths src --ignore-src -r -y

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

*更多细节请参考 [docs/](docs/) 目录下的各专题文档。*
