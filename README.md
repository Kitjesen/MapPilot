# 灵途 MapPilot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Orin-green?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-orin)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)](https://ubuntu.com/)

> 🐕 让四足机器人在野外自己走

<p align="center">
  <img src="docs/assets/demo.gif" width="600" alt="MapPilot Demo"/>
</p>

---

我们在做一个野外自主导航系统，跑在 ROS 2 Humble 上。主要解决的问题是：机器人怎么在没有 GPS、没有预设地图的野外环境里，自己建图、定位、规划路径、避开障碍物。

## ✨ 功能特性

**🗺️ 建图定位**
- Fast-LIO2 做实时 SLAM，MID-360 激光雷达输入
- PGO 回环检测，长时间跑不会漂
- ICP 重定位，关机重启能接着用之前的地图

**👁️ 感知**
- 地形分析，知道哪能走哪不能走
- 障碍物检测，动态的静态的都行
- 可穿越性评估，草地、碎石、坡度都考虑
- YOLO-E 实例分割 + Mobile-CLIP 语义编码，15Hz on Jetson Orin NX

**🧭 规划**
- PCT 全局规划，基于地形代价
- base_autonomy 局部规划，实时避障
- SCG（空间连通图）拓扑路径规划，无需预建地图

**🗣️ 语义导航**
- 自然语言指令导航（"找到餐桌"）
- Fast-Slow 双路径：关键词快速匹配（~0.17ms）+ LLM 慢路径推理（~2s）
- ConceptGraphs 场景图，动态增量更新
- Frontier 探索 + 知识图谱房间预测，未知环境零样本导航

**📱 远程监控**
- gRPC 服务端（端口 50051）
- Flutter App，手机上看状态、发指令

## 🔧 硬件要求

| 组件 | 型号 | 必需 |
|------|------|:----:|
| 激光雷达 | Livox MID-360 | ✅ |
| 深度相机 | Orbbec Gemini 330 | ❌ |
| 系统 | Ubuntu 22.04 | ✅ |
| 内存 | 8GB+ | ✅ |

我们在 Jetson Orin NX 16GB 上跑，也在普通 x86 机器上测试过。

## 🚀 快速开始

```bash
# 编译
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 模式一：建图（手柄遥控，实时建 SLAM 地图）
./mapping.sh
./save_map.sh   # 跑一圈后保存地图

# 模式二：导航（加载已有地图，自主导航到目标点）
./planning.sh

# 模式三：探索（无需预建地图，未知环境语义导航）
ros2 launch launch/navigation_explore.launch.py
ros2 launch launch/navigation_explore.launch.py target:="找到餐桌"
```

建图时手动遥控走一遍，系统自动构建地图。导航模式给目标点自己走过去。探索模式直接说目标，机器人边建图边找。

## 📁 项目结构

```
src/
├── slam/                  # SLAM 相关
│   ├── fastlio2/          # Fast-LIO2 前端
│   ├── pgo/               # 位姿图优化
│   └── localizer/         # ICP 重定位
├── base_autonomy/         # 局部规划 + 地形分析
├── global_planning/       # PCT 全局规划器
├── semantic_perception/   # 语义感知：YOLO-E + CLIP + 场景图
├── semantic_planner/      # 语义规划：Fast-Slow + 探索策略 + LLM
├── remote_monitoring/     # gRPC 服务（端口 50051）
└── drivers/               # 底盘驱动

client/
└── flutter_monitor/       # 手机 App 📱

config/                    # 参数配置
launch/                    # 启动文件（三种模式）
```

## 🏗️ 系统架构

```
激光雷达 → SLAM → 地形分析 → PCT 规划 → 底盘控制
                ↓                  ↑
          深度相机 → YOLO-E        SCG 拓扑规划（探索模式）
                ↓
          场景图（ConceptGraphs）
                ↓
        语义规划器（Fast-Slow）← 自然语言指令
                ↓
          手机 App ← gRPC ← 状态上报
```

我们用双板架构：
- **Nav Board** 🧠: 导航算法（SLAM、规划、感知、语义）
- **Dog Board** 🦿: 运动控制（强化学习策略、电机驱动）

两块板子通过以太网通信，Nav Board 发速度指令，Dog Board 执行。

## 📚 文档

| 文档 | 说明 |
|------|------|
| [📐 ARCHITECTURE.md](docs/ARCHITECTURE.md) | 系统架构详解 |
| [🔨 BUILD_GUIDE.md](docs/BUILD_GUIDE.md) | 编译和部署 |
| [⚙️ PARAMETER_TUNING.md](docs/PARAMETER_TUNING.md) | 参数调优指南 |
| [🔧 TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | 常见问题 |
| [🤖 AGENTS.md](AGENTS.md) | AI Agent 开发指引 |

## ❓ FAQ

<details>
<summary><b>建图时点云漂移严重</b></summary>

检查 IMU 标定，Fast-LIO2 对 IMU 外参很敏感。另外确保激光雷达没有遮挡。

</details>

<details>
<summary><b>重定位失败</b></summary>

初始位置要和建图时的某个位置大致对应。如果环境变化太大（比如多了很多障碍物），可能需要重新建图。

</details>

<details>
<summary><b>规划的路径绕远路</b></summary>

调整 `config/global_planning.yaml` 里的代价权重，或者检查地形分析的参数。

</details>

## 📄 License

[MIT](LICENSE) © 2026

---

<p align="center">
  Made with ❤️ for robotics
</p>
