# sim/ — MapPilot MuJoCo Simulation

硬件无关的全栈仿真：MuJoCo 物理 + 真实 LiDAR 射线追踪 + ROS2 nav 栈。

## LiDAR 实现：两个方案

| 方案 | 项目 | 原理 | GPU 要求 | 适用场景 |
|---|---|---|---|---|
| **A（推荐）** | [mujoco_ray_caster](https://github.com/Albusgive/mujoco_ray_caster) | MuJoCo C++ sensor plugin，`mj_ray()` 原生 | ❌ 不需要 | 主仿真，全平台 |
| **B（大规模）** | [OmniPerception](https://github.com/aCodeDog/OmniPerception) | Warp/CUDA GPU ray tracing，Livox Mid-360 原生 | ✅ CUDA 11+ | RL 训练，批量仿真 |

方案 A 默认使用：C++ plugin 编译后，在 XML 里一行配置 LiDAR，直接出 PointCloud2。
方案 B 备用：CoRL 2025 论文，Livox 扫描模式 100% 真实，但需要 GPU。

## 架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                       MuJoCo Physics                                │
│                                                                     │
│  worlds/spiral_terrain.xml                                          │
│  ├── terrain mesh (螺旋4层, gen_terrain_mesh.py 生成)               │
│  └── robot body                                                     │
│       └── lidar_link                                                │
│            └── <sensor type="ray_caster_lidar"> ← mujoco_ray_caster│
│                 fov_h=360, fov_v=59, size="400 16"                  │
└──────────────────┬──────────────────────────────────────────────────┘
                   │  d->sensordata (直接内存)
                   ▼
     bridge/mujoco_ros2_bridge.py
       │
       ├── /mujoco/pos_w_pointcloud (PointCloud2) → terrain_analysis
       ├── /nav/odometry (Odometry, 50Hz)
       └── TF: map → odom → body
                   ▲
       /nav/cmd_vel (TwistStamped) → 自研底层运动控制器
```

## 快速开始

### Step 1 — 编译 mujoco_ray_caster

```bash
# 克隆 MuJoCo 源码 + plugin
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git

# 修改 CMakeLists.txt，在 plugin 列表末尾加：
# add_subdirectory(plugin/mujoco_ray_caster)

cd .. && mkdir build && cd build
cmake .. && cmake --build . -j8

# 把 .so 放到 mujoco_plugin 目录
cd bin && mkdir -p mujoco_plugin && cp ../lib/libray_caster*.so ./mujoco_plugin/
export MUJOCO_PLUGIN_PATH=$(pwd)/mujoco_plugin
```

### Step 2 — 安装 Python 依赖

```bash
pip install mujoco numpy scipy
# ROS2 Humble（nav 栈用）
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Step 3 — 生成螺旋地形 mesh

```bash
python sim/scripts/gen_terrain_mesh.py
```

### Step 4 — 启动仿真

```bash
# 平地测试（快速验证，不需要 mesh）
python sim/scripts/run_sim.py --world open_field

# 螺旋全场景
ros2 launch sim/launch/sim.launch.py world:=spiral_terrain
```

### Step 5 — 发送导航目标

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: -47.8, y: -28.6, z: 20.2}}}"
```

## 方案 B：OmniPerception（GPU，Livox 精确模式）

```bash
pip install warp-lang[extras] taichi
cd LidarSensor && pip install -e .

# MuJoCo 可视化（需要 ROS2）
source /opt/ros/humble/setup.bash
python3 LidarSensor/sensor_pattern/sensor_lidar/lidar_vis_ros2.py
```

## 文件结构

```
sim/
├── README.md
├── worlds/
│   ├── open_field.xml          # 平地场景（含 ray_caster_lidar XML 配置）
│   └── spiral_terrain.xml      # 螺旋场景（需先 gen_terrain_mesh.py）
├── robot/
│   └── robot.xml               # 机器人 MJCF（填入自研 URDF）
├── sensors/
│   └── livox_mid360.py         # 方案 B fallback：纯 Python mj_multiRay
├── bridge/
│   └── mujoco_ros2_bridge.py   # MuJoCo ↔ ROS2（odom, TF, 点云, cmd_vel）
├── launch/
│   └── sim.launch.py           # 全栈 ROS2 launch
├── scripts/
│   ├── gen_terrain_mesh.py     # spiral pickle → .stl mesh
│   └── run_sim.py              # 主入口
└── assets/
    └── meshes/                 # 生成的地形 mesh
```

## ROS2 Topics

| Topic | Type | 方向 |
|---|---|---|
| `/mujoco/pos_w_pointcloud` | PointCloud2 | MuJoCo → terrain_analysis |
| `/nav/odometry` | Odometry | MuJoCo → nav_stack |
| `/nav/cmd_vel` | TwistStamped | nav_stack → MuJoCo |
| `tf` (map→odom→body) | TF2 | MuJoCo → nav_stack |

## 依赖

- MuJoCo ≥ 3.0（含 ray_caster plugin 编译）
- Python: `mujoco numpy scipy`
- ROS2 Humble: `sensor_msgs nav_msgs tf2_ros`
- cyclonedds-cpp（ROS2 bridge 用，fastdds 可能有问题）
