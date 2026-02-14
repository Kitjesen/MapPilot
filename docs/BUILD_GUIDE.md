# 编译指南

## 前提条件

- Ubuntu 22.04
- ROS 2 Humble
- 8GB+ RAM, 4核+ CPU

---

## 约定

本文档中 `$NAV_DIR` 代表工作区根目录 (即 `3d_NAV` 所在路径)。

部署到机器人时，该目录通常为 `/home/sunrise/data/SLAM/navigation`，
本地开发时是你 clone 仓库后的实际路径。

---

## 一键安装系统依赖

```bash
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    python3-pip \
    git \
    cmake

pip3 install numpy scipy scikit-learn
```

---

## 编译流程 (完整步骤)

### Step 1: 编译第三方库

#### 1.1 Sophus (李群库, SLAM 需要)

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

#### 1.2 GTSAM 4.1.1 (规划核心库, 必需)

```bash
cd $NAV_DIR/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
```

#### 1.3 libdatachannel (WebRTC 实时视频流, 推荐)

```bash
# 方式 A: 源码编译 (推荐, 版本可控)
cd ~
git clone https://github.com/nicknsy/libdatachannel.git
cd libdatachannel
git submodule update --init --recursive
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build

# 方式 B: 系统包 (Ubuntu 22.04, 版本可能较旧)
# sudo apt install libdatachannel-dev
```

> **验证**: `pkg-config --modversion libdatachannel` 或 `ls /usr/local/lib/libdatachannel.so`
>
> **不安装的后果**: `remote_monitoring` 仍可编译, 但 WebRTC 视频功能自动禁用。详见 [WEBRTC_GUIDE.md](WEBRTC_GUIDE.md)

#### 1.4 Livox SDK2 (可选, 仅使用 Livox LiDAR 时需要)

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

---

### Step 2: 设置环境变量

```bash
# 添加 GTSAM 库路径
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$NAV_DIR/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" >> ~/.bashrc

# 立即生效
source ~/.bashrc
```

---

### Step 3: 编译

#### 3.1 全量编译 (推荐)

```bash
cd $NAV_DIR
./build_all.sh
source install/setup.bash
```

`build_all.sh` 会依次编译:
1. PCT Planner C++ 核心 (pybind11)
2. 所有 ROS 2 包 (colcon build)
3. OTA Daemon (独立 CMake)

#### 3.2 仅 ROS 包

```bash
cd $NAV_DIR
./build_all.sh --ros-only
source install/setup.bash
```

#### 3.3 仅 PCT 核心

```bash
cd $NAV_DIR
./build_all.sh --pct-only
```

#### 3.4 分包编译

```bash
cd $NAV_DIR

# 仅基础导航 (无 SLAM)
colcon build --packages-select \
    local_planner terrain_analysis terrain_analysis_ext visualization_tools

# 仅全局规划
colcon build --packages-select pct_planner pct_adapters

# 仅远程监控
colcon build --packages-select remote_monitoring

# 仅 SLAM
colcon build --packages-select fastlio2 hba pgo
```

---

## 部署与服务管理

### 安装 systemd 服务

```bash
cd $NAV_DIR
sudo bash scripts/install_services.sh
```

安装脚本会:
1. 创建符号链接 `/opt/nav` → 工作区根目录
2. 将 `systemd/*.service` 复制到 `/etc/systemd/system/`
3. 配置 sudoers 规则

`.service` 文件统一使用 `/opt/nav/` 路径, 无需模板替换。
换路径只需: `sudo ln -sfn /new/path /opt/nav && sudo systemctl daemon-reload`

### 环境变量配置

| 文件 | 用途 |
|------|------|
| `/etc/nav/planning.env` | 规划参数 (地图路径, 初始位姿) |

### 启动服务

```bash
# 建图模式
sudo systemctl start nav-lidar nav-slam nav-autonomy nav-grpc

# 运行模式 (需先配置 /etc/nav/planning.env 中的地图路径)
sudo systemctl start nav-lidar nav-slam nav-planning nav-autonomy nav-grpc

# OTA 服务
sudo systemctl start ota-daemon

# 开机自启
sudo systemctl enable nav-grpc ota-daemon
```

### 手动启动 (开发)

```bash
cd $NAV_DIR
source install/setup.bash

# 建图模式 (全部节点一次启动)
ros2 launch launch/navigation_bringup.launch.py

# 运行模式
ros2 launch launch/navigation_run.launch.py map_path:=/path/to/map

# 单独启动某个子系统 (调试)
ros2 launch launch/subsystems/slam.launch.py
ros2 launch launch/subsystems/autonomy.launch.py maxSpeed:=0.5
```

---

## 启动架构

```
navigation_bringup.launch.py / navigation_run.launch.py
    └── include: launch/subsystems/
        ├── lidar.launch.py      ← nav-lidar.service
        ├── slam.launch.py       ← nav-slam.service
        ├── autonomy.launch.py   ← nav-autonomy.service
        ├── planning.launch.py   ← nav-planning.service
        ├── driver.launch.py     (由 bringup/run 选择模式)
        └── grpc.launch.py       ← nav-grpc.service
```

子系统 launch 文件是节点配置的唯一事实来源 (SSOT)。
手动 launch 和 systemd 都引用同一套文件, 确保行为一致。

---

## 验证编译

### 测试基础导航包

```bash
ros2 run local_planner localPlanner
ros2 run local_planner pathFollower
ros2 run terrain_analysis terrainAnalysis
```

### 测试全局规划器

```bash
# 检查环境变量
echo $LD_LIBRARY_PATH | grep gtsam

# 测试 Python 模块导入
python3 -c "from planner_py import OfflineElePlanner; print('Success')"

# 运行规划器
ros2 run pct_planner global_planner
```

---

## 常见问题

### GTSAM not found

```bash
ls $NAV_DIR/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4
```

### libgtsam.so.4: cannot open

```bash
echo $LD_LIBRARY_PATH | grep gtsam
# 如果为空:
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$NAV_DIR/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
```

### ModuleNotFoundError: planner_py

```bash
colcon build --packages-select pct_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Livox SDK2 not found

```bash
ls /usr/local/lib/liblivox_lidar_sdk_shared.so
```

### Sophus: fmt library not found

```bash
cd ~/Sophus/build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

---

## 快速参考

| 包 | 依赖 | 编译时间 |
|---|---|---|
| local_planner | PCL, Eigen | ~30s |
| terrain_analysis | PCL | ~20s |
| pct_planner | GTSAM, pybind11 | ~1min |
| remote_monitoring | gRPC, libdatachannel (可选) | ~1min |
| GTSAM 4.1.1 | Boost, Eigen | ~10min |
| fastlio2 | Sophus, Livox SDK2 | ~2min |
| ota_daemon | gRPC, yaml-cpp, OpenSSL, CURL | ~30s |

---

## 相关文档

- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 编译/运行时故障排查
- [PARAMETER_TUNING.md](PARAMETER_TUNING.md) — 参数调优
- [CHANGELOG.md](CHANGELOG.md) — 变更记录

---

*最后更新: 2026-02-11*
