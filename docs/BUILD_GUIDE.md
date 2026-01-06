# 编译指南

## 前提条件

- Ubuntu 22.04
- ROS 2 Humble
- 8GB+ RAM, 4核+ CPU

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

#### 1.1 Sophus (李群库,SLAM需要)

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

#### 1.2 GTSAM 4.1.1 (规划核心库,必需)

```bash
cd ~/ros2_ws/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
```

#### 1.3 Livox SDK2 (可选,仅使用Livox LiDAR时需要)

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

---

### Step 2: 设置环境变量

**⚠️ 重要**: 必须配置,否则运行时会报错

```bash
# 添加 GTSAM 库路径
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib' >> ~/.bashrc

# 立即生效
source ~/.bashrc
```

---

### Step 3: 编译 ROS 2 包

#### 3.1 仅导航包 (最小化,推荐)

```bash
cd ~/ros2_ws

colcon build --packages-select \
    local_planner \
    terrain_analysis \
    terrain_analysis_ext \
    visualization_tools \
    pct_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

#### 3.2 包含 SLAM (完整版)

```bash
cd ~/ros2_ws

colcon build \
    --packages-skip robot_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## 验证编译

### 测试基础导航包

```bash
# 测试 Local Planner
ros2 run local_planner localPlanner

# 测试 Path Follower
ros2 run local_planner pathFollower

# 测试 Terrain Analysis
ros2 run terrain_analysis terrainAnalysis
```

### 测试全局规划器

```bash
# 检查环境变量
echo $LD_LIBRARY_PATH | grep gtsam
# 应该看到 gtsam-4.1.1/install/lib

# 测试 Python 模块导入
python3 -c "from planner_py import OfflineElePlanner; print('Success')"

# 运行规划器
ros2 run pct_planner global_planner
```

---

## 常见问题

### ❌ 编译失败: GTSAM not found

```bash
# 确保 GTSAM 已编译且在正确位置
ls ~/ros2_ws/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4

# 重新编译 GTSAM
cd ~/ros2_ws/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/build
cmake .. -DCMAKE_INSTALL_PREFIX=../install && make install
```

### ❌ 运行时: libgtsam.so.4: cannot open

```bash
# 检查环境变量
echo $LD_LIBRARY_PATH | grep gtsam

# 如果为空,重新添加
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
```

### ❌ Python: ModuleNotFoundError: planner_py

```bash
# 重新编译 pct_planner
cd ~/ros2_ws
colcon build --packages-select pct_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

# 测试导入
python3 -c "import sys; sys.path.append('install/pct_planner/lib/python3.10/site-packages'); from planner_py import OfflineElePlanner"
```

### ❌ SLAM: Livox SDK2 not found

```bash
# 检查系统安装
ls /usr/local/lib/liblivox_lidar_sdk_shared.so

# 如果不存在,重新安装
cd ~/Livox-SDK2/build
sudo make install
```

### ❌ Sophus: fmt library not found

```bash
# 确保使用正确的编译选项
cd ~/Sophus/build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

---

## 分包编译 (按需选择)

### 仅基础导航 (无 SLAM)

```bash
colcon build --packages-select \
    local_planner \
    terrain_analysis \
    terrain_analysis_ext \
    visualization_tools
```

### 仅全局规划

```bash
# 前提: 已编译 GTSAM
colcon build --packages-select pct_planner
```

### 仅 SLAM

```bash
# 前提: 已安装 Sophus + Livox SDK2
colcon build --packages-select fastlio2 hba pgo
```

---

## 快速参考

| 包 | 依赖 | 编译时间 |
|---|---|---|
| local_planner | PCL, Eigen | ~30s |
| terrain_analysis | PCL | ~20s |
| pct_planner | GTSAM, pybind11 | ~1min |
| GTSAM 4.1.1 | Boost, Eigen | ~10min |
| fastlio2 | Sophus, Livox SDK2 | ~2min |

---

## Docker 快速编译 (可选)

```dockerfile
FROM ros:humble
RUN apt-get update && apt-get install -y libpcl-dev libeigen3-dev libboost-all-dev
COPY . /ws/src/
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash"]
```

**使用**:
```bash
docker build -t pct_planner .
docker run -it --rm pct_planner
```

---

**最后更新**: 2026-01
