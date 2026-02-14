# 部署指南（一键安装）

> 目标：新机器从零到跑通，全程复制粘贴，不需要记任何东西。

---

## 0. 前提

- Ubuntu 22.04 (aarch64 或 x86_64)
- 用户名: `sunrise`（如果不同，搜索替换即可）
- 网络已通

---

## 1. 系统依赖（一次性）

```bash
# ROS 2 Humble + 常用包
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs \
    libpcl-dev libeigen3-dev libboost-all-dev \
    libyaml-cpp-dev libgrpc++-dev protobuf-compiler-grpc \
    libssl-dev libcurl4-openssl-dev \
    python3-pip git cmake

pip3 install numpy scipy scikit-learn
```

---

## 2. 拉取代码

```bash
cd /home/sunrise/data/SLAM
git clone <你的仓库地址> navigation
cd navigation
git submodule update --init --recursive
```

> 工作区路径: `/home/sunrise/data/SLAM/navigation`
> 后面统一叫 `$NAV_DIR`

---

## 3. 编译第三方库

### 3.1 Sophus

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

### 3.2 GTSAM 4.1.1

```bash
cd /home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
```

```bash
# 加入环境变量（永久生效）
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib' >> ~/.bashrc
source ~/.bashrc
```

### 3.3 Livox SDK2（使用 Livox LiDAR 时必须）

```bash
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### 3.4 libdatachannel（WebRTC 视频流，可选）

```bash
cd /tmp
git clone https://github.com/nicknsy/libdatachannel.git
cd libdatachannel
git submodule update --init --recursive
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build
```

---

## 4. 编译工作区

```bash
cd /home/sunrise/data/SLAM/navigation
./build_all.sh
source install/setup.bash
```

验证：

```bash
ros2 pkg list | grep -E "fastlio2|local_planner|pct_planner|remote_monitoring"
# 应该看到这 4 个包
```

---

## 5. 安装系统服务（一条命令）

```bash
cd /home/sunrise/data/SLAM/navigation
sudo bash scripts/install_services.sh
```

这条命令做了 3 件事：

| 步骤 | 动作 | 结果 |
|------|------|------|
| 1 | `ln -sfn /home/sunrise/data/SLAM/navigation /opt/nav` | 创建符号链接 |
| 2 | 复制 `systemd/*.service` 到 `/etc/systemd/system/` | 注册服务 |
| 3 | 配置 sudoers | sunrise 用户可以无密码管理服务 |

验证：

```bash
ls -la /opt/nav
# 应显示: /opt/nav -> /home/sunrise/data/SLAM/navigation

systemctl list-unit-files | grep nav
# 应显示 6 个服务
```

---

## 6. 配置地图路径（运行模式需要）

```bash
sudo nano /etc/nav/planning.env
```

取消注释并填写：

```bash
NAV_MAP_PATH=/opt/nav/maps/site
NAV_INIT_X=0.0
NAV_INIT_Y=0.0
NAV_INIT_Z=0.0
NAV_INIT_YAW=0.0
```

---

## 7. 启动

### 建图模式

```bash
sudo systemctl start nav-lidar
sudo systemctl start nav-slam
sudo systemctl start nav-autonomy
sudo systemctl start nav-grpc
```

或一行：

```bash
sudo systemctl start nav-lidar nav-slam nav-autonomy nav-grpc
```

### 运行模式（自主导航）

```bash
sudo systemctl start nav-lidar nav-slam nav-planning nav-autonomy nav-grpc
```

### OTA 服务

```bash
sudo systemctl start ota-daemon
```

### 开机自启

```bash
sudo systemctl enable nav-lidar nav-slam nav-autonomy nav-grpc ota-daemon
```

---

## 8. 日常运维

### 查看状态

```bash
systemctl status nav-slam
systemctl status nav-autonomy
```

### 查看日志

```bash
journalctl -u nav-slam -f           # 实时跟踪
journalctl -u nav-slam --since today # 今天的日志
```

### 停止所有

```bash
sudo systemctl stop nav-planning nav-autonomy nav-slam nav-lidar nav-grpc ota-daemon
```

### 重启某个服务

```bash
sudo systemctl restart nav-slam
```

---

## 9. 手动启动（开发调试）

不走 systemd，直接在终端运行：

```bash
cd /home/sunrise/data/SLAM/navigation
source install/setup.bash

# 建图 (全部节点一次启动)
ros2 launch launch/navigation_bringup.launch.py

# 运行 (自主导航)
ros2 launch launch/navigation_run.launch.py map_path:=/opt/nav/maps/site

# 单独启动某个子系统 (调试)
ros2 launch launch/subsystems/slam.launch.py
ros2 launch launch/subsystems/autonomy.launch.py maxSpeed:=0.3
```

---

## 10. 目录结构速查

```
/opt/nav/                              ← 符号链接, 指向工作区
├── launch/
│   ├── navigation_bringup.launch.py   ← 建图模式 (组合入口)
│   ├── navigation_run.launch.py       ← 运行模式 (组合入口)
│   └── subsystems/                    ← 子系统 launch (唯一事实来源)
│       ├── lidar.launch.py            ← nav-lidar.service
│       ├── slam.launch.py             ← nav-slam.service
│       ├── autonomy.launch.py         ← nav-autonomy.service
│       ├── planning.launch.py         ← nav-planning.service
│       ├── driver.launch.py           ← 底盘驱动 (generic/dog)
│       └── grpc.launch.py             ← nav-grpc.service
├── scripts/
│   ├── services/
│   │   ├── env.sh                     ← 环境变量 (自动推导 NAV_DIR)
│   │   ├── nav-lidar.sh               ← systemd 调用的薄壳
│   │   ├── nav-slam.sh
│   │   ├── nav-autonomy.sh
│   │   ├── nav-planning.sh
│   │   ├── nav-grpc.sh
│   │   └── ota-daemon.sh
│   └── install_services.sh            ← 一键安装
├── systemd/                           ← .service 文件 (使用 /opt/nav 路径)
├── src/                               ← 源码
├── install/                           ← colcon 编译产物
└── maps/                              ← 地图文件
```

---

## 11. 换路径 / 迁移到新机器

如果工作区搬到了新位置：

```bash
# 只需更新符号链接
sudo ln -sfn /new/workspace/path /opt/nav
sudo systemctl daemon-reload

# 重启服务
sudo systemctl restart nav-lidar nav-slam nav-autonomy nav-grpc
```

不需要修改任何 .service 文件，不需要重新安装。

---

## 12. 完整一键脚本（新机器从零部署）

把以上步骤串成一个脚本，新机器直接跑：

```bash
#!/bin/bash
# 一键部署 — 在新机器上执行
set -e

NAV_DIR="/home/sunrise/data/SLAM/navigation"

echo "=== [1/5] 系统依赖 ==="
sudo apt update && sudo apt install -y \
    ros-humble-desktop ros-humble-pcl-conversions ros-humble-tf2-geometry-msgs \
    libpcl-dev libeigen3-dev libboost-all-dev libyaml-cpp-dev \
    libgrpc++-dev protobuf-compiler-grpc libssl-dev libcurl4-openssl-dev \
    python3-pip git cmake
pip3 install numpy scipy scikit-learn

echo "=== [2/5] Sophus ==="
cd /tmp && git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir -p build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make -j$(nproc) && sudo make install

echo "=== [3/5] Livox SDK2 ==="
cd /tmp && git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc) && sudo make install

echo "=== [4/5] 编译工作区 ==="
cd "$NAV_DIR"
# GTSAM
cd src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
cd "$NAV_DIR"

# LD_LIBRARY_PATH
grep -q gtsam-4.1.1 ~/.bashrc || \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$NAV_DIR/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" >> ~/.bashrc
source ~/.bashrc

# 全量编译
./build_all.sh

echo "=== [5/5] 安装服务 ==="
sudo bash scripts/install_services.sh

echo ""
echo "✓ 部署完成!"
echo "  启动建图: sudo systemctl start nav-lidar nav-slam nav-autonomy nav-grpc"
echo "  启动导航: sudo systemctl start nav-lidar nav-slam nav-planning nav-autonomy nav-grpc"
```

---

*最后更新: 2026-02-11*
