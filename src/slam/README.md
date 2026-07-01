# SLAM — 双算法 LiDAR SLAM 模块

## 概述

MapPilot 支持两种 LiDAR SLAM 算法，通过 launch 参数 `slam_profile` 切换:

| 算法 | Profile | 特点 |
|------|---------|------|
| **Fast-LIO2** | `fastlio2` (默认) | 成熟稳定，iKD-Tree，回环+重定位+地图优化全链路 |
| **Point-LIO** | `pointlio` | 逐点更新，更强抗振动能力，适合四足机器人 |

```bash
# 使用 Fast-LIO2 建图
python lingtu.py map

# 使用 Point-LIO (临时覆盖 SLAM backend)
python lingtu.py nav --slam pointlio

# 仅建图模式 (Fast-LIO2 + 保存 PCD)
python lingtu.py map
```

## 振动鲁棒性测试结果 (2026-03, S100P)

| 场景 | Point-LIO | Fast-LIO2 |
|------|-----------|-----------|
| 实时 Mid-360 (室内) | 0.006m 漂移 | 0.008m 漂移 |
| Leg-KILO 走廊 (四足振动) | PASS, 445s/52m, 10Hz | 仅支持 Livox CustomMsg |

详见 `docs/07-testing/SLAM_VIBRATION_TEST_REPORT.md`

---

## Fast-LIO2 详细说明

### 主要工作
1. 重构 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 适配 ROS2
2. 添加回环节点，基于位置先验+ICP 进行回环检测，基于 GTSAM 进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段 ICP 进行重定位
4. 增加一致性地图优化，基于 [BALM](https://github.com/hku-mars/BALM) (小场景地图) 和 [HBA](https://github.com/hku-mars/HBA) (大场景地图)


## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计
```shell
ros2 launch slam/launch/fastlio2_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch slam/launch/pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch slam/launch/localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch slam/launch/hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
2. [BALM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)

## 性能相关的问题
该代码主要使用 timerCB 作为频率触发主函数，由于 ROS2 中的 timer、subscriber 以及 service 的回调实际上运行在同一个线程上，在电脑性能不好的时候会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来 (如 timerCB) 来提升性能。

---

## Point-LIO

### 来源
[SMBU-PolarBear fork](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) (ROS2 Humble, iVox)

### 路径
`src/slam/pointlio/`

### 配置
`config/pointlio.yaml` (Livox Mid-360, lidar_type=1=AVIA)

### 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | 1 (AVIA) | 非 4 |
| `timestamp_unit` | 3 (ns) | 纳秒 |
| `scan_bodyframe_pub_en` | true | 体坐标系发布 |
| `MP_PROC_NUM` | 1 | aarch64 多线程禁用 |

### 注意事项
- 无 `save_map` 服务，用 `pcd_save_en` 参数代替
- aarch64 编译需要 `libgoogle-glog-dev` + `libunwind-dev`
- 编译时间 ~1m44s (S100P)

### 参考
4. [Point-LIO](https://github.com/hku-mars/Point-LIO)
