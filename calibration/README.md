# Calibration �?传感器标定工具箱

多传感器标定工具箱，�?S100P 四足机器人的三类传感器提供出厂标定支持�?

## 传感�?

| 传感�?| 型号 | 标定类型 |
|--------|------|----------|
| 相机 | Orbbec RGB-D | 内参 (棋盘�? |
| IMU | Livox 内置 (200Hz) | 噪声参数 (Allan Variance) |
| LiDAR | Livox Mid-360 | LiDAR-IMU 外参 + 时间偏移 |
| 相机+LiDAR | �?| 相机-LiDAR 外参 (target-less) |

## 目录结构

```
calibration/
├── camera/                    # 相机内参标定 (OpenCV)
�?  └── calibrate_intrinsic.py
├── imu/                       # IMU 噪声标定 (Allan Variance)
�?  ├── allan_variance_ros2/   # 第三�? Autoliv-Research
�?  └── config/                # 适配 Livox IMU 的配�?
├── lidar_imu/                 # LiDAR-IMU 外参标定
�?  ├── LiDAR_IMU_Init/        # 第三�? HKU-MARS (ROS1)
�?  └── ros2_adapter/          # ROS2 适配�?(rosbag 回放 + 结果解析)
├── camera_lidar/              # 相机-LiDAR 外参标定 (target-less)
�?  └── direct_visual_lidar_calibration/  # 第三�? koide3 (ROS2 原生)
├── apply_calibration.py       # 将所有标定结果写�?robot_config.yaml
├── verify.py                  # 一键验证所有标定结�?
└── README.md                  # 本文�?
```

## 出厂标定流程 (SOP)

### 前置条件

```bash
# 确保 ROS2 Humble 环境
source /opt/ros/humble/setup.bash

# 编译标定工具 (Camera-LiDAR 需�?colcon build)
cd calibration/camera_lidar/direct_visual_lidar_calibration
colcon build --packages-select direct_visual_lidar_calibration
```

### Step 1: 相机内参标定 (~5 分钟)

打印一�?9×6 棋盘格，在相机前多角度展示�?

```bash
python calibration/camera/calibrate_intrinsic.py auto --device 0

# 或者分步执�?
python calibration/camera/calibrate_intrinsic.py capture --device 0 --out calib_imgs/
python calibration/camera/calibrate_intrinsic.py calibrate --images calib_imgs/ --out calibration/camera/output/camera_calib.yaml
```

输出: `camera_calib.yaml` (fx, fy, cx, cy, 畸变系数)

### Step 2: IMU 噪声标定 (~2-3 小时)

将机器人放在稳定平面上，静置录制 IMU 数据�?

```bash
# 1. 录制静�?IMU 数据 (至少 2 小时)
ros2 bag record /livox/imu -o imu_static_bag

# 2. 运行 Allan Variance 分析
ros2 run allan_variance_ros2 allan_variance \
    imu_static_bag \
    calibration/imu/config/livox_mid360_imu.yaml

# 3. 拟合噪声模型
python3 calibration/imu/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py \
    --data allan_variance.csv
```

输出: `imu.yaml` (accelerometer_noise_density, gyroscope_noise_density, random_walk)

### Step 3: LiDAR-IMU 外参标定 (~2 分钟)

让机器人�?8 字运动，激发所有轴的运动�?

```bash
# 方法 A: 使用 ROS2 adapter (推荐)
# 先录一段包�?/livox/lidar + /livox/imu �?rosbag
ros2 bag record /livox/lidar /livox/imu -o lidar_imu_bag --duration 60

# 通过 ros1_bridge 回放�?LI-Init (需�?ROS1 环境)
# 或者用我们�?Python adapter:
python calibration/lidar_imu/ros2_adapter/run_calibration.py \
    --bag lidar_imu_bag \
    --config calibration/lidar_imu/config/mid360.yaml

# 方法 B: 直接�?ROS1 环境运行
roslaunch lidar_imu_init livox_mid360.launch
```

输出: `Initialization_result.txt` (r_il, t_il, time_offset, gravity, bias)

### Step 4: 相机-LiDAR 外参标定 (~10 分钟)

在有丰富纹理的环境中，静止录�?5-10 组数据�?

```bash
# 1. 录制数据 (每组静止 15 �?
ros2 bag record /camera/color/image_raw /camera/camera_info /livox/lidar \
    -o calib_bag_01 --duration 15

# 2. 预处�?
ros2 run direct_visual_lidar_calibration preprocess \
    calib_bag_01 preprocessed_01 \
    --image_topic /camera/color/image_raw \
    --camera_info_topic /camera/camera_info \
    --points_topic /livox/lidar

# 3. 初始猜测 (手动点�?3+ 对应�?
ros2 run direct_visual_lidar_calibration initial_guess_manual preprocessed_01

# 4. 优化标定
ros2 run direct_visual_lidar_calibration calibrate preprocessed_01

# 5. 验证 (可视化点云投�?
ros2 run direct_visual_lidar_calibration viewer preprocessed_01
```

输出: `calib.json` (T_lidar_camera: [x, y, z, qx, qy, qz, qw])

### Step 5: 一键应�?

```bash
python calibration/apply_calibration.py \
    --camera calibration/camera/output/camera_calib.yaml \
    --imu calibration/imu/output/imu.yaml \
    --lidar-imu calibration/lidar_imu/output/Initialization_result.txt \
    --camera-lidar calibration/camera_lidar/output/calib.json
```

### Step 6: 验证

```bash
python calibration/verify.py
```

## 第三方项�?

| 项目 | 来源 | License | 用�?|
|------|------|---------|------|
| allan_variance_ros2 | [Autoliv-Research](https://github.com/Autoliv-Research/allan_variance_ros2) | BSD-3 | IMU Allan Variance |
| LiDAR_IMU_Init | [HKU-MARS](https://github.com/hku-mars/LiDAR_IMU_Init) | GPL-2.0 | LiDAR-IMU 外参 |
| direct_visual_lidar_calibration | [koide3](https://github.com/koide3/direct_visual_lidar_calibration) | MIT | 相机-LiDAR 外参 |

## 标定参数输出到哪�?

所有标定结果最终写�?`config/robot_config.yaml`:

```yaml
camera:
  fx, fy, cx, cy           # �?Step 1 相机内参
  dist_k1..k3, dist_p1..p2 # �?Step 1 畸变系数
  position_x/y/z           # �?Step 4 相机-LiDAR 外参 (�?LiDAR-body �?
  roll, pitch, yaw         # �?Step 4 旋转

lidar:
  offset_x/y/z             # �?Step 3 LiDAR-IMU 外参 (t_il)
  roll, pitch, yaw         # �?Step 3 旋转 (r_il)

# Fast-LIO2 / Point-LIO 配置也同步更�?
#   src/localization/fastlio2/config/lio.yaml  �?na, ng, nba, nbg, r_il, t_il
#   config/pointlio.yaml               �?相应参数
```
