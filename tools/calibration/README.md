# Calibration — 传感器离线标定工具箱

本目录用于 S100P/Thunder 相关传感器的出厂、返修和现场复核标定。它不是产品运行时链路：LingTu 产品主路径保持 ROS-free，标定阶段可以在独立工位、容器或临时 ROS 环境里使用 ROS1/ROS2/rosbag 兼容工具，最后只把标定结果写回 LingTu 配置。

## 标定对象

| 传感器 | 型号/来源 | 标定类型 | 结果去向 |
| --- | --- | --- | --- |
| 相机 | Orbbec RGB-D | 相机内参、畸变 | `config/robot_config.yaml` |
| IMU | Livox Mid-360 内置 IMU | Allan Variance 噪声参数 | Fast-LIO2 / Point-LIO 配置 |
| LiDAR + IMU | Livox Mid-360 + 内置 IMU | 外参、时间偏移、初始 bias | SLAM/LIO 配置 |
| 相机 + LiDAR | Orbbec + Livox Mid-360 | Camera-LiDAR 外参 | `config/robot_config.yaml` |

## 目录结构

```text
tools/calibration/
├── camera/                         # LingTu-owned OpenCV 相机内参工具
│   └── calibrate_intrinsic.py
├── imu/                            # IMU Allan Variance 包装文档和配置
│   ├── allan_variance_ros2/         # 第三方 Autoliv-Research 离线工具
│   ├── config/
│   └── output/
├── lidar_imu/                      # LiDAR-IMU 标定包装文档和配置
│   ├── LiDAR_IMU_Init/              # 第三方 HKU-MARS ROS1 工具
│   ├── config/
│   ├── output/
│   └── ros2_adapter/                # 结果解析/兼容脚本；非产品运行时
├── camera_lidar/                   # Camera-LiDAR 标定工具集合
│   ├── direct_visual_lidar_calibration/   # 第三方 koide3 ROS2 工具
│   ├── livox_calib_standalone/            # ROS-free Livox wrapper/CLI
│   ├── livox_camera_calib/                # 用户新增：HKU-MARS Livox targetless
│   ├── livox_camera_lidar_calibration/    # 用户新增：Livox 官方 target-based
│   └── mlcc/                             # 用户新增：HKU-MARS 多 LiDAR/Camera
├── apply_calibration.py            # 将标定结果写入 LingTu 配置
├── verify.py                       # 标定配置一致性检查
└── README.md
```

## 运行边界

- 产品运行时不依赖 ROS、rclpy、roslaunch 或 rosbag。
- `allan_variance_ros2`、`LiDAR_IMU_Init`、`direct_visual_lidar_calibration`、`livox_camera_calib`、`livox_camera_lidar_calibration`、`mlcc` 都应视为离线标定/对比工具。
- ROS1/ROS2 环境建议放在专用 Ubuntu 工位、容器或 WSL2，不要安装到 ThunderV4/S100P 的产品控制栈中。
- 标定完成后的交付物是 YAML/JSON/TXT 参数文件，以及由 `apply_calibration.py` 写入的 LingTu 配置。

## 出厂标定 SOP

### Step 1: 相机内参标定

打印 9×6 棋盘格，在相机前多角度展示。

```bash
python tools/calibration/camera/calibrate_intrinsic.py auto --device 0

# 或分步执行
python tools/calibration/camera/calibrate_intrinsic.py capture --device 0 --out calib_imgs/
python tools/calibration/camera/calibrate_intrinsic.py calibrate --images calib_imgs/ --out tools/calibration/camera/output/camera_calib.yaml
```

输出：`tools/calibration/camera/output/camera_calib.yaml`。

### Step 2: IMU 噪声标定

将机器人静置在稳定平面，采集 2–3 小时 Livox IMU 数据，再用 Allan Variance 拟合噪声模型。

```bash
# 从仓库根目录执行；pushd/popd 避免后续仓库相对路径失效
source /opt/ros/humble/setup.bash
pushd tools/calibration/imu/allan_variance_ros2 >/dev/null
colcon build --packages-select allan_variance_ros2
source install/setup.bash
popd >/dev/null

# 薄入口负责采集、分析和写入；默认采集 3 小时
bash scripts/hardware/run_allan_variance.sh all
```

输出默认位于 `$HOME/data/imu_calib/<timestamp>/imu.yaml`，包含 `accelerometer_noise_density`、`gyroscope_noise_density` 和 random walk 参数。需要分阶段恢复时，可运行 `record`、`analyze <bag-path>` 或 `apply <imu.yaml>`。

### Step 3: LiDAR-IMU 外参标定

让机器人做 8 字运动，充分激发各轴。LI-Init 是 ROS1/catkin 工具；ROS2 bag 只能通过离线 replay/bridge 进入该工具。

```bash
ros2 bag record /livox/lidar /livox/imu -o lidar_imu_bag --duration 60

# ROS1 环境中运行 LI-Init
REPO_ROOT="$(git rev-parse --show-toplevel)"
roslaunch lidar_imu_init livox_mid360.launch \
  config:="$REPO_ROOT/tools/calibration/lidar_imu/config/mid360.yaml"

# 解析结果
python tools/calibration/lidar_imu/ros2_adapter/parse_result.py \
  --input tools/calibration/lidar_imu/LiDAR_IMU_Init/result/Initialization_result.txt \
  --output tools/calibration/lidar_imu/output/lidar_imu_calib.yaml
```

输出：`tools/calibration/lidar_imu/output/lidar_imu_calib.yaml` 或原始 `Initialization_result.txt`。

### Step 4: Camera-LiDAR 外参标定

默认优先使用 `direct_visual_lidar_calibration` 做 target-less 标定；对于 Livox 专用对比或需要标定板的场景，可使用下面“第三方工具”表中的替代工具。

```bash
python scripts/build/build_rust_kernels.py --target camera_lidar_optimizer --release
cd tools/calibration/camera_lidar/direct_visual_lidar_calibration
source /opt/ros/humble/setup.bash
colcon build --packages-select direct_visual_lidar_calibration
source install/setup.bash

ros2 bag record /camera/color/image_raw /camera/camera_info /livox/lidar \
  -o calib_bag_01 --duration 15

ros2 run direct_visual_lidar_calibration preprocess \
  calib_bag_01 preprocessed_01 \
  --image_topic /camera/color/image_raw \
  --camera_info_topic /camera/camera_info \
  --points_topic /livox/lidar

ros2 run direct_visual_lidar_calibration initial_guess_manual preprocessed_01
ros2 run direct_visual_lidar_calibration calibrate preprocessed_01
ros2 run direct_visual_lidar_calibration viewer preprocessed_01
```

输出：`preprocessed_01/calib.json`。

### Step 5: 写入 LingTu 配置

```bash
python tools/calibration/apply_calibration.py \
  --camera tools/calibration/camera/output/camera_calib.yaml \
  --imu tools/calibration/imu/output/imu.yaml \
  --lidar-imu tools/calibration/lidar_imu/output/lidar_imu_calib.yaml \
  --camera-lidar tools/calibration/camera_lidar/output/calib.json
```

### Step 6: 验证

```bash
python tools/calibration/verify.py
```

## 第三方工具清单

| 项目 | 来源 | License | 用途 | LingTu 使用边界 |
| --- | --- | --- | --- | --- |
| `allan_variance_ros2` | [Autoliv-Research](https://github.com/Autoliv-Research/allan_variance_ros2) | BSD-3 | IMU Allan Variance | 离线噪声拟合 |
| `LiDAR_IMU_Init` | [HKU-MARS](https://github.com/hku-mars/LiDAR_IMU_Init) | GPL-2.0 | LiDAR-IMU 外参 | 离线工具；不要并入产品 runtime |
| `direct_visual_lidar_calibration` | [koide3](https://github.com/koide3/direct_visual_lidar_calibration) | MIT；可选 SuperGlue 路径另受非商业许可限制 | Camera-LiDAR target-less 外参 | 默认离线标定工具；商业流程使用手工初始化，除非另行取得 SuperGlue 授权 |
| `livox_calib_standalone` | LingTu wrapper around Livox calibration code | GPL-2.0-only（派生适配） | ROS-free Livox Camera-LiDAR wrapper | vendor/离线验证；不要并入产品 runtime 或发布包 |
| `livox_camera_calib` | [hku-mars](https://github.com/hku-mars/livox_camera_calib) | GPL-2.0-only | Livox 专用 target-less Camera-LiDAR 外参 | vendor 快照；仅离线对比，不并入产品 runtime 或发布包 |
| `livox_camera_lidar_calibration` | [Livox-SDK](https://github.com/Livox-SDK/livox_camera_lidar_calibration) | MIT | Livox 官方 target-based Camera-LiDAR 外参 | vendor 快照；离线使用，需要标定板 |
| `mlcc` | [hku-mars](https://github.com/hku-mars/mlcc) | GPL-2.0-only | 多 LiDAR + 多 Camera 联合标定 | vendor 快照；研究/离线工具，不并入产品 runtime 或发布包 |

## 标定参数最终写到哪里

`apply_calibration.py` 按输入类型写入不同目标；不是所有结果都写入 `config/robot_config.yaml`：

| 输入 | 实际写入目标 | 保持不变 |
| --- | --- | --- |
| 相机内参 | `config/robot_config.yaml` 的 `camera` 内参和畸变字段 | LiDAR/IMU 配置 |
| IMU 噪声 | `src/localization/fastlio2/config/mid360_s100p.yaml`、`config/pointlio.yaml`（存在时） | 不写 `config/robot_config.yaml` |
| LiDAR-IMU 外参 | 上述 Fast-LIO2/Point-LIO 配置中的 `r_il`、`t_il` 和可用的时间偏移 | `config/robot_config.yaml` 中机械安装的 `T_body_lidar` |
| Camera-LiDAR 外参 | `config/robot_config.yaml` 的 `camera` 位姿字段 | 机械 `T_body_lidar` |

接受标定结果前至少完成：

- `python tools/calibration/verify.py`
- Camera-LiDAR 点云投影目视检查
- SLAM 启动后的静止漂移和短距离往返检查
