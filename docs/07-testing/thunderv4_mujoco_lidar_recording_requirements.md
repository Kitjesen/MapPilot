# ThunderV4 MuJoCo MID-360 录制规范

这是 ThunderV4 在 MuJoCo 中做 MID-360 雷达仿真录制的默认验收格式。

## 固定风格

- 主画面必须是 MuJoCo RGB render，不能用 Matplotlib 点云图代替。
- 机器人使用 `sim/robots/thunderv4/mjcf/thunderv4.xml`。
- 运动必须使用 `sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.pt`，report 中应显示 `policy_class=TorchScriptPolicyRunner`。
- 背景使用低干扰灰色 MuJoCo 场景，保留阴影和简单障碍物。
- 机器人本体可以为了可读性调暗，但不能改物理、关节、policy、碰撞体。
- 轨迹如果显示，必须是连续细线，不使用离散点。

## 雷达要求

- 雷达后端必须是 MuJoCo-LiDAR：`backend=mujoco_lidar`。
- 扫描 pattern 使用 `sim/assets/livox/mid360.npy`。
- 点云必须来自 `engine.get_lidar_points()` 的 XYZI raycast hit points。
- 不允许用 ground-projected pattern 当作正式雷达点云。
- 不画雷达射线，只画红色 hit points。
- 可以为了视频渲染抽样显示点数，但 report 中要保留真实 scan 点数。
- XYZI 的 intensity 是当前仿真的反射强度代理，不是材质真实反射率。
- 当前 intensity 模型：`180/(1+(range_m/25)^2)+N(0,3)`，并裁剪到 `[1,255]`。
- report 必须写明 `mature_lidar_backend_verified=true`、`fallback_used=false`、`ground_projected_pattern=false`、`rays_drawn=false`、`point_source=engine.get_lidar_points() XYZI raycast hits` 和 `intensity.min/p50/mean/p95/max`。

## IMU 与 SLAM 要求

- MuJoCo engine 必须能读到 IMU 状态：`imu_gyro`、`imu_projected_gravity` 和 `imu_linear_acceleration`。
- `MujocoDriverModule` 必须发布 runtime `raw_scan` 和 `imu` 输出口，对齐 `/lidar/raw_frame` / `TOPICS.raw_lidar_points` 与 `/imu/raw` / `TOPICS.imu`，frame 使用 `lidar_link`。
- 录制脚本本身不是运行时 topic 发布器；它只在 report 中记录 IMU 采样和 driver 是否具备 IMU 输出。
- 仅有录制视频不等于 SLAM 输入完整接通。直接跑 Fast-LIO 仍要验证 raw LiDAR frame、IMU、时间同步和 DDS/portable adapter 输入窗口。
- report 中 `ready_for_direct_slam=false` 时，不能宣称 SLAM 已经可以直接跑。

## 接地要求

- 轮子可见部分使用 `FR_wheel`、`FL_wheel`、`RR_wheel`、`RL_wheel` 四个 wheel cylinder。
- 四个旧 `*_foot_visual` mesh 会穿地，录制时不作为可见轮子使用。
- report 必须输出 `visible_wheel_clearance_min_m`。
- `visible_wheel_clearance_min_m` 必须大于等于 0，不能出现可见轮子嵌入地面。

## 生成命令

```powershell
$env:PYTHONPATH='D:\inovxio\brain\lingtu\third_party\MuJoCo-LiDAR\src;D:\inovxio\brain\lingtu\src;D:\inovxio\brain\lingtu'
python sim\scripts\record_thunderv4_mid360_policy.py
```

默认输出到：

```text
artifacts/mujoco_thunderv4_mid360_policy_<timestamp>/
  thunderv4_mid360_policy_browser.mp4
  thunderv4_mid360_policy_preview.gif
  first_frame.png
  mid_frame.png
  report.json
```

## 当前验收样例

```text
artifacts/mujoco_thunderv4_mid360_policy_20260704_074304/
```

关键结果：

- `.pt` policy 已使用：`TorchScriptPolicyRunner`。
- 机器人低速运动距离：约 1.08 m。
- 可见轮最低离地：约 1.1 mm。
- 雷达点源：真实 MuJoCo-LiDAR hit points。
- 未使用射线或地面投影 pattern。
- IMU/raw LiDAR：engine 内部可用，`MujocoDriverModule` 已具备 runtime `raw_scan`/`imu` 输出口；直接 SLAM 仍需 raw LiDAR/IMU 输入窗口验收。
## No Python SLAM

Do not write or ship Python SLAM for MuJoCo validation. Python may render
MuJoCo, record videos, publish simulated raw MID-360/IMU frames, and collect
diagnostic reports. Pose estimation, map building, and relocalization must be
provided by native C++ SLAM/localization (`lingtu_slam_cyclone_runtime` or an
explicit external native service). Reports must keep `no_python_slam=true` when
this path is used.
