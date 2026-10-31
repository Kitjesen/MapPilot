# TRaIL-Odom BikeTunnel2：MID-360 退化场景评测输入

本地绝对路径：`D:\inovxio\brain\lingtu\datasets\slam\trail-odom-biketunnel2`。

官方入口：<https://chiyunnoh.github.io/TRaIL-Odom/>。本次下载的是 BikeTunnel2 的 LiDAR、IMU、位置真值与静态外参子集，不是该数据集所有传感器。公开文件来源与精确下载地址见 `metadata/download-source.json`，下载状态见 `download-manifest.json`。

截至 2026-10-19，以下 5 个文件已下载，总计 **677,329,909 bytes**：

| 相对路径 | 字节数 | 内容 |
| --- | ---: | --- |
| `raw/2026-02-06-12-22-39_nuc_livox_ready.bag` | 673,997,681 | MID-360 点云与两种 LiDAR IMU 话题 |
| `raw/2026-02-06-12-22-39_cpt7_raw_imu.bag` | 2,086,435 | 外部 Honeywell HG4930 IMU |
| `ground_truth/2026-02-06-12-22-39_jetson_ap20_robot.bag` | 184,150 | AP20 棱镜位置真值 |
| `calibration/tf_static/metadata.yaml` | 779 | ROS2 bag 元数据 |
| `calibration/tf_static/tf_static.db3` | 1,060,864 | ROS2 SQLite3 静态外参 |

## 实际格式核对

检查脚本：`build/public-slam-download-20261019/inspect_biketunnel.py`；记录：`metadata/bag-inspection.json`。此次读取了 ROS1 容器连接头、索引计数及块时间范围，并完整解码 ROS2 `/tf_static` 的 CDR 消息。

| ROS1 bag | 实际话题 | 实际类型 | 索引消息数 |
| --- | --- | --- | ---: |
| `nuc_livox_ready` | `/gt_box/livox/lidar` | `sensor_msgs/PointCloud2` | 2,877 |
| `nuc_livox_ready` | `/gt_box/livox/imu` | `sensor_msgs/Imu` | 57,550 |
| `nuc_livox_ready` | `/gt_box/livox/imu_si_compliant` | `sensor_msgs/Imu` | 57,543 |
| `cpt7_raw_imu` | `/gt_box/cpt7/offline_from_novatel_logs/imu` | `sensor_msgs/Imu` | 29,027 |
| `jetson_ap20_robot` | `/gt_box/ap20/prism_position` | `geometry_msgs/PointStamped` | 3,684 |

三份 ROS1 bag 均采用 **lz4** 压缩。2026-10-19 已将读取器扩展为通过 `ctypes` 调用本机已有的 `liblz4`，没有安装 ROS、rosbags 或 Python lz4 依赖。MID-360 bag 的全部消息已解码，索引数量与逐条计数相符；位置真值已完整解码。外部 IMU 没有参与 2026-10-19 的基线，2026-10-20 的独立对照见下文。

转换入口为 `tools/datasets/biketunnel_to_ltu1.py`：选择 `/gt_box/livox/lidar` 和 `/gt_box/livox/imu_si_compliant`。实际 PointCloud2 字段为 `x/y/z/intensity` float32、`tag/line` uint8、`timestamp` float64，`point_step=26`；逐点 timestamp 是绝对 Unix **纳秒**，不是秒。该量级的 float64 固有时间量化为 256 ns，少数首点会比整数 Header 早最多 119 ns；LTU1 将这些帧的基准移到更早的首点，保留所有可表示的绝对点时刻，未截断负偏移。实际相对点时间范围为 `0–100316731 ns`。

全量转换保留 2,877 帧、37,300,421 个原始点及 57,543 个 SI IMU 样本。独立复核逐点 xyz/intensity 的 float 位模式、tag/line 和每个绝对点时刻，全部相符；每条 SI IMU 时间与 float32 序列化值也相符。原始 IMU 的加速度模中位数约 1.00209 g，SI 话题约 9.82651 m/s²；55,577 对匹配样本的 SI/raw 缩放中位数为 **9.806**。本次直接使用 SI 话题，`acc_scale: 1.0`，不进行第二次缩放。记录见 `build/scene-requirements-20261019/tunnel/input-verification.json`。

已读取的 bag 块记录时间范围（Unix 秒，非传感器 Header 时间）为：LiDAR bag `1770376959.9002788–1770377247.6929832`，约 287.79 秒；外部 IMU bag `1770376959.444317–1770377249.705111`；位置真值 bag `1770376989.2616403–1770377196.22181`，约 206.96 秒。评分需要裁剪有效真值时间段。

## 真值和外参限制

官方真值为棱镜的**位置真值**。实际连接类型是 `geometry_msgs/PointStamped`，没有姿态四元数字段，不可据此评估独立姿态真值。全部 3,684 条消息的 Header 参考系为 `leica_total_station`。真值跨度约 206.96 秒，但最长缺段为 **16.575 秒**，恰覆盖默认速度门槛触发时刻，因此不能从这段缺失数据推断当时的真实速度。`prism` 是被测棱镜的静态传感器坐标系，不同于世界参考系。

`calibration/tf_static` 应作为一个 ROS2 bag 目录保留，两文件不能拆散。SQLite 查询确认 `/tf_static` 类型是 `tf2_msgs/msg/TFMessage`，序列化格式为 CDR，共 1 条消息；完整解析得到 10,028 条 TransformStamped，其中有重复静态外参，合计 38 组不同的父子坐标变换。相关平移如下，完整四元数和重复次数见检查记录：

| 父坐标系 → 子坐标系 | 平移 xyz（m，约值） |
| --- | --- |
| `box_base` → `livox_lidar` | `(0.305535, -0.043835, -0.108916)` |
| `livox_lidar` → `livox_imu` | `(0.004185, 0.021568, -0.042292)` |
| `box_base` → `cpt7_imu` | `(0, 0, 0)` |
| `box_base` → `prism` | `(0.308838, -0.038313, -0.234944)` |

轨迹与真值比较时必须用完整刚体变换补偿 LiDAR/IMU 到棱镜的杆臂，不能只做常量位置相减。外部 HG4930 与 MID-360 内部 IMU 是不同输入方案，后续应分别声明所用话题、外参和单位，不能混为同一种硬件基线。

## 原生回放与当前结果

完整外参计算得到 IMU→棱镜的杆臂（在 IMU 系表达）为 `[-0.00105516, -0.01794135, -0.08376339] m`。评价使用 `p_prism = p_imu + R_world_imu * lever`，按真值传感器时间对估计位置线性插值、姿态 SLERP，再做一次尺度固定为 1 的 SE3 对齐。姿态没有独立真值，不能报告姿态误差；真值也没有用于改变输入、时间偏移或算法参数。

| 回放设置 | 位姿数 | 真值覆盖 | 位置 ATE RMSE | 结束状态 |
| --- | ---: | ---: | ---: | --- |
| 冻结基线，默认 3 m/s 速度门槛 | 1,238 | 1,538 / 3,684 | 6.404 m | `LOST: ltlio_velocity_out_of_bounds` |
| D1 仅诊断，同一默认设置 | 1,238 | 1,538 / 3,684 | 6.404 m | 同上 |
| 冻结基线，仅离线关闭速度门槛 | 2,862 | 3,684 / 3,684 | 44.402 m | `TRACKING`，但严重漂移 |
| D1 仅诊断，仅离线关闭速度门槛 | 2,862 | 3,684 / 3,684 | 44.402 m | 同上 |

同配置的 D1 与基线轨迹逐字节一致，确认 D1 没有改变估计结果。完整离线诊断有 2,862 个可用快照，59 帧报退化、最大 2 个自由度，`update_applied` 始终为 false；默认速度门槛触发前没有报退化。检测结果没有逐帧人工真值，不能把这些数字当成退化召回率或有效补偿的证明。

该数据存在明显定位失败，不能将“完整输出并仍为 TRACKING”称为定位正确。全真值对齐本身也使用了评分位置：作为更严格的补充，只用最初 10 秒真值对齐、后续留出评价时，完整轨迹 RMSE 为 **70.259 m**。当前结果应推动继续分析沿隧道方向的匹配、观测信息与漂移；不是本项目地下车库或 ≥30 m 走廊往返 5 次的现场验收。

复现和证据：

```powershell
python -m tools.datasets.biketunnel_to_ltu1 datasets/slam/trail-odom-biketunnel2 build/scene-requirements-20261019/tunnel/new-input
python build/scene-requirements-20261019/tunnel/verify_input.py
python build/scene-requirements-20261019/tunnel/evaluate.py
```

WSL 回放命令、冻结二进制、完整配置和运行摘要分别保留在 `tunnel/{baseline,d1,baseline-core,d1-core}-command.json`；位置评价见 `tunnel/evaluation.json`，图见 `tunnel/comparison.png`。生产代码的速度限制未因本实验改变。

## 2026-10-20 外置 IMU 独立对照

全部 29,027 条 `/gt_box/cpt7/offline_from_novatel_logs/imu` 已实际解码，frame 为 `cpt7_imu`，约 100 Hz，Header 时间无逆序/间断，原值使用 SI 单位。数据集/论文的硬件命名与公开默认话题分别记录，实际执行源以该 topic/frame 为准。完整 TF 将 LiDAR 变换到此外 IMU，评价采用外 IMU 系的棱镜杆臂 `[0.308838,-0.038313,-0.234944] m`，不能沿用内部 IMU 杆臂。

冻结同一五点 LTLIO，只更换 IMU 来源及相应外参：原 2877 帧 LiDAR、37,300,421 点、点时刻及载荷不变；按原 IMU 时间覆盖裁剪外部样本为 28,771 条，并保留 LiDAR 原交付顺序。200 样本初始化及既有过程/观测噪声保持，未标定外置硬件专属噪声，不能据此归因唯一硬件质量。

| 同条件评分范围 | 内部 IMU | 外部 CPT7 话题 IMU |
| --- | ---: | ---: |
| 默认运行，共同 1538 个棱镜时刻 | 6.4042 m RMSE | 16.3485 m RMSE |
| 完整离线 core，共同 3684 个棱镜时刻 | 44.4023 m RMSE | 42.0396 m RMSE |

外部默认/离线 core 均输出 2852 姿态、无丢帧或处理异常、结束 Tracking，两份轨迹逐字节一致，默认速度门并未触发。其完整漂移仍严重，不能记为修复。只用最初十秒真值对齐、后段留出评价，内部/外部分别约 70.2594/71.8759 m，更不支持仅凭全轨迹对齐的小幅 RMSE 下降称精度改善。没有修改现场 IMU 配置。

执行、完整配置、数值和图见 [外部 IMU 实验](../../../build/scene-next-20261020/external-imu/README.md)。TRaIL-Odom 论文同序列表列出 FAST-LIO2 66.072 m、作者 LIO 12.352 m、加入多普勒雷达的 RLIO 0.485 m；其硬件、有效真值筛选及公开代码雷达初始化与本实验不同，不作直接排名，见 [算法源码研究](../../../build/scene-next-20261020/research/tunnel-algorithms.md)。
