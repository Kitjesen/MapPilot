# AIST 平墙退化数据集（Livox Avia）

本地目录：`D:\inovxio\brain\lingtu\datasets\slam\aist-flatwall-avia`。

- 官方发布：[Zenodo 7641866](https://zenodo.org/records/7641866)。
- 作者说明：[Flatwall experiment dataset](https://staff.aist.go.jp/k.koide/projects/glimsupp/flatwall.html)。
- DOI：`10.5281/zenodo.7641866`；作者 Kenji Koide / AIST。
- 许可：官方记录为 CC BY 4.0；保留作者、来源与许可说明。
- 传感器：Livox Avia 点云和 IMU；相机观察 AprilTag 生成轨迹参考。
- 用途：短时完全/强烈几何退化、弱方向诊断、IMU 预测保持、结构恢复。

原始数据位于 `raw/flatwall_01.bag` 至 `raw/flatwall_08.bag`；官方真值包位于 `raw/gt.tar.gz`。下载记录见同目录的 `download-manifest.json`，官方原始元数据保存在 `metadata/zenodo-record.json`。

当前状态：**9 个发布文件全部下载完成，共 1,034,422,018 字节**。8 个 bag 已全量解析、转换为 LTU1 并完成冻结 LTLIO 基线回放。GT 包内容检查后已解压到 `ground_truth/gt/`。原始检查结果在 `metadata/bag-inspection.json`；2026-10-19 的 16 次真实回放、轨迹和条件精度结果在 `build/scene-requirements-20261019/aist/`。这些是平墙退化试验，不能代替长走廊、车库回环或玻璃场景验收。

实际输入如下，三个 topic 的 `frame_id` 均为 `livox_frame`：

| Topic | 消息类型 | 用法 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver/CustomMsg` | 优先输入；逐点 `offset_time` 为纳秒 |
| `/livox/imu` | `sensor_msgs/Imu` | 原始加速度量级需在转换前核清 |
| `/livox/points` | `sensor_msgs/PointCloud2` | 同扫描另一表示，不与 CustomMsg 重复融合 |

| 序列 | LiDAR 帧 | IMU 条数 | GT 样本 | LiDAR 时长 |
| --- | ---: | ---: | ---: | ---: |
| `flatwall_01` | 186 | 3755 | 729 | 18.5 s |
| `flatwall_02` | 137 | 2775 | 466 | 13.6 s |
| `flatwall_03` | 145 | 2935 | 568 | 14.4 s |
| `flatwall_04` | 132 | 2674 | 493 | 13.1 s |
| `flatwall_05` | 141 | 2861 | 552 | 14.0 s |
| `flatwall_06` | 89 | 1801 | 347 | 8.8 s |
| `flatwall_07` | 90 | 1820 | 352 | 8.9 s |
| `flatwall_08` | 122 | 2469 | 477 | 12.1 s |

真值文件为 `ground_truth/gt/flatwall_01.txt` 至 `flatwall_08.txt`，8 列 TUM 格式：`timestamp x y z qx qy qz qw`，数值有限、时间严格递增，并与对应扫描时间相交。

## 离线输入与首轮结果

转换器为 `tools/datasets/aist_flatwall_to_ltu1.py`，复用已有 ROS1 bag 读取器，不依赖 ROS 或 rosbags。转换保留原始点、逐点时间、六线信息、tag 和传感器时间戳；没有裁剪点云、改变坐标、拟合时间偏差或输入真值。仅输入 CustomMsg 和 IMU，不重复融合 `/livox/points`。

原始加速度模长约为 0.99，按 Livox 原始 g 单位保留；回放配置 `acc_scale: 9.80665`，只在算法入口乘一次。[Livox SDK 协议](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol)给出了该单位。不能把 catalog 的 `m/s²` 声明直接当作原始数值单位。

作者说明明确 AprilTag 参考轨迹属于 LiDAR。当前回放采用 [FAST_LIO 官方 Avia 标称外参](https://github.com/hku-mars/FAST_LIO/blob/main/config/avia.yaml)：`imu_from_lidar` 的旋转为单位阵、平移为 `[0.04165, 0.02326, -0.0284] m`，输出转换到 LiDAR 原点。**未取得本录制专用外参；以下误差属于标称标定条件下的比较，不能发布成完整校准后的绝对精度。** 未使用现场 MID360 的 RobotConfig。

冻结基线为 `/home/thunder/.cache/lingtu-entry-motion-20261019/after-build/ltlio_replay`，模式 `off`。按 GT 时间线性插值（相邻参考跨度最多 50 ms），每次回放只拟合一个 SE(3)，不拟合尺度或时差。关闭后端速度上限的试验仅是离线诊断，不是产品修复，也没有修改生产限制。

| 序列 | 默认发布姿态 / 扫描 | 默认最终状态 | 去除速度闸门的条件位置 RMSE |
| --- | ---: | --- | ---: |
| 01 | 171 / 186 | Tracking | 0.883 m |
| 02 | 121 / 137 | Tracking | 0.325 m |
| 03 | 81 / 145 | Lost：速度超限 | 6.246 m |
| 04 | 117 / 132 | Tracking | 0.485 m |
| 05 | 125 / 141 | Tracking | 0.653 m |
| 06 | 8 / 89 | Lost：速度超限 | 21.961 m |
| 07 | 74 / 90 | Tracking | 0.419 m |
| 08 | 108 / 122 | Tracking | 0.311 m |

6 段的默认/离线去闸门轨迹相同；03 和 06 去掉闸门后仍严重发散，说明不能靠放宽速度限制解决。`Tracking` 也不等于位置准确。

06 还有明显的初始化混杂：前 200 个 IMU 样本约 0.989 s 内，参考位移 0.438 m、转角 21.87°，并不满足静止均值初始化的前提。证据在 `initialization-motion.json`。8 个文件的原始时间相继连续，GT 跨文件边界的位置差约 0.5–6.9 mm。不能把独立片段启动失败全部归因于平墙退化。

已补充一次 **104.2 s 连续串联回放**：保留所有原始时间和观测，只重新编号 LTU 包序号；不在片段边界重置状态，不向算法喂入 GT。默认速度限制下，1042 帧扫描产生 1027 个姿态，全程 Tracking、0 接收/处理错误、0 丢帧；03/06 的独立启动失败没有重现。可是整段只使用一次 SE(3) 对齐后，位置 RMSE 仍为 **7.690 m**、最大 **12.093 m**，有明显累积漂移，**不能以全程 Tracking 宣布通过退化场景验收**。结果和整段图分别为 `continuous-evaluation.json`、`continuous-trajectories.png`。

输入中有 68 帧存在很小的原始逐点时间倒序，最大回退 118642 ns（约 0.119 ms）；每帧最后一个点仍为时间最大值，因此没有扫描结束时间截短。当前基线忠实保留原顺序；该现象不能直接解释米级漂移。

同一天新冻结 D1 二进制 `/home/thunder/.cache/lingtu-scene-requirements-20261019/build/ltlio_replay` 对同一连续输入、同一配置复测：1027 姿态的 TUM 文件与旧基线逐字节相同，最大位置差为 0；1027 帧都有几何诊断，其中 313 帧标为 1 个弱模态、714 帧未标退化，`update_applied` 全部为 false，0 错误/丢帧。证据为 `d1-comparison.json` 和 `d1-diagnostics.png`。这证明 D1 诊断接线没有改变轨迹，也明确它还没有修复此例的累积漂移；相对局部几何阈值尚未经过现场标定，不能把未报警的帧解释为绝对位置准确。

复现：

```powershell
python -m tools.datasets.aist_flatwall_to_ltu1 datasets/slam/aist-flatwall-avia/raw/flatwall_06.bag build/scene-requirements-20261019/aist/data
```

转换器拒绝覆盖已有结果。WSL 回放命令逐次保存在 `*.command.json`，汇总为 `results.json`；`evaluation.json` 保留匹配点数、时间覆盖、误差和失败状态，`trajectories.png` 展示条件对齐轨迹。
