# Hilti2023 Site2 Robot1：三层车库

2026-10-21 已完成全量转换和两组原生回放。关闭回环与默认回环均在约134.3 s后因速度越界 LOST，仅覆盖前2/7个检查点；完整三层建图尚未达标。来源：[官方数据页](https://hilti-challenge.com/dataset-2023.html)、[Robot硬件说明](https://github.com/Hilti-Research/hilti-slam-challenge-2023/blob/main/documentation/hardware/Robot.md)。[总索引与论文](../../README.md)。

| 内容 | 本地文件 / 实测情况 |
| --- | --- |
| 原始录制 | `raw/site2_robot_1.bag`，67,922,363,685字节，699.773117952 s |
| LiDAR | RoboSense BPearl，`/rslidar_points`共6996帧 |
| IMU | XSens MTi-600，`/imu/data`共139949条，实测约200 Hz，加速度为m/s² |
| 运动旁证 | `/track_odometry`共69046条，官方仅说明基于履带速度的二维估计、软件同步；未纳入本次LIO输入，不作为独立真值或已核实原始编码器 |
| 标定 | `calibration/`下5个官方Robot文件，复用已有标定包；不使用Site1手持标定 |
| 位置参考 | `ground_truth/site2_robot_1.txt`，7个独立测量点；恒等四元数为占位符，无独立姿态真值 |

BPearl点云为`point_step=26`：XYZ/intensity为float32，ring为uint16，timestamp为float64绝对秒；Header对应最晚点。完整转换输出`converted/site2_robot_1.ltu`（9,552,217,364字节），6996云/139949 IMU；过滤5,464,222个非有限XYZ，保留397,697,666个有效几何点及6个原始NaN扫描末端标记。后者仅保留真实扫描结束时刻，不产生有限回波。逐记录、点、时间、IMU及EOF检查通过，见[转换记录](metadata/conversion-full-20261021.json)。

回放固定官方Robot外参、零时间偏移与m/s²加速度，不使用Site1手持标定。两个配置仅切换回环时间门槛，输出1343个姿态且逐字节一致，接受回环0。前两个GCP间距真值28.284083 m、估计27.546558 m，差−0.737525 m；少于3点，不报告SE(3)拟合精度。GCP按官方Robot规则逐点变换实测LiDAR坐标，不使用手持杆尖固定偏移。

两张部分地图各自通过原生saved-ray构建与两条三维路线检查（3.684 m、18.887 m），其余四条因轨迹不覆盖而未测试。路径、连续扫掠及坡度检查通过仅表示估计地图上的离线几何结果，起点吸附接入段与实际机器人执行尚未验证；不能作为三层或跨层导航通过。[完整实验记录](../../../build/garage-validation-20261021/README.md)保留输入、冻结二进制、回放、故障诊断、GCP和导航证据。

两项隔离对照未采用：仅重算缓存平面残差只将速度越界延后3.6 s，前2个GCP间距误差却增至+25.774 m；仅替换官方IMU噪声密度使失锁提前到27.7 s、覆盖0/7 GCP。两项没有叠加，也未修改生产默认算法或参数。

同Linux平台另跑145 s固定原上游对照：当前和上游均出现速度>3 m/s，上游只晚3.4 s，前2个GCP间距误差还更大。它没有原生LOST策略，比较使用同一观察门槛；因此未回滚，也不将本次失败笼统归因Windows或迁移。

`/track_odometry`全段线速度最大1.149 m/s，130–145 s窗最大0.744 m/s，LIO首次越界附近约0.698 m/s；不支持真实高速误触3 m/s门槛的解释。其时间/来源有限制，未据此放宽门槛或伪造真值精度。

[挑战论文](https://arxiv.org/abs/2404.09765) §IV-B说明Robot1沿螺旋路线下降、没有直接回访；可测跨层稳定性，不能据此证明回环正确。数据没有逐点动态/静态标签。

初始下载检查覆盖完整bag索引及前2帧点云/200条IMU载荷，见 [初始检查报告](metadata/bag-inspection-20261020.json)；10-21全量转换及回放检查见上文。来源revision、下载区间、镜像记录和精确字节见 [下载清单](download-manifest.json) 与 [完成记录](metadata/download-completion.json)。数据许可：CC BY-NC-SA 3.0。
