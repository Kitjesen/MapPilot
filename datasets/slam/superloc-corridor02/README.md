# SuperLoc Corridor02 长走廊数据

本地目录：`D:\inovxio\brain\lingtu\datasets\slam\superloc-corridor02`。

官方来源：[SuperLoc dataset](https://superodometry.com/superloc.html)。官网列出的 Corridor02 行程约 690 m、时长约 893 s，提供 RGB/LiDAR/IMU 原始包、标定、GT 地图与 TUM 轨迹。当前用于长走廊退化基线；不据此宣称已满足本项目现场指标。

| 文件 | 本地位置 | 官方字节数 | 状态 |
| --- | --- | ---: | --- |
| 原始数据 ZIP | `raw/corridor02.zip` | 15,589,213,681 | 已完成并解压，ZIP 完整读取通过 |
| 轨迹 GT | `ground_truth/corridor02_gt.txt` | 475,805 | 已下载 |
| GT 地图 | `ground_truth/corridor02.pcd` | 66,056,109 | 已下载 |
| 外参与 IMU 噪声 | `calibration/corridor02_extrinsics.yaml` | 860 | 已下载 |
| 相机内参 | `calibration/corridor02_Intrinsics.yaml` | 478 | 已下载 |

下载 URL、官方体积和已完成状态见 `download-manifest.json`；原始下载清单位于 `metadata/download-source.json`。实际解压的 `raw/corridor02.bag` 为 24,494,797,838 字节。历史后台下载记录在 `metadata/download-job.json`，当前不需要重新启动下载。

实际读取到 `/velodyne_points` 的 8,859 帧与 `/imu/data` 的 178,499 条 IMU。点为 XYZ/intensity、16 线 ring、相对扫描时刻的 float32 秒；IMU 加速度模长中位数约 9.86 m/s²。原始包还含相机，但本次不将图像输入 LTLIO。

转换入口为 `python -m tools.datasets.superloc_corridor_to_ltu1 raw/corridor02.bag converted/corridor02.ltu`。已转换 215,963,143 点，约 893.51 s；保留点几何/强度/实际逐点时刻并按时间稳定排序。当前生产 intake 是 Livox 六线筛选，所以此离线载体将 line/tag 置零以保留全部 Velodyne 环线；原始 ring 数量写入 `.conversion.json`。这不是生产 Velodyne 驱动适配，也没有修改 RobotConfig。

回放使用官方 `laser_to_imu` 和 IMU 噪声配置，保留生产的 3 m/s 速度门槛与普通三维估计。完整 GT 生成质量和参考点仍需核对，精度比较明确标为条件结果。

轨迹文本已检查：5,522 个 TUM 样本、8 列、数值有限、时间严格递增，覆盖约 892.669 s。具体时间范围见 `metadata/gt-inspection.json`；此检查不证明传感器到 GT 的变换已经对齐。

完整约 893.51 s 已完成冻结 D1 基线与仅重算迭代残差的实验候选回放：均为 8843 姿态、Tracking、零输入错误/丢帧。排除参考间隔 >0.5 s 后共 8038 个时刻，参考路径约 693.83 m。一次无尺度 SE(3) 配准后的条件位置 RMSE 为 2.107 m / 1.628 m，高度分量 RMSE 为 1.457 m / 0.587 m。结果、配置、命令及包含高度曲线的图保存在 `build/scene-requirements-20261019/corridor-full-*`。

这不是需求书第 10.4 节的始末纵向漂移测法，也没有完成 ≥5 次目标平台往返；不可用 `RMSE/行程` 当作 <0.5% 通过。该候选在 Slippage 和 BikeTunnel2 上有回归，仍只留实验快照，没有替换生产核心。前 120 s 小样本仅有 10.40 m 行程，完整回放的结论优先。

原始数据公开可下载不等于允许随产品再分发；本目录只记录本机开发数据位置，保留官方来源。

## 2026-10-21 完整原生建图与规划

当前工作树在 Windows 构建的 `ltlio_replay` 使用同一完整载体，输出 **8,843** 个位姿、
最终 **TRACKING**，输入错误和 IMU/LiDAR 丢帧均为零。原生回放 575.761 s，
调用现有 `ISlamBackend::saveMap` 保存 20.278 s；保存 **10,644,022 点、495 关键帧**，
`patch_bundle.manifest` 完整且 dropped_count=0。本轮没有闭环图优化。

原生 MapsPipelineCore 使用保存的真实射线完成 OctoMap，分辨率 0.10 m，支撑膨胀和
自由层/自由膨胀均为零，`evidence_source=saved_rays`；没有生成假地面。第一次构建未取得
回执即中断，原因未知；保留记录后单独恢复该阶段，820.14 s 完成。

对 8,038 个有效配对时刻进行一次无尺度 SE(3) 拟合后，条件位置 RMSE **1.546 m**、
p95 **2.669 m**。参考覆盖 891.661 s。完整参考点/杆臂资格仍未认证，不能据此改写既往
基线/候选结论，亦不是现场始末漂移验收。

用生成地图执行两条固定原生全局规划：相对首个已发布 IMU 位姿 **140→180 s**、
**180→300 s** 均返回 `empty_path`，open 集耗尽而非达到最大迭代数。
日志保留支撑、机体碰撞和扫掠拒绝计数，尚未证明唯一根因；未挪动端点或缩小机体换取通过。

地图、三维查看、Arial 10 pt 独立 PNG/PDF、请求/结果/失败日志和复现命令统一见
[本轮实验记录](../../../build/dataset-mapping-20261021/README.md)。
这是本地离线组件测试，没有地图激活、Product 仿真执行或实机运动。

后续[地图与规划诊断](../../../build/dataset-planning-fix-20261021/README.md)已确认两类局部
缺陷：融合 PCD 与选中关键帧的覆盖不同；某些真实地面体素被从表面上方擦过的射线误清除。
原图固定路线在加入运动约束后仍失败，不能把支撑拒绝计数当作单一根因。射线保护修正
保留真实穿面、未知空间和机体限制；原证据重建与同起终点结果由上述记录维护。
