# Dynablox 室内动态物体与坡道数据

本地目录：`D:\inovxio\brain\lingtu\datasets\slam\dynablox`。

**已下载 2 个完整处理后序列并逐条读取验证，合计 4,209,484,064 bytes（约 4.21 GB）。** 用于补室内动态物体、低处残影、坡道及遮挡场景；未跑算法 benchmark。

| 实际文件 | 字节数 | 点云帧数 | bag 点云时间跨度 |
| --- | ---: | ---: | ---: |
| `processed/indoor.bag` | 1,680,021,637 | 267 | 26.596 s |
| `processed/ramp_2.bag` | 2,529,462,427 | 402 | 40.091 s |

这是**处理后的地图/分割回放输入**。两个包都只有 `/pointcloud`、`/tf` 和一条 `rosout`；**没有 IMU 和人工标签话题，不能单独作为原始 LIO 完整输入。** FAST-LIO2 位姿是估计结果，不是外部位姿真值。

## 来源与场景

官方项目：[Dynablox Dataset](https://projects.asl.ethz.ch/datasets/dynablox/)，DOI [10.3929/ethz-c-000788607](https://doi.org/10.3929/ethz-c-000788607)。官方说明存于 `metadata/README.pdf`，纯文本提取副本 `metadata/README.txt`，原始录制名称记录为 `metadata/Notes.txt`；Notes 中 indoor_1/ramp_2 的原始录制标记分别为 2021-07-28-17-40-23 / 2021-07-28-17-31-52，processed 包内时间戳则为后续处理写入值，不把二者混用。

128 线 Ouster OS0，包含行人、抛掷/弹跳球、坡道滚动行李，以及大物体遮挡后的行人；全数据集覆盖室内、坡道、楼梯及多层结构。此次选择 `indoor` 和 `ramp_2` 作为室内及坡道输入；官方的动态事件清单描述整个数据集，未据此宣称每个选中包都包含全部事件。

官方列 8 个序列；归档实际为 `corridor_1`、`corridor_2`、`hg_1`、`hg_2`、`indoor`、`ramp_1`、`ramp_2`、`stairs`（页面 `h_2` 的拼写与归档 `hg_2` 不同）。完整 `processed.zip` 为 14,729,834,629 bytes，包含 FAST-LIO2 状态及去畸变点云，通常去掉启动约 1 秒；完整 `raw.zip` 为 16,225,975,934 bytes，含原始点云、IMU 和机载 VI 状态。本轮未取 raw 包。

首条 TF 已实际解码：`header.frame_id=os_lidar`、`child_frame_id=map`，且时间戳与对应点云 Header 相同。接入时必须按这组父子坐标定义核对/求逆变换，不能直接假定它存的是 `map` 父、`lidar` 子的位姿；本轮没有转换该 TF。

## 真实验证范围

- 从官方新 ETH Research Collection 公开文件端点，用 HTTP Range 读取 ZIP64 中央目录，只取上述两条完整 bag 的压缩成员；没有下载整个 ZIP。压缩流完整、展开字节数与中央目录相符、ROS bag V2.0 头正确。
- 使用项目现有 `tools.datasets.elevator_bag_to_ltu1.Ros1Bag` 完整读取两个包的所有 chunk/消息；各话题 bag 时间无倒退；检查每帧 PointCloud2 数据长度和布局，统计所有点数。点云首帧为 128×1024、48 bytes/point、`os_lidar` 坐标系；字段含 XYZ、intensity、t、ring 等。
- 具体来源 URL、Range、大小与状态见 `download-manifest.json`；所有官方文件地址见 `metadata/official-files.json`；逐包话题及首帧 TF/点云结构见 `metadata/indoor-inspection.json`、`metadata/ramp_2-inspection.json`。
- 旧 ETH robotics 下载端点 HTTPS 握手 EOF；新端点曾 HTTP 429，等待后恢复 HTTP 206。此历史问题已恢复，不代表当前数据缺失。
- 未转换 LTU、未跑算法、未修改生产代码或传感器标定。processed 点云已去畸变，后续接入不能当作未处理的 raw cloud 重复去畸变。

## 标签与使用边界

**官方已读说明和所取 bag 未提供逐点人工动态状态/地面标签。** 这些数据可以用于同源回放和现象检查，不能直接报告有真值的动态清理/地面分割准确率；TF 估计也不能作为 SLAM 真值。相关 DOALS 的逐点动态物体索引主要依外观类别定义，也不能混称每时刻真实运动状态。

许可证：ETH 仓储标明 **In Copyright - Non-Commercial Use Permitted**（[InC-NC 1.0](http://rightsstatements.org/page/InC-NC/1.0/)）。这不是宽松开源数据许可，不推定允许商业使用或随产品再分发；如需商业用途须按权利声明另行取得许可。
