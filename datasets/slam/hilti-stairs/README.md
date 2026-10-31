# Hilti 2023 真实楼梯：已有录制补入索引

2026-10-20 核对。官方序列：Site 1 / Handheld / Stairs，`site1_handheld_5`。

**数据已存在，这次没有重复下载 17.14 GB 原始 bag。** 实际存储：
`D:/inovxio/datasets/lingtu/hilti2023_stair/`。本目录是统一数据集入口和检查记录，原文件仍在原位置。

- [官方数据页](https://hilti-challenge.com/dataset-2023)
- [官方手持硬件/序列说明](https://github.com/Hilti-Research/hilti-slam-challenge-2023/blob/main/documentation/hardware/Handheld.md)
- [发布者原始文件](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2023/tree/8e2a7a2de63670730a322d74090c1a5ea4b641eb)

## 实际输入

| 项目 | 核对结果 |
| --- | --- |
| 原始包 | `site1_handheld_5.bag`，17,140,264,234 字节 |
| 时长 | 159.42123477 秒 |
| LiDAR | Hesai PandarXT-32；`/hesai/pandar`；1,594 条 PointCloud2 |
| IMU | Alphasense；`/alphasense/imu`；63,633 条 Imu；实测约 400 Hz |
| 标定 | 原目录 `calibration.zip`、`calibration.yaml`、`calibration_lidar.yaml`、`calibration_imu.yaml`、`calibration.txt` |
| 已有规范输入 | `stairs.ltu`，2,361,155,020 字节；先前完整源流校核通过 |

本轮重新读取完整 bag 索引，读取前 2 帧点云与 200 条 IMU 载荷：格式有效、字段可解析；不是完整载荷重扫或新的算法评价。见 `metadata/bag-inspection-20261020.json`。完整源流校核是既有结果，见 `build/stair-validation-20261018/conversion-full-check.json`。

## 真值和用途边界

`site1_handheld_5.txt` 只有 **3 个独立测量的钢针位置**，不是密集轨迹；四元数恒等值是占位符，不能用于姿态误差。官方手持评价需要将 IMU 轨迹变换到钢针尖端，杆臂 `[0.059, -0.00855, 0.1964] m`。不能对三个点插值后宣称楼梯全程 ATE。

可用于检查真实楼梯、楼层变化、墙面和地面几何保留，以及位姿稳定性。不能当作动态物体逐点真值，也不能证明四足上下楼能力。

历史离线回放已有记录与失败：`docs/research/elevator-native.md`；本轮仅补统一入口并重新检查输入，**没有新增回放或精度结论**。

许可：发布者数据为 **CC BY-NC-SA 3.0**，本地研究下载不意味着商业再分发授权。原始来源、精确字节及校核路径见 `download-manifest.json`。
