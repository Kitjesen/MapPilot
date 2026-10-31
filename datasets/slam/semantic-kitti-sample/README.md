# SemanticKITTI 连续研究样本

日期：2026-10-20。**已实际下载并完成逐帧点标签配对检查，尚未运行 LingTu 分割或残影清理评测。**

## 下载范围与实际内容

| 原始序列 | 原始帧范围 | 帧数 | 段内秒数 | 点数 | 含运动标签帧数 | 运动标签点数/占比 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 00 | 000000..000199 | 200 | 20.63096 | 24,228,694 | 97 | 2,547 (0.0105%) |
| 08 | 000000..000099 | 100 | 10.29086 | 12,262,418 | 100 | 462,853 (3.7746%) |

**两段来自不同录制，分别连续，不能拼成无间断的 300 帧。** 原始帧号、各自时间戳与各自标定/pose 均保留。这只是完整 SemanticKITTI 的研究子集，不是完整序列或完整 benchmark。先取 00 作地面丰富的对照；标签活动探测后再选运动更密集的 08。选择有明确目的，不代表总体分布；探测只留数字结果在 metadata，未把无 scan 的探测标签混入正式 raw。

合计 **36,491,112 点、465,400 个实际 moving 标签点**。载荷及元数据实际 **733,851,892 字节（699.86 MiB）**，manifest 与 README 的少量开销另计。

这补上了逐点地面语义、运动/静止、物体实例的参考。车载室外场景不能替代四足室内长走廊、楼梯或低安装高度数据。

## 文件与读取约定

```text
raw/dataset/
  sequences/00/
    velodyne/000000.bin ... 000199.bin
    labels/000000.label ... 000199.label
    calib.txt / times.txt / poses.txt
  sequences/08/
    velodyne/000000.bin ... 000099.bin
    labels/000000.label ... 000099.label
    calib.txt / times.txt / poses.txt
  poses/00.txt / 08.txt
metadata/
  semantic-kitti.yaml
  semantic-kitti-dataset.html / semantic-kitti-api-README.md
  kitti-odometry.html / kitti-license-homepage.html
  *-archive-index.json
  label-activity-*.json
```

- `.bin`：小端 float32，每点 `x,y,z,reflectance`（16 字节），是该帧 LiDAR 坐标。
- `.label`：小端 uint32，4 字节/点，原始点顺序一一对应。`semantic = label & 0xffff`；`instance = label >> 16`。作者说明实例编号在同一原始序列内时序一致；两个序列的实例 ID 不互通。
- 原始语义 `252..259` 是作者标注的 `moving-*`，区别于静止 car/person 等；**不要先套普通 19 类 learning_map，它会把 moving-car 合并回 car**。原始完整类别定义见 `metadata/semantic-kitti.yaml`。
- 地面语义先分别报告 `40 road / 44 parking / 48 sidewalk / 49 other-ground`；`72 terrain` 单列。00 中前四类共 9,672,054 点、terrain 122,170；08 中前四类共 6,118,399、terrain 1,851,964。语义地面并不等同所有几何可支撑表面，合并范围需要显式说明。
- `raw/dataset/poses/00.txt` 与 `08.txt`：KITTI 官方轨迹真值。`sequences/<seq>/poses.txt`：SuMa 估计并人工闭环、用于 SemanticKITTI 标注的轨迹，**不是独立真值**。两套 pose 都在 camera 坐标系，须用对应 `calib.txt` 中 `Tr` 变换到 LiDAR，不能直接当 LiDAR pose。
- times/pose 保留完整原序列元数据，00 为 4541 行，08 为 4071 行；分别只取前 200/100 行与本地扫描对齐。每行 pose 12 值，按 KITTI 3×4 变换读取。

## 本次验证与未覆盖内容

300 对原始文件全部检查：扫描字节数为 16 的倍数、标签为 4 的倍数；逐帧点数与标签数一致；XYZ/反射率为有限数；每段时间戳严格递增；两套轨迹与时间戳均覆盖对应选段。逐段统计、类别直方图、官方来源 URL 和每个文件实际字节均记录在 `download-manifest.json`。

没有测地面 precision/recall、MOS IoU 或保存图误删/残影率，没有声称算法已通过。本样本没有 IMU、没有逐点时间戳、尚无 LTU1 转换，不能直接作为当前 LIO 原始 LiDAR+IMU 回放。逐点运动标签可给扫描级与按轨迹聚合的静/动态参考，但没有单独的“理想清理完成地图”真值。20.63 s 和 10.29 s 级的小段不能代替完整序列长期静/动态变化验证。

## 官方来源与许可

- [SemanticKITTI 数据说明](https://www.semantic-kitti.org/dataset.html)：J. Behley 等，*SemanticKITTI: A Dataset for Semantic Scene Understanding of LiDAR Sequences*, ICCV 2019。
- [KITTI Odometry 数据](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)：A. Geiger 等，*Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite*, CVPR 2012。
- 官方 scans/labels/poses/calibration ZIP URL、完整 archive 字节和所取条目实际字节见 manifest。HTTP Range 只取选段条目，未下载 84.8 GB 扫描全集；扫描和标签保持原样。
- KITTI 原数据 **CC BY-NC-SA 3.0**，SemanticKITTI 标签 **CC BY-NC-SA 4.0**。本目录作为研究评估材料保存；**离线不自动获得商业研发、训练、产品使用或再分发许可**，这些用途需另行核清授权。未代用户登录、签署协议或接受点击条款。

复现下载、标签活动探测和统计脚本保存在本次任务工作区 `work/dataset-supplement-20261020/dynamic/`。没有新增运行时依赖，没有改生产算法。
