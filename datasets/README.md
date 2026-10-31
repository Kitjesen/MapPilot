# 本地数据集

更新：2026-10-21。根目录：`D:/inovxio/brain/lingtu/datasets`。本页统一记录数据、剩余工作和相关论文；具体字段及转换注意事项见各数据集说明。

## 数据索引

| 数据与本地说明 | 用途 | 当前状态 |
| --- | --- | --- |
| [AIST Flat-wall / Avia](slam/aist-flatwall-avia/README.md) | 平墙退化与恢复 | 8段已转换、回放；连续104.2 s条件位置RMSE约7.69 m，未修复 |
| [SuperLoc Corridor02](slam/superloc-corridor02/README.md) | 长走廊、地图定位 | 10-21 当前Windows原生完整TRACKING；保存1064万点及saved-ray地图，条件RMSE1.546 m；两条固定路线均empty_path；旧基线/候选结果保留 |
| [TRaIL-Odom BikeTunnel2](slam/trail-odom-biketunnel2/README.md) | MID-360重复隧道 | 内部IMU完整core RMSE44.40 m；外部CPT7话题IMU对照完整Tracking但仍42.04 m，共同前段回归，未修复；论文额外传感器与本机结果分开 |
| [GEODE Stairs Alpha](slam/geode-stairs/README.md) | 楼梯、多层表面 | 10-21当前Windows完整TRACKING/3430姿态，保存624万点和saved-ray地图，条件RMSE0.769 m；长路线失败、短路线几何/扫掠通过但坡度诊断失败；旧WSL749姿态后LOST保留，平台差异未隔离 |
| [Hilti2023 Site1 Handheld5](slam/hilti-stairs/README.md) | 楼梯与高度变化 | 复用外部17.14 GB原包及已有LTU，1594帧点云、3个位置检查点 |
| [Hilti2023 Site2 Robot1](slam/hilti-garage/README.md) | 三层地下车库、坡道 | 全量转换9.552 GB LTU；关闭/默认回环均134.3 s后LOST，覆盖2/7 GCP、接受回环0；部分地图两条三维路线几何检查通过，跨层尚未覆盖 |
| [SemanticKITTI 00/08片段](slam/semantic-kitti-sample/README.md) | 地面语义、运动点和静态误删 | 约734 MB，共300对扫描/标签、46.54万运动标签点；两段分别回放，无IMU/逐点时间，待评测 |
| [Dynablox indoor / ramp_2](slam/dynablox/README.md) | 室内动态、遮挡、坡面 | 完整4.209 GB、669帧点云；格式检查通过，待转换/评测；估计位姿、无IMU或人工逐点标签 |
| [3DRef Seq1](slam/3dref-sequence/README.md) · [旧三帧示例](slam/glass-reflection-demo/README.md) | 连续玻璃、镜面、虚反射和透射背景 | 完整21.735 GB归档，三雷达各3732帧，共11196帧、375.282 s；结构/时间配对通过，740帧数值检查；无IMU/逐点时间，待清理对照 |

每套数据的 `download-manifest.json` 保存来源、路径、字节与完成状态，`metadata/` 保存检查记录；大文件保留本机、不纳入Git。下载、格式检查、回放、精度评测分别记录，表中条件误差不作为现场验收。

转换使用数据集自身标定与IMU单位。GEODE楼梯参考来自地图配准，Dynablox/3DRef位姿来自算法估计；Hilti仅有稀疏位置点。这些参考均不能替代独立密集轨迹真值或逐点清理标签。

2026-10-21 两组完整数据已跑通原生回放、保存与真实射线地图构建，三维图和固定规划记录见
[本轮实验](../build/dataset-mapping-20261021/README.md)。地图没有闭环图校正；三个长路线
`empty_path`，独立短路线仍有运动诊断失败。此处“完整TRACKING”仅指本轮Windows运行，
不覆盖旧WSL失败，也不证明原生仿真或实机导航通过。数据和结果均留在本机。

## 剩余工作

1. **地面：** 定位窄视野合成用例漏分，再用SemanticKITTI按距离/视野评价。Patchwork++原协议包含road、parking、sidewalk、other-ground、lane-marking、terrain；vegetation只在评分时忽略，目标地面定义另报。
2. **残影：** 在Dynablox、GEODE、Hilti选地面/墙柱/踏步/动态核验区，用相同扫描与位姿同时计漏删和误删；单帧语义标签不能直接充当融合地图的理想干净真值。
3. **定位：** 继续分析Corridor02/BikeTunnel2已知漂移与丢失，Site2补充坡道压力测试；保留失败结果，并检查位姿误差对清理的影响。
4. **目标机采集：** 同一安装/标定下记录无人静态场景与人员、推车经过后离开的对照，保留同步扫描/IMU、逐点时间、实测离地高度和独立参考。这是目前仍缺的专项输入。

## 相关论文

以下记录原论文实际使用的数据与可借鉴机制，未在本机复现论文成绩。数据集发布者重跑的基准，不等于各基线原论文使用过该数据集。

| 文献 | 实验数据与证据 | 借鉴点及前提 |
| --- | --- | --- |
| [Hilti SLAM Challenge 2023](https://arxiv.org/abs/2404.09765)（RA-L 2024） | §IV-B、VI-B实际分析Site2；Robot1无直接回访 | 冠军结合AdaLIO、Quatro/图优化、多会话GICP及结构约束；成绩属于完整系统，不能归给单个平面因子 |
| [AdaLIO](https://arxiv.org/abs/2304.12577)（2023） | §IV原论文用Hilti-Oxford 2022；2023参赛系统另见上一项 | 自适应采样/法向保留有效对应；同墙面多留点不会新增走廊纵向约束 |
| [GEODE](https://doi.org/10.1177/02783649251344967)（IJRR，2025在线/2026印刷） | §4–5、Table 6有真实楼梯/隧道基准 | 按传感器与退化方向统计失败；楼梯PALoc/Leica地图配准参考有依赖性，Avia楼梯参考不可靠 |
| [Dynablox](https://arxiv.org/abs/2304.10049) · [代码](https://github.com/ethz-asl/dynablox)（2023） | §VI定量用DOALS；本地indoor/ramp_2属新增定性序列 | 时空自由置信度和漂移容限；TSDF方法，薄物体/玻璃/遮挡仍有限制 |
| [DUFOMap](https://arxiv.org/abs/2403.01449) · [代码](https://github.com/KTH-RPL/dufomap)（2024） | §IV、Table I用KITTI/SemanticKITTI等，另有室内定性实验 | 保守自由空间与亚体素误差；区分离线/在线证据，兼顾静态保留与动态删除 |
| [FreeDOM](https://arxiv.org/abs/2504.11073) · [代码](https://github.com/LC-Robotics/FreeDOM)（2025） | §IV、Table III用SemanticKITTI/HeLiMOS及自采走廊、MID-360楼梯 | 增量空闲证据回查历史地图；自采楼梯不是本地Hilti/GEODE，体素PR/RR不与DUFOMap逐点SA/DA直接排名 |
| [Patchwork++](https://arxiv.org/abs/2207.11919) · [代码](https://github.com/url-kaist/patchwork-plusplus)（IROS 2022） | §IV-A、Table I用SemanticKITTI，地面定义见上文 | 自适应局部平面与候选恢复；先检查初始化、高度、坐标/分区和状态，论文平均召回率不与单个合成用例直接比较 |
| [3DRef](https://arxiv.org/abs/2403.06538)（3DV 2024） | §4.3.1描述Seq1，Table 5由数据集作者重训网络 | 分别评价材质表面、虚反射和真实透射背景；回波/训练条件与当前MID-360有差别 |
| [Li等：反射平面优化](https://doi.org/10.3390/s24154794) · [代码](https://github.com/STAR-Center/Reflection_detection)（Sensors 2024） | §6.1实际用3DRef Hesai QT64首末双回波 | 跨帧优化有限反射面再分类；双回波是关键输入，当前MID-360消息不满足该判据 |
| [GRASS](https://doi.org/10.3390/rs18020332) · [代码](https://github.com/wpshao/GRASS)（2026） | §4.2.1真实测试包含3DRef，相关结果在补充材料 | 多回波、几何分面、射线及对称/学习相似度；偏TLS离线处理，需核对密度和部署成本 |

代码许可：Dynablox/DUFOMap为BSD-3-Clause，FreeDOM为MIT，Patchwork++为BSD-2-Clause；反射平面优化/GRASS代码许可尚未核清。数据许可独立记录在各目录。

场景实现与验收进度见 [SLAM场景适配计划](../docs/design/slam-scene-adaptation.md)。
