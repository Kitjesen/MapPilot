# 位姿图后端对照：求解器与模型分开验证

后续 `.71` 已部署 [LIO-SAM 原生后端](lio-sam-migration-20260924.md)。
本文保留 `.70` 的模型对比数字；不得把这组测试数字当作 `.71` 的结果。

## .70 原生 GTSAM 实测与部署（最新）

已安装 v2.3.0-go2.20260924.70；ProductControl 启动 map/camera 成功，运行中
slamd 的实际路径为 .70/bin/slamd，内存映射确认 libgtsam.so.4.0.2。静止
SLAM TRACKING、扫描处理约 10 Hz、驱动输出零速度。未覆盖旧地图，尚无新闭环
现场精度结果。以下对照为 NX ARM 离线 25 节点合成用例，不是 903room_v2 修复。

保持同一初始位姿、25 条测量和完整信息矩阵、固定首节点：

| 方法 | 最终代价 | 最大重力偏差 | 高度跨度 | 迭代 |
| --- | ---: | ---: | ---: | ---: |
| 旧 Rust 六自由度 + 0.01 软重力 | 39.355450034 | 0.068193203 rad（3.907°） | 82.5910 mm | 150 |
| 原生 GTSAM，相同六自由度软重力目标 | 39.351572422 | 0.067847913 rad（3.887°） | 83.2896 mm | 1000，达到上限 |
| 独立 SciPy，相同六自由度软重力目标 | 39.351570048 | 0.067848783 rad | 83.2873 mm | 不比较 nfev |
| 已部署 GTSAM，固定重力四自由度 | 63.320287909 | 0 | 2.853145 mm | 3 |
| 独立 SciPy，相同四自由度目标 | 63.320287949 | 0 | 2.853145 mm | 不比较 nfev |

另将 GTSAM 旧模型的预算设为与 Rust 相同的 200 次：末代价 39.379071446、
倾斜 0.067854953 rad、高度跨度 83.1507 mm，同样触及上限。
GTSAM 旧模型组达到上限，不能称为收敛通过；只用其诊断末状态比较趋势。
旧 Rust 与该 GTSAM 末状态的代价差约 0.00388（0.0099%）、倾斜差约 0.020°、
高度跨度差约 0.70 mm，均远小于两者共同存在的约 8 cm 假高差。
GTSAM 同模型诊断使用全局旋转向量与 XYZ 变量；Rust 使用自身 SE(3) 更新。
两者目标相同，参数化、LM 阻尼策略及终止判断不同，不能称为逐步数值等价。
初始代价约 131152.678，各实现差小于 1e-4（含四元数归一化、Log 与白化舍入）。
固定重力组 GTSAM 与 SciPy 最终代价差约 4e-8；高度跨度差约 2e-13 m。
这验证该用例的实现一致性，不代表真实雷达达到该精度。

四自由度与六自由度可行域不同，不能用最终代价更低来选择旧模型。
改进主要来自匹配观测的变量建模；GTSAM 提供成熟求解实现，而非自动修正输入。
四自由度只固定原始 LIO 的 roll/pitch，仍优化 XYZ 和 yaw，地图仍为三维。
它也保留前端可能存在的重力估计误差，不能宣称消除所有漂移。

旧 Rust 数字由直接调用原有 FFI 的诊断复测，绕过生产拒绝门仅为观察失败结果；
没有重新启用旧后端。没有复制 FAST-LIO-SAM 全流程：当前使用原生 GTSAM batch LM，
因子适配、配准与连接策略仍由 LingTu 提供。无重力参考的通用接口仍保留 Rust。

证据目录：build/go2-3d-support-20260924；gtsam_numeric_probe.cpp、
gtsam_soft_probe.cpp、reference.poses、reference.constraints、
gtsam-numeric-result.txt、rust-numeric-result.txt、gtsam-soft-result.txt，
以及 deploy70-numeric-evidence.tar.gz（包含安装和运行状态证据）。

下面保留更早的研究过程；其中“当前”“尚未替换”“未部署”为当时状态。

## 前期结论

当前失败不能仅归结为 Rust 数值求解器：相同输入、相同信息矩阵、相同 SE(3)
残差与重力软约束交给独立 SciPy LM，仍出现约 3.89° 倾斜，与 NX 内核的
3.91° 接近。优先修正自由度与观测建模；单换求解库不足以解决这个复现。
这是特定失败用例的证据，不是 Rust 内核全面正确或成熟库不值得采用的结论。

## 官方实现对照

查阅日期 2026-09-24；链接为上游可变分支，未复制其代码进产品。

- [VINS-Fusion pose_graph.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/loop_fusion/src/pose_graph.cpp)：
  使用 IMU 时进入 optimize4DoF，变量为 XYZ 与 yaw，保留前端 roll/pitch。
  顺序约束来自同序列前端位姿，相连近邻最多四个；回环使用 Huber 损失，
  Ceres 稀疏求解。它是视觉惯性系统，不能直接宣称适配 Go2 MID-360。
- [VINS-Fusion pose_graph.h](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/loop_fusion/src/pose_graph.h)：
  FourDOFError 直接对三维相对平移与偏航构造残差。
- [GTSAM AttitudeFactor](https://borglab.github.io/gtsam/attitudefactor/)：
  三维状态也可以保留，但以方向观测构造二维姿态残差；这与当前投影旋转
  对数的 prior 不是完全相同的残差函数。软观测仍需要合理的不确定性建模。
- [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)：
  本轮仅使用本机已有 SciPy 的 LM 作为独立数值对照，没有安装新的产品依赖。

LingTu 当前后端使用独立扫描配准的实测因子、SE(3) Rust batch LM 与有限重力
prior，求解目标为普通加权平方误差；源码中没有回环级 Huber/Cauchy 鲁棒损失。
虽有回环几何和共识验收，但这不能代替求解时降低异常回环影响的机制。
尚未证明本次地图一定存在错误回环，因此不能把缺少鲁棒损失说成本次已证根因。

## 独立实验

输入是 constraint_assembly_test 的 25 帧合成平地往返失败用例：24 条顺序边、
1 条回环。从 NX 导出原样测量及完整 6x6 信息矩阵。不是 903room_v2 点云重建。

工具：`tools/diagnostics/compare_gravity_models.py`。独立实现相同 SE(3) Log 残差，
固定第 0 帧，使用信息矩阵的特征分解白化；比较两种变量参数化：

1. 六自由度 + sigma=0.01 的投影重力软约束。
2. 每帧 R=Rz(delta_yaw)*R_reference，仅优化 delta_yaw 与 XYZ，保持观测重力。

| 方法 | 初始代价 | 最终代价 | 最大重力偏差 | 输出高度跨度 |
| --- | ---: | ---: | ---: | ---: |
| NX Rust，当前模型 | 131153（日志取整） | 39.3555 | 0.0681932 rad | 本轮未导出 |
| SciPy LM，相同模型 | 131152.678079 | 39.351570 | 0.0678488 rad | 0.0832873 m |
| SciPy LM，固定重力四自由度 | 131152.678079 | 63.320288 | 0 | 0.00285314 m |

两组 SciPy 都报告成功。四自由度最终代价更高却符合姿态约束，进一步说明不能
只用目标函数下降证明地图几何正确。固定重力方案仍允许高度变化，另用真实
roll/pitch 和 0.6 m 高差的两帧一致测量验证：零初始残差、高差和倾斜均保留。

此工具没有运行 VINS、Ceres 或 GTSAM；四自由度组保持 LingTu 的测量及残差，
只比较变量自由度，不能称为与 VINS 完整系统等价或正式性能基准。
评估 nfev 不与 Rust 迭代次数比较，也不据桌面 SciPy 耗时推断 ARM 性能。

复现：

```powershell
C:/Users/99563/miniconda3/python.exe tools/diagnostics/compare_gravity_models.py build/go2-3d-support-20260924/reference.poses build/go2-3d-support-20260924/reference.constraints build/go2-3d-support-20260924/gravity-model-comparison.json
C:/Users/99563/miniconda3/python.exe build/go2-3d-support-20260924/check_reference_model.py
```

## 修复方向和边界

优先在生产后端采用与观测模型一致的四自由度优化，保留原始 LIO 重力方向及
真实 XYZ。不能先允许六自由度倾斜，再仅把输出姿态改回去：倾斜已经影响求出的
位置，必须在求解变量/更新流形中约束。重力参考必须独立于上轮优化结果。

顺序连接应重新评估如何消费经过质量筛选的 LIO 连续观测；当前只有独立扫描
配准可入图，恢复补丁尚余 25 帧未连接。不能照搬 VINS 权重或把 Fast-LIO 的
单帧边缘协方差直接当成两帧相对协方差，前后时刻有关联。

之后评估回环鲁棒损失及成熟 C++ 后端替换。替换要使用相同输入、自由度和验收
条件对照，并完成 ARM 构建与回放；不预设换库必然更准。本轮没有新增产品依赖，
没有修改生产优化器或部署候选，也未修复旧地图。此前 4/5 ARM 测试状态未改变。
