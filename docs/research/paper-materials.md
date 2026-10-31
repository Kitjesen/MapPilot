# 论文材料与实验入口

2026-10-21。按森哥要求，将当前 LingTu 工作树程序、实验记录、图表和历史产物
收集到同一个本地目录，保留原仓库相对路径。正式源码仍在仓库中维护；归档用于论文取证，
不移动运行中的文件，也不发布到 GitHub。

本机总入口：`D:/inovxio/brain/lingtu-paper/20261021/index.html`。
`latest.json` 指向最近一次完成收集的快照；每个 `runs/<UTC时间>/` 保留：

- `workspace/`：实际工作树源码、配置、测试、说明，以及已复制的实验结果与图片。
- `historical/`：2026-09-29 历史产物归档，保留原来的迁移说明。
- `inventory.csv`：每个文件的原绝对路径、归档路径、大小和复制/外部保留/排除理由。
- `catalog.json`、`index.html`：按实验查找程序、报告和图表。
- `git.json`：Git revision 和未提交状态；源码快照包含未提交文件，不能仅用 HEAD 代表本次算法。
- `summary.json`：收集体积、读取问题、复制期间变化和未遍历目录。

原始 bag/LTU/RGB-D、模型、第三方依赖和大型场景资产保留在清单中的原路径。
本目录是论文材料集合，不是脱离原数据与环境即可运行的软件发行包。
远程服务器和 WSL 中仅被历史记录引用的文件，不声称本次已下载或核验。

## 当前地图图表

| 材料 | 仓库内原位置 | 配套程序与记录 |
| --- | --- | --- |
| Corridor02 / GEODE 完整图、轨迹范围三视图 | `build/dataset-mapping-20261021/figures/{corridor,geode}/` | 同实验目录 `run.py`、`manifest.json`、`runs/`、`planning/`、`README.md`；共用绘图程序 `tools/bench/render_mapping.py` |
| GEODE 实际返回短路径 | `build/dataset-mapping-20261021/figures/geode/local_route/` | 原图为历史基线；后续约束修复及有效短路线见下一行记录 |
| 射线修正、更密 GEODE 与固定路线对照 | `build/dataset-planning-fix-20261021/` | `README.md`、`results.json`、`guarded/`、`dense-retry/`；图在 `dense-retry/figures/geode/` |
| 真实地面体素与射线解释图 | `build/dataset-planning-fix-20261021/cell-probe/measured-floor-voxel-rays.{png,pdf}` | 同目录实际查询、源点/射线和生成脚本 |
| 此轮保存覆盖、逐射线事件、重访与内存修复 | `build/map-repair-20261021/` | `README.md`、输入配置、命令、日志、源码差异、回归和失败结果 |
| 已拆分的论文规划图 | `artifacts/paper-figures-20261016/` | PNG/PDF/SVG、`export.py`、manifest 和字体检查 |
| 全局→SCAN→Follower/Executor固定地图实验 | `build/nav-dataset-execution-20261021/` | `README.md`、`results.json`、冻结输入/源码/二进制、5组成功与失败运行；`paper-note/`保留新增论文附录和Arial 10 pt图表 |
| 外部算法与当前瓶颈对照 | `build/algorithm-comparison-20261021/` | 官方Elevator-LIO/GenZ-LIO与LingTu相同参考样本比较、历史规划对照及条件差异；生成器 `tools/bench/algorithm_comparison.py`，图表、精确来源与报告同目录 |

`map3d_focus`、`topdown_focus`、`side_focus` 是轨迹范围的三维、俯视和侧视图；
不带 `_focus` 的版本显示更大范围。已声明的论文导出使用 Arial 10 pt。
历史 UI 截图和视频帧没有自动变成符合论文字号的科学图表。

## 引用结论的边界

当前两组完整原始录制已经生成基线地图。原四条请求中，短路线的离散坡度不一致已修正，
route/sweep/motion 均通过；三条长路线仍未通过。这是原生离线算法证据，未证明机器人执行。
官方 PCD 实验采用过建模自由空间，不能作为 saved-ray 建图正确性的证据。

语义、人物跟踪、原生 MuJoCo 和现场记录在材料目录分别列出。独立的真实视频识别与
独立的仿真运动，不能拼成一次已完成的端到端自然语言导航实验。
目录名中的 pass/acceptance 不代替原始报告。归档没有审阅到的科学结论明确保留为 unreviewed。

2026-10-21 新增组件结果：GEODE短路线在标准静态碰撞输入下通过理想XY反馈到达条件，
Garage正式配置仍局部阻挡；旧诊断Garage有有效跟踪但从吸附高度开始且未到达。
这些都不是同场景三维物理仿真，Z保持初始值；不能与独立地图精度或传感回放拼成实机导航成绩。
原论文 `output/pdf/lingtu-mapping-paper-20261021-expanded.pdf` 保留19:15证据截点。
`output/pdf/lingtu-mapping-paper-20261021-navigation.pdf` 在原稿后追加导航复测、
Corridor高度退化诊断和截点后GenZ v2失败结果；附录用独立D页码，原稿历史结论按截点阅读。

[算法对照与瓶颈报告](../../output/pdf/lingtu-algorithm-comparison-20261021.pdf)
单独集中展示已有的官方 Elevator-LIO、官方 GenZ-LIO 与 LingTu 共同参考样本结果，
同时保留 Slippage 配置限制、Corridor 外部实跑缺口以及903历史规划的不同约束。
它整理既有冻结结果，没有启动新的传感回放，也没有把内部GenZ适配记成官方算法成绩。
后续图表按[算法对照规范](../testing.md#algorithm-result-comparisons)同时呈现基线、失败与未测项。

## 更新材料

实验完成并写好结果后，运行已有收集工具创建新快照：

```powershell
.venv/Scripts/python.exe tools/bench/paper_bundle.py --output D:/inovxio/brain/lingtu-paper/20261021
```

这一步只复制、建立索引，不修改原实验或覆盖旧快照。引用前检查最新快照的
`errors` 和 `changed_during_copy`；正在运行的实验不作为最终冻结结果。
