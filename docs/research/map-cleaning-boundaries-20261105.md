# 地图清理：正式路径、研究参考与精简

2026-11-05。范围为保存时清理及其研究入口，没有更换 SLAM、OctoMap、导航算法或实机部署。

## 选择依据

正式保存只调用 `prune --map-dir ... --overwrite --apply`，并没有依次运行
ERASOR2、Patchwork++、DUFOMap 和 FreeDOM。这些名字混在源码目录、可选构建和检查脚本中，
曾让研究参考看起来像正式依赖。本次明确分开：`src/maps/prune` 是产品，`research/maps`
是独立实验。

| 对比 | 当前 prune | ERASOR2 |
| --- | --- | --- |
| 方法与输出 | 按后续实测射线反证和局部表面证据，分流原始地图点 | 使用地面、实例等信息处理子图，输出重建并体素化的静态地图 |
| 已有证据 | 本地行为测试、185 帧同源输出对照、玻璃透射反例 | 上游有 SemanticKITTI 等数据的静态保留/动态移除/F1 结果 |
| LingTu 正式调用 | 保存链使用 | 未使用 |
| 当前真实地图质量比较 | 缺少逐点人工真值 | 没有与 prune 对齐的完整质量/时延结果 |
| 玻璃 | 已证实稀疏/整片透射可能误删 | 未找到足以证明解决该玻璃问题的实现或验证 |

因此保留 prune 的正式接入，不能据此声称其精度胜过 ERASOR2；也不能用上游数据集的
成绩宣布 ERASOR2 能改善当前玻璃或 NX 保存问题。

## 为什么现有 ERASOR2 适配器不能用于公平排名

固定参考版本：[`url-kaist/ERASOR2@d43d94f7e06a900456042979e29c3c933a39fd48`](https://github.com/url-kaist/ERASOR2/tree/d43d94f7e06a900456042979e29c3c933a39fd48)。
上游 [USAGE](https://github.com/url-kaist/ERASOR2/blob/d43d94f7e06a900456042979e29c3c933a39fd48/USAGE.md)
要求地面与实例标签。当前 LingTu 适配器却：

- 用 `z <= -1.25 m` 代替地面分割，生成名为 `patchwork` 的标签；
- 用 0.8 m XY 格子代替 HDBSCAN 聚类，生成名为 `hdbscan` 的标签；
- 使用固定 0.55 m 传感器高度、单位外参、20 cm 体素，不读取当前 `scan_origin.txt`；
- 从扫描重建结果，不按 prune 的同一份源点逐点保留/移除。

它验证的是文件能否交给程序，不能验证官方正常输入下的清图能力。代码保留在
[`research/maps/erasor2`](../../research/maps/erasor2/README.md)，并写明限制。
本次没有安装 HDBSCAN/Open3D/Patchwork Python 依赖，也没有新增算法后端选择器。

## 删除与归属调整

真正删除的部分：

- `score.cpp/.hpp`：只写报告、不参与删点的 XY 分格运动评分。
- `flow.cpp/.hpp`：静态阶段清单及重复 JSON 转义；不是运行调度器。
- 报告专用 `VoxelEvidence/isProtected`：重复变换扫描和统计，正式可见性判断已有独立证据。
- 六个不影响当前删点的旧 CLI 参数、恒零字段和 `lingtu_field_v2` 报告标签。

仅移动、保留实验价值的部分：ERASOR2 适配器、Patchwork++ 地面模块、增量清理和回放、
UFOMap 补丁/测试、旧 DUFOMap 对照脚本。许可证和固定上游版本保留。研究构建入口见
[`research/maps/README.md`](../../research/maps/README.md)。

实际用于删点的局部平面助手从 `maps/cleaning/surface.hpp` 归到
`maps/prune/cpp/core/surface.hpp`。正式 CMake 和 smoke 检查只构建 prune，
不再要求存在 ERASOR2 checkout 或研究依赖。

报告仅解释真实归因：未满足空闲反证、得到有效邻面支持、空闲反证且无有效邻面。
CLI/JSON 的迁移条目见 [prune README](../../src/maps/prune/README.md#报告精简后的迁移)。
正式保存调用不使用被删除的选项；原始扫描、备份、空结果拒绝和提交行为继续保留。

## 本地验证

- Windows Release 的 `prune` 和 `prune_test` 重新构建成功，18 项 CTest 通过。
- 185 帧、1,375,003 源点：保留 1,354,643，移除 20,360。`clean.pcd` 与
  `removed.pcd` 均和性能优化前保留的 v4 参考文件逐字节相同；本次没有覆盖输入地图。
  记录在 `build/map-cleaning-cleanup-20261105/equivalence.json`。
- 六个已删除 CLI 参数均明确返回未知参数错误；迁移后的 DUFOMap 工具 `--help` 可运行。
- 4 项生产边界、3 项 ERASOR2 研究契约、1 项 mapd 部署入口测试通过。
- 独立 ground/incremental/replay 构建成功，6 项研究 CTest 通过。
- 独立 ERASOR2 的 `run_erasor2`、`erasor2_stage`、`erasor2_clean` 构建成功，
  2 项 help CTest 通过；stage 实际转换了 3 帧、15 点测试输入。
  这仍是高度阈值/XY 分格标签的格式检查，没有运行同源完整清图质量对比。
  上游 `include/dataloader/dataloader.h:150` 有非 void 模板空函数未返回值的编译警告，
  本次迁移代码未引入该诊断，也未修改该上游实现。
- 修改的 shell 入口语法通过；WSL 实际 `tools/maps/prune_check.sh` 为 `pass`。
  同时修正了原脚本“构建到 `prune_wsl`、检查却去 `prune`”的路径不一致。

本次 185 帧输出对照记录了约 73.8 秒，包含生成输出及逐字节检查，且当时还有并行构建。
它不是与此前 47.557 秒 dry-run 同条件的计时，不据此报告加速或退化，也不代表 NX 的完整保存耗时。

ERASOR2 配置使用纯 Linux 工具搜索路径、Linux `/tmp` 构建目录，复用了现有系统依赖；
跨 Windows 盘的初次依赖探测非常慢。该 `/tmp` 构建及原始 CTest 日志随后随 WSL 会话清理，
成功结果来自当次工具输出，工作区仅保留结果摘要；不为复制日志重新运行已通过的构建。

## 仍需解决

1. 玻璃透射与真实移走物体尚不能可靠区分；平面保护在整片穿透时可能失效。
2. prune 保留点云并不保证后续 OctoMap 的跨帧 miss 不把该格变为 free，两个输出需要联合验收。
3. 长期静止或未再次可见的动态残影可能保留，不能靠点稀疏就删除。
4. 本地 185 帧数据没有静态/动态人工真值，当前可证明行为与输出等价，不能证明准确率。

这些不是本次目录整理已经修好的问题。玻璃证据见
[保存后玻璃机制与复现](glass-map-persistence-20261105.md)，本地性能边界见
[保存延迟记录](map-save-latency-20261104.md)。下一次算法改动应同时检验真实玻璃保留与
移走物体残影清除，再核对生成的 OctoMap；不能用全面保留旧点或更大的阈值冒充修复。
