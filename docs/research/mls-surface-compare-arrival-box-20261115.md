# Rust MLS 与 C++ SurfaceIndex 同图对照

日期：2026-11-15

本文保留 2026-11-15 各阶段的历史结果。文中的“当前”和运行命令均指对应试验版本。
使用 C++ `SurfaceIndex` 的 `mls_surface_compare.cpp` 已退役，历史源码可从
`79f9562a` 获取。当前生产入口及迁移状态见
[`mls-rust-migration-20261115.md`](../design/mls-rust-migration-20261115.md)。

本轮使用同一份 OctoMap 文件：

```text
build/arrival-box-20261027/occlusion-map-final/same_source_map/octomap.ot
```

地图分辨率为 0.10 m。占据叶节点展开为相同的整数体素列后，分别输入 Rust `lingtu_mls_core` 和 C++ `SurfaceIndex`。表面图统计使用同一个 C++ `SurfaceGraph` 和同一套搜索配置，隔离表面表示差异。全局几何复核使用当前 C++ `OctoPlanner3D`，起点和终点来自已通过的 MuJoCo 场景请求。

历史运行命令：

```powershell
build/artifact-check/Release/octoplanner3d_mls_surface_compare.exe `
  --map build/arrival-box-20261027/occlusion-map-final/same_source_map/octomap.ot `
  --start 1.5 0 0.444792 `
  --goal 5.1 0 0.435
```

## 结果

> 本节保留修正前的历史快照。它记录了为什么不能直接把第一版 Rust
> surface provider 切入生产。

| 指标 | C++ SurfaceIndex | Rust MLS | 差异 |
| --- | ---: | ---: | ---: |
| 占据体素列输入 | 7,329 | 7,329 | 相同 |
| observed surface | 2,240 | 2,240 | 一致 |
| inferred surface | 0 | 4 | Rust 多补 4 格 |
| surface 总数 | 2,240 | 2,244 | Rust +4 |
| XY 列数 | 2,240 | 2,240 | 一致 |
| 含多个 Z 层的 XY 列 | 0 | 4 | Rust +4 |
| 最大 Z 层数 | 1 | 2 | Rust +1 |
| 图节点 | 36 | 39 | Rust +3 |
| 图边 | 43 | 53 | Rust +10 |
| 图连通组件 | 9 | 9 | 一致 |
| 起点接入 | 成功 | 成功 | 一致 |
| 终点接入 | 成功 | 成功 | 一致 |
| 图路线点数 | 31 | 30 | Rust -1 |
| 图路线长度 | 3.98489 m | 3.76980 m | Rust -0.21509 m |

observed surface 的逐格集合也已比较：`observed_only_cpp = []`，`observed_only_rust = []`。因此这次差异不是观测表面坐标漂移，而是 Rust 的 bounded closing 额外产生了 4 个 inferred cell：

```text
[0, -6, 0]
[15, 0, 0]
[19, -1, 0]
[45, -1, 0]
```

C++ `SurfaceIndex` 在这张图上扫描了 11,940 个未填充候选，但这 4 个候选没有进入 C++ 的最终候选拒绝记录，输出为 `not_generated_by_cpp`。C++ 的拒绝原因计数为 `open_boundary=11909`、`hole_width=28`、`fitted_level=2`、`plane_residual=1`。这说明当前两套 bounded closing 的候选域已经不同，不能仅通过调大或调小一个公共面积参数来宣称等价。

逐格拒绝记录只在对照工具构造 `SurfaceIndex` 时显式开启，现场生产构造不保存这些字符串记录。

当前 C++ OctoPlanner3D 使用相同地图和请求能够返回路线：

```text
returned             = true
used_surface_graph   = true
route_points         = 2
route_length         = 3.60001 m
```

这说明 Rust 表面提取已经能接入 C++ 的表面图边界，observed surface、XY 覆盖、图连通组件、起终点接入均能对齐。差异集中在 inferred support。Rust 的简单 bounded closing 在该图中推断了 4 个小缺口，C++ 当前的平面边界和自由证据规则没有推断这些位置。这个差异必须先解释清楚，再决定是否允许 Rust surface provider 进入生产准备阶段。

## 几何复核状态

当前命令已经把 C++ 生产路线和 Rust 图路线都送入同一个 `OctoPlanner3D` 几何复核入口。结果如下：

| 路线 | metric limits | support | body | continuous sweep | 结果 |
| --- | --- | --- | --- | --- | --- |
| C++ 生产路线 | true | true | true | true | 通过 |
| Rust 图路线 | true | false | true | false | 第 0 段 `ground_support` 失败 |

Rust 路线的第一个连接点由拓扑图回调接受，但同一个 C++ 运行时复核拒绝了它的地面支撑。因此“图能接入起终点”不等于“路线可执行”。

因此本轮的结论是：

- Rust surface provider 已经完成真实 OctoMap 输入的跨语言接入。
- observed surface 和基础 XY 覆盖已经对齐。
- inferred surface 还没有对齐，差异为 4 个格子。
- 节点和边数量受 inferred surface 差异影响，当前还不能作为生产图规模结论。
- 图组件数量、起点接入和终点接入已经一致。
- C++ 生产路线几何复核通过。
- Rust 图路线已经完成同入口复核，但当前失败，不能进入生产。

## 修正前的下一步（已完成）

先定位这 4 个候选为什么没有进入 C++ 的 bounded-hole 候选域，再决定 Rust 是否复用同一候选域、补齐平面/坡度/自由证据规则，或明确把 inferred 关闭为观测表面模式。完成后再次比较表面数量、Z 层、节点、边和路线长度。随后要求 Rust 图路线通过同一个 C++ body、support、step、slope 和 continuous sweep 复核。两个条件都满足后，才允许把 Rust surface provider 接到生产 `PlannerArtifact`。

SCAN、Follower 和 Driver 仍然不参与这轮全局表面 A/B，继续使用现有链路。

DimOS 源码级对齐和本轮 MuJoCo saved-map 门结果见
[`mls-dimos-alignment-20261115.md`](mls-dimos-alignment-20261115.md)。

## 修正后复测（2026-11-15）

Rust `close_holes` 已按 C++ `SurfaceIndex` 的实际规则重写：同一高度带取最近观测片、外边界拒绝、面积和宽度限制、边界平面拟合、坡度和残差限制、已知 free、占据格和头部净空检查。比较工具同时把表面准备净空设为生产值 `0.45 m`，把图节点高度设为机身支撑高度 `0.35 m`；此前把两者都当成同一个高度，曾使 Rust 图路线在几何复核中落到错误的高度。

arrival-box 使用显式起终点重新运行：

| 指标 | C++ | Rust | 结果 |
| --- | ---: | ---: | --- |
| observed / inferred / total surface | 2240 / 0 / 2240 | 2240 / 0 / 2240 | 一致 |
| 图节点 / 边 / 组件 | 36 / 43 / 9 | 36 / 43 / 9 | 一致 |
| 图路线点数 / 长度 | 32 / 4.61499 m | 32 / 4.61499 m | 一致 |
| 同一 C++ 几何复核 | 通过 | 通过 | 一致 |

同一批 Rust surface cells 再送入当前 Rust 原生 `build_graph()`，得到
`30 nodes / 50 edges / 1 component`，总 corridor 长度为 `711` 个格，最长
corridor 为 `22` 个格；C++ `SurfaceGraph` 是 `36 nodes / 43 edges / 9
components`。节点和边的数量级已经接近，但 Rust 仍保留了 C++ 会因机身、支撑
或连续扫掠而拒绝的连接，因此组件数偏少，不能切入生产。

另外两张现有地图也完成回归：

| 地图 | Surface | Graph | 几何复核 |
| --- | --- | --- | --- |
| `body-depth-20261025/baseline/align` | 3456 / 0 / 3456，两边一致 | 11 / 13 / 3，两边一致 | 两边通过 |
| `body-depth-20261025/baseline/upper_floor` | 9600 / 0 / 9600，两边一致 | 28 / 48 / 2，两边一致 | 两边都无可接入起终点，未生成路线 |
| `batch-room-20261031/occupied-10cm` | 99510 / 1362 / 99510，两边一致 | 8759 / 1542 / 7930，两边一致 | 自动端点未接入，未生成路线 |

本轮验证的是离线 surface、graph 和同一 C++ 几何合同。Rust 仍未进入
`PlannerArtifact` 生产准备、正式 GlobalRoute、SCAN、Follower 或 Driver；
这一步的结果足以进入下一阶段的 Rust node/edge 接口对照，但不足以宣称
现场默认规划器已经替换。

该版本的历史复测命令为：

```powershell
build/artifact-check/Release/octoplanner3d_mls_surface_compare.exe `
  --map build/arrival-box-20261027/occlusion-map-final/same_source_map/octomap.ot `
  --start 1.5 0 0.444792 `
  --goal 5.1 0 0.435
```

## 适配层复测补充（2026-11-15）

对 Rust native corridor 的逐格路线复核曾在第 41 段失败，原因是相邻表面格的
高度变化不满足 C++ 的 metric slope 条件。对照工具现在只在 C++ 适配层从原有
corridor 选择可执行的子序列，再重复同一条 body、support、step、slope 和 sweep
复核。该过程没有放宽阈值，也没有删除碰撞检查。

arrival-box 的结果是：50 条 Rust 候选边中 6 条获得可执行子序列，16 条仍被拒绝，
过滤后保留 34 条边，起终点重新连通，34 段路线通过全部几何复核。这个结果改善了
Rust 候选路线的可执行性，但仍属于离线对照工具，不代表生产 `PlannerArtifact` 已
切换，也没有形成超过 DimOS 的同图性能证据。

## 生产接入补充（2026-11-15）

上述段落保留离线工具的历史结论。当前生产 `PlannerArtifact` 已创建 Rust graph
handle，`startPlan()` 先查询 Rust 路线，并对端点连接和每段路线执行同一套 C++
几何复核。Rust query 保持一次静态路线搜索，几何拒绝交给 C++ fallback 或外层重新规划。该接入已在 arrival-box headless
查询中看到 `used_rust_graph=true`，但还不代表 C++ `SurfaceGraph` 已删除或 Rust
surface/cache 已完成迁移。


## 2026-11-16 迁移收口

生产 `PlannerArtifact` 持有 Rust 表面和路网。候选经 C++ OctoMap 几何复核，拒绝后
最多进行一次 Rust 绕行重试，再回退 XYZ 网格 A*。C++ `SurfaceIndex` 已退出生产。
原始支撑直接查询 OctoMap，Rust 支撑查询用于 inferred 补面，避免图候选的净空
筛选丢失可用原始地面。prepared cache 使用 `.7` 格式恢复已保存的 Rust 表面和图。

本文中的 C++/Rust 表面对齐、构图耗时和通过数量均保留其当时版本和地图范围。
本次构建、取消、缓存恢复以及真实机器人执行的结果由对应验证记录给出。
