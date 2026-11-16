# 当前全局规划器公开数据集对照

日期：2026-11-14

## 本轮主结果：统一 matched 口径

本节替代前面 `go2` 参考 profile 作为本轮主要比较。两边使用同一份
OctoPlanner3D 官方 PCD、同一组起点和终点，并把地图分辨率和机身通行合同固定为
一组可复现的 matched 配置。数据提交、输入坐标和请求文件没有改变。

统一输入：

```text
PCD commit = 9a9cc431ea905a5878975cc6fbbce6c9618b31a4
building2_9.pcd / plaza3_10.pcd
building2_9-requests.txt / plaza3_10-requests.txt
OctoMap resolution = 0.05 m
surface closing radius = 0.20 m
support height = 0.35 m, tolerance = 0.05 m
max step = 0.45 m, max slope = 0.57735
```

LingTu matched 还固定了 `robot_radius=0.25 m`、机身下方净空 `0.10 m`、上方净空
`0.25 m`、支撑深度 2 个体素和 4 个入口搜索单元；输入地图不添加显式 free layers，
只使用同一份原始点云的占据几何。DimOS matched runner 使用源码中对应的
`voxel_size=0.05 m`、`robot_height=0.60 m`、`wall_clearance=0.25 m`，起点高度偏移
仍为 `0.35 m`，节点间距 1 m、墙距软缓冲 0.20 m。两套参数名称和内部语义并非一一
同义；这里固定的是共同的地图分辨率、支撑高度、水平通行余量和终点请求，避免继续
拿 `go2` 的 8 cm／0.4 m／5 cm 参考 profile 充当公平结果。

DimOS 使用 Rust 1.92.0 在 Linux x86_64 重新构建的 `build/planner-compare-20261114/mls-bench-matched`，
主结果固定 `TEST_THREADS=1`，与当前 C++ 全局规划器的单线程执行口径一致。构建关闭了
LTO、保留 release 优化；这与旧的 `go2` 二进制不是同一个构建快照。LingTu
使用当前工作树 2026-11-13 构建的 `nav_endpoint` 全局规划器。

### 同一请求的冷启动与路线结果

冷启动总耗时包含读图、索引或表面图准备，不能当成纯目标查询。LingTu 的冷启动由
同一 `PlannerSession` 驱动器单独记录，DimOS 的 `BUILD` 行记录点云体素化和图构建。

|案例|LingTu 冷启动|LingTu 返回|LingTu 路线|DimOS matched 构图|DimOS matched 查询|DimOS 返回|统一复核|
|---|---:|---|---:|---:|---:|---|---|
|建筑跨区|32.9 s|成功|129.29 m / 208 点|4.02 s|14.10 ms|失败，无路径|—|
|建筑楼梯|21.9 s|成功|55.21 m / 98 点|同一 building 图|9.45 ms|失败，无路径|—|
|广场穿越|98.2 s|成功|21.18 m / 45 点|19.84 s|35.70 ms|成功，21.28 m / 220 点|失败：第 3 点 `point_ground_support`|

LingTu 三个路线均由当前 `octoplanner3d_verify_path` 通过，占据、机身扫掠、支撑、
坡度和终点检查均通过。DimOS 的建筑两条路线在它自己的 matched runner 中没有生成，
所以没有把它们伪装成“返回后被 LingTu 拒绝”；广场路线确实返回了，但放进同一张
0.05 m 地图和同一机身复核器后，在第 3 个点就没有支撑证据。复核结果保存在
`build/planner-compare-20261114/dimos-matched-plaza-audit.json`。

### 已准备图上的真实热查询

为了排除冷启动，LingTu 的 `PlannerSession` 在每个案例先 `prepare()` 一次，再对同一
起终点连续调用 10 次；DimOS 对广场使用一次构图后连续调用 10 次。中位数如下：

|案例|LingTu 热查询中位数|DimOS matched 热查询中位数|相对值|
|---|---:|---:|---:|
|建筑跨区|318.7 ms|无有效路线（单次失败约 24.5 ms）|不可作成功路线速度比较|
|建筑楼梯|97.1 ms|无有效路线（单次失败约 25.5 ms）|不可作成功路线速度比较|
|广场穿越|74.3 ms|16.4 ms|LingTu 约 4.5 倍|

LingTu 的热查询输出来自 `build/planner-compare-20261114/warm-*.log`；DimOS 的重复
请求来自 `matched-1t/warm-plaza.log`。这组数字只回答“图已经准备好后，下一次目标查询要
多久”，不包含读 PCD、构建体素、表面索引和稀疏图的时间。

### DimOS 建筑请求为什么没有路径

这次用探针在 DimOS `plan()` 的每个失败出口打印了原因。matched 建筑图的失败不是
进程异常，而是入口连接失败：

```text
building_cross_building_modeled:
  PROBE_FAIL disconnected candidates=64 reachable_nodes=84 total_nodes=6095

building_stairs_modeled:
  PROBE_FAIL disconnected candidates=64 reachable_nodes=279 total_nodes=6095
```

也就是说，起点确实找到了表面候选，但在 `select_entry` 时，起点所在的可达组件和
目标组件没有连通，最终返回 `None`。DimOS 现有离线 runner 没有把这个内部原因带到
结果 JSON，所以旧日志只显示 `ok=false`；探针日志保存在
`build/planner-compare-20261114/probes-diagnostic.log`。

参数探针固定同一份 `building2_9.xyz` 和同一起终点，结果如下：

|分辨率|DimOS `robot_height`|墙距|建筑跨区／楼梯|
|---:|---:|---:|---|
|0.05 m|0.60 m|0.25 m|失败，组件断开|
|0.05 m|0.40 m|0.25 m|失败，组件断开|
|0.05 m|0.40 m|0.155 m|失败，组件断开|
|0.05 m|0.40 m|0.05 m|成功|
|0.08 m|0.40 m|0.25 m|成功|

因此当前证据指向“5 cm 表面量化与较大的 DimOS 硬墙距组合后，表面图被切成多个
组件”，而不是单独的 `robot_height=0.60` 导致失败。把墙距降到 5 cm 或把分辨率放宽
到 8 cm 会恢复连通，但这已经改变了机器人通行合同，不能为了让路线通过而直接替换
正式参数。这个差异也解释了为什么 LingTu 在自己的 0.25 m 三维机身检查下仍能找到
路线：两边的“墙距”并不是同一个几何谓词。

### LingTu 为什么慢

当前 LingTu 的准备阶段不是只建立一个稀疏节点图，而是先对整张 OctoMap 建立表面索引，
再为每个候选表面点调用完整的三维 `queryWorld`，最后用 `sweptRouteFree` 检查候选边。
对应源码在 `vendor/planner/src/global_planner.cpp` 的
`rebuildSurfaceIndex()`、`rebuildSurfaceGraph()`，以及 `surface_graph.cpp` 的
表面单元和边构建循环。

5 cm 分辨率会同时放大三件事：

1. 表面索引中的候选表面数量变多；
2. 每个表面点的机身半径和上下净空查询覆盖更多细体素；
3. 每条候选边都按地图分辨率采样扫掠路径，再重复做三维占据和支撑查询。

本轮广场数据的实测拆分是：表面索引约 10.8 s，表面图约 56.8 s，首次准备总耗时约
98.2 s。跨区是表面索引约 3.9 s、表面图约 18.8 s；楼梯是约 3.4 s 和 12.1 s。
所以慢点主要在“全量表面候选 + 每点三维机身检查 + 每边扫掠检查”，不是 A* 搜索本身。

保存后的 `.nav/profile-*.bin` 能避免重新计算表面图，但进程重启仍要读 OctoMap 和
准备文件。跨区缓存复用的第二次启动没有重新打印表面索引和表面图构建，却仍花了约
63.9 s 读取地图和 82 MB 的准备文件；同一个进程内保留 `PlannerSession` 时，热查询才
回到约 0.3–0.7 s。当前真正的优化顺序应是：让运行进程长期复用内存中的 session，
再优化准备缓存的读取格式和表面图构建；不能用放宽机身或支撑检查来换速度。

### 这次一致对比说明了什么

1. 之前看到的 `0.9–2.6 s` 不能继续作为 LingTu 单次搜索耗时。统一 5 cm matched
   配置下，LingTu 的热搜索已经是约 `74–319 ms`；慢点主要仍是图准备，尤其广场的
   表面图构建约 98 s。
2. 在本轮统一机身合同下，LingTu 3/3 返回并通过自己的三维复核；DimOS matched
   只有 1/3 返回，且返回的广场路线没有通过 LingTu 的支撑检查。这个结果支持“当前
   LingTu 对三维机身和支撑更严格”的判断，但不能单独证明 DimOS 算法本身错误，
   因为 `robot_height`、墙距和表面支撑在两套实现中的含义不同。
3. 性能上，当前 LingTu 仍落后于 DimOS matched；优先级应放在持久化表面图／节点图
   的准备缓存和热查询入口，而不是继续把冷启动时间算在每次查询上。缓存命中后的
   规划仍需要在真实 `navd` 进程中复测，当前驱动是同一 C++ `PlannerSession` 的离线
   证据。

这组结果仍是离线全局路线证据，不代表 SCAN 局部执行、MuJoCo 动力学或实机行走已经
通过。

### 建筑跨区为什么会爬到高层

对建筑跨区请求，单纯把 `floor_change_penalty` 从 6 提高到 50 或 500，或者关闭
`same_floor_preference`，路线仍然到达约 13.9 m 的高层。结果说明高层路线不是一个
仅由代价权重造成的选择。

把起点和终点直接连成同一高度 `z=0.3 m` 的地面线，再用当前地图和机身条件复核，
直线在 `(0.810257, -0.602564, 0.3)` 处报告 `ground_support`。当前地图的地面表面
在这条直连方向上没有连续支撑，规划器只能沿有支撑的楼梯和高层表面建立连接，再
下降到终点。

证据文件是 `build/planner-compare-20261114/direct-ground-audit.json`。这解释了图中
出现的大幅折返。它反映的是公开建筑点云和当前支撑图的连通关系，不能仅靠调大楼层
惩罚消除。需要检查建筑地图的地面缺口、楼梯入口和支撑表面是否与真实建筑一致。

### LingTu 比 DimOS 多出来的距离集中在哪里

用同一建筑跨区请求的旧 `go2` 对照路线按高度分段后，差异集中在 9 m 以上的高层
横穿和下降段：

|高度段|LingTu 路线长度|DimOS go2 路线长度|差值|
|---|---:|---:|---:|
|0–9 m|57.22 m|60.38 m|-3.16 m|
|9–15 m|59.24 m|36.25 m|+22.99 m|

因此 LingTu 的额外距离主要来自高层区域。两条路线在低层的起步和楼梯上升段长度
接近，LingTu 在高层表面之间选择了更外侧的连接和下降入口。`robot_radius`、上下
机身净空和整段 `sweptRouteFree` 检查会过滤一部分高层候选边，剩下的图连接更长。
DimOS 的 `wall_clearance=0.05 m` 表面图保留了更直接的高层边，所以总路线更短。

这解释了 LingTu 比 DimOS 多出的约 19.8 m，也说明当前需要针对高层平台和楼梯出口
优化候选边，而不是只改楼层代价。

这部分属于 SurfaceGraph 主路径的整体行为。候选表面点先在
`global_planner.cpp::rebuildSurfaceGraph()` 中做机身和支撑查询，候选边再通过
`sweptRouteFree()`，之后 `SurfaceGraph` 才建立邻接、节点、走廊和搜索结果，最后
由 `smoothRoute()` 尝试删除仍然满足完整检查的中间点。

代码中还有一个边界问题。`same_floor_preference` 在 `global_planner.cpp::searchGrid()`
和起点吸附逻辑中使用，SurfaceGraph 的主路径只读取 `floor_change_penalty`，没有把
`same_floor_preference` 传入它的边代价或搜索过程。如果产品需要这个偏好覆盖 SurfaceGraph
路径，这里需要补齐。当前公开建筑案例的高层绕行仍然首先由地面支撑不连通决定，不能
把这个缺口单独当成爬楼的根因。

本记录使用 OctoPlanner3D 官方公开点云固定提交：

```text
repository: https://github.com/JackJu-HIT/OctoPlanner3D
commit: 9a9cc431ea905a5878975cc6fbbce6c9618b31a4
license: MIT
```

输入为 `building2_9.pcd` 和 `plaza3_10.pcd`。LingTu 使用 2026-11-13
重新构建的 `build/nav_endpoint/octoplanner3d_runtime/octoplanner3d_headless`，
运行参数来自 `config/acceptance/official_global_planner.json`。结果目录为：

- `build/official-global-planner-navd-positive/`
- `build/official-global-planner-navd-positive-r2/`
- `build/official-global-planner-navd-negative/`

这三个点云案例的正向规划均通过独立的 `octoplanner3d_verify_path`：终点误差、
26 邻接、三维机身扫掠、支撑和坡度检查均通过。

## LingTu 当前结果

|案例|查询时间第 1 次|查询时间第 2 次|中位近似|路线长度|路点|复核|
|---|---:|---:|---:|---:|---:|---|
|building 跨区|958 ms|890 ms|924 ms|116.45 m|119|通过|
|building 楼梯|951 ms|1123 ms|1037 ms|52.39 m|59|通过|
|plaza 穿越|2639 ms|1972 ms|2306 ms|20.08 m|34|通过|

查询时间包含当前 headless 进程加载和准备地图图结构的成本；它不是已经缓存
`PlannerSession` 后的纯热查询时间。正式 navd 会复用 map-bound session，后续应再做
热查询测试。

## 负向案例的变化

当前版本的两个目标拒绝案例仍按预期失败：

```text
occupied goal       -> goal_snap_exhausted
unsupported goal    -> goal_snap_exhausted
```

原 benchmark 预期拒绝的 `building_stairs_geometry_only_rejected` 现在返回了路线，
并且三维复核也通过。planner 日志显示：

```text
surface index: observed=24151 inferred=0
```

这次通过没有使用推断补面；当前表面图直接使用了点云中观测到的表面。它说明旧的
`start_snap_exhausted` 预期已经不再描述当前实现，不能直接把它算成错误，也不能在
没有产品决定前把它写成安全能力提升。需要明确“只有观测表面、没有显式 free layer”
是否允许作为全局导航输入；如果不允许，应在表面图进入搜索前加回相应的自由空间证据门。

## DimOS 同一公开点云参考

现有 DimOS runner 的固定核心版本仍是：

```text
26144f7fd8424bdc33ab97dcd58e9497f2d413ee
```

当前可执行的旧 runner 只接受 `go2` profile，因此这组数值是 DimOS 参考 profile，
不是与 LingTu 完全同参数的最终排名。它在同一份 PCD 和同一组起终点上得到：

|案例|建图时间|查询时间|路线长度|路点|
|---|---:|---:|---:|---:|
|building 跨区|1323 ms|56 ms|96.62 m|612|
|building 楼梯|未单独拆出（同 building 构图）|13 ms|42.54 m|271|
|plaza 穿越|4523 ms|8 ms|20.27 m|133|

DimOS 的 `go2` 配置是 8 cm 体素、0.4 m 机器人高度、0.05 m 墙距；LingTu 使用
0.2 m OctoMap、0.155 m 路径半径、0.35 m 支撑高度及三维机身扫掠。两边的输入
表达和通行模型不完全相同，因此不能把 `56 ms` 对 `924 ms` 直接写成同条件性能倍数。

## 同地图机身复核

将 DimOS 输出的高度加回其 `start_z_offset_m=0.35` 后，使用 LingTu 当前
`octoplanner3d_verify_path` 和同一公开地图复核：

|DimOS 路线|LingTu 机身复核|
|---|---|
|building 跨区|失败：第 26 个点 `point_occupied_body`|
|building 楼梯|失败：第 17 个点 `point_occupied_body`|
|plaza 穿越|通过，三维扫掠和运动诊断均通过|

这与此前自有地图对照中看到的现象一致：DimOS 候选路线更快、更平滑，但部分
候选点在 LingTu 的实际机身占据模型下不可执行。它不能直接证明 DimOS 路线错误，
因为两边的墙距、机身模型和起点接入规则不同；它证明了仅用“返回路径”作为成功指标
不够。

## `point_occupied_body` 的含义

这个错误来自 LingTu 的路径复核器，不是 DimOS 进程崩溃，也不是 LingTu
自己的返回路线失败。复核器对 DimOS 路线的每个中心点调用同一套 `queryWorld`：
以 `robot_radius=0.155 m`、上下各 `0.10 m` 的机身包络检查占据体素。建筑跨区
路线在第 26 个点、楼梯路线在第 17 个点第一次命中 `OccupiedBody`，因此复核器
在检查扫掠段以前就停止。

把地图分辨率从 0.20 m 改成 0.10 m／0.05 m 后，失败原因转为
`point_ground_support`，说明差异不只是“0.20 m 体素太厚”：DimOS 的表面图允许
的表面高度和墙距，在 LingTu 的支撑深度、支撑连续性和机身包络合同下并不总是可执行。
广场路线在 0.20 m 地图上通过了相同复核。正确的比较是分别记录“各自算法返回成功”
和“放入 LingTu 执行合同后通过”，不能把后者写成 DimOS 算法自身失败。

## 为什么看起来很慢

公开样例的 planner 日志把一次冷启动拆成了三段。以当前 0.20 m 地图为例：

|案例|表面索引|表面图|真正搜索与平滑|
|---|---:|---:|---:|
|建筑跨区|78 ms|576 ms|约 44 ms|
|建筑楼梯|85 ms|646 ms|约 7 ms|
|广场穿越|386 ms|1960 ms|约 2 ms|

headless runner 的 `elapsed_ms` 包含地图加载和表面图准备，所以显示为约
0.9–2.6 s。它不是一个已经准备好 `PlannerSession` 后的纯目标查询时间。
正式 navd 的 `PlannerSession::prepare` 会按地图身份和 preparation 配置复用
OctoMap、表面索引和稀疏图；`PlannerSession::run` 在配置不变时只更新 query 参数
并直接规划。因此当前首要问题是准备阶段和缓存命中率的测量与优化，不是已经证明
A* 每次都要跑几秒。

按同一批 planner 日志中的 `graph_ms + smooth_ms` 重新估算已准备图上的查询段：

|案例|LingTu 路线查询段|DimOS go2 参考查询|
|---|---:|---:|
|建筑跨区|约 44 ms|56 ms|
|建筑楼梯|约 7 ms|13 ms|
|广场穿越|约 2 ms|8 ms|

这组数仍不是 matched profile 的严格性能排名，但它纠正了“LingTu 每个目标都要
0.9–2.6 s”的误解。当前公开样例里，慢的是首次表面图准备；图准备完成后，路径
搜索和后处理已经是毫秒级。正式 navd 的 warm `PlannerSession` 还需要专门跑 10 个
连续目标来确认端到端数字。

## 当前结论和下一步

当前源码在公开 OctoPlanner3D 样例上已经达到：

```text
正向案例：3/3
正向路线几何复核：3/3
明确占据/缺支撑目标拒绝：2/2
```

相对旧的自有地图记录，不能直接声称成功率提升，因为地图、起终点和输入表达不同。
查询时间仍明显高于 DimOS 的参考 profile；下一步应优先测缓存 `PlannerSession` 的
热查询，而不是只优化首次地图准备。

还没有完成的公平对照是 DimOS `matched` profile。现有 runner 的源码包含该 profile，
但已编译二进制只接受 `go2`；当前 WSL Cargo 为 1.75，无法直接解析 lockfile v4，
且缺少离线 `rayon` 包。因此必须在带现代 Rust 的 Linux 构建环境中重编 runner，
然后固定同一体素、墙距、机器人高度和请求集再报告性能差异。

这些结果都是离线全局路线几何证据，不代表 SCAN 局部执行、MuJoCo 动力学或实机运动
已经通过。
