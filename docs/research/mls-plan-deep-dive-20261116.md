# MLS 查询输入、入口半径与图规模对照

2026-11-16。DimOS 固定官方 main `83d807bb91bf6436a1a0e9c2a3260d96478636aa`。本轮检查官方 plan 是否复用同目标结果，并增加移动起点和变化目标的查询实验。LingTu 图规模从 `build/mls-performance-20261116/release/` 的 `.7` prepared cache 读取。最终性能比较使用相同最新几何规则的隔离前版本和最终后版本，四图按 ABBA 顺序采样，每个版本各 60 次查询，路径逐点相同，16 批均通过 Rust 路线和几何复核。

[固定上游 MLS 源码](https://github.com/dimensionalOS/dimos/tree/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src)

## 官方 plan 每次重新搜索

`Planner::plan(&self, start, goal, config)` 每次调用 `planner::plan`。后者每次吸附起终点，恢复 goal 所属 Voronoi chain，调用 node_dijkstra 计算各 node 到 goal 的代价，再选择机器人入口并执行路径 string pull 和 smoothing。

`last_path` 由 `plan_or_truncate(&mut self, ...)` 管理。该方法先重新规划，失败后才尝试沿相同目标的旧路径安全截断。现有 `tools/bench/dimos_same_map.rs` 使用 `Planner::plan`，原 30 次固定输入采样每次都执行上述完整查询。普通 CPU 数据缓存与分配器预热仍会影响采样。

[Planner 入口及 last_path](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/mls_planner.rs#L683)

[每次重新计算 goal Dijkstra](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/planner.rs#L163)

## 三组输入的实测

临时 runner 放在 `build/mls-performance-20261116/query-scenarios/bench_query_scenarios.rs`，编译为冻结上游 crate 的 example。地图只构建一次，每种模式预热一次后执行 30 次查询，单线程官方 worker pool。timer 只包含 Planner::plan，文件读写、建图、日志输出和 LingTu GeometryGate 均在 timer 外。本任务在采样时停止并行编译。

| 模式 | 起点 | 目标 |
|---|---|---|
| fixed_pair | 正式案例原始起点 | 正式案例原始目标 |
| moving_start_fixed_goal | 沿原官方路线 5% 至 45% 处移动，共 30 个不同点 | 固定原始目标 |
| changing_goal | 固定原始起点 | 沿原官方路线 55% 至 95% 处变化，共 30 个不同点 |

移动输入按官方原路线弧长插值。共享请求文件同时保存 ground 与 body XYZ，body Z 使用相同 Thunder 支撑高度 0.435 m。原始请求、每次耗时、成功状态和路径长度保存在 `query-scenarios/*.requests.json` 与 `*.raw.json`，汇总在 `report.json`。每次返回路线均单独保存为 XYZ。

| 场景 | fixed_pair 中位数 / P95 | moving_start_fixed_goal 中位数 / P95 | changing_goal 中位数 / P95 |
|---|---:|---:|---:|
| arrival_box | 0.808 / 0.962 ms | 0.907 / 1.123 ms | 0.865 / 1.183 ms |
| upper_floor | 0.954 / 1.618 ms | 1.166 / 1.383 ms | 1.174 / 1.802 ms |
| stairs | 0.468 / 0.626 ms | 0.488 / 0.679 ms | 0.566 / 0.886 ms |
| multifloor | 4.471 / 7.192 ms | 5.207 / 6.213 ms | 5.387 / 6.198 ms |

12 组各 30 次查询均返回官方 surface 路线，共 360/360。表中 P95 按 nearest rank 取第 29 个有序样本。聚合器分开记录成功和失败耗时，当前失败数为 0。

stairs 和 multifloor 的输入来自官方 surface 路线，历史同图三维复核分别出现 point_occupied_body。这两组记录官方 provider 的搜索工作量。将移动输入用于 LingTu 完整机身比较时，应先使用可执行路线点，并单独报告被拒绝输入。arrival_box 与 upper_floor 原官方路线已经通过共同 GeometryGate，可优先使用其共享请求做完整产品对照。

本组 DimOS wall_clearance=0，0.32 m 机身由计时后的 LingTu verifier 检查，因此官方 plan 时间未覆盖完整的同机身合同。旧两条路线的后置准入失败反映本次机器人约束的差异，现有证据不足以判定官方算法自身出错。

## 半径参数承担的职责不同

| 职责 | 官方 DimOS | LingTu release 快照 |
|---|---|---|
| 端点吸附范围 | XY 方窗半宽固定 1.5 m，每列按高度容差选择一层 | max_snap_cells 的 XYZ 球形范围 |
| 起点候选尝试 | 最多 64 个 surface 候选，先试最近点 | 先试最近的可执行入口，失败再扩其余候选 |
| 起点到 sparse node 的局部搜索 | radius=max(node_spacing_m×3, voxel_size)，本组为 0.3 m | surface_search 的 centers 半径同样使用 max_snap_cells |
| 半径的度量 | robot_search 按沿表面的几何累计距离限制扩展 | centers 按与原始起终点的 XYZ 直线距离限制范围 |
| 机器人碰撞半径 | 官方此接口的 wall_clearance=0，后续由 LingTu verifier 使用 0.32 m | 正式 GeometryGate 使用 0.32 m 与机身净空 |

LingTu 当前将一个 max_snap_cells 同时用于寻找可接入表面和入口附近的 surface 搜索窗口。正式 profile 取 24 格，arrival_box 的分辨率为 0.10 m，upper_floor 为 0.05 m，stairs/multifloor 为 0.20 m，对应半径分别为 2.4、1.2、4.8 m。DimOS 的入口附近搜索参数为 0.3 m，因此相同地图上的入口搜索范围和几何回调数量会明显不同。官方吸附代码枚举 XY 方窗，1.5 m 对应单轴半宽，角点距离可到约 2.12 m。

这给后续优化提供了明确的测量位置。应分别记录端点候选数、实际调用几何次数、局部扩展数、是否进入 sparse/surface detour及完整任务结果，再判断是否需要按职责调整局部搜索范围。简单缩小 max_snap_cells 会同时改变吸附可达性。

[官方吸附范围及候选上限](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/planner.rs#L25)

[官方入口搜索与半径](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/planner.rs#L438)

## 相同物理参数下的实际图规模

两边均使用 node_spacing=0.1 m、closing=0.2 m、step_threshold=0.45 m、wall_clearance=0、wall_weight=0、step_weight=0。但离散化、邻接和 NMS 规则仍有差异。下表 LingTu 的 surface 合并 observed/inferred，components 按缓存 sparse edges 的无向连通关系计算。全部数据保存在 `query-scenarios/graph-comparison.json`。

| 场景 | 分辨率 | LingTu surface / node / edge / component | DimOS surface / node / edge / component |
|---|---:|---:|---:|
| arrival_box | 0.10 m | 2240 / 2240 / 4058 / 14 | 2240 / 1159 / 3065 / 2 |
| upper_floor | 0.05 m | 4800 / 1200 / 2466 / 1 | 4800 / 800 / 2205 / 1 |
| stairs | 0.20 m | 968 / 968 / 1790 / 7 | 968 / 968 / 1711 / 7 |
| multifloor | 0.20 m | 7693 / 7693 / 14694 / 9 | 7693 / 7693 / 14610 / 4 |

DimOS nodes.near 使用 `distance <= node_spacing_m` 排除已有节点附近的候选。LingTu 使用离散 spacing 格数，距离等于 spacing 的格子仍可保留。spacing=1 时不同整数 XYZ 格满足这个条件，LingTu 会保留全部通过墙距筛选的格子。这个等距边界解释了 arrival_box 和 upper_floor 的 node 数差异，不能为追求相同数量直接删除性能快路径，应先确定所需的 NMS 语义。

最大步高也存在 floor/ceil 差异。DimOS step_cells 使用 floor(step_threshold/voxel_size)，LingTu 图配置使用 ceil(max_step_height/resolution)。本组 arrival_box 为 4 与 5 格，upper_floor 为 9 与 9 格，stairs/multifloor 为 2 与 3 格。LingTu 同时把最大坡度、horizontal hop 和 metric GeometryGate 作为可执行限制，DimOS 官方配置没有对应 max_slope 选项。组件数和边数还会受这些条件影响，当前实验未将某一个规则单独消融，因而不能把全部 component 差异归于 NMS。

[官方 NMS 等距边界](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/nodes.rs#L139)

[官方 step_cells 离散化](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner/rust/src/mls_planner.rs#L145)

prepared cache 的 component 数只反映 sparse graph。LingTu 查询还可在相同 surface 上做 checked detour，因此需同时检查查询返回、detour 路径和最终机身复核结果。

## 已实现的准备数据复用

当前 query::Surface 在地图和 graph 的准备生命周期内保存 detour offsets、node ID 到索引映射、node 对应 surface 格和 sparse adjacency。旧实现对每个展开格重新枚举 detour 的整数 XYZ 偏移、距离和坡度条件，每次查询还重新组装 node 索引和 links。现在一次准备这些不依赖请求的结果，查询只筛选当前格实际存在的邻居。地图区域更新会撤下这个 prepared query surface，下次查询按新地图准备。

平滑的高度采样循环增加等价提前结束。当 changing_height 已经为 true 且 follows_height 已经为 false，后续采样无法把该 shortcut 改回可接受，继续求 height 没有结果作用。新实现立即结束该候选的高度采样，继续保留高度过渡点。

这两项已进入当前源码。`build/mls-deep-20261116/cache/report.json` 的四图当时通过对应版本的几何复核，路径逐点与上一轮 release 一致。其中 upper_floor warm 中位数从 9.780 ms 到 6.411 ms，multifloor 从 121.160 ms 到 88.319 ms。这是不同批次的中间采样，未控制后续几何变更，最终收益使用下文的隔离 ABBA 实验。原始请求、采样、路线和中间源码保存在 `cache/`、`query.before.rs` 与 `query.after-cache.rs`。

## 缩小整个窗口的实验改变了路线

实验把 endpoint 附近的局部 surface 搜索范围与较大的 snap 范围分开，局部范围按节点间距确定。四图在当时版本完成查询和几何复核，但 stairs 从 6.686159 m 变成 6.829192 m，multifloor 从 30.915221 m 变成 32.141206 m。整个窗口的缩小方案已经撤回。

multifloor 的差异已分段记录。第一段 14.386466 m 完全相同，第二段楼梯 5.289160 m 也完全相同。中间平台增加 0.212093 m，终点接近段增加 1.013892 m，占总新增距离的约 82.7%。终点前一位置由 `(4.7, 4.7, 3.935)` 变成 `(2.5, 3.1, 3.935)`，最后一条目标连接由 3.303029 m 变成 4.910193 m。终点入口构造是需要继续对照的位置。

stairs 的共同末尾仍为 `(5.3, 0.1, 1.735) → (7.1, 0, 1.635)`，变化出现在上楼的前缀。实验源码、四图结果和分段报告位于 `build/mls-deep-20261116/radius/`、`query.before-radius.rs`、`query.after-radius.rs` 与 `radius-route-sections.json`。本节数字用于判断行为变化，最终保留方案的性能另行采样。

## short-pretry 实验也已撤回

第二个实验保留原 wide 24-cell lead，只让较远请求跳过大窗口的局部 surface 预尝试。四图均完成查询并通过几何复核，结果保存在 `build/mls-deep-20261116/pretry/report.json`。

| 场景 | short-pretry warm 中位数 | 路线长度 |
|---|---:|---:|
| arrival_box | 29.627 ms | 3.613871 m |
| upper_floor | 20.902 ms | 2.400000 m |
| stairs | 5.462 ms | 6.660061 m |
| multifloor | 60.783 ms | 31.127314 m |

该实验的两个平地案例比准备数据复用方案慢，multifloor 当时记录的路线为 31.127314 m，旧 cache 实验为 30.915221 m。采样期间其他会话也修改了扫掠几何规则，这个跨批次长度变化无法单独归因于 pretry。short-pretry 仍已撤回。两个半径或搜索顺序行为实验均不进入最终保留实现，当前 query 已恢复到 `query.before-radius.rs` 所记录的入口与窗口行为。保留的本轮源码修改仅为 prepared query 快照复用及平滑高度采样的等价提前结束。

## multifloor 的拓扑诊断

使用 release 的 7,693 个 prepared surface 格和当前整数 detour 邻接规则，独立 BFS 检查原先 24 格、即 4.8 m 的起终点球并集。该诊断取消几何回调，单独判断窗口内拓扑是否已经限制了可达性。

| 范围 | 几何回调 | 连通 | 已访问格 |
|---|---|---|---:|
| 原 4.8 m 起终点球并集 | 省略 | false | 919 |
| 全 surface | 省略 | true | 6,739 |

原窗口已扩大到 4.8 m，在这个 multifloor 请求的无几何诊断中，独立 BFS 访问 919 格后仍无法连接两端。919 是该 BFS 的访问计数，原生查询的实际扩展和几何调用量需要单独记录。这个结果说明局部窗口搜索的工作量与其解决该请求的能力需要分别衡量。证据位于 `build/mls-deep-20261116/local-topology.json`，其中保存原始格坐标、首选入口格、128 个 detour offset 数量和诊断范围。

当前保存的 sparse graph 有 9 个组件，DimOS 为 4 个，两者图规则和连通结构不同。这些组件可能包含合法的独立表面，组件更多本身不能证明有图连接缺陷。上述 BFS 证明的是原局部窗口无法连接该请求，全 surface 的 detour 邻接可连接该请求，还没有逐项核对入口所属的 sparse component 和具体 corridor 的几何可执行性。需要沿具体 route 确认入口、候选边和拒绝原因，再决定哪些连通差异合理。“更安全”也不能替代这项 route 检查。查询中重复的局部 surface、sparse graph 和全 surface 尝试，其实际工作量与必要性需要继续测量。

## 最新几何复核与隔离前版本

本轮采样期间，其他会话在 global_planner.cpp 加入 sweptBodyFree，在 Rust route_check 中加入解析扫掠。这些几何修复已进入最终源码，贡献单独归属该会话。本轮保留的两项性能修改继续使用这些最新几何规则。

历史 before 可执行文件经最新 verifier 重查后，stairs 和 multifloor 虽然逐点 route_valid=true，连续扫掠分别有 5 段和 1 段失败，结果记录在 `build/mls-deep-20261116/paired-before/report.json`。之前“四图全部通过”的结论只适用于当时的 checker，不能直接沿用为最新几何验收。原始采样和路线继续保留。

因此重新构建隔离 before。先冻结当前 headless.obj、runtime.lib、global_planner.lib 和当前 Rust static lib，复制当前 mls_core，仅将 query.rs 替换为 query.before.rs。平滑使用当前源码，仅删除本轮唯一的高度采样 early break。隔离 before 与最终 after 共享最新 C++/Rust 几何代码，从而将这次性能差异收敛到 query 准备数据复用和平滑提前结束。构建脚本、命令及冻结输入位于 `build/mls-deep-20261116/isolation/`。

## 最终 ABBA 对照

最终源码归档在 `build/mls-deep-20261116/final-source/`，最终可执行文件在 `final-bin/`。每图依次运行 before、after、after、before 四批，每批预热后查询 30 次，汇总为每版本各 60 个 warm 样本。报告位于 `build/mls-deep-20261116/abba/report.json`，保留全部原始样本，P95 按 nearest rank 取第 57 个有序样本。采样期间本任务无并行构建。

| 场景 | before 中位数 / P95 | after 中位数 / P95 | 中位数降低 | 最终路线长度 |
|---|---:|---:|---:|---:|
| arrival_box | 7.515 / 9.201 ms | 6.214 / 8.121 ms | 17.3% | 3.613871 m |
| upper_floor | 7.630 / 11.855 ms | 2.343 / 2.819 ms | 69.3% | 2.400000 m |
| stairs | 6.139 / 8.919 ms | 4.440 / 7.228 ms | 27.7% | 6.660061 m |
| multifloor | 94.926 / 110.486 ms | 73.921 / 123.691 ms | 22.1% | 31.127314 m |

四图 16 批均为 rust_pass，使用 Rust graph 并通过最新共同 GeometryGate。每图四批路径逐点相同，重复查询和 cache 恢复结果一致。最新 stairs 和 multifloor 的路径分别为 6.660061 m 和 31.127314 m，与旧几何快照下的路径有差别，但在本次相同最新几何的 before/after 之间保持相同。

multifloor after 的中位数降低 22.1%，P95 从 110.4864 ms 增加到 123.6909 ms。完整原始样本包含较慢的一批，当前结果支持中位数收益，慢尾尚未证实改善。ABBA 降低时间顺序干扰，机器负载未完全隔离，不能将其视为实时延迟上限。

最终 Rust 测试 60 项通过，native smoke 7/7 通过。日志位于 `build/mls-deep-20261116/tests-rust.log` 和 `tests-final.log`，最终源码记录还包含 `snapshot.json`。本轮只归因 query 准备数据复用与平滑 early break 的收益，其他会话的解析扫掠修复由相同几何隔离实验控制。

## 后续研究

### 等成本路线与端点高度的进一步定位

在统一冻结的历史 C++ 几何、平滑器、地图和请求下，重新运行 wide 与 narrow 查询。两者均返回 171 个原始 cell，原始三维长度均为 38.202631123 m，sparse search 总代价均为 192.01317。wide 选择 node 3596、cell `(17,26,17)` 作为终点侧接入，narrow 选择 node 6346、cell `(35,15,17)`。原始路线首次在第 127 个点分开，最后平台的原始代价同为 58 cells。独立核对 prepared graph 得到缺边 0、额外边 0、corridor 步进或代价错误 0。

四邻域图上的多个横竖走法可以等价。查询在等成本时保留先找到的前驱，而平滑只在已选原始路线的点之间寻找捷径。因此，入口集合改变后，即使原始成本相同，实际几何路线仍会不同。本次相同快照回放得到 30.915220737 m 与 31.928278774 m，增加的 1.013058037 m 全部发生在最后平台。历史报告中的总增加量为 1.225985293 m，中间平台约 0.212 m 的差异未在此冻结回放中重现，仍需保留快照范围。

继续定位发现，elastic 阶段要求相邻点与当前点严格等高。真实 goal 为 4.035 m，最后平台内部点为 3.935 m，这使最后一个内部转折点一直跳过 elastic。直接调用 `height(goal)` 仍返回 4.035 m，因为现有 height 接口保留 surface voxel 高度区间内的 expected Z。

保留的小修只调整真实首末端点参与 elastic 的层级比较。分别在端点 XY 使用相邻内部点的 Z 查询现有 height callback，只有该高度确实获得支持，才在等高比较中使用它。每个端点最多查询一次，真实端点坐标、内部点 Z、完整几何准入及已有 EdgeCost 约束继续参与原流程。中间楼梯的等高条件沿用已有规则。

| 同一冻结历史输入 | wide | narrow |
|---|---:|---:|
| 修正前路线长度 | 30.915220737 m | 31.928278774 m |
| 端点层级 probe 后 | 30.806805105 m | 31.446324589 m |
| 减少 | 0.108415632 m | 0.481954185 m |
| 最终几何复核 | 通过 | 通过 |

两条路线的全部 Z 序列、真实首末坐标和 17 个内部高度变化端点 XYZ 均逐点相同。两条路线的长度差降到 0.639519484 m，剩余差异与原始路线的空间走向及贪心捷径有关，本轮尚未解决。反向平滑实验也未消除此差异，因此没有加入第二次平滑。

正式源码位于 `route_smoothing.cpp`。对应 smoke 覆盖绕墙情况下同层端点高度微偏仍可缩短、真实终点台阶保留内部转角、端点 probe 期间取消，以及原有多层、楼梯、成本和碰撞检查。冻结实验记录在 `build/mls-deep-20261116/route-cause/standalone-comparison.json` 和 `endpoint-probe-comparison.json`。此处实验验证的是历史冻结几何下的原因和局部修改效果，最新几何四图回归及产品运行验证另行记录，上文 ABBA 数字仍归属于修改前的冻结版本。

后续研究优先检查以下具体工作。每项均需以当前路线和几何结果为基线验证。

- 对齐 prepared graph 和 checked surface detour 的邻接表达，逐段核对同一可执行路线如何进入 graph、哪些 corridor 被拒绝，并据此判断合法组件和需要补齐的连接。
- 借鉴官方使用已准备的 Voronoi ownership 和 predecessor chain 接入终点，评估能否省去每次 goal 侧的大范围 surface tree，同时保留终点实际机身连接检查。
- 核对 start tree、goal tree 与最终 surface detour 的重复扩展，利用已有 ownership 和已找到的目标代价减少重复工作。
- 将可复用的静态三维几何结果放在持有相同地图与机身 profile 的对象中，明确地图变化、机身配置变化和 query no-go、临时区域、实际起点豁免各自影响的结果。

mapd 持久区域生产链路、跨重启恢复 occupied/free source、MuJoCo 和实机完整执行仍按现有迁移计划推进。

[上一轮完整产品查询优化及正式边界](mls-query-performance-20261116.md)

## 复现资产

```powershell
# 所有临时源码和生成器均在 build 下。
.venv/Scripts/python.exe build/mls-performance-20261116/prepare_scenarios.py
.venv/Scripts/python.exe build/mls-performance-20261116/write_scenario_runner.py
cargo build --locked --release --manifest-path build/dimos-latest-20261116/rust/Cargo.toml --example bench_query_scenarios
.venv/Scripts/python.exe build/mls-performance-20261116/run_scenarios.py
```

prepare_scenarios.py 同时解码最终 release 的 prepared artifact 并生成图规模表。run_scenarios.py 保存具体运行命令、全部原始样本和每次路线。更换基线源码或地图后，应同步 case/source 快照并重新生成共享请求。

最终 LingTu 隔离构建使用 `build/mls-deep-20261116/build_isolation.cmd`，实际编译和链接参数记录在 `isolation/build.commands.json`。ABBA runner 为 `build/mls-deep-20261116/run_abba.py`，查询案例、每批请求、原始结果和 geometry 输出均保留在 `abba/`。
