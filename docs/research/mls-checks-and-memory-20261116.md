# MLS 几何、路线与内存复核

核对日期为 2026-11-16。本页汇总离线原生测试，记录各组冻结输入和当前双圆柱结果。MuJoCo 动力学、目标硬件执行仍需单独验证。

## 几何检查与起点修复

检查项数量不能证明导航质量。需要核对机器人模型、具体障碍、可执行路线及错误拒绝。全局、局部和最终命令检查各自负责的模型已有说明，当前 nav2 会话正在统一全局双圆柱、yaw 和转身检查。此前单圆柱数据继续保留其冻结版本范围。[几何职责与参数](mls-geometry-checks-20261116.md)

起点有支撑却能通过首连接跨过缺地的情况已修复。C++ 首接入两处支撑开关已调整，Rust 同步相应规则。相同 0.8 m 缺口输入在单圆柱和双圆柱构建中均得到 `ok=false`。grid smoke 同时验证补齐地面后可成功规划，避免修复阻塞正常启动。[单圆柱修复结果](../../build/mls-start-gap-20261116/result-fixed.json) · [双圆柱修复结果](../../build/mls-start-gap-20261116/result-dual-fixed.json)

## 等成本路线和端点高度

统一历史冻结几何的 wide/narrow 回放均返回 171 个原始 cell，三维长度均为 38.202631123 m。图连接和 corridor 代价核对无缺边、额外边或错误成本。终点接入集合改变后，等成本搜索保留了不同空间走向的前驱，前向贪心平滑得到 30.915221 m 和 31.928279 m。[原始路线对照](../../build/mls-deep-20261116/route-cause/standalone-comparison.json)

真实终点 4.035 m 与内部平台点 3.935 m 的高度差还冻结了末尾 elastic 转角。已保留端点层级 probe 小修，在真实端点 XY 验证相邻内部点高度，每端至多调用一次 height。实际端点坐标、内部 Z、台阶过渡、完整几何和成本条件继续参与原流程。同一冻结输入分别缩短到 30.806805 m 和 31.446325 m，两者几何复核通过，全部 Z 序列和 17 个内部高度过渡端点 XYZ 保持相同。剩余长度差为 0.639519 m，原始路线空间走向和贪心捷径的影响仍未解决。半径行为实验已经撤回，搜索范围统一尚未完成。[端点实验](../../build/mls-deep-20261116/route-cause/endpoint-probe-comparison.json) · [详细根因](mls-plan-deep-dive-20261116.md)

## 分配减少与整进程内存

分配插桩对照中，四图的结果、路线及几何回调轨迹保持相同。multifloor 的 allocation 次数由 3,945 降至 50，reallocation 次数由 3,849 降至 67。后续前驱存储调整保持相同结果和调用轨迹，使该查询的峰值请求存活字节从 794,740 降至 733,196，其他三图该指标相同。这些计数覆盖插桩中的 Rust 查询分配。[分配对照](../../build/mls-unified-20261116/allocation-comparison.json) · [前驱存储对照](../../build/mls-unified-20261116/previous-comparison.json)

整进程内存单独运行三轮，覆盖冷准备、30 次 warm 查询和第二会话 cache 恢复。multifloor 的 peak working set 范围为修改前约 24.58–24.72 MiB、修改后约 24.52–24.77 MiB，其他图也存在重叠。当前没有证明整进程 RSS 明显下降。此组带内存轮询的数据只用于内存比较。[整进程 control](../../build/mls-unified-20261116/memory-control.json) · [整进程 after](../../build/mls-unified-20261116/memory-after.json)

单独 ABBA 计时每版本 60 个 warm 样本，四图路径一致，结果均为当时冻结几何的 `rust_pass`。

| 场景 | before 中位数 / P95 | after 中位数 / P95 |
|---|---:|---:|
| arrival_box | 5.258 / 7.089 ms | 4.079 / 6.084 ms |
| upper_floor | 2.725 / 3.899 ms | 2.733 / 4.316 ms |
| stairs | 4.526 / 5.621 ms | 5.094 / 8.457 ms |
| multifloor | 80.150 / 139.214 ms | 80.228 / 111.446 ms |

本批 stairs 变慢，upper_floor 和 multifloor 中位数接近。当前支持减少重复分配的结论，尚不支持普遍提速。该 ABBA 使用其冻结源码，后续双圆柱当前构建另行记录。[计时原始样本](../../build/mls-unified-20261116/memory-abba-final/report.json)

## 当前双圆柱结果与剩余问题

`current-dual` 四图都有通过当前路线、扫掠和运动诊断的结果。arrival_box、upper_floor 由 Rust 返回，stairs、multifloor 依赖 C++ fallback，后两图 Rust 报 `Rust surface route exhausted`。因此 Rust 完整接管仍未完成。[四图报告](../../build/mls-unified-20261116/current-dual/report.json)

对 fallback 原始路线同时使用 C++ 和 Rust checker，stairs 与 multifloor 均通过。仅将中间 Z 量化到 surface cell 中心加 0.435 m 后，两边均拒绝。stairs 第 10 段的姿态降低到约 0.635 m，机身底面由 0.685 m 降至 0.585 m，与台阶顶面 0.6 m 重叠约 1.5 cm，前圆柱触及四个障碍体素 `(17,-2..1,2)`。日志中的 `(14,-1,3)` 表示失败机身姿态 cell。multifloor 第 33 段量化后高差约 0.2 m、水平距离约 0.2 m，坡度 1 超过 0.577。该证据确认量化会丢失已验证可行的姿态，固定中心高度的候选表达仍有缺口。完整 Rust 查询失败的逐阶段归因及相关修复尚未完成。[stairs 诊断](../../build/mls-dual-diagnostic-20261116/stairs.log) · [multifloor 诊断](../../build/mls-dual-diagnostic-20261116/multifloor.log)

本次记录的测试为 Rust 71 项通过、C++ native smoke 7/7 通过。通过范围覆盖所列契约与 fixture，Rust 两个复杂图的正式返回问题继续保留。[Rust 日志](../../build/mls-unified-20261116/tests-final-rust.log) · [C++ 日志](../../build/mls-unified-20261116/tests-final-cpp.log)
