# 地形重建、导航地图与起终点接入

核查日期：2026-10-31。本页保留实施前的源码与固定上游审查，描述的缺口属于当时状态。
本页后面的“当前”“现有”均指该次审查时点，不能作为同日修改后的状态说明。

同日后续实施：用户要求执行本审查建议后，已接通实时自由证据到全局查询、共享补全
支撑到SCAN、原始目标到达判断，并实现实际导航地图分辨率派生重建。已执行原生测试
和MuJoCo物理验证，未部署NX。最新代码、修复前后结果与尚未完成项见
[导航修复记录](../../build/navigation-repair-20261031/README.md)。

## 实施前的结论与需要纠正的表述

1. Web 的 10/20 cm 选项只控制显示合并；导航查询仍使用原始地图。不能把该改动称为
   地形重建或导航改进。它有查看用途，但没有完成用户要求的地形可通行性改进。
2. 当前原生 `SurfaceIndex` 已把推断支撑接入全局查询。Web 尚未显示该支撑层。
   SCAN 有实测地形用于合法接触和制动，但不读取全局这份推断支撑索引。
3. 起终点邻域搜索来自 pinned OctoPlanner 原版。真实起点接入检查、起点锚定与
   终点位置容差是 LingTu 扩展。
4. 当前常规机身中心模式的搜索格以真实起点为锚点，不能泛称机器人首先被移动到
   邻近格中心。真实起点拒绝发生在候选搜索之前。
5. nav 的 15 cm 约束规划末点相对请求目标的误差；执行对路线末点又允许 XY 15 cm。
   两者可叠加，因此不能声称实际机器人一定在原始点击目标 15 cm 内完成任务。

## 三种改变不能混用

| 操作 | 改变了什么 | 对规划的影响 |
| --- | --- | --- |
| 放大点、合并显示方块 | 浏览器/RViz 的绘制对象 | 不改变后端地图和路径 |
| 改导航地图体素尺寸、重采样 | 占据/自由/未知单元与查询精度 | 会改变通行空间，但不会自动恢复真实地面 |
| 根据邻域几何补全地形 | 不同高度的表面、支撑与连接关系 | 当规划器实际查询它时才改变路线 |

10 cm 父体素包含八个 5 cm 子体素。一个子体素占据就把父体素作为占据用于导航，
会扩大障碍边界；把一个地面命中推广为整个父格有支撑，也可能掩盖缺口。
粗化是一种表示选择，不能单凭方块覆盖范围证明连续平面、法向或真实边缘。

现有接线：

```
实测三维占据 ──> 全局占据查询 ───────────┐
     └─> SurfaceIndex 推断支撑 ──> 全局支撑查询 ──> XYZ 路线
实时占据 + 实测地形 ──> SCAN 碰撞/接触/制动 ──> 优化轨迹与速度
原始 .ot ──> Web 解码 ──> 显示合并
```

`surface.cpp` 按 Z 分层闭运算、检查邻域实测占据和上方净空；
`global_planner.cpp:982` 将推断面用于支撑查询。它没有改写原始占据。
`scan/grid.cpp:448` 的机体检查使用原始占据与 `collision_.terrain` 中的实测地形；
不含 `SurfaceIndex`。局部曲线与重建地形的统一约束仍需明确接入和验证。

## DimOS 固定版本核查

版本：`26144f7fd8424bdc33ab97dcd58e9497f2d413ee`（2026-10-30）。本地 checkout
`build/dimos-review-20261025` HEAD 较早，但 Git 对象包含此提交；相关保存/重定位/
规划代码已按该提交核对，Rerun 显示也使用该提交。

| 能力 | 已存在的实现 | 不能延伸的结论 |
| --- | --- | --- |
| 建图、持久保存、加载、重定位 | `unitree-go2-map` 使用 PGOVoxelMapper，把关键帧、位姿、图边保存到 `map_go2.map.db`；再次加载、定位并继续更新 | 默认组合是 CostMapper + ReplanningAStarPlanner，不能冒充全部三维 MLS 接线 |
| 只加载、不更新保存文件 | `unitree-go2 --map-file=...` | 不表示所有加载方式都停止实时局部障碍更新 |
| 三维导航 | 独立 RayTracingVoxelMap + MLS 配置，另有重定位变体 | 所查三维 blueprint 未发现与持久 PGO 方案相同的自动 `.map.db` 保存循环 |
| 地图修改 | 射线更新/清除、关键帧及 PGO 地图更新 | 未查到与 LingTu Web 相同的通用人工三维编辑闭环 |
| 补面与显示 | MLS 真正使用多层补面，发布 `surface_map`，Rerun 渲染该面 | 不代表新 Web cockpit 默认显示相同表面，也不代表完整足步控制 |

官方固定来源：

- [Go2 mapping 文档](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/docs/capabilities/navigation/go2/index.md)
- [默认与持久 Go2 组合](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py)
- [地图持久化](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/go2/loop_closure/premap.py)
- [三维导航及重定位组合](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/robot/unitree/go2/blueprints/navigation/unitree_go2_nav_3d.py)
- [多层表面补全](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs)
- [真实 surface_map 的 Rerun 渲染](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/viz.py)

MLS 的端点搜索见
[planner.rs](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/rust/src/planner.rs)。
它在 XY 每轴约 ±1.5 m 邻域选择表面，起点最多尝试 64 个，目标只取最近候选；
每列找兼容 Z，垂直容差取 robot_height。随后从选定的表面节点检查图连通性。
`cells_to_waypoints` 在前后加入 raw start/raw goal；所查最终路径没有与 LingTu
相同的 raw→selected 机体连续接入验收。断连测试也特意让缺口超过端点搜索范围。
其 `goal_tolerance=0.3` 是 XY 到达/停止重规划阈值，不能直接当成我们的端点偏移限制。

## ROS 2 对照

ROS 是框架，具体行为由地图、显示和规划插件决定。通过官方 GitHub API 固定：

- RViz：`3756da1f1ee5f2c1e71fc073fb02f39fde0b0bf4`（2026-10-30）。
  [PointCloudCommon](https://github.com/ros2/rviz/blob/3756da1f1ee5f2c1e71fc073fb02f39fde0b0bf4/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp)
  的 Size (m)/Size (Pixels) 调用渲染对象 `setDimensions`，最后 `queueRender`，不写回地图。
- Nav2：`1e1abb8dc29ffa14de93a060d37a56f61d18446a`（2026-10-29）。
  [InflationLayer](https://github.com/ros-navigation/navigation2/blob/1e1abb8dc29ffa14de93a060d37a56f61d18446a/nav2_costmap_2d/plugins/inflation_layer.cpp)
  实际更新代价图，因此会影响规划；扩大的是障碍代价范围，不是恢复缺失地面。
- 同版 [NavFn](https://github.com/ros-navigation/navigation2/blob/1e1abb8dc29ffa14de93a060d37a56f61d18446a/nav2_navfn_planner/src/navfn_planner.cpp)
  `makePlan:244` 清除机器人所在起点单元，然后生成代价势场；目标不可达时，在目标
  容差区域中找有有限势值的可达点。它不是先锁定最近空格再独立判断是否能到达。
- 同版 [Smac2D](https://github.com/ros-navigation/navigation2/blob/1e1abb8dc29ffa14de93a060d37a56f61d18446a/nav2_smac_planner/include/nav2_smac_planner/smac_planner_2d_impl.hpp)
  保留 `StartOccupied` 分类；默认目标 tolerance 是 0.125 m。因此不能概括成
  “ROS 都删除起点检查”或“别人都使用很大的目标容差”。

借鉴 NavFn 时应区分机器人所在位置的虚假二维代价与真实三维障碍/支撑；直接把整块
机器人三维体积清空会同时删掉墙、低顶或地面依据。我们应先纠正自身回波、地面接触、
坐标和实时射线更新造成的误占据，而不是用地图清空操作掩盖原因。

## LingTu 起终点解析的真实语义

入口是 `global_planner.cpp:200 resolvePlanEndpoints`。

```
实际机身起点 → 锚定规划网格 → 检查实际起点
  ├─ 失败：立即结束，不搜索邻域
  └─ 通过：先尝试种子节点
       ├─ 通过：接受
       └─ 失败：搜索邻近节点，同时检查实际起点到该节点的连接

原始目标 → 尝试附近节点 → 检查节点可通行与目标误差 → 确定规划终点
两端确定 → A* 搜索
```

`findNearestFreeCell:501` 用 `isStartConnectionAllowed(actual,actual)` 检查起点。
普通 Go2 body-centre 模式的格点与实际起点重合，因此实际点通过后通常直接接受种子；
“实际点通过但量化节点失败”主要与受支持的 legacy/unanchored/lowest 模式有关，
不能拿来泛解释当前模式的 blocked。

起点候选连接还检查步高、坡度、纯竖直移动及连续占据/支撑。终点没有这一条
候选到 raw goal 的连接检查：选中的候选本身就是容差内允许结束的位置。

现有 `EndpointResolutionInfo` 把实际起点失败、候选耗尽、连接失败合并为
`StartSnapExhausted`。公开结果已单独区分越界 (`start_outside_static_map`)，
但占据/缺支撑等低层 `TraversabilityFailure` 没有在此保留；应复用现有原因，
避免新增重复查询或另一套失败枚举。

原版 OctoPlanner 提交 `9a9cc431ea905a5878975cc6fbbce6c9618b31a4` 已有
findNearestFreeCell：逐壳层找第一个可通行节点，默认 8 地图格，没有真实起点接入
和目标容差。核查用 `git show HEAD:...`；本地 third_party 工作树已修改，不能作为原版。

## 15 cm 的具体缺口

`config/system/products/nav.yaml:181` 的 3D/XY/Z 规划末点误差均为 0.15 m。
例如 XY 12 cm、Z 12 cm，三维距离约 17 cm，仍会被 3D 15 cm 限制拒绝。
相同三个上限下，XY/Z 判定不比 3D 球额外严格。

邻域搜索参数是地图格：汇编默认 24 格，5 cm 图对应每轴约 ±1.2 m 的候选搜索盒，
不能解释为允许目标移动 1.2 m；目标仍受上述误差约束。

正式路径流为 `goal/task.cpp` → `goal/plan.cpp:613` → `main.cpp:847` → Executor。
路线末点被原样传入，`navigation/route.cpp:336 atGoal()` 以 `route.back()` 为中心，
用 XY 0.15 m、Z 0.35 m 判到达。到达上报没有再对原始目标位置比较。

因此一维位置条件允许：原始目标 x=1.00 m → 规划末点 x=0.85 m → 实际停点
x=0.70 m。其余停车、朝向条件仍须满足。这是代码语义的可达误差上界，**不是
已测得的实机30cm偏差**。需要把任务到达区定义在原始目标上，并让全局末点选择和
局部到达使用同一目标区；直接追加可能位于障碍中的 raw goal 或缩小两个数字都不充分。

## 推荐实施顺序与验收

1. 先贯通已有补全：从原生地图侧提供规划实际使用的多层表面，让全局查询、局部
   曲线的地形约束和 Web 读取相同几何。实时障碍仍按实测更新。复用当前 mapd/
   TerrainSurface 接口边界，避免再写一套 Python/浏览器补洞算法。
2. 在同一原始地图、真实起点、原始目标和机体参数下验证：补全关闭/开启，比较
   路线连通性、局部轨迹、地形高度误差、真实边缘误连以及规划/执行耗时。
   连续平地缺格、坡面、台阶、上下层和真实槽口均有明确用途；已有大缺口测试
   不证明小真实槽口不会被误填。
3. 让端点失败保留现有占据/支撑/接入原因。当前锚定起点被拒绝时先修导致拒绝的
   真实地图或查询问题；扩大候选范围不会改变候选搜索前的失败。
4. 用原始任务目标统一到达区，测试规划末点偏移与执行停车容差的组合，避免误差累加。

本轮的官方源代码副本与提交信息位于 `build/map-navigation-review-20261031/`。
现有补面研究与历史实验见 [terrain-surface-completion.md](terrain-surface-completion.md)。
以上是当时的实施建议，具体完成范围以页首链接的实施记录为准；上游审查本身不能
作为部署或运动验收结论。
