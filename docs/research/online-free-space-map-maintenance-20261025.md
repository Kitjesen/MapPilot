# 在线自由空间、临时变化与长期地图：论文和源码核查

日期：2026-10-25。对象：当前 LingTu 工作树，以及下列论文的相关章节、官方配置和上游源码。
这是一份实现审计，不是上游算法复现实验；本轮没有修改导航参数、部署 NX 或下发运动。
SCAN/FAST 固定版本源码与来源清单保存在
`build/map-fusion-research-20261025/upstream/`。

后续实施：用户确认移除额外的全机体 unknown 拒绝后，全局 BodyQuery 已调整为仅对
Occupied 做机体相交判断，与实时 SCAN 的共享机体查询一致。Unknown 位图和三态证据
保持原样，地面支撑仍要求实测顶面与相应自由证据。下文中“全机体 unknown 拒绝”及
“只补保存图 unknown 的补丁”描述的是本次研究时的改前状态；最新验证见
`build/start-live-free-20261025/unknown-policy.md`。
后续也已接入：实时层完整确认旧占据体素为空时，全局机体碰撞分支忽略该旧障碍，
不改写保存图或支撑证据。

## 结论

1. 有效测量的传感器原点到命中端点之间，射线穿过的体素可累计为空的证据；端点是命中证据。
   没收到点、被遮挡、未扫描到，不能等同于射线穿过。
2. 实时观测更新、规划器如何处理 unknown、长期地图何时保存，是三个独立决策。
   SCAN 默认更新射线占据，但其已核查的碰撞调用路径并不要求全机体体积全部 known-free。
3. LingTu 的 SCAN 产品配置已经默认更新实时射线占据，时间衰减关闭。
   当前默认靠后续射线观测清除障碍，或将移出滚动窗口的数据重置；不会因等待几秒自动确认为空。
4. 保存图与实时层分开是合理的系统职责划分；“只能手动保存”不是论文共识。
   可显式保存，也可在生命周期边界自动保存，但瞬时观测进入长期地图仍需要明确的处理规则。
5. 当前本地全局规划补丁只用实时自由证据补足保存图的 unknown，不覆盖保存图 occupied。
   它还不是长期地图更新，也不能保证现场起点一定通过。

## LingTu 当前实际行为

| 问题 | 当前实现 | 源码 |
|---|---|---|
| 导航时是否自动更新实时层 | 是。接受当前地图身份匹配、定位为 TRACKING/MAPPING 的观测，使用 map 坐标点云和传感器原点更新 | `src/maps/cpp/mapd/engine.cpp:887` |
| nav 关闭扩展层是否连带关闭射线 | 否，occupancy 更新在扩展层判断之前 | `src/lingtu/assembly/compile_plan.py:557`、`engine.cpp:921` |
| 射线怎样更新 | 有效端点提供 hit；穿过体素提供 miss；超量程截断处不制造障碍命中；每帧累计后更新 log-odds | `src/maps/cpp/layers/rolling_occupancy.cpp:968` |
| 正式 SCAN 配置 | 0.05 m，200×200×100，即 10×10×5 m；射线最大 5 m；p_hit=.85、p_miss=.30、p_occ=.80 | `src/lingtu/assembly/config/native_navigation.py:99` |
| 障碍是否随时间默认消失 | 否，`LINGTU_MAPD_OCCUPANCY_DECAY_AFTER_S=0`；250 ms 是 decay 调度间隔，不是障碍寿命 | `native_nav.py:120`、`rolling_occupancy.cpp:881` |
| 新障碍会不会被历史 free 压住 | 新端点置 `unresolved_hit`，清除 known-free 并保留碰撞阻挡；后续真正穿过且无端点的观测才解除这项保护 | `rolling_occupancy.cpp:1077`、`:461` |
| 移入窗口的数据 | 初始化为 unknown，不是 free | `rolling_occupancy.cpp:619`、`:657` |
| 是否用同 XY 的点推断整根柱子自由 | 当前 rolling occupancy 不读取 column_carving 字段；mapd 的扩展表面层也默认关闭列清除，nav 还关闭扩展层 | `src/maps/include/lingtu/maps/mapd/engine.hpp:99`、`rolling_occupancy.cpp` |
| 正式保存地图 | SaveMap 要求当前为 `map` Product；抓取 SLAM 来源，重建导航包后提交。不是直接把 rolling bitmap 覆盖进旧地图 | `src/maps/cpp/mapd/save.cpp:65`、`src/maps/cpp/save.cpp:2020` |
| 导航观测是否自动修改磁盘 OctoMap | 没有发现该调用链；当前没有 nav 观测自动晋升到长期地图的能力 | `src/maps/cpp/build/pipeline.cpp:1098`、`:1151` |

以上为仓库配置，不代表已经读取 NX 正在运行的环境覆盖值。
保存 OctoMap 与实时网格使用不同 sensor model；实时位图不是保存图概率的直接增量，不能直接相加。

## OctoMap：自由空间的基础依据

Hornung 等，*OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees*，
Autonomous Robots，2013。
[DOI](https://doi.org/10.1007/s10514-012-9321-0)；
[作者 PDF](https://www.arminhornung.de/Research/pub/hornung13auro.pdf)。

相关 §3.1–3.2 将测量端点与传感器原点之间的体素作为 free 更新，未初始化区域为 unknown；
多次 hit/miss 采用 log-odds 累积，并用上下限允许环境变化后的新证据改变估计。
论文也明确讨论避免未知区域与无碰撞导航的关系；不能反过来说论文支持将任意 unknown 当 free。

官方 [`OccupancyOcTreeBase.hxx`](https://github.com/OctoMap/octomap/blob/devel/octomap/include/octomap/OccupancyOcTreeBase.hxx)
的 `insertPointCloud()` / `computeUpdate()` 分别构建 free_cells 和 occupied_cells，
同一扫描中命中的端点从 free 集合去掉，以保证端点优先。
[`AbstractOccupancyOcTree.cpp`](https://github.com/OctoMap/octomap/blob/devel/octomap/src/AbstractOccupancyOcTree.cpp)
提供独立的写文件操作；在线融合与磁盘序列化不是同一个操作。

适合借鉴：沿真实三维测量射线更新、保留 unknown、多次观测改变概率。
不能由此推出：整机体每个体素必须先被照射过，或永久地图只能由人点击保存。

## SCAN-Planner：最直接的上游对照

Zheng 等，*SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation*，
arXiv:2606.19555v1，2026-06-17 提交；本次未核实期刊或会议接收。
[论文 §V-A](https://arxiv.org/html/2606.19555v1#S5.SS1)。
源码固定为 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。

- 论文和实现均采用机器人中心滚动地图、raycasting、有界 log-odds 与增量障碍膨胀。
  LiDAR 是默认示例输入；有效点云和 sensor pose 到达后，由 0.05 s 定时器融合。
  [grid_map.cpp](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L139-L148)。
- 示例分辨率、窗口、射线范围和概率参数与 LingTu 的上述 SCAN profile 对应。
  [advanced_param.xml](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml#L40-L71)。
- 障碍用后续 miss 更新或窗口滚动清除；已读完整 grid_map.cpp 中未发现基于时间 TTL 的自动消失实现。
- raw occupancy 初始化为 unknown 哨兵，inflated buffer 却初始化为零。
  [初始化](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L108-L110)。
  已核查的 A*、双圆柱与优化器碰撞路径主要读 `getInflateOccupancy()`，没有另加全体积 known-free 门槛。
  因此窗口内未观测、且无障碍膨胀的格子，在这些路径中可返回零。
  [查询](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/include/plan_env/grid_map.h#L303-L382)；
  [优化器最终检查](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/bspline_opt/src/bspline_optimizer.cpp#L1097-L1105)。

这说明“地图记录 unknown”不等于“所有 unknown 都阻止该规划器”。
LingTu 的全机体 unknown 拒绝是额外策略，不能描述成 SCAN 原样移植的要求。
但 SCAN 这条查询路径也不能直接证明多层地形、无地面支撑区域可通行。

SCAN 的局部地图组件未见长期地图文件保存接口；这只说明持久化不属于该组件，
不能据此断言它配套的 SLAM 永不保存地图。
更详细的固定源码核查见 `build/map-fusion-research-20261025/upstream/scan-fast-findings.md`。

## nvblox / Dynablox：动态物体与静态结构分开

*nvblox: GPU-Accelerated Incremental Signed Distance Field Mapping*，ICRA 2024。
[论文](https://arxiv.org/abs/2311.00626)；[相关 §IV-B](https://arxiv.org/html/2311.00626v2#S4.SS2)。
相关动态方法为 *Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments*，RA-L 2023，
[论文](https://arxiv.org/abs/2304.10049)。

nvblox 以增量融合更新三维结构和距离信息；TSDF 不是一张简单的 free 位图。
[release-3.1 官方技术说明](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/technical_details.html)
区分静态重建、人物或动态占据；后者可以随时间衰减，衰减目标是未知概率 0.5，不能说时间本身证明为空。
动态模式需要选择，不能把其行为说成所有 nvblox 模式的默认。

当前官方 core 的
[`occupancy_decay_integrator_params.h`](https://github.com/nvidia-isaac/nvblox/blob/public/nvblox/include/nvblox/integrators/occupancy_decay_integrator_params.h)
中 `occupancy_decay_to_free=false`。
ROS wrapper 的独立 `save_map` 服务调用静态 mapper 的 `saveLayerCake()`：
[`nvblox_node.cpp`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/nvblox_ros/src/lib/nvblox_node.cpp)。
这里的当前代码与 release-3.1 文档分开引用，不把它们冒充一个固定发布包。

适合借鉴的是静态/动态职责与增量更新，不是为了修复当前起点拒绝引入另一套 GPU/ROS 地图库。

## Nav2：工程配置案例，不是三维轮足通行证明

核查 Jazzy 官方分支。
[`nav2_bringup/params/nav2_params.yaml`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/params/nav2_params.yaml)
为 obstacle/voxel 观测显式配置 `marking: True`、`clearing: True`，让新观测清除实时障碍。
但 [`obstacle_layer.cpp`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp)
的库级 `clearing` 回退默认值是 false；必须区分库默认与官方启动示例。

其静态层、观测层、规划 unknown 策略是分开的；同一官方示例的 Navfn 还设置 `allow_unknown: true`。
footprint clearing 则是清理机器人脚印内的 costmap；不是把三维墙体或地面证据无条件抹掉。
[Map Saver](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/map_server/configuring_map_saver/)
通过独立服务/CLI 保存。

## 两个能避免误读的对照

- FAST-Planner 的深度路径会 raycast；但固定提交
  `41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd` 的点云
  [`cloudCallback()`](https://github.com/HKUST-Aerial-Robotics/Fast-Planner/blob/41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd/fast_planner/plan_env/src/sdf_map.cpp#L860-L929)
  重置局部膨胀缓冲后按当前点云标障碍。一个项目有 raycaster，不代表其所有输入入口都生成自由证据。
- Cartographer 的
  [`node_main.cc`](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/node_main.cc)
  可配置 `save_state_filename`，在退出时序列化状态。因此长期保存不必一定手动，
  但“何时落盘”与“哪些观测成为长期结构”仍是不同问题。

## 对当前 blocked 修复的直接含义

保持现有默认实时三维射线融合，无需再增加一个用户开关。
全局规划读取实时自由证据，可以补足保存时缺失的空间，但需要明确当前本地补丁的范围：

- `src/nav/cpp/planning/global/octoplanner/body_query.cpp:31`：仅在保存图为 Unknown 时询问实时层；
  保存图 Occupied 不会被覆盖。
- `src/nav/cpp/planning/local/planner.cpp:106`：`observedFreeBox()` 要求盒内所有正体积重叠的实时体素有自由证据且无命中。
- `src/nav/cpp/endpoint/nav/app/loop.cpp:298`：预览、正式目标、恢复/重规划取得当前实时快照。

该补丁已做本地定向测试，但未部署 NX；记录位于 `build/start-live-free-20261025/README.md`。
原始 `start_pose_blocked` 请求在遇到第一个 unknown 后即返回；日志中的 occupied=0
不能证明整个机器人身体都没有障碍，也不能证明现场观测覆盖了机体全部空间。

下一步最有价值的是重放这一个真实起点和它对应的实时位图，明确拒绝体素属于
保存图 unknown、实际障碍、扫描盲区/自身遮挡还是支撑缺失，再调整对应查询策略。
不应继续拿“整个体积尚未逐格观测”直接解释成“机器人已经碰撞”，
也不应把所有 unknown 无条件改成 free。

导航中的实时更新与长期保存继续分开。当前正式 SaveMap 仍属于 `map` Product；
从 nav 观测更新长期图的产品能力尚未实现，不能承诺现在点一次保存就会融合这些临时变化。

## 核查边界

读取了论文相关章节和以上源码，没有编译运行外部项目；当前分支链接可能随上游变化。
未读取本轮 NX 运行配置或重新采集实际起点的实时位图，未证明候选补丁已解决实机阻塞。
文档变更不需要重跑导航测试；没有新的运行行为需要通过测试验证。
