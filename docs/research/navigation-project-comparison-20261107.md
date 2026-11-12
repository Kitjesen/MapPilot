# 导航项目对照：LingTu、DimOS、Jie 与 DDDMR

核查日期：2026-11-07，北京时间。本轮新增 jie_3d_nav 与 dddmr_navigation 的官方源码对照，
纳入已有 DimOS／OctoPlanner3D／SCAN 研究入口。完成了版本、实际配置和关键调用链核查，
尚未运行两个新项目的构建、仿真或同条件性能测试。

## 固定版本与比较范围

| 项目 | 本轮依据 | 在比较中的位置 |
|---|---|---|
| LingTu | 2026-11-07 本地工作区与下方源码入口 | 原生建图、保存重载、全局路线、SCAN 与最终控制的现有实现 |
| DimOS | `26144f7fd8424bdc33ab97dcd58e9497f2d413ee`，沿用既有固定源码与实测 | 多层支撑面、补面、稀疏图和地图更新参考；不把默认持久建图组合与独立 3D MLS 组合混为一套 |
| [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) | `995a3a544ce6e7bec6f205569339d5125843d727`，本次 API 确认仍为 main，提交于 2026-05-24 | OctoMap 导入、地图包、编辑与 XYZ 路径跟踪参考 |
| [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) | `02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4`，本次 API 确认为 main，提交于 2026-11-03 | 地面点云图导航、动态三维感知、采样轨迹和定向盒体碰撞参考 |

DDDMR 当前官方入口是 `dfl-rlab`，旧 `dddmobilerobot` 页面已声明迁移。
Jie 并非首次被本项目引用：[9 月导航职责审查](../operations/nav-review-20260928.md)
和 [OctoPlanner 转换说明](../../src/nav/cpp/planning/global/octoplanner/README.md)
已经引用同一固定版本，本次将它集中纳入项目比较。

## 与当前 LingTu 的主要差别

| 维度 | LingTu | jie_3d_nav | DDDMR：已核实的 Go2 示例 |
|---|---|---|---|
| 运行环境 | 原生 C++／DDS，Host 提供 API 与产品装配 | ROS 2 Humble，Web 经 rosbridge，另有 Qt 工具 | ROS 2 Humble，模块与插件组成导航栈，提供 Gazebo 示例 |
| 建图与定位 | LTLIO、在线图优化及原生定位链 | 完整导航依赖外部 Odin driver，SLAM／重定位模式及 `.bin` 地图另行配置 | 修改版 LeGO-LOAM-BOR 建图、修改版 mcl_3dl 定位 |
| 导航地图 | 保存的 OctoMap；独立多层支撑索引与稀疏通道图；局部接收实时碰撞图 | OctoMap，另存禁行、可通行和风险派生层 | XYZ 地面点云及静态／动态图，三维障碍簇更新地面节点代价 |
| 全局路线 | 多层表面通道图与 XYZ 回退；位置搜索不沿途搜索 yaw | XYZ 26 邻接 A*；仅发布末端姿态，yaw 不在搜索状态中 | 地面点云图 A*，三维邻域连接；Go2 另用前视目标与周期重规划包装 |
| 局部运动 | SCAN B 样条与 L-BFGS，异步轨迹接管；Follower 与最终控制负责命令 | D1 控制器依据 XY 误差跟踪路径，终点校正 yaw；已读链路未见局部障碍优化器 | 实际选择 vx/wz 可达速度窗采样、轨迹评分与原地转向插件 |
| 机体几何 | 正式局部／制动用双圆柱；全局仍为圆柱通道近似，完整几何尚未统一 | 路径格中心上方的半球采样；没有方向相关长宽包络 | 每个离散轨迹姿态放置定向长方体，对三维观测点做盒内检查 |
| 动态障碍 | Mapd 更新局部占据，SCAN 和最终控制消费实时证据 | 控制器没有地图／点云订阅；人工编辑可触发全局重算 | 有障碍簇标记、FOV 与遮挡射线清除流程 |
| 当前证据 | 具名组件测试和 MuJoCo 记录；仍有失败与停顿，见下方链接 | 作者声明 D1 + Odin 1 测试通过，有视频和交互示例；本轮未独立复现 | 有官方 Go2 Gazebo、坡道与多层演示；官方也记录坡道打滑和重复楼层问题 |

LingTu 的当前依据为 [全局实现说明](../../src/nav/cpp/planning/global/octoplanner/README.md)、
[SCAN 实现说明](../../src/nav/cpp/planning/local/scan/README.md)、
[当前碰撞模型](../design/navigation-body-model-and-motion-posture.md) 与
[窄通道物理记录](narrow-passages-physical-20261106.md)。1.20 m L 弯已有两次到达记录，
仍有暂停和验收限制；0.90 m L 弯尚未通过。它们不是与两个新项目的配对实验。

## Jie：哪些能力来自哪里

### 清洗与补点：后续源码补查

PCD GUI 有体素降采样、框选删除／仅保留、统计离群过滤、半径离群过滤、DBSCAN 删除小簇、
RANSAC 平面范围裁剪。统计过滤使用 20 邻点／1.5 倍标准差；半径过滤要求至少 12 个邻点；
DBSCAN 后仅保留至少 100 点的簇。RANSAC 最多找 6 个平面，以平面点扩展后的包围盒筛选
原有点，不生成新的平面点，也不自动补洞。
[PCD GUI 清洗源码](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/pcd_map_import_gui.py#L1110-L1205)。

转换节点另有每体素点数过滤和 26 邻接小体素簇过滤。实际默认要区分入口：节点类默认
为 0.20 m／最少 3 点／最小 4 格簇；导入 launch 和 GUI 初始转换为 0.50 m／1 点／1 格。
GUI 读取时默认按 0.10 m 降采样，转换时将节点降采样设为 0，避免重复处理。
“推荐参数”按点间距和范围选分辨率，并将两个删除门槛设为 1／1；不能把类默认的 3／4
当成 GUI 默认清洗强度。这些默认值也不能直接作为 LingTu 的导航分辨率建议。
[转换过滤](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/src/pcd_to_octomap_node.cpp#L91-L199)、
[launch 默认](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/launch/import_pcd_map.launch.py#L94-L117)、
[GUI 转换覆盖](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/pcd_map_import_gui.py#L712-L742)。

这些按钮修改工作点云，转换时写临时 PCD，保存 PCD 时默认另存为 `原名_edited.pcd`。
已查链路未见利用历次扫描射线辨别动态痕迹的清理，也未见自动几何补面。
[工作点云另存](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/pcd_map_import_gui.py#L780-L814)。

人工编辑则有不同效果，不能统称“刷了就参与规划”：

| 编辑类型 | 实际效果 |
|---|---|
| `occupied` | 先修改 GUI 占据层，刷新／保存时提交给 C++ 重建 OctoMap。可以手工加障碍；在合适地面高度加体素，也可能成为其上方格子的支撑 |
| `preblocked` | 修改禁行层并立即同步，规划器实际消费 |
| `traversable` | 人工添加的是 GUI 显示体素，规划器不订阅该编辑层；刷新后由 C++ 重新计算可通行层 |
| `clear` | 从 GUI 各相关层删掉选区体素；禁行变化立即同步，占据变化需刷新／保存。不是向原始射线地图写入已观测自由证据 |

地面也是占据几何。手工添加 occupied 没有地面专属标签，在路径机身高度添加会成为障碍；
这与根据邻域几何重建支撑面的 MLS 不同。
[编辑类型与同步](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/map_viewer_gui.py#L1151-L1311)。

### 保存、加载与显示

地图包保存 `octomap_msg.npz`、`layers.npz` 和 `meta.yaml`。前者保存序列化 OctoMap 消息，
layers 保存禁行、可通行、风险层，meta 保存地图身份、边界、时间和规划参数。
加载函数重建并发布地图／图层，但未将 meta 中的规划参数重新应用到规划器。
因此“保存了参数元数据”与“恢复了全部运行配置”应分开描述。

来源：[保存实现](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/map_package_manager.py#L265-L363)、
[加载实现](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/jie_octomap/jie_octomap/map_package_manager.py#L405-L479)。

派生禁行与可通行集合不会自行写入原始 OctoMap；人工 occupied 编辑则会新建占据树并替换发布。
人工编辑的规划效果与派生层可视化可以分别追踪；禁行层实际参与碰撞拒绝，风险代价参与 A*，
并非显示专用。但不能把从 occupied 点重建树等同于保留原树所有自由射线与占据概率证据，
也不能把 traversable 派生层叫作 MLS 地形补全。

来源：[地图编辑](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L282-L325)、
[派生层](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L824-L867)。

### 支撑、碰撞与控制

搜索状态只有 XYZ。碰撞检查在路径格中心采样上半球，unknown 节点不被该碰撞分支拒绝。
类默认要求正下方一格支撑；完整 `nav.launch.py` 实际覆盖为半径 0.25 m、非严格支撑，
允许下方一层 3×3 邻域中的占据格提供支撑。比较时应采用实际 launch，而不是仅看类默认。
该路径点没有明确的机身中心／离地高度语义，不能直接把它的 0.25 m 与 Thunder 圆柱参数等同。

来源：[支撑检查](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L474-L503)、
[半球碰撞](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L870-L930)、
[实际导航参数](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/launch/nav.launch.py#L320-L350)、
[A* 与末端姿态](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L1089-L1151)。

控制器订阅路径、启停与手动速度，用 TF 下的 XY 误差生成速度，末端处理 yaw；没有接收实时
地图或障碍点云。原始 OctoMap 更新会清空路径与起终点，人工编辑／禁行更新可以重算 A*。
这条链不能作为“动态障碍出现后局部优化仍连续执行”的现成证据。

来源：[控制输入](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/d1_controller.cpp#L90-L109)、
[路径控制](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/d1_controller.cpp#L289-L327)、
[地图更新](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp#L262-L279)。

建图与重定位由外部 Odin 实现，Odin `.bin` 与 OctoMap 导航包分别配置。
[完整导航说明](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/README.md#L385-L397)
不构成该仓库自带 SLAM、全局重定位算法的证据。

## DDDMR：实际 Go2 配置与三维能力边界

### 建图、定位和地面图

修改版 LeGO-LOAM-BOR 提供地图、位姿图保存及回环工具，mcl_3dl 消费保存结果进行定位。
全局搜索以 XYZ 地面点为节点、KD-tree 三维邻域为连接，A* 使用三维距离及障碍／边缘代价。
同 XY 不同高度可以保留在点云图中；已读邻接循环的坡度拒绝代码被注释，不能由三维邻接
直接推出每条边满足机器人台阶、支撑和机体运动条件。

来源：[建图模块](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_lego_loam/README.md#L32-L49)、
[定位模块](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_mcl_3dl/README.md#L18-L30)、
[点云图 A*](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_global_planner/src/a_star_on_pc.cpp#L230-L285)。

地面处理会在相邻扫描环之间按条件插值补点，Go2 示例允许的环间补点距离为 1 m。
因此地面图中可能包含推断点，不能把每个点都视作直接激光命中。
[地面提取与补点源码](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp#L891-L958)。

### 实时障碍与清除

感知层保存三维障碍簇，并将障碍投影到地面图更新代价。清除会考虑感知范围、FOV 与当前
点云的遮挡射线，这部分值得与 Mapd 的实际射线更新对照。其实现也有明确边界：
部分地面更新只取机器人高度附近 0.2 m 的节点，跨楼层清除的额外地面遮挡逻辑仍被注释，
源码指出平地机器人向坡面投影的不足。空／极少观测还存在单独的清除分支。
这些是本次源码限定，不应改写成“完整多层自由证据融合已解决”。

来源：[标记与投影](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_perception_3d/plugins/multilayer_spinning_lidar.cpp#L387-L554)、
[清除与遮挡](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_perception_3d/plugins/multilayer_spinning_lidar.cpp#L603-L725)。

### 局部轨迹与机体

Go2 YAML 实际选择 `DDSimpleTrajectoryGeneratorTheory` 和原地旋转生成器，采样 vx/wz 的
可达速度窗，预测 1 s，以多个 critic 评分后选轨迹。这属于 DWA/DWB 类采样滚动规划；
`mpc_critics` 的模块名、框架支持的其他生成器，都不能证明该 Go2 配置采用连续 MPC 或 MPPI。

碰撞模型是配置的长方体：x=[−0.35,0.42]、y=[−0.36,0.36]、z=[0,0.6] m。
这些是 Go2 示例参数，不是 Thunder 的尺寸建议。候选在当前机器人姿态的局部平面展开
x/y/yaw，再变换到三维坐标；没有沿每个未来地面重新求 z、roll/pitch 或落脚。
碰撞检查遍历离散位姿，观测点数少于 5 时直接返回通过。必须把这些实际条件与“有三维盒体”一起比较。

来源：[Go2 实际选择与参数](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_p2p_move_base/config/go2_localization.yaml#L171-L263)、
[速度采样](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_local_planner/trajectory_generators/theories/dd_simple_trajectory_generator_theory.cpp#L241-L305)、
[未来轨迹展开](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_local_planner/trajectory_generators/theories/dd_simple_trajectory_generator_theory.cpp#L398-L455)、
[盒体碰撞](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_local_planner/mpc_critics/models/collision_model.cpp#L51-L138)。

官方提供 Go2 Gazebo 示例，同时列出坡道偶发打滑与建图重复楼层。
本次未取得可比的四足上下楼成功率、实体接触或实机量化结果；多层地图、坡道展示和楼梯
可执行性分别记录。[官方已知问题](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/src/dddmr_beginner_guide/README.md#L68-L72)。

## 如何进入我们的后续比较

优先级是可验证的机制对照，不据项目名更换现有算法。

1. **Jie：地图操作闭环。** 对照导入、原始占据／派生层显示、人工编辑、保存重载，
   检查显示对象是否真是规划使用的数据。复用 LingTu 现有 Mapd 和保存任务的职责。
2. **DDDMR：感知与局部碰撞。** 对照旧障碍何时保留／清除、跨高度层是否误删、
   离散盒体检查与当前双圆柱检查在哪些窄门／转弯位置产生差异。
3. **DimOS：继续保留 MLS 对照。** 既有 [表面图实测](surface-graph-20261031.md)
   与 [路线复核](planner-comparison-20261031.md) 保留原版本和结果，不套用到新项目。

后续配对实验应使用同一地图、同一起终点、同一物理机器人和速度条件，分别记录：

| 场景 | 要回答的问题 |
|---|---|
| 平地箱体、直门、1.20 m L 弯；保留 0.90 m 失败回归 | 全局找到路线后能否实际到达；停顿、方向变化、接触和耗时分别如何 |
| 坡道、上下楼梯、同 XY 多高度 | 图边连接是否合理；局部高度／姿态与实体运动是否能跟上 |
| 新障碍、障碍移走、遮挡与空扫描 | 标记、保留和清除语义是否正确；规划与执行是否一致 |
| 稀疏地面与玻璃，保存后重载 | 哪些点被处理掉，哪些只是显示变化；原始证据是否保留 |

先分别复现各自示例，再做尺寸与坐标语义对齐。不能把 Jie 的路径半球半径、DDDMR 的 Go2
盒体、Thunder 双圆柱直接当成同一碰撞条件，也不能只凭路线成功率排优劣。
上述实验尚未执行，本页没有新增通过率或耗时排名。

### LingTu 编辑已有能力与下一步复用

本轮追问后的源码核对发现，LingTu 已有 [MapEditor](../../web/src/components/MapEditor.tsx)
与 [MapEditOperation](../../web/src/types/mapEdit.ts)：XYZ 框选删点／仅保留、原图与草稿切换、
撤销／重做、另存新地图。保存请求由 Gateway 转给 Mapd，后端处理完整源点云；Web 预览
处理浏览器已加载的点云，不能把客户端统计一律当成完整源数据统计。
保存后调用 OctoMap 重建，不自动激活新地图。
[保存调用](../../web/src/components/MapEditor.tsx)、
[原生另存实现](../../src/maps/cpp/build/pipeline.cpp)。

工作台另有体素标记入口，当前 `occupied/preblocked` 写 occupied，
`free/traversable/clear` 写 free，修改保存 OctoMap 并记录日志。`clear` 不是撤销恢复原证据。
这条旧接口不会自动刷新运行中 nav 的私有地图，也不会把修改注入局部实时碰撞图；
重建地图还不会重放该编辑日志。因此不能把“文件已修改”说成全局／SCAN／最终控制已经统一生效。
当前 Web 的导航体素层也明确未叠加推断支撑面。
[现有编辑合同](../../src/maps/README.md)、[Web 图层说明](../../web/src/components/SceneView.tsx)。

保存清理的当前源码已经使用 DUFOMap，命令仍叫 `prune`，与 11 月 5 日的旧清理实现有别。
它利用扫描与位姿的空闲证据分离记录，不做补面；本轮不重新宣称该算法的实机质量。
[当前清理器](../../src/maps/prune/README.md)、[替换与验证记录](dufomap-integration-20261106.md)。

建议沿现有编辑器完善三个直接表达意图的操作：清理点云、标障碍／禁行、补地面。
清洗先显示将删除的点，沿用另存与撤销；不把 Jie 的稀疏点删除阈值设为默认保存规则。
玻璃和稀疏墙面同样可能被这些静态滤波条件误删。
补地面应使用现有多层 SurfaceIndex 的实际支撑结果，限定选区和高度层、预览补出的面，
保留人工／推断来源，不把绿色体素显示当作可走依据。
优先补齐编辑后版本加载及局部碰撞消费，再扩展滤波按钮；这个顺序针对已查实的交接缺口。
以上是设计建议，本轮只补研究记录，未修改产品行为。

## 许可与证据保存

DDDMR 根目录为 [BSD-3-Clause](https://github.com/dfl-rlab/dddmr_navigation/blob/02f3bfe56217ac4e99f3af1e273cc3cbcb2024e4/LICENSE)。
Jie 的三个 ROS package.xml 声明 MIT，但固定树无独立 LICENSE，GitHub license 字段为 null；
直接复用代码前需要核实许可材料，不能将包元数据推断成整个依赖树都已完成授权核查。
[Jie 包声明](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/package.xml#L5-L8)。

本次查阅的固定源码保存在 `artifacts/nav/project-compare-20261107/jie/` 与 `dddmr/`。
只修改研究文档和索引，未安装 ROS／新依赖，未修改导航产品配置或执行运动。


## 2026-11-07 手动点云清洗已接入

后续按用户要求实施了五项手动滤波，补充本页前面的源码调查与设计建议。
现有框选删除／仅保留、撤销／重做、原图／草稿和另存流程继续使用。
新增体素降采样、统计离群、半径离群、小簇过滤和平面范围裁剪，参数由操作者选择。

纯框选历史由浏览器即时预览。包含任意清洗操作的历史，通过 Gateway 的 `edit_preview`
交给 Mapd 读取完整源点云，预览与另存共用原生 `SelectSourceEdits`。
原生返回完整保留／删除数量，画面每组最多 30,000 点并标注采样。
操作顺序及全部有效参数随新地图保存，丢失响应后可以确认同一次另存。

全部处理保留原始点坐标并输出源点子集。体素降采样保留该格首个实测代表点。
平面范围裁剪使用 RANSAC 与扩展包围盒筛点，本次没有添加地面补点或 MLS 补面功能。
实现沿用本项目原生 C++ 与已有编辑入口，增加的邻域索引使用标准库。

Windows x64 的实际 Mapd／UDS／公共 `MapClient.service()` 验证中，29,900 点地面、墙面、
随机噪点 fixture 的五种操作都完成了预览、另存与 OctoMap 重建，原始源文件和版本保持一致。
统计、半径、小簇与平面裁剪分别保留 29,600 点、分离 300 点，结果属于可控合成输入。

大图查询暴露了同步管理请求等待。相同百万点平面源与参数，KD 索引修正后统计离群从
78.370 s 降到 6.596 s，半径过滤从 7.690 s 降到 2.471 s，保留／删除数量一致。
并发管理查询仍分别等待 6.537 s 和 2.414 s。完整验证与边界记录在
[点云编辑接入记录](../../artifacts/maps/pcd-editor-20261107/README.md)。

本次状态更新仅覆盖点云清洗、预览与另存。前文记录的运行地图热加载、人工占据／禁行
贯穿局部碰撞消费、推断支撑面显示等缺口仍按各自记录管理。
