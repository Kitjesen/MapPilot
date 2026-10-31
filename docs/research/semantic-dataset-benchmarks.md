# 语义地图数据集验证路线

这份记录把开源数据集、论文中的输入输出和 LingTu 的验证阶段对齐。当前目标是先证明“数据能进入语义场景图，再证明语义节点能绑定到三维几何，最后才让导航消费这些节点”。图像标签或离线关系图本身不能授权机器人运动。

## 当前采用的论文边界

当前实现不是只对应一篇论文。3DSSG（CVPR 2020）用于验证“对象节点和关系边”这种图结构；最新的系统设计参考是 2026 年 CVPR Highlight 的 [OVI-MAP: Open-Vocabulary Instance-Semantic Mapping](https://openaccess.thecvf.com/content/CVPR2026/html/Deng_OVI-MAP_Open-Vocabulary_Instance-Semantic_Mapping_CVPR_2026_paper.html)。OVI-MAP 把类别无关的三维实例重建和 VLM 语义推断拆成两个异步过程：先用 RGB-D 与位姿建立实例几何，再从覆盖充分的关键视角提取语义证据，最后绑定到持久实例上。官方项目页和代码在 [ovi-map.github.io](https://ovi-map.github.io/) 与 [github.com/OVI-MAP/OVI-MAP](https://github.com/OVI-MAP/OVI-MAP)。

因此 LingTu 的目标接口按以下边界推进：

```text
同步传感器帧 + 位姿
        ↓
类别无关 instance_map（几何先行）
        ↓
关键视角选择
        ↓
VLM semantic_evidence
        ↓
持久 object_memory
        ↓
文本查询 → 候选实例 → goal_pose
        ↓
native 3D planner 验证与执行
```

这条边界仍然是设计目标，不代表当前代码已经达到 OVI-MAP 的完整效果。

## 第一套：3DSSG

[3DSSG 官方页面](https://3dssg.github.io/)把场景表示为节点和边：节点是物体，边是关系。论文 [Learning 3D Semantic Scene Graphs from 3D Indoor Reconstructions](https://openaccess.thecvf.com/content_CVPR_2020/papers/Wald_Learning_3D_Semantic_Scene_Graphs_From_3D_Indoor_Reconstructions_CVPR_2020_paper.pdf)使用点云和实例分割作为输入，用 PointNet 和图卷积预测语义场景图，并把场景图用于 3D-3D 与 2D-3D 匹配。

仓库测试下载的是官方 3DSSG 标注压缩包。它包含 `objects.json` 和 `relationships.json`，不包含 3RScan 的重建几何。当前本地文件统计为：

- `objects.json`：1482 个场景记录
- `relationships.json`：1335 个场景记录
- 选取的测试场景：`0cac7676-8d6f-2d13-8f3a-d7bf7f03e721`
- 节点：43
- 关系：421
- 关系样例：`table (27) close by box (39)`

运行方式：

```powershell
.venv\Scripts\python.exe tools\datasets\3dssg_probe.py `
  build\datasets\3dssg\3DSSG `
  --output build\datasets\3dssg\lingtu_scene.json
```

适配器输出 `lingtu.semantic_scene_graph.v1`。每个节点保留 3DSSG 的类别、本体 ID、属性和 affordance；每条边保留主体、谓词、谓词 ID 和客体。因为当前下载没有 3RScan 几何，`geometry_binding.status` 明确为 `unavailable`，这份结果用于验证语义结构和任务解析，不能直接进入导航碰撞查询。

随后运行 `tools/datasets/3dssg_goal_query.py`，把图投影成当前 `GoalResolver` 的输入格式。它同时计算一份精确的图关系约束：本场景中 `table` 旁边的候选箱子是对象 `39`。我们已修正快速解析器：当指令包含明确的目标和参照物时，先保留真正连接的关系边，再进行评分；当前结果返回对象 `39`，`spatial=1.0`，`resolver_candidate_matches_relation=true`。因为几何仍不可用，结果继续标记为 `motion_authorization: false`，不会被伪装成可导航目标。

## 第二套：Replica

[Replica 官方仓库](https://github.com/facebookresearch/Replica-Dataset)提供室内重建网格、纹理、平面预分割、语义类别与实例分割，并提供 Habitat 格式的语义网格和导航网格。它适合验证 RGB-D 渲染、语义投影、房间与物体关系，以及在仿真中的“建图—保存—重载—导航”闭环。它的资产是重建场景，不是机器狗现场采集的旋转 LiDAR 序列，所以要把传感器时序和实机 SLAM 误差单独验证。

本轮已实际使用官方 [ReplicaCAD Dataset](https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset) 的 `apt_0` 场景做几何绑定，而不是只引用页面。已下载该场景所引用的 80 个唯一 object config/render GLB，加上官方场景配置、语义词典、stage GLB 和 navmesh；本地缓存约 65 MB。原始场景有 113 个物体实例，当前 113 个实例全部绑定到 GLB 几何，缺失资产为 0。

重现命令：

```powershell
.venv\Scripts\python.exe tools\datasets\replicacad_probe.py `
  build\datasets\replicacad `
  --scene apt_0 `
  --output build\datasets\replicacad\lingtu_replicacad.json
```

适配器读取官方 `scene_instance.json` 的物体变换、object config 的 `semantic_id` 和 GLB 的 POSITION 几何边界，输出 `lingtu.semantic_scene_graph.v1`。当前结果为 `113` 个 geometry-bound 节点、`1` 条真实几何关系：`box object:apt_0:108` 与 `table object:apt_0:19` 的中心距离为 `1.5284377412 m`，`geometry_binding.status=bound`，同时保留 `apt_0.navmesh`。ReplicaCAD 场景使用 Y-up；适配器显式转换为 LingTu 的 Z-up map frame（`[x,y,z] → [x,z,y]`），但官方 navmesh 的坐标系仍需在 native planner 接入前单独确认，不能把这个离线结果直接当作可执行目标。可视化证据在 `build/datasets/replicacad/lingtu_replicacad_topdown.png`。这证明了“官方实例语义 → map-frame 几何 → near 关系”的 D3 链路；它仍然不授权运动，`motion_authorization=false`。许可证需要单独记录：官方 HF README 元数据写 CC BY 4.0，但随仓库下载的 `LICENSE.txt` 文本写 CC BY-NC 4.0；在产品化或再分发前必须按上游最终条款确认，不能把数据集默认为可商用。

适配器同时输出 `graph_level=hierarchical`、一个已知 scene/floor 和按语义标签聚合的 object groups；`room_segmentation=unavailable` 被显式保留。这样可以测试 HOV-SG 所需的层级数据契约，却不会把 ReplicaCAD 没有提供的房间标签伪造成真值。完整的 `floor → room → object` 仍需要 RGB-D/VLM 观察或带房间标注的官方场景数据。

`3dssg_goal_query.py` 现在复用运行时的 `observation_candidates`：对绑定的
`box:108` 生成 8 个 0.8 m standoff 候选位姿，并把每个候选标为
`requires_native_preview=true`。对象中心本身不再被当作可执行机体目标；没有几何绑定时，候选列表为空。这与 HOV-SG 的“语义目标 → 导航图/观察位姿”和 OneMap 的“候选实例 → 运动准入”边界一致，但候选仍需原生 planner preview 和验证器通过。

适配器还输出 `instance_records`，记录 `instance_id`、`geometry_ref`、
`map_identity`、观测引用和语义证据引用；`replicacad_persistence_check.py`
将 113 条记录写盘再重载，ID 和几何引用保持一致。这个结果只证明数据集
artifact 的保存/重载，不替代 `mapd` 原生地图的保存/重载验收。

`tools/bench/semantic_goal_preview.py` 随后逐一调用原生 headless planner
检查 8 个接近候选。在只使用修正后顶点的 OctoMap 上接纳数为 `0/8`，报告状态
为 `target_unreachable`，失败原因逐候选保留在
`build/datasets/replicacad/semantic_goal_preview.json`。加入楼板三角面采样后，
离散 planner 能接纳 `1/8`，但独立 verifier 将该路径标为
`verifier_blocked`，所以最终可执行数仍是 `0/8`。这不是把候选生成误报成
语义导航成功，而是明确告诉上层当前地图/连续碰撞准入仍不能提供可执行观察位姿。

这套官方几何走过 LingTu 的原生地图转换器：全量顶点导出为
`build/datasets/replicacad/apt_0_lingtu_full.pcd`（320,997 点），再生成
`build/datasets/replicacad/apt_0_full_free_d2.ot`。过程中发现并修正了一个真实
的 GLB 变换 bug：stage 根节点和 mesh 子节点各自带有旋转，旧适配器只应用子
节点，导致楼板被转到错误的高度和范围。现在适配器递归合成完整的
TRS/matrix 变换；修正后的点云范围约为
`x=[-2.68,4.58], y=[-4.83,8.16], z=[-0.09,3.04]`，与室内场景一致。

直接把物体中心作为目标仍得到 `goal_snap_exhausted`，这是正确的，因为中心
落在箱体占据几何内。增大 support/free 膨胀可以让部分候选暂时返回
`reached_goal`，但独立验证仍失败，所以不能把膨胀当修复，也不能授权运动。
完整请求和失败证据保存在 `build/datasets/replicacad/official_planner_report.json`
和 `build/datasets/replicacad/semantic_goal_preview.json`。

语义查询层现在把对象中心转换成 8 个带 standoff 的接近位姿候选，再交给 native planner 做只读预览；对象中心不会直接作为机体目标。候选输出保留 `requires_native_preview=true` 和 `motion_authorization=false`，因此“语义定位到物体”与“规划器接受可执行位姿”仍是两个独立验收门。

## 第三套：SemanticKITTI

[SemanticKITTI 官方页面](https://www.semantic-kitti.org/)和 [ICCV 论文](https://openaccess.thecvf.com/content_ICCV_2019/papers/Behley_SemanticKITTI_A_Dataset_for_Semantic_Scene_Understanding_of_LiDAR_Sequences_ICCV_2019_paper.pdf)提供 10 Hz 的连续旋转 LiDAR 扫描和逐点语义标注，覆盖单帧语义、多帧语义和语义场景补全。它适合验证点云标签传播、时序融合、动态物体过滤和保存地图后的语义一致性；场景是车载室外环境，不能代替室内机器狗的楼层、门和狭窄通道测试。

## 第四套：KITTI-360

[KITTI-360 官方页面](https://www.cvlibs.net/datasets/kitti-360/)同时提供 RGB、立体相机、Velodyne LiDAR、时间戳、位姿，以及 2D/3D 语义和实例标注。官方描述包含 73.7 km 路程、约 320k 图像和 100k 激光扫描，并提供相机与点云标签的开发工具。它适合验证真正的 RGB-LiDAR 外参投影和跨帧实例 ID；数据下载需要注册，并且官方许可是非商业、署名、相同方式共享条款，产品使用前要单独审查。

## 公开视频基线

2025 年 ICRA 的 [One Map to Find Them All](https://www.finnbusch.com/OneMap/)展示了开放词汇特征图和连续多目标搜索，并包含 Spot 与 Jetson Orin AGX 的实机视频；2024 年 RSS 的 [HOV-SG](https://hovsg.github.io/)展示了 `floor → room → object` 层级查询和语言目标导航视频。它们是论文项目的外部基线，不是 LingTu 的实机证据。LingTu 的仿真视频必须在画面和报告中明确标记 `sim`，并单独记录文本解析、候选实例、规划路径和运动轨迹。

## LingTu 的验收阶段

| 阶段 | 输入 | 产物 | 当前状态 |
| --- | --- | --- | --- |
| A. 结构解析 | 3DSSG 标注 | `semantic_scene_graph.v1` 节点和关系 | 已跑通 |
| B. 几何绑定 | ReplicaCAD `apt_0` 几何 + 实例 ID | 113 个节点对应 GLB 包围盒，1 条几何 `near` 边 | 已完成（D3；navmesh 坐标待 native 接入确认） |
| C. RGB-LiDAR 投影 | 图像、深度或 LiDAR、内外参、时间戳、位姿 | 带语义的三维点/体素 | 现有 RGB-D 投影已有局部验证，待公开数据集闭环 |
| D. 持久化 | 语义点云、占据、碰撞几何、地图身份 | 保存/重载后语义一致 | 待 B/C |
| E. 导航 | 几何地图 + 语义约束 | 全局规划与局部执行证据 | 语义层不能绕过 native 3D 碰撞查询 |

ReplicaCAD 的几何绑定已经回答了“桌子旁边的箱子”是否能成为带 map-frame 几何的语义候选；下一步是把这个候选送入 native `preview_plan`，先处理官方 navmesh 与 LingTu Z-up 坐标的确认，再决定是否允许规划器接收目标。3RScan/ScanNet 仍用于后续真实 RGB-D 重建和跨帧实例关联，不把它们的许可申请状态冒充为已下载数据。
