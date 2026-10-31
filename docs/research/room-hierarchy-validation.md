# Room / floor / object 验证路线

这份记录把 HOV-SG 的 `floor → room → object` 结构和 LingTu 当前已经
存在的几何、语义接口接起来。它只定义验收路径，不把数据集里没有的房间
标签补出来。

## 论文依据

- [HOV-SG (RSS 2024)](https://www.roboticsproceedings.org/rss20/p077.html)
  把楼层、房间和物体作为不同层级的查询对象，并使用层级场景图支持语言
  导航。
- [OneMap (ICRA 2025)](https://www.finnbusch.com/OneMap/) 把语义特征保留在
  可复用地图中，连续查询不重建几何地图。
- [OVI-MAP (CVPR 2026)](https://openaccess.thecvf.com/content/CVPR2026/html/Deng_OVI-MAP_Open-Vocabulary_Instance-Semantic_Mapping_CVPR2026_paper.html)
  将稳定的实例几何和选取的视觉语义证据分开维护。

三篇工作的共同前提是：房间节点必须有可追溯的观测或数据集标签；仅凭
物体类别猜出的“会议室”只能叫作 hypothesis，不能进入导航准入。

## 当前官方数据能证明什么

本仓库的 ReplicaCAD `apt_0` 已经提供官方场景变换、GLB 几何、语义类别和
导航网格。`tools/datasets/replicacad_probe.py` 将它们转换为
`lingtu.semantic_scene_graph.v1`，当前输出为：

```text
floor (scene-level, known)
  └── semantic label group
        └── object instance (geometry-bound)
```

它可以验证实例 ID、map-frame 几何、`near` 关系和对象查询；但是
`apt_0.scene_instance.json` 没有 room/region 标注。因此输出中的
`rooms=[]` 和 `room_segmentation=unavailable` 是正确结果，不能把按距离
聚类得到的区域写成真实房间。

3RScan 也适合作为一个小的基线：其官方数据说明每个扫描场景是一个室内
空间，并提供 RGB-D、位姿和实例语义标注，但它不是带多房间区域标签的
数据源。[3RScan 官方工具仓库](https://github.com/WaldJohannaU/3RScan)
和[官方文档](https://vmnavab26.in.tum.de/3RScan/documentation.php)
都要求按数据条款申请，当前仓库没有下载其大型序列。

真正需要多房间区域标注时，优先候选是 [HM3D Semantics](https://aihabitat.org/datasets/hm3d-semantics/)。
官方页面说明 HM3D Semantics v0.2 有 216 个 3D spaces、3,100 个 rooms
和 142,646 个 object instance annotations，并注明仅限学术、非商业研究；
它比 ReplicaCAD 更适合 L2，但必须先取得访问许可和一个小场景样本。官方
3RScan 格式文档的 `semseg.v2.json` 只展示 `segGroups` 的 object 实例字段，
所以 3RScan 可以做单 room baseline，不能替代 HM3D 的多 room 验证。

## 最小可用验收顺序

### L0：结构契约（仓库内、无需数据下载）

`tests/perception/test_scene_hierarchy.py` 已有一个 1 floor、2 room、
2 group、3 object 的固定 tracker 快照。它验证：

1. floor、room、group、object、relation 的 ID 保持不变；
2. room 中心和包含边被保留；
3. `graph_level=hierarchical`；
4. `SceneGraph.validate_hierarchy()` 接受合法图，拒绝未知子节点和环；
5. JSON 保存/重载不改变层级边。

这是 HOV-SG 结构接口的最小回归测试，不是感知准确率测试。

### L1：官方几何对象（ReplicaCAD）

```powershell
.venv\Scripts\python.exe tools\datasets\replicacad_probe.py `
  build\datasets\replicacad_full --scene apt_0 `
  --output build\datasets\replicacad\lingtu_scene.json

.venv\Scripts\python.exe tools\datasets\3dssg_goal_query.py `
  build\datasets\replicacad\lingtu_scene.json `
  --output build\datasets\replicacad\query_result.json
```

验收的是：对象有稳定 `instance_id`、`geometry_ref` 和 `map_identity`，
`box near table` 从几何中心重新计算，文本查询只返回 geometry-bound
对象。由于没有 room 真值，本层不能声称 `floor → room → object` 已完成。

### L2：房间真值接入（需要带 room/region 标注的输入）

当获得 HM3D-Semantics、Matterport3D 或经许可的实机 RGB-D 序列后，适配器
只需把数据集的 region ID 映射到现有结构：

```text
dataset floor_id      -> SceneGraph.floors[].floor_id
dataset region_id     -> SceneGraph.rooms[].room_id
dataset instance_id   -> SceneGraph objects[].id
region contains obj   -> hierarchy_edges(floor -> room -> object)
dataset room label    -> RoomNode.name / semantic_labels
```

每个 object 必须恰好挂到一个 room（未知时保留 `room_id=-1`），每个 room
必须挂到一个 floor。房间名称、中心和包含关系都必须携带 `source` 或
`observation_refs`；LLM 只能提出候选名称，不能替代数据集的 region ID。

### L3：语义任务和原生规划预览

只有 L2 通过后，才运行：

```text
先去 <room label>
再找 <object label> near <reference label>
  -> room_id
  -> geometry-bound instance_id
  -> standoff candidate
  -> native preview_plan
  -> independent path verifier
```

`preview_plan` 被拒绝时不得调用 `send_goal`。规划准入还必须检查
`map_identity`、`content_epoch`、定位状态和 3D 支撑面；语义候选本身不授权
运动。

## 当前缺口和下一步

当前缺口只有一个数据层问题：官方 ReplicaCAD 缺少 room/region 真值。现有
代码和测试已经能承载完整层级，但没有理由用空间聚类或 LLM 猜测填满
`rooms`。最小下一步是取得一个许可的、只含一小段场景元数据和房间标注的
HM3D/Matterport3D 样本，新增一个离线适配器测试 L2，然后把同一实例送入
现有 `3dssg_goal_query.py` 和 native preview。

在 L2 之前，任何“语义地图已经达到 HOV-SG 论文效果”的表述都不成立；目前
能声称的是官方 ReplicaCAD 的 floor/label-group/object 几何绑定，以及仓库
内固定快照的完整 floor/room/object 结构契约。
