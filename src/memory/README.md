# Memory — 长期记忆与语义地点

## 概述

Memory 为机器人提供空间记忆、事件追溯和语义检索，供决策层解释任务。
这里的 Python 代码处理名称、别名和记忆；实机点云、地形、地图保存和路径规划由各自的 C++ 服务负责。
项目层级以 [`config/architecture_layers.yaml`](../../config/architecture_layers.yaml) 为准。

## 两种地点入口尚未统一

| 实现 | 保存在哪里 | 当前调用者 |
| --- | --- | --- |
| `spatial/tagged_locations.py` | Host 的 `tagged_locations.json`，按名称索引 | 网页“常用位置”、`/api/v1/locations`、名称目标、标签命令 |
| `spatial/places.py` | 不自建数据库；通过 mapd 读取每张地图的 `pois.tsv` | `/api/v1/places`、语义地点解析 |

两者目前不会自动同步。前者还依赖可选的 `TaggedLocationsModule`，当前标准 `map/nav` Product 没有装配该模块。
因此，某个文件存在或某个独立测试通过，不代表该功能已在标准产品中可用。
后续应统一到 mapd 的地图地点数据，具体缺口和迁移顺序见[地图与地点归属审查](../../docs/roadmap.md#map-and-place-ownership-review--2026-09-17)。

现有 JSON 存储仅在写盘成功后更新内存；写入失败会保留旧地点并向调用者报告错误。
这项修复解决保存结果失真，不代表两套地点数据已经合并。

## modules/ — 核心记忆模块

| 模块 | 职责 |
|------|------|
| `semantic_mapper_module.py` | 语义地图：SceneGraph -> RoomObjectKG + TopologySemGraph |
| `episodic_module.py` | 情景记忆：事件序列记录与回放 |
| `tagged_locations_module.py` | 地标位置：命名/模糊匹配 Tag 位置 |
| `vector_memory_module.py` | 向量记忆：CLIP + ChromaDB 语义向量检索，备用 numpy 暴力搜索 |
| `temporal_memory_module.py` | 时序记忆 |
| `topological_module.py` | 拓扑图记忆（已注册但**未**加入 `memory()` stack factory，当前非生产路径；场景级拓扑由 `semantic_mapper_module.py` 的 `TopologySemGraph` 负责） |
| `mission_logger_module.py` | 任务日志记录 |

## knowledge/

语义知识库：`knowledge_graph.py`、`room_object_kg.py`、`belief/`、`semantic_prior.py`

## spatial/

空间记忆子模块：`topology_graph.py`、`topological.py`、`episodic.py`、`room_manager.py`、`tagged_locations.py`、`places.py`

向量记忆的位置与观测同步保存：`PerceptionModule.robot_pose` 提供地图位姿，
`host.bus.navigation_state` 提供当前地图及内容版本。只有位姿和场景同帧、数据新鲜且地图绑定完整的
快照，才可作为语义导航候选。原始里程计和没有绑定的历史记录仍可查询，但不能直接下发运动。
切换地图或地图版本后会重新判断候选资格；不会删除历史记录。持久化快照使用独立 ID，避免重启覆盖旧记录。
这些约束已做本地模块和接线回归，尚未构成实机验证。

## storage/

持久化存储层：`sqlite_store.py`、`temporal_store.py`、`timeseries_store.py`

## scheduling/

记忆管理调度：`voi_scheduler.py` — Value-of-Information 调度器，决定何时更新/检索/压缩记忆
