# Memory — L3 长期记忆与语义地图模块

## 概述

Memory 层为机器人提供场景理解、空间记忆、事件追溯和语义检索能力，支撑 L4 决策层的推理与导航任务。

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

空间记忆子模块：`topology_graph.py`、`topological.py`、`episodic.py`、`room_manager.py`、`tagged_locations.py`

## storage/

持久化存储层：`sqlite_store.py`、`temporal_store.py`、`timeseries_store.py`

## scheduling/

记忆管理调度：`voi_scheduler.py` — Value-of-Information 调度器，决定何时更新/检索/压缩记忆
