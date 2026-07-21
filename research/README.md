# Research References

This directory contains research references and integration candidates. Nothing
under `research/` is part of the shipped LingTu product runtime unless it is
explicitly migrated into LingTu-owned `src/`, `config`, `scripts`, or `web`
surfaces with tests and a license review.

Semantic map 相关论文实现参考。这些仓库 **不纳入 git 跟踪**，需要时手动 clone。

## 参考仓库

| 名称 | 来源 | 用途 |
|------|------|------|
| concept-graphs | https://github.com/concept-graphs/concept-graphs | 3D scene graph construction |
| dimos | https://github.com/dimensionalOS/dimos | Agent-native robotics OS with semantic maps, replay, sim, and MCP |
| DualMap | https://github.com/NKU-MobFly-NLP/DualMap | Dual semantic-geometric mapping |
| HOV-SG | https://github.com/hovsg/HOV-SG | Hierarchical open-vocabulary 3D scene graph |
| OVO | https://github.com/google/ovo | Open-vocabulary occupancy |
| vlmaps | https://github.com/vlmaps/vlmaps | Visual language maps |
| CORE Planner | https://github.com/BBD00/core_planner | Contextual-memory RL exploration research; local clone only |

## First-party research packages

| Path | Status | Notes |
| --- | --- | --- |
| `vehicle_parking_detection_package/` | Research/integration candidate | RDK-side no-parking validation package. It is not the shipped Inspection Workbench analyzer; production inspection currently requires trusted observations through `src/runtime/contracts/inspection_evidence.py`. |

## 恢复方法

```bash
cd research/semantic_map_refs
git clone https://github.com/concept-graphs/concept-graphs
git clone https://github.com/dimensionalOS/dimos
git clone https://github.com/NKU-MobFly-NLP/DualMap
git clone https://github.com/hovsg/HOV-SG
git clone https://github.com/google/ovo
git clone https://github.com/vlmaps/vlmaps
```
