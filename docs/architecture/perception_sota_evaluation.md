# LingTu Perception 模块 SOTA 对标评估报告

## 执行摘要

LingTu perception 模块整体处于**业界先进但非领先**水平(综合评分 **4.1 / 5.0**)。其定位是对 2024–2025 年成熟 SOTA 思想的**精细工程化实现**,而非算法层面的原始创新。

- **核心优势**:轻量级实时架构、确定性追踪、多后端可插拔。
- **主要短板**:密集语义特征、几何精细分割、持久化闭环建图。
- **核心竞争力**:Module-First 解耦架构 + 在 RDK X5 边缘算力约束下的**确定性性能保证**。

换言之,LingTu perception 不追求学术指标的绝对领先,而是在边缘平台上把已被验证的 SOTA 思想落地为**可维护、可配置、可确定性验证**的生产级实现。

---

## 一、技术画像(采用的 SOTA + 代码位置)

下表汇总各子模块采用的技术方案、SOTA 出处、代码位置与配置化程度。

| 模块 | 采用方案 | SOTA 出处 | 代码位置 | 配置化度 |
| --- | --- | --- | --- | --- |
| 检测 | YOLO-World + YOLOE | CVPR 2024 + 2025 | `src/perception/detection/` | 高(可切换 BPU) |
| 编码 | CLIP + MobileClip | CVPR 2024 | `src/perception/encoding/` | 高 |
| 追踪 | USS-Nav 双指标融合(语义 Ωsem > 0.75 + 几何 Ωgeo > 0.1 的三阶梯匹配) | USS-Nav | `src/perception/tracking/instance_tracker.py` | 高 |
| 信念 | BA-HSG 贝叶斯信念(belief_alpha / beta + 深度依赖的观测方差) + EMA | 论文级 | `tracked_objects.py` | 中 |
| 场景图 | Floor → Room → Group → Object 五层结构 | Hydra / HOV-SG / SG-Nav | `scene_graph_builder.py` | 中 |
| 空间关系 | BBox-aware 距离 + cKDTree 加速,7 种关系(near / on / above / below / 左右 / 前后) | ConceptGraphs (ICRA 2024) | `scene_graph_builder.py` | 中 |
| 区域聚类 | DBSCAN(半径 3m) + 楼层 z 聚类 | Hydra | `scene_graph_builder.py` | 中 |
| 查询 | CLIP + KG 别名 + 子串级联;空间推理;affordance;floor;embedding kNN | EmbodiedRAG + SG-Nav + OpenFunGraph | `tracker_queries.py` | 高 |
| 房间推理 | LLM 异步命名(3 并发 / 15s 超时) + CLIP EMA 聚合 + 稳定性检测 | SG-Nav | `src/memory/spatial/room_inferencer.py` | 中 |
| KG 增强 | 概念注入 + 安全级别 + 功能属性 | ConceptBot + OpenFunGraph | `tracker_knowledge.py` | 中 |

---

## 二、七维度对齐度矩阵

从七个关键维度评估 LingTu 方案与对应 SOTA 的对齐程度(五星制)。

| # | 维度 | 我们的方案 | 对标 SOTA | 对齐度 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 1 | 开放词汇检测 | YOLO-World + YOLOE | CVPR 2024 明星作 | ★★★★★ | 对齐略领先(50MB 权重,Jetson 10+ FPS) |
| 2 | 特征编码 | CLIP ViT-B/32 + MobileClip | 工业标配 | ★★★★☆ | 对齐,未启用多尺度 |
| 3 | 实例追踪 | USS-Nav 双指标 + 贝叶斯信念 | LOVON / SG-Nav | ★★★★★ | 对齐且有特色,可引入 Hungarian |
| 4 | 分层场景图 | 5 层 Floor → Room → Group → Object | Hydra / HOV-SG | ★★★☆☆ | 层数对齐,缺 dense feature / place-level / 功能深度 |
| 5 | 语义查询 | CLIP + KG + 空间推理级联 | EmbodiedRAG | ★★★★☆ | 完善,缺完整 RAG 深度 |
| 6 | 房间推理 | LLM 异步命名 + CLIP 聚合 | SG-Nav | ★★★★☆ | 工程完善 |
| 7 | 边缘部署 | RDK X5 BPU 原生 | Jetson 系列 | ★★★★☆ | 适配好,缺实测数据 |

---

## 三、架构优势(相比学术原型)

相较于典型学术原型,LingTu perception 在工程架构上具备以下显著优势:

1. **Module-First 解耦**:perception ↔ memory 依赖反转。`room_inferencer` 单向消费 scene graph,`scene_graph_builder` 不 import memory,保证层次边界清晰。
2. **多后端可插拔**:检测器 / 编码器通过 `@register` 工厂模式,一行配置即可切换后端。
3. **配置全外化**:`robot_config.yaml` 集中管理检测 / 追踪 / 编码 / 查询的所有参数,无硬编码散落。
4. **确定性验证**:39 个 golden 快照测试 + phase 化重构,保障行为可回归。
5. **轻量追踪器**:`instance_tracker.py` 从 1787 行重构至 895 行,且无 GPU 依赖。
6. **点云融合**:mask → 3D 重建,提供精确的 extent 估计。

---

## 四、能力短板与差距

以下为当前实现相较前沿 SOTA 的主要能力缺口:

1. **缺密集开放词汇特征场**:仅有 per-object CLIP 特征,而 HOV-SG(RSS 2024)具备 pixel-level 的 dense feature grid。
2. **缺几何精细分割与 Panoptic 3D**:有 bbox + mask,但无 3D panoptic label,无法理解墙 / 地板 / 天花等 stuff 类别(对标 PanoGS,CVPR 2025)。
3. **缺闭环检测与持久化建图**:session-based,无 loop closure,reboot 后场景图消失(对标 Hydra,RSS 2018)。
4. **缺多楼层全场景扩展性**:单层支持,200 对象上限,O(n) 查询,楼层间连接不完善(对标 HOV-SG 多楼层)。
5. **缺功能性场景图深层推理**:affordance 为 tag-based lookup,无 VLM 深层推理(对标 OpenFunGraph,CVPR 2025)。
6. **缺动态环境更新机制**:无主动 change detection 与 dirty propagation(对标 DovSG,RA-L 2025)。

---

## 五、分优先级改进建议

### P0(立即,收益大)

| 编号 | 改进项 | 关键做法 | 工作量 | 风险 |
| --- | --- | --- | --- | --- |
| P0-1 | 补充 RDK X5 实测性能基准 | fps / latency / 功耗 实测 | 3–5 天 | 低 |
| P0-2 | 多楼层自动检测完善 | elevator / stairs 检测 + z-gap + 楼层桥接 | 2–3 天 | 中 |
| P0-3 | 对象去重与合并检测 | 同类别 + 距离 < 0.3m + CLIP > 0.85 + 未见 < 5s | 1–2 天 | 低 |

### P1(中期)

| 编号 | 改进项 | 关键做法 | 工作量 | 风险 |
| --- | --- | --- | --- | --- |
| P1-1 | 闭环检测与增量建图 | 场景图磁盘序列化 + place-recognition + graph matching merge | 5–7 天 | 中 |
| P1-2 | Room-level dense feature grid | 5×5 网格 CLIP embedding | 3–4 天 | 低 |
| P1-3 | Hungarian 关联替代贪心 | scipy `linear_sum_assignment`,n < 200 约 < 5ms | 2 天 | 低 |

### P2(长期,按需)

| 编号 | 改进项 | 关键做法 | 工作量 | 风险 |
| --- | --- | --- | --- | --- |
| P2-1 | Panoptic 3D 语义分割 | RDK X5 < 1Hz | — | 高 |
| P2-2 | 动态环境变化追踪 | DovSG-style change detection | 3–5 天 | 中 |
| P2-3 | 功能性场景图推理 | OpenFunGraph-style,VLM > 2GB,RDK X5 难支撑 | — | 高 |

### 改进优先级速查表

| 建议 | 优先级 | 收益 | 工作量 | 风险 | 技术债 | 推荐度 |
| --- | --- | --- | --- | --- | --- | --- |
| RDK X5 实测性能基准 | P0-1 | 高 | 3–5 天 | 低 | 低 | ★★★★★ |
| 多楼层自动检测完善 | P0-2 | 高 | 2–3 天 | 中 | 中 | ★★★★☆ |
| 对象去重与合并检测 | P0-3 | 中高 | 1–2 天 | 低 | 低 | ★★★★★ |
| 闭环检测与增量建图 | P1-1 | 高 | 5–7 天 | 中 | 中 | ★★★★☆ |
| Room-level dense feature grid | P1-2 | 中 | 3–4 天 | 低 | 低 | ★★★★☆ |
| Hungarian 关联替代贪心 | P1-3 | 中 | 2 天 | 低 | 低 | ★★★★☆ |
| Panoptic 3D 语义分割 | P2-1 | 中 | 高投入 | 高 | 高 | ★★☆☆☆ |
| 动态环境变化追踪 | P2-2 | 中 | 3–5 天 | 中 | 中 | ★★★☆☆ |
| 功能性场景图推理 | P2-3 | 中 | 高投入 | 高 | 高 | ★★☆☆☆ |

---

## 六、竞品对标表

将 LingTu 与代表性学术系统在关键能力维度上横向对比(✓✓ = 强,✓ = 具备,✗ = 缺失)。

| 系统 | 开放词汇检测 | 分层场景图 | 查询 API | 边缘适配 | 生产就绪 |
| --- | --- | --- | --- | --- | --- |
| **LingTu** | YOLO-W ✓✓ | 5 层 ✓ | 丰富 ✓✓ | RDK ✓✓ | 4.0 |
| Hydra (MIT) | 无 ✗ | 5 层 ✓✓ | 基础 ✓ | 无 ✗ | 研究 |
| HOV-SG (RSS 2024) | 无 ✗ | 5 层 ✓✓ | 基础 ✓ | 否 ✗ | 研究 |
| SG-Nav (NeurIPS 2024) | 无 ✗ | 简化 ✓ | LLM 驱动 ✓✓ | 无 ✗ | 研究 |
| DovSG (RA-L 2025) | GroundDINO ✓ | 动态 ✓✓ | 丰富 ✓✓ | 否 ✗ | 研究 |

---

## 七、总体结论与评分

| 评估维度 | 评分 | 星级 |
| --- | --- | --- |
| 技术先进性 | 4.0 | ★★★★☆ |
| 工程化水平 | 4.5 | ★★★★★ |
| 边缘适配 | 4.5 | ★★★★☆ |
| 学术完成度 | 3.5 | ★★★☆☆ |
| 生产就绪度 | 4.0 | ★★★★☆ |
| **综合** | **4.1 / 5.0** | **★★★★☆** |

**适合场景**:边缘平台实时感知、中小型室内导航(< 300 对象)、快速原型、生产级稳定性需求。

**不适合场景**:大型多楼层建筑、长期 SLAM 建图、稠密语义理解、高端 GPU 平台。

**演进路线图**:

- **Phase 1(2–3 周)**:补齐 RDK X5 实测基准。
- **Phase 2(1 月)**:多楼层 + 对象去重 + Hungarian 关联。
- **Phase 3(2–3 月)**:闭环建图 + dense feature grid。
- **Phase 4(按需)**:panoptic / 动态环境 / 功能性推理。

---

> **评估日期**:2026-07-08
> **评估方法**:代码交叉验证 + 联网 SOTA 检索
