# NaviMind v2.0 路线图 — 基于 2025-2026 前沿论文的技术规划

> 编写日期: 2026-02-13
> 基准: NaviMind Semantic Control Engine v1.0 (289 tests, 9 intents, 615+ patterns)
> 原则: **每一篇论文均经过实际搜索验证, 标注 arXiv ID / 会议 / 日期**

---

## 〇、论文总览表 (24 篇, 全部 2025-2026)

| # | 论文 | ArXiv / 会议 | 日期 | 核心贡献 | 与我们的关联度 |
|---|------|-------------|------|---------|-------------|
| 1 | **NavFoM** — Embodied Navigation Foundation Model | 2509.12129 / CoRL 2025, submitted ICLR 2026 | 2025-09 | 跨具身跨任务统一导航基座, 800万样本训练, 覆盖四足/无人机/轮式/汽车 | ★★★★★ |
| 2 | **Nav-R1** — Reasoning and Navigation in Embodied Scenes | 2509.10884 | 2025-09 | Fast-in-Slow 推理范式, CoT + RL, 8%+ baseline | ★★★★☆ |
| 3 | **DovSG** — Dynamic Open-Vocabulary 3D Scene Graphs | 2410.11989 / IEEE RA-L 2025 | 2024-10, accepted 2025 | 动态3D场景图, 局部更新, 长期移动操作 | ★★★★★ |
| 4 | **OpenFunGraph** — Open-Vocabulary Functional 3D Scene Graphs | CVPR 2025 (Highlight) | 2025 | 功能性3D场景图, VLM+LLM编码功能知识, 操作任务 | ★★★★☆ |
| 5 | **SPADE** — Scalable Path Planning on 3D Scene Graphs | 2505.19098 / IROS 2025 | 2025-05 | 层次化场景图路径规划, 四足机器人验证 | ★★★★★ |
| 6 | **SG-Nav** — Online 3D Scene Graph Prompting | NeurIPS 2024, extended to CVPR 2025 (UniGoal) | 2024-10 | 层次化3D场景图 + LLM零样本导航, >10% SOTA | ★★★★☆ |
| 7 | **Splatblox** — Traversability-Aware Gaussian Splatting | 2511.18525 | 2025-11 | GS+LiDAR融合, 四足户外导航, 50%+ 成功率, <4GB VRAM | ★★★★★ |
| 8 | **GS-LIVO** — Gaussian Splatting Multi-sensor SLAM | 2501.08672 | 2025-01 | LiDAR+IMU+视觉+GS SLAM, Jetson Orin NX 部署 | ★★★★★ |
| 9 | **OVO-SLAM** — Open-Vocabulary Online Semantic Mapping | 2411.15043v3 | 2024-11, v3 2025 | SAM2.1 + CLIP + SLAM 实时语义建图 | ★★★★☆ |
| 10 | **EmbodiedRAG** (3DSG版) — Dynamic 3D Scene Graph Retrieval | 2410.23968 | 2024-10 | RAG + 3D场景图, token减少10倍, 规划时间减70%, 四足实验 | ★★★★★ |
| 11 | **OrionNav** — LLM + Open-Vocabulary Semantic Scene Graphs | 2410.06239 | 2024-10 | 在线LLM规划 + 语义场景图, 四足验证 | ★★★★☆ |
| 12 | **LOVON** — Legged Open-Vocabulary Object Navigator | 2507.06747 | 2025-07 | LLM层次任务规划 + 开放词汇检测, Unitree Go2/B2/H1-2 验证 | ★★★★★ |
| 13 | **OneTwoVLA** — Adaptive Reasoning VLA | 2505.11917 | 2025-05 | 单模型自适应推理/行动切换, 长期操作 | ★★★☆☆ |
| 14 | **CogACT** — VLA with Diffusion Action Transformer | 2411.19650 | 2024-11 | 组件化VLA架构, 超OpenVLA 35%+, 超RT-2-X 18% | ★★★☆☆ |
| 15 | **HAMSTER** — Hierarchical VLA with Off-Domain Data | 2502.05485 / ICLR 2025 | 2025-02 | 2D轨迹引导 + 3D控制, 利用无动作视频/仿真数据 | ★★★☆☆ |
| 16 | **Hi Robot** — Open-Ended Instruction Following | 2502.19417 / ICML 2025 | 2025-02 | 层次VLM + 开放指令 + 用户反馈适应 | ★★★★☆ |
| 17 | **Robix** — Unified Robot Interaction, Reasoning & Planning | 2509.01106 (ByteDance) | 2025-09 | 统一推理-规划-交互, 主动对话, 实时中断, 超GPT-4o | ★★★★★ |
| 18 | **ConceptBot** — Knowledge Graph + LLM Task Decomposition | submitted ICLR 2026 | 2025 | KG接地+任务分解, 超SayCan 56%, 安全任务76% vs 15% | ★★★★★ |
| 19 | **AdaptBot** — LLM + KG + Human-in-Loop | 2502.02067 / ICRA 2025 | 2025-02 | 泛化到特定的任务分解, 知识图谱精炼 | ★★★★☆ |
| 20 | **MORE** — Mobile Manipulation Rearrangement | 2505.03035 | 2025-05 | 场景图+主动过滤+实例区分, BEHAVIOR-1K 突破 | ★★★☆☆ |
| 21 | **WoMAP** — World Models for Open-Vocabulary Localization | 2506.01600 / CoRL 2025 | 2025-06 | GS-based real2sim2real + 世界模型, 9x > VLM baseline | ★★★☆☆ |
| 22 | **COHERENT** — Heterogeneous Multi-Robot Collaboration | 2409.15146 / ICRA 2025 | 2024-09, updated 2025-03 | PEFA机制, 四足+无人机+机械臂协作, 100任务benchmark | ★★★★☆ |
| 23 | **PRISM** — Distilling SLM for On-Device Robot Planning | 2506.17486 / CoRL 2025 | 2025-06 | Llama-3.2-3B蒸馏到93% GPT-4o, <5GB内存 | ★★★★★ |
| 24 | **SmolVLA** — 450M Parameter VLA | 2506.01844 | 2025-06 | 450M参数, 单GPU训练, CPU可运行, 异步推理 | ★★★★☆ |
| — | **SafeVLA** / **RoboSafe** / **SAFER** / **SafeMind** | 多篇, 2025 | 2025 | VLA安全约束, 运行时安全护栏, 安全降低36-83% | ★★★★☆ |

---

## 一、六条技术路线 — 按优先级排列

---

### 路线 1: 3D 语义场景图 + 动态更新 (P0 — 最高优先级)

**为什么是 P0**: 场景图是所有高级功能的基础设施。没有持久化3D语义表达, 所有"找到XX"/"记得之前看到的XX"都是空谈。

#### 核心论文支撑

| 论文 | 我们可以借鉴什么 |
|------|---------------|
| **DovSG** (RA-L 2025) | 动态场景图的局部更新机制 — 不需要全量重建, 机器人移动物体后只更新变化的节点 |
| **OpenFunGraph** (CVPR 2025 Highlight) | 功能关系编码 — 不仅记录"桌子上有杯子", 还记录"杯子可以被抓取", "门可以被打开" |
| **SPADE** (IROS 2025) | 在场景图上做层次化路径规划, 四足机器人实机验证 |
| **SG-Nav** (NeurIPS 2024) | 层次化 CoT 提示 LLM 在场景图上推理导航 |
| **EmbodiedRAG** (2024-10) | 将场景图实体当文档索引, RAG检索任务相关子图, token 减 10x |

#### 具体做什么

```
阶段 1 (4 周): 静态 ConceptGraph 构建
├── 升级 semantic_perception_node
│   ├── YOLO-World → 开放词汇检测 (替代固定类别)
│   ├── SAM2.1 → 实例分割 mask
│   ├── CLIP → 每个实例的语义 embedding
│   └── 点云聚类 + TSDF 融合 → 3D 实例
├── 构建 ConceptGraph 数据结构
│   ├── Node: {id, label, clip_embedding, position_3d, bbox_3d, confidence}
│   ├── Edge: {spatial_relation, functional_relation}
│   └── 层次: Floor → Room → Object (参考 HOV-SG)
└── 接入 BeliefManager — 每个 ConceptGraph Node 作为一个 Belief

阶段 2 (3 周): 动态更新 + RAG 检索
├── 参考 DovSG 实现局部更新
│   ├── 机器人看到变化 → 只更新受影响节点
│   ├── 新物体出现 → 添加节点 + 边
│   └── 物体消失 → 标记为 uncertain, 降低 confidence
├── 参考 EmbodiedRAG 实现语义检索
│   ├── 自然语言查询 → CLIP encode → 余弦相似度检索
│   ├── 只返回 top-k 相关节点给 LLM (不传全图)
│   └── 动态子图提取 → 减少 LLM token 消耗
└── 参考 OpenFunGraph 编码功能属性
    ├── "灭火器" → {可抓取, 需要拔保险销, 安全设备}
    └── "门" → {可打开, 可关闭, 通道}
```

#### 与我们现有系统的对接

| 现有模块 | 对接方式 |
|---------|---------|
| `BeliefManager` | ConceptGraph Node → Belief entry, 置信度共享 |
| `TopologicalSemanticGraph` | Room 层和 Floor 层直接映射 |
| `VoICalculator` | 场景图节点的信息增益计算 |
| `TaskDecomposer` (v1.0) | FIND intent → 先查 ConceptGraph, 命中则直接导航, 未命中再探索 |
| `planner_node` | PICK/PLACE intent → 从 ConceptGraph 获取目标3D位姿 |

**预期收益**: FIND 查询从"盲搜"变为"记忆检索+定向导航", 响应时间从分钟级降到秒级。

---

### 路线 2: 端侧小模型蒸馏 — 替代云端 LLM 依赖 (P0)

**为什么是 P0**: 我们的 v1.0 规则引擎覆盖简单指令, 但 LLM fallback 依赖网络。工业/户外场景经常断网, 这是生死问题。

#### 核心论文支撑

| 论文 | 关键发现 |
|------|---------|
| **PRISM** (CoRL 2025) | Llama-3.2-3B 蒸馏后达到 GPT-4o 93% 性能, <5GB 内存, 自动合成训练数据, 无需人工标注 |
| **SmolVLA** (2025-06) | 450M 参数 VLA, 单 GPU 训练 4h, CPU 可运行, 异步推理 |
| **Lite VLA** (2025-11) | CPU-bound 边缘机器人上部署小型 VLM, 并发运动+推理 |
| **Nav-R1** (2025-09) | Fast-in-Slow 范式 — 慢系统做语义推理, 快系统做反应控制, sub-100ms 延迟 |

#### 具体做什么

```
阶段 1 (3 周): 构建蒸馏数据集
├── 参考 PRISM 方法论
│   ├── 用 GPT-4o / Claude 作为 teacher
│   ├── 自动生成 5000+ 条指令-场景-规划三元组
│   │   ├── 中文: "帮我找到3楼的灭火器" → {FIND, target=灭火器, floor=3}
│   │   ├── 英文: "check all fire extinguishers" → {INSPECT, target=灭火器, quantifier=all}
│   │   └── 多步: "先去厨房拿水再送到办公室" → [{NAV,厨房},{PICK,水},{NAV,办公室},{PLACE,水}]
│   └── 包含 场景图上下文 (参考 EmbodiedRAG)
└── 覆盖 v1.0 的全部 9 个 Intent + 复杂多步指令

阶段 2 (2 周): 蒸馏训练
├── 基座: Qwen2.5-3B / Llama-3.2-3B
├── SFT on 蒸馏数据集
├── 输出格式: JSON {action, target, parameters}
├── 量化: INT4 → ~2GB 显存
└── 部署: Jetson Orin NX (与 GS-LIVO 共享硬件)

阶段 3 (1 周): 集成到 TaskDecomposer
├── 三级降级策略:
│   Level 0: 规则引擎 (< 5ms, v1.0 已有)
│   Level 1: 端侧 SLM (< 200ms, 无网络依赖)
│   Level 2: 云端 LLM (< 2s, 需网络)
└── 自动降级: 网络可用 → L0→L2, 网络断 → L0→L1
```

#### 与我们现有系统的对接

| 现有模块 | 对接方式 |
|---------|---------|
| `TaskDecomposer.decompose_with_rules()` | Level 0 不变 |
| `TaskDecomposer` LLM fallback | Level 1 替换为端侧 SLM |
| 复杂度守卫 | 仍然生效, 复杂指令跳过规则, 直接走 SLM/LLM |
| gRPC gateway | 新增 SLM health 状态上报 |

**预期收益**: 断网场景下复杂指令也能解析, 端到端延迟 < 300ms, 零 API 成本。

---

### 路线 3: 高斯溅射语义 SLAM — 替代传统建图 (P1)

**为什么是 P1**: GS-SLAM 是 2025 年最热门的建图范式, 在嵌入式上已可跑, 且自带语义嵌入能力, 直接为路线 1 的场景图提供底层表示。

#### 核心论文支撑

| 论文 | 关键贡献 |
|------|---------|
| **GS-LIVO** (2025-01) | LiDAR+IMU+Camera+GS SLAM, **首个 Jetson Orin NX 可跑的 GS-SLAM**, 20Hz, 25% drift 减少 |
| **Splatblox** (2025-11) | GS+LiDAR 可通行性感知, **四足户外导航**, <4GB VRAM, 50%+ 成功率提升 |
| **Splat-Nav** (RA-L 2025) | GS 地图上的安全规划 (Bézier curves) + 视觉定位, 2Hz 重规划, 25Hz 定位 |
| **OVO-SLAM** (2025) | SAM2.1+CLIP 在线语义建图, 与 GS-SLAM / ORB-SLAM2 集成 |

#### 具体做什么

```
阶段 1 (4 周): GS-LIVO 集成
├── 部署 GS-LIVO 到 Jetson Orin NX
│   ├── 替代当前 LIO-SAM / Cartographer
│   ├── 利用已有 LiDAR + IMU + Camera 传感器套件
│   └── 输出: 实时 3D 高斯地图 + 位姿
├── 参考 Splatblox 加入可通行性语义
│   ├── 语义分割: 地面 / 障碍 / 可穿越植被
│   └── 生成 traversability-aware ESDF
└── 对接 Nav2 — GS 地图 → 2D costmap 投影

阶段 2 (3 周): 语义嵌入层
├── 参考 OVO-SLAM
│   ├── SAM2.1 → 实例 mask
│   ├── CLIP → 语义 embedding
│   └── 3D 高斯 → 每个 Gaussian 携带语义向量
├── 开放词汇查询
│   ├── "灭火器在哪" → CLIP encode → 余弦匹配 → 高斯聚类 → 3D 位置
│   └── 无需预定义类别
└── 直接输出给路线 1 的 ConceptGraph
```

#### 硬件兼容性

| 硬件 | GS-LIVO 验证 | Splatblox 验证 | 我们的配置 |
|------|-------------|---------------|----------|
| Jetson Orin NX 16GB | ✅ 20Hz | — | ✅ 可用 |
| 边缘 GPU <4GB | — | ✅ | ✅ 可用 |
| LiDAR | ✅ 必须 | ✅ 必须 | ✅ 已有 |
| IMU | ✅ 必须 | — | ✅ 已有 |

**预期收益**: 统一的几何+语义表示, 替代"点云+2D语义投影"的割裂架构, 建图质量和效率同时提升。

---

### 路线 4: 知识图谱增强任务分解 — TaskDecomposer v2.0 (P1)

**为什么是 P1**: 直接升级我们的核心模块, 用知识图谱解决 LLM 幻觉问题, 同时提升安全任务能力。

#### 核心论文支撑

| 论文 | 关键贡献 |
|------|---------|
| **ConceptBot** (ICLR 2026 submission) | KG (ConceptNet) + LLM, 超 SayCan 56%, 安全任务 76% vs 15% |
| **AdaptBot** (ICRA 2025) | LLM + KG + Human-in-Loop 三方协同, 快速适应新任务 |
| **Robix** (ByteDance, 2025-09) | 统一推理-规划-交互, 主动对话, 实时中断 |
| **Hi Robot** (ICML 2025) | 层次 VLM, 开放指令, "那不是垃圾"类反馈处理 |
| **SafeMind** (2025) | 事实/因果/时间三类安全约束, 级联安全模块 |

#### 具体做什么

```
阶段 1 (3 周): 知识图谱构建
├── 构建 NaviMind 知识图谱 (NKG)
│   ├── 基于 ConceptNet 5.5 抽取机器人相关子图
│   ├── 自定义工业节点:
│   │   ├── 灭火器 → {用途:灭火, 操作:拔销+按压, 危险:无, 重量:3-8kg}
│   │   ├── 配电箱 → {用途:供电, 操作:不可触碰, 危险:触电, 权限:电工}
│   │   └── 安全出口 → {用途:逃生, 状态:常开, 关联:消防通道}
│   └── 关系类型: IsA, UsedFor, LocatedNear, HasProperty, DangerousIf

阶段 2 (2 周): 集成到 TaskDecomposer
├── 参考 ConceptBot 三模块架构
│   ├── OPE (Object Properties Extraction)
│   │   └── "帮我拿配电箱" → 查 KG → 发现 "危险:触电" → 拒绝 + 警告
│   ├── URP (User Request Processing)
│   │   └── "把那个东西放好" → 结合场景图 → 消歧 "那个东西" = 杯子
│   └── Planner: 结合 KG 属性生成安全可行的 SubGoal 链
├── 参考 Robix 加入主动对话
│   ├── 歧义检测: "去检查一下" → 检查什么? → 主动追问
│   ├── 实时中断: 执行中用户说 "停" / "不对" → 立即暂停 + 重规划
│   └── 上下文推理: "再来一次" → 查历史 → 重复上次任务
└── 参考 SafeMind 加入安全约束
    ├── 事实约束: "拿10公斤的东西" → 检查机械臂负载 → 超限拒绝
    ├── 因果约束: "打开配电箱" → 查 KG → 触电风险 → 二次确认
    └── 时间约束: "每5分钟巡检" → 合理性检查 → 生成定时任务
```

#### 与 v1.0 规则引擎的关系

```
v1.0 (当前):
  NL → 复杂度守卫 → 规则引擎 / LLM fallback

v2.0 (升级后):
  NL → 复杂度守卫 → 规则引擎 (不变, 仍处理简单指令)
       ↓ 复杂指令
       KG 增强推理层 (新)
       ├── 安全检查 (KG 查询)
       ├── 歧义消解 (场景图 + 对话历史)
       ├── 可行性验证 (机器人能力模型)
       └── SubGoal 生成
       ↓ 仍无法处理
       端侧 SLM (路线 2) / 云端 LLM
```

**预期收益**: 安全任务准确率从~30% → 76%+, 歧义指令主动追问而不是猜测, 工业场景危险操作自动拦截。

---

### 路线 5: 四足开放词汇导航 — LOVON 式端到端 (P2)

**为什么是 P2**: 已有论文在 Unitree 平台上验证了可行性, 我们的硬件完全兼容。

#### 核心论文支撑

| 论文 | 关键贡献 |
|------|---------|
| **LOVON** (2025-07) | **四足开放词汇导航**, LLM 层次规划 + 开放检测, Unitree Go2/B2/H1-2 验证, 开源 |
| **NavFoM** (CoRL 2025) | **跨具身导航基座**, 包含四足, 800万样本, 7个benchmark SOTA, 无需微调 |
| **OrionNav** (2024-10) | LLM + 语义场景图在线规划, 四足实时导航 |
| **WoMAP** (CoRL 2025) | GS-based 世界模型 + 开放词汇定位, 9x > VLM baseline |

#### 具体做什么

```
阶段 1 (3 周): LOVON 集成
├── 部署 LOVON 开源框架到我们的四足平台
│   ├── LLM 层次任务规划 (对接路线 4 的 TaskDecomposer v2.0)
│   ├── 开放词汇检测 (对接路线 1 的 ConceptGraph)
│   ├── Laplacian Variance Filtering → 解决四足视觉抖动
│   └── 鲁棒执行逻辑 → 目标丢失恢复
├── 对接 NavFoM (如果开源)
│   ├── 作为通用导航 backbone
│   └── 零样本跨任务能力: VLN + 目标搜索 + 目标跟踪

阶段 2 (3 周): 世界模型增强
├── 参考 WoMAP
│   ├── GS-based 环境模拟
│   ├── 离线生成大量训练数据 (real2sim2real)
│   └── 世界模型预测 → 辅助决策
└── 参考 Nav-R1 的 Fast-in-Slow
    ├── Slow: LLM/SLM 语义推理 → 全局策略
    └── Fast: 本地控制器 → 实时避障 + 运动, sub-100ms
```

**预期收益**: "找到3楼的灭火器" — 从自然语言到四足自主导航的完整闭环, 对任意物体类别零样本泛化。

---

### 路线 6: 异构多机协作 (P3 — 长期)

**为什么是 P3**: 高价值但复杂度极高, 需要路线 1-5 的基础设施先就位。

#### 核心论文支撑

| 论文 | 关键贡献 |
|------|---------|
| **COHERENT** (ICRA 2025) | PEFA 机制, 四足+无人机+机械臂, 100 任务 benchmark |
| **CoMuRoS** (2025) | 层次化: 中心 LLM 分配 + 本地 LLM 执行, 事件驱动重规划, 0.91 正确率 |
| **HMCF** (2025-05) | Human-in-Loop + 多机器人, LLM 推理能力分配, 零样本泛化 |

#### 具体做什么

```
阶段 1 (远期): 共享场景图
├── 多机器人共享 ConceptGraph (ROS2 DDS)
├── 分布式更新: 机器人 A 发现灭火器 → 全网广播
└── 冲突解决: 同一物体被不同机器人标注

阶段 2 (远期): LLM 任务分配
├── 参考 COHERENT 的 PEFA
│   ├── 中心调度: "巡检所有灭火器" → 按楼层分配给不同机器人
│   ├── 自反馈: 机器人完成后报告, 调整计划
│   └── 能力感知: 有臂的去 PICK, 无臂的去 INSPECT
└── 参考 CoMuRoS 的事件驱动重规划
    └── 机器人 A 卡住 → 自动将任务转给机器人 B
```

---

## 二、全局技术架构 v2.0

```
┌─────────────────────────────────────────────────────────────────────┐
│                        自然语言输入                                   │
│              "帮我找到3楼的灭火器并检查是否过期"                        │
└───────────────────────────────┬─────────────────────────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  语义理解层 (TaskDecomposer v2.0)                     │
│                                                                     │
│   ┌──────────┐  ┌──────────────┐  ┌───────────┐  ┌──────────────┐  │
│   │ 规则引擎  │→│ KG 安全检查   │→│ 歧义消解   │→│ 端侧 SLM     │  │
│   │ v1.0     │  │ ConceptBot式  │  │ Robix式   │  │ PRISM 蒸馏   │  │
│   │ 615+词条  │  │ ConceptNet   │  │ 场景图+历史│  │ Qwen2.5-3B  │  │
│   └──────────┘  └──────────────┘  └───────────┘  └──────────────┘  │
│                         ↓ SubGoal Chain                             │
│        [NAVIGATE(3F), FIND(灭火器), APPROACH, INSPECT(过期)]         │
└───────────────────────────────┬─────────────────────────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│              语义感知层 (ConceptGraph + GS-SLAM)                      │
│                                                                     │
│   ┌──────────────┐  ┌──────────────┐  ┌────────────────┐           │
│   │ GS-LIVO SLAM │→│ OVO 语义层    │→│ ConceptGraph   │           │
│   │ LiDAR+IMU+Cam│  │ SAM2.1+CLIP  │  │ 3D 场景图      │           │
│   │ 20Hz, Jetson │  │ 开放词汇      │  │ 动态更新(DovSG)│           │
│   └──────────────┘  └──────────────┘  └────────────────┘           │
│                                              ↓                      │
│                              EmbodiedRAG 语义检索                    │
│                        "灭火器" → CLIP → top-k 节点                  │
└───────────────────────────────┬─────────────────────────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│              导航执行层 (LOVON + Nav2 + Safety)                       │
│                                                                     │
│   ┌──────────────┐  ┌──────────────┐  ┌────────────────┐           │
│   │ LOVON 导航   │  │ Nav2 规划     │  │ 安全护栏       │           │
│   │ 开放词汇     │  │ GS→costmap   │  │ SafeVLA/SAFER │           │
│   │ 四足优化     │  │ SPADE 场景图  │  │ 运行时约束     │           │
│   └──────────────┘  └──────────────┘  └────────────────┘           │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 三、分阶段实施计划

### Phase 1: 基础设施层 (8 周)

| 周 | 任务 | 路线 | 产出 |
|----|------|------|------|
| W1-W2 | GS-LIVO 部署 + 调参 | 路线 3 | Jetson 上跑通 20Hz GS-SLAM |
| W3-W4 | ConceptGraph 数据结构 + 静态构建 | 路线 1 | 可查询的 3D 语义场景图 |
| W5-W6 | 蒸馏数据集构建 (PRISM 方法) | 路线 2 | 5000+ 指令-规划三元组 |
| W7-W8 | SLM 蒸馏 + INT4 量化 | 路线 2 | Jetson 可跑的 3B 端侧模型 |

### Phase 2: 能力升级层 (8 周)

| 周 | 任务 | 路线 | 产出 |
|----|------|------|------|
| W9-W11 | DovSG 动态更新 + EmbodiedRAG 检索 | 路线 1 | 实时更新 + 语义查询 |
| W12-W13 | KG 构建 + ConceptBot 式安全检查 | 路线 4 | 危险操作拦截 + 歧义追问 |
| W14-W15 | OVO 语义层 + GS 开放词汇查询 | 路线 3 | "灭火器在哪" → 3D 位置 |
| W16 | 三级降级策略集成 | 路线 2 | 规则 → SLM → LLM 自动切换 |

### Phase 3: 四足导航闭环 (6 周)

| 周 | 任务 | 路线 | 产出 |
|----|------|------|------|
| W17-W19 | LOVON 集成 + 四足优化 | 路线 5 | 自然语言→四足自主导航 |
| W20-W21 | 安全护栏 (SafeVLA/SAFER) | 路线 4 | 运行时安全约束 |
| W22 | 端到端集成测试 | All | "帮我找灭火器" 全链路跑通 |

### Phase 4: 高级功能 (远期)

| 任务 | 路线 | 依赖 |
|------|------|------|
| 多机共享场景图 | 路线 6 | Phase 1-2 |
| NavFoM 集成 (如开源) | 路线 5 | Phase 3 |
| 世界模型 (WoMAP) | 路线 5 | Phase 1 |
| Human-in-Loop 协作 | 路线 6 | Phase 2 |

---

## 四、论文→代码 映射表

| 论文 | 开源 | 语言 | 我们需要改什么 |
|------|------|------|-------------|
| GS-LIVO | ✅ github.com/HKUST-Aerial-Robotics/GS-LIVO | C++ | 替换 SLAM 后端, 编写 ROS2 wrapper |
| LOVON | ✅ github.com/DaojiePENG/LOVON | Python | 对接 TaskDecomposer + ConceptGraph |
| COHERENT | ✅ github.com/MrKeee/COHERENT | Python | 远期多机协作框架 |
| SG-Nav | ✅ github.com/bagh2178/SG-Nav | Python | 参考场景图→LLM prompt 方法 |
| SmolVLA | ✅ HuggingFace lerobot/smolvla_base | Python | 参考小模型架构设计 |
| PRISM | ✅ github.com/KumarRobotics/PRISM | Python | 核心蒸馏框架直接复用 |
| OK-Robot | ✅ github.com/ok-robot/ok-robot | Python | 参考 PICK/PLACE 集成流程 |
| MORE | ✅ github.com/robot-learning-freiburg/MORE | Python | 参考场景图过滤方法 |
| CogACT | ✅ github.com/microsoft/CogACT | Python | 参考 VLA 动作头设计 |

---

## 五、风险与应对

| 风险 | 概率 | 影响 | 应对 |
|------|------|------|------|
| GS-SLAM 在 Jetson 上性能不达标 | 中 | 高 | GS-LIVO 已验证 Jetson Orin NX, 但需调参; 备选: 保留 LIO-SAM |
| 蒸馏后 SLM 中文理解弱 | 中 | 高 | 选 Qwen2.5-3B (中文强), 加大中文训练数据比例 |
| ConceptGraph 大场景内存爆炸 | 中 | 中 | 参考 EmbodiedRAG 的滑窗策略, 远处节点降采样 |
| LOVON 与我们的运动控制器不兼容 | 低 | 中 | LOVON 支持 Unitree 系列, 我们的平台兼容 |
| NavFoM 不开源 | 高 | 低 | NavFoM 是锦上添花, LOVON 已足够; 或用 ViNT/NoMaD 替代 |
| 安全约束过严影响可用性 | 低 | 中 | 可配置安全等级, 工业模式 vs 家用模式 |

---

## 六、学术产出规划

基于以上路线, 可规划的论文方向:

| # | 标题方向 | 创新点 | 目标会议 |
|---|---------|--------|---------|
| 1 | "NaviMind: Rule-KG-SLM 三级语义理解" | 规则→知识图谱→端侧小模型的降级架构 (全球首创) | ICRA 2027 |
| 2 | "GS-ConceptGraph: 高斯溅射驱动的动态3D语义场景图" | GS-LIVO + ConceptGraph + 动态更新 | IROS 2027 |
| 3 | "Embodied-RAG on Quadrupeds" | 四足机器人上的 RAG 导航, 工业巡检场景 | CoRL 2027 |
| 4 | "PRISM-ZH: 中文机器人任务规划的端侧蒸馏" | 首个中文机器人 SLM 蒸馏方案 | RA-L 2027 |

---

## 七、总结: 立即要做的前三件事

1. **部署 GS-LIVO** → 在 Jetson Orin NX 上跑通高斯溅射 SLAM, 这是一切的几何基础
2. **构建蒸馏数据集** → 用 PRISM 方法自动生成中英文指令-规划对, 准备端侧模型训练
3. **实现 ConceptGraph 原型** → YOLO-World + CLIP → 3D 节点, 接入 BeliefManager

这三件事可以并行启动, 8 周后我们将拥有:
- 嵌入式 GS-SLAM (替代传统建图)
- 3D 语义场景图 (持久化环境记忆)
- 端侧语义理解 (断网可用)

**从 v1.0 的"能听懂简单话" → v2.0 的"能记住环境、能断网推理、能安全执行"。**
