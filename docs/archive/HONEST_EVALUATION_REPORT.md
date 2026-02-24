# 语义导航系统 — 诚实技术评估报告

**测试日期**: 2026-02-15
**测试环境**: Windows 11, Python 3.13.5 (非 Jetson 目标平台)
**代码版本**: 3D-NAV 语义导航 v1.1 (修复空间推理 / CLIP伪分数 / 中文分词后)

---

## 1. 前次报告的核心问题

上一份报告 (`FAST_SLOW_PERFORMANCE_REPORT.md`) 存在以下严重误导:

| 问题 | 说明 |
|------|------|
| **测试场景只有目标物体** | 每个 benchmark 场景只放了 1 个目标物体，没有干扰物。等价于 "空房间里只有一把椅子，问：椅子在哪？" — 100% 正确不说明任何能力 |
| **CLIP 分数是伪造的** | 无真实 CLIP 编码器时，代码用 `clip_score = label_score * 0.8` 伪造。报告号称 "CLIP 验证通过" 但实际上从未运行过 CLIP 推理 |
| **阈值为伪分数量身定做** | `fast_path_threshold=0.75` 恰好能让 `0.35*1.0 + 0.35*0.8 + 0.15*0.9 = 0.765` 通过。这是循环论证 |
| **命中率 90% 的含义被夸大** | 90% 命中率在全是目标物体的场景里毫无意义。任何 `if label in instruction` 都能做到 |
| **"超过论文 1000 倍"** | 论文的 Fast Path 包含 CLIP 推理 (~50ms on GPU)，我们的只是字符串匹配 (<1ms)。不是"更快"，是少做了关键工作 |
| **空间推理测试实际失败** | "find chair near the door" 选了 door 而非 chair，报告将此归为 "需要调整测试用例" 而非代码 bug |

---

## 2. 本次修复清单

### Bug 1: 空间推理主语/修饰语不区分 (Critical)

**问题**: "find chair near the door" 中，chair 和 door 都匹配 `label in inst_lower`，door 检测分更高 → 选了 door。

**修复**: 新增 `_parse_instruction_roles()` 方法，通过正则模式提取:
- **主语** (subject): "find **chair** near the door" → chair, label_score=1.0
- **修饰语** (modifier): "find chair near the **door**" → door, label_score=0.3

支持模式:
- 英文: `find X near/by/beside/next to Y`
- 中文: `Y旁边/附近/左边的X`

**测试验证**: 3 个专门的空间推理测试全部通过 (含 chair 检测分故意低于 door 的对抗场景)。

### Bug 2: CLIP 伪分数导致虚假融合 (Critical)

**问题**: 无 CLIP 编码器时，`clip_score = label_score * 0.8` 制造了不存在的"多模态信号"。

**修复**: 无 CLIP 时使用重分配权重:
```
有 CLIP: label=0.35, clip=0.35, det=0.15, spatial=0.15
无 CLIP: label=0.55, det=0.25, spatial=0.20
```
不再伪造任何数据源。

### Bug 3: 中英混合文本分词错误 (Medium)

**问题**: jieba 处理 "找fire extinguisher" 时将 "找" 和 "f" 粘连，导致 "fire" 被切断。

**修复**: 先分离中英文文本再分别处理:
1. 中文部分 → jieba 精确分词
2. 英文部分 → regex 提取 (`[a-zA-Z]+`)
3. 合并去重

---

## 3. 诚实的测试结果

### 3.1 测试套件总览

| 包 | 通过 | 失败 | 跳过 | 说明 |
|----|------|------|------|------|
| `semantic_planner` | **108** | **0** | **4** | 4 个 skipped 需要真实 LLM API key |
| `semantic_perception` | **14** | **27** | **0** | CLIP/YOLO 测试因 API 接口不一致失败 |

### 3.2 Fast Path 命中率 (含干扰物体)

**测试方法**: 每个场景包含 1 个目标 + 8 个随机干扰物体 (随机位置和检测分)。

| 指令类型 | 通过/总数 | 命中率 | 说明 |
|----------|-----------|--------|------|
| 简单英文 ("go to X") | 10/10 | **100%** | 精确标签匹配 + 高检测分 |
| 目标不在场景中 | 正确 defer | **100%** | "find elephant" → None |

**诚实评价**: 
- 当前 100% 命中率仅限于 **精确标签匹配 + 高检测分 + 无属性歧义** 的情况
- **已知局限**: "find red chair" 会匹配 "blue chair" (无 CLIP，不能区分颜色属性)
- **论文 VLingNav 70%** 是在 R2R/REVERIE 真实数据集上，包含模糊指令、属性歧义、视角变化 — 不可直接对比

### 3.3 空间关系推理

| 测试 | 结果 | 说明 |
|------|------|------|
| "find chair near the door" (chair 分低于 door) | **chair ✓** | 主语/修饰语区分生效 |
| "find chair near the door" (chair 分高) | **chair ✓** | 基础场景 |
| "找门旁边的椅子" (中文) | **椅子 ✓** | 中文介词模式匹配 |

### 3.4 响应时间

| 场景大小 | 平均耗时 | P99 | 说明 |
|----------|----------|-----|------|
| 50 物体 | **< 0.5ms** | < 1ms | 纯字符串匹配，无 CLIP |
| 200 物体 | **< 2ms** | < 5ms | 线性扫描，无优化 |

**诚实评价**: 这个速度 **不可与论文对比**。论文 Fast Path 包含:
- CLIP 特征提取 (~20-50ms on GPU)
- 特征相似度计算 (~5-10ms)
- 场景图查询 (~1-5ms)

我们只做了最后一步。

### 3.5 ESCA Token 减少

| 原始物体数 | 过滤后 | 减少率 | 说明 |
|------------|--------|--------|------|
| 201 | 15 | **92.5%** | keyword + 1-hop 关系扩展 |

**诚实评价**: 
- 我们的 ESCA 是关键词匹配筛选，论文 ESCA/SGCLIP 用 CLIP 特征计算语义相关度
- 92.5% 减少率有效，但筛选质量取决于关键词是否准确
- 若关键词提取失败 → 可能误删目标物体

### 3.6 多源融合

| 测试 | 结果 | 说明 |
|------|------|------|
| 标签匹配 vs 检测器高分 | **标签优先 ✓** | fire extinguisher (det=0.7) 赢 red box (det=0.95) |
| 空间关系加分 | **主语加分 ✓** | chair (spatial=1.0) 赢 door (spatial=0.3) |
| 无 CLIP 权重重分配 | **0.775 ✓** | 不伪造数据，权重正确归一化 |

**诚实评价**: 
- 当前"多源"实际只有 **2-3 源** (标签 + 检测器 + 空间关系)
- 论文 AdaNav 的 4 源融合需要真实 CLIP 才完整
- 无 CLIP 时准确率提升无法量化 (无基线对比)

---

## 4. 与论文的差距分析

### 4.1 我们实现了什么

| 论文 | 我们实现的部分 | 实现程度 | 缺失的关键部分 |
|------|---------------|----------|---------------|
| **VLingNav (2026)** | Fast-Slow 双路径架构 | 框架 ✓，Fast Path 工作 | AdaCoT 动态触发；真实 CLIP 特征；Slow Path 未完整测试 (需 LLM API) |
| **OmniNav (ICLR 2026)** | Fast-Slow 概念 | 概念 ✓ | Fast 模块无 waypoint 生成；无 obstacle prediction；无 visual encoder |
| **ESCA (NeurIPS 2025)** | 场景图过滤 | 简化版 ✓ | 用关键词匹配代替 CLIP 特征筛选；无 semantic clustering |
| **AdaNav (ICLR 2026)** | 多源融合权重 | 框架 ✓ | 缺少真实 CLIP 源；无不确定性估计；无动态权重调整 |
| **SG-Nav (NeurIPS 2024)** | 3D 场景图 + 空间关系 | 数据结构 ✓ | 无层次场景图；无 region 聚类；LLM 推理未测试 |
| **ConceptGraphs (ICRA 2024)** | 实例追踪 + 场景图 | 基础版 ✓ | 无增量式 3D 重建；无 CLIP 特征存储 |
| **LOVON (2024)** | 动作原语定义 | 定义 ✓ | 未在真实机器人上执行；Laplacian 滤波未集成到闭环 |
| **MTU3D (ICCV 2025)** | frontier + grounding 融合 | 框架 ✓ | 无真实 costmap 测试；grounding potential 近似计算 |

### 4.2 我们无法做到的 (诚实声明)

1. **无真实数据集评测**: 未在 R2R, REVERIE, MP3D, HM3D 上测试。无法报告 SR (Success Rate), SPL (Success weighted by Path Length), NE (Navigation Error) 等标准指标。

2. **无端到端闭环**: 代码是模块级测试 (Unit Test)，未进行 ROS2 节点间通信测试，更未在真实机器人上运行。

3. **无 CLIP 推理**: 最核心的跨模态能力缺失。所有 "CLIP score" 要么是 0，要么是代码近似。

4. **Slow Path 基本未测试**: LLM 调用需要 API key，4 个测试被跳过。

5. **感知层测试全面失败**: CLIP Encoder 和 YOLO-World Detector 的测试有 27 个失败 (API 接口不匹配)，说明感知层代码和测试之间存在断层。

---

## 5. 实际技术水平定位

### 当前状态: **原型验证阶段 (TRL 3-4)**

| 层级 | 状态 | 说明 |
|------|------|------|
| 语言接口层 | ✅ 完成 | 支持中英文指令解析，jieba 分词集成 |
| 语义规划层 | ⚠️ 部分完成 | Fast Path 工作，Slow Path 框架就绪但未测试 |
| 语义感知层 | ❌ 待修复 | 代码存在但测试全面失败，接口不一致 |
| 几何导航层 | ✅ 已有 | 原有 A*/DWB 导航栈，与语义层解耦 |
| 硬件驱动层 | ✅ 已有 | Unitree + Orbbec 驱动已有 |

### 如果要达到论文级别，还需要:

1. **CLIP 模型加载** (~1周): 在 Jetson 上加载 CLIP ViT-B/32，修复 CLIPEncoder API
2. **YOLO-World 调通** (~1周): 修复 YOLOWorldDetector API 不一致，TensorRT 导出
3. **Slow Path LLM 集成** (~2天): 配置 OpenAI/Qwen API，测试端到端
4. **ROS2 节点集成** (~1周): planner_node ↔ perception_node ↔ nav_stack 实际通信
5. **真实场景测试** (~2周): 在实际环境中测试，收集数据
6. **标准数据集评测** (~1周): 使用 Habitat/MP3D 评测 SR/SPL

**预估从当前状态到论文级别: 6-8 周工程量**

---

## 6. 本次修复后的真实能力

### ✅ 确实能做到的:
- 简单指令 → 精确标签匹配 → 导航目标坐标 (Fast Path)
- 空间关系指令正确区分主语和修饰语
- 大场景图过滤到可控规模 (ESCA 简化版)
- 中英文混合指令分词
- 任务分解为子目标序列 (规则方式)
- 拓扑记忆维护已访问位置

### ❌ 还做不到的:
- 视觉-语言跨模态匹配 (需要 CLIP)
- 属性区分 ("red chair" vs "blue chair")
- 开放词汇检测 (需要 YOLO-World 调通)
- LLM 推理补足 (需要 API key)
- 真实机器人导航 (需要 ROS2 集成)
- 在标准 benchmark 上报告数字

---

## 7. 测试统计

```
semantic_planner:  108 passed, 0 failed, 4 skipped  (Slow Path 需 API key)
semantic_perception: 14 passed, 27 failed, 0 skipped (CLIP/YOLO API 不一致)
---------------------------------------------------------------
总计:              122 passed, 27 failed, 4 skipped
真实通过率:        79.7% (122/153)
核心逻辑通过率:    100% (108/108, 仅 planner)
```

---

**报告编写**: Claude (审阅模式)
**原则**: 所有数字来自实际测试运行，不伪造，不夸大
