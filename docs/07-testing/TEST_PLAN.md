# MapPilot 论文验证测试计划

## 一、当前测试状态

### ✅ 已完成测试
| 模块 | 测试内容 | 结果 |
|------|----------|------|
| Habitat Simulator | 场景加载、NavMesh生成 | ✅ 308 FPS (RTX 3090) |
| YOLO-World Detection | 21类物体检测 | ✅ 检测到22个物体 |
| Goal Resolver - Fast Path | 关键词匹配 | ✅ ~5ms 响应 |
| Goal Resolver - Slow Path | LLM语义推理 (Kimi-K2.5) | ✅ 100%准确率 |
| NavMesh Path Planning | A*最短路径 | ✅ 100%成功率 |
| 端到端导航 | 指令→目标→路径→执行 | ✅ 3/3 成功 |

### ❌ 未测试模块
| 模块 | 文件 | 重要性 |
|------|------|--------|
| Frontier Scorer (MTU3D) | frontier_scorer.py | ⭐⭐⭐ 探索策略核心 |
| Exploration Strategy | exploration_strategy.py | ⭐⭐⭐ 主动探索 |
| Topological Memory | topological_memory.py | ⭐⭐⭐ 空间记忆 |
| Instance Tracker (ConceptGraphs) | instance_tracker.py | ⭐⭐⭐ 场景图增量构建 |
| CLIP Encoder | clip_encoder.py | ⭐⭐ 语义特征 |
| Task Decomposer (SayCan) | task_decomposer.py | ⭐⭐ 复杂任务分解 |
| VLA Model (AdaCoT) | model/adacot.py | ⭐⭐⭐ 端到端学习 |
| VLingMem | model/vlingmem.py | ⭐⭐ 情景记忆 |
| Spot Robot Integration | - | ⭐⭐⭐ 真实机器人 |

---

## 二、论文验证测试计划

### 阶段 1: 核心模块单元测试 (1-2天)

#### 1.1 Frontier Scorer (MTU3D)
```
测试目标: 验证边界评分算法
指标:
  - Grounding Potential 计算正确性
  - 与 baseline (random, nearest) 对比
  - 探索效率 (覆盖率 vs 步数)
```

#### 1.2 Topological Memory
```
测试目标: 验证空间记忆构建
指标:
  - 节点添加/查询正确性
  - 回环检测准确率
  - 内存占用
```

#### 1.3 Instance Tracker (ConceptGraphs)
```
测试目标: 验证增量场景图构建
指标:
  - 物体去重准确率
  - 关系推断正确性
  - 实时性 (FPS)
```

#### 1.4 CLIP Encoder
```
测试目标: 验证语义特征提取
指标:
  - 特征维度正确性
  - 相似度计算准确性
  - 缓存命中率
```

### 阶段 2: 集成测试 (2-3天)

#### 2.1 主动探索测试
```
场景: HM3D 多个场景
任务: "Find the kitchen" (目标不在视野内)
流程:
  1. 初始位置随机
  2. 构建初始场景图
  3. Goal Resolver 返回 "explore"
  4. Frontier Scorer 选择探索方向
  5. 移动并更新场景图
  6. 重复直到找到目标
指标:
  - 探索步数
  - 成功率
  - 与 baseline 对比
```

#### 2.2 长程导航测试
```
场景: HM3D 大型场景
任务: 多房间导航
指标:
  - SPL (Success weighted by Path Length)
  - SR (Success Rate)
  - 导航时间
```

#### 2.3 复杂指令测试
```
指令类型:
  - 简单: "Go to the chair"
  - 属性: "Go to the red chair"
  - 关系: "Go to the chair near the table"
  - 推理: "Go to where people eat"
  - 多步: "First go to kitchen, then find the fridge"
指标:
  - Fast Path 命中率
  - Slow Path 准确率
  - 端到端成功率
```

### 阶段 3: Spot Robot 集成 (3-5天)

#### 3.1 Spot Robot 仿真
```
测试目标: 在 Habitat 中使用 Spot 机器人模型
内容:
  - 加载 Spot URDF
  - 腿部运动控制
  - 相机视角调整
  - 碰撞检测
```

#### 3.2 动作空间测试
```
动作类型:
  - 前进/后退
  - 左转/右转
  - 原地旋转
指标:
  - 动作执行准确性
  - 与 NavMesh 路径的跟踪误差
```

### 阶段 4: Benchmark 对比实验 (3-5天)

#### 4.1 数据集
```
- HM3D (Habitat-Matterport 3D)
- MP3D (Matterport3D)
- Gibson
```

#### 4.2 Baseline 方法
```
- Random exploration
- Frontier-based exploration
- Learning-based (VLN-CE, ETPNav)
```

#### 4.3 评估指标
```
导航性能:
  - SR (Success Rate): 成功率
  - SPL (Success weighted by Path Length): 路径效率
  - NE (Navigation Error): 终点误差
  - Oracle SR: 最优路径成功率

探索性能:
  - Coverage: 场景覆盖率
  - Exploration Efficiency: 覆盖率/步数
  - Object Discovery Rate: 物体发现率

语义理解:
  - Goal Resolution Accuracy: 目标解析准确率
  - Fast Path Hit Rate: 快速路径命中率
  - LLM Reasoning Accuracy: LLM推理准确率
```

### 阶段 5: 消融实验 (2-3天)

#### 5.1 Fast-Slow 双进程消融
```
对比:
  - Only Fast Path
  - Only Slow Path (LLM)
  - Fast-Slow (Ours)
```

#### 5.2 场景图消融
```
对比:
  - No scene graph (direct detection)
  - Static scene graph
  - Incremental scene graph (Ours)
```

#### 5.3 探索策略消融
```
对比:
  - Random
  - Nearest frontier
  - MTU3D frontier scoring (Ours)
```

---

## 三、测试执行顺序

### Week 1: 单元测试
- [ ] Day 1-2: Frontier Scorer + Exploration Strategy
- [ ] Day 3: Topological Memory
- [ ] Day 4: Instance Tracker
- [ ] Day 5: CLIP Encoder + Task Decomposer

### Week 2: 集成测试
- [ ] Day 1-2: 主动探索完整流程
- [ ] Day 3-4: 长程导航测试
- [ ] Day 5: 复杂指令测试

### Week 3: Robot + Benchmark
- [ ] Day 1-2: Spot Robot 集成
- [ ] Day 3-5: Benchmark 数据收集

### Week 4: 消融 + 论文
- [ ] Day 1-2: 消融实验
- [ ] Day 3-5: 结果整理 + 论文撰写

---

## 四、预期论文贡献点

### 主要贡献
1. **Fast-Slow Dual Process Goal Resolution**
   - Fast Path: 关键词+CLIP匹配 (<10ms)
   - Slow Path: LLM语义推理
   - 90%+ Fast Path命中率，显著降低延迟

2. **MTU3D-inspired Frontier Scoring**
   - Grounding Potential 指导探索
   - 比 random/nearest 更高效

3. **Incremental Scene Graph (ConceptGraphs)**
   - 实时物体跟踪与去重
   - 空间关系推断

### 实验验证
- HM3D/MP3D benchmark 对比
- 消融实验证明各模块有效性
- Spot Robot 仿真验证

---

## 五、下一步行动

### 立即执行 (今天)
1. 测试 Frontier Scorer
2. 测试 Topological Memory
3. 测试 Instance Tracker

### 本周完成
1. 所有单元测试
2. 主动探索集成测试
3. Spot Robot 基础集成

需要我现在开始执行哪个测试？
