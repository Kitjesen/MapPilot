# 论文实验补充分析

**日期**: 2026-02-21
**目的**: 分析论文实验章节需要补充的内容

---

## 1. 当前已完成的实验

### 1.1 离线算法验证 ✅

| 实验 | 状态 | 结果 |
|------|------|------|
| Fast Path 目标解析 | ✅ 完成 | L1/L2 英文 100% 命中率, <5ms |
| Slow Path LLM 推理 | ✅ 完成 | 12/12 测试通过, Kimi-k2.5 |
| 任务分解 | ✅ 完成 | L1/L2 100%, L3 50% |
| BA-HSG 信念系统 | ✅ 完成 | 26 测试通过 |
| 多假设目标规划 | ✅ 完成 | 100% 成功率, 平均 1.6 次尝试 |
| VoI 调度分析 | ✅ 完成 | 77% continue, 23% reperceive |
| TSG 拓扑探索 | ✅ 完成 | 33 测试通过 |

### 1.2 Habitat 仿真验证 ✅

| 实验 | 状态 | 结果 |
|------|------|------|
| 场景加载 | ✅ 完成 | HM3D 场景, 188.28 m² |
| YOLO-World 检测 | ✅ 完成 | 18 类物体 |
| CLIP 编码 | ✅ 完成 | 512 维特征, CUDA |
| 导航路径规划 | ✅ 完成 | NavMesh A* |
| 轨迹可视化 | ✅ 完成 | 带背景俯视图 |
| 动作执行 | ✅ 完成 | sit/sleep/eat/watch |
| 视频录制 | ✅ 完成 | 3 个导航视频 |

---

## 2. 论文要求但尚未完成的实验 ❌

### 2.1 定量基准对比 (Critical)

**论文 §4.5 要求的 Baselines:**

| Baseline | 描述 | 状态 | 优先级 |
|----------|------|------|--------|
| Nav2-GT | 导航到真实坐标 (上界) | ❌ 未实现 | P0 |
| CLIP-Frontier | CoW 风格 CLIP 匹配 + 随机 frontier | ❌ 未实现 | P0 |
| Flat-SG | 扁平物体列表 + LLM (ESC 风格) | ❌ 未实现 | P0 |
| SG-Nav-Heur | SG-Nav 子图评分, 仅启发式 | ❌ 未实现 | P1 |

**需要实现:**
```python
# 1. Nav2-GT: 直接导航到 ground truth 坐标
# 2. CLIP-Frontier: 不用场景图, 只用 CLIP 匹配 frontier
# 3. Flat-SG: 不用层级结构, 扁平物体列表给 LLM
# 4. SG-Nav-Heur: 只用启发式评分, 不用 VoI
```

### 2.2 消融实验 (Critical)

**论文 §4.5 要求的 Ablations:**

| 配置 | 描述 | 状态 | 优先级 |
|------|------|------|--------|
| Full BA-NaviMind | 完整系统 | ✅ 基线 | - |
| w/o BeliefState | 确定性置信度 | ❌ 未实现 | P0 |
| w/o VoI | 固定 2m 重感知 | ❌ 未实现 | P0 |
| w/o MultiHypothesis | 单目标选择 | ❌ 未实现 | P1 |
| w/o SceneGraph | 仅 CLIP 匹配 | ❌ 未实现 | P0 |
| w/o Hierarchy | 扁平物体列表 | ❌ 未实现 | P0 |
| w/o RePerception | 无重感知 | ❌ 未实现 | P1 |

### 2.3 标准数据集评估 (Critical)

**论文需要在标准数据集上报告 SR/SPL:**

| 数据集 | 描述 | 状态 | 优先级 |
|--------|------|------|--------|
| HM3D ObjectNav | Habitat-Matterport 3D | ❌ 未评估 | P0 |
| MP3D ObjectNav | Matterport3D | ❌ 未评估 | P1 |
| R2R VLN | Room-to-Room | ❌ 未评估 | P2 |

**需要:**
- 下载 HM3D/MP3D 数据集
- 实现标准 ObjectNav 评估协议
- 报告 SR, SPL, SoftSPL 指标

### 2.4 真实机器人实验 (Important)

**论文 §4.8 计划的实验:**

| 阶段 | 内容 | 状态 | 优先级 |
|------|------|------|--------|
| Phase 1 | L1×20×3 + L2×15×3 + L3×10×3 试验 | ❌ 未完成 | P1 |
| Phase 2 | 消融实验 + baseline 对比 | ❌ 未完成 | P1 |
| Phase 3 | 动态场景测试 + Jetson 性能 | ❌ 未完成 | P2 |

### 2.5 边缘设备性能基准 (Important)

**论文 Table 4 需要的数据:**

| 模块 | 指标 | 目标 | 状态 |
|------|------|------|------|
| YOLO-World (L) | FPS | > 10 | ❌ 未测 |
| CLIP (ViT-B/32) | 延迟 | < 50ms | ❌ 未测 |
| 场景图构建 | 延迟 | < 100ms | ❌ 未测 |
| Fast Path | 延迟 | < 200ms | ✅ <5ms |
| Slow Path (LLM) | 延迟 | 记录 | ✅ ~26s |
| GPU 利用率 | 峰值 | - | ❌ 未测 |
| 内存 | 峰值 | < 12GB | ❌ 未测 |

---

## 3. 实验补充优先级

### P0 - 必须完成 (论文核心)

1. **HM3D ObjectNav 标准评估**
   - 实现 Habitat Challenge 评估协议
   - 报告 SR, SPL 在 val/test split
   - 与 SG-Nav, ESC 等方法对比

2. **Baseline 对比实验**
   - 实现 CLIP-Frontier baseline
   - 实现 Flat-SG baseline
   - 在相同场景对比 SR/SPL

3. **消融实验**
   - w/o SceneGraph
   - w/o Hierarchy
   - w/o BeliefState
   - w/o VoI

### P1 - 重要 (增强说服力)

4. **真实机器人演示**
   - 至少 10 个 L1 指令
   - 录制视频证据
   - 报告成功率

5. **Jetson 性能基准**
   - 各模块延迟
   - GPU/内存使用

### P2 - 可选 (锦上添花)

6. **MP3D 数据集评估**
7. **动态场景测试**
8. **多楼层扩展**

---

## 4. 具体实验实现计划

### 4.1 HM3D ObjectNav 评估 (2-3 天)

```python
# 需要实现:
# 1. 下载 HM3D ObjectNav 数据集
# 2. 实现 Habitat Challenge 评估器
# 3. 运行 val split (1000+ episodes)
# 4. 计算 SR, SPL, SoftSPL

# 预期结果:
# - SR: 40-60% (与 SG-Nav 相当)
# - SPL: 30-50%
```

### 4.2 Baseline 实现 (1-2 天)

```python
# CLIP-Frontier Baseline:
class CLIPFrontierBaseline:
    def resolve_goal(self, instruction, frontiers):
        # 1. CLIP 编码指令
        # 2. 对每个 frontier 方向的图像编码
        # 3. 选择最高相似度的 frontier
        # 4. 导航到该 frontier
        pass

# Flat-SG Baseline:
class FlatSGBaseline:
    def resolve_goal(self, instruction, objects):
        # 1. 将所有物体扁平列表给 LLM
        # 2. 不使用 Room/Group 层级
        # 3. LLM 直接选择目标
        pass
```

### 4.3 消融实验 (1 天)

```python
# 配置文件:
ablation_configs = {
    "full": {"scene_graph": True, "hierarchy": True, "belief": True, "voi": True},
    "w/o_sg": {"scene_graph": False, ...},
    "w/o_hier": {"hierarchy": False, ...},
    "w/o_belief": {"belief": False, ...},
    "w/o_voi": {"voi": False, ...},
}

# 每个配置运行 100 episodes, 报告 SR/SPL
```

### 4.4 Jetson 性能测试 (0.5 天)

```python
# 需要在 Jetson Orin NX 上运行:
# 1. YOLO-World FPS 测试
# 2. CLIP 延迟测试
# 3. 场景图构建延迟
# 4. GPU/内存监控

# 使用 tegrastats 监控
```

---

## 5. 预期论文实验结果

### Table 3: Main Results (HM3D ObjectNav)

| Method | SR ↑ | SPL ↑ | SoftSPL ↑ |
|--------|------|-------|-----------|
| CLIP-Frontier | ~25% | ~15% | ~20% |
| Flat-SG | ~35% | ~25% | ~30% |
| SG-Nav (reported) | 53.9% | - | - |
| **NaviMind (Ours)** | **~50%** | **~35%** | **~40%** |

### Table 4: Ablation Study

| Config | SR ↑ | Δ SR |
|--------|------|------|
| Full NaviMind | 50% | - |
| w/o SceneGraph | 25% | -25% |
| w/o Hierarchy | 40% | -10% |
| w/o BeliefState | 45% | -5% |
| w/o VoI | 47% | -3% |

### Table 5: Jetson Performance

| Module | Latency | FPS | Memory |
|--------|---------|-----|--------|
| YOLO-World | 80ms | 12.5 | 2.1GB |
| CLIP | 35ms | 28.6 | 0.8GB |
| Scene Graph | 50ms | 20 | 0.3GB |
| Fast Path | 5ms | 200 | 0.1GB |
| **Total** | **170ms** | **5.9** | **3.3GB** |

---

## 6. 时间估算

| 任务 | 时间 | 依赖 |
|------|------|------|
| HM3D 数据集下载 | 0.5 天 | 网络 |
| 评估器实现 | 1 天 | - |
| HM3D 评估运行 | 1 天 | GPU |
| Baseline 实现 | 1 天 | - |
| 消融实验 | 1 天 | - |
| Jetson 测试 | 0.5 天 | 硬件 |
| 结果整理 | 0.5 天 | - |
| **总计** | **5.5 天** | |

---

## 7. 结论

### 必须补充的实验:

1. **HM3D ObjectNav 标准评估** - 论文核心数据
2. **Baseline 对比** - 证明方法有效性
3. **消融实验** - 证明各组件贡献

### 可选但推荐:

4. **真实机器人演示** - 增强说服力
5. **Jetson 性能** - 边缘部署价值

### 当前差距:

- 离线算法验证: ✅ 完成
- 仿真功能测试: ✅ 完成
- **标准数据集评估: ❌ 缺失** (Critical)
- **Baseline 对比: ❌ 缺失** (Critical)
- **消融实验: ❌ 缺失** (Critical)
- 真实机器人: ❌ 缺失 (Important)
