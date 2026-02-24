# 定量实验使用指南

本指南介绍如何使用定量实验脚本在 HM3D 数据集上运行完整的性能评估。

---

## 📋 功能概览

定量实验脚本提供以下功能:

1. **多场景批量评估**: 在 10+ 个场景上自动运行评估
2. **性能对比**: PCT A* vs USS-Nav 全面对比
3. **统计分析**: t 检验、Cohen's d 效应量、显著性检验
4. **自动报告**: 生成完整的 Markdown 实验报告
5. **可视化**: 性能对比图表、箱线图

---

## 🚀 快速开始

### 1. 准备数据集

确保 HM3D 数据集已下载并解压:

```bash
# 数据集目录结构
/path/to/hm3d/
├── train/
│   ├── 00800-TEEsavR23oF/
│   │   ├── rgb/
│   │   ├── depth/
│   │   ├── poses/
│   │   └── metadata.json
│   ├── 00801-HaxA7YrQdEC/
│   └── ...
└── val/
```

### 2. 运行实验

```bash
cd src/semantic_perception

# 基本用法（10 个场景，每场景 50 帧）
python examples/run_quantitative_experiments.py \
    --dataset-root /path/to/hm3d \
    --num-scenes 10 \
    --num-frames 50 \
    --planning-queries 10 \
    --methods pct_astar uss_nav \
    --output-dir experiment_results

# 快速测试（3 个场景，每场景 10 帧）
python examples/run_quantitative_experiments.py \
    --dataset-root /path/to/hm3d \
    --num-scenes 3 \
    --num-frames 10 \
    --planning-queries 5 \
    --methods pct_astar \
    --output-dir quick_test

# 完整实验（20 个场景，每场景 100 帧）
python examples/run_quantitative_experiments.py \
    --dataset-root /path/to/hm3d \
    --num-scenes 20 \
    --num-frames 100 \
    --planning-queries 20 \
    --methods pct_astar uss_nav \
    --output-dir full_experiment
```

### 3. 查看结果

实验完成后，输出目录包含:

```
experiment_results/
├── experiment_report.md          # 完整实验报告
├── raw_results.json              # 原始评估结果
├── statistics.json               # 统计分析结果
├── performance_comparison.png    # 性能对比图
├── memory_boxplot.png            # 内存箱线图（如果生成）
└── update_time_boxplot.png       # 更新时间箱线图（如果生成）
```

---

## 📊 输出说明

### 1. 实验报告 (experiment_report.md)

完整的 Markdown 格式报告，包含:

- **实验配置**: 数据集、场景数、帧数、方法
- **结果概览**: 总评估数、各方法评估数
- **统计分析**: t 检验、p 值、Cohen's d、显著性
- **详细结果**: 各方法的内存、更新时间、规划时间统计
- **可视化**: 图表文件列表
- **结论**: 基于统计分析的结论

示例:

```markdown
# USS-Nav 定量实验报告

## 统计分析

| 指标 | 方法 1 | 方法 2 | t 统计量 | p 值 | Cohen's d | 相对差异 | 显著性 |
|------|--------|--------|----------|------|-----------|----------|--------|
| 内存占用 (MB) | 1.53 ± 0.12 | 0.52 ± 0.08 | 15.23 | 0.0001 | 2.45 | 194.2% | ✅ |
| 更新时间 (ms) | 28.60 ± 5.32 | 60.43 ± 8.21 | -8.45 | 0.0003 | -1.87 | -52.7% | ✅ |
| 规划时间 (ms) | 3.57 ± 1.12 | 0.17 ± 0.05 | 7.89 | 0.0005 | 1.92 | 2000.0% | ✅ |
```

### 2. 原始结果 (raw_results.json)

JSON 格式的完整评估结果，包含每次评估的详细数据:

```json
[
  {
    "method_name": "PCT A*",
    "scene_id": "00800-TEEsavR23oF",
    "timestamp": 1708675200.0,
    "memory": {
      "total_memory_mb": 1.53,
      "map_memory_mb": 1.0,
      "graph_memory_mb": 0.3,
      "other_memory_mb": 0.23,
      "peak_memory_mb": 2.0
    },
    "update": {
      "avg_update_time_ms": 28.60,
      "max_update_time_ms": 45.2,
      "min_update_time_ms": 18.3,
      "update_frequency_hz": 30.0,
      "total_updates": 100
    },
    "path": {
      "path_length": 12.5,
      "path_smoothness": 0.85,
      "path_clearance": 0.5,
      "planning_time_ms": 3.57,
      "success_rate": 1.0,
      "num_waypoints": 32
    }
  }
]
```

### 3. 统计结果 (statistics.json)

统计分析的详细数据:

```json
[
  {
    "metric": "内存占用 (MB)",
    "method1_mean": 1.53,
    "method1_std": 0.12,
    "method2_mean": 0.52,
    "method2_std": 0.08,
    "t_statistic": 15.23,
    "p_value": 0.0001,
    "cohens_d": 2.45,
    "relative_diff_percent": 194.2,
    "significant": true
  }
]
```

---

## 🔬 统计分析说明

### t 检验

独立样本 t 检验用于比较两组数据的均值是否有显著差异。

- **零假设 (H0)**: 两组均值相等
- **备择假设 (H1)**: 两组均值不相等
- **显著性水平**: α = 0.05

### p 值解释

- **p < 0.05**: 拒绝零假设，差异具有统计显著性 ✅
- **p ≥ 0.05**: 不能拒绝零假设，差异不显著 ❌

### Cohen's d 效应量

衡量两组数据差异的大小:

- **|d| < 0.2**: 小效应
- **0.2 ≤ |d| < 0.8**: 中等效应
- **|d| ≥ 0.8**: 大效应

### 相对差异

```
相对差异 = (方法1均值 - 方法2均值) / 方法2均值 × 100%
```

- 正值: 方法 1 更高
- 负值: 方法 1 更低

---

## ⚙️ 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--dataset-root` | 数据集根目录 | 必需 |
| `--dataset-type` | 数据集类型 (hm3d/gibson) | hm3d |
| `--num-scenes` | 场景数量 | 10 |
| `--num-frames` | 每场景帧数 | 50 |
| `--planning-queries` | 规划查询数 | 10 |
| `--methods` | 对比方法列表 | pct_astar |
| `--output-dir` | 输出目录 | experiment_results |

---

## 📈 实验建议

### 快速测试 (5-10 分钟)

```bash
--num-scenes 3 --num-frames 10 --planning-queries 5
```

适用于:
- 验证脚本是否正常工作
- 快速检查数据集是否正确
- 调试实验流程

### 标准实验 (1-2 小时)

```bash
--num-scenes 10 --num-frames 50 --planning-queries 10
```

适用于:
- 论文实验
- 性能基准测试
- 方法对比

### 完整实验 (4-6 小时)

```bash
--num-scenes 20 --num-frames 100 --planning-queries 20
```

适用于:
- 最终论文结果
- 详细性能分析
- 鲁棒性验证

---

## 🐛 故障排除

### 问题 1: 数据集未找到

```
错误: Scene not found: 00800-TEEsavR23oF
```

**解决方案**:
- 检查数据集路径是否正确
- 确认场景目录存在
- 验证 metadata.json 文件存在

### 问题 2: 内存不足

```
错误: MemoryError
```

**解决方案**:
- 减少 `--num-frames`
- 减少 `--num-scenes`
- 使用更强大的机器

### 问题 3: 评估时间过长

**解决方案**:
- 先运行快速测试验证流程
- 使用 `--methods pct_astar` 只测试一种方法
- 减少场景数和帧数

---

## 📝 示例工作流

### 1. 验证环境

```bash
# 测试数据集加载
python -c "
from semantic_perception.dataset_loader import HM3DDatasetLoader
loader = HM3DDatasetLoader('/path/to/hm3d')
print(f'可用场景: {len(loader.list_scenes())}')
"
```

### 2. 快速测试

```bash
python examples/run_quantitative_experiments.py \
    --dataset-root /path/to/hm3d \
    --num-scenes 3 \
    --num-frames 10 \
    --planning-queries 5 \
    --methods pct_astar \
    --output-dir quick_test
```

### 3. 查看结果

```bash
cat quick_test/experiment_report.md
```

### 4. 运行完整实验

```bash
python examples/run_quantitative_experiments.py \
    --dataset-root /path/to/hm3d \
    --num-scenes 10 \
    --num-frames 50 \
    --planning-queries 10 \
    --methods pct_astar uss_nav \
    --output-dir full_experiment
```

### 5. 分析结果

```bash
# 查看报告
cat full_experiment/experiment_report.md

# 查看统计结果
cat full_experiment/statistics.json | python -m json.tool

# 查看可视化
open full_experiment/performance_comparison.png
```

---

## 🎯 下一步

完成定量实验后，可以:

1. **性能优化**: 根据实验结果优化瓶颈
2. **论文撰写**: 使用实验报告和图表
3. **真实验证**: 在真实机器人上验证

---

**相关文档**:
- `visualization_tools_summary.md` - 可视化工具文档
- `project_summary.md` - 项目总体进度
- `CHANGELOG.md` - 版本变更记录
