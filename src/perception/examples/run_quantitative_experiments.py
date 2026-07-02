#!/usr/bin/env python3
"""
定量实验脚本 (Quantitative Experiments)

在 HM3D 数据集上运行完整的定量实验，对比 PCT A* 和 USS-Nav 的性能。

实验内容:
1. 多场景评估（10+ 场景）
2. 内存占用对比
3. 更新速率对比
4. 路径质量对比
5. 统计分析和显著性检验
6. 自动生成实验报告

用法:
    python run_quantitative_experiments.py --dataset-root /path/to/hm3d --num-scenes 10
"""

import argparse
import json
import logging
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.research.end_to_end_evaluation import EndToEndEvaluator
from perception.research.evaluation_framework import BenchmarkResult
from perception.research.visualization_tools import (
    PerformanceVisualizer,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  统计分析
# ══════════════════════════════════════════════════════════════════

class StatisticalAnalyzer:
    """统计分析器 — 显著性检验和效应量计算。"""

    @staticmethod
    def compute_statistics(
        method1_values: list[float],
        method2_values: list[float],
        metric_name: str,
    ) -> dict:
        """
        计算两组数据的统计量。

        Args:
            method1_values: 方法 1 的数据
            method2_values: 方法 2 的数据
            metric_name: 指标名称

        Returns:
            统计结果字典
        """
        # 基本统计量
        mean1 = np.mean(method1_values)
        mean2 = np.mean(method2_values)
        std1 = np.std(method1_values, ddof=1)
        std2 = np.std(method2_values, ddof=1)

        # t 检验（独立样本）
        t_stat, p_value = stats.ttest_ind(method1_values, method2_values)

        # Cohen's d 效应量
        pooled_std = np.sqrt((std1**2 + std2**2) / 2)
        cohens_d = (mean1 - mean2) / pooled_std if pooled_std > 0 else 0

        # 相对差异
        relative_diff = (mean1 - mean2) / mean2 * 100 if mean2 != 0 else 0

        return {
            'metric': metric_name,
            'method1_mean': mean1,
            'method1_std': std1,
            'method2_mean': mean2,
            'method2_std': std2,
            't_statistic': t_stat,
            'p_value': p_value,
            'cohens_d': cohens_d,
            'relative_diff_percent': relative_diff,
            'significant': p_value < 0.05,
        }

    @staticmethod
    def generate_statistics_table(stats_list: list[dict]) -> str:
        """生成统计表格（Markdown 格式）。"""
        lines = []
        lines.append("| 指标 | 方法 1 | 方法 2 | t 统计量 | p 值 | Cohen's d | 相对差异 | 显著性 |")
        lines.append("|------|--------|--------|----------|------|-----------|----------|--------|")

        for stat in stats_list:
            sig_mark = "✅" if stat['significant'] else "❌"
            lines.append(
                f"| {stat['metric']} "
                f"| {stat['method1_mean']:.2f} ± {stat['method1_std']:.2f} "
                f"| {stat['method2_mean']:.2f} ± {stat['method2_std']:.2f} "
                f"| {stat['t_statistic']:.2f} "
                f"| {stat['p_value']:.4f} "
                f"| {stat['cohens_d']:.2f} "
                f"| {stat['relative_diff_percent']:.1f}% "
                f"| {sig_mark} |"
            )

        return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════
#  实验报告生成器
# ══════════════════════════════════════════════════════════════════

class ExperimentReportGenerator:
    """实验报告生成器 — 生成完整的实验报告。"""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def generate_report(
        self,
        results: list[BenchmarkResult],
        statistics: list[dict],
        config: dict,
    ):
        """
        生成完整的实验报告。

        Args:
            results: 评估结果列表
            statistics: 统计分析结果
            config: 实验配置
        """
        report_file = self.output_dir / "experiment_report.md"

        with open(report_file, 'w', encoding='utf-8') as f:
            # 标题
            f.write("# USS-Nav 定量实验报告\n\n")
            f.write(f"**生成时间**: {config.get('timestamp', 'N/A')}\n\n")
            f.write("---\n\n")

            # 实验配置
            f.write("## 实验配置\n\n")
            f.write(f"- **数据集**: {config.get('dataset_type', 'N/A')}\n")
            f.write(f"- **场景数量**: {config.get('num_scenes', 'N/A')}\n")
            f.write(f"- **每场景帧数**: {config.get('num_frames', 'N/A')}\n")
            f.write(f"- **规划查询数**: {config.get('planning_queries', 'N/A')}\n")
            f.write(f"- **对比方法**: {', '.join(config.get('methods', []))}\n\n")

            # 结果概览
            f.write("## 结果概览\n\n")
            f.write(f"- **总评估数**: {len(results)}\n")

            # 按方法分组
            methods = {}
            for result in results:
                if result.method_name not in methods:
                    methods[result.method_name] = []
                methods[result.method_name].append(result)

            for method_name, method_results in methods.items():
                f.write(f"- **{method_name}**: {len(method_results)} 次评估\n")

            f.write("\n---\n\n")

            # 统计分析
            f.write("## 统计分析\n\n")
            f.write(StatisticalAnalyzer.generate_statistics_table(statistics))
            f.write("\n\n")

            # 显著性解释
            f.write("### 显著性解释\n\n")
            f.write("- **p < 0.05**: 差异具有统计显著性 ✅\n")
            f.write("- **p ≥ 0.05**: 差异不具有统计显著性 ❌\n")
            f.write("- **Cohen's d**:\n")
            f.write("  - |d| < 0.2: 小效应\n")
            f.write("  - 0.2 ≤ |d| < 0.8: 中等效应\n")
            f.write("  - |d| ≥ 0.8: 大效应\n\n")

            f.write("---\n\n")

            # 详细结果
            f.write("## 详细结果\n\n")

            for method_name, method_results in methods.items():
                f.write(f"### {method_name}\n\n")

                # 内存统计
                memory_vals = [r.memory.total_memory_mb for r in method_results if r.memory]
                if memory_vals:
                    f.write("**内存占用**:\n")
                    f.write(f"- 平均: {np.mean(memory_vals):.2f} MB\n")
                    f.write(f"- 标准差: {np.std(memory_vals):.2f} MB\n")
                    f.write(f"- 最小: {np.min(memory_vals):.2f} MB\n")
                    f.write(f"- 最大: {np.max(memory_vals):.2f} MB\n\n")

                # 更新时间统计
                update_vals = [r.update.avg_update_time_ms for r in method_results if r.update]
                if update_vals:
                    f.write("**更新时间**:\n")
                    f.write(f"- 平均: {np.mean(update_vals):.2f} ms\n")
                    f.write(f"- 标准差: {np.std(update_vals):.2f} ms\n")
                    f.write(f"- 最小: {np.min(update_vals):.2f} ms\n")
                    f.write(f"- 最大: {np.max(update_vals):.2f} ms\n\n")

                # 路径质量统计
                path_vals = [r.path.planning_time_ms for r in method_results if r.path]
                if path_vals:
                    f.write("**规划时间**:\n")
                    f.write(f"- 平均: {np.mean(path_vals):.2f} ms\n")
                    f.write(f"- 标准差: {np.std(path_vals):.2f} ms\n")
                    f.write(f"- 最小: {np.min(path_vals):.2f} ms\n")
                    f.write(f"- 最大: {np.max(path_vals):.2f} ms\n\n")

                success_vals = [r.path.success_rate for r in method_results if r.path]
                if success_vals:
                    f.write(f"**成功率**: {np.mean(success_vals):.2%}\n\n")

            f.write("---\n\n")

            # 可视化
            f.write("## 可视化\n\n")
            f.write("详细的性能对比图表请参见:\n")
            f.write("- `performance_comparison.png`: 性能对比柱状图\n")
            f.write("- `memory_boxplot.png`: 内存占用箱线图\n")
            f.write("- `update_time_boxplot.png`: 更新时间箱线图\n\n")

            f.write("---\n\n")

            # 结论
            f.write("## 结论\n\n")
            self._write_conclusions(f, statistics)

        logger.info(f"实验报告已生成: {report_file}")

    def _write_conclusions(self, f, statistics: list[dict]):
        """写入结论部分。"""
        f.write("基于统计分析结果:\n\n")

        for stat in statistics:
            if stat['significant']:
                direction = "更高" if stat['relative_diff_percent'] > 0 else "更低"
                f.write(
                    f"- **{stat['metric']}**: 方法 1 比方法 2 {direction} "
                    f"{abs(stat['relative_diff_percent']):.1f}%，"
                    f"差异具有统计显著性 (p = {stat['p_value']:.4f})\n"
                )
            else:
                f.write(
                    f"- **{stat['metric']}**: 两种方法无显著差异 "
                    f"(p = {stat['p_value']:.4f})\n"
                )

        f.write("\n")


# ══════════════════════════════════════════════════════════════════
#  主实验流程
# ══════════════════════════════════════════════════════════════════

def run_quantitative_experiments(
    dataset_root: str,
    dataset_type: str = "hm3d",
    num_scenes: int = 10,
    num_frames: int = 50,
    planning_queries: int = 10,
    methods: list[str] | None = None,
    output_dir: str = "experiment_results",
):
    """
    运行定量实验。

    Args:
        dataset_root: 数据集根目录
        dataset_type: 数据集类型
        num_scenes: 场景数量
        num_frames: 每场景帧数
        planning_queries: 规划查询数
        methods: 对比方法列表
        output_dir: 输出目录
    """
    if methods is None:
        methods = ["pct_astar", "uss_nav"]

    logger.info("=" * 60)
    logger.info("开始定量实验")
    logger.info("=" * 60)

    # 创建输出目录
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # 创建评估器
    evaluator = EndToEndEvaluator()

    # 获取场景列表
    from perception.research.dataset_loader import HM3DDatasetLoader
    loader = HM3DDatasetLoader(dataset_root)
    available_scenes = loader.list_scenes()[:num_scenes]

    logger.info(f"选择 {len(available_scenes)} 个场景进行评估")

    # 运行评估
    logger.info("开始评估...")
    results = evaluator.run_evaluation(
        dataset_type=dataset_type,
        dataset_root=dataset_root,
        scene_ids=available_scenes,
        methods=methods,
        num_frames=num_frames,
        planning_queries=planning_queries,
    )

    logger.info(f"评估完成，共 {len(results)} 个结果")

    # 保存原始结果
    results_file = output_path / "raw_results.json"
    evaluator.save_results(str(results_file))
    logger.info(f"原始结果已保存: {results_file}")

    # 统计分析
    logger.info("开始统计分析...")
    analyzer = StatisticalAnalyzer()
    statistics = []

    # 按方法分组
    methods_data = {}
    for result in results:
        if result.method_name not in methods_data:
            methods_data[result.method_name] = []
        methods_data[result.method_name].append(result)

    # 对比分析（假设有两种方法）
    if len(methods_data) == 2:
        method_names = list(methods_data.keys())
        method1_results = methods_data[method_names[0]]
        method2_results = methods_data[method_names[1]]

        # 内存对比
        memory1 = [r.memory.total_memory_mb for r in method1_results if r.memory]
        memory2 = [r.memory.total_memory_mb for r in method2_results if r.memory]
        if memory1 and memory2:
            stat = analyzer.compute_statistics(memory1, memory2, "内存占用 (MB)")
            statistics.append(stat)

        # 更新时间对比
        update1 = [r.update.avg_update_time_ms for r in method1_results if r.update]
        update2 = [r.update.avg_update_time_ms for r in method2_results if r.update]
        if update1 and update2:
            stat = analyzer.compute_statistics(update1, update2, "更新时间 (ms)")
            statistics.append(stat)

        # 规划时间对比
        plan1 = [r.path.planning_time_ms for r in method1_results if r.path]
        plan2 = [r.path.planning_time_ms for r in method2_results if r.path]
        if plan1 and plan2:
            stat = analyzer.compute_statistics(plan1, plan2, "规划时间 (ms)")
            statistics.append(stat)

    # 保存统计结果
    stats_file = output_path / "statistics.json"
    with open(stats_file, 'w', encoding='utf-8') as f:
        json.dump(statistics, f, indent=2, ensure_ascii=False)
    logger.info(f"统计结果已保存: {stats_file}")

    # 生成可视化
    logger.info("生成可视化...")
    visualizer = PerformanceVisualizer()
    fig = visualizer.plot_comparison(results)
    visualizer.save(fig, str(output_path / "performance_comparison.png"))
    plt.close(fig)

    # 生成实验报告
    logger.info("生成实验报告...")
    import datetime
    config = {
        'timestamp': datetime.datetime.now().isoformat(),
        'dataset_type': dataset_type,
        'num_scenes': len(available_scenes),
        'num_frames': num_frames,
        'planning_queries': planning_queries,
        'methods': methods,
    }

    report_gen = ExperimentReportGenerator(output_dir)
    report_gen.generate_report(results, statistics, config)

    logger.info("=" * 60)
    logger.info("实验完成!")
    logger.info(f"结果保存在: {output_path}")
    logger.info("=" * 60)


def main():
    """主函数。"""
    parser = argparse.ArgumentParser(description="USS-Nav 定量实验")
    parser.add_argument(
        "--dataset-root",
        type=str,
        required=True,
        help="数据集根目录",
    )
    parser.add_argument(
        "--dataset-type",
        type=str,
        default="hm3d",
        choices=["hm3d", "gibson"],
        help="数据集类型",
    )
    parser.add_argument(
        "--num-scenes",
        type=int,
        default=10,
        help="场景数量",
    )
    parser.add_argument(
        "--num-frames",
        type=int,
        default=50,
        help="每场景帧数",
    )
    parser.add_argument(
        "--planning-queries",
        type=int,
        default=10,
        help="规划查询数",
    )
    parser.add_argument(
        "--methods",
        type=str,
        nargs="+",
        default=["pct_astar"],
        help="对比方法列表",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="experiment_results",
        help="输出目录",
    )

    args = parser.parse_args()

    run_quantitative_experiments(
        dataset_root=args.dataset_root,
        dataset_type=args.dataset_type,
        num_scenes=args.num_scenes,
        num_frames=args.num_frames,
        planning_queries=args.planning_queries,
        methods=args.methods,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
