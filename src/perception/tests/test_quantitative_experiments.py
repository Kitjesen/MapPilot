#!/usr/bin/env python3
"""
定量实验测试脚本

测试内容:
1. 统计分析器功能
2. 实验报告生成
3. 完整实验流程（模拟数据）
"""

import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use('Agg')

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from examples.run_quantitative_experiments import (
    ExperimentReportGenerator,
    StatisticalAnalyzer,
)

from perception.evaluation_framework import (
    BenchmarkResult,
    MemoryMetrics,
    PathMetrics,
    UpdateMetrics,
)


def test_statistical_analyzer():
    """测试统计分析器。"""
    print("=" * 60)
    print("测试 1: 统计分析器")
    print("=" * 60)

    # 创建模拟数据
    method1_values = [1.5, 1.6, 1.4, 1.7, 1.5]
    method2_values = [0.5, 0.6, 0.4, 0.7, 0.5]

    analyzer = StatisticalAnalyzer()
    stat = analyzer.compute_statistics(
        method1_values,
        method2_values,
        "内存占用 (MB)",
    )

    print("\n统计结果:")
    print(f"  方法 1 均值: {stat['method1_mean']:.2f}")
    print(f"  方法 2 均值: {stat['method2_mean']:.2f}")
    print(f"  t 统计量: {stat['t_statistic']:.2f}")
    print(f"  p 值: {stat['p_value']:.4f}")
    print(f"  Cohen's d: {stat['cohens_d']:.2f}")
    print(f"  相对差异: {stat['relative_diff_percent']:.1f}%")
    print(f"  显著性: {stat['significant']}")

    assert stat['method1_mean'] > stat['method2_mean'], "方法 1 应该大于方法 2"
    assert stat['significant'], "差异应该显著"

    print("\n✓ 统计分析器测试通过")
    print()


def test_statistics_table():
    """测试统计表格生成。"""
    print("=" * 60)
    print("测试 2: 统计表格生成")
    print("=" * 60)

    stats_list = [
        {
            'metric': '内存占用 (MB)',
            'method1_mean': 1.5,
            'method1_std': 0.1,
            'method2_mean': 0.5,
            'method2_std': 0.1,
            't_statistic': 10.0,
            'p_value': 0.001,
            'cohens_d': 2.0,
            'relative_diff_percent': 200.0,
            'significant': True,
        },
        {
            'metric': '更新时间 (ms)',
            'method1_mean': 30.0,
            'method1_std': 5.0,
            'method2_mean': 60.0,
            'method2_std': 10.0,
            't_statistic': -5.0,
            'p_value': 0.01,
            'cohens_d': -1.5,
            'relative_diff_percent': -50.0,
            'significant': True,
        },
    ]

    analyzer = StatisticalAnalyzer()
    table = analyzer.generate_statistics_table(stats_list)

    print("\n统计表格:")
    print(table)

    assert "内存占用" in table, "表格应包含内存占用"
    assert "更新时间" in table, "表格应包含更新时间"
    assert "✅" in table, "表格应包含显著性标记"

    print("\n✓ 统计表格生成测试通过")
    print()


def test_report_generator():
    """测试实验报告生成器。"""
    print("=" * 60)
    print("测试 3: 实验报告生成器")
    print("=" * 60)

    # 创建临时目录
    temp_dir = Path(tempfile.mkdtemp())

    try:
        # 创建模拟结果
        results = []
        for i in range(3):
            # PCT A* 结果
            result = BenchmarkResult(
                method_name="PCT A*",
                scene_id=f"scene_{i}",
                timestamp=0.0,
                memory=MemoryMetrics(
                    total_memory_mb=1.5 + np.random.rand() * 0.5,
                    map_memory_mb=1.0,
                    graph_memory_mb=0.3,
                    other_memory_mb=0.2,
                    peak_memory_mb=2.0,
                ),
                update=UpdateMetrics(
                    avg_update_time_ms=30.0 + np.random.rand() * 10,
                    max_update_time_ms=50.0,
                    min_update_time_ms=20.0,
                    update_frequency_hz=30.0,
                    total_updates=100,
                ),
                path=PathMetrics(
                    path_length=10.0 + np.random.rand() * 5,
                    path_smoothness=0.8 + np.random.rand() * 0.15,
                    path_clearance=0.5,
                    planning_time_ms=3.0 + np.random.rand() * 2,
                    success_rate=1.0,
                    num_waypoints=30,
                ),
            )
            results.append(result)

            # USS-Nav 结果
            result = BenchmarkResult(
                method_name="USS-Nav",
                scene_id=f"scene_{i}",
                timestamp=0.0,
                memory=MemoryMetrics(
                    total_memory_mb=0.5 + np.random.rand() * 0.3,
                    map_memory_mb=0.3,
                    graph_memory_mb=0.1,
                    other_memory_mb=0.1,
                    peak_memory_mb=0.8,
                ),
                update=UpdateMetrics(
                    avg_update_time_ms=60.0 + np.random.rand() * 15,
                    max_update_time_ms=80.0,
                    min_update_time_ms=50.0,
                    update_frequency_hz=15.0,
                    total_updates=100,
                ),
                path=PathMetrics(
                    path_length=9.0 + np.random.rand() * 4,
                    path_smoothness=0.85 + np.random.rand() * 0.1,
                    path_clearance=0.6,
                    planning_time_ms=0.15 + np.random.rand() * 0.1,
                    success_rate=1.0,
                    num_waypoints=25,
                ),
            )
            results.append(result)

        # 创建统计结果
        statistics = [
            {
                'metric': '内存占用 (MB)',
                'method1_mean': 1.5,
                'method1_std': 0.1,
                'method2_mean': 0.5,
                'method2_std': 0.1,
                't_statistic': 10.0,
                'p_value': 0.001,
                'cohens_d': 2.0,
                'relative_diff_percent': 200.0,
                'significant': True,
            },
        ]

        # 配置
        config = {
            'timestamp': '2026-02-23T12:00:00',
            'dataset_type': 'hm3d',
            'num_scenes': 3,
            'num_frames': 50,
            'planning_queries': 10,
            'methods': ['PCT A*', 'USS-Nav'],
        }

        # 生成报告
        report_gen = ExperimentReportGenerator(str(temp_dir))
        report_gen.generate_report(results, statistics, config)

        # 检查报告文件
        report_file = temp_dir / "experiment_report.md"
        assert report_file.exists(), "报告文件应该存在"

        # 读取报告内容
        with open(report_file, encoding='utf-8') as f:
            content = f.read()

        print("\n报告预览（前 500 字符）:")
        print(content[:500])

        assert "USS-Nav 定量实验报告" in content, "报告应包含标题"
        assert "实验配置" in content, "报告应包含配置"
        assert "统计分析" in content, "报告应包含统计分析"
        assert "PCT A*" in content, "报告应包含 PCT A*"
        assert "USS-Nav" in content, "报告应包含 USS-Nav"

        print(f"\n✓ 报告已生成: {report_file}")
        print("✓ 实验报告生成器测试通过")
        print()

    finally:
        # 清理临时文件
        import shutil
        shutil.rmtree(temp_dir)


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("定量实验测试套件")
    print("=" * 60 + "\n")

    try:
        test_statistical_analyzer()
        test_statistics_table()
        test_report_generator()

        print("=" * 60)
        print("✓ 所有测试通过!")
        print("=" * 60)
        return 0

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
