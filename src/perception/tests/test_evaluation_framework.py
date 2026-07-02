#!/usr/bin/env python3
"""
评估框架测试脚本

测试内容:
1. 内存评估器
2. 更新速率评估器
3. 路径质量评估器
4. 探索效率评估器
5. 基准测试框架
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.research.evaluation_framework import (
    BenchmarkFramework,
    ExplorationEvaluator,
    ExplorationMetrics,
    MemoryEvaluator,
    MemoryMetrics,
    PathEvaluator,
    PathMetrics,
    UpdateEvaluator,
    UpdateMetrics,
)


def test_memory_evaluator():
    """测试内存评估器。"""
    print("=" * 60)
    print("测试 1: 内存评估器")
    print("=" * 60)

    evaluator = MemoryEvaluator()
    evaluator.start()

    # 分配一些内存
    [np.random.rand(1000, 1000) for _ in range(10)]

    metrics = evaluator.stop()

    print(f"总内存: {metrics.total_memory_mb:.2f} MB")
    print(f"峰值内存: {metrics.peak_memory_mb:.2f} MB")

    assert metrics.total_memory_mb >= 0

    print("\n✓ 内存评估器测试通过")
    print()


def test_update_evaluator():
    """测试更新速率评估器。"""
    print("=" * 60)
    print("测试 2: 更新速率评估器")
    print("=" * 60)

    evaluator = UpdateEvaluator()

    # 模拟 10 次更新
    for _i in range(10):
        evaluator.start_update()
        time.sleep(0.01)  # 模拟更新耗时
        evaluator.end_update()

    metrics = evaluator.get_metrics()

    print(f"平均更新时间: {metrics.avg_update_time_ms:.2f} ms")
    print(f"最大更新时间: {metrics.max_update_time_ms:.2f} ms")
    print(f"最小更新时间: {metrics.min_update_time_ms:.2f} ms")
    print(f"更新频率: {metrics.update_frequency_hz:.2f} Hz")
    print(f"总更新次数: {metrics.total_updates}")

    assert metrics.total_updates == 10
    assert metrics.avg_update_time_ms > 0

    print("\n✓ 更新速率评估器测试通过")
    print()


def test_path_evaluator():
    """测试路径质量评估器。"""
    print("=" * 60)
    print("测试 3: 路径质量评估器")
    print("=" * 60)

    # 创建测试路径
    path = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [2, 0, 0],
        [3, 1, 0],
        [4, 2, 0],
    ], dtype=np.float32)

    metrics = PathEvaluator.evaluate_path(path, planning_time=0.001)

    print(f"路径长度: {metrics.path_length:.2f} m")
    print(f"路径平滑度: {metrics.path_smoothness:.2f}")
    print(f"路径间隙: {metrics.path_clearance:.2f} m")
    print(f"规划时间: {metrics.planning_time_ms:.2f} ms")
    print(f"成功率: {metrics.success_rate:.2f}")
    print(f"路径点数: {metrics.num_waypoints}")

    assert metrics.path_length > 0
    assert 0 <= metrics.path_smoothness <= 1
    assert metrics.num_waypoints == 5

    print("\n✓ 路径质量评估器测试通过")
    print()


def test_path_smoothness():
    """测试路径平滑度计算。"""
    print("=" * 60)
    print("测试 4: 路径平滑度")
    print("=" * 60)

    # 直线路径（完全平滑）
    straight_path = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [2, 0, 0],
        [3, 0, 0],
    ], dtype=np.float32)

    metrics1 = PathEvaluator.evaluate_path(straight_path)
    print(f"直线路径平滑度: {metrics1.path_smoothness:.2f}")

    # 曲折路径（不平滑）
    zigzag_path = np.array([
        [0, 0, 0],
        [1, 1, 0],
        [2, 0, 0],
        [3, 1, 0],
    ], dtype=np.float32)

    metrics2 = PathEvaluator.evaluate_path(zigzag_path)
    print(f"曲折路径平滑度: {metrics2.path_smoothness:.2f}")

    # 直线应该比曲折更平滑
    assert metrics1.path_smoothness > metrics2.path_smoothness

    print("\n✓ 路径平滑度测试通过")
    print()


def test_exploration_evaluator():
    """测试探索效率评估器。"""
    print("=" * 60)
    print("测试 5: 探索效率评估器")
    print("=" * 60)

    evaluator = ExplorationEvaluator()
    evaluator.start()

    # 模拟机器人移动
    positions = [
        np.array([0, 0, 0]),
        np.array([1, 0, 0]),
        np.array([2, 1, 0]),
        np.array([3, 2, 0]),
    ]

    for pos in positions:
        evaluator.update_position(pos)
        time.sleep(0.01)

    metrics = evaluator.stop(coverage_ratio=0.75, num_frontiers=10)

    print(f"覆盖率: {metrics.coverage_ratio:.2%}")
    print(f"探索时间: {metrics.exploration_time_s:.2f} s")
    print(f"行驶距离: {metrics.travel_distance_m:.2f} m")
    print(f"前沿数量: {metrics.num_frontiers}")
    print(f"效率: {metrics.efficiency:.4f}")

    assert metrics.coverage_ratio == 0.75
    assert metrics.travel_distance_m > 0
    assert metrics.num_frontiers == 10

    print("\n✓ 探索效率评估器测试通过")
    print()


def test_benchmark_framework():
    """测试基准测试框架。"""
    print("=" * 60)
    print("测试 6: 基准测试框架")
    print("=" * 60)

    framework = BenchmarkFramework()

    # 定义测试方法
    def run_method_a():
        time.sleep(0.01)
        return {
            "update_metrics": UpdateMetrics(
                avg_update_time_ms=10.0,
                max_update_time_ms=15.0,
                min_update_time_ms=5.0,
                update_frequency_hz=100.0,
                total_updates=100,
            ),
        }

    def run_method_b():
        time.sleep(0.02)
        return {
            "update_metrics": UpdateMetrics(
                avg_update_time_ms=20.0,
                max_update_time_ms=30.0,
                min_update_time_ms=10.0,
                update_frequency_hz=50.0,
                total_updates=50,
            ),
        }

    # 运行基准测试
    framework.run_benchmark(
        method_name="Method A",
        scene_id="test_scene",
        run_fn=run_method_a,
        evaluate_memory=True,
        evaluate_update=True,
    )

    framework.run_benchmark(
        method_name="Method B",
        scene_id="test_scene",
        run_fn=run_method_b,
        evaluate_memory=True,
        evaluate_update=True,
    )

    print(f"结果数量: {len(framework.results)}")
    assert len(framework.results) == 2

    # 打印摘要
    framework.print_summary()

    # 保存结果
    import tempfile
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        framework.save_results(f.name)
        print(f"\n结果已保存到: {f.name}")

    print("\n✓ 基准测试框架测试通过")
    print()


def test_metrics_dataclasses():
    """测试指标数据类。"""
    print("=" * 60)
    print("测试 7: 指标数据类")
    print("=" * 60)

    # MemoryMetrics
    mem = MemoryMetrics(
        total_memory_mb=100.0,
        map_memory_mb=50.0,
        graph_memory_mb=30.0,
        other_memory_mb=20.0,
        peak_memory_mb=120.0,
    )
    print(f"MemoryMetrics: {mem.total_memory_mb} MB")

    # UpdateMetrics
    update = UpdateMetrics(
        avg_update_time_ms=10.0,
        max_update_time_ms=15.0,
        min_update_time_ms=5.0,
        update_frequency_hz=100.0,
        total_updates=100,
    )
    print(f"UpdateMetrics: {update.avg_update_time_ms} ms")

    # PathMetrics
    path = PathMetrics(
        path_length=10.0,
        path_smoothness=0.9,
        path_clearance=0.5,
        planning_time_ms=1.0,
        success_rate=1.0,
        num_waypoints=10,
    )
    print(f"PathMetrics: {path.path_length} m")

    # ExplorationMetrics
    exploration = ExplorationMetrics(
        coverage_ratio=0.8,
        exploration_time_s=100.0,
        travel_distance_m=50.0,
        num_frontiers=10,
        efficiency=0.008,
    )
    print(f"ExplorationMetrics: {exploration.coverage_ratio:.2%}")

    print("\n✓ 指标数据类测试通过")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("评估框架测试套件")
    print("=" * 60 + "\n")

    try:
        test_metrics_dataclasses()
        test_memory_evaluator()
        test_update_evaluator()
        test_path_evaluator()
        test_path_smoothness()
        test_exploration_evaluator()
        test_benchmark_framework()

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
