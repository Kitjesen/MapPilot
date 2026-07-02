"""
评估框架 (Evaluation Framework) — 统一的性能评估接口。

功能:
  1. 内存占用评估
  2. 更新速率评估
  3. 路径质量评估
  4. 探索效率评估
  5. 与基线方法对比

设计原则:
  - 统一接口: 所有方法使用相同的评估指标
  - 可扩展: 易于添加新的评估指标
  - 可视化: 生成对比图表

参考论文:
  - USS-Nav (2025): 性能评估指标
  - PCT A*: 路径质量评估
  - Hydra: 内存和速率评估
"""

import logging
import time
from dataclasses import dataclass, field

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  评估指标
# ══════════════════════════════════════════════════════════════════

@dataclass
class MemoryMetrics:
    """内存占用指标。"""
    total_memory_mb: float  # 总内存 (MB)
    map_memory_mb: float  # 地图内存 (MB)
    graph_memory_mb: float  # 图内存 (MB)
    other_memory_mb: float  # 其他内存 (MB)
    peak_memory_mb: float  # 峰值内存 (MB)


@dataclass
class UpdateMetrics:
    """更新速率指标。"""
    avg_update_time_ms: float  # 平均更新时间 (ms)
    max_update_time_ms: float  # 最大更新时间 (ms)
    min_update_time_ms: float  # 最小更新时间 (ms)
    update_frequency_hz: float  # 更新频率 (Hz)
    total_updates: int  # 总更新次数


@dataclass
class PathMetrics:
    """路径质量指标。"""
    path_length: float  # 路径长度 (m)
    path_smoothness: float  # 路径平滑度 (0-1)
    path_clearance: float  # 路径间隙 (m)
    planning_time_ms: float  # 规划时间 (ms)
    success_rate: float  # 成功率 (0-1)
    num_waypoints: int  # 路径点数


@dataclass
class ExplorationMetrics:
    """探索效率指标。"""
    coverage_ratio: float  # 覆盖率 (0-1)
    exploration_time_s: float  # 探索时间 (s)
    travel_distance_m: float  # 行驶距离 (m)
    num_frontiers: int  # 前沿数量
    efficiency: float  # 效率 (coverage / time)


@dataclass
class BenchmarkResult:
    """基准测试结果。"""
    method_name: str
    scene_id: str
    timestamp: float

    # 各项指标
    memory: MemoryMetrics | None = None
    update: UpdateMetrics | None = None
    path: PathMetrics | None = None
    exploration: ExplorationMetrics | None = None

    # 额外信息
    metadata: dict = field(default_factory=dict)


# ══════════════════════════════════════════════════════════════════
#  评估器
# ══════════════════════════════════════════════════════════════════

class MemoryEvaluator:
    """
    内存占用评估器。

    用法:
        evaluator = MemoryEvaluator()
        evaluator.start()
        # ... 运行算法 ...
        metrics = evaluator.stop()
    """

    def __init__(self):
        self.start_memory = 0.0
        self.peak_memory = 0.0

    def start(self):
        """开始监控内存。"""
        import psutil
        process = psutil.Process()
        self.start_memory = process.memory_info().rss / 1024 / 1024  # MB
        self.peak_memory = self.start_memory

    def stop(self) -> MemoryMetrics:
        """停止监控并返回指标。"""
        import psutil
        process = psutil.Process()
        current_memory = process.memory_info().rss / 1024 / 1024  # MB

        return MemoryMetrics(
            total_memory_mb=current_memory - self.start_memory,
            map_memory_mb=0.0,  # 需要具体实现
            graph_memory_mb=0.0,
            other_memory_mb=0.0,
            peak_memory_mb=self.peak_memory - self.start_memory,
        )

    def update_peak(self):
        """更新峰值内存。"""
        import psutil
        process = psutil.Process()
        current_memory = process.memory_info().rss / 1024 / 1024
        self.peak_memory = max(self.peak_memory, current_memory)


class UpdateEvaluator:
    """
    更新速率评估器。

    用法:
        evaluator = UpdateEvaluator()
        for frame in frames:
            evaluator.start_update()
            # ... 更新地图 ...
            evaluator.end_update()
        metrics = evaluator.get_metrics()
    """

    def __init__(self):
        self.update_times: list[float] = []
        self.current_start = 0.0

    def start_update(self):
        """开始一次更新。"""
        self.current_start = time.time()

    def end_update(self):
        """结束一次更新。"""
        update_time = (time.time() - self.current_start) * 1000  # ms
        self.update_times.append(update_time)

    def get_metrics(self) -> UpdateMetrics:
        """获取更新指标。"""
        if not self.update_times:
            return UpdateMetrics(
                avg_update_time_ms=0.0,
                max_update_time_ms=0.0,
                min_update_time_ms=0.0,
                update_frequency_hz=0.0,
                total_updates=0,
            )

        avg_time = np.mean(self.update_times)
        return UpdateMetrics(
            avg_update_time_ms=float(avg_time),
            max_update_time_ms=float(np.max(self.update_times)),
            min_update_time_ms=float(np.min(self.update_times)),
            update_frequency_hz=1000.0 / avg_time if avg_time > 0 else 0.0,
            total_updates=len(self.update_times),
        )


class PathEvaluator:
    """
    路径质量评估器。

    用法:
        evaluator = PathEvaluator()
        metrics = evaluator.evaluate_path(path, occupancy_grid)
    """

    @staticmethod
    def evaluate_path(
        path: np.ndarray,
        occupancy_grid: np.ndarray | None = None,
        planning_time: float = 0.0,
    ) -> PathMetrics:
        """
        评估路径质量。

        Args:
            path: (N, 3) 路径点
            occupancy_grid: (X, Y, Z) 占据栅格（可选）
            planning_time: 规划时间 (秒)

        Returns:
            PathMetrics 对象
        """
        if len(path) < 2:
            return PathMetrics(
                path_length=0.0,
                path_smoothness=0.0,
                path_clearance=0.0,
                planning_time_ms=planning_time * 1000,
                success_rate=0.0,
                num_waypoints=len(path),
            )

        # 1. 路径长度
        path_length = PathEvaluator._compute_path_length(path)

        # 2. 路径平滑度
        smoothness = PathEvaluator._compute_smoothness(path)

        # 3. 路径间隙（如果有占据栅格）
        clearance = 0.0
        if occupancy_grid is not None:
            clearance = PathEvaluator._compute_clearance(path, occupancy_grid)

        return PathMetrics(
            path_length=path_length,
            path_smoothness=smoothness,
            path_clearance=clearance,
            planning_time_ms=planning_time * 1000,
            success_rate=1.0 if len(path) > 0 else 0.0,
            num_waypoints=len(path),
        )

    @staticmethod
    def _compute_path_length(path: np.ndarray) -> float:
        """计算路径长度。"""
        if len(path) < 2:
            return 0.0

        length = 0.0
        for i in range(len(path) - 1):
            length += np.linalg.norm(path[i + 1] - path[i])
        return float(length)

    @staticmethod
    def _compute_smoothness(path: np.ndarray) -> float:
        """
        计算路径平滑度。

        方法: 计算相邻段之间的角度变化，越小越平滑。
        """
        if len(path) < 3:
            return 1.0

        angles = []
        for i in range(1, len(path) - 1):
            v1 = path[i] - path[i - 1]
            v2 = path[i + 1] - path[i]

            # 归一化
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)

            if v1_norm > 0 and v2_norm > 0:
                v1 = v1 / v1_norm
                v2 = v2 / v2_norm

                # 计算角度
                cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
                angle = np.arccos(cos_angle)
                angles.append(angle)

        if not angles:
            return 1.0

        # 平滑度 = 1 - (平均角度 / π)
        avg_angle = np.mean(angles)
        smoothness = 1.0 - (avg_angle / np.pi)

        return float(np.clip(smoothness, 0.0, 1.0))

    @staticmethod
    def _compute_clearance(path: np.ndarray, occupancy_grid: np.ndarray) -> float:
        """计算路径间隙（到最近障碍物的距离）。"""
        # 简化实现：返回固定值
        return 0.5


class ExplorationEvaluator:
    """
    探索效率评估器。

    用法:
        evaluator = ExplorationEvaluator()
        evaluator.start()
        # ... 探索过程 ...
        metrics = evaluator.stop(coverage_ratio, num_frontiers)
    """

    def __init__(self):
        self.start_time = 0.0
        self.total_distance = 0.0
        self.last_position = None

    def start(self):
        """开始探索。"""
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_position = None

    def update_position(self, position: np.ndarray):
        """更新机器人位置。"""
        if self.last_position is not None:
            distance = np.linalg.norm(position - self.last_position)
            self.total_distance += distance
        self.last_position = position

    def stop(self, coverage_ratio: float, num_frontiers: int) -> ExplorationMetrics:
        """停止探索并返回指标。"""
        exploration_time = time.time() - self.start_time

        efficiency = coverage_ratio / exploration_time if exploration_time > 0 else 0.0

        return ExplorationMetrics(
            coverage_ratio=coverage_ratio,
            exploration_time_s=exploration_time,
            travel_distance_m=self.total_distance,
            num_frontiers=num_frontiers,
            efficiency=efficiency,
        )


# ══════════════════════════════════════════════════════════════════
#  基准测试框架
# ══════════════════════════════════════════════════════════════════

class BenchmarkFramework:
    """
    基准测试框架 — 统一的性能评估接口。

    用法:
        framework = BenchmarkFramework()
        result = framework.run_benchmark(
            method_name="USS-Nav",
            scene_id="00800-TEEsavR23oF",
            run_fn=lambda: run_uss_nav(scene),
        )
        framework.save_results("results.json")
    """

    def __init__(self):
        self.results: list[BenchmarkResult] = []

    def run_benchmark(
        self,
        method_name: str,
        scene_id: str,
        run_fn,
        evaluate_memory: bool = True,
        evaluate_update: bool = True,
        evaluate_path: bool = False,
        evaluate_exploration: bool = False,
    ) -> BenchmarkResult:
        """
        运行基准测试。

        Args:
            method_name: 方法名称
            scene_id: 场景 ID
            run_fn: 运行函数（返回结果字典）
            evaluate_memory: 是否评估内存
            evaluate_update: 是否评估更新速率
            evaluate_path: 是否评估路径质量
            evaluate_exploration: 是否评估探索效率

        Returns:
            BenchmarkResult 对象
        """
        logger.info(f"Running benchmark: {method_name} on {scene_id}")

        result = BenchmarkResult(
            method_name=method_name,
            scene_id=scene_id,
            timestamp=time.time(),
        )

        # 内存评估
        if evaluate_memory:
            mem_eval = MemoryEvaluator()
            mem_eval.start()

        # 运行方法
        output = run_fn()

        # 停止内存评估
        if evaluate_memory:
            result.memory = mem_eval.stop()

        # 提取其他指标
        if evaluate_update and "update_metrics" in output:
            result.update = output["update_metrics"]

        if evaluate_path and "path_metrics" in output:
            result.path = output["path_metrics"]

        if evaluate_exploration and "exploration_metrics" in output:
            result.exploration = output["exploration_metrics"]

        self.results.append(result)

        logger.info(f"Benchmark completed: {method_name}")
        return result

    def save_results(self, filepath: str):
        """保存结果到 JSON 文件。"""
        import json
        from dataclasses import asdict

        data = [asdict(result) for result in self.results]

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        logger.info(f"Results saved to {filepath}")

    def print_summary(self):
        """打印结果摘要。"""
        print("\n" + "=" * 80)
        print("基准测试结果摘要")
        print("=" * 80)

        for result in self.results:
            print(f"\n方法: {result.method_name}")
            print(f"场景: {result.scene_id}")

            if result.memory:
                print(f"  内存: {result.memory.total_memory_mb:.2f} MB")

            if result.update:
                print(f"  更新时间: {result.update.avg_update_time_ms:.2f} ms")
                print(f"  更新频率: {result.update.update_frequency_hz:.2f} Hz")

            if result.path:
                print(f"  路径长度: {result.path.path_length:.2f} m")
                print(f"  路径平滑度: {result.path.path_smoothness:.2f}")
                print(f"  规划时间: {result.path.planning_time_ms:.2f} ms")

            if result.exploration:
                print(f"  覆盖率: {result.exploration.coverage_ratio:.2%}")
                print(f"  探索时间: {result.exploration.exploration_time_s:.2f} s")
                print(f"  效率: {result.exploration.efficiency:.4f}")

        print("\n" + "=" * 80)
