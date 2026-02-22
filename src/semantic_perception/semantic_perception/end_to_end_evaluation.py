"""
端到端系统评估 (End-to-End Evaluation) — 完整的系统评估流程。

功能:
  1. 在 HM3D/Gibson 数据集上运行完整流程
  2. 对比 PCT A* 和 USS-Nav 性能
  3. 生成详细的评估报告
  4. 可视化结果

评估指标:
  - 内存占用
  - 更新速率
  - 路径质量
  - 探索效率

输出:
  - JSON 格式的详细结果
  - Markdown 格式的报告
  - 性能对比图表
"""

import json
import logging
import time
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from semantic_perception.baseline_wrappers import create_planner
from semantic_perception.dataset_loader import create_dataset_loader
from semantic_perception.evaluation_framework import (
    BenchmarkFramework,
    MemoryEvaluator,
    UpdateEvaluator,
    PathEvaluator,
    ExplorationEvaluator,
    BenchmarkResult,
)

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  端到端评估器
# ══════════════════════════════════════════════════════════════════

class EndToEndEvaluator:
    """
    端到端评估器 — 完整的系统评估流程。

    用法:
        evaluator = EndToEndEvaluator()
        results = evaluator.run_evaluation(
            dataset_type="hm3d",
            dataset_root="/path/to/hm3d",
            scene_ids=["scene1", "scene2"],
            methods=["pct_astar", "uss_nav"],
        )
        evaluator.save_results("results.json")
        evaluator.generate_report("report.md")
    """

    def __init__(self):
        self.results: List[BenchmarkResult] = []
        self.framework = BenchmarkFramework()

    def run_evaluation(
        self,
        dataset_type: str,
        dataset_root: Path,
        scene_ids: List[str],
        methods: List[str],
        num_frames: int = 10,
        planning_queries: int = 5,
    ) -> List[BenchmarkResult]:
        """
        运行完整评估。

        Args:
            dataset_type: 数据集类型 ("hm3d" | "gibson")
            dataset_root: 数据集根目录
            scene_ids: 场景 ID 列表
            methods: 方法列表 ("pct_astar" | "uss_nav")
            num_frames: 每个场景使用的帧数
            planning_queries: 路径规划查询次数

        Returns:
            评估结果列表
        """
        logger.info(f"Starting end-to-end evaluation")
        logger.info(f"  Dataset: {dataset_type}")
        logger.info(f"  Scenes: {len(scene_ids)}")
        logger.info(f"  Methods: {methods}")
        logger.info(f"  Frames per scene: {num_frames}")
        logger.info(f"  Planning queries: {planning_queries}")

        # 加载数据集
        try:
            dataset = create_dataset_loader(dataset_type, dataset_root)
        except Exception as e:
            logger.error(f"Failed to load dataset: {e}")
            return []

        # 对每个场景和方法运行评估
        for scene_id in scene_ids:
            logger.info(f"\n{'='*60}")
            logger.info(f"Evaluating scene: {scene_id}")
            logger.info(f"{'='*60}")

            for method in methods:
                logger.info(f"\nMethod: {method}")

                try:
                    result = self._evaluate_scene_method(
                        dataset=dataset,
                        scene_id=scene_id,
                        method=method,
                        num_frames=num_frames,
                        planning_queries=planning_queries,
                    )

                    if result is not None:
                        self.results.append(result)
                        self.framework.results.append(result)

                except Exception as e:
                    logger.error(f"Evaluation failed for {method} on {scene_id}: {e}")
                    import traceback
                    traceback.print_exc()

        logger.info(f"\n{'='*60}")
        logger.info(f"Evaluation completed: {len(self.results)} results")
        logger.info(f"{'='*60}")

        return self.results

    def _evaluate_scene_method(
        self,
        dataset,
        scene_id: str,
        method: str,
        num_frames: int,
        planning_queries: int,
    ) -> Optional[BenchmarkResult]:
        """
        评估单个场景和方法。

        Args:
            dataset: 数据集加载器
            scene_id: 场景 ID
            method: 方法名称
            num_frames: 帧数
            planning_queries: 规划查询次数

        Returns:
            BenchmarkResult 或 None
        """
        # 1. 创建规划器
        if method == "pct_astar":
            planner = create_planner("pct_astar", resolution=0.1, size=(200, 200, 40))
        elif method == "uss_nav":
            planner = create_planner("uss_nav")
        else:
            logger.error(f"Unknown method: {method}")
            return None

        # 2. 加载场景数据
        try:
            metadata = dataset.load_scene_metadata(scene_id)
            frame_ids = list(range(min(num_frames, metadata.num_frames)))
            frames = dataset.load_trajectory(
                scene_id,
                frame_ids=frame_ids,
                load_depth=True,
                load_point_cloud=True,
            )
        except Exception as e:
            logger.error(f"Failed to load scene data: {e}")
            return None

        # 3. 评估器
        mem_eval = MemoryEvaluator()
        update_eval = UpdateEvaluator()
        path_eval = PathEvaluator()

        # 4. 开始评估
        mem_eval.start()

        # 5. 更新地图
        logger.info(f"  Updating map with {len(frames)} frames...")
        for frame in frames:
            if frame.point_cloud is None or len(frame.point_cloud) == 0:
                continue

            robot_pose = frame.camera_pose[:3, 3]  # 提取位置
            robot_pose = np.concatenate([robot_pose, [0, 0, 0]])  # 添加姿态

            update_eval.start_update()
            planner.update(robot_pose, frame.point_cloud)
            update_eval.end_update()
            mem_eval.update_peak()

        update_metrics = update_eval.get_metrics()

        # 6. 路径规划
        logger.info(f"  Planning {planning_queries} paths...")
        path_metrics_list = []

        for i in range(planning_queries):
            # 随机生成起点和终点
            bounds = metadata.bounds
            start = np.random.uniform(bounds[0], bounds[1])
            goal = np.random.uniform(bounds[0], bounds[1])

            start_time = time.time()
            path = planner.plan(start, goal)
            planning_time = time.time() - start_time

            if path is not None:
                metrics = path_eval.evaluate_path(path, planning_time=planning_time)
                path_metrics_list.append(metrics)

        # 平均路径指标
        if path_metrics_list:
            from semantic_perception.evaluation_framework import PathMetrics
            avg_path_metrics = PathMetrics(
                path_length=np.mean([m.path_length for m in path_metrics_list]),
                path_smoothness=np.mean([m.path_smoothness for m in path_metrics_list]),
                path_clearance=np.mean([m.path_clearance for m in path_metrics_list]),
                planning_time_ms=np.mean([m.planning_time_ms for m in path_metrics_list]),
                success_rate=sum(1 for m in path_metrics_list if m.success_rate > 0) / len(path_metrics_list),
                num_waypoints=int(np.mean([m.num_waypoints for m in path_metrics_list])),
            )
        else:
            avg_path_metrics = None

        # 7. 停止内存评估
        memory_metrics = mem_eval.stop()

        # 8. 获取规划器统计
        planner_stats = planner.get_statistics()

        # 9. 创建结果
        result = BenchmarkResult(
            method_name=method,
            scene_id=scene_id,
            timestamp=time.time(),
            memory=memory_metrics,
            update=update_metrics,
            path=avg_path_metrics,
            metadata={
                "num_frames": len(frames),
                "planning_queries": planning_queries,
                "planner_stats": planner_stats,
            },
        )

        logger.info(f"  ✓ Evaluation completed")
        logger.info(f"    Memory: {memory_metrics.total_memory_mb:.2f} MB")
        logger.info(f"    Update time: {update_metrics.avg_update_time_ms:.2f} ms")
        if avg_path_metrics:
            logger.info(f"    Planning time: {avg_path_metrics.planning_time_ms:.2f} ms")
            logger.info(f"    Success rate: {avg_path_metrics.success_rate:.2%}")

        return result

    def save_results(self, filepath: str):
        """保存结果到 JSON 文件。"""
        data = []
        for result in self.results:
            result_dict = asdict(result)
            # 转换 numpy 数组为列表
            if result.memory:
                pass  # 已经是基本类型
            data.append(result_dict)

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

        logger.info(f"Results saved to {filepath}")

    def generate_report(self, filepath: str):
        """生成 Markdown 格式的报告。"""
        with open(filepath, "w") as f:
            f.write("# 端到端系统评估报告\n\n")
            f.write(f"**生成时间**: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"**评估场景数**: {len(set(r.scene_id for r in self.results))}\n\n")
            f.write(f"**评估方法数**: {len(set(r.method_name for r in self.results))}\n\n")

            f.write("---\n\n")
            f.write("## 评估结果\n\n")

            # 按方法分组
            methods = {}
            for result in self.results:
                if result.method_name not in methods:
                    methods[result.method_name] = []
                methods[result.method_name].append(result)

            # 对每个方法生成报告
            for method_name, results in methods.items():
                f.write(f"### {method_name}\n\n")

                # 内存统计
                memory_values = [r.memory.total_memory_mb for r in results if r.memory]
                if memory_values:
                    f.write(f"**内存占用**:\n")
                    f.write(f"- 平均: {np.mean(memory_values):.2f} MB\n")
                    f.write(f"- 最大: {np.max(memory_values):.2f} MB\n")
                    f.write(f"- 最小: {np.min(memory_values):.2f} MB\n\n")

                # 更新速率统计
                update_values = [r.update.avg_update_time_ms for r in results if r.update]
                if update_values:
                    f.write(f"**更新速率**:\n")
                    f.write(f"- 平均更新时间: {np.mean(update_values):.2f} ms\n")
                    f.write(f"- 更新频率: {1000.0 / np.mean(update_values):.2f} Hz\n\n")

                # 路径质量统计
                path_lengths = [r.path.path_length for r in results if r.path]
                path_smoothness = [r.path.path_smoothness for r in results if r.path]
                planning_times = [r.path.planning_time_ms for r in results if r.path]

                if path_lengths:
                    f.write(f"**路径质量**:\n")
                    f.write(f"- 平均路径长度: {np.mean(path_lengths):.2f} m\n")
                    f.write(f"- 平均平滑度: {np.mean(path_smoothness):.2f}\n")
                    f.write(f"- 平均规划时间: {np.mean(planning_times):.2f} ms\n\n")

                f.write("---\n\n")

            # 对比表格
            f.write("## 性能对比\n\n")
            f.write("| 方法 | 内存 (MB) | 更新时间 (ms) | 规划时间 (ms) | 路径平滑度 |\n")
            f.write("|------|-----------|--------------|--------------|------------|\n")

            for method_name, results in methods.items():
                memory = np.mean([r.memory.total_memory_mb for r in results if r.memory])
                update_time = np.mean([r.update.avg_update_time_ms for r in results if r.update])
                planning_time = np.mean([r.path.planning_time_ms for r in results if r.path])
                smoothness = np.mean([r.path.path_smoothness for r in results if r.path])

                f.write(f"| {method_name} | {memory:.2f} | {update_time:.2f} | {planning_time:.2f} | {smoothness:.2f} |\n")

            f.write("\n---\n\n")
            f.write("## 结论\n\n")
            f.write("（待补充）\n")

        logger.info(f"Report generated: {filepath}")


# ══════════════════════════════════════════════════════════════════
#  命令行接口
# ══════════════════════════════════════════════════════════════════

def main():
    """命令行入口。"""
    import argparse

    parser = argparse.ArgumentParser(description="端到端系统评估")
    parser.add_argument("--dataset", type=str, default="hm3d", help="数据集类型")
    parser.add_argument("--dataset-root", type=str, required=True, help="数据集根目录")
    parser.add_argument("--scenes", type=str, nargs="+", help="场景 ID 列表")
    parser.add_argument("--methods", type=str, nargs="+", default=["pct_astar", "uss_nav"], help="方法列表")
    parser.add_argument("--num-frames", type=int, default=10, help="每个场景的帧数")
    parser.add_argument("--planning-queries", type=int, default=5, help="路径规划查询次数")
    parser.add_argument("--output", type=str, default="results.json", help="输出文件")
    parser.add_argument("--report", type=str, default="report.md", help="报告文件")

    args = parser.parse_args()

    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # 创建评估器
    evaluator = EndToEndEvaluator()

    # 运行评估
    results = evaluator.run_evaluation(
        dataset_type=args.dataset,
        dataset_root=Path(args.dataset_root),
        scene_ids=args.scenes,
        methods=args.methods,
        num_frames=args.num_frames,
        planning_queries=args.planning_queries,
    )

    # 保存结果
    if results:
        evaluator.save_results(args.output)
        evaluator.generate_report(args.report)
        print(f"\n✓ Evaluation completed: {len(results)} results")
        print(f"  Results: {args.output}")
        print(f"  Report: {args.report}")
    else:
        print("\n✗ Evaluation failed: no results")


if __name__ == "__main__":
    main()
