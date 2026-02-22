"""
可视化工具 (Visualization Tools) — 路径、地图、SCG 和性能可视化。

功能:
  1. 路径可视化（2D/3D）
  2. 地图可视化（点云 + 占据栅格）
  3. SCG 可视化（节点 + 边）
  4. 性能曲线图
  5. 对比图表

依赖:
  - matplotlib: 2D/3D 绘图
  - plotly: 交互式可视化
  - numpy: 数据处理

用法:
    visualizer = PathVisualizer()
    visualizer.plot_path(path, occupancy_grid)
    visualizer.save("path.png")
"""

import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from mpl_toolkits.mplot3d import Axes3D

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  路径可视化器
# ══════════════════════════════════════════════════════════════════

class PathVisualizer:
    """
    路径可视化器 — 2D/3D 路径可视化。

    用法:
        visualizer = PathVisualizer()
        visualizer.plot_path_2d(path, occupancy_grid)
        visualizer.save("path_2d.png")
    """

    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """
        Args:
            figsize: 图像大小 (width, height)
        """
        self.figsize = figsize
        self.fig = None
        self.ax = None

    def plot_path_2d(
        self,
        path: np.ndarray,
        occupancy_grid: Optional[np.ndarray] = None,
        start: Optional[np.ndarray] = None,
        goal: Optional[np.ndarray] = None,
        title: str = "Path Visualization (2D)",
    ):
        """
        绘制 2D 路径。

        Args:
            path: (N, 3) 路径点
            occupancy_grid: (X, Y, Z) 占据栅格（可选）
            start: 起点 [x, y, z]（可选）
            goal: 终点 [x, y, z]（可选）
            title: 图表标题
        """
        self.fig, self.ax = plt.subplots(figsize=self.figsize)

        # 绘制占据栅格（如果提供）
        if occupancy_grid is not None:
            # 投影到 2D（取 z=0 层）
            grid_2d = occupancy_grid[:, :, 0]
            extent = [0, grid_2d.shape[1], 0, grid_2d.shape[0]]
            self.ax.imshow(
                grid_2d.T,
                origin='lower',
                cmap='gray_r',
                alpha=0.5,
                extent=extent,
            )

        # 绘制路径
        if len(path) > 0:
            self.ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label='Path')
            self.ax.scatter(path[:, 0], path[:, 1], c='blue', s=20, alpha=0.6)

        # 绘制起点和终点
        if start is not None:
            self.ax.scatter(start[0], start[1], c='green', s=200, marker='o',
                          edgecolors='black', linewidths=2, label='Start', zorder=5)

        if goal is not None:
            self.ax.scatter(goal[0], goal[1], c='red', s=200, marker='*',
                          edgecolors='black', linewidths=2, label='Goal', zorder=5)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title(title)
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

    def plot_path_3d(
        self,
        path: np.ndarray,
        point_cloud: Optional[np.ndarray] = None,
        start: Optional[np.ndarray] = None,
        goal: Optional[np.ndarray] = None,
        title: str = "Path Visualization (3D)",
    ):
        """
        绘制 3D 路径。

        Args:
            path: (N, 3) 路径点
            point_cloud: (M, 3) 点云（可选）
            start: 起点 [x, y, z]（可选）
            goal: 终点 [x, y, z]（可选）
            title: 图表标题
        """
        self.fig = plt.figure(figsize=self.figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')

        # 绘制点云（如果提供）
        if point_cloud is not None and len(point_cloud) > 0:
            # 下采样点云以提高性能
            if len(point_cloud) > 10000:
                indices = np.random.choice(len(point_cloud), 10000, replace=False)
                point_cloud = point_cloud[indices]

            self.ax.scatter(
                point_cloud[:, 0],
                point_cloud[:, 1],
                point_cloud[:, 2],
                c='gray',
                s=1,
                alpha=0.3,
                label='Point Cloud',
            )

        # 绘制路径
        if len(path) > 0:
            self.ax.plot(path[:, 0], path[:, 1], path[:, 2],
                        'b-', linewidth=2, label='Path')
            self.ax.scatter(path[:, 0], path[:, 1], path[:, 2],
                          c='blue', s=20, alpha=0.6)

        # 绘制起点和终点
        if start is not None:
            self.ax.scatter(start[0], start[1], start[2],
                          c='green', s=200, marker='o',
                          edgecolors='black', linewidths=2, label='Start')

        if goal is not None:
            self.ax.scatter(goal[0], goal[1], goal[2],
                          c='red', s=200, marker='*',
                          edgecolors='black', linewidths=2, label='Goal')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(title)
        self.ax.legend()

    def save(self, filepath: str, dpi: int = 300):
        """保存图像。"""
        if self.fig is not None:
            self.fig.savefig(filepath, dpi=dpi, bbox_inches='tight')
            logger.info(f"Saved visualization to {filepath}")
        else:
            logger.warning("No figure to save")

    def show(self):
        """显示图像。"""
        if self.fig is not None:
            plt.show()
        else:
            logger.warning("No figure to show")

    def close(self):
        """关闭图像。"""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.ax = None


# ══════════════════════════════════════════════════════════════════
#  SCG 可视化器
# ══════════════════════════════════════════════════════════════════

class SCGVisualizer:
    """
    SCG 可视化器 — 空间连通图可视化。

    用法:
        visualizer = SCGVisualizer()
        visualizer.plot_scg(scg_builder)
        visualizer.save("scg.png")
    """

    def __init__(self, figsize: Tuple[int, int] = (14, 10)):
        """
        Args:
            figsize: 图像大小 (width, height)
        """
        self.figsize = figsize
        self.fig = None
        self.ax = None

    def plot_scg_2d(
        self,
        scg_builder,
        title: str = "Spatial Connectivity Graph (2D)",
    ):
        """
        绘制 2D SCG。

        Args:
            scg_builder: SCGBuilder 对象
            title: 图表标题
        """
        self.fig, self.ax = plt.subplots(figsize=self.figsize)

        # 绘制多面体（圆形表示）
        for poly_id, poly in scg_builder.nodes.items():
            circle = patches.Circle(
                (poly.center[0], poly.center[1]),
                poly.radius,
                fill=True,
                facecolor='lightblue',
                edgecolor='blue',
                alpha=0.3,
                linewidth=2,
            )
            self.ax.add_patch(circle)

            # 标注 ID
            self.ax.text(
                poly.center[0],
                poly.center[1],
                str(poly_id),
                ha='center',
                va='center',
                fontsize=10,
                fontweight='bold',
            )

        # 绘制边
        edge_colors = {
            'adjacency': 'green',
            'connectivity': 'blue',
            'accessibility': 'orange',
        }

        for edge in scg_builder.edges:
            poly1 = scg_builder.nodes[edge.from_id]
            poly2 = scg_builder.nodes[edge.to_id]

            color = edge_colors.get(edge.edge_type.value, 'gray')
            linestyle = '-' if edge.edge_type.value == 'adjacency' else '--'

            self.ax.plot(
                [poly1.center[0], poly2.center[0]],
                [poly1.center[1], poly2.center[1]],
                color=color,
                linestyle=linestyle,
                linewidth=1.5,
                alpha=0.6,
            )

        # 图例
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='green', linewidth=2, label='Adjacency'),
            Line2D([0], [0], color='blue', linewidth=2, linestyle='--', label='Connectivity'),
            Line2D([0], [0], color='orange', linewidth=2, linestyle='--', label='Accessibility'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title(title)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

    def plot_scg_3d(
        self,
        scg_builder,
        title: str = "Spatial Connectivity Graph (3D)",
    ):
        """
        绘制 3D SCG。

        Args:
            scg_builder: SCGBuilder 对象
            title: 图表标题
        """
        self.fig = plt.figure(figsize=self.figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')

        # 绘制多面体中心
        centers = np.array([poly.center for poly in scg_builder.nodes.values()])
        if len(centers) > 0:
            self.ax.scatter(
                centers[:, 0],
                centers[:, 1],
                centers[:, 2],
                c='blue',
                s=200,
                alpha=0.6,
                edgecolors='black',
                linewidths=2,
            )

            # 标注 ID
            for poly_id, poly in scg_builder.nodes.items():
                self.ax.text(
                    poly.center[0],
                    poly.center[1],
                    poly.center[2],
                    str(poly_id),
                    fontsize=8,
                )

        # 绘制边
        edge_colors = {
            'adjacency': 'green',
            'connectivity': 'blue',
            'accessibility': 'orange',
        }

        for edge in scg_builder.edges:
            poly1 = scg_builder.nodes[edge.from_id]
            poly2 = scg_builder.nodes[edge.to_id]

            color = edge_colors.get(edge.edge_type.value, 'gray')

            self.ax.plot(
                [poly1.center[0], poly2.center[0]],
                [poly1.center[1], poly2.center[1]],
                [poly1.center[2], poly2.center[2]],
                color=color,
                linewidth=1.5,
                alpha=0.6,
            )

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(title)

    def save(self, filepath: str, dpi: int = 300):
        """保存图像。"""
        if self.fig is not None:
            self.fig.savefig(filepath, dpi=dpi, bbox_inches='tight')
            logger.info(f"Saved SCG visualization to {filepath}")

    def show(self):
        """显示图像。"""
        if self.fig is not None:
            plt.show()

    def close(self):
        """关闭图像。"""
        if self.fig is not None:
            plt.close(self.fig)


# ══════════════════════════════════════════════════════════════════
#  性能可视化器
# ══════════════════════════════════════════════════════════════════

class PerformanceVisualizer:
    """
    性能可视化器 — 性能曲线和对比图表。

    用法:
        visualizer = PerformanceVisualizer()
        visualizer.plot_comparison(results)
        visualizer.save("performance.png")
    """

    def __init__(self, figsize: Tuple[int, int] = (14, 10)):
        """
        Args:
            figsize: 图像大小 (width, height)
        """
        self.figsize = figsize

    def plot_comparison(
        self,
        results: List,
        metrics: List[str] = ['memory', 'update_time', 'planning_time'],
        title: str = "Performance Comparison",
    ):
        """
        绘制性能对比图。

        Args:
            results: BenchmarkResult 列表
            metrics: 要对比的指标列表
            title: 图表标题
        """
        # 按方法分组
        methods = {}
        for result in results:
            if result.method_name not in methods:
                methods[result.method_name] = []
            methods[result.method_name].append(result)

        # 创建子图
        n_metrics = len(metrics)
        fig, axes = plt.subplots(1, n_metrics, figsize=self.figsize)
        if n_metrics == 1:
            axes = [axes]

        # 对每个指标绘制对比图
        for idx, metric in enumerate(metrics):
            ax = axes[idx]

            method_names = []
            values = []
            errors = []

            for method_name, method_results in methods.items():
                method_names.append(method_name)

                # 提取指标值
                if metric == 'memory':
                    vals = [r.memory.total_memory_mb for r in method_results if r.memory]
                elif metric == 'update_time':
                    vals = [r.update.avg_update_time_ms for r in method_results if r.update]
                elif metric == 'planning_time':
                    vals = [r.path.planning_time_ms for r in method_results if r.path]
                else:
                    vals = []

                if vals:
                    values.append(np.mean(vals))
                    errors.append(np.std(vals))
                else:
                    values.append(0)
                    errors.append(0)

            # 绘制柱状图
            x_pos = np.arange(len(method_names))
            ax.bar(x_pos, values, yerr=errors, capsize=5, alpha=0.7)
            ax.set_xticks(x_pos)
            ax.set_xticklabels(method_names, rotation=45, ha='right')

            # 设置标签
            if metric == 'memory':
                ax.set_ylabel('Memory (MB)')
                ax.set_title('Memory Usage')
            elif metric == 'update_time':
                ax.set_ylabel('Time (ms)')
                ax.set_title('Update Time')
            elif metric == 'planning_time':
                ax.set_ylabel('Time (ms)')
                ax.set_title('Planning Time')

            ax.grid(True, alpha=0.3, axis='y')

        fig.suptitle(title, fontsize=16, fontweight='bold')
        plt.tight_layout()

        return fig

    def plot_time_series(
        self,
        data: Dict[str, List[float]],
        title: str = "Performance Over Time",
        ylabel: str = "Value",
    ):
        """
        绘制时间序列图。

        Args:
            data: {method_name: [values]} 字典
            title: 图表标题
            ylabel: Y 轴标签
        """
        fig, ax = plt.subplots(figsize=self.figsize)

        for method_name, values in data.items():
            ax.plot(values, label=method_name, linewidth=2, marker='o')

        ax.set_xlabel('Frame')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)

        return fig

    @staticmethod
    def save(fig, filepath: str, dpi: int = 300):
        """保存图像。"""
        fig.savefig(filepath, dpi=dpi, bbox_inches='tight')
        logger.info(f"Saved performance visualization to {filepath}")

    @staticmethod
    def show(fig):
        """显示图像。"""
        plt.show()


# ══════════════════════════════════════════════════════════════════
#  综合可视化器
# ══════════════════════════════════════════════════════════════════

class ComprehensiveVisualizer:
    """
    综合可视化器 — 一站式可视化接口。

    用法:
        visualizer = ComprehensiveVisualizer()
        visualizer.visualize_all(path, scg_builder, results, output_dir)
    """

    def __init__(self):
        self.path_viz = PathVisualizer()
        self.scg_viz = SCGVisualizer()
        self.perf_viz = PerformanceVisualizer()

    def visualize_all(
        self,
        path: Optional[np.ndarray] = None,
        scg_builder = None,
        results: Optional[List] = None,
        output_dir: str = "visualizations",
        point_cloud: Optional[np.ndarray] = None,
        occupancy_grid: Optional[np.ndarray] = None,
    ):
        """
        生成所有可视化。

        Args:
            path: 路径点
            scg_builder: SCG 构建器
            results: 评估结果
            output_dir: 输出目录
            point_cloud: 点云
            occupancy_grid: 占据栅格
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # 1. 路径可视化
        if path is not None and len(path) > 0:
            # 2D 路径
            self.path_viz.plot_path_2d(
                path,
                occupancy_grid=occupancy_grid,
                start=path[0] if len(path) > 0 else None,
                goal=path[-1] if len(path) > 0 else None,
            )
            self.path_viz.save(str(output_path / "path_2d.png"))
            self.path_viz.close()

            # 3D 路径
            self.path_viz.plot_path_3d(
                path,
                point_cloud=point_cloud,
                start=path[0] if len(path) > 0 else None,
                goal=path[-1] if len(path) > 0 else None,
            )
            self.path_viz.save(str(output_path / "path_3d.png"))
            self.path_viz.close()

            logger.info("✓ Path visualizations saved")

        # 2. SCG 可视化
        if scg_builder is not None and len(scg_builder.nodes) > 0:
            # 2D SCG
            self.scg_viz.plot_scg_2d(scg_builder)
            self.scg_viz.save(str(output_path / "scg_2d.png"))
            self.scg_viz.close()

            # 3D SCG
            self.scg_viz.plot_scg_3d(scg_builder)
            self.scg_viz.save(str(output_path / "scg_3d.png"))
            self.scg_viz.close()

            logger.info("✓ SCG visualizations saved")

        # 3. 性能可视化
        if results is not None and len(results) > 0:
            fig = self.perf_viz.plot_comparison(results)
            self.perf_viz.save(fig, str(output_path / "performance_comparison.png"))
            plt.close(fig)

            logger.info("✓ Performance visualizations saved")

        logger.info(f"All visualizations saved to {output_dir}")
