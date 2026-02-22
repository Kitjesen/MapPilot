#!/usr/bin/env python3
"""
多面体扩展算法调试脚本
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
    SphereSampler,
)


def debug_polyhedron_expansion():
    """调试多面体扩展流程。"""
    print("=" * 60)
    print("调试: 多面体扩展")
    print("=" * 60)

    # 创建占据栅格 (30×30×10, 模拟更大的走廊)
    occupancy_grid = np.ones((30, 30, 10), dtype=np.float32)

    # 走廊: 中间 20×30 区域自由
    occupancy_grid[5:25, :, :] = 0.0

    # 添加一些障碍物
    occupancy_grid[12:18, 12:18, :] = 1.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    print(f"\n栅格信息:")
    print(f"  形状: {occupancy_grid.shape}")
    print(f"  分辨率: {grid_resolution}m")
    print(f"  原点: {grid_origin}")
    print(f"  自由空间栅格数: {np.sum(occupancy_grid < 0.5)}")
    print(f"  占据空间栅格数: {np.sum(occupancy_grid >= 0.5)}")

    # 配置
    config = PolyhedronExpansionConfig(
        num_sphere_samples=32,
        r_min=0.3,
        r_max=1.5,
        r_step=0.3,
        min_polyhedron_volume=0.1,
        max_polyhedra=15,
        coverage_threshold=0.5,
        collision_threshold=0.5,
    )

    print(f"\n配置参数:")
    print(f"  球面采样数: {config.num_sphere_samples}")
    print(f"  半径范围: {config.r_min}m - {config.r_max}m (步长 {config.r_step}m)")
    print(f"  最小体积: {config.min_polyhedron_volume}m³")
    print(f"  最大数量: {config.max_polyhedra}")

    # 创建扩展器
    expander = PolyhedronExpander(config)

    # 1. 提取候选点
    candidates = expander._extract_free_space_candidates(
        occupancy_grid, grid_resolution, grid_origin
    )
    print(f"\n候选点:")
    print(f"  数量: {len(candidates)}")
    if len(candidates) > 0:
        print(f"  范围: x=[{candidates[:, 0].min():.2f}, {candidates[:, 0].max():.2f}]")
        print(f"        y=[{candidates[:, 1].min():.2f}, {candidates[:, 1].max():.2f}]")
        print(f"        z=[{candidates[:, 2].min():.2f}, {candidates[:, 2].max():.2f}]")
        print(f"  中心: {candidates.mean(axis=0)}")

    # 2. 选择种子点
    from semantic_perception.polyhedron_expansion import SeedSelector
    seed_selector = SeedSelector()
    seed = seed_selector.select_next_seed(candidates, [])
    print(f"\n种子点: {seed}")

    # 3. 球面采样
    sampler = SphereSampler()
    directions = sampler.fibonacci_sphere(config.num_sphere_samples)
    radii = np.arange(config.r_min, config.r_max + config.r_step, config.r_step)

    print(f"\n球面采样:")
    print(f"  方向数: {len(directions)}")
    print(f"  半径数: {len(radii)} {radii}")

    sample_points = sampler.sample_free_space(
        seed=seed,
        directions=directions,
        radii=radii,
        occupancy_grid=occupancy_grid,
        grid_resolution=grid_resolution,
        grid_origin=grid_origin,
    )

    print(f"  采样点数: {len(sample_points)}")

    if len(sample_points) < 4:
        print(f"\n⚠️ 采样点不足 4 个，无法构成凸包")

        # 调试: 检查几个方向的采样
        print(f"\n调试采样 (前 5 个方向, 第一个半径 r={radii[0]}):")
        for i, d in enumerate(directions[:5]):
            p = seed + radii[0] * d
            grid_pos = ((p - grid_origin) / grid_resolution).astype(int)
            in_bounds = np.all(grid_pos >= 0) and np.all(grid_pos < occupancy_grid.shape)
            if in_bounds:
                is_free = occupancy_grid[tuple(grid_pos)] < 0.5
                print(f"  方向 {i}: p={p}, grid={grid_pos}, in_bounds={in_bounds}, is_free={is_free}")
            else:
                print(f"  方向 {i}: p={p}, grid={grid_pos}, in_bounds={in_bounds}")

        return

    # 4. 计算凸包
    from semantic_perception.polyhedron_expansion import ConvexHullComputer
    hull_computer = ConvexHullComputer()
    hull_result = hull_computer.compute(sample_points)

    if hull_result is None:
        print(f"\n⚠️ 凸包计算失败")
        return

    print(f"\n凸包:")
    print(f"  顶点数: {len(hull_result['vertices'])}")
    print(f"  面片数: {len(hull_result['faces'])}")
    print(f"  体积: {hull_result['volume']:.4f}m³")
    print(f"  中心: {hull_result['center']}")
    print(f"  半径: {hull_result['radius']:.4f}m")

    if hull_result["volume"] < config.min_polyhedron_volume:
        print(f"\n⚠️ 体积 ({hull_result['volume']:.4f}m³) < 最小体积 ({config.min_polyhedron_volume}m³)")
        return

    # 5. 碰撞检测
    from semantic_perception.polyhedron_expansion import CollisionChecker
    collision_checker = CollisionChecker()
    has_collision = collision_checker.check_collision(
        polyhedron_vertices=hull_result["vertices"],
        occupancy_grid=occupancy_grid,
        grid_resolution=grid_resolution,
        grid_origin=grid_origin,
        num_samples=config.collision_check_samples,
        threshold=config.collision_threshold,
    )

    print(f"\n碰撞检测:")
    print(f"  碰撞: {has_collision}")

    if has_collision:
        print(f"\n⚠️ 检测到碰撞")
        return

    print(f"\n✓ 成功生成一个多面体!")


if __name__ == "__main__":
    debug_polyhedron_expansion()
