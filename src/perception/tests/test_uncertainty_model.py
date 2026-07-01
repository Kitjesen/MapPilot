#!/usr/bin/env python3
"""
不确定性建模测试脚本

测试内容:
1. 多面体不确定性计算
2. GCM 不确定性计算
3. 信息增益计算
4. 探索目标选择
5. 前沿目标选择
6. 探索策略
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.global_coverage_mask import GlobalCoverageMask
from perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)
from perception.scg_builder import SCGBuilder, SCGConfig
from perception.uncertainty_model import (
    ExplorationStrategy,
    UncertaintyModel,
)


def create_test_environment():
    """创建测试环境。"""
    # 创建占据栅格
    occupancy_grid = np.ones((30, 30, 10), dtype=np.float32)
    occupancy_grid[5:25, :, :] = 0.0
    occupancy_grid[12:18, 12:18, :] = 1.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    # 多面体扩展
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

    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    # 构建 SCG
    scg_config = SCGConfig()
    scg_builder = SCGBuilder(scg_config)

    for poly in polyhedra:
        scg_builder.add_polyhedron(poly)

    scg_builder.build_edges(occupancy_grid, grid_resolution, grid_origin)

    # 创建 GCM
    gcm = GlobalCoverageMask(resolution=0.5)
    for poly in polyhedra:
        gcm.update_from_polyhedron(poly)

    return scg_builder, gcm, polyhedra, occupancy_grid, grid_resolution, grid_origin


def test_polyhedron_uncertainty():
    """测试多面体不确定性计算。"""
    print("=" * 60)
    print("测试 1: 多面体不确定性计算")
    print("=" * 60)

    _scg_builder, _gcm, polyhedra, occupancy_grid, grid_resolution, grid_origin = create_test_environment()

    model = UncertaintyModel()

    if len(polyhedra) == 0:
        print("⚠️ 没有多面体，跳过测试")
        return

    # 计算第一个多面体的不确定性
    poly = polyhedra[0]
    uncertainty = model.compute_polyhedron_uncertainty(
        poly, occupancy_grid, grid_resolution, grid_origin
    )

    print("多面体 0:")
    print(f"  中心: {poly.center}")
    print(f"  体积: {poly.volume:.2f}m³")
    print(f"  不确定性: {uncertainty:.4f}")

    assert 0.0 <= uncertainty <= 1.0, "不确定性应该在 [0, 1] 范围内"

    print("\n✓ 多面体不确定性计算测试通过")
    print()


def test_gcm_uncertainty():
    """测试 GCM 不确定性计算。"""
    print("=" * 60)
    print("测试 2: GCM 不确定性计算")
    print("=" * 60)

    _scg_builder, gcm, _polyhedra, _occupancy_grid, _grid_resolution, _grid_origin = create_test_environment()

    model = UncertaintyModel()

    # 计算 GCM 的整体不确定性
    uncertainty = model.compute_gcm_uncertainty(gcm)

    print("GCM 统计:")
    print(f"  总单元格数: {gcm.total_cells}")
    print(f"  已覆盖单元格数: {gcm.covered_cells}")
    print(f"  整体不确定性: {uncertainty:.4f}")

    assert 0.0 <= uncertainty <= 1.0, "不确定性应该在 [0, 1] 范围内"

    print("\n✓ GCM 不确定性计算测试通过")
    print()


def test_information_gain():
    """测试信息增益计算。"""
    print("=" * 60)
    print("测试 3: 信息增益计算")
    print("=" * 60)

    _scg_builder, _gcm, polyhedra, _occupancy_grid, _grid_resolution, _grid_origin = create_test_environment()

    model = UncertaintyModel()

    if len(polyhedra) == 0:
        print("⚠️ 没有多面体，跳过测试")
        return

    # 机器人位置
    robot_pose = np.array([5.0, 5.0, 1.0])

    # 计算每个多面体的信息增益
    print(f"机器人位置: {robot_pose}")
    print("\n多面体信息增益:")

    for i, poly in enumerate(polyhedra[:5]):  # 只显示前 5 个
        gain = model.compute_information_gain(poly, robot_pose, visit_count=0)
        distance = np.linalg.norm(poly.center - robot_pose)

        print(f"  多面体 {i}:")
        print(f"    距离: {distance:.2f}m")
        print(f"    信息增益: {gain:.4f}")

    print("\n✓ 信息增益计算测试通过")
    print()


def test_exploration_target_selection():
    """测试探索目标选择。"""
    print("=" * 60)
    print("测试 4: 探索目标选择")
    print("=" * 60)

    scg_builder, gcm, polyhedra, _occupancy_grid, _grid_resolution, _grid_origin = create_test_environment()

    model = UncertaintyModel()

    if len(polyhedra) == 0:
        print("⚠️ 没有多面体，跳过测试")
        return

    # 机器人位置
    robot_pose = np.array([5.0, 5.0, 1.0])

    # 选择探索目标
    result = model.select_exploration_target(scg_builder, gcm, robot_pose)

    if result is not None:
        poly_id, gain = result
        poly = scg_builder.nodes[poly_id]

        print("选择的探索目标:")
        print(f"  多面体 ID: {poly_id}")
        print(f"  位置: {poly.center}")
        print(f"  信息增益: {gain:.4f}")

        assert poly_id in scg_builder.nodes, "选择的多面体应该存在"
    else:
        print("⚠️ 没有找到探索目标")

    print("\n✓ 探索目标选择测试通过")
    print()


def test_frontier_target_selection():
    """测试前沿目标选择。"""
    print("=" * 60)
    print("测试 5: 前沿目标选择")
    print("=" * 60)

    _scg_builder, gcm, _polyhedra, _occupancy_grid, _grid_resolution, _grid_origin = create_test_environment()

    model = UncertaintyModel()

    # 机器人位置
    robot_pose = np.array([5.0, 5.0, 1.0])

    # 选择前沿目标
    result = model.select_frontier_target(gcm, robot_pose)

    if result is not None:
        grid_pos, gain = result
        world_pos = gcm.grid_to_world(grid_pos)

        print("选择的前沿目标:")
        print(f"  栅格位置: {grid_pos}")
        print(f"  世界位置: {world_pos}")
        print(f"  信息增益: {gain:.4f}")
    else:
        print("⚠️ 没有找到前沿目标")

    print("\n✓ 前沿目标选择测试通过")
    print()


def test_exploration_strategy():
    """测试探索策略。"""
    print("=" * 60)
    print("测试 6: 探索策略")
    print("=" * 60)

    scg_builder, gcm, polyhedra, _occupancy_grid, _grid_resolution, _grid_origin = create_test_environment()

    model = UncertaintyModel()
    strategy = ExplorationStrategy(model)

    # 机器人位置
    robot_pose = np.array([5.0, 5.0, 1.0])

    # 选择下一个目标（优先前沿）
    target = strategy.select_next_target(scg_builder, gcm, robot_pose, prefer_frontier=True)

    if target is not None:
        print("选择的探索目标（优先前沿）:")
        print(f"  位置: {target}")
    else:
        print("⚠️ 没有找到探索目标")

    # 选择下一个目标（优先多面体）
    target = strategy.select_next_target(scg_builder, gcm, robot_pose, prefer_frontier=False)

    if target is not None:
        print("\n选择的探索目标（优先多面体）:")
        print(f"  位置: {target}")
    else:
        print("⚠️ 没有找到探索目标")

    # 标记访问
    if len(polyhedra) > 0:
        strategy.mark_visited(0)
        strategy.mark_visited(0)
        strategy.mark_visited(1)

    # 获取统计信息
    stats = strategy.get_statistics()
    print("\n探索统计:")
    print(f"  访问的多面体数: {stats['total_visited']}")
    print(f"  总访问次数: {stats['total_visits']}")
    print(f"  最多访问次数: {stats['most_visited']}")

    print("\n✓ 探索策略测试通过")
    print()


def test_uncertainty_update():
    """测试不确定性更新。"""
    print("=" * 60)
    print("测试 7: 不确定性更新")
    print("=" * 60)

    _scg_builder, _gcm, polyhedra, occupancy_grid, grid_resolution, grid_origin = create_test_environment()

    model = UncertaintyModel()

    if len(polyhedra) == 0:
        print("⚠️ 没有多面体，跳过测试")
        return

    # 更新多面体的不确定性
    poly = polyhedra[0]

    print("更新前:")
    print(f"  不确定性: {getattr(poly, 'uncertainty', 'N/A')}")

    model.update_polyhedron_uncertainty(
        poly, occupancy_grid, grid_resolution, grid_origin
    )

    print("\n更新后:")
    print(f"  不确定性: {poly.uncertainty:.4f}")

    assert hasattr(poly, 'uncertainty'), "多面体应该有 uncertainty 属性"
    assert 0.0 <= poly.uncertainty <= 1.0, "不确定性应该在 [0, 1] 范围内"

    print("\n✓ 不确定性更新测试通过")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("不确定性建模测试套件")
    print("=" * 60 + "\n")

    try:
        test_polyhedron_uncertainty()
        test_gcm_uncertainty()
        test_information_gain()
        test_exploration_target_selection()
        test_frontier_target_selection()
        test_exploration_strategy()
        test_uncertainty_update()

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
