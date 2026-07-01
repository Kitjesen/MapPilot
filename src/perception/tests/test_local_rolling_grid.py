#!/usr/bin/env python3
"""
局部滚动栅格测试脚本

测试内容:
1. 基本功能测试
2. 栅格滚动测试
3. 占据更新测试
4. 坐标转换测试
5. 内存占用测试
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.local_rolling_grid import (
    LocalRollingGrid,
    create_mock_point_cloud,
)


def test_basic_functionality():
    """测试基本功能。"""
    print("=" * 60)
    print("测试 1: 基本功能")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    print(f"栅格大小: {grid.grid_size}")
    print(f"分辨率: {grid.resolution}m")
    print(f"中心: {grid.center}")

    # 获取占据栅格
    occupancy = grid.get_occupancy_grid()
    print(f"占据栅格形状: {occupancy.shape}")
    print(f"初始占据概率: {occupancy[0, 0, 0]:.2f}")

    assert occupancy.shape == grid.grid_size
    assert np.allclose(occupancy, 0.5), "初始应该全部为未知（0.5）"

    print("\n✓ 基本功能测试通过")
    print()


def test_grid_rolling():
    """测试栅格滚动。"""
    print("=" * 60)
    print("测试 2: 栅格滚动")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1, roll_threshold=2.0)

    # 初始位置
    robot_pos = np.array([0.0, 0.0, 0.0])
    grid.update(robot_pos)

    print(f"初始中心: {grid.center}")
    print(f"滚动次数: {grid.roll_count}")

    # 移动机器人（触发滚动）
    robot_pos = np.array([3.0, 0.0, 0.0])
    grid.update(robot_pos)

    print(f"\n移动后中心: {grid.center}")
    print(f"滚动次数: {grid.roll_count}")

    assert grid.roll_count > 0, "应该触发滚动"

    print("\n✓ 栅格滚动测试通过")
    print()


def test_occupancy_update():
    """测试占据更新。"""
    print("=" * 60)
    print("测试 3: 占据更新")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    # 机器人位置
    robot_pos = np.array([0.0, 0.0, 0.0])

    # 创建模拟点云
    point_cloud = create_mock_point_cloud(robot_pos, num_points=100, max_range=3.0)

    print(f"点云数量: {len(point_cloud)}")

    # 更新占据栅格
    grid.update(robot_pos, point_cloud)

    # 获取统计信息
    stats = grid.get_statistics()
    print("\n统计信息:")
    print(f"  更新次数: {stats['update_count']}")
    print(f"  占据单元格数: {stats['occupied_cells']}")
    print(f"  自由单元格数: {stats['free_cells']}")
    print(f"  未知单元格数: {stats['unknown_cells']}")

    assert stats['update_count'] > 0
    assert stats['occupied_cells'] > 0

    print("\n✓ 占据更新测试通过")
    print()


def test_coordinate_conversion():
    """测试坐标转换。"""
    print("=" * 60)
    print("测试 4: 坐标转换")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    # 测试世界坐标 → 栅格坐标 → 世界坐标
    world_pos = np.array([1.0, 2.0, 1.5])

    grid_pos = grid.world_to_grid(world_pos)
    print(f"世界坐标: {world_pos}")
    print(f"栅格坐标: {grid_pos}")

    if grid_pos is not None:
        world_pos_back = grid.grid_to_world(grid_pos)
        print(f"转换回世界坐标: {world_pos_back}")

        # 检查误差（应该在一个栅格单元内）
        error = np.linalg.norm(world_pos - world_pos_back)
        print(f"转换误差: {error:.4f}m")

        assert error < grid.resolution, "转换误差应该小于分辨率"
    else:
        print("⚠️ 点超出栅格边界")

    print("\n✓ 坐标转换测试通过")
    print()


def test_memory_usage():
    """测试内存占用。"""
    print("=" * 60)
    print("测试 5: 内存占用")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    # 计算内存占用
    occupancy_size = grid.occupancy.nbytes
    total_cells = np.prod(grid.grid_size)

    print(f"栅格尺寸: {grid.grid_size}")
    print(f"总单元格数: {total_cells}")
    print(f"占据栅格内存: {occupancy_size / 1024:.2f} KB")
    print(f"每个单元格: {occupancy_size / total_cells:.2f} bytes")

    # 模拟多次更新（内存不应增长）
    robot_pos = np.array([0.0, 0.0, 0.0])

    for _i in range(10):
        robot_pos += np.array([0.5, 0.0, 0.0])
        point_cloud = create_mock_point_cloud(robot_pos, num_points=100)
        grid.update(robot_pos, point_cloud)

    # 检查内存是否增长
    occupancy_size_after = grid.occupancy.nbytes

    print("\n10 次更新后:")
    print(f"  占据栅格内存: {occupancy_size_after / 1024:.2f} KB")
    print(f"  内存增长: {(occupancy_size_after - occupancy_size) / 1024:.2f} KB")

    assert occupancy_size_after == occupancy_size, "内存不应增长"

    print("\n✓ 内存占用测试通过")
    print()


def test_binary_occupancy():
    """测试二值占据栅格。"""
    print("=" * 60)
    print("测试 6: 二值占据栅格")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    # 更新一些占据
    robot_pos = np.array([0.0, 0.0, 0.0])
    point_cloud = create_mock_point_cloud(robot_pos, num_points=100)
    grid.update(robot_pos, point_cloud)

    # 获取二值占据栅格
    binary = grid.get_binary_occupancy(threshold=0.5)

    print(f"二值栅格形状: {binary.shape}")
    print(f"占据单元格数: {np.sum(binary == 1)}")
    print(f"自由单元格数: {np.sum(binary == 0)}")

    assert binary.shape == grid.grid_size
    assert np.all((binary == 0) | (binary == 1)), "应该只有 0 和 1"

    print("\n✓ 二值占据栅格测试通过")
    print()


def test_reset():
    """测试重置功能。"""
    print("=" * 60)
    print("测试 7: 重置功能")
    print("=" * 60)

    grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)

    # 更新一些数据
    robot_pos = np.array([5.0, 5.0, 0.0])
    point_cloud = create_mock_point_cloud(robot_pos, num_points=100)
    grid.update(robot_pos, point_cloud)

    print("更新前:")
    print(f"  中心: {grid.center}")
    print(f"  更新次数: {grid.update_count}")
    print(f"  滚动次数: {grid.roll_count}")

    # 重置
    grid.reset()

    print("\n重置后:")
    print(f"  中心: {grid.center}")
    print(f"  更新次数: {grid.update_count}")
    print(f"  滚动次数: {grid.roll_count}")

    assert np.allclose(grid.center, [0, 0, 0])
    assert grid.update_count == 0
    assert grid.roll_count == 0
    assert np.allclose(grid.occupancy, 0.5)

    print("\n✓ 重置功能测试通过")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("局部滚动栅格测试套件")
    print("=" * 60 + "\n")

    try:
        test_basic_functionality()
        test_grid_rolling()
        test_occupancy_update()
        test_coordinate_conversion()
        test_memory_usage()
        test_binary_occupancy()
        test_reset()

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
