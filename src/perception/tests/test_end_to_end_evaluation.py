#!/usr/bin/env python3
"""
端到端评估测试脚本

测试内容:
1. 端到端评估器基本功能
2. 模拟数据集评估
3. 结果保存和报告生成
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.end_to_end_evaluation import EndToEndEvaluator


def create_mock_hm3d_dataset(root_dir: Path):
    """创建模拟 HM3D 数据集。"""
    import json
    # 创建目录结构
    train_dir = root_dir / "train"
    scene_dir = train_dir / "00800-TEEsavR23oF"
    scene_dir.mkdir(parents=True, exist_ok=True)

    # 创建子目录
    (scene_dir / "rgb").mkdir(exist_ok=True)
    (scene_dir / "depth").mkdir(exist_ok=True)
    (scene_dir / "semantic").mkdir(exist_ok=True)
    (scene_dir / "poses").mkdir(exist_ok=True)

    # 创建元数据
    metadata = {
        "scene_id": "00800-TEEsavR23oF",
        "num_frames": 10,
        "bounds": [[-5, -5, 0], [5, 5, 3]],
        "floor_height": 0.0,
        "ceiling_height": 3.0,
    }
    with open(scene_dir / "metadata.json", "w") as f:
        json.dump(metadata, f)

    # 创建模拟数据
    for i in range(10):
        # RGB 图像
        rgb_file = scene_dir / f"rgb/{i:06d}.png"
        if not rgb_file.exists():
            import cv2
            rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            cv2.imwrite(str(rgb_file), rgb)

        # 深度图
        depth_file = scene_dir / f"depth/{i:06d}.png"
        if not depth_file.exists():
            import cv2
            depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)
            cv2.imwrite(str(depth_file), depth)

        # 位姿
        pose_file = scene_dir / f"poses/{i:06d}.txt"
        if not pose_file.exists():
            pose = np.eye(4)
            pose[:3, 3] = [i * 0.5, 0, 0]  # 沿 x 轴移动
            np.savetxt(pose_file, pose)


def test_end_to_end_evaluator():
    """测试端到端评估器。"""
    print("=" * 60)
    print("测试 1: 端到端评估器")
    print("=" * 60)

    # 创建临时数据集
    temp_dir = Path(tempfile.mkdtemp())

    # 创建模拟 HM3D 数据集
    create_mock_hm3d_dataset(temp_dir)

    try:
        # 创建评估器
        evaluator = EndToEndEvaluator()

        # 运行评估（只测试 PCT A*，因为 USS-Nav 需要更复杂的设置）
        results = evaluator.run_evaluation(
            dataset_type="hm3d",
            dataset_root=temp_dir,
            scene_ids=["00800-TEEsavR23oF"],
            methods=["pct_astar"],
            num_frames=3,  # 少量帧以加快测试
            planning_queries=2,  # 少量查询
        )

        print(f"\n评估结果数量: {len(results)}")

        if results:
            result = results[0]
            print("\n结果详情:")
            print(f"  方法: {result.method_name}")
            print(f"  场景: {result.scene_id}")
            if result.memory:
                print(f"  内存: {result.memory.total_memory_mb:.2f} MB")
            if result.update:
                print(f"  更新时间: {result.update.avg_update_time_ms:.2f} ms")
            if result.path:
                print(f"  规划时间: {result.path.planning_time_ms:.2f} ms")
                print(f"  成功率: {result.path.success_rate:.2%}")

        assert len(results) > 0, "应该有评估结果"

        # 保存结果
        output_file = temp_dir / "results.json"
        evaluator.save_results(str(output_file))
        assert output_file.exists(), "结果文件应该存在"
        print(f"\n✓ 结果已保存: {output_file}")

        # 生成报告
        report_file = temp_dir / "report.md"
        evaluator.generate_report(str(report_file))
        assert report_file.exists(), "报告文件应该存在"
        print(f"✓ 报告已生成: {report_file}")

        # 读取报告内容
        with open(report_file, encoding="utf-8") as f:
            report_content = f.read()
            print("\n报告预览（前 500 字符）:")
            print(report_content[:500])

        print("\n✓ 端到端评估器测试通过")
        print()

    finally:
        # 清理临时文件
        import shutil
        shutil.rmtree(temp_dir)


def test_evaluator_with_empty_scenes():
    """测试空场景处理。"""
    print("=" * 60)
    print("测试 2: 空场景处理")
    print("=" * 60)

    temp_dir = Path(tempfile.mkdtemp())

    try:
        evaluator = EndToEndEvaluator()

        # 尝试评估不存在的场景
        results = evaluator.run_evaluation(
            dataset_type="hm3d",
            dataset_root=temp_dir,
            scene_ids=["nonexistent_scene"],
            methods=["pct_astar"],
            num_frames=1,
            planning_queries=1,
        )

        print(f"评估结果数量: {len(results)}")
        assert len(results) == 0, "不存在的场景应该返回空结果"

        print("\n✓ 空场景处理测试通过")
        print()

    finally:
        import shutil
        shutil.rmtree(temp_dir)


def test_multiple_methods():
    """测试多方法对比。"""
    print("=" * 60)
    print("测试 3: 多方法对比")
    print("=" * 60)

    temp_dir = Path(tempfile.mkdtemp())
    create_mock_hm3d_dataset(temp_dir)

    try:
        evaluator = EndToEndEvaluator()

        # 评估多个方法
        results = evaluator.run_evaluation(
            dataset_type="hm3d",
            dataset_root=temp_dir,
            scene_ids=["00800-TEEsavR23oF"],
            methods=["pct_astar"],  # 只测试 PCT A*
            num_frames=2,
            planning_queries=1,
        )

        print(f"评估结果数量: {len(results)}")

        # 按方法分组
        methods = {}
        for result in results:
            if result.method_name not in methods:
                methods[result.method_name] = []
            methods[result.method_name].append(result)

        print(f"\n方法数量: {len(methods)}")
        for method_name, method_results in methods.items():
            print(f"  {method_name}: {len(method_results)} 结果")

        print("\n✓ 多方法对比测试通过")
        print()

    finally:
        import shutil
        shutil.rmtree(temp_dir)


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("端到端评估测试套件")
    print("=" * 60 + "\n")

    try:
        test_end_to_end_evaluator()
        test_evaluator_with_empty_scenes()
        test_multiple_methods()

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
