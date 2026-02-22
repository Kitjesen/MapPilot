#!/usr/bin/env python3
"""
数据集加载器测试脚本

测试内容:
1. HM3D 加载器基本功能
2. Gibson 加载器基本功能
3. 场景元数据加载
4. 帧数据加载
5. 轨迹加载
6. 点云生成
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from semantic_perception.dataset_loader import (
    HM3DLoader,
    GibsonLoader,
    create_dataset_loader,
    SceneMetadata,
    Frame,
)


def create_mock_hm3d_dataset(root_dir: Path):
    """创建模拟 HM3D 数据集。"""
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
    import json
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

    print(f"✓ 创建模拟 HM3D 数据集: {scene_dir}")


def test_hm3d_loader():
    """测试 HM3D 加载器。"""
    print("=" * 60)
    print("测试 1: HM3D 加载器")
    print("=" * 60)

    # 创建临时数据集
    import tempfile
    temp_dir = Path(tempfile.mkdtemp())
    create_mock_hm3d_dataset(temp_dir)

    try:
        # 创建加载器
        loader = HM3DLoader(temp_dir, split="train")

        # 列出场景
        scenes = loader.list_scenes()
        print(f"场景数量: {len(scenes)}")
        print(f"场景列表: {scenes}")

        assert len(scenes) > 0, "应该至少有一个场景"

        # 加载场景元数据
        scene_id = scenes[0]
        metadata = loader.load_scene_metadata(scene_id)

        print(f"\n场景元数据:")
        print(f"  场景 ID: {metadata.scene_id}")
        print(f"  数据集: {metadata.dataset}")
        print(f"  帧数: {metadata.num_frames}")
        print(f"  边界: {metadata.bounds}")

        assert metadata.scene_id == scene_id
        assert metadata.dataset == "hm3d"
        assert metadata.num_frames == 10

        # 加载单帧
        frame = loader.load_frame(
            scene_id,
            0,
            load_rgb=True,
            load_depth=True,
            load_point_cloud=True,
        )

        print(f"\n帧数据:")
        print(f"  帧 ID: {frame.frame_id}")
        print(f"  时间戳: {frame.timestamp}")
        print(f"  相机位姿:\n{frame.camera_pose}")
        if frame.rgb is not None:
            print(f"  RGB 形状: {frame.rgb.shape}")
        if frame.depth is not None:
            print(f"  深度形状: {frame.depth.shape}")
        if frame.point_cloud is not None:
            print(f"  点云数量: {len(frame.point_cloud)}")

        assert frame.frame_id == 0
        assert frame.rgb is not None
        assert frame.depth is not None
        assert frame.point_cloud is not None

        print("\n✓ HM3D 加载器测试通过")
        print()

    finally:
        # 清理临时文件
        import shutil
        shutil.rmtree(temp_dir)


def test_trajectory_loading():
    """测试轨迹加载。"""
    print("=" * 60)
    print("测试 2: 轨迹加载")
    print("=" * 60)

    # 创建临时数据集
    import tempfile
    temp_dir = Path(tempfile.mkdtemp())
    create_mock_hm3d_dataset(temp_dir)

    try:
        loader = HM3DLoader(temp_dir, split="train")
        scenes = loader.list_scenes()
        scene_id = scenes[0]

        # 加载轨迹
        frames = loader.load_trajectory(
            scene_id,
            frame_ids=[0, 2, 4, 6, 8],
            load_depth=True,
            load_point_cloud=True,
        )

        print(f"加载的帧数: {len(frames)}")

        for i, frame in enumerate(frames):
            print(f"  帧 {i}: ID={frame.frame_id}, 点云={len(frame.point_cloud) if frame.point_cloud is not None else 0}")

        assert len(frames) == 5
        assert frames[0].frame_id == 0
        assert frames[-1].frame_id == 8

        print("\n✓ 轨迹加载测试通过")
        print()

    finally:
        import shutil
        shutil.rmtree(temp_dir)


def test_point_cloud_generation():
    """测试点云生成。"""
    print("=" * 60)
    print("测试 3: 点云生成")
    print("=" * 60)

    # 创建模拟深度图
    depth = np.ones((480, 640), dtype=np.float32) * 2.0  # 2m 深度
    camera_pose = np.eye(4)
    camera_pose[:3, 3] = [1, 2, 0]  # 相机位置

    # 生成点云
    point_cloud = HM3DLoader._depth_to_point_cloud(depth, camera_pose)

    print(f"深度图形状: {depth.shape}")
    print(f"点云数量: {len(point_cloud)}")
    print(f"点云范围:")
    print(f"  X: [{point_cloud[:, 0].min():.2f}, {point_cloud[:, 0].max():.2f}]")
    print(f"  Y: [{point_cloud[:, 1].min():.2f}, {point_cloud[:, 1].max():.2f}]")
    print(f"  Z: [{point_cloud[:, 2].min():.2f}, {point_cloud[:, 2].max():.2f}]")

    assert len(point_cloud) > 0
    assert point_cloud.shape[1] == 3

    print("\n✓ 点云生成测试通过")
    print()


def test_dataset_factory():
    """测试数据集工厂。"""
    print("=" * 60)
    print("测试 4: 数据集工厂")
    print("=" * 60)

    # 创建临时数据集
    import tempfile
    temp_dir = Path(tempfile.mkdtemp())
    create_mock_hm3d_dataset(temp_dir)

    try:
        # 使用工厂创建加载器
        loader = create_dataset_loader("hm3d", temp_dir, split="train")

        print(f"加载器类型: {type(loader).__name__}")
        assert isinstance(loader, HM3DLoader)

        scenes = loader.list_scenes()
        print(f"场景数量: {len(scenes)}")

        print("\n✓ 数据集工厂测试通过")
        print()

    finally:
        import shutil
        shutil.rmtree(temp_dir)


def test_gibson_loader():
    """测试 Gibson 加载器。"""
    print("=" * 60)
    print("测试 5: Gibson 加载器")
    print("=" * 60)

    # 创建临时 Gibson 数据集
    import tempfile
    temp_dir = Path(tempfile.mkdtemp())

    scene_dir = temp_dir / "Allensville"
    scene_dir.mkdir(parents=True, exist_ok=True)

    # 创建 mesh 文件（空文件）
    (scene_dir / "mesh_z_up.obj").touch()

    try:
        loader = GibsonLoader(temp_dir)

        scenes = loader.list_scenes()
        print(f"场景数量: {len(scenes)}")
        print(f"场景列表: {scenes}")

        assert len(scenes) > 0

        # 加载元数据
        scene_id = scenes[0]
        metadata = loader.load_scene_metadata(scene_id)

        print(f"\n场景元数据:")
        print(f"  场景 ID: {metadata.scene_id}")
        print(f"  数据集: {metadata.dataset}")

        assert metadata.dataset == "gibson"

        print("\n✓ Gibson 加载器测试通过")
        print()

    finally:
        import shutil
        shutil.rmtree(temp_dir)


def test_scene_metadata():
    """测试场景元数据。"""
    print("=" * 60)
    print("测试 6: 场景元数据")
    print("=" * 60)

    metadata = SceneMetadata(
        scene_id="test_scene",
        dataset="hm3d",
        scene_path=Path("/tmp/test"),
        num_frames=100,
        bounds=np.array([[-10, -10, 0], [10, 10, 3]]),
        floor_height=0.0,
        ceiling_height=3.0,
        metadata={"key": "value"},
    )

    print(f"场景 ID: {metadata.scene_id}")
    print(f"数据集: {metadata.dataset}")
    print(f"帧数: {metadata.num_frames}")
    print(f"边界: {metadata.bounds}")
    print(f"楼层高度: {metadata.floor_height}")
    print(f"天花板高度: {metadata.ceiling_height}")

    assert metadata.scene_id == "test_scene"
    assert metadata.dataset == "hm3d"
    assert metadata.num_frames == 100

    print("\n✓ 场景元数据测试通过")
    print()


def main():
    """运行所有测试。"""
    print("\n" + "=" * 60)
    print("数据集加载器测试套件")
    print("=" * 60 + "\n")

    try:
        test_scene_metadata()
        test_point_cloud_generation()
        test_hm3d_loader()
        test_trajectory_loading()
        test_dataset_factory()
        test_gibson_loader()

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
