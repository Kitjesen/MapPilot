"""
数据集加载器 (Dataset Loader) — 统一的数据集加载接口。

支持的数据集:
  1. HM3D (Habitat-Matterport 3D)
  2. Gibson
  3. Replica
  4. 自定义数据集

功能:
  - 统一的数据加载接口
  - 场景元数据管理
  - 点云/深度图加载
  - 相机轨迹加载
  - 数据预处理

设计原则:
  - 统一接口: 所有数据集使用相同的 API
  - 懒加载: 按需加载数据
  - 缓存: 避免重复加载

参考:
  - HM3D: https://aihabitat.org/datasets/hm3d/
  - Gibson: http://gibsonenv.stanford.edu/
"""

import json
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class SceneMetadata:
    """场景元数据。"""
    scene_id: str
    dataset: str  # "hm3d" | "gibson" | "replica" | "custom"
    scene_path: Path
    num_frames: int
    bounds: np.ndarray  # (2, 3) [min, max]
    floor_height: float
    ceiling_height: float
    metadata: Dict


@dataclass
class Frame:
    """单帧数据。"""
    frame_id: int
    timestamp: float

    # 相机位姿 (世界坐标系)
    camera_pose: np.ndarray  # (4, 4) 变换矩阵

    # 传感器数据
    rgb: Optional[np.ndarray] = None  # (H, W, 3)
    depth: Optional[np.ndarray] = None  # (H, W)
    semantic: Optional[np.ndarray] = None  # (H, W)

    # 点云 (世界坐标系)
    point_cloud: Optional[np.ndarray] = None  # (N, 3)
    point_colors: Optional[np.ndarray] = None  # (N, 3)

    # 相机内参
    intrinsics: Optional[np.ndarray] = None  # (3, 3)


# ══════════════════════════════════════════════════════════════════
#  基础数据集加载器
# ══════════════════════════════════════════════════════════════════

class BaseDatasetLoader(ABC):
    """
    基础数据集加载器 — 所有数据集加载器的基类。

    子类需要实现:
      - load_scene_metadata()
      - load_frame()
      - list_scenes()
    """

    def __init__(self, dataset_root: Path):
        """
        Args:
            dataset_root: 数据集根目录
        """
        self.dataset_root = Path(dataset_root)
        self._scene_cache: Dict[str, SceneMetadata] = {}

    @abstractmethod
    def load_scene_metadata(self, scene_id: str) -> SceneMetadata:
        """
        加载场景元数据。

        Args:
            scene_id: 场景 ID

        Returns:
            SceneMetadata 对象
        """
        pass

    @abstractmethod
    def load_frame(
        self,
        scene_id: str,
        frame_id: int,
        load_rgb: bool = False,
        load_depth: bool = True,
        load_semantic: bool = False,
        load_point_cloud: bool = True,
    ) -> Frame:
        """
        加载单帧数据。

        Args:
            scene_id: 场景 ID
            frame_id: 帧 ID
            load_rgb: 是否加载 RGB 图像
            load_depth: 是否加载深度图
            load_semantic: 是否加载语义分割
            load_point_cloud: 是否加载点云

        Returns:
            Frame 对象
        """
        pass

    @abstractmethod
    def list_scenes(self) -> List[str]:
        """
        列出所有场景。

        Returns:
            场景 ID 列表
        """
        pass

    def load_trajectory(
        self,
        scene_id: str,
        frame_ids: Optional[List[int]] = None,
        **kwargs,
    ) -> List[Frame]:
        """
        加载轨迹（多帧）。

        Args:
            scene_id: 场景 ID
            frame_ids: 帧 ID 列表（None = 所有帧）
            **kwargs: 传递给 load_frame 的参数

        Returns:
            Frame 列表
        """
        metadata = self.load_scene_metadata(scene_id)

        if frame_ids is None:
            frame_ids = list(range(metadata.num_frames))

        frames = []
        for frame_id in frame_ids:
            frame = self.load_frame(scene_id, frame_id, **kwargs)
            frames.append(frame)

        logger.info(f"Loaded {len(frames)} frames from scene {scene_id}")
        return frames


# ══════════════════════════════════════════════════════════════════
#  HM3D 数据集加载器
# ══════════════════════════════════════════════════════════════════

class HM3DLoader(BaseDatasetLoader):
    """
    HM3D 数据集加载器。

    数据集结构:
      hm3d/
        train/
          00800-TEEsavR23oF/
            TEEsavR23oF.basis.glb
            TEEsavR23oF.semantic.glb
            ...
        val/
        test/

    用法:
        loader = HM3DLoader("/path/to/hm3d")
        scenes = loader.list_scenes()
        metadata = loader.load_scene_metadata(scenes[0])
        frame = loader.load_frame(scenes[0], 0)
    """

    def __init__(self, dataset_root: Path, split: str = "train"):
        """
        Args:
            dataset_root: HM3D 数据集根目录
            split: 数据集划分 ("train" | "val" | "test")
        """
        super().__init__(dataset_root)
        self.split = split
        self.split_dir = self.dataset_root / split

        if not self.split_dir.exists():
            raise FileNotFoundError(f"HM3D split directory not found: {self.split_dir}")

    def load_scene_metadata(self, scene_id: str) -> SceneMetadata:
        """加载 HM3D 场景元数据。"""
        if scene_id in self._scene_cache:
            return self._scene_cache[scene_id]

        scene_path = self.split_dir / scene_id

        if not scene_path.exists():
            raise FileNotFoundError(f"Scene not found: {scene_path}")

        # 读取场景信息（如果有 metadata.json）
        metadata_file = scene_path / "metadata.json"
        if metadata_file.exists():
            with open(metadata_file) as f:
                metadata_dict = json.load(f)
        else:
            metadata_dict = {}

        # 创建元数据
        metadata = SceneMetadata(
            scene_id=scene_id,
            dataset="hm3d",
            scene_path=scene_path,
            num_frames=metadata_dict.get("num_frames", 0),
            bounds=np.array(metadata_dict.get("bounds", [[-10, -10, 0], [10, 10, 3]])),
            floor_height=metadata_dict.get("floor_height", 0.0),
            ceiling_height=metadata_dict.get("ceiling_height", 3.0),
            metadata=metadata_dict,
        )

        self._scene_cache[scene_id] = metadata
        return metadata

    def load_frame(
        self,
        scene_id: str,
        frame_id: int,
        load_rgb: bool = False,
        load_depth: bool = True,
        load_semantic: bool = False,
        load_point_cloud: bool = True,
    ) -> Frame:
        """加载 HM3D 单帧数据。"""
        scene_path = self.split_dir / scene_id

        # 加载相机位姿
        pose_file = scene_path / f"poses/{frame_id:06d}.txt"
        if pose_file.exists():
            camera_pose = np.loadtxt(pose_file).reshape(4, 4)
        else:
            # 默认位姿
            camera_pose = np.eye(4)

        frame = Frame(
            frame_id=frame_id,
            timestamp=frame_id * 0.1,  # 假设 10 FPS
            camera_pose=camera_pose,
        )

        # 加载 RGB
        if load_rgb:
            rgb_file = scene_path / f"rgb/{frame_id:06d}.png"
            if rgb_file.exists():
                import cv2
                frame.rgb = cv2.imread(str(rgb_file))
                frame.rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2RGB)

        # 加载深度
        if load_depth:
            depth_file = scene_path / f"depth/{frame_id:06d}.png"
            if depth_file.exists():
                import cv2
                frame.depth = cv2.imread(str(depth_file), cv2.IMREAD_ANYDEPTH)
                frame.depth = frame.depth.astype(np.float32) / 1000.0  # mm -> m

        # 加载语义
        if load_semantic:
            semantic_file = scene_path / f"semantic/{frame_id:06d}.png"
            if semantic_file.exists():
                import cv2
                frame.semantic = cv2.imread(str(semantic_file), cv2.IMREAD_GRAYSCALE)

        # 加载点云（从深度图生成）
        if load_point_cloud and frame.depth is not None:
            frame.point_cloud = self._depth_to_point_cloud(
                frame.depth,
                frame.camera_pose,
                frame.intrinsics,
            )

        return frame

    def list_scenes(self) -> List[str]:
        """列出 HM3D 所有场景。"""
        scenes = []
        for scene_dir in self.split_dir.iterdir():
            if scene_dir.is_dir():
                scenes.append(scene_dir.name)
        return sorted(scenes)

    @staticmethod
    def _depth_to_point_cloud(
        depth: np.ndarray,
        camera_pose: np.ndarray,
        intrinsics: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        从深度图生成点云。

        Args:
            depth: (H, W) 深度图
            camera_pose: (4, 4) 相机位姿
            intrinsics: (3, 3) 相机内参（None = 默认）

        Returns:
            (N, 3) 点云
        """
        H, W = depth.shape

        # 默认内参（假设 FOV=90°）
        if intrinsics is None:
            fx = fy = W / 2.0
            cx, cy = W / 2.0, H / 2.0
        else:
            fx, fy = intrinsics[0, 0], intrinsics[1, 1]
            cx, cy = intrinsics[0, 2], intrinsics[1, 2]

        # 生成像素坐标网格
        u, v = np.meshgrid(np.arange(W), np.arange(H))

        # 反投影到相机坐标系
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 组合为点云 (相机坐标系)
        points_cam = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        # 过滤无效点
        valid = (points_cam[:, 2] > 0) & (points_cam[:, 2] < 10.0)
        points_cam = points_cam[valid]

        # 转换到世界坐标系
        points_world = (camera_pose[:3, :3] @ points_cam.T).T + camera_pose[:3, 3]

        return points_world


# ══════════════════════════════════════════════════════════════════
#  Gibson 数据集加载器
# ══════════════════════════════════════════════════════════════════

class GibsonLoader(BaseDatasetLoader):
    """
    Gibson 数据集加载器。

    数据集结构:
      gibson/
        Allensville/
          mesh_z_up.obj
          pano/
            rgb/
            depth/
        ...

    用法:
        loader = GibsonLoader("/path/to/gibson")
        scenes = loader.list_scenes()
        frame = loader.load_frame(scenes[0], 0)
    """

    def __init__(self, dataset_root: Path):
        """
        Args:
            dataset_root: Gibson 数据集根目录
        """
        super().__init__(dataset_root)

    def load_scene_metadata(self, scene_id: str) -> SceneMetadata:
        """加载 Gibson 场景元数据。"""
        if scene_id in self._scene_cache:
            return self._scene_cache[scene_id]

        scene_path = self.dataset_root / scene_id

        if not scene_path.exists():
            raise FileNotFoundError(f"Scene not found: {scene_path}")

        # Gibson 场景信息
        metadata = SceneMetadata(
            scene_id=scene_id,
            dataset="gibson",
            scene_path=scene_path,
            num_frames=0,  # 需要扫描目录
            bounds=np.array([[-20, -20, 0], [20, 20, 3]]),
            floor_height=0.0,
            ceiling_height=3.0,
            metadata={},
        )

        self._scene_cache[scene_id] = metadata
        return metadata

    def load_frame(
        self,
        scene_id: str,
        frame_id: int,
        load_rgb: bool = False,
        load_depth: bool = True,
        load_semantic: bool = False,
        load_point_cloud: bool = True,
    ) -> Frame:
        """加载 Gibson 单帧数据。"""
        scene_path = self.dataset_root / scene_id

        # Gibson 使用全景图，这里简化处理
        frame = Frame(
            frame_id=frame_id,
            timestamp=frame_id * 0.1,
            camera_pose=np.eye(4),
        )

        # 实际实现需要处理全景图
        logger.warning("Gibson loader not fully implemented yet")

        return frame

    def list_scenes(self) -> List[str]:
        """列出 Gibson 所有场景。"""
        scenes = []
        for scene_dir in self.dataset_root.iterdir():
            if scene_dir.is_dir() and (scene_dir / "mesh_z_up.obj").exists():
                scenes.append(scene_dir.name)
        return sorted(scenes)


# ══════════════════════════════════════════════════════════════════
#  数据集工厂
# ══════════════════════════════════════════════════════════════════

def create_dataset_loader(
    dataset_type: str,
    dataset_root: Path,
    **kwargs,
) -> BaseDatasetLoader:
    """
    创建数据集加载器。

    Args:
        dataset_type: 数据集类型 ("hm3d" | "gibson" | "replica")
        dataset_root: 数据集根目录
        **kwargs: 额外参数

    Returns:
        数据集加载器实例
    """
    if dataset_type == "hm3d":
        return HM3DLoader(dataset_root, **kwargs)
    elif dataset_type == "gibson":
        return GibsonLoader(dataset_root, **kwargs)
    else:
        raise ValueError(f"Unknown dataset type: {dataset_type}")
