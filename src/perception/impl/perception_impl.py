"""
感知系统实现 - PerceptionImpl

整合检测器、编码器、追踪器，实现完整的感知流程
"""

import logging
import math
import time

import numpy as np

from runtime.runtime_interface import map_frame_id

from ..api.detector_api import DetectorAPI
from ..api.encoder_api import EncoderAPI
from ..api.exceptions import InvalidDepthError, InvalidImageError, PerceptionAPIError
from ..api.perception_api import PerceptionAPI
from ..api.tracker_api import TrackerAPI
from ..api.types import (
    BBox2D,
    CameraInfo,
    Detection3D,
    PerceptionConfig,
    Position3D,
    Region,
    Relation,
    SceneGraph,
)

logger = logging.getLogger(__name__)
PERCEPTION_IMPL_MAP_FRAME_ID = map_frame_id()


class PerceptionImpl(PerceptionAPI):
    """
    感知系统实现

    整合三个核心组件：
    - DetectorAPI: 2D物体检测
    - EncoderAPI: CLIP特征编码
    - TrackerAPI: 实例追踪

    处理流程：
    1. 2D检测 (DetectorAPI)
    2. CLIP编码 (EncoderAPI)
    3. 3D投影 (内部实现)
    4. 实例追踪 (TrackerAPI)
    5. 场景图构建 (内部实现)
    """

    def __init__(
        self,
        detector: DetectorAPI,
        encoder: EncoderAPI,
        tracker: TrackerAPI,
        config: PerceptionConfig | None = None
    ):
        """
        初始化感知系统

        Args:
            detector: 检测器实例
            encoder: 编码器实例
            tracker: 追踪器实例
            config: 配置对象
        """
        self.detector = detector
        self.encoder = encoder
        self.tracker = tracker
        self.config = config or PerceptionConfig()

        # 场景图状态
        self._scene_graph: SceneGraph | None = None
        self._frame_count = 0

        # 性能统计
        self._total_process_time = 0.0
        self._process_count = 0

        logger.info("PerceptionImpl initialized")

    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
        transform: np.ndarray | None = None
    ) -> list[Detection3D]:
        """
        处理单帧图像

        Args:
            rgb_image: RGB图像 (H, W, 3)
            depth_image: 深度图像 (H, W)
            camera_info: 相机内参
            transform: 相机到世界坐标系的变换矩阵 (4x4)

        Returns:
            3D检测结果列表

        Raises:
            InvalidImageError: 图像格式无效
            InvalidDepthError: 深度图像无效
            PerceptionAPIError: 处理失败
        """
        start_time = time.time()

        try:
            # 1. 验证输入
            self._validate_inputs(rgb_image, depth_image, camera_info)

            # 2. 2D检测
            detections_2d = self.detector.detect(rgb_image)
            logger.debug(f"Detected {len(detections_2d)} objects in 2D")

            if not detections_2d:
                return []

            # 3. CLIP编码（批量）
            detections_3d = []
            for det_2d in detections_2d:
                # 裁剪图像区域
                bbox = det_2d.bbox
                x1, y1 = int(bbox.x1), int(bbox.y1)
                x2, y2 = int(bbox.x2), int(bbox.y2)

                # 边界检查
                h, w = rgb_image.shape[:2]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)

                if x2 <= x1 or y2 <= y1:
                    continue

                cropped = rgb_image[y1:y2, x1:x2]

                # 编码CLIP特征
                try:
                    clip_feature = self.encoder.encode_image(cropped)
                except Exception as e:
                    logger.warning(f"CLIP encoding failed for {det_2d.label}: {e}")
                    clip_feature = None

                # 4. 3D投影
                position_3d = self._project_to_3d(
                    det_2d.bbox,
                    depth_image,
                    camera_info,
                    transform
                )

                if position_3d is None:
                    continue

                # 创建3D检测
                det_3d = Detection3D(
                    id=f"det_{self._frame_count}_{len(detections_3d)}",
                    label=det_2d.label,
                    confidence=det_2d.confidence,
                    bbox_2d=det_2d.bbox,
                    position_3d=position_3d,
                    clip_feature=clip_feature,
                    detection_count=1,
                    last_seen=time.time()
                )

                detections_3d.append(det_3d)

            logger.debug(f"Projected {len(detections_3d)} objects to 3D")

            # 5. 实例追踪
            tracked_objects = self.tracker.update(detections_3d)
            logger.debug(f"Tracked {len(tracked_objects)} objects")

            # 6. 更新场景图
            self._update_scene_graph(tracked_objects)

            # 性能统计
            elapsed = time.time() - start_time
            self._total_process_time += elapsed
            self._process_count += 1
            self._frame_count += 1

            if self._frame_count % 100 == 0:
                avg_time = self._total_process_time / self._process_count
                logger.info(
                    f"Processed {self._frame_count} frames, "
                    f"avg_time={avg_time:.3f}s, "
                    f"fps={1.0/avg_time:.1f}"
                )

            return tracked_objects

        except (InvalidImageError, InvalidDepthError):
            raise
        except Exception as e:
            raise PerceptionAPIError(f"Frame processing failed: {e}") from e

    def _validate_inputs(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: CameraInfo
    ):
        """验证输入"""
        if rgb_image is None or not isinstance(rgb_image, np.ndarray):
            raise InvalidImageError("RGB image must be a numpy array")

        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise InvalidImageError(
                f"RGB image must be HxWx3, got shape {rgb_image.shape}"
            )

        if depth_image is None or not isinstance(depth_image, np.ndarray):
            raise InvalidDepthError("Depth image must be a numpy array")

        if depth_image.ndim != 2:
            raise InvalidDepthError(
                f"Depth image must be HxW, got shape {depth_image.shape}"
            )

        if rgb_image.shape[:2] != depth_image.shape:
            raise InvalidImageError(
                f"RGB and depth image sizes must match: "
                f"rgb={rgb_image.shape[:2]}, depth={depth_image.shape}"
            )

    def _project_to_3d(
        self,
        bbox: BBox2D,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
        transform: np.ndarray | None = None
    ) -> Position3D | None:
        """
        将2D边界框投影到3D

        Args:
            bbox: 2D边界框
            depth_image: 深度图像
            camera_info: 相机内参
            transform: 相机到世界坐标系的变换矩阵

        Returns:
            3D位置，如果投影失败返回None
        """
        # 计算边界框中心
        cx_2d = (bbox.x1 + bbox.x2) / 2
        cy_2d = (bbox.y1 + bbox.y2) / 2

        # 获取深度值（取中心区域的中位数）
        x1, y1 = int(bbox.x1), int(bbox.y1)
        x2, y2 = int(bbox.x2), int(bbox.y2)

        h, w = depth_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        if x2 <= x1 or y2 <= y1:
            return None

        # 取中心区域的深度
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        region_size = 5

        x_start = max(0, center_x - region_size)
        x_end = min(w, center_x + region_size)
        y_start = max(0, center_y - region_size)
        y_end = min(h, center_y + region_size)

        depth_region = depth_image[y_start:y_end, x_start:x_end]

        # 过滤无效深度 (0, NaN, inf)
        valid_mask = (depth_region > 0) & np.isfinite(depth_region)
        valid_depths = depth_region[valid_mask]
        if len(valid_depths) == 0:
            return None

        depth = float(np.median(valid_depths)) * camera_info.depth_scale
        if not math.isfinite(depth):
            return None

        # 深度范围检查
        if depth < self.config.min_depth or depth > self.config.max_depth:
            return None

        # 反投影到3D（相机坐标系）
        x_cam = (cx_2d - camera_info.cx) * depth / camera_info.fx
        y_cam = (cy_2d - camera_info.cy) * depth / camera_info.fy
        z_cam = depth

        # 转换到世界坐标系
        if transform is not None:
            point_cam = np.array([x_cam, y_cam, z_cam, 1.0])
            point_world = transform @ point_cam
            x, y, z = point_world[:3]
        else:
            x, y, z = x_cam, y_cam, z_cam

        return Position3D(x=float(x), y=float(y), z=float(z))

    def _update_scene_graph(self, objects: list[Detection3D]):
        """更新场景图"""
        if not objects:
            self._scene_graph = SceneGraph(
                objects=[],
                relations=[],
                regions=[],
                frame_id=PERCEPTION_IMPL_MAP_FRAME_ID,
                timestamp=time.time()
            )
            return

        # 计算空间关系
        relations = self._compute_relations(objects)

        # 识别区域
        regions = self._identify_regions(objects)

        # 创建场景图
        self._scene_graph = SceneGraph(
            objects=objects,
            relations=relations,
            regions=regions,
            timestamp=time.time(),
            frame_id=PERCEPTION_IMPL_MAP_FRAME_ID
        )

    def _compute_relations(self, objects: list[Detection3D]) -> list[Relation]:
        """计算物体间的空间关系"""
        relations = []
        near_threshold = 1.5  # 米

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i >= j:
                    continue

                # 计算距离
                pos1 = np.array([obj1.position_3d.x, obj1.position_3d.y, obj1.position_3d.z])
                pos2 = np.array([obj2.position_3d.x, obj2.position_3d.y, obj2.position_3d.z])
                distance = np.linalg.norm(pos1 - pos2)

                # near关系
                if distance < near_threshold:
                    relations.append(Relation(
                        subject_id=obj1.id,
                        predicate="near",
                        object_id=obj2.id,
                        confidence=1.0 - (distance / near_threshold)
                    ))

        return relations

    def _identify_regions(self, objects: list[Detection3D]) -> list[Region]:
        """识别空间区域（简单聚类）"""
        if not objects:
            return []

        # 简单的基于距离的聚类
        regions = []
        cluster_radius = 3.0  # 米
        assigned = set()

        for _i, obj in enumerate(objects):
            if obj.id in assigned:
                continue

            # 创建新区域
            region_objects = [obj.id]
            assigned.add(obj.id)

            # 查找附近的物体
            pos1 = np.array([obj.position_3d.x, obj.position_3d.y])
            for _j, other in enumerate(objects):
                if other.id in assigned:
                    continue

                pos2 = np.array([other.position_3d.x, other.position_3d.y])
                distance = np.linalg.norm(pos1 - pos2)

                if distance < cluster_radius:
                    region_objects.append(other.id)
                    assigned.add(other.id)

            # 计算区域中心
            positions = [
                np.array([o.position_3d.x, o.position_3d.y, o.position_3d.z])
                for o in objects if o.id in region_objects
            ]
            center = np.mean(positions, axis=0)

            regions.append(Region(
                name=f"region_{len(regions)}",
                object_ids=region_objects,
                center=Position3D(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            ))

        return regions

    def get_scene_graph(self) -> SceneGraph:
        """
        获取当前场景图

        Returns:
            场景图对象
        """
        if self._scene_graph is None:
            return SceneGraph(
                objects=[],
                relations=[],
                regions=[],
                frame_id=PERCEPTION_IMPL_MAP_FRAME_ID,
                timestamp=time.time()
            )
        return self._scene_graph

    def query_objects(
        self,
        label: str | None = None,
        min_confidence: float = 0.0,
        region_name: str | None = None
    ) -> list[Detection3D]:
        """
        查询物体

        Args:
            label: 物体标签（可选）
            min_confidence: 最小置信度
            region_name: 区域名称（可选）

        Returns:
            匹配的物体列表
        """
        if self._scene_graph is None:
            return []

        results = self._scene_graph.objects

        # 按标签过滤
        if label is not None:
            results = [
                obj for obj in results
                if obj.label.lower() == label.lower()
            ]

        # 按置信度过滤
        results = [
            obj for obj in results
            if obj.confidence >= min_confidence
        ]

        # 按区域过滤
        if region_name is not None:
            region = next(
                (r for r in self._scene_graph.regions if r.name == region_name),
                None
            )
            if region:
                results = [
                    obj for obj in results
                    if obj.id in region.object_ids
                ]

        return results

    def get_statistics(self) -> dict:
        """
        获取统计信息

        Returns:
            统计信息字典
        """
        stats = {
            "frame_count": self._frame_count,
            "process_count": self._process_count,
            "avg_process_time": (
                self._total_process_time / self._process_count
                if self._process_count > 0 else 0.0
            ),
            "avg_fps": (
                self._process_count / self._total_process_time
                if self._total_process_time > 0 else 0.0
            ),
        }

        # 添加场景图统计
        if self._scene_graph:
            stats.update({
                "object_count": len(self._scene_graph.objects),
                "relation_count": len(self._scene_graph.relations),
                "region_count": len(self._scene_graph.regions),
            })

        # 添加追踪器统计
        tracker_stats = self.tracker.get_statistics()
        stats.update({"tracker": tracker_stats})

        # 添加检测器统计
        detector_info = self.detector.get_model_info()
        stats.update({"detector": detector_info})

        # 添加编码器统计
        encoder_info = self.encoder.get_model_info()
        stats.update({"encoder": encoder_info})

        return stats

    def reset(self):
        """重置感知系统"""
        self.tracker.reset()
        self._scene_graph = None
        self._frame_count = 0
        self._total_process_time = 0.0
        self._process_count = 0
        logger.info("Perception system reset")

    def configure(self, config: PerceptionConfig):
        """
        配置感知系统

        Args:
            config: 配置对象
        """
        self.config = config

        # 更新检测器配置
        self.detector.set_confidence_threshold(config.confidence_threshold)

        # 更新追踪器配置
        self.tracker.configure({
            "merge_distance": config.merge_distance,
            "iou_threshold": config.iou_threshold,
        })

        logger.info("Perception system reconfigured")
