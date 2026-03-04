"""
semantic_perception_node — 语义感知 ROS2 节点 (USS-Nav 风格 15Hz 管线)

管道 (每帧):
  1. 接收 RGB + Depth (message_filters 同步)
  2. Laplacian 模糊过滤 → 跳过模糊帧
  3. YOLO-E 实例分割 → mask + label + score (USS-Nav §IV-C)
  4. Mobile-CLIP 文本编码 → 语义向量 (仅编码标签, 缓存后零开销)
  5. Mask + Depth → 物体点云 (USS-Nav: mask→cloud projection)
  6. 双指标优先级融合 (语义+几何) → 实例匹配
  7. 发布场景图 (JSON over std_msgs/String)

USS-Nav vs 原实现:
  原: YOLO-World bbox → CLIP 图像裁剪编码 → 单点投影 → 质心+标签匹配
  新: YOLO-E mask → Mobile-CLIP 文本编码 → 完整点云 → 双指标融合

性能: 15Hz on Jetson Orin NX (vs 原 5Hz)

参考:
  - USS-Nav (2026): Unified Spatio-Semantic Scene Graph
  - YOLOE (2025): Real-Time Seeing Anything
  - MobileCLIP (CVPR 2024): Fast Image-Text Models
"""

import asyncio
import json
import os
import re
import time
import traceback
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from semantic_perception.srv import QueryScene

import message_filters
from cv_bridge import CvBridge

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .laplacian_filter import is_blurry
from .clip_encoder import CLIPEncoder
from .mobileclip_encoder import MobileCLIPEncoder
from .projection import (
    CameraIntrinsics,
    Detection3D,
    bbox_center_depth,
    project_to_3d,
    transform_point,
    mask_to_pointcloud,
    pointcloud_centroid,
)
from .instance_tracker import InstanceTracker


class SemanticPerceptionNode(Node):
    """语义感知 ROS2 节点 (论文级实现)。"""

    def __init__(self):
        super().__init__("semantic_perception_node")

        # ── 参数声明 ──
        self.declare_parameter("detector_type", "yoloe")  # USS-Nav: yoloe (默认) | yolo_world | grounding_dino
        # YOLO-E 分割参数 (USS-Nav 默认)
        self.declare_parameter("yoloe.model_size", "l")
        self.declare_parameter("yoloe.confidence", 0.3)
        self.declare_parameter("yoloe.iou_threshold", 0.5)
        self.declare_parameter("yoloe.tensorrt", False)
        self.declare_parameter("yoloe.max_detections", 30)
        # YOLO-World 参数 (legacy fallback)
        self.declare_parameter("yolo_world.model_size", "l")
        self.declare_parameter("yolo_world.confidence", 0.3)
        self.declare_parameter("yolo_world.iou_threshold", 0.5)
        self.declare_parameter("yolo_world.tensorrt", False)
        # 语义编码 (USS-Nav: text-only Mobile-CLIP)
        self.declare_parameter("clip.enable", True)
        self.declare_parameter("clip.model", "ViT-B/32")
        self.declare_parameter("clip.text_only", True)  # USS-Nav: 仅文本编码
        # GroundingDINO 参数 (legacy)
        self.declare_parameter("grounding_dino.config_path", "")
        self.declare_parameter("grounding_dino.weights_path", "")
        self.declare_parameter("grounding_dino.box_threshold", 0.35)
        self.declare_parameter("grounding_dino.text_threshold", 0.25)
        self.declare_parameter(
            "default_classes",
            "door . chair . fire extinguisher . person . desk . stairs . elevator . sign",
        )
        self.declare_parameter("laplacian_threshold", 100.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("max_depth", 6.0)
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("clip_similarity_threshold", 0.75)
        self.declare_parameter("merge_distance", 0.5)
        self.declare_parameter("scene_graph.enable", True)
        self.declare_parameter("scene_graph.publish_rate", 1.0)
        self.declare_parameter("semantic_map.enable", True)  # 点云到语义地图渲染
        self.declare_parameter("scene_graph.max_objects", 200)
        self.declare_parameter("target_fps", 15.0)   # USS-Nav: 15Hz on Jetson Orin NX
        self.declare_parameter("skip_frames", 1)      # USS-Nav: 处理每帧
        # TF2 参数 (A1 修复)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.1)
        # 实例追踪参数 (C10 参数化)
        self.declare_parameter("tracker.stale_timeout", 300.0)
        self.declare_parameter("tracker.ema_alpha", 0.3)
        self.declare_parameter("tracker.near_threshold", 1.5)
        self.declare_parameter("tracker.on_threshold", 0.3)
        self.declare_parameter("tracker.region_cluster_radius", 3.0)
        # 可配置阈值 (D13)
        self.declare_parameter("adaptive_blur_detection", False)
        # Room LLM 命名 (创新1 补强)
        self.declare_parameter("room_naming.llm_enable", False)
        self.declare_parameter("room_naming.api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("room_naming.model", "gpt-4o-mini")
        self.declare_parameter("room_naming.language", "zh")
        # 指令→检测器文本处理: 动态合并用户指令目标词到检测类别
        self.declare_parameter("instruction_topic", "instruction")
        self.declare_parameter("instruction_merge_enable", True)

        # ── 读取参数 ──
        self._detector_type = self.get_parameter("detector_type").value
        self._laplacian_threshold = self.get_parameter("laplacian_threshold").value
        self._depth_scale = self.get_parameter("depth_scale").value
        self._max_depth = self.get_parameter("max_depth").value
        self._min_depth = self.get_parameter("min_depth").value
        self._default_classes = self.get_parameter("default_classes").value
        self._target_fps = self.get_parameter("target_fps").value
        self._skip_frames = self.get_parameter("skip_frames").value
        self._sg_enable = self.get_parameter("scene_graph.enable").value
        self._sg_publish_rate = self.get_parameter("scene_graph.publish_rate").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._world_frame = self.get_parameter("world_frame").value
        self._tf_timeout = self.get_parameter("tf_timeout_sec").value

        # ── 内部状态 ──
        self._bridge = CvBridge()
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._frame_count = 0
        self._last_process_time = 0.0
        self._min_interval = 1.0 / max(self._target_fps, 0.1)
        self._warned_no_camera_info = False
        self._warned_no_tf = False
        self._tf_fail_count = 0
        self._tf_total_count = 0

        # 指令→检测器: 缓存最新指令和合并后的类别
        self._instruction_merge_enable = self.get_parameter("instruction_merge_enable").value
        self._latest_instruction: str = ""
        self._merged_classes_cache: Optional[str] = None
        self._last_instruction_for_cache: str = ""

        # 实例追踪器
        self._tracker = InstanceTracker(
            merge_distance=self.get_parameter("merge_distance").value,
            iou_threshold=self.get_parameter("iou_threshold").value,
            clip_threshold=self.get_parameter("clip_similarity_threshold").value,
            max_objects=self.get_parameter("scene_graph.max_objects").value,
            stale_timeout=self.get_parameter("tracker.stale_timeout").value,
        )

        # 知识图谱注入 (ConceptBot / DovSG: 每个物体创建时查 KG 补属性)
        try:
            from .knowledge_graph import IndustrialKnowledgeGraph
            self._knowledge_graph = IndustrialKnowledgeGraph()
            self._tracker.set_knowledge_graph(self._knowledge_graph)
            kg_stats = self._knowledge_graph.get_stats()
            self.get_logger().info(
                "KG injected: %d concepts, %d relations, %d safety constraints",
                kg_stats["total_concepts"],
                kg_stats["total_relations"],
                kg_stats["total_safety_constraints"],
            )

            # 用 KG 词汇表扩展检测器类别 (开放词汇增强)
            kg_vocab = self._knowledge_graph.get_clip_vocabulary()
            if kg_vocab and self._default_classes:
                existing = set(c.strip() for c in self._default_classes.split(".") if c.strip())
                new_terms = [v for v in kg_vocab if v.lower() not in {e.lower() for e in existing}]
                if new_terms:
                    expanded = self._default_classes.rstrip(" .") + " . " + " . ".join(new_terms[:30])
                    self._default_classes = expanded
                    self.get_logger().info(
                        "Detection classes expanded with %d KG terms (total: %d)",
                        min(len(new_terms), 30),
                        len(expanded.split(".")),
                    )
        except Exception as e:
            self._knowledge_graph = None
            self.get_logger().warning("KG initialization failed (non-critical): %s", e)

        # 场景图快照 (DovSG 动态更新)
        self._prev_scene_snapshot: dict = {}

        # 创新1 补强: Room LLM 命名 (可选)
        if self.get_parameter("room_naming.llm_enable").value:
            api_key_env = self.get_parameter("room_naming.api_key_env").value
            api_key = os.environ.get(api_key_env, "")
            if api_key:
                model = self.get_parameter("room_naming.model").value
                lang = self.get_parameter("room_naming.language").value
                self._tracker.set_room_namer(
                    self._make_room_llm_namer(api_key, model, lang)
                )
                self.get_logger().info("Room LLM naming enabled")
            else:
                self.get_logger().warn(
                    f"Room LLM naming requested but {api_key_env} not set"
                )

        # ── TF2 (A1 修复: 精确 camera→map 变换) ──
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 语义编码器 (USS-Nav: text-only Mobile-CLIP) ──
        self._clip_encoder = None
        self._clip_enabled = self.get_parameter("clip.enable").value
        self._text_only_mode = self.get_parameter("clip.text_only").value
        if self._clip_enabled:
            try:
                if self._text_only_mode:
                    self._clip_encoder = MobileCLIPEncoder(
                        model_name=self.get_parameter("clip.model").value.replace("/", "-"),
                    )
                    self._clip_encoder.load_model()
                    # 预编码默认检测类别
                    default_labels = [c.strip() for c in self._default_classes.split(".") if c.strip()]
                    if default_labels:
                        self._clip_encoder.precompute_labels(default_labels)
                    self.get_logger().info(
                        "USS-Nav MobileCLIP text encoder loaded (text-only, %d labels cached)",
                        len(default_labels),
                    )
                else:
                    self._clip_encoder = CLIPEncoder(
                        model_name=self.get_parameter("clip.model").value,
                    )
                    self._clip_encoder.load_model()
                    self.get_logger().info("CLIP encoder loaded (legacy image+text mode)")
            except Exception as e:
                self.get_logger().warn(
                    f"Semantic encoder load failed, using string matching only: {e}"
                )
                self._clip_encoder = None

        # ── 检测器 ──
        self._detector = None
        self._init_detector()

        # ── 订阅 ──
        self._sub_camera_info = self.create_subscription(
            CameraInfo, "camera_info",
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        # RGB + Depth 同步
        self._sub_color = message_filters.Subscriber(
            self, Image, "color_image", qos_profile=qos_profile_sensor_data
        )
        self._sub_depth = message_filters.Subscriber(
            self, Image, "depth_image", qos_profile=qos_profile_sensor_data
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_depth],
            queue_size=5,
            slop=0.05,
        )
        self._sync.registerCallback(self._rgbd_callback)

        # Costmap (A5: 为 FrontierScorer 提供数据)
        self._latest_costmap: Optional[OccupancyGrid] = None
        self._sub_costmap = self.create_subscription(
            OccupancyGrid, "/nav/costmap",
            self._costmap_callback,
            10,
        )

        # 指令订阅 (开放词汇: 动态合并用户指令目标词到检测类别)
        if self._instruction_merge_enable:
            instruction_topic = self.get_parameter("instruction_topic").value
            self._sub_instruction = self.create_subscription(
                String, instruction_topic,
                self._instruction_callback,
                10,
            )
            self.get_logger().info(
                f"Instruction merge enabled: subscribing to '{instruction_topic}'"
            )

        # ── 发布 ──
        self._pub_detections = self.create_publisher(String, "detections_3d", 10)
        self._pub_scene_graph = self.create_publisher(String, "scene_graph", 10)
        self._pub_scene_diff = self.create_publisher(String, "scene_diff", 10)
        self._semantic_map_enable = self.get_parameter("semantic_map.enable").value
        self._pub_semantic_markers = (
            self.create_publisher(MarkerArray, "semantic_map_markers", 10)
            if self._semantic_map_enable
            else None
        )

        # ── 场景图定时发布 ──
        if self._sg_enable and self._sg_publish_rate > 0:
            period = 1.0 / self._sg_publish_rate
            self._sg_timer = self.create_timer(period, self._publish_scene_graph)

        # ── 查询服务 ──
        self._query_srv = self.create_service(
            QueryScene, "/nav/semantic/query", self._query_callback
        )

        self.get_logger().info(
            f"SemanticPerceptionNode started: detector={self._detector_type}, "
            f"TF: {self._camera_frame}→{self._world_frame}, "
            f"classes={self._default_classes}, "
            f"target_fps={self._target_fps}, skip_frames={self._skip_frames}"
        )

    # ================================================================
    #  初始化
    # ================================================================

    def _init_detector(self):
        """初始化检测器后端 (USS-Nav: 默认 YOLO-E 分割)。"""
        try:
            if self._detector_type == "yoloe":
                from .yoloe_detector import YOLOEDetector
                self._detector = YOLOEDetector(
                    model_size=self.get_parameter("yoloe.model_size").value,
                    confidence=self.get_parameter("yoloe.confidence").value,
                    iou_threshold=self.get_parameter("yoloe.iou_threshold").value,
                    tensorrt=self.get_parameter("yoloe.tensorrt").value,
                    max_detections=self.get_parameter("yoloe.max_detections").value,
                )
            elif self._detector_type == "yolo_world":
                from .yolo_world_detector import YOLOWorldDetector
                self._detector = YOLOWorldDetector(
                    model_size=self.get_parameter("yolo_world.model_size").value,
                    confidence=self.get_parameter("yolo_world.confidence").value,
                    iou_threshold=self.get_parameter("yolo_world.iou_threshold").value,
                    tensorrt=self.get_parameter("yolo_world.tensorrt").value,
                )
            elif self._detector_type == "grounding_dino":
                from .grounding_dino_detector import GroundingDINODetector
                config_path = self.get_parameter("grounding_dino.config_path").value
                weights_path = self.get_parameter("grounding_dino.weights_path").value
                self._detector = GroundingDINODetector(
                    config_path=config_path or None,
                    weights_path=weights_path or None,
                    box_threshold=self.get_parameter("grounding_dino.box_threshold").value,
                    text_threshold=self.get_parameter("grounding_dino.text_threshold").value,
                )
            else:
                self.get_logger().error(f"Unknown detector_type: {self._detector_type}")
                return

            self._detector.load_model()
            self.get_logger().info(f"Detector '{self._detector_type}' loaded")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load detector '{self._detector_type}': {e}\n"
                f"{traceback.format_exc()}"
            )
            self._detector = None

    # ================================================================
    #  TF2 变换查询 (A1 修复)
    # ================================================================

    def _lookup_tf_camera_to_world(self, stamp) -> Optional[np.ndarray]:
        """
        使用 TF2 查询 camera_link → map 精确变换矩阵。

        A1 修复: 替换之前用里程计近似 camera pose 的做法。
        TF2 会自动处理 camera_link → body → odom → map 的链路。

        Args:
            stamp: ROS2 Time (帧的时间戳)

        Returns:
            4x4 变换矩阵, 或 None (TF 不可用时)
        """
        try:
            # 非阻塞查询 (关键不变量 P1 #9: TF non-blocking)
            if not self._tf_buffer.can_transform(
                self._world_frame,
                self._camera_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            ):
                # TF 尚未就绪 — 尝试用最新可用变换
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._world_frame,
                        self._camera_frame,
                        rclpy.time.Time(),  # 最新可用
                        timeout=Duration(seconds=self._tf_timeout),
                    )
                except Exception:
                    return None
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._world_frame,
                    self._camera_frame,
                    stamp,
                    timeout=Duration(seconds=self._tf_timeout),
                )

            # TransformStamped → 4x4 矩阵
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = self._quat_to_rotation(q.x, q.y, q.z, q.w)
            tf_mat = np.eye(4)
            tf_mat[:3, :3] = rot
            tf_mat[:3, 3] = [t.x, t.y, t.z]
            return tf_mat

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f"TF2 lookup {self._camera_frame}→{self._world_frame} failed: {e}. "
                    f"3D projection disabled until TF available."
                )
                self._warned_no_tf = True
            return None

    @staticmethod
    def _quat_to_rotation(x, y, z, w) -> np.ndarray:
        """四元数 → 3x3 旋转矩阵。"""
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    # ================================================================
    #  Callbacks
    # ================================================================

    def _camera_info_callback(self, msg: CameraInfo):
        """接收相机内参 (只需一次)。"""
        if self._intrinsics is not None:
            return

        self._intrinsics = CameraIntrinsics(
            fx=msg.k[0], fy=msg.k[4],
            cx=msg.k[2], cy=msg.k[5],
            width=msg.width, height=msg.height,
        )
        self.get_logger().info(
            f"Camera intrinsics received: fx={self._intrinsics.fx:.1f}, "
            f"fy={self._intrinsics.fy:.1f}, "
            f"{self._intrinsics.width}x{self._intrinsics.height}"
        )

    def _costmap_callback(self, msg: OccupancyGrid):
        """缓存最新 costmap (A5: 为 FrontierScorer 提供数据)。"""
        self._latest_costmap = msg

    def _instruction_callback(self, msg: String):
        """
        缓存最新用户指令 (开放词汇: 用于动态合并检测类别)。

        Planner 发布 JSON: {"instruction": "...", "language": "zh", ...}
        或纯文本。
        """
        try:
            data = json.loads(msg.data)
            inst = data.get("instruction", msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            inst = msg.data if msg.data else ""
        if isinstance(inst, str) and inst.strip():
            self._latest_instruction = inst.strip()
            self._merged_classes_cache = None  # 失效缓存, 下次合并时重新计算

    def _extract_instruction_keywords(self, instruction: str) -> List[str]:
        """
        从指令中提取目标词 (用于合并到检测类别)。

        轻量实现, 不依赖 semantic_planner, 支持中英文。
        """
        if not instruction or not instruction.strip():
            return []

        stop_words = {
            "the", "a", "an", "to", "go", "find", "get", "me", "for", "and", "or",
            "is", "at", "in", "on", "near", "next", "by", "of", "with", "from",
            "去", "到", "找", "拿", "的", "在", "旁边", "附近", "那个",
            "请", "帮", "我", "一个", "把", "了", "着", "过", "that", "this",
        }

        inst_lower = instruction.lower().strip()
        # 英文词
        en_tokens = re.findall(r"[a-zA-Z]+", inst_lower)
        # 中文 (2-4 字词组, 如 灭火器/红色/杯子)
        zh_tokens = re.findall(r"[\u4e00-\u9fff]{2,6}", instruction)

        keywords: List[str] = []
        for w in en_tokens:
            if w not in stop_words and len(w) > 1:
                keywords.append(w)
        for w in zh_tokens:
            if w not in stop_words:
                keywords.append(w)

        # 去重且保留顺序
        seen = set()
        unique: List[str] = []
        for k in keywords:
            k_lower = k.lower()
            if k_lower not in seen:
                seen.add(k_lower)
                unique.append(k)

        return unique

    def _merge_detection_classes(self) -> str:
        """
        合并 default_classes + 指令中的目标词。

        有缓存: 指令未变时复用, 避免每帧重复解析。
        """
        if not self._instruction_merge_enable or not self._latest_instruction:
            return self._default_classes

        if (
            self._merged_classes_cache is not None
            and self._last_instruction_for_cache == self._latest_instruction
        ):
            return self._merged_classes_cache

        default_set = {
            c.strip().lower()
            for c in self._default_classes.split(".")
            if c.strip()
        }
        keywords = self._extract_instruction_keywords(self._latest_instruction)

        for kw in keywords:
            if kw.lower() not in default_set:
                default_set.add(kw.lower())

        # 保持 default_classes 顺序, 追加新词
        default_list = [
            c.strip() for c in self._default_classes.split(".") if c.strip()
        ]
        for kw in keywords:
            if kw.lower() not in {c.lower() for c in default_list}:
                default_list.append(kw)

        merged = " . ".join(default_list)
        self._merged_classes_cache = merged
        self._last_instruction_for_cache = self._latest_instruction
        return merged

    def _rgbd_callback(self, color_msg: Image, depth_msg: Image):
        """RGB-D 同步回调 — 主处理管道。"""
        # 帧率控制
        self._frame_count += 1
        if self._frame_count % max(self._skip_frames, 1) != 0:
            return

        now = time.time()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now

        # 前置条件检查
        if self._detector is None:
            return
        if self._intrinsics is None:
            if not self._warned_no_camera_info:
                self.get_logger().warn("Waiting for camera_info...")
                self._warned_no_camera_info = True
            return

        # A1 修复: TF2 查询精确变换
        self._tf_total_count += 1
        tf_camera_to_world = self._lookup_tf_camera_to_world(color_msg.header.stamp)
        if tf_camera_to_world is None:
            self._tf_fail_count += 1
            if self._tf_fail_count % 50 == 0:
                fail_rate = self._tf_fail_count / max(self._tf_total_count, 1) * 100
                self.get_logger().warn(
                    "TF fail rate: %d/%d (%.1f%%) — 3D projection degraded",
                    self._tf_fail_count, self._tf_total_count, fail_rate,
                )
            return

        try:
            self._process_frame(color_msg, depth_msg, tf_camera_to_world)
        except Exception as e:
            # A2 修复: 不再静默吞异常, 完整记录日志
            self.get_logger().error(
                f"Frame processing error (frame={self._frame_count}): {e}\n"
                f"{traceback.format_exc()}"
            )

    # ================================================================
    #  核心处理
    # ================================================================

    def _process_frame(
        self,
        color_msg: Image,
        depth_msg: Image,
        tf_camera_to_world: np.ndarray,
    ):
        """
        USS-Nav 风格处理单帧 RGB-D。

        Pipeline: YOLO-E mask → Mobile-CLIP text → mask+depth→点云 → 双指标融合
        """
        # 1. 转换为 numpy
        bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # 2. Laplacian 模糊检测
        if is_blurry(bgr, threshold=self._laplacian_threshold):
            return

        # 3. 开放词汇检测 (USS-Nav: YOLO-E 实例分割 → mask + label)
        classes_to_detect = self._merge_detection_classes()
        detections_2d = self._detector.detect(bgr, classes_to_detect)
        if not detections_2d:
            return

        # 3.5 USS-Nav: Mobile-CLIP 文本编码 (仅编码标签, 缓存后零开销)
        if self._clip_encoder is not None and self._text_only_mode:
            try:
                unique_labels = list({d.label for d in detections_2d})
                if isinstance(self._clip_encoder, MobileCLIPEncoder):
                    self._clip_encoder.precompute_labels(unique_labels)
                    for det in detections_2d:
                        det.features = self._clip_encoder.encode_label(det.label)
                else:
                    label_feats = self._clip_encoder.encode_text(unique_labels)
                    label_map = {l: f for l, f in zip(unique_labels, label_feats)}
                    for det in detections_2d:
                        det.features = label_map.get(det.label, np.array([]))
            except Exception as e:
                self.get_logger().warn(f"Text encoding failed: {e}")
        elif self._clip_encoder is not None:
            # Legacy: 图像裁剪编码 (慢, 仅 YOLO-World fallback)
            try:
                bboxes = [d.bbox for d in detections_2d]
                clip_features = self._clip_encoder.encode_image_crops(bgr, bboxes)
                for det, feat in zip(detections_2d, clip_features):
                    det.features = feat
            except Exception as e:
                self.get_logger().warn(f"CLIP encoding failed: {e}")

        # 4. USS-Nav: mask + depth → 物体点云 + 质心投影
        detections_3d = []
        for det2d in detections_2d:
            # 4a. 尝试 mask→点云 (USS-Nav 主路径)
            points = None
            centroid = None
            center_depth = None

            if det2d.mask is not None:
                points = mask_to_pointcloud(
                    mask=det2d.mask,
                    depth_image=depth,
                    intrinsics=self._intrinsics,
                    tf_camera_to_world=tf_camera_to_world,
                    depth_scale=self._depth_scale,
                    min_depth=self._min_depth,
                    max_depth=self._max_depth,
                )
                if points is not None and len(points) > 0:
                    centroid = pointcloud_centroid(points)
                    center_depth = float(np.linalg.norm(centroid - tf_camera_to_world[:3, 3]))

            # 4b. Fallback: bbox 中心深度投影 (无 mask 时)
            if centroid is None:
                d = bbox_center_depth(depth, det2d.bbox, depth_scale=self._depth_scale)
                if d is None or d < self._min_depth or d > self._max_depth:
                    continue
                cx = (det2d.bbox[0] + det2d.bbox[2]) / 2
                cy = (det2d.bbox[1] + det2d.bbox[3]) / 2
                p_camera = project_to_3d(cx, cy, d, self._intrinsics)
                centroid = transform_point(p_camera, tf_camera_to_world)
                center_depth = d

            detections_3d.append(Detection3D(
                position=centroid,
                label=det2d.label,
                score=det2d.score,
                bbox_2d=det2d.bbox,
                depth=center_depth,
                features=getattr(det2d, 'features', np.array([])),
                points=points if points is not None else np.empty((0, 3)),
            ))

        if not detections_3d:
            return

        # 5. USS-Nav 双指标融合实例追踪
        tracked_objs = self._tracker.update(detections_3d)

        # 5.5 开放词汇: 未知物体 → KG 概念映射 (DovSG / LOVON)
        if self._knowledge_graph is not None:
            for obj in tracked_objs:
                if not obj.kg_concept_id and obj.detection_count <= 2:
                    mapped = self._knowledge_graph.map_unknown_to_concept(
                        obj.label,
                        clip_embedding=obj.features if obj.features.size > 0 else None,
                        clip_encoder=self._clip_encoder,
                    )
                    if mapped is not None:
                        props = self._knowledge_graph.enrich_object_properties(mapped.concept_id)
                        obj.kg_concept_id = props.get("concept_id", "")
                        obj.safety_level = props.get("safety_level", "safe")
                        obj.affordances = props.get("affordances", [])
                        obj.functional_properties = props

        # 5.6 记录关键视角 (view 节点)
        try:
            camera_pos = tf_camera_to_world[:3, 3]
            observed_ids = [o.object_id for o in tracked_objs]
            self._tracker.record_view(camera_pos, observed_ids)
        except Exception as e:
            self.get_logger().debug(f"record_view skipped: {e}")

        # 6. 发布本帧检测 (JSON)
        det_json = json.dumps({
            "timestamp": time.time(),
            "frame_id": self._frame_count,
            "detections": [
                {
                    "label": d.label,
                    "score": round(d.score, 3),
                    "position": {
                        "x": round(float(d.position[0]), 3),
                        "y": round(float(d.position[1]), 3),
                        "z": round(float(d.position[2]), 3),
                    },
                    "depth": round(d.depth, 3),
                }
                for d in detections_3d
            ],
        }, ensure_ascii=False)

        msg = String()
        msg.data = det_json
        self._pub_detections.publish(msg)

    def _publish_scene_graph(self):
        """定时发布场景图 + DovSG diff + 语义地图 MarkerArray。"""
        if not self._tracker.objects:
            if self._pub_semantic_markers:
                self._publish_empty_semantic_markers()
            return

        msg = String()
        sg_json = self._tracker.get_scene_graph_json()

        # DovSG 动态场景图: 计算与上次快照的差异
        if self._prev_scene_snapshot:
            try:
                diff = self._tracker.compute_scene_diff(self._prev_scene_snapshot)
                if diff["total_events"] > 0:
                    sg_data = json.loads(sg_json)
                    sg_data["scene_diff"] = diff
                    sg_json = json.dumps(sg_data, ensure_ascii=False)
                    diff_msg = String()
                    diff_msg.data = json.dumps(diff, ensure_ascii=False)
                    self._pub_scene_diff.publish(diff_msg)
                    self.get_logger().info(
                        "Scene diff: %s", diff["summary"],
                    )
            except Exception as e:
                self.get_logger().debug("Scene diff failed: %s", e)

        # 缓存快照用于下次 diff
        try:
            self._prev_scene_snapshot = json.loads(sg_json)
        except Exception:
            pass

        msg.data = sg_json
        self._pub_scene_graph.publish(msg)

        if self._pub_semantic_markers:
            self._publish_semantic_map_markers(msg.data)

    def _publish_semantic_map_markers(self, scene_graph_json: str):
        """
        从场景图发布 MarkerArray，用于 RViz/地图上叠加语义物体。

        点云到语义地图渲染: 将 3D 检测物体以球体+文本标签形式渲染到地图上，
        可与 /cloud_map 点云叠加显示。
        """
        try:
            sg = json.loads(scene_graph_json)
            objects = sg.get("objects", [])
            if not isinstance(objects, list):
                return

            markers = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            frame = self._world_frame
            colors = [
                (1.0, 0.2, 0.2, 0.9),   # 红 - door
                (0.2, 0.6, 1.0, 0.9),   # 蓝 - chair
                (1.0, 0.5, 0.0, 0.9),   # 橙 - fire extinguisher
                (0.2, 0.8, 0.2, 0.9),   # 绿 - person/desk
                (0.6, 0.2, 0.8, 0.9),   # 紫 - stairs/elevator
                (1.0, 0.8, 0.2, 0.9),   # 黄 - sign
            ]
            color_by_label = {
                "door": 0, "chair": 1, "fire extinguisher": 2,
                "person": 3, "desk": 3, "stairs": 4, "elevator": 4, "sign": 5,
            }

            for i, obj in enumerate(objects[:80]):
                if not isinstance(obj, dict):
                    continue
                pos = obj.get("position", {})
                x = float(pos.get("x", 0))
                y = float(pos.get("y", 0))
                z = float(pos.get("z", 0))
                label = str(obj.get("label", "object"))
                score = float(obj.get("score", 0.5))
                first_word = label.lower().split()[0] if label else "object"
                idx = color_by_label.get(first_word, abs(hash(first_word)) % len(colors))
                r, g, b, a = colors[idx]

                sphere = Marker()
                sphere.header.frame_id = frame
                sphere.header.stamp = stamp
                sphere.ns = "semantic_objects"
                sphere.id = i * 2
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = x
                sphere.pose.position.y = y
                sphere.pose.position.z = z
                sphere.pose.orientation.w = 1.0
                size = max(0.15, min(0.5, 0.2 + score * 0.3))
                sphere.scale.x = sphere.scale.y = sphere.scale.z = size
                sphere.color.r = r
                sphere.color.g = g
                sphere.color.b = b
                sphere.color.a = a
                markers.markers.append(sphere)

                text = Marker()
                text.header.frame_id = frame
                text.header.stamp = stamp
                text.ns = "semantic_labels"
                text.id = i * 2 + 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = z + size * 0.8
                text.pose.orientation.w = 1.0
                text.scale.z = 0.15
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = label[:24]
                markers.markers.append(text)

            self._pub_semantic_markers.publish(markers)
        except (json.JSONDecodeError, TypeError, KeyError) as e:
            self.get_logger().debug(f"Semantic map markers error: {e}")

    def _publish_empty_semantic_markers(self):
        """发布空 MarkerArray，用于清除之前的语义标记。"""
        delete = Marker()
        delete.header.frame_id = self._world_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = "semantic_objects"
        delete.id = 0
        delete.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(delete)
        self._pub_semantic_markers.publish(arr)

    # ================================================================
    #  Room LLM 命名 (创新1 补强)
    # ================================================================

    @staticmethod
    def _make_room_llm_namer(api_key: str, model: str, language: str):
        """创建异步 Room LLM 命名回调 (轻量直接 HTTP 调用, 不依赖 planner 包)。"""

        async def _namer(labels: List[str]) -> str:
            labels_str = ", ".join(labels[:12])
            if language == "zh":
                system = (
                    "你是一个室内场景理解助手。根据房间内包含的物体，"
                    "推断这个区域最可能是什么类型的空间。"
                    "只输出一个简短的名称 (2-4个字)，如: 走廊、办公室、厨房、"
                    "卫生间、会议室、大厅、储物间、楼梯间。不要输出任何额外文字。"
                )
                user = f"房间内包含以下物体: {labels_str}\n这个区域是什么?"
            else:
                system = (
                    "You are an indoor scene understanding assistant. "
                    "Given objects found in a room, infer the most likely room type. "
                    "Output ONLY a short name (1-3 words). No extra text."
                )
                user = f"Objects in this area: {labels_str}\nWhat is this room?"

            try:
                import httpx
                async with httpx.AsyncClient(timeout=10.0) as client:
                    resp = await client.post(
                        "https://api.openai.com/v1/chat/completions",
                        headers={
                            "Authorization": f"Bearer {api_key}",
                            "Content-Type": "application/json",
                        },
                        json={
                            "model": model,
                            "messages": [
                                {"role": "system", "content": system},
                                {"role": "user", "content": user},
                            ],
                            "temperature": 0.1,
                            "max_tokens": 20,
                        },
                    )
                    data = resp.json()
                    return data["choices"][0]["message"]["content"].strip()
            except ImportError:
                # httpx 不可用, 用 aiohttp
                try:
                    import aiohttp
                    async with aiohttp.ClientSession() as session:
                        async with session.post(
                            "https://api.openai.com/v1/chat/completions",
                            headers={
                                "Authorization": f"Bearer {api_key}",
                                "Content-Type": "application/json",
                            },
                            json={
                                "model": model,
                                "messages": [
                                    {"role": "system", "content": system},
                                    {"role": "user", "content": user},
                                ],
                                "temperature": 0.1,
                                "max_tokens": 20,
                            },
                            timeout=aiohttp.ClientTimeout(total=10),
                        ) as resp:
                            data = await resp.json()
                            return data["choices"][0]["message"]["content"].strip()
                except ImportError:
                    return ""

        return _namer

    # ================================================================
    #  查询服务
    # ================================================================

    def _query_callback(self, request, response):
        """
        查询服务: 返回场景图 JSON。

        Args:
            request.query: 空字符串返回完整场景图，否则返回匹配的物体
        """
        try:
            if not request.query:
                # 返回完整场景图
                response.scene_graph = self._tracker.get_scene_graph_json()
            else:
                # 返回匹配的物体
                scene_data = json.loads(self._tracker.get_scene_graph_json())
                matched_objects = []
                query_lower = request.query.lower()

                for obj in scene_data.get("objects", []):
                    if query_lower in obj.get("label", "").lower():
                        matched_objects.append(obj)

                response.scene_graph = json.dumps({
                    "query": request.query,
                    "matched_count": len(matched_objects),
                    "objects": matched_objects
                })

            response.success = True
            response.message = ""
        except Exception as e:
            self.get_logger().error(f"Query service error: {e}")
            response.success = False
            response.scene_graph = ""
            response.message = str(e)
        return response

    # ================================================================
    #  生命周期
    # ================================================================

    def destroy_node(self):
        """清理资源。"""
        if self._detector is not None:
            try:
                self._detector.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Detector shutdown error: {e}")
        if self._clip_encoder is not None:
            try:
                self._clip_encoder.shutdown()
            except Exception as e:
                self.get_logger().warn(f"CLIP shutdown error: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
