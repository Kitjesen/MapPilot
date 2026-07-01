"""
Semantic Perception API - 工厂类

用于创建各种感知组件的实例
"""


from runtime.registry import get, list_plugins, register

from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .exceptions import ConfigurationError
from .perception_api import PerceptionAPI
from .tracker_api import TrackerAPI
from .types import PerceptionConfig


@register("detector_factory", "yolo_world", description="YOLO-World DetectorAPI adapter")
class _YOLOWorldFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> DetectorAPI:
        from ..impl.yolo_world_detector import YOLOWorldDetector

        return YOLOWorldDetector(config)


@register("encoder_factory", "clip", description="CLIP EncoderAPI adapter")
class _CLIPFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> EncoderAPI:
        from ..impl.clip_encoder import CLIPEncoder

        return CLIPEncoder(config)


@register("perception_factory_tracker", "instance", description="Instance TrackerAPI adapter")
class _InstanceTrackerFactoryProvider:
    @staticmethod
    def create(config: PerceptionConfig | None) -> TrackerAPI:
        from ..impl.instance_tracker import InstanceTracker

        return InstanceTracker(config)


class PerceptionFactory:
    """
    感知系统工厂类

    提供统一的创建接口，隐藏具体实现细节
    """

    @staticmethod
    def create_perception(
        detector_type: str = "yolo_world",
        encoder_type: str = "clip",
        tracker_type: str = "instance",
        config: PerceptionConfig | None = None
    ) -> PerceptionAPI:
        """
        创建完整的感知系统

        Args:
            detector_type: 检测器类型 (yolo_world | grounding_dino)
            encoder_type: 编码器类型 (clip | blip)
            tracker_type: 追踪器类型 (instance)
            config: 配置对象

        Returns:
            PerceptionAPI实例

        Raises:
            ConfigurationError: 配置错误

        Example:
            >>> from perception.api import PerceptionFactory
            >>> perception = PerceptionFactory.create_perception(
            ...     detector_type="yolo_world",
            ...     encoder_type="clip",
            ...     config=config
            ... )
            >>> detections = perception.process_frame(rgb, depth, camera_info)
        """
        try:
            # 创建各个组件
            detector = PerceptionFactory.create_detector(detector_type, config)
            encoder = PerceptionFactory.create_encoder(encoder_type, config)
            tracker = PerceptionFactory.create_tracker(tracker_type, config)

            # 创建感知系统
            from ..impl.perception_impl import PerceptionImpl
            return PerceptionImpl(detector, encoder, tracker, config)

        except Exception as e:
            raise ConfigurationError(f"Failed to create perception system: {e}") from e

    @staticmethod
    def create_detector(
        detector_type: str,
        config: PerceptionConfig | None = None
    ) -> DetectorAPI:
        """
        创建检测器

        Args:
            detector_type: 检测器类型
                - "yolo_world": YOLO-World开放词汇检测器
                - "grounding_dino": Grounding DINO检测器
            config: 配置对象

        Returns:
            DetectorAPI实例

        Raises:
            ConfigurationError: 不支持的检测器类型

        Example:
            >>> detector = PerceptionFactory.create_detector("yolo_world")
            >>> detector.set_classes(["chair", "table", "person"])
            >>> detections = detector.detect(image)
        """
        try:
            provider = get("detector_factory", detector_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown detector type: {detector_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_detectors())}"
            ) from None

    @staticmethod
    def create_encoder(
        encoder_type: str,
        config: PerceptionConfig | None = None
    ) -> EncoderAPI:
        """
        创建编码器

        Args:
            encoder_type: 编码器类型
                - "clip": CLIP视觉-语言编码器
                - "blip": BLIP编码器
            config: 配置对象

        Returns:
            EncoderAPI实例

        Raises:
            ConfigurationError: 不支持的编码器类型

        Example:
            >>> encoder = PerceptionFactory.create_encoder("clip")
            >>> image_feat = encoder.encode_image(image)
            >>> text_feat = encoder.encode_text("a red chair")
            >>> similarity = encoder.compute_similarity(image_feat, text_feat)
        """
        try:
            provider = get("encoder_factory", encoder_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown encoder type: {encoder_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_encoders())}"
            ) from None

    @staticmethod
    def create_tracker(
        tracker_type: str,
        config: PerceptionConfig | None = None
    ) -> TrackerAPI:
        """
        创建追踪器

        Args:
            tracker_type: 追踪器类型
                - "instance": 实例追踪器
            config: 配置对象

        Returns:
            TrackerAPI实例

        Raises:
            ConfigurationError: 不支持的追踪器类型

        Example:
            >>> tracker = PerceptionFactory.create_tracker("instance")
            >>> tracked = tracker.update(detections_3d)
        """
        try:
            provider = get("perception_factory_tracker", tracker_type)
            return provider.create(config)
        except KeyError:
            raise ConfigurationError(
                f"Unknown tracker type: {tracker_type}. "
                f"Supported types: {', '.join(PerceptionFactory.get_available_trackers())}"
            ) from None

    @staticmethod
    def create_from_config(config: PerceptionConfig) -> PerceptionAPI:
        """
        从配置对象创建感知系统

        Args:
            config: 配置对象，包含detector_type和encoder_type

        Returns:
            PerceptionAPI实例

        Example:
            >>> config = PerceptionConfig(
            ...     detector_type="yolo_world",
            ...     encoder_type="clip",
            ...     confidence_threshold=0.3
            ... )
            >>> perception = PerceptionFactory.create_from_config(config)
        """
        return PerceptionFactory.create_perception(
            detector_type=config.detector_type,
            encoder_type=config.encoder_type,
            tracker_type="instance",
            config=config
        )

    @staticmethod
    def get_available_detectors() -> list:
        """
        获取可用的检测器类型

        Returns:
            检测器类型列表
        """
        return list_plugins("detector_factory")

    @staticmethod
    def get_available_encoders() -> list:
        """
        获取可用的编码器类型

        Returns:
            编码器类型列表
        """
        return list_plugins("encoder_factory")

    @staticmethod
    def get_available_trackers() -> list:
        """
        获取可用的追踪器类型

        Returns:
            追踪器类型列表
        """
        return list_plugins("perception_factory_tracker")
