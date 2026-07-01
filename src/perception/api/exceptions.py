"""
Semantic Perception API - 异常定义

定义所有API接口使用的异常类型
"""


class PerceptionAPIError(Exception):
    """感知API基础异常"""
    pass


class DetectorError(PerceptionAPIError):
    """检测器异常"""
    pass


class DetectorInitError(DetectorError):
    """检测器初始化失败"""
    pass


class DetectorInferenceError(DetectorError):
    """检测器推理失败"""
    pass


class EncoderError(PerceptionAPIError):
    """编码器异常"""
    pass


class EncoderInitError(EncoderError):
    """编码器初始化失败"""
    pass


class EncoderInferenceError(EncoderError):
    """编码器推理失败"""
    pass


class TrackerError(PerceptionAPIError):
    """追踪器异常"""
    pass


class SceneGraphError(PerceptionAPIError):
    """场景图异常"""
    pass


class InvalidImageError(PerceptionAPIError):
    """无效图像"""
    pass


class InvalidDepthError(PerceptionAPIError):
    """无效深度图"""
    pass


class InvalidCameraInfoError(PerceptionAPIError):
    """无效相机参数"""
    pass


class ConfigurationError(PerceptionAPIError):
    """配置错误"""
    pass
