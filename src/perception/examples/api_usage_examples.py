"""
Semantic Perception API - 使用示例

展示如何使用新的API接口
"""

import numpy as np

from perception.api import (
    CameraInfo,
    PerceptionConfig,
    PerceptionFactory,
)


def example_1_basic_usage():
    """示例1: 基本使用"""
    print("=== 示例1: 基本使用 ===")

    # 创建配置
    config = PerceptionConfig(
        detector_type="yolo_world",
        encoder_type="clip",
        confidence_threshold=0.3,
        merge_distance=0.5
    )

    # 创建感知系统
    perception = PerceptionFactory.create_perception(
        detector_type="yolo_world",
        encoder_type="clip",
        config=config
    )

    # 准备输入数据
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_image = np.zeros((480, 640), dtype=np.float32)
    camera_info = CameraInfo(
        fx=525.0, fy=525.0,
        cx=319.5, cy=239.5,
        width=640, height=480
    )

    # 设置检测类别
    perception.detector.set_classes(["chair", "table", "person"])

    # 处理图像
    detections = perception.process_frame(rgb_image, depth_image, camera_info)

    print(f"检测到 {len(detections)} 个物体")

    # 获取场景图
    scene_graph = perception.get_scene_graph()
    print(f"场景图包含 {len(scene_graph.objects)} 个物体")
    print(f"场景图包含 {len(scene_graph.relations)} 个关系")


def example_2_component_usage():
    """示例2: 单独使用组件"""
    print("\n=== 示例2: 单独使用组件 ===")

    # 1. 单独使用检测器
    detector = PerceptionFactory.create_detector("yolo_world")
    detector.set_classes(["chair", "table"])

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    detections_2d = detector.detect(image)
    print(f"2D检测: {len(detections_2d)} 个物体")

    # 2. 单独使用编码器
    encoder = PerceptionFactory.create_encoder("clip")

    image_feat = encoder.encode_image(image)
    text_feat = encoder.encode_text("a red chair")
    similarity = encoder.compute_similarity(image_feat, text_feat)
    print(f"相似度: {similarity:.3f}")

    # 3. 单独使用追踪器
    tracker = PerceptionFactory.create_tracker("instance")
    print(f"追踪器已创建，当前追踪数: {tracker.get_track_count()}")


def example_3_query_objects():
    """示例3: 查询物体"""
    print("\n=== 示例3: 查询物体 ===")

    perception = PerceptionFactory.create_perception()

    # 模拟处理一帧
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.zeros((480, 640), dtype=np.float32)
    camera_info = CameraInfo(fx=525.0, fy=525.0, cx=319.5, cy=239.5, width=640, height=480)

    perception.detector.set_classes(["chair", "table"])
    perception.process_frame(rgb, depth, camera_info)

    # 查询特定标签的物体
    chairs = perception.query_objects(label="chair")
    print(f"找到 {len(chairs)} 个椅子")

    # 查询高置信度物体
    high_conf = perception.query_objects(min_confidence=0.8)
    print(f"找到 {len(high_conf)} 个高置信度物体")


def example_4_statistics():
    """示例4: 获取统计信息"""
    print("\n=== 示例4: 获取统计信息 ===")

    perception = PerceptionFactory.create_perception()

    # 获取统计信息
    stats = perception.get_statistics()

    print("统计信息:")
    print(f"  处理帧数: {stats.get('frame_count', 0)}")
    print(f"  平均FPS: {stats.get('avg_fps', 0):.1f}")
    print(f"  物体数量: {stats.get('object_count', 0)}")

    # 获取检测器信息
    detector_info = perception.detector.get_model_info()
    print("\n检测器信息:")
    print(f"  名称: {detector_info['name']}")
    print(f"  版本: {detector_info['version']}")
    print(f"  设备: {detector_info['device']}")

    # 获取编码器信息
    encoder_info = perception.encoder.get_model_info()
    print("\n编码器信息:")
    print(f"  名称: {encoder_info['name']}")
    print(f"  特征维度: {encoder_info['feature_dim']}")
    print(f"  缓存命中率: {encoder_info['cache_hit_rate']:.2%}")


def example_5_configuration():
    """示例5: 配置和重配置"""
    print("\n=== 示例5: 配置和重配置 ===")

    # 创建初始配置
    config = PerceptionConfig(
        detector_type="yolo_world",
        encoder_type="clip",
        confidence_threshold=0.3,
        merge_distance=0.5
    )

    perception = PerceptionFactory.create_perception(config=config)
    print(f"初始置信度阈值: {perception.detector.get_confidence_threshold()}")

    # 重新配置
    new_config = PerceptionConfig(
        confidence_threshold=0.5,
        merge_distance=0.8
    )
    perception.configure(new_config)
    print(f"新置信度阈值: {perception.detector.get_confidence_threshold()}")

    # 重置系统
    perception.reset()
    print("系统已重置")


def example_6_error_handling():
    """示例6: 错误处理"""
    print("\n=== 示例6: 错误处理 ===")

    from perception.api.exceptions import (
        InvalidImageError,
        PerceptionAPIError,
    )

    perception = PerceptionFactory.create_perception()

    try:
        # 尝试处理无效图像
        invalid_image = np.zeros((100, 100), dtype=np.uint8)  # 错误：应该是3通道
        depth = np.zeros((100, 100), dtype=np.float32)
        camera_info = CameraInfo(fx=525.0, fy=525.0, cx=50, cy=50, width=100, height=100)

        perception.process_frame(invalid_image, depth, camera_info)

    except InvalidImageError as e:
        print(f"捕获到图像错误: {e}")

    except PerceptionAPIError as e:
        print(f"捕获到感知错误: {e}")


def example_7_factory_methods():
    """示例7: 工厂方法"""
    print("\n=== 示例7: 工厂方法 ===")

    # 查看可用的实现
    print(f"可用检测器: {PerceptionFactory.get_available_detectors()}")
    print(f"可用编码器: {PerceptionFactory.get_available_encoders()}")
    print(f"可用追踪器: {PerceptionFactory.get_available_trackers()}")

    # 从配置创建
    config = PerceptionConfig(
        detector_type="yolo_world",
        encoder_type="clip"
    )
    PerceptionFactory.create_from_config(config)
    print("从配置创建感知系统成功")


if __name__ == "__main__":
    print("Semantic Perception API 使用示例\n")

    try:
        example_1_basic_usage()
        example_2_component_usage()
        example_3_query_objects()
        example_4_statistics()
        example_5_configuration()
        example_6_error_handling()
        example_7_factory_methods()

        print("\n✅ 所有示例运行完成！")

    except Exception as e:
        print(f"\n❌ 示例运行失败: {e}")
        import traceback
        traceback.print_exc()
