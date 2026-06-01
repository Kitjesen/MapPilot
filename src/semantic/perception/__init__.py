"""VLN semantic perception package. See MODULES.md for full module descriptions."""

# --- 根节点 ---
# perception_node.py        ROS2 主节点，驱动完整感知流水线
# perception_publishers.py  ROS2 话题统一发布出口

# --- Detector 目标检测 (detector_base / yolo_world_detector / bpu_detector /
#         yoloe_detector / grounding_dino_detector) ---
# detector_base.py              DetectorBase 抽象接口
# yolo_world_detector.py        YOLO-World 开放词汇，125 类，10-15 FPS
# bpu_detector.py               Nash BPU YOLO11s-seg，~45ms/帧，实例分割
# yoloe_detector.py             YOLOE 轻量备选后端
# grounding_dino_detector.py    高精度慢速，离线标注 / slow path 验证

# --- Encoder 特征编码 (clip_encoder / mobileclip_encoder / projection) ---
# clip_encoder.py       HOV-SG encode_three_source (f_g+f_l+f_m)，LRU 缓存
# mobileclip_encoder.py 轻量 MobileCLIP，适配低延迟场景
# projection.py         2D→3D 投影，TF 变换封装

# --- Tracker 实例跟踪 (instance_tracker / tracked_objects / keyframe_selector / bpu_tracker) ---
# instance_tracker.py   场景图核心：HOV-SG RoomNode + DBSCAN 精化 + TrackedObject 管理
# tracked_objects.py    TrackedObject 数据结构，CLIP FIFO，置信度衰减
# keyframe_selector.py  关键帧选择，控制 CLIP 编码频率
# bpu_tracker.py        BPU 专用跟踪器，与 bpu_detector 配套

# --- SceneGraph 场景图 (scg_builder / scg_path_planner / hybrid_planner) ---
# scg_builder.py        物体节点 + 关系边，增量更新，JSON 序列化
# scg_path_planner.py   基于语义关系的可达性搜索
# hybrid_planner.py     语义场景图 + 几何 costmap 联合规划

# --- Topology 拓扑图 (topology_graph / topology_types / room_manager / leiden_segmentation) ---
# topology_graph.py         ViewNode + RoomNode，跨房间可达性
# topology_types.py             ViewNode / RoomNode / TopoEdge 数据类型
# room_manager.py           房间边界估计，房间-物体归属，动态合并/分裂
# leiden_segmentation.py    Leiden 图聚类 → 语义房间区域

# --- Belief (belief_propagation) ---
# belief_propagation.py  Loopy BP scene-graph semantic label inference (optional)

# --- KnowledgeGraph 知识图谱 (knowledge_graph / knowledge_data / bpu_qp_bridge) ---
# knowledge_graph.py    室内场景本体，三元组，SPARQL 风格查询
# knowledge_data.py            预定义分类 + 属性词典 + 房间-物体先验 (1944 行)
# bpu_qp_bridge.py      BPU 量化输出 → 知识图谱置信度校准

# --- Geometry 几何处理 (geometry_extractor / polyhedron_expansion / laplacian_filter / local_rolling_grid) ---
# geometry_extractor.py     点云 PCA + 包围盒 + 法向量
# polyhedron_expansion.py   障碍物安全边界膨胀
# laplacian_filter.py       激光点云离群噪声滤波
# local_rolling_grid.py     以机器人为中心的滑动窗口占据栅格

# --- Coverage 覆盖与不确定性 (global_coverage_mask / uncertainty_model / perception_pipeline) ---
# global_coverage_mask.py   已探索区域掩码，引导 frontier
# uncertainty_model.py      感知置信度 + 地图不确定性，输出 VOI 信号
# perception_pipeline.py    检测→投影→跟踪→场景图完整调用链封装

# --- Evaluation 评测框架 (evaluation_framework / dataset_loader / baseline_wrappers / end_to_end_evaluation) ---
# evaluation_framework.py   precision/recall/F1，多后端对比
# dataset_loader.py         HM3D / ScanQA / 自定义格式，统一 Episode 接口
# baseline_wrappers.py      CLIP-Nav 等 baseline 统一调用接口
# end_to_end_evaluation.py  SR / SPL / SoftSPL，NaviMind 论文数据来源

# --- Visualization 可视化 (visualization_tools) ---
# visualization_tools.py    场景图 RViz marker + 检测框 + Perception Viewer MJPEG
