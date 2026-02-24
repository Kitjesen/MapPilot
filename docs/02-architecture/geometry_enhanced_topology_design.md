# 几何增强拓扑图 - 接口设计文档

## 1. 现状分析

### 1.1 TopoNode 当前结构
```python
@dataclass
class TopoNode:
    node_id: int
    node_type: str                      # "room" | "frontier"
    name: str
    center: np.ndarray                  # [x, y] - 仅质心点
    room_type: str = "unknown"
    semantic_labels: List[str]

    # 探索状态
    visited: bool
    visit_count: int
    last_visited: float
    objects_found: int
```

**问题**: 节点只有质心点，缺乏几何形状信息，无法用于路径规划。

### 1.2 Tomogram 数据结构
```python
class Tomogram:
    # 多层高程地图
    layers_g: np.ndarray  # (n_slice, map_dim_x, map_dim_y) - 地面高程
    layers_c: np.ndarray  # (n_slice, map_dim_x, map_dim_y) - 天花板高程

    # 可通行性
    trav_cost: np.ndarray      # (n_slice, map_dim_x, map_dim_y) - 可通行代价
    inflated_cost: np.ndarray  # (n_slice, map_dim_x, map_dim_y) - 膨胀后代价

    # 地图参数
    center: np.ndarray  # [x, y] - 地图中心
    resolution: float   # 栅格分辨率 (米/格)
    map_dim_x: int      # X 维度格数
    map_dim_y: int      # Y 维度格数
    n_slice_init: int   # 高度层数
    slice_h0: float     # 起始高度
    slice_dh: float     # 层间距
```

**能力**: 提供全局点云转换的多层可通行性栅格，可用于提取房间几何。

---

## 2. 扩展设计

### 2.1 扩展 TopoNode 数据结构

```python
@dataclass
class TopoNode:
    # ===== 原有字段 (保持不变) =====
    node_id: int
    node_type: str
    name: str
    center: np.ndarray                  # [x, y]
    room_type: str = "unknown"
    semantic_labels: List[str] = field(default_factory=list)
    visited: bool = False
    visit_count: int = 0
    last_visited: float = 0.0
    objects_found: int = 0
    frontier_direction: Optional[np.ndarray] = None
    frontier_size: float = 0.0
    predicted_room_type: str = ""

    # ===== 新增几何字段 =====
    # 边界框 (AABB - Axis-Aligned Bounding Box)
    bounding_box: Optional[Dict[str, float]] = None
    # 格式: {"x_min": float, "x_max": float, "y_min": float, "y_max": float}

    # 凸包 (2D 多边形顶点)
    convex_hull: Optional[np.ndarray] = None
    # 格式: (N, 2) 数组，顺时针或逆时针排列的顶点

    # 可通行区域面积 (平方米)
    traversable_area: float = 0.0

    # 高度范围 (米)
    height_range: Optional[Dict[str, float]] = None
    # 格式: {"floor": float, "ceiling": float}

    # 几何质量指标
    geometry_confidence: float = 0.0
    # 0.0-1.0，表示几何信息的可靠性

    # 最后几何更新时间
    geometry_updated: float = 0.0
```

**设计原则**:
1. **向后兼容**: 所有新字段都是 Optional 或有默认值
2. **轻量级**: 只存储关键几何信息，不存储完整栅格
3. **可序列化**: 所有字段都可转换为 JSON

---

### 2.2 GeometryExtractor 类设计

```python
class GeometryExtractor:
    """从 Tomogram 提取房间几何信息的工具类。"""

    def __init__(self, tomogram: Tomogram):
        """
        Args:
            tomogram: PCT Tomogram 实例
        """
        self.tomogram = tomogram
        self.resolution = tomogram.resolution
        self.map_center = tomogram.center

    def extract_room_geometry(
        self,
        room_center: np.ndarray,
        search_radius: float = 5.0,
        cost_threshold: float = 0.5,
    ) -> Dict[str, Any]:
        """
        提取房间的几何信息。

        算法流程:
        1. 在 Tomogram 中定位房间中心对应的栅格坐标
        2. 在 search_radius 范围内提取可通行栅格 (trav_cost < threshold)
        3. 计算边界框 (AABB)
        4. 生成凸包 (QuickHull / scipy.spatial.ConvexHull)
        5. 计算可通行面积
        6. 提取高度范围 (从 layers_g 和 layers_c)

        Args:
            room_center: 房间中心 [x, y] (世界坐标)
            search_radius: 搜索半径 (米)
            cost_threshold: 可通行代价阈值 (0-1)

        Returns:
            {
                "bounding_box": {"x_min": ..., "x_max": ..., "y_min": ..., "y_max": ...},
                "convex_hull": np.ndarray (N, 2),
                "traversable_area": float,
                "height_range": {"floor": float, "ceiling": float},
                "confidence": float,
            }
        """
        pass

    def world_to_grid(self, world_pos: np.ndarray) -> Tuple[int, int]:
        """世界坐标 → 栅格坐标。"""
        pass

    def grid_to_world(self, grid_x: int, grid_y: int) -> np.ndarray:
        """栅格坐标 → 世界坐标。"""
        pass

    def extract_traversable_cells(
        self,
        center_grid: Tuple[int, int],
        radius_cells: int,
        cost_threshold: float,
    ) -> List[Tuple[int, int]]:
        """提取可通行栅格单元。"""
        pass

    def compute_bounding_box(
        self,
        cells: List[Tuple[int, int]],
    ) -> Dict[str, float]:
        """计算边界框 (AABB)。"""
        pass

    def compute_convex_hull(
        self,
        cells: List[Tuple[int, int]],
    ) -> np.ndarray:
        """计算凸包 (使用 scipy.spatial.ConvexHull)。"""
        pass

    def compute_traversable_area(
        self,
        cells: List[Tuple[int, int]],
    ) -> float:
        """计算可通行面积 (平方米)。"""
        pass

    def extract_height_range(
        self,
        center_grid: Tuple[int, int],
        radius_cells: int,
    ) -> Dict[str, float]:
        """提取高度范围 (从 layers_g 和 layers_c)。"""
        pass
```

---

### 2.3 集成到 TopologySemGraph

在 `TopologySemGraph.update_from_scene_graph()` 中集成几何提取：

```python
class TopologySemGraph:
    def __init__(self):
        # ... 原有字段 ...
        self._geometry_extractor: Optional[GeometryExtractor] = None

    def set_geometry_extractor(self, tomogram: Tomogram) -> None:
        """设置几何提取器 (连接到 Tomogram)。"""
        self._geometry_extractor = GeometryExtractor(tomogram)

    def update_from_scene_graph(self, sg: Dict) -> None:
        """从场景图同步拓扑图 (增强版 - 包含几何提取)。"""
        rooms = sg.get("rooms", [])
        topology_edges = sg.get("topology_edges", [])

        # ... 原有逻辑 ...

        for room in rooms:
            rid = room.get("room_id", -1)
            if rid < 0:
                continue

            center = np.array([
                room.get("center", {}).get("x", 0.0),
                room.get("center", {}).get("y", 0.0),
            ])

            # 创建节点
            node = TopoNode(
                node_id=rid,
                node_type="room",
                name=room.get("name", f"room_{rid}"),
                center=center,
                room_type=self._infer_room_type(room.get("name", "")),
                semantic_labels=room.get("semantic_labels", [])[:12],
            )

            # ===== 新增: 提取几何信息 =====
            if self._geometry_extractor is not None:
                try:
                    geometry = self._geometry_extractor.extract_room_geometry(
                        room_center=center,
                        search_radius=5.0,
                        cost_threshold=0.5,
                    )
                    node.bounding_box = geometry["bounding_box"]
                    node.convex_hull = geometry["convex_hull"]
                    node.traversable_area = geometry["traversable_area"]
                    node.height_range = geometry["height_range"]
                    node.geometry_confidence = geometry["confidence"]
                    node.geometry_updated = time.time()
                except Exception as e:
                    logger.warning(f"Failed to extract geometry for room {rid}: {e}")

            # 恢复访问状态
            if rid in existing_visits:
                node.visited, node.visit_count, node.last_visited, node.objects_found = (
                    existing_visits[rid]
                )

            self._nodes[rid] = node

        # ... 原有边重建逻辑 ...
```

---

## 3. 使用示例

### 3.1 初始化

```python
from topology_graph import TopologySemGraph
from geometry_extractor import GeometryExtractor
from PCT_planner.tomography.scripts.tomogram import Tomogram

# 1. 加载 Tomogram
tomogram = Tomogram(cfg)
tomogram.initMappingEnv(center=[0, 0], map_dim_x=200, map_dim_y=200,
                        n_slice_init=10, slice_h0=0.0)
tomogram.point2map(point_cloud)

# 2. 创建拓扑图并连接几何提取器
tsg = TopologySemGraph()
tsg.set_geometry_extractor(tomogram)

# 3. 从场景图更新 (自动提取几何)
tsg.update_from_scene_graph(scene_graph_dict)
```

### 3.2 查询几何信息

```python
# 获取房间节点
room_node = tsg.get_node(room_id=1)

# 访问几何信息
if room_node.bounding_box:
    print(f"边界框: {room_node.bounding_box}")
    print(f"可通行面积: {room_node.traversable_area:.2f} m²")
    print(f"高度范围: {room_node.height_range}")
    print(f"凸包顶点数: {len(room_node.convex_hull)}")
```

### 3.3 序列化 (扩展 to_dict)

```python
def to_dict(self) -> Dict:
    """导出为可 JSON 序列化的字典 (包含几何信息)。"""
    nodes_list = []
    for n in self._nodes.values():
        d = {
            "node_id": n.node_id,
            "node_type": n.node_type,
            "name": n.name,
            "center": {"x": round(float(n.center[0]), 2), "y": round(float(n.center[1]), 2)},
            "room_type": n.room_type,
            "visited": n.visited,
            "visit_count": n.visit_count,
        }

        # ===== 新增: 几何字段序列化 =====
        if n.bounding_box:
            d["bounding_box"] = n.bounding_box
        if n.convex_hull is not None:
            d["convex_hull"] = n.convex_hull.tolist()
        if n.traversable_area > 0:
            d["traversable_area"] = round(n.traversable_area, 2)
        if n.height_range:
            d["height_range"] = n.height_range
        if n.geometry_confidence > 0:
            d["geometry_confidence"] = round(n.geometry_confidence, 2)

        # ... 原有前沿字段 ...
        nodes_list.append(d)

    # ... 原有边和其他字段 ...
    return {"nodes": nodes_list, "edges": edges_list, ...}
```

---

## 4. 性能考虑

### 4.1 计算开销
- **几何提取**: 每个房间约 5-10ms (取决于搜索半径)
- **凸包计算**: scipy.spatial.ConvexHull 对 <1000 点约 1-2ms
- **总开销**: 10 个房间约 50-100ms，可接受

### 4.2 内存占用
- **凸包**: 每个房间约 50-200 个顶点 × 8 字节 × 2 = 0.8-3.2 KB
- **10 个房间**: 约 8-32 KB，可忽略

### 4.3 更新策略
- **增量更新**: 只在房间首次发现或 Tomogram 重建时更新几何
- **缓存**: 几何信息缓存在 TopoNode 中，避免重复计算
- **按需计算**: 如果不需要几何信息，可以不调用 `set_geometry_extractor()`

---

## 5. 下一步: 混合路径规划器

有了几何增强的拓扑图后，可以实现 **HybridPlanner**：

```python
class HybridPlanner:
    """拓扑图辅助的混合路径规划器。"""

    def plan_path(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        tsg: TopologySemGraph,
        tomogram: Tomogram,
    ) -> List[np.ndarray]:
        """
        混合规划算法:
        1. 在拓扑图上做 Dijkstra → 获得房间序列 [R1, R2, R3]
        2. 对每对相邻房间 (R1→R2)，在 Tomogram 上做局部 A*
        3. 拼接路径

        优势:
        - 减少 A* 搜索空间 (只在房间对之间搜索)
        - 利用拓扑图的高层连通性
        - 比全局 A* 快 3-10 倍
        """
        pass
```

---

## 6. 总结

### 6.1 本阶段目标
✅ 扩展 TopoNode 添加几何字段 (向后兼容)
✅ 实现 GeometryExtractor 从 Tomogram 提取几何
✅ 集成到 TopologySemGraph 的更新流程
✅ 保持现有语义探索功能不受影响

### 6.2 下一阶段预览
- 实现 HybridPlanner (拓扑 + A* 混合规划)
- 性能对比测试 (vs 纯 PCT A*)
- 为引入多面体扩展做准备

### 6.3 风险评估
- **低风险**: 所有修改都是增量式的，不影响现有功能
- **可回退**: 如果不设置 GeometryExtractor，系统行为与原来完全一致
- **测试友好**: 可以逐步启用几何功能，对比效果
