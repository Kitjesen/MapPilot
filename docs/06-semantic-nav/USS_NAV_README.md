# USS-Nav 空间表示系统

基于 USS-Nav 论文的空间表示系统，融合到 lingtu 机器人导航框架中。

## 🎯 核心功能

### 1. 几何增强拓扑图
拓扑图节点包含完整几何信息（边界框、凸包、面积、高度），而不仅仅是质心点。

### 2. 混合路径规划器
分层路径规划：拓扑层（Dijkstra）+ 几何层（局部 A*），预估 3-10× 加速。

### 3. 多面体扩展
从局部滚动栅格生成多面体节点，无需全局稠密地图。

### 4. 空间连通图（SCG）
将多面体节点连接成拓扑图，支持三种拓扑边（Adjacency/Connectivity/Accessibility）。

### 5. Leiden 区域分割
基于图结构的社区检测，自动分割语义区域（走廊/房间）。

---

## 📦 安装

### 依赖库

```bash
# 核心依赖
pip install numpy scipy

# Leiden 聚类（可选，三选一）
pip install python-igraph  # 推荐
# 或
pip install networkx leidenalg
# 或使用内置的连通分量检测（无需额外依赖）
```

### 集成到 lingtu

所有代码已集成到 `src/semantic_perception/semantic_perception/` 目录：

```
semantic_perception/
├── geometry_extractor.py       # 几何提取器
├── hybrid_planner.py            # 混合路径规划器
├── polyhedron_expansion.py      # 多面体扩展
├── scg_builder.py               # SCG 构建器
├── leiden_segmentation.py       # Leiden 区域分割
└── topology_graph.py            # 拓扑图（已扩展）
```

---

## 🚀 快速开始

### 1. 几何增强拓扑图

```python
from semantic_perception.geometry_extractor import GeometryExtractor
from semantic_perception.topology_graph import TopologySemGraph

# 创建拓扑图
tsg = TopologySemGraph()

# 连接几何提取器
extractor = GeometryExtractor(tomogram)
tsg.set_geometry_extractor(extractor)

# 更新场景图（自动提取几何）
tsg.update_from_scene_graph(scene_graph_dict)

# 访问几何信息
room = tsg.get_node(room_id)
print(f"边界框: {room.bounding_box}")
print(f"可通行面积: {room.traversable_area:.2f} m²")
```

### 2. 混合路径规划器

```python
from semantic_perception.hybrid_planner import HybridPlanner

# 创建规划器
planner = HybridPlanner(topology_graph=tsg, tomogram=tomogram)

# 规划路径
start = np.array([0.0, 0.0, 0.0])
goal = np.array([10.0, 5.0, 0.0])

result = planner.plan_path(start, goal)

if result.success:
    print(f"房间序列: {result.room_sequence}")
    print(f"规划时间: {result.total_planning_time*1000:.2f}ms")
```

### 3. 多面体扩展

```python
from semantic_perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)

# 配置
config = PolyhedronExpansionConfig(
    num_sphere_samples=48,
    r_min=0.5,
    r_max=3.0,
    max_polyhedra=50,
)

# 扩展
expander = PolyhedronExpander(config)
polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

print(f"生成 {len(polyhedra)} 个多面体")
```

### 4. SCG 构建

```python
from semantic_perception.scg_builder import SCGBuilder, SCGConfig

# 创建 SCG
config = SCGConfig(adjacency_threshold=0.2)
builder = SCGBuilder(config)

# 添加多面体
for poly in polyhedra:
    builder.add_polyhedron(poly)

# 构建边
builder.build_edges(occupancy_grid, grid_resolution, grid_origin)

# 路径搜索
cost, path = builder.shortest_path(from_id=0, to_id=5)
```

### 5. Leiden 区域分割

```python
from semantic_perception.leiden_segmentation import (
    LeidenSegmenter,
    LeidenConfig,
)

# 配置
config = LeidenConfig(resolution=1.0, min_region_size=2)
segmenter = LeidenSegmenter(config)

# 分割
regions = segmenter.segment(scg_builder)

for region in regions:
    print(f"区域 {region.region_id} ({region.region_type})")
    print(f"  节点数: {len(region.node_ids)}")
    print(f"  体积: {region.volume:.2f} m³")
```

---

## 📚 完整示例

运行完整集成演示：

```bash
cd src/semantic_perception
python examples/uss_nav_integration_demo.py
```

这个演示展示了如何将所有组件组合在一起。

---

## 🧪 测试

运行单元测试：

```bash
cd src/semantic_perception

# 测试几何增强拓扑图
python tests/test_geometry_enhanced_topology.py

# 测试混合规划器
python tests/test_hybrid_planner.py

# 测试多面体扩展
python tests/test_polyhedron_expansion.py

# 测试 SCG 构建器
python tests/test_scg_builder.py
```

---

## 📖 文档

详细文档位于 `docs/` 目录：

### 设计文档
- `geometry_enhanced_topology_design.md` - 接口设计
- `polyhedron_expansion_algorithm.md` - 算法详解
- `hybrid_planner_usage.md` - 使用指南

### 总结报告
- `phase1_completion_summary.md` - 阶段 1 总结
- `phase2_progress_summary.md` - 阶段 2 总结
- `final_project_summary.md` - 项目总结

---

## 🎯 性能指标

| 模块 | 性能 | 说明 |
|------|------|------|
| 几何提取 | < 10ms/房间 | 单个房间几何信息提取 |
| 混合规划 | 3-10× 加速 | 相比全局 A* |
| 多面体扩展 | ~100ms | 局部栅格 (8×8×4m) |
| SCG 构建 | O(n²) | n 为多面体数量 |
| Leiden 分割 | < 100ms | 取决于节点数 |

---

## 🔧 配置参数

### 几何提取器

```python
GeometryExtractor(tomogram)
# 参数在 extract_room_geometry() 中设置：
# - search_radius: 搜索半径 (默认 5.0m)
# - cost_threshold: 可通行代价阈值 (默认 0.5)
```

### 混合规划器

```python
HybridPlanner(topology_graph, tomogram)
# 参数在 plan_path() 中设置：
# - search_radius_factor: 局部搜索半径因子 (默认 1.5)
# - max_planning_time: 最大规划时间 (默认 5.0s)
```

### 多面体扩展

```python
PolyhedronExpansionConfig(
    num_sphere_samples=48,      # 球面采样点数
    r_min=0.5,                  # 最小半径 (米)
    r_max=3.0,                  # 最大半径 (米)
    r_step=0.5,                 # 半径步长 (米)
    min_polyhedron_volume=1.0,  # 最小多面体体积 (立方米)
    max_polyhedra=50,           # 最大多面体数量
    coverage_threshold=0.8,     # 覆盖率阈值
)
```

### SCG 构建器

```python
SCGConfig(
    adjacency_threshold=0.2,        # 邻接阈值 (米)
    connectivity_samples=20,        # 连通性检测采样数
    loop_closure_threshold=0.5,     # 回环检测距离阈值 (米)
)
```

### Leiden 分割

```python
LeidenConfig(
    resolution=1.0,          # 分辨率参数 (越大越多社区)
    min_region_size=2,       # 最小区域大小 (节点数)
    use_weights=True,        # 是否使用边权重
)
```

---

## 🐛 故障排除

### 问题：几何提取失败

**原因**: Tomogram 未正确初始化或房间中心超出地图范围

**解决**:
```python
# 检查 Tomogram 参数
print(f"地图中心: {tomogram.map_center}")
print(f"地图尺寸: {tomogram.map_dim_x}×{tomogram.map_dim_y}")

# 检查房间中心是否在地图内
room_center = np.array([x, y])
grid_pos = extractor.world_to_grid(room_center)
print(f"栅格坐标: {grid_pos}")
```

### 问题：混合规划器返回失败

**原因**: 起点或终点不在任何房间内，或拓扑图中没有连通路径

**解决**:
```python
# 检查房间定位
start_room = planner._locate_room(start[:2])
goal_room = planner._locate_room(goal[:2])
print(f"起点房间: {start_room}, 终点房间: {goal_room}")

# 检查拓扑连通性
if start_room and goal_room:
    cost, path = planner.tsg.shortest_path(start_room, goal_room)
    print(f"拓扑路径: {path}, 代价: {cost}")
```

### 问题：Leiden 分割失败

**原因**: 缺少 igraph 或 leidenalg 库

**解决**:
```bash
# 安装 igraph (推荐)
pip install python-igraph

# 或安装 networkx + leidenalg
pip install networkx leidenalg

# 或使用内置的连通分量检测（无需额外依赖）
# 系统会自动回退到连通分量检测
```

---

## 🤝 贡献

欢迎贡献！请遵循以下步骤：

1. Fork 项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

---

## 📄 许可证

本项目遵循 lingtu 项目的许可证。

---

## 📧 联系

如有问题或建议，请通过以下方式联系：

- 项目主页: https://github.com/inovxio/brain/lingtu
- 问题追踪: https://github.com/inovxio/brain/lingtu/issues

---

## 🙏 致谢

- USS-Nav 论文作者
- lingtu 项目团队
- scipy、igraph 等开源库的维护者

---

**版本**: 1.0.0
**最后更新**: 2026-02-23
