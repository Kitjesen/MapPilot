# 混合路径规划器使用指南

## 概述

混合路径规划器 (HybridPlanner) 是一个分层路径规划系统，结合了拓扑图的高层连通性和 Tomogram 的几何精度。

## 核心思想

```
全局路径规划 = 拓扑层规划 + 几何层规划

拓扑层: 在拓扑图上做 Dijkstra → 房间序列 [R1, R2, ..., Rn]
几何层: 对每对相邻房间做局部 A* → 几何路径
```

## 优势

1. **减少搜索空间**: 局部 A* 只在房间对之间搜索，而非全局
2. **利用拓扑连通性**: 拓扑图提供高层路径指导
3. **性能提升**: 预估 3-10× 加速 (相比全局 A*)
4. **可解释性**: 路径以房间序列形式呈现，便于理解

## 快速开始

### 1. 创建混合规划器

```python
from semantic_perception.geometry_extractor import GeometryExtractor
from semantic_perception.hybrid_planner import HybridPlanner
from semantic_perception.topology_graph import TopologySemGraph

# 1. 创建拓扑图并连接几何提取器
tsg = TopologySemGraph()
extractor = GeometryExtractor(tomogram)
tsg.set_geometry_extractor(extractor)

# 2. 更新拓扑图 (从场景图)
tsg.update_from_scene_graph(scene_graph_dict)

# 3. 创建混合规划器
planner = HybridPlanner(topology_graph=tsg, tomogram=tomogram)
```

### 2. 规划路径

```python
# 定义起点和终点
start = np.array([0.0, 0.0, 0.0])  # [x, y, z]
goal = np.array([20.0, 10.0, 0.0])

# 规划路径
result = planner.plan_path(start, goal)

# 检查结果
if result.success:
    print(f"路径规划成功!")
    print(f"  房间序列: {result.room_sequence}")
    print(f"  路径点数: {result.num_waypoints}")
    print(f"  总代价: {result.total_cost:.2f}")
    print(f"  规划时间: {result.total_planning_time*1000:.2f}ms")
else:
    print("路径规划失败")
```

### 3. 访问路径详情

```python
# 完整路径点
for i, waypoint in enumerate(result.waypoints):
    print(f"路径点 {i}: {waypoint}")

# 路径段详情
for seg in result.segments:
    print(f"段: 房间 {seg.from_room_id} → {seg.to_room_id}")
    print(f"  路径点数: {len(seg.waypoints)}")
    print(f"  代价: {seg.cost:.2f}")
    print(f"  规划时间: {seg.planning_time*1000:.2f}ms")
```

## API 参考

### HybridPlanner

#### 构造函数

```python
HybridPlanner(topology_graph, tomogram)
```

**参数**:
- `topology_graph`: TopologySemGraph 实例
- `tomogram`: Tomogram 实例

#### plan_path()

```python
plan_path(
    start: np.ndarray,
    goal: np.ndarray,
    search_radius_factor: float = 1.5,
    max_planning_time: float = 5.0,
) -> HybridPath
```

**参数**:
- `start`: 起点 [x, y, z] (世界坐标)
- `goal`: 终点 [x, y, z] (世界坐标)
- `search_radius_factor`: 局部搜索半径因子 (默认 1.5)
- `max_planning_time`: 最大规划时间 (秒，默认 5.0)

**返回**: `HybridPath` 实例

### HybridPath

路径规划结果数据类。

**字段**:
- `success`: bool - 规划是否成功
- `waypoints`: List[np.ndarray] - 完整路径点序列
- `room_sequence`: List[int] - 房间序列
- `segments`: List[PathSegment] - 路径段列表
- `total_cost`: float - 总代价
- `total_planning_time`: float - 总规划时间 (秒)
- `topology_planning_time`: float - 拓扑规划时间 (秒)
- `geometry_planning_time`: float - 几何规划时间 (秒)
- `num_waypoints`: int - 路径点数量
- `num_rooms`: int - 经过的房间数

### PathSegment

路径段数据类。

**字段**:
- `from_room_id`: int - 起始房间 ID
- `to_room_id`: int - 目标房间 ID
- `waypoints`: List[np.ndarray] - 路径点序列
- `cost`: float - 路径代价
- `planning_time`: float - 规划时间 (秒)

## 性能对比

### compare_planners()

```python
from semantic_perception.hybrid_planner import compare_planners

# 定义测试用例
test_cases = [
    (np.array([0.0, 0.0, 0.0]), np.array([20.0, 10.0, 0.0])),
    (np.array([0.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0])),
]

# 对比性能
results = compare_planners(
    hybrid_planner=planner,
    baseline_planner=pct_planner,  # PCT Planner 或其他全局规划器
    test_cases=test_cases,
)

# 分析结果
for i, result in enumerate(results):
    print(f"测试用例 {i+1}:")
    print(f"  混合规划器: {result.hybrid_time*1000:.2f}ms")
    print(f"  基线规划器: {result.baseline_time*1000:.2f}ms")
    print(f"  加速比: {result.speedup:.2f}×")
```

## 高级用法

### 自定义搜索半径

```python
# 增大搜索半径 (更宽松的局部搜索)
result = planner.plan_path(
    start, goal,
    search_radius_factor=2.0,  # 默认 1.5
)
```

### 限制规划时间

```python
# 设置最大规划时间
result = planner.plan_path(
    start, goal,
    max_planning_time=2.0,  # 2 秒超时
)
```

### 访问中间结果

```python
# 拓扑层规划结果
print(f"拓扑规划时间: {result.topology_planning_time*1000:.2f}ms")
print(f"房间序列: {result.room_sequence}")

# 几何层规划结果
print(f"几何规划时间: {result.geometry_planning_time*1000:.2f}ms")
print(f"路径段数: {len(result.segments)}")
```

## 集成到 ROS

### 创建 ROS 节点

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class HybridPlannerNode(Node):
    def __init__(self):
        super().__init__('hybrid_planner_node')

        # 初始化混合规划器
        self.planner = HybridPlanner(tsg, tomogram)

        # 订阅目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # 发布路径
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

    def goal_callback(self, msg):
        # 获取当前位置
        start = self.get_current_pose()

        # 提取目标位置
        goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

        # 规划路径
        result = self.planner.plan_path(start, goal)

        if result.success:
            # 转换为 ROS Path 消息
            path_msg = self.convert_to_path_msg(result.waypoints)
            self.path_pub.publish(path_msg)

            self.get_logger().info(
                f"Path planned: {result.num_rooms} rooms, "
                f"{result.num_waypoints} waypoints, "
                f"{result.total_planning_time*1000:.2f}ms"
            )
        else:
            self.get_logger().warn("Path planning failed")
```

## 注意事项

### 当前限制

1. **简化的局部 A***: 当前实现使用直线路径作为简化版本，实际应用需要集成 PCT Planner 的 A* 实现
2. **2D 规划**: 当前主要在 2D 平面规划，未充分利用 3D 信息
3. **静态环境**: 假设环境静态，不处理动态障碍物

### 未来改进

1. **集成 PCT A***: 将 PCT Planner 的 A* 实现集成到局部规划中
2. **3D 规划**: 利用 Tomogram 的多层高程信息进行 3D 规划
3. **动态重规划**: 支持动态障碍物和实时重规划
4. **并行规划**: 并行规划多个路径段以提升性能
5. **路径平滑**: 添加路径平滑算法以提升路径质量

## 故障排除

### 问题: 规划失败 (success=False)

**可能原因**:
1. 起点或终点不在任何房间内
2. 拓扑图中没有连通路径
3. 局部 A* 失败

**解决方案**:
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

### 问题: 规划时间过长

**可能原因**:
1. 搜索半径过大
2. 房间数量过多
3. 局部 A* 搜索空间过大

**解决方案**:
```python
# 减小搜索半径
result = planner.plan_path(start, goal, search_radius_factor=1.2)

# 设置超时
result = planner.plan_path(start, goal, max_planning_time=2.0)
```

### 问题: 路径质量差

**可能原因**:
1. 拓扑图不准确
2. 房间几何信息缺失
3. 局部 A* 使用简化版本

**解决方案**:
```python
# 检查房间几何信息
for room in planner.tsg.rooms:
    print(f"房间 {room.name}:")
    print(f"  边界框: {room.bounding_box}")
    print(f"  凸包: {room.convex_hull is not None}")
    print(f"  几何置信度: {room.geometry_confidence}")

# 更新拓扑图
planner.tsg.update_from_scene_graph(latest_scene_graph)
```

## 示例代码

完整示例请参考:
- `tests/test_hybrid_planner.py` - 单元测试
- `examples/hybrid_planner_demo.py` - 演示脚本 (待创建)

## 相关文档

- [几何增强拓扑图设计文档](geometry_enhanced_topology_design.md)
- [阶段 1 完成总结](phase1_completion_summary.md)
- [项目进度报告](project_progress_report.md)
