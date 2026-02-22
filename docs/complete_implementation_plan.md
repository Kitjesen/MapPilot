# USS-Nav 完整实现计划 - 弥补劣势

**制定日期**: 2026-02-23
**目标**: 将原型升级为论文级别的完整实现

---

## 📚 房间拓扑图的来源

### 原始设计来源

**文件**: `topology_graph.py`

**参考论文**:
1. **Hydra** (RSS 2022) - MIT-SPARK
   - 层次3D场景图
   - Places → Rooms 社区检测
   - 实时拓扑连通

2. **TopoNav** (2025)
   - 拓扑图作为空间记忆
   - 连接性 + 邻接性 + 语义

3. **L3MVN** (IROS 2023)
   - LLM-guided 拓扑探索
   - Frontier 评分

4. **SG-Nav** (NeurIPS 2024)
   - 子图推理 + LLM 常识
   - Frontier 插值

5. **TACS-Graphs** (2025)
   - 可通行性感知场景图
   - 一致性房间分割

6. **Concept-Guided Exploration** (2025)
   - Room + Door 自治概念
   - 层次约束传播

### 核心创新

**创新5: Topology-Aware Information Gain Exploration**

将房间拓扑图从"连通性描述"升级为"探索决策引擎":
1. 语义联想边: room_type → expected_objects (带概率)
2. 前沿节点 (Frontier): 已知空间边界处的未探索方向
3. 穿越记忆 (Traversal Memory): 记录机器人实际路径
4. 信息增益评分: IG(node) = semantic_prior × novelty × reachability_decay
5. 图上最短路径: Dijkstra 选择最优探索目标

### 数据结构

```python
@dataclass
class TopoNode:
    """拓扑图节点 (房间 或 前沿)"""
    node_id: int
    node_type: str  # "room" | "frontier"
    center: np.ndarray  # [x, y]
    room_type: str  # corridor, office, kitchen, ...

    # 探索状态
    visited: bool
    visit_count: int
    objects_found: int

    # USS-Nav 几何信息
    bounding_box: Dict[str, float]
    convex_hull: np.ndarray
    traversable_area: float
    height_range: Dict[str, float]
    geometry_confidence: float
```

---

## 🎯 完整实现计划

### 阶段 1: 实现缺失的核心组件 (3-4 周)

#### 任务 1: 实现 GCM (全局覆盖掩码) ⭐⭐⭐

**目标**: 替代全局点云，轻量级探索追踪

**设计**:
```python
class GlobalCoverageMask:
    """
    全局覆盖掩码 (GCM) - 替代全局点云的轻量级表示

    特点:
    - 稀疏存储（只存储已探索区域）
    - 粗粒度（0.5-1.0m 分辨率）
    - 内存固定（不随时间增长）
    """

    def __init__(self, resolution=0.5):
        self.resolution = resolution
        self.coverage_map = {}  # 稀疏字典: (x, y) -> CoverageCell
        self.total_cells = 0
        self.covered_cells = 0

    def update_from_polyhedron(self, polyhedron):
        """从多面体更新覆盖区域"""
        # 1. 计算多面体投影到 2D 的覆盖区域
        # 2. 栅格化为 GCM 单元
        # 3. 更新覆盖状态
        pass

    def update_from_local_grid(self, local_grid, robot_pose):
        """从局部滚动栅格更新覆盖区域"""
        # 1. 提取自由空间
        # 2. 转换到全局坐标
        # 3. 更新 GCM
        pass

    def get_coverage_ratio(self):
        """计算探索覆盖率"""
        return self.covered_cells / max(self.total_cells, 1)

    def get_frontier_cells(self):
        """提取前沿单元（已知与未知的边界）"""
        frontiers = []
        for (x, y), cell in self.coverage_map.items():
            if cell.is_frontier():
                frontiers.append((x, y))
        return frontiers

    def save(self, filepath):
        """保存 GCM（稀疏格式）"""
        pass

    def load(self, filepath):
        """加载 GCM"""
        pass

@dataclass
class CoverageCell:
    """GCM 单元"""
    x: int
    y: int
    covered: bool = False
    visit_count: int = 0
    last_visited: float = 0.0
    uncertainty: float = 1.0  # 0.0-1.0

    def is_frontier(self):
        """判断是否为前沿单元"""
        # 已覆盖且邻居有未覆盖的
        return self.covered and self.has_uncovered_neighbors()
```

**实现步骤**:
1. 实现 `GlobalCoverageMask` 类
2. 实现 `CoverageCell` 数据结构
3. 实现从多面体更新的逻辑
4. 实现从局部栅格更新的逻辑
5. 实现前沿检测
6. 实现覆盖率计算
7. 实现序列化/反序列化
8. 编写单元测试

**预计时间**: 1 周

---

#### 任务 2: 实现 SCG-based 路径规划器 ⭐⭐⭐

**目标**: 纯基于 SCG 的路径规划，不依赖全局 Tomogram

**设计**:
```python
class SCGPathPlanner:
    """
    SCG-based 路径规划器

    特点:
    - 在 SCG 上搜索多面体序列
    - 在多面体内部和边界生成路径
    - 不依赖全局栅格地图
    """

    def __init__(self, scg_builder):
        self.scg = scg_builder
        self.path_smoother = PathSmoother()

    def plan(self, start, goal):
        """规划路径"""
        # 1. 定位起点和终点所在的多面体
        start_poly = self._locate_polyhedron(start)
        goal_poly = self._locate_polyhedron(goal)

        if start_poly is None or goal_poly is None:
            return None

        # 2. 在 SCG 上 A* 搜索多面体序列
        poly_sequence = self._search_polyhedron_sequence(
            start_poly, goal_poly
        )

        if poly_sequence is None:
            return None

        # 3. 生成穿过多面体序列的路径
        path = self._generate_path_through_polyhedra(
            poly_sequence, start, goal
        )

        # 4. 路径平滑
        smoothed_path = self.path_smoother.smooth(path)

        return smoothed_path

    def _locate_polyhedron(self, point):
        """定位点所在的多面体"""
        for poly_id, poly in self.scg.polyhedra.items():
            if self._point_in_polyhedron(point, poly):
                return poly_id
        return None

    def _search_polyhedron_sequence(self, start_id, goal_id):
        """A* 搜索多面体序列"""
        # 使用 SCG 的边（Adjacency/Connectivity/Accessibility）
        # 启发式: 欧氏距离
        pass

    def _generate_path_through_polyhedra(self, poly_sequence, start, goal):
        """生成穿过多面体序列的路径"""
        path = [start]

        for i in range(len(poly_sequence) - 1):
            # 找到两个多面体的连接点
            connection = self._find_connection(
                poly_sequence[i],
                poly_sequence[i+1]
            )
            path.append(connection)

        path.append(goal)
        return np.array(path)

    def _find_connection(self, poly1_id, poly2_id):
        """找到两个多面体的连接点"""
        # 1. 检查是否有 Adjacency 边（共享面）
        # 2. 如果有，返回共享面的中心
        # 3. 如果没有，返回两个中心的中点
        pass

class PathSmoother:
    """路径平滑器"""

    def smooth(self, path, iterations=10):
        """平滑路径（梯度下降）"""
        smoothed = path.copy()

        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                # 平滑: 向邻居的平均位置移动
                smoothed[i] = 0.5 * smoothed[i] + 0.25 * (
                    smoothed[i-1] + smoothed[i+1]
                )

        return smoothed
```

**实现步骤**:
1. 实现 `SCGPathPlanner` 类
2. 实现多面体定位逻辑
3. 实现 A* 搜索（在 SCG 上）
4. 实现路径生成逻辑
5. 实现路径平滑
6. 实现碰撞检测（可选）
7. 编写单元测试
8. 与混合规划器对比测试

**预计时间**: 1.5 周

---

#### 任务 3: 实现不确定性建模 ⭐⭐

**目标**: 为多面体和 GCM 添加不确定性度量

**设计**:
```python
class UncertaintyModel:
    """
    不确定性建模

    用途:
    - 评估多面体的不确定性
    - 指导探索决策
    - 计算信息增益
    """

    def __init__(self):
        self.entropy_threshold = 0.5

    def compute_polyhedron_uncertainty(self, polyhedron, occupancy_grid):
        """计算多面体的不确定性"""
        # 1. 在多面体内部采样点
        # 2. 查询占据栅格的不确定性
        # 3. 计算平均熵

        samples = self._sample_points_in_polyhedron(polyhedron, n=100)
        uncertainties = []

        for point in samples:
            # 查询占据概率
            p_occupied = self._query_occupancy_probability(
                point, occupancy_grid
            )
            # 计算熵: H = -p*log(p) - (1-p)*log(1-p)
            entropy = self._compute_entropy(p_occupied)
            uncertainties.append(entropy)

        return np.mean(uncertainties)

    def compute_gcm_uncertainty(self, gcm):
        """计算 GCM 的整体不确定性"""
        total_uncertainty = 0.0
        for cell in gcm.coverage_map.values():
            total_uncertainty += cell.uncertainty

        return total_uncertainty / len(gcm.coverage_map)

    def select_exploration_target(self, scg, gcm):
        """选择探索目标（最大信息增益）"""
        best_target = None
        max_gain = -float('inf')

        for poly_id, poly in scg.polyhedra.items():
            # 计算信息增益
            gain = self._compute_information_gain(poly, gcm)

            if gain > max_gain:
                max_gain = gain
                best_target = poly_id

        return best_target

    def _compute_information_gain(self, polyhedron, gcm):
        """计算信息增益"""
        # IG = uncertainty × novelty × reachability
        uncertainty = polyhedron.uncertainty
        novelty = 1.0 - polyhedron.visit_count / 10.0
        reachability = 1.0 / (1.0 + polyhedron.distance_to_robot)

        return uncertainty * novelty * reachability

    @staticmethod
    def _compute_entropy(p):
        """计算二元熵"""
        if p <= 0 or p >= 1:
            return 0.0
        return -p * np.log2(p) - (1-p) * np.log2(1-p)
```

**实现步骤**:
1. 实现 `UncertaintyModel` 类
2. 实现多面体不确定性计算
3. 实现 GCM 不确定性计算
4. 实现信息增益计算
5. 实现探索目标选择
6. 集成到 SCG 和 GCM
7. 编写单元测试

**预计时间**: 1 周

---

#### 任务 4: 实现局部滚动栅格 ⭐⭐

**目标**: 替代全局点云，固定内存的局部地图

**设计**:
```python
class LocalRollingGrid:
    """
    局部滚动栅格

    特点:
    - 固定大小（8×8×4m）
    - 跟随机器人移动
    - 内存不增长
    """

    def __init__(self, size=(8.0, 8.0, 4.0), resolution=0.1):
        self.size = size  # (x, y, z) in meters
        self.resolution = resolution

        # 栅格尺寸
        self.grid_size = (
            int(size[0] / resolution),
            int(size[1] / resolution),
            int(size[2] / resolution),
        )

        # 占据栅格
        self.occupancy = np.zeros(self.grid_size, dtype=np.float32)

        # 当前中心位置（世界坐标）
        self.center = np.array([0.0, 0.0, 0.0])

    def update(self, robot_pose, point_cloud):
        """更新局部栅格"""
        # 1. 如果机器人移动超过阈值，滚动栅格
        if self._should_roll(robot_pose):
            self._roll_grid(robot_pose)

        # 2. 更新占据栅格
        self._update_occupancy(point_cloud)

    def _should_roll(self, robot_pose):
        """判断是否需要滚动"""
        distance = np.linalg.norm(robot_pose[:2] - self.center[:2])
        return distance > self.size[0] / 4  # 移动超过 1/4 大小

    def _roll_grid(self, new_center):
        """滚动栅格到新中心"""
        # 1. 计算偏移量
        offset = new_center - self.center

        # 2. 平移栅格内容
        # 使用 numpy.roll 或重新分配

        # 3. 清空新区域

        self.center = new_center

    def _update_occupancy(self, point_cloud):
        """更新占据栅格"""
        # 1. 将点云转换到栅格坐标
        # 2. 更新占据概率
        # 3. 使用贝叶斯更新
        pass

    def get_occupancy_grid(self):
        """获取占据栅格"""
        return self.occupancy

    def world_to_grid(self, world_pos):
        """世界坐标 → 栅格坐标"""
        relative_pos = world_pos - self.center
        grid_pos = (relative_pos / self.resolution +
                   np.array(self.grid_size) / 2).astype(int)
        return grid_pos
```

**实现步骤**:
1. 实现 `LocalRollingGrid` 类
2. 实现栅格滚动逻辑
3. 实现占据更新（贝叶斯）
4. 实现坐标转换
5. 集成到多面体扩展
6. 测试内存占用
7. 编写单元测试

**预计时间**: 0.5 周

---

### 阶段 2: 数据集集成与基线对比 (3-4 周)

#### 任务 5: 集成 HM3D 数据集 ⭐⭐⭐

**目标**: 在标准数据集上测试

**步骤**:
1. 下载 HM3D 数据集
2. 编写数据加载器
3. 转换为 lingtu 格式
4. 创建测试场景
5. 运行完整流程

**预计时间**: 1 周

---

#### 任务 6: 实现 PCT A* 基线 ⭐⭐

**目标**: 标准化 PCT A* 接口用于对比

**步骤**:
1. 封装现有 PCT A* 实现
2. 统一接口
3. 添加性能监控
4. 编写测试脚本

**预计时间**: 0.5 周

---

#### 任务 7: 集成 Hydra 基线 ⭐⭐⭐

**目标**: 与 Hydra 对比

**步骤**:
1. 安装 Hydra
2. 编写接口适配器
3. 转换数据格式
4. 运行对比实验

**预计时间**: 1.5 周

---

#### 任务 8: 集成 Voxblox 基线 ⭐⭐

**目标**: 与 Voxblox 对比

**步骤**:
1. 安装 Voxblox
2. 编写接口适配器
3. 运行对比实验

**预计时间**: 1 周

---

### 阶段 3: 性能评估与实验 (3-4 周)

#### 任务 9: 内存占用测试 ⭐⭐⭐

**设计**:
```python
class MemoryProfiler:
    """内存占用分析器"""

    def profile_long_run(self, system, duration=3600):
        """长时间运行内存测试"""
        memory_log = []
        start_time = time.time()

        while time.time() - start_time < duration:
            # 记录内存占用
            mem_usage = self._get_memory_usage()
            memory_log.append({
                'time': time.time() - start_time,
                'memory_mb': mem_usage,
                'num_polyhedra': len(system.scg.polyhedra),
                'gcm_cells': len(system.gcm.coverage_map),
            })

            # 模拟探索
            system.step()

            time.sleep(1.0)

        return memory_log

    def plot_memory_curve(self, memory_log):
        """绘制内存曲线"""
        import matplotlib.pyplot as plt

        times = [log['time'] for log in memory_log]
        memories = [log['memory_mb'] for log in memory_log]

        plt.plot(times, memories)
        plt.xlabel('Time (s)')
        plt.ylabel('Memory (MB)')
        plt.title('Memory Usage Over Time')
        plt.savefig('memory_profile.png')
```

**预计时间**: 1 周

---

#### 任务 10: 更新频率测试 ⭐⭐

**设计**:
```python
class UpdateRateProfiler:
    """更新频率分析器"""

    def measure_update_rate(self, system, num_updates=1000):
        """测量更新频率"""
        update_times = []

        for i in range(num_updates):
            start = time.time()
            system.update(sensor_data)
            elapsed = time.time() - start
            update_times.append(elapsed)

        avg_time = np.mean(update_times)
        update_rate = 1.0 / avg_time

        return {
            'avg_update_time': avg_time,
            'update_rate_hz': update_rate,
            'min_time': np.min(update_times),
            'max_time': np.max(update_times),
        }
```

**预计时间**: 0.5 周

---

#### 任务 11: 路径质量评估 ⭐⭐⭐

**设计**:
```python
class PathQualityEvaluator:
    """路径质量评估器"""

    def evaluate(self, path, ground_truth_path):
        """评估路径质量"""
        metrics = {}

        # 1. 路径长度
        metrics['path_length'] = self._compute_path_length(path)

        # 2. 平滑度
        metrics['smoothness'] = self._compute_smoothness(path)

        # 3. 与最优路径的偏差
        metrics['deviation'] = self._compute_deviation(
            path, ground_truth_path
        )

        # 4. 碰撞次数
        metrics['collisions'] = self._count_collisions(path)

        return metrics
```

**预计时间**: 1 周

---

#### 任务 12: 探索效率评估 ⭐⭐⭐

**设计**:
```python
class ExplorationEvaluator:
    """探索效率评估器"""

    def evaluate(self, system, environment, max_steps=1000):
        """评估探索效率"""
        coverage_log = []

        for step in range(max_steps):
            # 执行探索步骤
            system.explore_step()

            # 记录覆盖率
            coverage = system.gcm.get_coverage_ratio()
            coverage_log.append({
                'step': step,
                'coverage': coverage,
                'time': system.elapsed_time,
            })

            # 如果完全覆盖，停止
            if coverage >= 0.95:
                break

        return {
            'total_steps': len(coverage_log),
            'final_coverage': coverage_log[-1]['coverage'],
            'time_to_90_percent': self._time_to_coverage(
                coverage_log, 0.9
            ),
        }
```

**预计时间**: 1 周

---

### 阶段 4: 真实验证 (2-3 周)

#### 任务 13: 真实机器人集成 ⭐⭐⭐

**步骤**:
1. 部署到 TurtleBot3
2. ROS 接口适配
3. 传感器数据集成
4. 实时性能测试

**预计时间**: 1.5 周

---

#### 任务 14: 真实环境测试 ⭐⭐⭐

**测试场景**:
1. 办公室环境
2. 走廊环境
3. 复杂室内环境

**预计时间**: 1 周

---

## 📊 总体时间表

| 阶段 | 任务数 | 预计时间 | 优先级 |
|------|--------|---------|--------|
| **阶段 1: 核心组件** | 4 | 3-4 周 | ⭐⭐⭐ |
| **阶段 2: 数据集与基线** | 4 | 3-4 周 | ⭐⭐⭐ |
| **阶段 3: 性能评估** | 4 | 3-4 周 | ⭐⭐⭐ |
| **阶段 4: 真实验证** | 2 | 2-3 周 | ⭐⭐ |
| **总计** | 14 | **11-15 周** | - |

---

## 🎯 里程碑

### 里程碑 1: 核心功能完整 (4 周后)
- ✅ GCM 实现
- ✅ SCG-based 路径规划
- ✅ 不确定性建模
- ✅ 局部滚动栅格

### 里程碑 2: 实验就绪 (8 周后)
- ✅ HM3D 数据集集成
- ✅ 基线方法集成
- ✅ 评估框架完成

### 里程碑 3: 论文级别 (12 周后)
- ✅ 完整性能对比
- ✅ 定量实验结果
- ✅ 真实机器人验证

---

## 📝 下一步行动

### 立即开始 (本周)
1. 创建任务跟踪系统
2. 开始实现 GCM
3. 设计 SCG-based 路径规划器接口

### 短期目标 (2 周内)
1. 完成 GCM 实现和测试
2. 完成 SCG-based 路径规划器
3. 开始不确定性建模

### 中期目标 (1 个月内)
1. 完成所有核心组件
2. 集成 HM3D 数据集
3. 开始基线对比

---

**制定人员**: Claude Sonnet 4.6
**制定日期**: 2026-02-23
