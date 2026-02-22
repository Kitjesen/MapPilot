# USS-Nav 多面体扩展算法研究

## 概述

USS-Nav 的核心创新是**多面体扩展 (Polyhedron Expansion)**，它从局部滚动栅格直接生成几何拓扑图，无需全局稠密地图。

## Algorithm 1: Polyhedron Expansion

### 伪代码 (来自 USS-Nav 论文)

```
Input: 局部占据栅格 O_local (8×8×4m, 0.2m 分辨率)
Output: 多面体节点 P, 空间连通图 SCG

1: 初始化候选点集 C = ∅
2: for each 栅格单元 c in O_local do
3:     if c 是自由空间 then
4:         C ← C ∪ {c}
5:     end if
6: end for
7:
8: 初始化多面体集合 P = ∅
9: while C ≠ ∅ do
10:    选择种子点 s ∈ C (优先选择远离已有多面体的点)
11:    初始化采样点集 S = {s}
12:
13:    // 球面采样扩展
14:    for radius r = r_min to r_max step Δr do
15:        for each 球面方向 d ∈ Sphere(s, r) do
16:            p ← s + r · d
17:            if p 在自由空间 AND p 不与障碍物碰撞 then
18:                S ← S ∪ {p}
19:            end if
20:        end for
21:    end for
22:
23:    // QuickHull 凸包计算
24:    H ← QuickHull(S)
25:
26:    // 碰撞检测
27:    if H 与障碍物碰撞 then
28:        缩小 H 或丢弃
29:    end if
30:
31:    // 添加多面体节点
32:    P ← P ∪ {H}
33:    C ← C \ S  // 移除已覆盖的点
34: end while
35:
36: // 构建空间连通图 (SCG)
37: for each 多面体对 (Pi, Pj) in P do
38:    if Pi 和 Pj 邻接 then
39:        添加 Adjacency 边
40:    else if Pi 和 Pj 连通 then
41:        添加 Connectivity 边
42:    else if Pi 和 Pj 可达 then
43:        添加 Accessibility 边
44:    end if
45: end for
46:
47: return P, SCG
```

---

## 关键组件详解

### 1. 球面采样 (Spherical Sampling)

**目的**: 从种子点向外扩展，探索自由空间的边界。

**实现策略**:

#### 方法 A: Fibonacci 球面采样 (推荐)
```python
def fibonacci_sphere_sampling(num_samples: int) -> np.ndarray:
    """
    Fibonacci 球面均匀采样。

    优势: 分布均匀，无极点聚集

    Returns:
        (num_samples, 3) 单位方向向量
    """
    points = []
    phi = np.pi * (3.0 - np.sqrt(5.0))  # 黄金角

    for i in range(num_samples):
        y = 1 - (i / float(num_samples - 1)) * 2  # y: 1 → -1
        radius = np.sqrt(1 - y * y)

        theta = phi * i

        x = np.cos(theta) * radius
        z = np.sin(theta) * radius

        points.append([x, y, z])

    return np.array(points)
```

**参数选择**:
- `num_samples`: 32-64 (平衡精度和性能)
- `r_min`: 0.5m (最小半径)
- `r_max`: 3.0m (最大半径，取决于局部栅格大小)
- `Δr`: 0.5m (半径步长)

#### 方法 B: 经纬度网格采样
```python
def lat_lon_sphere_sampling(num_lat: int, num_lon: int) -> np.ndarray:
    """
    经纬度网格采样。

    缺点: 极点处采样密度过高

    Returns:
        (num_lat * num_lon, 3) 单位方向向量
    """
    points = []
    for i in range(num_lat):
        theta = np.pi * i / (num_lat - 1)  # 0 → π
        for j in range(num_lon):
            phi = 2 * np.pi * j / num_lon  # 0 → 2π

            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)

            points.append([x, y, z])

    return np.array(points)
```

---

### 2. QuickHull 凸包算法

**目的**: 从采样点集计算凸包，形成多面体。

**实现选择**:

#### 选项 A: scipy.spatial.ConvexHull (推荐)
```python
from scipy.spatial import ConvexHull

def compute_convex_hull(points: np.ndarray) -> dict:
    """
    使用 scipy 计算凸包。

    Args:
        points: (N, 3) 点集

    Returns:
        {
            "vertices": (M, 3) 凸包顶点,
            "faces": (F, 3) 三角面片索引,
            "volume": float 体积,
        }
    """
    hull = ConvexHull(points)

    return {
        "vertices": points[hull.vertices],
        "faces": hull.simplices,
        "volume": hull.volume,
    }
```

**优势**:
- 成熟稳定，经过充分测试
- 性能优秀 (O(n log n))
- 支持任意维度

#### 选项 B: 自实现 QuickHull
```python
def quickhull_3d(points: np.ndarray) -> dict:
    """
    QuickHull 3D 实现 (简化版)。

    仅用于教学目的，实际应用推荐 scipy。
    """
    # 1. 找到极值点 (x_min, x_max, y_min, y_max, z_min, z_max)
    # 2. 构建初始四面体
    # 3. 递归扩展
    # ...
    pass
```

---

### 3. 碰撞检测 (Collision Detection)

**目的**: 检查多面体是否与障碍物碰撞。

**实现策略**:

#### 方法 A: 采样点检测 (快速但不精确)
```python
def collision_check_sampling(
    polyhedron_vertices: np.ndarray,
    occupancy_grid: np.ndarray,
    num_samples: int = 100,
) -> bool:
    """
    在多面体内部采样点，检查是否有点在障碍物内。

    Args:
        polyhedron_vertices: (N, 3) 凸包顶点
        occupancy_grid: 占据栅格
        num_samples: 采样点数

    Returns:
        True if 碰撞
    """
    # 1. 计算多面体边界框
    bbox_min = polyhedron_vertices.min(axis=0)
    bbox_max = polyhedron_vertices.max(axis=0)

    # 2. 在边界框内随机采样
    samples = np.random.uniform(bbox_min, bbox_max, (num_samples, 3))

    # 3. 过滤出多面体内部的点
    inside_points = []
    for p in samples:
        if point_in_convex_hull(p, polyhedron_vertices):
            inside_points.append(p)

    # 4. 检查内部点是否在障碍物内
    for p in inside_points:
        if is_occupied(p, occupancy_grid):
            return True  # 碰撞

    return False  # 无碰撞
```

#### 方法 B: 面片-栅格相交检测 (精确但慢)
```python
def collision_check_exact(
    polyhedron_faces: np.ndarray,
    polyhedron_vertices: np.ndarray,
    occupancy_grid: np.ndarray,
) -> bool:
    """
    检查多面体的每个面片是否与占据栅格相交。

    更精确但计算量大。
    """
    for face in polyhedron_faces:
        # 获取三角面片的三个顶点
        v0, v1, v2 = polyhedron_vertices[face]

        # 检查三角形与栅格的相交
        if triangle_grid_intersection(v0, v1, v2, occupancy_grid):
            return True

    return False
```

**推荐**: 方法 A (采样点检测) 用于实时系统，方法 B 用于离线验证。

---

### 4. 种子点选择策略

**目的**: 选择下一个多面体的种子点，最大化空间覆盖。

**策略**:

#### 策略 A: 最远点优先
```python
def select_seed_farthest(
    candidates: List[np.ndarray],
    existing_polyhedra: List[dict],
) -> np.ndarray:
    """
    选择距离所有已有多面体最远的候选点。

    优势: 最大化覆盖，避免重叠
    """
    max_dist = -1
    best_seed = None

    for c in candidates:
        # 计算到最近多面体的距离
        min_dist = float('inf')
        for poly in existing_polyhedra:
            dist = distance_to_polyhedron(c, poly)
            min_dist = min(min_dist, dist)

        # 选择最远的
        if min_dist > max_dist:
            max_dist = min_dist
            best_seed = c

    return best_seed
```

#### 策略 B: 信息增益优先
```python
def select_seed_information_gain(
    candidates: List[np.ndarray],
    existing_polyhedra: List[dict],
    occupancy_grid: np.ndarray,
) -> np.ndarray:
    """
    选择能覆盖最多未覆盖自由空间的候选点。

    优势: 更智能，考虑实际覆盖
    """
    max_gain = -1
    best_seed = None

    for c in candidates:
        # 估算以 c 为中心的多面体能覆盖多少新空间
        gain = estimate_coverage_gain(c, existing_polyhedra, occupancy_grid)

        if gain > max_gain:
            max_gain = gain
            best_seed = c

    return best_seed
```

---

## 空间连通图 (SCG) 构建

### 三种拓扑边

#### 1. Adjacency (邻接边)
**定义**: 两个多面体共享面或边。

```python
def check_adjacency(poly1: dict, poly2: dict, threshold: float = 0.1) -> bool:
    """
    检查两个多面体是否邻接。

    方法: 检查顶点距离，如果有顶点非常接近，则邻接。
    """
    vertices1 = poly1["vertices"]
    vertices2 = poly2["vertices"]

    for v1 in vertices1:
        for v2 in vertices2:
            if np.linalg.norm(v1 - v2) < threshold:
                return True

    return False
```

#### 2. Connectivity (连通边)
**定义**: 两个多面体之间存在自由空间通道。

```python
def check_connectivity(
    poly1: dict,
    poly2: dict,
    occupancy_grid: np.ndarray,
) -> bool:
    """
    检查两个多面体是否连通。

    方法: 在两个多面体中心之间采样点，检查是否都在自由空间。
    """
    center1 = poly1["vertices"].mean(axis=0)
    center2 = poly2["vertices"].mean(axis=0)

    # 在两个中心之间采样
    num_samples = 20
    for i in range(num_samples):
        t = i / (num_samples - 1)
        p = center1 * (1 - t) + center2 * t

        if is_occupied(p, occupancy_grid):
            return False  # 有障碍物阻挡

    return True  # 连通
```

#### 3. Accessibility (可达边)
**定义**: 两个多面体之间可以通过其他多面体间接到达。

```python
def check_accessibility(
    poly1: dict,
    poly2: dict,
    all_polyhedra: List[dict],
    scg: dict,
) -> bool:
    """
    检查两个多面体是否可达。

    方法: 在 SCG 上做 BFS，检查是否存在路径。
    """
    # BFS 搜索
    visited = set()
    queue = [poly1["id"]]

    while queue:
        current_id = queue.pop(0)
        if current_id == poly2["id"]:
            return True

        visited.add(current_id)

        # 遍历邻居 (Adjacency + Connectivity 边)
        for neighbor_id in scg[current_id]:
            if neighbor_id not in visited:
                queue.append(neighbor_id)

    return False
```

---

## 参数调优

### 关键参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 局部栅格大小 | 8×8×4m | 平衡覆盖范围和计算量 |
| 栅格分辨率 | 0.2m | 与 USS-Nav 一致 |
| 球面采样数 | 32-64 | Fibonacci 采样 |
| 最小半径 r_min | 0.5m | 避免过小的多面体 |
| 最大半径 r_max | 3.0m | 取决于局部栅格大小 |
| 半径步长 Δr | 0.5m | 平衡精度和性能 |
| 碰撞检测采样数 | 100 | 采样点检测 |
| 邻接阈值 | 0.1m | 顶点距离阈值 |

### 性能优化

1. **并行化**: 多个种子点可以并行扩展
2. **空间索引**: 使用 KD-Tree 加速最近邻查询
3. **增量更新**: 只更新变化的区域
4. **缓存**: 缓存凸包计算结果

---

## Python 实现方案

### 依赖库

```python
# 核心依赖
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial import KDTree

# 可选依赖 (可视化)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
```

### 模块结构

```
polyhedron_expansion.py
├── SphereSampler          # 球面采样
├── ConvexHullComputer     # 凸包计算
├── CollisionChecker       # 碰撞检测
├── SeedSelector           # 种子点选择
├── PolyhedronExpander     # 主算法
└── SCGBuilder             # SCG 构建
```

---

## 与 lingtu 的集成点

### 1. 数据输入
- **局部滚动栅格**: 从 `terrain_analysis` 获取
- **占据信息**: 从 Fast-LIO2 点云转换

### 2. 数据输出
- **多面体节点**: 存储到拓扑图 (扩展 TopoNode)
- **SCG**: 存储为拓扑图的边

### 3. 更新频率
- **15 Hz**: 与 USS-Nav 一致
- **增量更新**: 只更新变化的区域

---

## 下一步

1. ✅ 完成算法研究和设计
2. 🎯 实现 `polyhedron_expansion.py` 原型
3. 🎯 在模拟数据上测试
4. 🎯 集成到 lingtu 的 terrain_analysis
5. 🎯 性能优化和参数调优

---

## 参考资料

- USS-Nav 论文: Algorithm 1 (Polyhedron Expansion)
- QuickHull 算法: Barber et al. (1996)
- Fibonacci 球面采样: Gonzalez (2010)
- scipy.spatial.ConvexHull 文档
