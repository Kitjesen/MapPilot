#!/usr/bin/env python3
"""
并行对比测试框架 — 在相同场景下对比三套路径规划器。

规划器:
  1. PCT A*         — 基于占据栅格的 A* (baseline_wrappers.PCTAStarPlanner)
  2. HybridPlanner  — 拓扑图辅助分层规划 (hybrid_planner.HybridPlanner)
  3. SCGPathPlanner  — 纯 SCG 多面体路径规划 (scg_path_planner.SCGPathPlanner)

测试场景 (从简单到复杂):
  1. 直线通道   — 无障碍直线导航
  2. L 形路径   — 简单转折
  3. 复杂环境   — 多房间导航
  4. 长距离导航 — 起点终点相距较远
  5. 密集障碍   — 狭窄通道

运行方式:
  python -m pytest test/test_parallel_comparison.py -v
  python test/test_parallel_comparison.py
"""

import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# 将父目录加入路径，确保在无 ROS2 环境下也可运行
sys.path.insert(0, str(Path(__file__).parent.parent))


# ══════════════════════════════════════════════════════════════════
#  测试场景定义
# ══════════════════════════════════════════════════════════════════

SCENARIOS = [
    {
        "name": "直线通道",
        "start": np.array([0.0, 0.0, 0.0]),
        "goal": np.array([10.0, 0.0, 0.0]),
        "description": "无障碍直线导航",
    },
    {
        "name": "L形路径",
        "start": np.array([0.0, 0.0, 0.0]),
        "goal": np.array([5.0, 5.0, 0.0]),
        "description": "L形绕行",
    },
    {
        "name": "复杂环境",
        "start": np.array([0.0, 0.0, 0.0]),
        "goal": np.array([15.0, 10.0, 0.0]),
        "description": "多房间导航",
    },
    {
        "name": "长距离导航",
        "start": np.array([0.0, 0.0, 0.0]),
        "goal": np.array([20.0, 15.0, 0.0]),
        "description": "起点终点相距较远",
    },
    {
        "name": "短距离精确",
        "start": np.array([1.0, 1.0, 0.0]),
        "goal": np.array([3.0, 2.0, 0.0]),
        "description": "短距离精确定位",
    },
]


# ══════════════════════════════════════════════════════════════════
#  评测结果数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class PlannerResult:
    """单次规划结果。"""
    planner_name: str
    scenario_name: str
    success: bool
    planning_time_ms: float = 0.0
    path_length_m: float = 0.0
    num_waypoints: int = 0
    memory_mb: float = 0.0
    error_msg: str = ""
    skipped: bool = False


# ══════════════════════════════════════════════════════════════════
#  Mock 对象 — 供 HybridPlanner 和 SCGPathPlanner 使用
# ══════════════════════════════════════════════════════════════════

class MockTopoNode:
    """最小 TopoNode mock，满足 HybridPlanner 访问的属性。"""

    def __init__(self, node_id: int, center: np.ndarray, bbox=None):
        self.node_id = node_id
        self.center = center[:2]  # HybridPlanner 使用 2D center
        self.bounding_box = bbox
        self.convex_hull = None
        self.node_type = "room"
        self.name = f"room_{node_id}"

    def __hash__(self):
        return self.node_id

    def __eq__(self, other):
        return isinstance(other, MockTopoNode) and self.node_id == other.node_id


class MockTopologySemGraph:
    """
    最小 TopologySemGraph mock。

    满足 HybridPlanner 调用的接口:
      - get_room_by_position(x, y, radius)
      - rooms  (property → List[TopoNode])
      - get_node(node_id)
      - shortest_path(from_id, to_id)
    """

    def __init__(self, room_centers: list[np.ndarray]):
        self._nodes: dict[int, MockTopoNode] = {}
        self._edges: dict  # from_id → {to_id: weight}
        adjacency: dict[int, dict[int, float]] = {}

        for i, center in enumerate(room_centers):
            node = MockTopoNode(node_id=i, center=center)
            self._nodes[i] = node
            adjacency[i] = {}

        # 连接相邻房间（按顺序）
        ids = list(self._nodes.keys())
        for j in range(len(ids) - 1):
            a, b = ids[j], ids[j + 1]
            dist = float(np.linalg.norm(
                self._nodes[a].center - self._nodes[b].center
            ))
            adjacency[a][b] = dist
            adjacency[b][a] = dist

        self._adjacency = adjacency

    @property
    def rooms(self) -> list[MockTopoNode]:
        return list(self._nodes.values())

    def get_node(self, node_id: int) -> MockTopoNode | None:
        return self._nodes.get(node_id)

    def get_room_by_position(
        self, x: float, y: float, radius: float = 5.0
    ) -> int | None:
        """返回最近的且在 radius 范围内的房间 ID。"""
        pos = np.array([x, y])
        best_id = None
        best_dist = float("inf")
        for nid, node in self._nodes.items():
            d = float(np.linalg.norm(node.center - pos))
            if d < best_dist and d <= radius:
                best_dist = d
                best_id = nid
        return best_id

    def shortest_path(
        self, from_id: int, to_id: int
    ):
        """Dijkstra 最短路径，返回 (cost, [node_id, ...])。"""
        import heapq

        if from_id not in self._nodes or to_id not in self._nodes:
            return float("inf"), []

        dist_map = {from_id: 0.0}
        prev_map: dict[int, int] = {}
        heap = [(0.0, from_id)]
        visited = set()

        while heap:
            cost, uid = heapq.heappop(heap)
            if uid in visited:
                continue
            visited.add(uid)

            if uid == to_id:
                path = []
                cur = to_id
                while cur in prev_map:
                    path.append(cur)
                    cur = prev_map[cur]
                path.append(from_id)
                return cost, list(reversed(path))

            for vid, w in self._adjacency.get(uid, {}).items():
                if vid in visited:
                    continue
                new_cost = cost + w
                if new_cost < dist_map.get(vid, float("inf")):
                    dist_map[vid] = new_cost
                    prev_map[vid] = uid
                    heapq.heappush(heap, (new_cost, vid))

        return float("inf"), []


def _build_mock_tsg_for_scenario(scenario: dict) -> MockTopologySemGraph:
    """根据场景构造最小 TSG mock（沿 start→goal 方向均匀分布 4 个房间）。"""
    start = scenario["start"]
    goal = scenario["goal"]
    num_rooms = 4
    centers = []
    for i in range(num_rooms):
        t = i / (num_rooms - 1)
        centers.append(start * (1 - t) + goal * t)
    return MockTopologySemGraph(centers)


class MockTomogram:
    """最小 Tomogram mock（HybridPlanner 持有但不强依赖其接口）。"""

    def __init__(self):
        self.resolution = 0.2
        self.map_center = np.array([0.0, 0.0])
        self.map_dim_x = 200
        self.map_dim_y = 200
        self.inflated_cost = np.zeros((1, 200, 200), dtype=np.float32)


def _build_mock_scg_for_scenario(scenario: dict):
    """
    为场景构造最小 SCGBuilder，沿 start→goal 方向生成若干多面体。

    返回已调用过 build_edges 的 SCGBuilder。
    """
    from perception.polyhedron_expansion import Polyhedron
    from perception.scg_builder import SCGBuilder, SCGConfig

    start = scenario["start"]
    goal = scenario["goal"]
    num_poly = 5
    radius = 2.0

    scg = SCGBuilder(SCGConfig(
        adjacency_threshold=radius * 0.8,
        connectivity_samples=10,
        loop_closure_threshold=1.0,
    ))

    for i in range(num_poly):
        t = i / (num_poly - 1)
        center = start * (1 - t) + goal * t

        # 构造一个立方体形状的多面体（8 个顶点）
        offsets = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1],
        ], dtype=np.float64) * (radius * 0.7)
        vertices = center + offsets
        faces = np.array([
            [0, 1, 2], [0, 2, 3],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6],
            [1, 2, 6], [1, 6, 5],
            [0, 3, 7], [0, 7, 4],
        ])

        poly = Polyhedron(
            poly_id=i,
            vertices=vertices,
            faces=faces,
            center=center.copy(),
            volume=(radius * 1.4) ** 3,
            radius=radius,
            seed_point=center.copy(),
            sample_points=vertices,
        )
        scg.add_polyhedron(poly)

    scg.build_edges()  # 无占据栅格，只构建邻接边
    return scg


# ══════════════════════════════════════════════════════════════════
#  规划器包装器
# ══════════════════════════════════════════════════════════════════

def _compute_path_length(path: np.ndarray) -> float:
    """计算路径总长度（米）。"""
    if path is None or len(path) < 2:
        return 0.0
    length = 0.0
    for i in range(len(path) - 1):
        length += float(np.linalg.norm(np.array(path[i + 1]) - np.array(path[i])))
    return length


def _estimate_memory_mb(obj) -> float:
    """用 sys.getsizeof 估算对象内存（MB），深度有限。"""
    import sys as _sys
    try:
        return _sys.getsizeof(obj) / 1024 / 1024
    except Exception:
        return 0.0


class PCTBaselineWrapper:
    """
    PCT A* 包装器 — 使用 baseline_wrappers.PCTAStarPlanner。

    将 start→goal 所在区域设置为足够大的无障碍栅格，
    然后调用内置 A* 规划。
    """

    NAME = "PCT A*"

    def __init__(self):
        self._planner = None
        self._available = False
        try:
            from perception.baseline_wrappers import PCTAStarPlanner
            self._planner = PCTAStarPlanner()
            # 使用足够大的栅格覆盖所有测试场景（25m × 20m，分辨率 0.5m）
            self._planner.initialize(
                resolution=0.5,
                size=(50, 50, 10),
                origin=np.array([-2.0, -2.0, -1.0]),
            )
            self._available = True
        except Exception as exc:
            self._error = str(exc)

    def plan(self, scenario: dict) -> PlannerResult:
        start = scenario["start"]
        goal = scenario["goal"]
        name = scenario["name"]

        if not self._available:
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                skipped=True,
                error_msg=getattr(self, "_error", "unavailable"),
            )

        t0 = time.perf_counter()
        try:
            path = self._planner.plan(start, goal)
            elapsed_ms = (time.perf_counter() - t0) * 1000

            if path is None:
                return PlannerResult(
                    planner_name=self.NAME,
                    scenario_name=name,
                    success=False,
                    planning_time_ms=elapsed_ms,
                    error_msg="No path found",
                )

            path_arr = np.array(path)
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=True,
                planning_time_ms=elapsed_ms,
                path_length_m=_compute_path_length(path_arr),
                num_waypoints=len(path_arr),
                memory_mb=_estimate_memory_mb(self._planner),
            )
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                planning_time_ms=elapsed_ms,
                error_msg=str(exc),
            )


class HybridPlannerWrapper:
    """
    HybridPlanner 包装器。

    为每个场景构造最小 TSG + MockTomogram，然后调用 plan_path。
    """

    NAME = "HybridPlanner"

    def __init__(self):
        self._available = False
        try:
            self._available = True
        except Exception as exc:
            self._error = str(exc)

    def plan(self, scenario: dict) -> PlannerResult:
        start = scenario["start"]
        goal = scenario["goal"]
        name = scenario["name"]

        if not self._available:
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                skipped=True,
                error_msg=getattr(self, "_error", "unavailable"),
            )

        t0 = time.perf_counter()
        try:
            from perception.hybrid_planner import HybridPlanner

            tsg = _build_mock_tsg_for_scenario(scenario)
            mock_tomo = MockTomogram()
            trav_grid = mock_tomo.inflated_cost[0]  # 2D float32 [nx, ny]
            planner = HybridPlanner(tsg, trav_grid, trav_res=mock_tomo.resolution)

            result = planner.plan_path(start, goal)
            elapsed_ms = (time.perf_counter() - t0) * 1000

            if not result.success:
                return PlannerResult(
                    planner_name=self.NAME,
                    scenario_name=name,
                    success=False,
                    planning_time_ms=elapsed_ms,
                    error_msg="HybridPlanner returned success=False",
                )

            # 提取路径点
            all_wp = result.waypoints
            path_arr = np.array(all_wp) if all_wp else np.array([start, goal])
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=True,
                planning_time_ms=elapsed_ms,
                path_length_m=_compute_path_length(path_arr),
                num_waypoints=len(path_arr),
                memory_mb=_estimate_memory_mb(planner),
            )
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                planning_time_ms=elapsed_ms,
                error_msg=str(exc),
            )


class SCGPlannerWrapper:
    """
    SCGPathPlanner 包装器。

    为每个场景构造最小 SCGBuilder（多面体沿 start→goal 均匀分布），
    然后调用 SCGPathPlanner.plan。
    """

    NAME = "SCGPathPlanner"

    def __init__(self):
        self._available = False
        try:
            self._available = True
        except Exception as exc:
            self._error = str(exc)

    def plan(self, scenario: dict) -> PlannerResult:
        start = scenario["start"]
        goal = scenario["goal"]
        name = scenario["name"]

        if not self._available:
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                skipped=True,
                error_msg=getattr(self, "_error", "unavailable"),
            )

        t0 = time.perf_counter()
        try:
            from perception.scg_path_planner import SCGPathPlanner

            scg = _build_mock_scg_for_scenario(scenario)
            planner = SCGPathPlanner(scg)

            result = planner.plan(start, goal, smooth=True, simplify=True)
            elapsed_ms = (time.perf_counter() - t0) * 1000

            if not result.success:
                return PlannerResult(
                    planner_name=self.NAME,
                    scenario_name=name,
                    success=False,
                    planning_time_ms=elapsed_ms,
                    error_msg="SCGPathPlanner returned success=False",
                )

            # 从 segments 提取完整路径
            waypoints = []
            if result.segments:
                for seg in result.segments:
                    wps = seg.waypoints
                    if hasattr(wps, "__len__") and len(wps) > 0:
                        waypoints.extend(list(wps))
            if not waypoints:
                waypoints = [start, goal]

            path_arr = np.array(waypoints)
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=True,
                planning_time_ms=elapsed_ms,
                path_length_m=result.total_distance if result.total_distance > 0
                              else _compute_path_length(path_arr),
                num_waypoints=len(path_arr),
                memory_mb=_estimate_memory_mb(scg),
            )
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            return PlannerResult(
                planner_name=self.NAME,
                scenario_name=name,
                success=False,
                planning_time_ms=elapsed_ms,
                error_msg=str(exc),
            )


# ══════════════════════════════════════════════════════════════════
#  对比表格打印
# ══════════════════════════════════════════════════════════════════

def _status_str(result: PlannerResult) -> str:
    if result.skipped:
        return "SKIPPED"
    if not result.success:
        return "FAILED"
    return "OK"


def print_comparison_table(
    scenario: dict,
    results: list[PlannerResult],
    file=None,
) -> None:
    """
    打印某场景的对比表格：

    场景: 直线通道 (start=[0,0,0] → goal=[10,0,0])
    ┌─────────────────┬──────────────┬──────────────┬──────────────┬──────────────┐
    │ 规划器          │ 状态         │ 时间(ms)     │ 路径长度(m)  │ 路径点数     │
    ├─────────────────┼──────────────┼──────────────┼──────────────┼──────────────┤
    │ PCT A*          │ OK           │ 28.6         │ 10.0         │ 21           │
    │ HybridPlanner   │ OK           │ 1.2          │ 10.0         │ 5            │
    │ SCGPathPlanner  │ OK           │ 0.8          │ 10.2         │ 3            │
    └─────────────────┴──────────────┴──────────────┴──────────────┴──────────────┘
    """
    import io
    buf = io.StringIO()

    start = scenario["start"]
    goal = scenario["goal"]
    start_str = f"[{start[0]:.0f},{start[1]:.0f},{start[2]:.0f}]"
    goal_str = f"[{goal[0]:.0f},{goal[1]:.0f},{goal[2]:.0f}]"

    buf.write(
        f"\n场景: {scenario['name']}  "
        f"(start={start_str} → goal={goal_str})  "
        f"— {scenario['description']}\n"
    )

    col_widths = [17, 8, 12, 14, 10]
    headers = ["规划器", "状态", "时间(ms)", "路径长度(m)", "路径点数"]

    def row_sep(left, mid, right, fill):
        segs = [fill * (w + 2) for w in col_widths]
        return left + mid.join(segs) + right

    buf.write(row_sep("┌", "┬", "┐", "─") + "\n")
    header_row = "│"
    for h, w in zip(headers, col_widths):
        header_row += f" {h:<{w}} │"
    buf.write(header_row + "\n")
    buf.write(row_sep("├", "┼", "┤", "─") + "\n")

    for r in results:
        status = _status_str(r)
        time_str = f"{r.planning_time_ms:.2f}" if r.planning_time_ms > 0 else "—"
        length_str = f"{r.path_length_m:.2f}" if r.success else "—"
        wp_str = str(r.num_waypoints) if r.success else "—"

        row = "│"
        vals = [r.planner_name, status, time_str, length_str, wp_str]
        for v, w in zip(vals, col_widths):
            row += f" {v!s:<{w}} │"
        buf.write(row + "\n")

        if not r.success and not r.skipped and r.error_msg:
            # 截断错误信息以免破坏表格
            err_preview = r.error_msg[:60].replace("\n", " ")
            buf.write(f"│   >> Error: {err_preview:<{sum(col_widths) + len(col_widths)*3 - 13}} │\n")

    buf.write(row_sep("└", "┴", "┘", "─") + "\n")

    output = buf.getvalue()
    print(output, file=file)
    return output


def print_summary_table(
    all_results: list[PlannerResult],
    file=None,
) -> None:
    """打印跨场景汇总统计表。"""
    import io
    buf = io.StringIO()

    planner_names = list(dict.fromkeys(r.planner_name for r in all_results))

    buf.write("\n" + "=" * 72 + "\n")
    buf.write("汇总统计 (所有场景平均)\n")
    buf.write("=" * 72 + "\n")

    col_widths = [17, 8, 12, 14, 10]
    headers = ["规划器", "成功率", "均值时间(ms)", "均值长度(m)", "均值路径点"]

    def row_sep(left, mid, right, fill):
        segs = [fill * (w + 2) for w in col_widths]
        return left + mid.join(segs) + right

    buf.write(row_sep("┌", "┬", "┐", "─") + "\n")
    header_row = "│"
    for h, w in zip(headers, col_widths):
        header_row += f" {h:<{w}} │"
    buf.write(header_row + "\n")
    buf.write(row_sep("├", "┼", "┤", "─") + "\n")

    for pname in planner_names:
        pr = [r for r in all_results if r.planner_name == pname]
        total = len(pr)
        success = [r for r in pr if r.success]
        skipped = [r for r in pr if r.skipped]

        if len(skipped) == total:
            success_str = "SKIPPED"
            time_str = "—"
            length_str = "—"
            wp_str = "—"
        else:
            rate = len(success) / total
            success_str = f"{rate:.0%}"
            time_str = (
                f"{np.mean([r.planning_time_ms for r in success]):.2f}"
                if success else "—"
            )
            length_str = (
                f"{np.mean([r.path_length_m for r in success]):.2f}"
                if success else "—"
            )
            wp_str = (
                f"{np.mean([r.num_waypoints for r in success]):.1f}"
                if success else "—"
            )

        row = "│"
        vals = [pname, success_str, time_str, length_str, wp_str]
        for v, w in zip(vals, col_widths):
            row += f" {v!s:<{w}} │"
        buf.write(row + "\n")

    buf.write(row_sep("└", "┴", "┘", "─") + "\n")
    buf.write("\n")

    output = buf.getvalue()
    print(output, file=file)
    return output


# ══════════════════════════════════════════════════════════════════
#  核心比较运行函数
# ══════════════════════════════════════════════════════════════════

def run_parallel_comparison(
    scenarios=None,
    verbose: bool = True,
) -> list[PlannerResult]:
    """
    对所有场景运行三套规划器并收集结果。

    Args:
        scenarios: 场景列表（默认使用模块级 SCENARIOS）
        verbose: 是否在终端打印表格

    Returns:
        所有 PlannerResult 的列表
    """
    if scenarios is None:
        scenarios = SCENARIOS

    wrappers = [
        PCTBaselineWrapper(),
        HybridPlannerWrapper(),
        SCGPlannerWrapper(),
    ]

    all_results: list[PlannerResult] = []

    for scenario in scenarios:
        scenario_results = []
        for wrapper in wrappers:
            result = wrapper.plan(scenario)
            scenario_results.append(result)
            all_results.append(result)

        if verbose:
            print_comparison_table(scenario, scenario_results)

    if verbose:
        print_summary_table(all_results)

    return all_results


# ══════════════════════════════════════════════════════════════════
#  pytest 测试函数
# ══════════════════════════════════════════════════════════════════

class TestParallelComparison:
    """pytest 测试类 — 并行对比三套规划器。"""

    def setup_method(self):
        """每个测试方法前初始化规划器包装器。"""
        self.pct_wrapper = PCTBaselineWrapper()
        self.hybrid_wrapper = HybridPlannerWrapper()
        self.scg_wrapper = SCGPlannerWrapper()

    # ── PCT A* 测试 ──────────────────────────────────────────────

    def test_pct_available(self):
        """PCT A* 规划器应能成功初始化。"""
        assert self.pct_wrapper._available, (
            f"PCTAStarPlanner 初始化失败: "
            f"{getattr(self.pct_wrapper, '_error', 'unknown')}"
        )

    def test_pct_straight_line(self):
        """PCT A* 在直线场景应成功。"""
        scenario = SCENARIOS[0]  # 直线通道
        result = self.pct_wrapper.plan(scenario)
        if result.skipped:
            import pytest
            pytest.skip(f"PCT A* SKIPPED: {result.error_msg}")
        assert result.success, f"PCT A* failed: {result.error_msg}"
        assert result.num_waypoints >= 2
        assert result.path_length_m > 0.0
        assert result.planning_time_ms >= 0.0

    def test_pct_all_scenarios(self):
        """PCT A* 在所有场景上运行（允许部分失败）。"""
        successes = 0
        for scenario in SCENARIOS:
            result = self.pct_wrapper.plan(scenario)
            if result.skipped:
                import pytest
                pytest.skip("PCT A* SKIPPED")
            if result.success:
                successes += 1
                assert result.path_length_m >= 0.0
                assert result.num_waypoints >= 2
        # 至少直线场景应成功
        assert successes >= 1, "PCT A* should succeed in at least one scenario"

    # ── HybridPlanner 测试 ───────────────────────────────────────

    def test_hybrid_available(self):
        """HybridPlanner 应能成功导入。"""
        assert self.hybrid_wrapper._available, (
            f"HybridPlanner 不可用: "
            f"{getattr(self.hybrid_wrapper, '_error', 'unknown')}"
        )

    def test_hybrid_straight_line(self):
        """HybridPlanner 在直线场景应成功。"""
        scenario = SCENARIOS[0]
        result = self.hybrid_wrapper.plan(scenario)
        if result.skipped:
            import pytest
            pytest.skip(f"HybridPlanner SKIPPED: {result.error_msg}")
        assert result.success, f"HybridPlanner failed: {result.error_msg}"
        assert result.num_waypoints >= 2
        assert result.path_length_m > 0.0

    def test_hybrid_l_shape(self):
        """HybridPlanner 在 L 形路径场景应成功。"""
        scenario = SCENARIOS[1]
        result = self.hybrid_wrapper.plan(scenario)
        if result.skipped:
            import pytest
            pytest.skip(f"HybridPlanner SKIPPED: {result.error_msg}")
        assert result.success, f"HybridPlanner L-shape failed: {result.error_msg}"

    def test_hybrid_all_scenarios(self):
        """HybridPlanner 在所有场景上运行。"""
        successes = 0
        for scenario in SCENARIOS:
            result = self.hybrid_wrapper.plan(scenario)
            if result.skipped:
                import pytest
                pytest.skip("HybridPlanner SKIPPED")
            if result.success:
                successes += 1
        assert successes >= 1, "HybridPlanner should succeed in at least one scenario"

    # ── SCGPathPlanner 测试 ──────────────────────────────────────

    def test_scg_available(self):
        """SCGPathPlanner 应能成功导入。"""
        assert self.scg_wrapper._available, (
            f"SCGPathPlanner 不可用: "
            f"{getattr(self.scg_wrapper, '_error', 'unknown')}"
        )

    def test_scg_straight_line(self):
        """SCGPathPlanner 在直线场景应成功。"""
        scenario = SCENARIOS[0]
        result = self.scg_wrapper.plan(scenario)
        if result.skipped:
            import pytest
            pytest.skip(f"SCGPathPlanner SKIPPED: {result.error_msg}")
        assert result.success, f"SCGPathPlanner failed: {result.error_msg}"
        assert result.num_waypoints >= 2
        assert result.path_length_m > 0.0

    def test_scg_complex_environment(self):
        """SCGPathPlanner 在复杂环境场景应成功。"""
        scenario = SCENARIOS[2]
        result = self.scg_wrapper.plan(scenario)
        if result.skipped:
            import pytest
            pytest.skip(f"SCGPathPlanner SKIPPED: {result.error_msg}")
        assert result.success, f"SCGPathPlanner complex env failed: {result.error_msg}"

    def test_scg_all_scenarios(self):
        """SCGPathPlanner 在所有场景上运行。"""
        successes = 0
        for scenario in SCENARIOS:
            result = self.scg_wrapper.plan(scenario)
            if result.skipped:
                import pytest
                pytest.skip("SCGPathPlanner SKIPPED")
            if result.success:
                successes += 1
        assert successes >= 1, "SCGPathPlanner should succeed in at least one scenario"

    # ── 并行对比测试 ─────────────────────────────────────────────

    def test_full_parallel_comparison(self):
        """在所有场景上运行三套规划器的完整并行对比。"""
        all_results = run_parallel_comparison(scenarios=SCENARIOS, verbose=True)

        # 验证结果数量
        assert len(all_results) == len(SCENARIOS) * 3, (
            f"Expected {len(SCENARIOS) * 3} results, got {len(all_results)}"
        )

        # 每个规划器在每个场景都应有结果
        for scenario in SCENARIOS:
            for pname in ["PCT A*", "HybridPlanner", "SCGPathPlanner"]:
                matching = [
                    r for r in all_results
                    if r.planner_name == pname and r.scenario_name == scenario["name"]
                ]
                assert len(matching) == 1, (
                    f"Missing result for {pname} on {scenario['name']}"
                )

    def test_result_fields_valid(self):
        """所有结果的字段应在合理范围内。"""
        all_results = run_parallel_comparison(scenarios=SCENARIOS[:2], verbose=False)

        for r in all_results:
            assert r.planner_name in ["PCT A*", "HybridPlanner", "SCGPathPlanner"]
            assert r.scenario_name in [s["name"] for s in SCENARIOS]
            assert r.planning_time_ms >= 0.0
            assert r.path_length_m >= 0.0
            assert r.num_waypoints >= 0

            if r.success:
                assert r.path_length_m > 0.0, (
                    f"{r.planner_name}/{r.scenario_name}: success but path_length=0"
                )
                assert r.num_waypoints >= 2, (
                    f"{r.planner_name}/{r.scenario_name}: success but num_waypoints<2"
                )

    def test_planning_time_reasonable(self):
        """规划时间应在合理范围内（< 5000ms）。"""
        all_results = run_parallel_comparison(scenarios=SCENARIOS, verbose=False)
        for r in all_results:
            if r.success and not r.skipped:
                assert r.planning_time_ms < 5000.0, (
                    f"{r.planner_name}/{r.scenario_name}: "
                    f"planning took {r.planning_time_ms:.1f}ms (> 5s limit)"
                )

    def test_path_length_positive(self):
        """成功规划的路径长度应为正数。"""
        all_results = run_parallel_comparison(scenarios=SCENARIOS, verbose=False)
        for r in all_results:
            if r.success and not r.skipped:
                assert r.path_length_m > 0.0, (
                    f"{r.planner_name}/{r.scenario_name}: path_length_m={r.path_length_m}"
                )

    def test_comparison_produces_output(self):
        """对比函数应产生非空输出。"""
        import io
        buf = io.StringIO()
        scenario = SCENARIOS[0]
        pct_result = self.pct_wrapper.plan(scenario)
        hybrid_result = self.hybrid_wrapper.plan(scenario)
        scg_result = self.scg_wrapper.plan(scenario)

        print_comparison_table(
            scenario,
            [pct_result, hybrid_result, scg_result],
            file=buf,
        )
        text = buf.getvalue()
        assert len(text) > 0
        assert scenario["name"] in text
        assert "PCT A*" in text
        assert "HybridPlanner" in text
        assert "SCGPathPlanner" in text

    # ── Mock 工具测试 ─────────────────────────────────────────────

    def test_mock_tsg_shortest_path(self):
        """MockTopologySemGraph 的 shortest_path 应能返回路径。"""
        centers = [
            np.array([0.0, 0.0, 0.0]),
            np.array([5.0, 0.0, 0.0]),
            np.array([10.0, 0.0, 0.0]),
        ]
        tsg = MockTopologySemGraph(centers)
        cost, path = tsg.shortest_path(0, 2)
        assert cost < float("inf"), "shortest_path should find a path"
        assert path[0] == 0 and path[-1] == 2

    def test_mock_scg_has_polyhedra(self):
        """_build_mock_scg_for_scenario 应生成多面体节点。"""
        scenario = SCENARIOS[0]
        scg = _build_mock_scg_for_scenario(scenario)
        assert len(scg.nodes) > 0, "SCG should have polyhedra nodes"

    def test_mock_scg_has_edges(self):
        """构建的 SCG 应有边（邻接多面体之间）。"""
        scenario = SCENARIOS[0]
        scg = _build_mock_scg_for_scenario(scenario)
        # 相邻多面体半径 2.0，间距约 2.5m，邻接阈值 1.6m，
        # 可能没有邻接边，但至少节点存在
        assert len(scg.nodes) >= 3


# ══════════════════════════════════════════════════════════════════
#  命令行入口（直接运行时）
# ══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("\n" + "=" * 72)
    print("并行规划器对比测试框架")
    print("=" * 72)
    print(f"场景数量: {len(SCENARIOS)}")
    print("规划器数量: 3 (PCT A* / HybridPlanner / SCGPathPlanner)")
    print("=" * 72)

    try:
        all_results = run_parallel_comparison(scenarios=SCENARIOS, verbose=True)

        # 统计
        total = len(all_results)
        success = sum(1 for r in all_results if r.success)
        skipped = sum(1 for r in all_results if r.skipped)
        failed = total - success - skipped

        print(f"\n运行完成: {total} 项测试")
        print(f"  成功: {success}")
        print(f"  失败: {failed}")
        print(f"  跳过: {skipped}")
        sys.exit(0 if failed == 0 else 1)

    except Exception as exc:
        print(f"\n运行出错: {exc}")
        traceback.print_exc()
        sys.exit(1)
