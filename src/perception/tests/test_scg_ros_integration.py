#!/usr/bin/env python3
"""
SCG ROS2 闆嗘垚娴嬭瘯 鈥?绾?Python锛屾棤闇€ ROS2 杩愯鏃躲€?

娴嬭瘯瑕嗙洊:
  1. SCGPathPlanner 鍩烘湰璺緞瑙勫垝 (start 鈫?goal 鈫?waypoints)
  2. JSON 璇锋眰/鍝嶅簲搴忓垪鍖栦笌鍙嶅簭鍒楀寲
  3. Fallback 閫昏緫 (SCG 涓虹┖鏃堕檷绾у埌鐩存帴鐩爣)
  4. 杈圭晫鎯呭喌 (璧风偣/缁堢偣涓嶅湪澶氶潰浣撳唴)
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

# 灏?semantic_perception 鍖呰矾寰勫姞鍏?sys.path
_pkg_root = Path(__file__).parent.parent
sys.path.insert(0, str(_pkg_root))

from perception.polyhedron_expansion import (
    Polyhedron,
    PolyhedronExpander,
    PolyhedronExpansionConfig,
)
from perception.scg_builder import SCGBuilder, SCGConfig
from perception.scg_path_planner import SCGPath, SCGPathPlanner

# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
#  Fixtures
# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

def _build_minimal_scg():
    """
    鏋勫缓鏈€灏忔祴璇?SCG銆?

    浣跨敤杈冨皬鐨勫弬鏁颁繚璇佹祴璇曢€熷害锛屽悓鏃朵繚璇佽兘鐢熸垚鑷冲皯 2 涓闈綋銆?
    """
    # 鍒涘缓涓€涓畝鍗曠殑鑷敱绌洪棿锛氳蛋寤婂舰鐘讹紙Y 杞存柟鍚戝欢浼革級
    occupancy_grid = np.ones((20, 40, 5), dtype=np.float32)
    # 璧板粖: x=[4,16], y=[0,40], z=[0,5] 鍏ㄩ儴鑷敱
    occupancy_grid[4:16, :, :] = 0.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    config = PolyhedronExpansionConfig(
        num_sphere_samples=24,
        r_min=0.3,
        r_max=1.0,
        r_step=0.3,
        min_polyhedron_volume=0.05,
        max_polyhedra=10,
        coverage_threshold=0.4,
        collision_threshold=0.5,
    )

    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    scg_config = SCGConfig(
        adjacency_threshold=0.3,
        connectivity_samples=10,
        loop_closure_threshold=0.5,
    )
    builder = SCGBuilder(scg_config)
    for poly in polyhedra:
        builder.add_polyhedron(poly)

    builder.build_edges(occupancy_grid, grid_resolution, grid_origin)
    return builder, polyhedra


def _make_fake_polyhedron(poly_id: int, center: np.ndarray, radius: float = 1.0) -> Polyhedron:
    """
    鏋勯€犱竴涓亣澶氶潰浣?(鐢ㄤ簬闅旂娴嬭瘯锛屼笉渚濊禆澶氶潰浣撴墿灞曠畻娉?銆?
    """
    # 绠€鍗曟鍏潰浣撻《鐐?
    offsets = np.array([
        [radius, 0, 0], [-radius, 0, 0],
        [0, radius, 0], [0, -radius, 0],
        [0, 0, radius], [0, 0, -radius],
    ])
    vertices = center + offsets
    faces = np.array([[0, 2, 4], [0, 2, 5], [0, 3, 4], [0, 3, 5],
                      [1, 2, 4], [1, 2, 5], [1, 3, 4], [1, 3, 5]])
    return Polyhedron(
        poly_id=poly_id,
        vertices=vertices,
        faces=faces,
        center=center.copy(),
        volume=(4.0 / 3.0) * np.pi * radius ** 3,
        radius=radius,
        seed_point=center.copy(),
        sample_points=vertices,
    )


# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
#  娴嬭瘯 1: SCGPathPlanner 鍩烘湰璺緞瑙勫垝
# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

class TestSCGPathPlannerBasic:
    """Test documentation."""

    def test_plan_returns_scg_path_object(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([5.0, 5.0, 0.0])

        result = planner.plan(start, goal)
        assert isinstance(result, SCGPath)

    def test_plan_fails_gracefully_when_no_nodes(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 10.0, 0.0])

        result = planner.plan(start, goal)
        assert result.success is False
        assert result.total_distance == 0.0
        assert len(result.polyhedron_sequence) == 0

    def test_plan_same_polyhedron(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        center = np.array([0.0, 0.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=2.0)
        builder.add_polyhedron(poly)
        builder.build_edges()  # 鏃?occupancy_grid

        planner = SCGPathPlanner(builder)
        start = center + np.array([0.1, 0.0, 0.0])
        goal = center + np.array([-0.1, 0.0, 0.0])

        result = planner.plan(start, goal)
        # 鍚屼竴澶氶潰浣撳唴锛屽簭鍒楅暱搴︿负 1
        assert result.success is True
        assert len(result.polyhedron_sequence) == 1
        assert result.polyhedron_sequence[0] == 0

    def test_plan_two_adjacent_polyhedra(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        start = c1
        goal = c2

        result = planner.plan(start, goal)
        assert result.success is True
        assert len(result.polyhedron_sequence) >= 1

    def test_plan_with_real_scg(self):
        """Test documentation."""
        builder, polyhedra = _build_minimal_scg()
        if len(polyhedra) < 2:
            pytest.skip("need at least two polyhedra for path planning")

        planner = SCGPathPlanner(builder)
        start = polyhedra[0].center
        goal = polyhedra[-1].center

        result = planner.plan(start, goal, smooth=True, simplify=True)

        # 鑷冲皯搴旇繑鍥炵粨鏋勬纭殑 SCGPath
        assert isinstance(result, SCGPath)
        assert hasattr(result, "success")
        assert hasattr(result, "total_distance")
        assert hasattr(result, "planning_time")
        assert result.planning_time >= 0.0

        if result.success:
            assert result.total_distance > 0.0
            assert len(result.polyhedron_sequence) >= 1


# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
#  娴嬭瘯 2: JSON 搴忓垪鍖?鍙嶅簭鍒楀寲
# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

class TestSCGJsonSerialization:
    """Test documentation."""

    def _simulate_scg_plan_request(self, builder: SCGBuilder, planner: SCGPathPlanner,
                                   start_list: list, goal_list: list) -> dict:
        """
        妯℃嫙 perception_node._scg_plan_request_callback 鐨勬牳蹇冮€昏緫
        (涓嶄緷璧?ROS2锛屼粎娴嬭瘯绾笟鍔￠€昏緫)銆?
        """
        # --- 妯℃嫙鎺ユ敹璇锋眰 ---
        request_json = json.dumps({"start": start_list, "goal": goal_list})
        data = json.loads(request_json)
        start_raw = data.get("start", [0.0, 0.0, 0.0])
        goal_raw = data.get("goal", [0.0, 0.0, 0.0])
        start = np.array(start_raw, dtype=np.float64)
        goal = np.array(goal_raw, dtype=np.float64)

        if len(builder.nodes) == 0:
            return {
                "success": False,
                "waypoints": [],
                "poly_count": 0,
                "distance": 0.0,
                "error": "SCG has no nodes",
            }

        path = planner.plan(start, goal)

        if path.success:
            all_waypoints = []
            seen = set()
            for seg in path.segments:
                for pt in seg.waypoints:
                    key = (round(pt[0], 4), round(pt[1], 4), round(pt[2], 4))
                    if key not in seen:
                        seen.add(key)
                        all_waypoints.append([
                            round(float(pt[0]), 4),
                            round(float(pt[1]), 4),
                            round(float(pt[2]), 4),
                        ])
            return {
                "success": True,
                "waypoints": all_waypoints,
                "poly_count": len(path.polyhedron_sequence),
                "distance": round(path.total_distance, 4),
                "error": "",
            }
        else:
            return {
                "success": False,
                "waypoints": [],
                "poly_count": 0,
                "distance": 0.0,
                "error": "SCG planner could not find a path",
            }

    def test_request_json_valid(self):
        """Test documentation."""
        start = [1.0, 2.0, 0.0]
        goal = [5.0, 6.0, 0.0]
        request = json.dumps({"start": start, "goal": goal})
        parsed = json.loads(request)
        assert parsed["start"] == start
        assert parsed["goal"] == goal

    def test_response_json_success_format(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
        )

        # 搴忓垪鍖栦负 JSON 鍐嶅弽搴忓垪鍖?
        result_json = json.dumps(result)
        parsed = json.loads(result_json)

        assert "success" in parsed
        assert "waypoints" in parsed
        assert "poly_count" in parsed
        assert "distance" in parsed
        assert "error" in parsed
        assert isinstance(parsed["success"], bool)
        assert isinstance(parsed["waypoints"], list)
        assert isinstance(parsed["poly_count"], int)
        assert isinstance(parsed["distance"], float)

    def test_response_json_failure_format(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [10.0, 10.0, 0.0]
        )

        result_json = json.dumps(result)
        parsed = json.loads(result_json)

        assert parsed["success"] is False
        assert parsed["waypoints"] == []
        assert parsed["poly_count"] == 0
        assert parsed["distance"] == 0.0
        assert "error" in parsed
        assert len(parsed["error"]) > 0

    def test_waypoints_are_serializable(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
        )

        if result["success"]:
            for wp in result["waypoints"]:
                assert len(wp) == 3
                assert all(isinstance(v, float) for v in wp)

    def test_invalid_request_json_handled(self):
        """Test documentation."""
        bad_jsons = [
            "",
            "not json",
            '{"start": "not_a_list", "goal": [1,2,3]}',
            '{"start": [1,2], "goal": null}',
        ]
        for bad in bad_jsons:
            try:
                data = json.loads(bad)
                start_raw = data.get("start", [0.0, 0.0, 0.0])
                goal_raw = data.get("goal", [0.0, 0.0, 0.0])
                # 濡傛灉 start/goal 涓嶆槸鍒楄〃锛屼細鍦?np.array() 鏃跺け璐?
                np.array(start_raw, dtype=np.float64)
                np.array(goal_raw, dtype=np.float64)
            except (json.JSONDecodeError, ValueError, TypeError):
                pass  # 鏈熸湜: 琚崟鑾凤紝涓嶄紶鎾?


# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
#  娴嬭瘯 3: Fallback 閫昏緫
# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

class TestSCGFallbackLogic:
    """Test documentation."""

    def test_fallback_when_scg_empty(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        # 妯℃嫙 _plan_with_scg 鐨勬牳蹇冮€昏緫
        def simulate_plan_with_scg(robot_pos, goal_xyz):
            if len(builder.nodes) == 0:
                return None  # fallback 鏉′欢

            start = np.array([robot_pos["x"], robot_pos["y"], robot_pos["z"]])
            goal = np.array(goal_xyz)
            path = planner.plan(start, goal)
            if not path.success:
                return None

            all_waypoints = []
            for seg in path.segments:
                for pt in seg.waypoints:
                    all_waypoints.append({"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])})
            return all_waypoints if len(all_waypoints) > 1 else None

        robot_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        goal = (5.0, 5.0, 0.0)
        result = simulate_plan_with_scg(robot_pos, goal)

        assert result is None, "SCG 涓虹┖鏃跺簲杩斿洖 None (瑙﹀彂 fallback)"

    def test_fallback_when_start_outside_polyhedra(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        center = np.array([5.0, 5.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=1.0)
        builder.add_polyhedron(poly)
        builder.build_edges()

        planner = SCGPathPlanner(builder)

        # 璧风偣杩滅鎵€鏈夊闈綋 鈥?_locate_polyhedron 鏈€杩戦偦鍥為€€鍒?poly 0
        start = np.array([100.0, 100.0, 0.0])
        goal = center

        result = planner.plan(start, goal)
        # 鏈€杩戦偦鍥為€€浣胯鍒掓垚鍔燂紙polyhedron_sequence 闈炵┖锛?
        assert result.success is True

    def test_fallback_when_goal_outside_polyhedra(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        center = np.array([0.0, 0.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=1.0)
        builder.add_polyhedron(poly)
        builder.build_edges()

        planner = SCGPathPlanner(builder)

        start = center
        goal = np.array([100.0, 100.0, 0.0])

        result = planner.plan(start, goal)
        # 鏈€杩戦偦鍥為€€浣胯鍒掓垚鍔燂紙polyhedron_sequence 闈炵┖锛?
        assert result.success is True

    def test_fallback_preserves_direct_goal(self):
        """
        fallback 鍚庯紝鐩爣浣嶇疆搴斾笌鍘熷 GoalResult 涓€鑷达紙涓嶈 SCG 瑕嗙洊锛夈€?

        妯℃嫙 _handle_navigate_result 涓?SCG 澶辫触鏃剁洿鎺ヤ娇鐢ㄥ師濮?target_x/y/z銆?
        """
        target_x, target_y, target_z = 3.14, 2.72, 0.0

        # 妯℃嫙 SCG 涓嶅彲鐢?
        scg_waypoints = None  # 妯℃嫙 _plan_with_scg 杩斿洖 None

        # fallback 閫昏緫: 浣跨敤鍘熷鐩爣
        if scg_waypoints is None or len(scg_waypoints) <= 1:
            goal_x = target_x
            goal_y = target_y
            goal_z = target_z
        else:
            final_wp = scg_waypoints[-1]
            goal_x = final_wp["x"]
            goal_y = final_wp["y"]
            goal_z = final_wp["z"]

        assert goal_x == target_x
        assert goal_y == target_y
        assert goal_z == target_z

    def test_no_fallback_when_scg_provides_path(self):
        """
        SCG 鎴愬姛鏃讹紝璺緞搴旀潵鑷?SCG 澶氳矾寰勭偣锛岃€岄潪鍗曠偣鐩爣銆?

        浣跨敤涓や釜鐩搁偦澶氶潰浣撻獙璇?SCG 鐢熸垚鐨勮矾寰勭偣澶氫簬 1 涓€?
        """
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        start = c1
        goal = c2

        path = planner.plan(start, goal)

        if path.success:
            # 鏀堕泦鎵€鏈夎矾寰勭偣
            all_waypoints = []
            seen = set()
            for seg in path.segments:
                for pt in seg.waypoints:
                    key = (round(pt[0], 4), round(pt[1], 4), round(pt[2], 4))
                    if key not in seen:
                        seen.add(key)
                        all_waypoints.append({"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])})

            # SCG 鎴愬姛鏃跺簲鏈夊浜?1 涓矾寰勭偣
            assert len(all_waypoints) > 1, (
                f"SCG should produce more than one waypoint, got {len(all_waypoints)}"
            )


# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
#  娴嬭瘯 4: 杈圭晫鎯呭喌
# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

class TestSCGEdgeCases:
    """Test documentation."""

    def test_plan_with_nan_start(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([float("nan"), 0.0, 0.0])
        goal = np.array([1.0, 1.0, 0.0])
        result = planner.plan(start, goal)
        assert result.success is False

    def test_plan_with_inf_goal(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([float("inf"), 0.0, 0.0])
        result = planner.plan(start, goal)
        assert result.success is False

    def test_scg_result_distance_is_nonnegative(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        result = planner.plan(c1, c2)

        if result.success:
            assert result.total_distance >= 0.0

    def test_planning_time_is_measured(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 10.0, 0.0])
        result = planner.plan(start, goal)
        assert result.planning_time >= 0.0

    def test_json_roundtrip_waypoints(self):
        """Test documentation."""
        original_waypoints = [
            [1.2345, -2.6789, 0.0],
            [3.0001, 4.9999, 1.5],
        ]
        json_str = json.dumps({"waypoints": original_waypoints})
        recovered = json.loads(json_str)["waypoints"]

        for orig, rec in zip(original_waypoints, recovered):
            for a, b in zip(orig, rec):
                assert abs(a - b) < 1e-9, f"JSON roundtrip澶辫触: {a} != {b}"

    def test_multiple_calls_independent(self):
        """Test documentation."""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        results = []
        for _ in range(3):
            start = np.array([0.0, 0.0, 0.0])
            goal = np.array([float(_ + 1), 0.0, 0.0])
            results.append(planner.plan(start, goal))

        # 鎵€鏈夌粨鏋滈兘搴旇鏄嫭绔嬬殑 SCGPath 瀵硅薄
        assert len(results) == 3
        for r in results:
            assert isinstance(r, SCGPath)
