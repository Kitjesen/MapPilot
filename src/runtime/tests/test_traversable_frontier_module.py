from __future__ import annotations

import json

import numpy as np

from runtime.msgs.geometry import Pose
from runtime.msgs.nav import Odometry


def _grid_payload(grid: np.ndarray) -> dict:
    return {
        "grid": grid,
        "resolution": 1.0,
        "origin": [-4.0, -4.0],
        "origin_x": -4.0,
        "origin_y": -4.0,
        "width": int(grid.shape[1]),
        "height": int(grid.shape[0]),
        "frame_id": "map",
    }


def _elevation_payload(grid: np.ndarray) -> dict:
    return {
        "max_z": grid,
        "resolution": 1.0,
        "origin": [-4.0, -4.0],
        "width": int(grid.shape[1]),
        "height": int(grid.shape[0]),
        "frame_id": "map",
    }


def _frontier_grid() -> np.ndarray:
    grid = np.full((9, 9), -1, dtype=np.int16)
    grid[2:7, 2:7] = 0
    grid[4, 4] = 0
    return grid


def test_traversable_frontier_module_outputs_preview_candidates_without_motion():
    from nav.exploration.traversable_frontier_module import TraversableFrontierModule

    module = TraversableFrontierModule(
        min_frontier_size=1,
        safe_distance=1.0,
        max_slope_deg=30.0,
        max_frontier_cost=80.0,
    )
    module.setup()

    published_frontiers: list[list] = []
    published_best: list[dict] = []
    module.traversable_frontiers._add_callback(published_frontiers.append)
    module.frontier_candidate._add_callback(published_best.append)

    grid = _frontier_grid()
    cost = np.zeros_like(grid, dtype=np.float32)
    slope = np.full_like(grid, 5.0, dtype=np.float32)
    clearance = np.full_like(grid, 2.0, dtype=np.float32)
    elevation = np.full_like(grid, 0.35, dtype=np.float32)

    module.odometry._deliver(Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map"))
    module.exploration_grid._deliver(_grid_payload(grid))
    module.costmap._deliver(_grid_payload(cost))
    module.fused_cost._deliver(_grid_payload(cost))
    module.slope_grid._deliver(_grid_payload(slope))
    module.esdf_field._deliver(
        {
            "distance_field": clearance,
            "resolution": 1.0,
            "origin": [-4.0, -4.0],
            "width": 9,
            "height": 9,
            "frame_id": "map",
        }
    )
    module.elevation_map._deliver(_elevation_payload(elevation))

    result = json.loads(module.refresh_candidates())

    assert result["command_published"] is False
    assert result["candidate_count"] > 0
    assert module.exploration_goal.msg_count == 0
    assert len(published_frontiers) == 1
    assert len(published_best) == 1
    best = published_best[0]
    assert best["source"] == "traversable_frontier"
    assert best["preview"] is True
    assert best["command_published"] is False
    assert best["state"] == "active"
    assert best["support_type"] == "flat"
    assert abs(best["support_height"] - 0.35) < 1e-6
    assert abs(best["centroid_3d"][2] - 0.35) < 1e-6
    assert best["reachable_score"] > 0.0
    assert isinstance(best["centroid_3d"], list)
    assert best["nearby_labels"] == []


def test_traversable_frontier_marks_steep_or_high_cost_support_blocked():
    from nav.exploration.traversable_frontier_module import TraversableFrontierModule

    module = TraversableFrontierModule(
        min_frontier_size=1,
        safe_distance=1.0,
        max_slope_deg=20.0,
        max_frontier_cost=70.0,
    )
    module.setup()

    grid = _frontier_grid()
    cost = np.zeros_like(grid, dtype=np.float32)
    slope = np.full_like(grid, 30.0, dtype=np.float32)
    clearance = np.full_like(grid, 0.2, dtype=np.float32)
    elevation = np.full_like(grid, 0.2, dtype=np.float32)

    module.odometry._deliver(Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map"))
    module.exploration_grid._deliver(_grid_payload(grid))
    module.costmap._deliver(_grid_payload(cost))
    module.fused_cost._deliver(_grid_payload(cost))
    module.slope_grid._deliver(_grid_payload(slope))
    module.esdf_field._deliver(
        {
            "distance_field": clearance,
            "resolution": 1.0,
            "origin": [-4.0, -4.0],
            "width": 9,
            "height": 9,
            "frame_id": "map",
        }
    )
    module.elevation_map._deliver(_elevation_payload(elevation))

    candidates = json.loads(module.get_traversable_frontiers())

    assert candidates
    assert candidates[0]["state"] == "blocked"
    assert candidates[0]["support_type"] == "steep"
    assert "slope_over_limit" in candidates[0]["reasons"]


def test_traversable_frontier_marks_configured_high_cost_support_blocked():
    from nav.exploration.traversable_frontier_module import TraversableFrontierModule

    module = TraversableFrontierModule(
        min_frontier_size=1,
        safe_distance=1.0,
        max_slope_deg=20.0,
        max_frontier_cost=70.0,
    )
    module.setup()

    grid = _frontier_grid()
    reachable_cost = np.zeros_like(grid, dtype=np.float32)
    fused_cost = np.full_like(grid, 75.0, dtype=np.float32)
    slope = np.full_like(grid, 5.0, dtype=np.float32)
    clearance = np.full_like(grid, 1.5, dtype=np.float32)
    elevation = np.full_like(grid, 0.2, dtype=np.float32)

    module.odometry._deliver(Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map"))
    module.exploration_grid._deliver(_grid_payload(grid))
    module.costmap._deliver(_grid_payload(reachable_cost))
    module.fused_cost._deliver(_grid_payload(fused_cost))
    module.slope_grid._deliver(_grid_payload(slope))
    module.esdf_field._deliver(
        {
            "distance_field": clearance,
            "resolution": 1.0,
            "origin": [-4.0, -4.0],
            "width": 9,
            "height": 9,
            "frame_id": "map",
        }
    )
    module.elevation_map._deliver(_elevation_payload(elevation))

    candidates = json.loads(module.get_traversable_frontiers())

    assert candidates
    assert candidates[0]["state"] == "blocked"
    assert candidates[0]["support_type"] == "blocked"
    assert candidates[0]["terrain_cost"] == 75.0
    assert "high_traversability_cost" in candidates[0]["reasons"]


def test_traversable_frontier_uses_nearby_scene_graph_semantics_without_motion():
    from runtime.msgs.geometry import Vector3
    from runtime.msgs.semantic import Detection3D, SceneGraph
    from nav.exploration.traversable_frontier_module import TraversableFrontierModule

    module = TraversableFrontierModule(
        min_frontier_size=1,
        safe_distance=1.0,
        max_slope_deg=30.0,
        max_frontier_cost=80.0,
        semantic_prior_weight=0.4,
    )
    module.setup()

    grid = _frontier_grid()
    cost = np.zeros_like(grid, dtype=np.float32)
    slope = np.full_like(grid, 5.0, dtype=np.float32)
    clearance = np.full_like(grid, 2.0, dtype=np.float32)
    elevation = np.full_like(grid, 0.35, dtype=np.float32)

    module.odometry._deliver(Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map"))
    module.exploration_grid._deliver(_grid_payload(grid))
    module.costmap._deliver(_grid_payload(cost))
    module.fused_cost._deliver(_grid_payload(cost))
    module.slope_grid._deliver(_grid_payload(slope))
    module.esdf_field._deliver(
        {
            "distance_field": clearance,
            "resolution": 1.0,
            "origin": [-4.0, -4.0],
            "width": 9,
            "height": 9,
            "frame_id": "map",
        }
    )
    module.elevation_map._deliver(_elevation_payload(elevation))

    before = json.loads(module.get_traversable_frontiers())
    assert before
    target = before[0]
    module.scene_graph._deliver(
        SceneGraph(
            objects=[
                Detection3D(
                    id="pump-a",
                    label="inspection_pump",
                    confidence=0.9,
                    position=Vector3(target["cx"], target["cy"], target["z"]),
                ),
                Detection3D(
                    id="far-box",
                    label="far_box",
                    confidence=0.9,
                    position=Vector3(target["cx"] + 20.0, target["cy"], target["z"]),
                ),
            ],
            frame_id="map",
        )
    )

    after = json.loads(module.refresh_candidates())
    candidates = json.loads(module.get_traversable_frontiers())
    enriched = next(
        item
        for item in candidates
        if abs(float(item["cx"]) - float(target["cx"])) < 1e-6
        and abs(float(item["cy"]) - float(target["cy"])) < 1e-6
    )

    assert after["command_published"] is False
    assert module.exploration_goal.msg_count == 0
    assert enriched["semantic_value"] > 0.0
    assert enriched["reachable_score"] > target["reachable_score"]
    assert enriched["nearby_labels"] == ["inspection_pump"]
    assert enriched["semantic_evidence"]["object_count"] == 1
    assert enriched["semantic_evidence"]["source"] == "scene_graph"


def test_full_stack_can_add_traversable_frontier_without_wiring_it_to_control():
    from lingtu.assembly.products.thunder import thunder_blueprint

    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=True,
        enable_traversable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    assert system.get_module("TraversableFrontierModule") is not None
    assert (
        "OccupancyGridModule",
        "exploration_grid",
        "TraversableFrontierModule",
        "exploration_grid",
    ) in system.connections
    assert (
        "TraversabilityCostModule",
        "fused_cost",
        "TraversableFrontierModule",
        "costmap",
    ) in system.connections
    assert (
        "TraversabilityCostModule",
        "fused_cost",
        "TraversableFrontierModule",
        "fused_cost",
    ) in system.connections
    assert (
        "TraversabilityCostModule",
        "slope_grid",
        "TraversableFrontierModule",
        "slope_grid",
    ) in system.connections
    assert (
        "TraversabilityCostModule",
        "esdf_field",
        "TraversableFrontierModule",
        "esdf_field",
    ) in system.connections
    assert (
        "TraversableFrontierModule",
        "traversable_frontiers",
        "GatewayModule",
        "traversable_frontiers",
    ) in system.connections
    assert (
        "TraversableFrontierModule",
        "frontier_candidate",
        "GatewayModule",
        "frontier_candidate",
    ) in system.connections
    assert (
        "ElevationMapModule",
        "elevation_map",
        "TraversableFrontierModule",
        "elevation_map",
    ) in system.connections
    assert (
        "nav.mission",
        "mission_status",
        "TraversableFrontierModule",
        "navigation_status",
    ) in system.connections
    assert (
        "TraversableFrontierModule",
        "exploration_goal",
        "nav.mission",
        "goal_pose",
    ) not in system.connections


def test_full_stack_wires_scene_graph_to_traversable_frontier_when_semantic_enabled():
    from lingtu.assembly.products.thunder import thunder_blueprint

    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        enable_map_modules=True,
        enable_traversable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    assert (
        "PerceptionModule",
        "scene_graph",
        "TraversableFrontierModule",
        "scene_graph",
    ) in system.connections
    assert (
        "TraversableFrontierModule",
        "frontier_candidate",
        "nav.mission",
        "goal_pose",
    ) not in system.connections
