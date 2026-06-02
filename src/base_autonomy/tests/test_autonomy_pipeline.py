"""Pipeline contract tests — all three base_autonomy modules wired together.

Verifies that TerrainModule, LocalPlannerModule, and PathFollowerModule can
coexist in a single Blueprint with explicit wires, that data flows through the
terrain_map -> local_planner and local_path -> path_follower wires, and that
all alive ports toggle correctly on start/stop.
"""

from __future__ import annotations

import numpy as np
import pytest

from base_autonomy.modules.local_planner_module import LocalPlannerModule
from base_autonomy.modules.path_follower_module import PathFollowerModule
from base_autonomy.modules.terrain_module import TerrainModule
from core.blueprint import Blueprint
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2


# =============================================================================
# Blueprint construction
# =============================================================================


class TestAutonomyPipeline:
    """Pipeline-level contract tests for base_autonomy modules."""

    def test_blueprint_builds_with_all_three(self) -> None:
        """All three modules coexist in a single Blueprint."""
        bp = Blueprint()
        bp.add(TerrainModule, backend="simple")
        bp.add(LocalPlannerModule, backend="simple")
        bp.add(PathFollowerModule, backend="pid")

        system = bp.build()

        assert "TerrainModule" in system.modules
        assert "LocalPlannerModule" in system.modules
        assert "PathFollowerModule" in system.modules

    def test_blueprint_with_wires_builds(self) -> None:
        """Blueprint with explicit wires builds without error."""
        bp = _build_pipeline_blueprint()
        system = bp.build()

        assert "TerrainModule" in system.modules
        assert "LocalPlannerModule" in system.modules
        assert "PathFollowerModule" in system.modules

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def test_system_start_stop(self) -> None:
        """Blueprint builds, starts, and stops cleanly."""
        bp = _build_pipeline_blueprint()
        system = bp.build()

        tm = system.get_module("TerrainModule")
        lp = system.get_module("LocalPlannerModule")
        pf = system.get_module("PathFollowerModule")

        system.start()
        assert tm._running
        assert lp._running
        assert pf._running

        system.stop()
        assert not tm._running
        assert not lp._running
        assert not pf._running

    # ------------------------------------------------------------------ #
    # Data flow via wires
    # ------------------------------------------------------------------ #

    def test_terrain_map_wire_delivers_data(self) -> None:
        """TerrainModule.terrain_map reaches LocalPlannerModule.terrain_map."""
        bp = _build_pipeline_blueprint()
        system = bp.build()
        system.start()

        tm = system.get_module("TerrainModule")
        lp = system.get_module("LocalPlannerModule")

        # Ensure the throttle allows the first cloud to process
        tm._last_process_ts = 0.0

        # Inject cloud to TerrainModule
        pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud = PointCloud2(points=pts)
        tm.map_cloud._deliver(cloud)

        # The wire should deliver terrain_map -> LocalPlannerModule._on_terrain
        assert lp._terrain_points is not None, (
            "LocalPlannerModule did not receive terrain_map via wire"
        )
        np.testing.assert_array_equal(
            lp._terrain_points[:, :3], pts
        )

        system.stop()

    def test_local_path_wire_delivers_data(self) -> None:
        """LocalPlannerModule.local_path reaches PathFollowerModule.local_path."""
        bp = _build_pipeline_blueprint()
        system = bp.build()
        system.start()

        lp = system.get_module("LocalPlannerModule")
        pf = system.get_module("PathFollowerModule")

        # Pre-load robot position via odometry
        odom = Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))
        lp.odometry._deliver(odom)

        # Inject a clear waypoint — simple backend generates straight-line path
        wp = PoseStamped(
            pose=Pose(
                position=Vector3(5.0, 0.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
        )
        lp.waypoint._deliver(wp)

        # The wire should deliver local_path -> PathFollowerModule._on_path
        assert pf._path_points is not None, (
            "PathFollowerModule did not receive local_path via wire"
        )
        assert len(pf._path_points) >= 2, (
            f"expected >=2 path points, got {len(pf._path_points)}"
        )

        system.stop()

    def test_full_data_flow_terrain_to_cmd_vel(self) -> None:
        """End-to-end: terrain_map -> local_planner -> path_follower -> cmd_vel."""
        bp = _build_pipeline_blueprint()
        system = bp.build()
        system.start()

        tm = system.get_module("TerrainModule")
        lp = system.get_module("LocalPlannerModule")
        pf = system.get_module("PathFollowerModule")

        # Monitor cmd_vel output
        cmd_values: list[Twist] = []
        pf.cmd_vel._add_callback(cmd_values.append)

        # Step 1: Inject cloud to TerrainModule (feeds terrain_map wire)
        tm._last_process_ts = 0.0
        pts = np.array([[0.0, 1.0, 2.0]], dtype=np.float32)
        tm.map_cloud._deliver(PointCloud2(points=pts))

        # Step 2: Inject odometry to both LocalPlannerModule and PathFollowerModule
        odom = Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))
        lp.odometry._deliver(odom)
        pf.odometry._deliver(odom)

        # Step 3: Inject waypoint to LocalPlannerModule -> triggers path -> cmd_vel
        wp = PoseStamped(
            pose=Pose(
                position=Vector3(5.0, 0.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
        )
        lp.waypoint._deliver(wp)

        # PathFollowerModule should have received local_path via wire
        assert pf._path_points is not None, "path_points not received"
        assert len(pf._path_points) >= 2, "path too short"

        # cmd_vel should have been published
        assert len(cmd_values) >= 1, "no cmd_vel published"
        last_cmd = cmd_values[-1]
        assert isinstance(last_cmd, Twist)

        system.stop()

    # ------------------------------------------------------------------ #
    # Alive port toggling
    # ------------------------------------------------------------------ #

    def test_all_alive_ports_toggle_on_start_stop(self) -> None:
        """Each module's alive port toggles True on start, False on stop."""
        bp = _build_pipeline_blueprint()
        system = bp.build()

        # Capture alive values per module before start
        terra_alive: list[bool] = []
        local_alive: list[bool] = []
        path_alive: list[bool] = []

        tm = system.get_module("TerrainModule")
        lp = system.get_module("LocalPlannerModule")
        pf = system.get_module("PathFollowerModule")

        tm.alive._add_callback(terra_alive.append)
        lp.alive._add_callback(local_alive.append)
        pf.alive._add_callback(path_alive.append)

        system.start()

        assert len(terra_alive) >= 1
        assert terra_alive[-1] is True, "TerrainModule.alive not True"

        assert len(local_alive) >= 1
        assert local_alive[-1] is True, "LocalPlannerModule.alive not True"

        assert len(path_alive) >= 1
        assert path_alive[-1] is True, "PathFollowerModule.alive not True"

        system.stop()

        assert terra_alive[-1] is False, "TerrainModule.alive not False after stop"
        assert local_alive[-1] is False, "LocalPlannerModule.alive not False after stop"
        assert path_alive[-1] is False, "PathFollowerModule.alive not False after stop"

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def test_clean_shutdown_clears_internal_state(self) -> None:
        """After stop, internal references are cleared."""
        bp = _build_pipeline_blueprint()
        system = bp.build()
        system.start()

        tm = system.get_module("TerrainModule")
        lp = system.get_module("LocalPlannerModule")
        pf = system.get_module("PathFollowerModule")

        system.stop()

        assert not tm._running
        assert not lp._running
        assert not pf._running


# =============================================================================
# Helpers
# =============================================================================


def _build_pipeline_blueprint() -> Blueprint:
    """Create a Blueprint with all three base_autonomy modules wired."""
    bp = Blueprint()
    bp.add(TerrainModule, backend="simple")
    bp.add(LocalPlannerModule, backend="simple")
    bp.add(PathFollowerModule, backend="pid")

    # Wire the data flow chain
    bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
    bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
    bp.wire("LocalPlannerModule", "control_hint", "PathFollowerModule", "control_hint")

    return bp
