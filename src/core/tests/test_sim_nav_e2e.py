"""End-to-end test: sim_nav profile — pure Python navigation pipeline.

Builds the sim_nav blueprint (StubDogModule + SimPointCloudProvider +
OccupancyGridModule + NavigationModule A* + PathFollowerModule pid),
sends a goal, and verifies the robot moves toward it.

TODO: Replace time.sleep with threading.Event for costmap readiness
      synchronization. SimPointCloudProvider publishes at 2Hz; the
      costmap-arrived event can gate test progression.
      The poll loop in test_robot_moves_toward_goal is acceptable.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import pytest

pytestmark = [pytest.mark.sim]

_scipy_available = True
try:
    import scipy.ndimage  # noqa: F401 -- availability check for skipUnless
except ImportError:
    _scipy_available = False

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

_SCENE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "sim", "worlds", "mujoco", "building_scene.xml"
))


def _build_sim_nav():
    from core.blueprints.full_stack import full_stack_blueprint
    return full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        planner_backend="astar",
        scene_xml=_SCENE,
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        # Start inside building_scene (robot_placeholder is at 2,3)
        initial_x=2.0,
        initial_y=3.0,
        odom_frame_id="map",
    )


@pytest.mark.skipif(not _scipy_available, reason="scipy not installed in this environment")
class TestSimNavEndToEnd:

    def test_blueprint_has_required_modules(self):
        bp = _build_sim_nav()
        names = {e.name for e in bp._entries}
        assert "SimPointCloudProvider" in names
        assert "OccupancyGridModule" in names
        assert "NavigationModule" in names

    def test_costmap_generated_on_start(self):
        """SimPointCloudProvider → OccupancyGridModule produces a costmap."""
        bp = _build_sim_nav()
        system = bp.build()

        # Subscribe to OccupancyGridModule's costmap OUTPUT (not nav's input)
        costmaps = []
        og = system.get_module("OccupancyGridModule")
        og.costmap.subscribe(lambda c: costmaps.append(c))

        system.start()
        time.sleep(1.2)
        system.stop()

        assert len(costmaps) > 0, "No costmap produced by OccupancyGridModule"
        cm = costmaps[0]
        assert "grid" in cm
        assert "resolution" in cm
        assert cm["grid"].shape[0] > 0
        # Verify obstacles exist in the grid
        assert cm["grid"].max() > 0, "Costmap is all-free — no obstacles detected"

    def test_navigation_plans_path(self):
        """Send a goal via _deliver and verify A* produces a path."""
        bp = _build_sim_nav()
        system = bp.build()

        paths = []
        nav = system.get_module("NavigationModule")
        nav.global_path.subscribe(lambda p: paths.append(p))

        system.start()
        time.sleep(1.2)  # wait for costmap

        # Inject goal directly into NavigationModule's In port
        from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
        goal = PoseStamped(
            pose=Pose(position=Vector3(5.0, 3.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
            ts=time.time(), frame_id="map",
        )
        nav.goal_pose._deliver(goal)
        time.sleep(1.2)  # allow planning
        system.stop()

        assert len(paths) > 0, "No path planned — A* did not produce output"
        path = paths[0]
        assert len(path) >= 2, f"Path too short: {len(path)} waypoints"
        print(f"A* path: {len(path)} waypoints, "
              f"start=({path[0][0]:.1f},{path[0][1]:.1f}), "
              f"end=({path[-1][0]:.1f},{path[-1][1]:.1f})")

    def test_robot_moves_toward_goal(self):
        """Full loop: goal → A* → PathFollower → cmd_vel → odometry."""
        bp = _build_sim_nav()
        system = bp.build()

        positions = []
        drv = system.get_module("StubDogModule")
        drv.odometry.subscribe(lambda o: positions.append(np.array([o.x, o.y])))

        system.start()
        time.sleep(1.2)

        from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
        goal = PoseStamped(
            pose=Pose(position=Vector3(5.0, 3.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
            ts=time.time(), frame_id="map",
        )
        nav = system.get_module("NavigationModule")
        nav.goal_pose._deliver(goal)

        # Run for up to 10s, check every 100ms
        deadline = time.time() + 10.0
        while time.time() < deadline:
            time.sleep(0.1)
            if positions and np.linalg.norm(positions[-1] - np.array([5.0, 3.0])) < 2.0:
                break

        system.stop()

        assert len(positions) > 0, "No odometry — robot did not move"
        dist_moved = np.linalg.norm(positions[-1] - positions[0])
        assert dist_moved > 0.3, f"Robot barely moved: {dist_moved:.2f}m"
        print(f"Robot moved {dist_moved:.1f}m, "
              f"final=({positions[-1][0]:.1f},{positions[-1][1]:.1f}), "
              f"{len(positions)} odom updates")
