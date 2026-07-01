"""End-to-end test: sim_nav profile no-ROS navigation pipeline.

Builds the real sim_nav profile, sends a goal, and verifies the robot moves.
The profile contract is OctoPlanner3D global planning plus the C++ local
planner/path follower. If the native kernel is not built on this machine, the
runtime tests skip instead of silently falling back to the old Python demo.
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
    os.path.dirname(__file__),
    "..",
    "..",
    "..",
    "sim",
    "worlds",
    "mujoco",
    "building_scene.xml",
))


def _build_sim_nav():
    from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
    from runtime.profiles.resolver import resolve_profile_config

    config = resolve_profile_config(
        "sim_nav",
        overrides={
            "enable_gateway": False,
            "scene_xml": _SCENE,
            "initial_x": 2.0,
            "initial_y": 3.0,
            "odom_frame_id": "map",
        },
    )
    return blueprint_for_resolved_profile("sim_nav", config)


def _require_native_nav_kernel():
    from nav.kernel import nav_kernel_available, nav_kernel_build_hint

    if not nav_kernel_available():
        pytest.skip(nav_kernel_build_hint())


def test_sim_nav_profile_selects_octoplanner3d_and_cpp_local_chain():
    from runtime.profiles.resolver import resolve_profile_config

    config = resolve_profile_config("sim_nav")

    assert config["planner"] == "octoplanner3d"
    assert config["planner_backend"] == "octoplanner3d"
    assert config["python_autonomy_backend"] == "nanobind"
    assert config["python_path_follower_backend"] == "nav_kernel"
    assert config["fallback_planner_name"] == ""


@pytest.mark.skipif(not _scipy_available, reason="scipy not installed in this environment")
class TestSimNavEndToEnd:
    def setup_method(self):
        _require_native_nav_kernel()

    def test_blueprint_has_required_modules(self):
        bp = _build_sim_nav()
        names = {e.name for e in bp._entries}
        assert "SimPointCloudProvider" in names
        assert "OccupancyGridModule" in names
        assert "nav.mission" in names
        assert "nav.local_planner" in names
        assert "nav.path_follower" in names

    def test_costmap_generated_on_start(self):
        bp = _build_sim_nav()
        system = bp.build()

        costmaps = []
        og = system.get_module("OccupancyGridModule")
        og.costmap.subscribe(lambda c: costmaps.append(c))

        system.start()
        time.sleep(1.2)
        system.stop()

        assert costmaps, "No costmap produced by OccupancyGridModule"
        cm = costmaps[0]
        assert "grid" in cm
        assert "resolution" in cm
        assert cm["grid"].shape[0] > 0
        assert cm["grid"].max() > 0, "Costmap is all-free; no obstacles detected"

    def test_navigation_plans_path(self):
        bp = _build_sim_nav()
        system = bp.build()

        paths = []
        nav = system.get_module("nav.mission")
        nav.global_path.subscribe(lambda p: paths.append(p))

        system.start()
        time.sleep(1.2)

        from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

        goal = PoseStamped(
            pose=Pose(
                position=Vector3(5.0, 3.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            ts=time.time(),
            frame_id="map",
        )
        nav.goal_pose._deliver(goal)
        time.sleep(1.2)
        system.stop()

        assert paths, "No path planned; OctoPlanner3D did not produce output"
        assert len(paths[0]) >= 2, f"Path too short: {len(paths[0])} waypoints"

    def test_robot_moves_toward_goal(self):
        bp = _build_sim_nav()
        system = bp.build()

        positions = []
        drv = system.get_module("StubDogModule")
        drv.odometry.subscribe(lambda o: positions.append(np.array([o.x, o.y])))

        system.start()
        time.sleep(1.2)

        from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

        goal = PoseStamped(
            pose=Pose(
                position=Vector3(5.0, 3.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            ts=time.time(),
            frame_id="map",
        )
        nav = system.get_module("nav.mission")
        nav.goal_pose._deliver(goal)

        deadline = time.time() + 10.0
        while time.time() < deadline:
            time.sleep(0.1)
            if positions and np.linalg.norm(positions[-1] - np.array([5.0, 3.0])) < 2.0:
                break

        system.stop()

        assert positions, "No odometry; robot did not move"
        dist_moved = np.linalg.norm(positions[-1] - positions[0])
        assert dist_moved > 0.3, f"Robot barely moved: {dist_moved:.2f}m"
