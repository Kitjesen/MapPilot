"""End-to-end test: sim_nav profile no-ROS navigation pipeline.

Builds the real sim_nav profile, sends a goal, and verifies the robot moves.
The profile contract is OctoPlanner3D global planning plus the C++ local
planner/path follower. If the native kernel is not built on this machine, the
runtime tests skip instead of silently falling back to the old Python demo.
"""

from __future__ import annotations

import json
import os
import sys
import time

import numpy as np
import pytest

pytestmark = [pytest.mark.sim]

_scipy_available = True
try:
    import scipy.ndimage
except ImportError:
    _scipy_available = False

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

_SCENE = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "sim",
        "worlds",
        "mujoco",
        "building_scene.xml",
    )
)


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.1) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _build_sim_nav():
    from lingtu.assembly.profile_builder import blueprint_for_resolved_profile
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
    assert config["map_artifact_gate_required"] is False
    assert config["octoplanner3d_timeout_s"] == pytest.approx(30.0)


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
        _wait_until(lambda: bool(paths), timeout_s=35.0)
        system.stop()

        assert paths, "No path planned; OctoPlanner3D did not produce output"
        assert len(paths[0]) >= 2, f"Path too short: {len(paths[0])} waypoints"

    def test_navigation_skill_plans_path(self):
        bp = _build_sim_nav()
        system = bp.build()

        paths = []
        nav = system.get_module("nav.mission")
        skills = system.get_module("nav.skills")
        nav.global_path.subscribe(paths.append)

        system.start()
        time.sleep(1.2)
        try:
            ack = json.loads(skills.navigate_to(5.0, 3.0, z=0.0))
            assert ack["accepted"] is True, ack
            assert ack["sink"] == "module"
            _wait_until(lambda: bool(paths), timeout_s=35.0)
        finally:
            system.stop()

        assert paths, "nav.skills goal produced no global path"
        assert len(paths[0]) >= 2, f"Path too short: {len(paths[0])} waypoints"

    def test_robot_moves_toward_goal(self):
        bp = _build_sim_nav()
        system = bp.build()

        positions = []
        global_paths = []
        local_paths = []
        follower_cmds = []
        mux_cmds = []
        drv = system.get_module("StubDogModule")
        nav = system.get_module("nav.mission")
        local_planner = system.get_module("nav.local_planner")
        path_follower = system.get_module("nav.path_follower")
        velocity_mux = system.get_module("nav.velocity_mux")
        drv.odometry.subscribe(lambda o: positions.append((time.time(), np.array([o.x, o.y]))))
        nav.global_path.subscribe(lambda p: global_paths.append(p))
        local_planner.local_path.subscribe(lambda p: local_paths.append(p))
        path_follower.cmd_vel.subscribe(lambda v: follower_cmds.append(v))
        velocity_mux.driver_cmd_vel.subscribe(lambda v: mux_cmds.append(v))

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
        goal_ts = time.time()
        nav.goal_pose._deliver(goal)

        def moved_after_goal() -> bool:
            after_goal = [pos for ts, pos in positions if ts >= goal_ts]
            if len(after_goal) < 2:
                return False
            return bool(np.linalg.norm(after_goal[-1] - after_goal[0]) > 0.3)

        _wait_until(
            moved_after_goal,
            timeout_s=60.0,
        )

        system.stop()

        assert positions, "No odometry; robot did not move"
        after_goal = [pos for ts, pos in positions if ts >= goal_ts]
        assert after_goal, "No odometry after goal was sent"
        dist_moved = np.linalg.norm(after_goal[-1] - after_goal[0])
        assert global_paths, "No global path published"
        assert any(len(getattr(path, "poses", []) or []) >= 2 for path in local_paths), (
            "No trackable local path published"
        )
        assert follower_cmds, "Path follower did not publish cmd_vel"
        assert mux_cmds, "Velocity mux did not publish driver_cmd_vel"
        assert dist_moved > 0.3, (
            f"Robot barely moved: {dist_moved:.2f}m; "
            f"global_paths={len(global_paths)} local_paths={len(local_paths)} "
            f"follower_cmds={len(follower_cmds)} mux_cmds={len(mux_cmds)}"
        )
