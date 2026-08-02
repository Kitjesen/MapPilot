from __future__ import annotations

from nav.local.local_planner import LocalPlanner
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.numpy_compat import np


class _EmptyResult:
    path = []
    slow_down = 0
    near_field_stop = False
    path_found = False
    recovery_state = 0

class _RecoveryResult(_EmptyResult):
    recovery_state = 1


class _FakeCore:
    def set_vehicle(self, *args):
        pass

    def set_goal(self, *args):
        pass

    def plan(self, *args):
        return _EmptyResult()


class _RecoveryCore(_FakeCore):
    def plan(self, *args):
        return _RecoveryResult()

def test_direct_track_fallback_rejects_obstacle_on_line() -> None:
    planner = LocalPlanner(backend="nanobind", allow_direct_track_fallback=True)
    planner._core = _FakeCore()
    planner._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
    planner._terrain_points = np.array([[0.5, 0.0, 0.0]], dtype=np.float32)
    planner._latest_waypoint = PoseStamped(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0)),
        frame_id="map",
    )
    paths = []
    hints = []
    planner.local_path._add_callback(paths.append)
    planner.control_hint._add_callback(hints.append)

    planner._run_nanobind(1.0)

    assert paths[-1].poses == []
    assert hints[-1]["safety_stop"] is True
    assert hints[-1]["reason"] == "no_local_path"


def test_direct_track_fallback_does_not_override_recovery_stop() -> None:
    planner = LocalPlanner(backend="nanobind", allow_direct_track_fallback=True)
    planner._core = _RecoveryCore()
    planner._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
    planner._latest_waypoint = PoseStamped(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0)),
        frame_id="map",
    )
    paths = []
    hints = []
    planner.local_path._add_callback(paths.append)
    planner.control_hint._add_callback(hints.append)

    planner._run_nanobind(1.0)

    assert paths[-1].poses == []
    assert hints[-1]["safety_stop"] is True
    assert hints[-1]["reason"] == "no_local_path"
