from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

from core.msgs.geometry import Pose, PoseStamped, Vector3
from core.msgs.nav import Path as NavPath
from sim.scripts import dynamic_obstacle_local_planner_gate
from sim.scripts.dynamic_obstacle_local_planner_gate import run_gate


def test_dynamic_obstacle_local_planner_gate_replans_without_hardware_cmd_vel():
    pytest.importorskip("_nav_core")

    report = run_gate(backend="nanobind")

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["backend_actual"] == "nanobind"
    assert report["dynamic_replan_verified"] is True
    assert report["obstacle_response_verified"] is True
    assert report["clear_path_recovery_verified"] is True
    assert report["min_clearance_m"] >= 0.25

    phases = {item["name"]: item for item in report["phases"]}
    assert phases["clear_initial"]["avoidance_side"] == "straight"
    assert phases["obstacle_left"]["avoidance_side"] == "right"
    assert phases["obstacle_right"]["avoidance_side"] == "left"
    assert phases["clear_recovered"]["avoidance_side"] == "straight"
    assert all(item["path_frame_id"] == "map" for item in report["phases"])


def test_dynamic_obstacle_local_planner_gate_reports_effective_backend_after_fallback(
    monkeypatch,
):
    class FakeOut:
        def __init__(self):
            self._callbacks = []

        def _add_callback(self, callback):
            self._callbacks.append(callback)

        def publish(self, value):
            for callback in self._callbacks:
                callback(value)

    class FakeLocalPlanner:
        def __init__(self, backend: str):
            self._backend = "cmu_py" if backend == "nanobind" else backend
            self._backend_status = type(
                "FakeBackendStatus",
                (),
                {
                    "configured": backend,
                    "degraded": self._backend != backend,
                    "degraded_reason": "compatible _nav_core missing"
                    if self._backend != backend
                    else "",
                },
            )()
            self.local_path = FakeOut()
            self._obstacle_center_y = None

        def setup(self):
            pass

        def stop(self):
            pass

        def _on_odom(self, _odom):
            pass

        def _on_waypoint(self, _waypoint):
            pass

        def _on_added_obstacles(self, cloud):
            points = cloud.points
            self._obstacle_center_y = None if len(points) == 0 else float(points[:, 1].mean())

        def _run_nanobind(self, _timestamp):
            raise AssertionError("gate should use the effective fallback backend")

        def _run_cmu_py(self):
            center_y = self._obstacle_center_y
            if center_y is None:
                y = 0.0
            elif center_y > 0.1:
                y = -0.45
            else:
                y = 0.45

            poses = [
                PoseStamped(pose=Pose(position=Vector3(float(x), y, 0.0)))
                for x in [idx * 0.2 for idx in range(21)]
            ]
            self.local_path.publish(NavPath(poses=poses, frame_id="map", ts=1.0))

    monkeypatch.setattr(
        dynamic_obstacle_local_planner_gate,
        "LocalPlannerModule",
        FakeLocalPlanner,
    )

    report = run_gate(backend="nanobind")

    assert report["ok"] is True
    assert report["backend_requested"] == "nanobind"
    assert report["backend_actual"] == "cmu_py"
    assert report["native_backend_used"] is False
    assert report["algorithm_backends"]["local_planner"]["requested"] == "nanobind"
    assert report["algorithm_backends"]["local_planner"]["backend_actual"] == "cmu_py"
    assert report["algorithm_backends"]["local_planner"]["degraded"] is True
    assert (
        report["algorithm_backends"]["local_planner"]["degraded_reason"]
        == "compatible _nav_core missing"
    )
    assert report["algorithm_backends"]["local_planner"]["exercised_by"] == "dynamic_obstacle"
    assert report["algorithm_backends"]["path_follower"]["status"] == "not_exercised"
    assert report["dynamic_replan_verified"] is True
