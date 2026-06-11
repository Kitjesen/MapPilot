from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from core.msgs.geometry import Pose, PoseStamped, Vector3
from core.msgs.nav import Path as NavPath
from sim.scripts import dynamic_obstacle_local_planner_gate
from sim.scripts.dynamic_obstacle_local_planner_gate import run_gate


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_dynamic_obstacle_local_planner_gate_blocks_unsupported_windows_host(
    monkeypatch,
):
    monkeypatch.setattr(dynamic_obstacle_local_planner_gate, "LocalPlannerModule", None)

    def fail_load_runtime():
        raise AssertionError("unsupported Windows guard should run before runtime import")

    monkeypatch.setattr(dynamic_obstacle_local_planner_gate, "_load_runtime", fail_load_runtime)

    report = run_gate(backend="nanobind", platform_system="Windows")

    assert report["ok"] is False
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["execution_mode"] == "host_guard"
    assert report["backend_actual"] == ""
    assert report["environment"]["accepted_host"] is False
    assert report["environment"]["blocked_reason"] == "windows_mingw_numpy_not_accepted"
    assert report["environment"]["claim_boundary"] == (
        "environment_blocked_no_algorithm_claim"
    )
    assert report["environment"]["manual_diagnosis_flag"] == (
        "--allow-unstable-windows-numpy"
    )
    assert any("Windows/MINGW NumPy" in item for item in report["errors"])


def test_dynamic_obstacle_local_planner_gate_cli_writes_red_report_on_windows(tmp_path: Path):
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/dynamic_obstacle_local_planner_gate.py"),
            "--backend",
            "nanobind",
            "--json-out",
            str(report_path),
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    if sys.platform.startswith("win"):
        assert probe.returncode == 1
        report = json.loads(report_path.read_text(encoding="utf-8"))
        assert report["ok"] is False
        assert report["execution_mode"] == "host_guard"
        assert report["environment"]["accepted_host"] is False
        assert any("Windows/MINGW NumPy" in item for item in report["errors"])
    else:
        assert report_path.exists()


def test_dynamic_obstacle_local_planner_gate_replans_without_hardware_cmd_vel():
    if sys.platform.startswith("win"):
        pytest.skip("Windows/MINGW NumPy local planner runtime is intentionally blocked")
    pytest.importorskip("_nav_core")

    report = run_gate(backend="nanobind")

    assert report["ok"] is True
    assert report["execution_mode"] == "runtime_gate"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["backend_actual"] == "nanobind"
    assert report["environment"]["accepted_host"] is True
    assert report["environment"]["claim_boundary"] == "dynamic_obstacle_algorithm_gate"
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
    if sys.platform.startswith("win"):
        pytest.skip("Windows/MINGW NumPy local planner runtime is intentionally blocked")

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
    assert report["execution_mode"] == "runtime_gate"
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
