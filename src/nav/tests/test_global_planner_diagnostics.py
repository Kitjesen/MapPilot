"""Focused tests for GlobalPlanner report diagnostics."""

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from maps.artifacts import build_saved_map_metadata, sha256_file
from nav.services.plan.global_planner.service import GlobalPlanner
from runtime.runtime_interface import TOPICS, topic_default_frame_id


class _EmptyPctBackend:
    _last_plan_error = "pct native plan raised exception"
    _last_plan_diagnostics = {
        "planner": "pct",
        "stage": "native_plan_exception",
        "start_xy": [0.0, 0.0],
        "goal_xy": [1.0, 0.0],
        "error_type": "RuntimeError",
        "error_message": "native planner transient state",
    }

    def plan(self, _start, _goal):
        return []


class _FallbackDirectBackend:
    _last_plan_reached_goal = True
    _last_plan_diagnostics = {"planner": "direct", "stage": "fallback_success"}

    def __init__(self):
        self.calls = 0

    def plan(self, start, goal):
        self.calls += 1
        return [np.asarray(start, dtype=float), np.asarray(goal, dtype=float)]


class _UnavailableOctoBackend:
    available = False
    _load_error = "OctoPlanner3D headless executable not configured"


class _SuccessfulRustPctBackend:
    _last_plan_reached_goal = True

    def __init__(self):
        self._last_plan_diagnostics = {}

    def plan(self, start, goal):
        self._last_plan_diagnostics = {
            "planner": "pct",
            "stage": "native_plan_success",
            "pct_planner_runtime": {
                "runtime": "rust_process",
                "ok": True,
            },
            "last_path_mode": "rust_optimized_trajectory",
            "pct_planner_path_mode": "optimized_trajectory",
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": True,
            "pct_optimizer_linear_solver": "block_tridiagonal",
            "pct_optimizer_linear_solve_fallbacks": 0,
            "pct_optimizer_initial_cost": 42.0,
            "pct_optimizer_final_cost": 3.5,
            "pct_optimizer_iterations": 8,
            "pct_optimizer_accepted_steps": 5,
            "pct_optimizer_input_states": 24,
        }
        return [np.asarray(start, dtype=float), np.asarray(goal, dtype=float)]


def _write_minimal_pcd(path: Path) -> None:
    path.write_text(
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH 1\n"
        "HEIGHT 1\n"
        "POINTS 1\n"
        "DATA ascii\n"
        "0.0 0.0 0.0\n",
        encoding="utf-8",
    )


def _write_active_octomap_map(
    tmp_path: Path,
    *,
    occupancy: bool = True,
    map_name: str = "active",
    activate: bool = True,
) -> Path:
    maps_dir = tmp_path / "maps"
    active_dir = maps_dir / map_name
    active_dir.mkdir(parents=True, exist_ok=True)
    pcd = active_dir / "map.pcd"
    octomap = active_dir / "octomap.ot"
    _write_minimal_pcd(pcd)
    octomap.write_bytes(f"# OctoMap OcTree binary placeholder: {map_name}".encode())

    frame_id = topic_default_frame_id(TOPICS.saved_map_cloud)
    source_profile = "unit_test"
    data_source = "unit_test"
    map_sha = sha256_file(pcd)
    artifacts: dict[str, dict[str, Any]] = {
        "map_pcd": {
            "path": "map.pcd",
            "source_profile": source_profile,
            "data_source": data_source,
            "slam_source": "unit_test_slam",
            "frame_id": frame_id,
            "point_count": 1,
            "sha256": map_sha,
        },
        "octomap": {
            "path": "octomap.ot",
            "source_map_sha256": map_sha,
            "source_profile": source_profile,
            "data_source": data_source,
            "frame_id": frame_id,
            "sha256": sha256_file(octomap),
        },
    }
    if occupancy:
        occupancy_path = active_dir / "occupancy.npz"
        np.savez(
            str(occupancy_path),
            grid=np.zeros((3, 3), dtype=np.float32),
            resolution=np.array(0.5),
            origin=np.array([-0.5, -0.5]),
        )
        artifacts["occupancy_grid"] = {
            "path": "occupancy.npz",
            "source_map_sha256": map_sha,
            "source_profile": source_profile,
            "data_source": data_source,
            "frame_id": frame_id,
            "sha256": sha256_file(occupancy_path),
        }
    metadata = build_saved_map_metadata(
        source_profile=source_profile,
        data_source=data_source,
        slam_source="unit_test_slam",
        localization_source="unit_test_localizer",
        mapping_source="unit_test_mapping",
        frame_id=frame_id,
        artifacts=artifacts,
    )
    (active_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    if activate:
        (maps_dir / "active_map.txt").write_text(f"{map_name}\n", encoding="utf-8")
    return maps_dir


def test_global_planner_preserves_rust_pct_optimizer_diagnostics():
    svc = GlobalPlanner(
        planner_name="pct",
        plan_safety_policy="off",
        downsample_dist=0.1,
    )
    svc._backend = _SuccessfulRustPctBackend()
    svc._map_artifact_gate = {
        "required": False,
        "ok": True,
        "reason": "not_required",
        "blockers": [],
    }

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([2.5, 0.0, 0.0], dtype=float),
    )

    assert len(path) == 2
    report = svc.last_plan_report
    diagnostics = report["planner_diagnostics"]
    assert report["primary_planner"] == "pct"
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"] == ""
    assert diagnostics["pct_planner_runtime"] == {
        "runtime": "rust_process",
        "ok": True,
    }
    assert diagnostics["last_path_mode"] == "rust_optimized_trajectory"
    assert diagnostics["pct_planner_path_mode"] == "optimized_trajectory"
    assert diagnostics["pct_optimizer_enabled"] is True
    assert diagnostics["pct_optimizer_attempted"] is True
    assert diagnostics["pct_optimizer_accepted"] is True
    assert diagnostics["pct_optimizer_linear_solver"] == "block_tridiagonal"
    assert diagnostics["pct_optimizer_linear_solve_fallbacks"] == 0
    assert diagnostics["pct_optimizer_final_cost"] < diagnostics["pct_optimizer_initial_cost"]
    assert diagnostics["pct_optimizer_iterations"] == 8
    assert diagnostics["pct_optimizer_accepted_steps"] == 5
    assert diagnostics["pct_optimizer_input_states"] == 24
    status = svc.backend_status()
    assert status["backend"] == "pct"
    assert status["degraded"] is False


def test_global_planner_reports_backend_diagnostics_on_pct_failure():
    svc = GlobalPlanner(planner_name="pct", plan_safety_policy="reject")
    svc._backend = _EmptyPctBackend()
    svc._map_artifact_gate = {
        "required": True,
        "ok": True,
        "reason": "saved_map_artifact_ok",
        "blockers": [],
    }

    with pytest.raises(RuntimeError, match="pct native plan raised exception"):
        svc.plan(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
        )

    report = svc.last_plan_report
    diagnostics = report["planner_diagnostics"]
    assert diagnostics["stage"] == "native_plan_exception"
    assert diagnostics["error_type"] == "RuntimeError"
    assert report["rejected_plans"][0]["planner_diagnostics"] == diagnostics


def test_global_planner_canonicalizes_octplanner_alias():
    svc = GlobalPlanner(planner_name="octplanner")

    assert svc.planner_name == "octoplanner3d"


def test_global_planner_status_reports_unavailable_octoplanner3d_without_fallback_before_plan():
    svc = GlobalPlanner(
        planner_name="octoplanner3d",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="direct",
    )
    svc._backend = _UnavailableOctoBackend()

    status = svc.backend_status()

    assert status["configured_backend"] == "octoplanner3d"
    assert status["backend"] == "octoplanner3d"
    assert status["fallback_backend"] == "direct"
    assert status["degraded"] is True
    assert status["degraded_reason"] == ("OctoPlanner3D headless executable not configured")


def test_global_planner_pct_falls_back_when_primary_returns_empty_path():
    fallback = _FallbackDirectBackend()
    svc = GlobalPlanner(
        planner_name="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="direct",
    )
    svc._backend = _EmptyPctBackend()
    svc._map_artifact_gate = {
        "required": False,
        "ok": True,
        "reason": "not_required",
        "blockers": [],
    }
    svc._create_backend = lambda name=None: fallback

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([1.0, 0.0, 0.0], dtype=float),
    )

    assert fallback.calls == 1
    assert len(path) == 2
    report = svc.last_plan_report
    assert report["primary_planner"] == "pct"
    assert report["selected_planner"] == "direct"
    assert report["fallback_reason"] == "pct native plan raised exception"
    assert report["rejected_plans"][0]["planner"] == "pct"
    assert report["rejected_plans"][0]["planner_diagnostics"]["stage"] == ("native_plan_exception")
    status = svc.backend_status()
    assert status["backend"] == "direct"
    assert status["degraded"] is True


def test_global_planner_pct_falls_back_when_primary_plan_raises():
    class RaisingBackend:
        def plan(self, _start, _goal):
            raise TimeoutError("planner timed out")

    fallback = _FallbackDirectBackend()
    svc = GlobalPlanner(
        planner_name="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="direct",
    )
    svc._backend = RaisingBackend()
    svc._map_artifact_gate = {
        "required": False,
        "ok": True,
        "reason": "not_required",
        "blockers": [],
    }
    svc._create_backend = lambda name=None: fallback

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([1.0, 0.0, 0.0], dtype=float),
    )

    assert fallback.calls == 1
    assert len(path) == 2
    report = svc.last_plan_report
    assert report["selected_planner"] == "direct"
    assert report["fallback_reason"] == "planner timed out"
    assert report["rejected_plans"][0]["planner_diagnostics"] == {
        "stage": "backend_plan_exception",
        "error_type": "TimeoutError",
        "error_message": "planner timed out",
    }


def test_octoplanner3d_does_not_fallback_to_legacy_planner_when_primary_fails():
    fallback = _FallbackDirectBackend()
    svc = GlobalPlanner(
        planner_name="octoplanner3d",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="direct",
        map_artifact_gate_required=False,
    )
    svc._backend = _EmptyPctBackend()
    svc._map_artifact_gate = {
        "required": False,
        "ok": True,
        "reason": "not_required",
        "blockers": [],
    }
    svc._create_backend = lambda name=None: fallback

    with pytest.raises(RuntimeError, match="pct native plan raised exception"):
        svc.plan(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([1.0, 0.0, 0.0], dtype=float),
        )

    assert fallback.calls == 0
    report = svc.last_plan_report
    assert report["primary_planner"] == "octoplanner3d"
    assert report["selected_planner"] == "octoplanner3d"
    assert report["fallback_reason"] == "pct native plan raised exception"


def test_octoplanner3d_resolves_active_octomap_and_requires_metadata_gate(
    tmp_path,
    monkeypatch,
):
    maps_dir = _write_active_octomap_map(tmp_path)
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    svc = GlobalPlanner(planner_name="octoplanner3d")

    assert svc._resolve_map_artifact_path().endswith("octomap.ot")
    gate = svc._validate_map_artifact_gate()
    assert gate["required"] is True
    assert gate["ok"] is True, gate
    assert gate["planner"] == "octoplanner3d"
    assert gate["octomap"].endswith("octomap.ot")
    assert "octomap" in gate["checked_required_artifacts"]


def test_octoplanner3d_active_gate_rejects_missing_octomap(tmp_path, monkeypatch):
    maps_dir = _write_active_octomap_map(tmp_path)
    (maps_dir / "active" / "octomap.ot").unlink()
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    svc = GlobalPlanner(planner_name="octoplanner3d")

    gate = svc._validate_map_artifact_gate()
    assert gate["required"] is True
    assert gate["ok"] is False
    assert any("octomap" in blocker for blocker in gate["blockers"])


def test_octoplanner3d_active_gate_rejects_octomap_from_stale_pointcloud(
    tmp_path,
    monkeypatch,
):
    maps_dir = _write_active_octomap_map(tmp_path)
    pcd = maps_dir / "active" / "map.pcd"
    pcd.write_text(
        pcd.read_text(encoding="utf-8").replace(
            "0.0 0.0 0.0\n",
            "1.0 0.0 0.0\n",
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    svc = GlobalPlanner(planner_name="octoplanner3d")

    gate = svc._validate_map_artifact_gate()
    assert gate["required"] is True
    assert gate["ok"] is False
    assert any("sha256" in blocker or "source_map_sha256" in blocker for blocker in gate["blockers"]), gate


def test_octoplanner3d_active_gate_rejects_tampered_octomap(tmp_path, monkeypatch):
    maps_dir = _write_active_octomap_map(tmp_path)
    (maps_dir / "active" / "octomap.ot").write_bytes(b"tampered octomap")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    svc = GlobalPlanner(planner_name="octoplanner3d")

    gate = svc._validate_map_artifact_gate()
    assert gate["required"] is True
    assert gate["ok"] is False
    assert any("octomap" in blocker and "sha256" in blocker for blocker in gate["blockers"]), gate


@pytest.mark.parametrize("map_only", [False, True])
def test_octoplanner3d_revalidates_artifacts_before_every_plan(
    tmp_path,
    monkeypatch,
    map_only,
):
    maps_dir = _write_active_octomap_map(tmp_path)
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))
    backend = _FallbackDirectBackend()
    svc = GlobalPlanner(planner_name="octoplanner3d", plan_safety_policy="off")
    svc._backend = backend
    svc._map_artifact_gate = svc._validate_map_artifact_gate()
    assert svc._map_artifact_gate["ok"] is True

    (maps_dir / "active" / "octomap.ot").write_bytes(b"tampered after setup")

    plan_call = svc.plan_map_only if map_only else svc.plan
    with pytest.raises(RuntimeError, match="saved map artifact gate failed"):
        plan_call(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([1.0, 0.0, 0.0], dtype=float),
        )

    assert backend.calls == 0


@pytest.mark.parametrize("map_only", [False, True])
def test_octoplanner3d_rejects_plan_when_backend_is_bound_to_previous_active_map(
    tmp_path,
    monkeypatch,
    map_only,
):
    maps_dir = _write_active_octomap_map(tmp_path, map_name="map_a")
    _write_active_octomap_map(
        tmp_path,
        map_name="map_b",
        activate=False,
    )
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    backend = _FallbackDirectBackend()
    loaded_paths: list[str] = []

    def create_backend(_name, map_path, _obstacle_thr, **_kwargs):
        loaded_paths.append(str(Path(map_path).resolve()))
        return backend

    monkeypatch.setattr(
        "nav.services.plan.global_planner.service.create_planner_backend",
        create_backend,
    )
    svc = GlobalPlanner(planner_name="octoplanner3d", plan_safety_policy="off")
    svc.setup()
    assert loaded_paths == [str((maps_dir / "map_a" / "octomap.ot").resolve())]

    (maps_dir / "active_map.txt").write_text("map_b\n", encoding="utf-8")

    plan_call = svc.plan_map_only if map_only else svc.plan
    with pytest.raises(RuntimeError, match="reload"):
        plan_call(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([1.0, 0.0, 0.0], dtype=float),
        )

    assert backend.calls == 0
