from __future__ import annotations

import json
import pickle
import subprocess

import pytest

from runtime.tests.numpy_guard import import_numpy_or_skip

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

from nav.services.plan.global_planner.service import GlobalPlanner  # noqa: E402
from sim.engine.scenarios.large_terrain_assets import (  # noqa: E402
    DEFAULT_START,
    build_large_terrain_assets,
)
from sim.scripts.large_terrain_nav_validation import run_validation  # noqa: E402


def _route(assets, name: str):
    return next(route for route in assets.routes if route.name == name)


def _builder_cell(tomo: dict, x: float, y: float) -> float:
    origin = tomo["origin"]
    res = float(tomo["resolution"])
    ix = int(round((x - origin[0]) / res))
    iy = int(round((y - origin[1]) / res))
    return float(tomo["data"][0, 0, ix, iy])


def test_large_terrain_assets_write_schema_and_route_catalog(tmp_path):
    assets = build_large_terrain_assets(tmp_path)

    assert assets.scene_xml.exists()
    assert assets.tomogram.exists()
    assert assets.map_pcd.exists()
    assert assets.metadata.exists()

    metadata = json.loads(assets.metadata.read_text(encoding="utf-8"))
    assert metadata["name"] == "large_terrain_field"
    assert metadata["shape_xy"] == [121, 81]
    assert len(metadata["obstacles"]) >= 10
    localization_landmarks = [
        obstacle for obstacle in metadata["obstacles"] if obstacle.get("kind") == "localization_landmark"
    ]
    assert {landmark["name"] for landmark in localization_landmarks} >= {
        "localization_start_south_panel",
        "localization_start_west_panel",
        "localization_start_south_plinth",
        "localization_start_west_plinth",
        "localization_lane_north_panel",
        "localization_lane_south_plinth",
        "localization_lane_east_panel",
        "localization_midfield_south_panel",
        "localization_gate_north_panel",
    }
    assert len(localization_landmarks) >= 16
    assert {zone["name"] for zone in metadata["terrain_zones"]} >= {
        "rough_gravel_patch",
        "slope_ramp_band",
        "ditch_no_go",
    }
    assert {route["name"] for route in metadata["routes"]} >= {
        "terrain_short",
        "terrain_long",
        "terrain_narrow_gap",
        "terrain_slope_bypass",
        "terrain_complex_slalom",
        "terrain_patrol_loop",
    }


def test_large_terrain_assets_publish_same_source_saved_map_metadata(tmp_path):
    assets = build_large_terrain_assets(tmp_path)

    metadata = json.loads(assets.metadata.read_text(encoding="utf-8"))
    map_pcd = metadata["artifacts"]["map_pcd"]
    tomogram = metadata["artifacts"]["tomogram"]

    assert metadata["source_profile"] == "large_terrain_synthetic_assets"
    assert metadata["data_source"] == "synthetic_large_terrain_geometry"
    assert metadata["frame_id"] == "map"
    assert map_pcd["sha256"]
    assert map_pcd["point_count"] > 0
    assert tomogram["sha256"]
    assert tomogram["source_map_sha256"] == map_pcd["sha256"]

def test_large_terrain_tomogram_marks_obstacles_and_terrain_costs(tmp_path):
    assets = build_large_terrain_assets(tmp_path)

    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)

    data = tomo["data"]
    assert data.shape == (5, 1, 121, 81)
    assert data[0, 0].max() >= 100.0
    assert _builder_cell(tomo, -3.5, 3.9) == 18.0
    assert _builder_cell(tomo, 4.8, 4.1) == 12.0
    assert _builder_cell(tomo, 4.1, -3.1) >= 100.0
    assert _builder_cell(tomo, -5.55, 2.85) >= 100.0
    assert _builder_cell(tomo, -9.4, -6.65) >= 100.0
    assert _builder_cell(tomo, -8.0, -6.15) >= 100.0
    assert _builder_cell(tomo, 1.75, 2.75) >= 100.0
    assert _builder_cell(tomo, -9.5, -5.6) < 49.9
    assert _builder_cell(tomo, 9.4, 5.4) < 49.9


def test_large_terrain_assets_can_align_map_products_to_start_odom_frame(tmp_path):
    assets = build_large_terrain_assets(
        tmp_path,
        map_frame_origin_world_xy=DEFAULT_START[:2],
    )

    metadata = json.loads(assets.metadata.read_text(encoding="utf-8"))
    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)

    assert metadata["map_frame"] == "start_odom"
    assert metadata["map_frame_origin_world_xy"] == list(DEFAULT_START[:2])
    assert _builder_cell(tomo, 0.0, 0.0) < 49.9
    assert _builder_cell(tomo, 0.10, -1.05) >= 100.0
    assert _builder_cell(tomo, 1.65, 1.90) >= 100.0
    assert _builder_cell(tomo, 2.35, -1.55) >= 100.0
    assert _builder_cell(tomo, 3.20, -0.80) >= 100.0
    assert _builder_cell(tomo, 1.40, 0.10) < 49.9
    assert _builder_cell(tomo, 18.90, 11.00) < 49.9


def test_large_terrain_octoplanner3d_route_uses_central_gate(tmp_path):
    assets = build_large_terrain_assets(tmp_path)
    route = _route(assets, "terrain_long")

    svc = GlobalPlanner(
        planner_name="octoplanner3d",
        map_path=str(assets.tomogram),
        map_artifact_gate_required=False,
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    if getattr(svc._backend, "available", True) is False:
        pytest.skip(str(getattr(svc._backend, "_load_error", "planner unavailable")))
    path, _plan_ms = svc.plan(
        np.asarray(route.start, dtype=float),
        np.asarray(route.goal, dtype=float),
        safe_goal_tolerance=0.0,
    )

    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    assert len(xy) > 20
    assert np.linalg.norm(xy[-1] - np.asarray(route.goal[:2], dtype=float)) < 1e-6

    near_wall_crossing = xy[np.abs(xy[:, 0]) < 0.45]
    assert len(near_wall_crossing) > 0
    assert float(np.min(near_wall_crossing[:, 1])) > -1.30
    assert float(np.max(near_wall_crossing[:, 1])) < 1.15

    direct = float(np.linalg.norm(np.asarray(route.goal[:2]) - np.asarray(route.start[:2])))
    routed = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    assert routed > direct * 1.05
    assert routed >= route.min_routed_distance_m


def test_large_terrain_complex_slalom_has_safe_octoplanner3d_route(tmp_path):
    assets = build_large_terrain_assets(tmp_path)
    route = _route(assets, "terrain_complex_slalom")

    svc = GlobalPlanner(
        planner_name="octoplanner3d",
        map_path=str(assets.tomogram),
        map_artifact_gate_required=False,
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    if getattr(svc._backend, "available", True) is False:
        pytest.skip(str(getattr(svc._backend, "_load_error", "planner unavailable")))
    path, _plan_ms = svc.plan(
        np.asarray(route.start, dtype=float),
        np.asarray(route.goal, dtype=float),
        safe_goal_tolerance=0.0,
    )

    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    assert len(xy) > 30
    assert np.linalg.norm(xy[-1] - np.asarray(route.goal[:2], dtype=float)) < 1e-6
    routed = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    assert routed >= route.min_routed_distance_m


def test_large_terrain_validation_report_is_non_motion_and_route_safe(tmp_path):
    report = run_validation(tmp_path, routes=("terrain_long",))
    first_plan = report["cases"][0]["planning"][0]
    if "backend unavailable" in str(first_plan.get("error") or ""):
        pytest.skip(first_plan["error"])

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["algorithm_backends"]["local_planner"]["status"] == "not_exercised"
    assert report["algorithm_backends"]["path_follower"]["status"] == "not_exercised"
    assert report["deliverable_contract"]["checks"]["same_source_map_artifact"] is True
    assert report["map_artifacts"]["ok"] is True
    assert report["map_artifacts"]["source_contract"]["same_source_pcd"] is True
    assert report["map_artifacts"]["source_contract"]["same_source_tomogram"] is True
    case = report["cases"][0]
    assert case["route"] == "terrain_long"
    assert case["ok"] is True
    assert case["deliverable_contract"]["checks"]["same_source_map_artifact"] is True
    assert case["map_artifacts"]["ok"] is True
    assert case["planning"][0]["feasible"] is True
    assert case["planning"][0]["route_ok"] is True
    assert case["planning"][0]["path_safety"]["ok"] is True
    assert (
        case["planning"][0]["metrics"]["route_distance_m"]
        >= case["planning"][0]["metrics"]["min_required_route_distance_m"]
    )
    assert case["path_safety"]["ok"] is True
    assert case["gate_crossing"]["passed_gate"] is True


def test_large_terrain_validation_records_blocked_pct_without_faking_success(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    class BlockedService:
        def __init__(
            self,
            planner_name: str,
            map_path: str,
            obstacle_thr: float,
            downsample_dist: float,
            map_artifact_gate_required: bool = False,
        ) -> None:
            self._backend = type(
                "BlockedPCT",
                (),
                {"available": False, "_load_error": "planner runtime missing"},
            )()

        def setup(self) -> None:
            return None

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            raise RuntimeError("GlobalPlanner: planner returned empty path")

    monkeypatch.setattr(mod, "GlobalPlanner", BlockedService)
    monkeypatch.setattr(
        mod,
        "_pct_planner_runtime_evidence",
        lambda: {
            "runtime": "rust_process",
            "ok": False,
            "missing": ["gpmp_optimize"],
            "error": "Rust PCT planner process is unavailable",
        },
    )

    report = mod.run_validation(tmp_path, routes=("terrain_short",), planners=("pct",))

    assert report["ok"] is False
    assert report["pct_planner_runtime"]["runtime"] == "rust_process"
    assert report["pct_planner_runtime_ok"] is False
    assert report["environment_blockers"] == ["PCT planner runtime unavailable"]
    assert "native_runtime" not in report
    plan = report["cases"][0]["planning"][0]
    assert plan["planner"] == "pct"
    assert plan["feasible"] is False
    assert plan["blocked"] is True
    assert plan["status"] == "blocked"
    assert plan["failure_category"] == "environment_runtime"
    assert plan["pct_planner_runtime"]["runtime"] == "rust_process"
    assert plan["pct_planner_runtime_ok"] is False
    assert "native_backend_used" not in plan
    assert plan["route_ok"] is False
    assert report["cases"][0]["selection"]["selected_planner"] == ""
    assert report["cases"][0]["selection"]["selected_route_ok"] is False
    assert report["cases"][0]["selection"]["rejected_planners"][0]["reason"] == "environment_blocked"

def test_large_terrain_validation_records_pct_child_process_crash(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    assets = build_large_terrain_assets(tmp_path)
    route = _route(assets, "terrain_short")

    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=139,
            stdout="planner stdout",
            stderr="Segmentation fault (core dumped)",
        )

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    plan = mod._plan_with_backend_subprocess(
        "pct",
        assets,
        route,
        obstacle_thr=49.9,
        pct_planner_runtime={"runtime": "rust_process", "ok": True},
    )

    assert plan["feasible"] is False
    assert plan["pct_planner_runtime_ok"] is True
    assert "native_backend_used" not in plan
    assert plan["status"] == "failed"
    assert plan["failure_category"] == "planner_process_crash"
    assert plan["returncode"] == 139
    assert "exited with code 139" in plan["error"]
    assert "Segmentation fault" in plan["stderr_tail"]

def test_large_terrain_validation_rejects_path_that_does_not_reach_goal(
    tmp_path,
    monkeypatch,
):
    from sim.scripts import large_terrain_nav_validation as mod

    class FakeBackend:
        available = True
        _load_error = ""

    class PartialPathService:
        def __init__(
            self,
            planner_name: str,
            map_path: str,
            obstacle_thr: float,
            downsample_dist: float,
            map_artifact_gate_required: bool = False,
        ) -> None:
            self._planner_name = planner_name
            self._backend = FakeBackend()
            self._fallback_backend = None
            self.last_plan_report = {}

        def setup(self) -> None:
            return None

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            self.last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "fallback_reason": "",
                "policy": "observe",
                "reached_goal": False,
            }
            return [
                [float(start[0]), float(start[1]), 0.0],
                [float(start[0]) + 0.6, float(start[1]), 0.0],
            ], 1.0

    monkeypatch.setattr(mod, "GlobalPlanner", PartialPathService)

    report = mod.run_validation(tmp_path, routes=("terrain_short",), planners=("octoplanner3d",))

    plan = report["cases"][0]["planning"][0]
    assert plan["feasible"] is True
    assert plan["path_safety"]["ok"] is True
    assert plan["path_goal"]["reached_goal"] is False
    assert plan["route_ok"] is False
    assert report["cases"][0]["selection"]["selected_route_ok"] is False
    assert report["cases"][0]["selection"]["rejected_planners"][0]["reason"] == "unsafe_or_invalid_route"


def test_large_terrain_validation_records_effective_global_planner_when_service_falls_back(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    class FakeBackend:
        available = True
        _load_error = ""

    class FallbackService:
        def __init__(
            self,
            planner_name: str,
            map_path: str,
            obstacle_thr: float,
            downsample_dist: float,
            map_artifact_gate_required: bool = False,
        ) -> None:
            self._planner_name = planner_name
            self._backend = FakeBackend()
            self._fallback_backend = FakeBackend()
            self.last_plan_report = {}

        def setup(self) -> None:
            return None

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            self.last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": "octoplanner3d",
                "fallback_reason": "pct path_safety failed",
                "policy": "fallback_astar",
                "rejected_plans": [
                    {"planner": self._planner_name, "reason": "unsafe_primary_path"},
                ],
                "reached_goal": True,
            }
            return [
                [float(start[0]), float(start[1]), 0.0],
                [float(goal[0]), float(goal[1]), 0.0],
            ], 1.0

    monkeypatch.setattr(mod, "GlobalPlanner", FallbackService)
    monkeypatch.setattr(
        mod,
        "_pct_planner_runtime_evidence",
        lambda: {"runtime": "rust_process", "ok": True},
    )

    report = mod.run_validation(tmp_path, routes=("terrain_short",), planners=("pct",))

    plan = report["cases"][0]["planning"][0]
    assert plan["planner"] == "pct"
    assert plan["planner_requested"] == "pct"
    assert plan["selected_planner"] == "octoplanner3d"
    assert plan["fallback_reason"] == "pct path_safety failed"
    assert plan["plan_safety_policy"] == "fallback_astar"
    assert plan["rejected_plans"][0]["planner"] == "pct"
    assert plan["pct_planner_runtime_ok"] is True
    assert "native_backend_used" not in plan


def test_large_terrain_pct_runtime_evidence_reports_selected_rust_process(monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    monkeypatch.setattr(
        mod,
        "inspect_pct_runtime",
        lambda root: {
            "ok": False,
            "planner_runtime": {
                "requested": "rust_process",
                "resolved": "rust_process",
                "supported": True,
                "error": "",
            },
            "rust_optimizer_call_mode": "process",
            "searched": ["pct/runtime/rust/gpmp_optimize"],
            "required": ["gpmp_optimize"],
            "missing": ["gpmp_optimize"],
            "recommended_build_command": "cargo build --release --bin gpmp_optimize",
            "error": "Rust PCT planner process is unavailable",
            "platform_system": "windows",
            "python_tag": "py313",
        },
    )

    evidence = mod._pct_planner_runtime_evidence()

    assert evidence["runtime"] == "rust_process"
    assert evidence["ok"] is False
    assert evidence["call_mode"] == "process"
    assert evidence["missing"] == ["gpmp_optimize"]
    assert "cargo build" in evidence["recommended_build_command"]
    assert "parity_requirements" not in evidence


def test_large_terrain_pct_runtime_evidence_keeps_explicit_native_parity_requirements(
    monkeypatch,
):
    from sim.scripts import large_terrain_nav_validation as mod

    monkeypatch.setattr(
        mod,
        "inspect_pct_runtime",
        lambda root: {
            "ok": False,
            "planner_runtime": {
                "requested": "native",
                "resolved": "native",
                "supported": True,
                "error": "",
            },
            "platform_system": "linux",
            "canonical_arch": "x86_64",
            "python_tag": "py313",
            "known_good_python_tag": "py310",
            "python_abi_matches_known_good": False,
            "host_platform_supported": True,
            "host_platform_blocker": "",
            "missing": ["a_star.cpython-313-x86_64-linux-gnu.so"],
            "error": "Legacy parity runtime is unavailable",
        },
    )

    evidence = mod._pct_planner_runtime_evidence()

    assert evidence["runtime"] == "native"
    assert evidence["ok"] is False
    assert evidence["parity_requirements"]["python_abi_matches_known_good"] is False
    assert evidence["parity_requirements"]["known_good_python_tag"] == "py310"

def test_large_terrain_validation_records_safe_fallback_selection(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    RealService = mod.GlobalPlanner
    probe = RealService(planner_name="octoplanner3d", map_artifact_gate_required=False)
    probe.setup()
    if getattr(probe._backend, "available", True) is False:
        pytest.skip(str(getattr(probe._backend, "_load_error", "planner unavailable")))

    class MixedService:
        def __init__(
            self,
            planner_name: str,
            map_path: str,
            obstacle_thr: float,
            downsample_dist: float,
            map_artifact_gate_required: bool = False,
        ) -> None:
            self._planner_name = planner_name
            self._inner = RealService(
                planner_name="octoplanner3d",
                map_path=map_path,
                map_artifact_gate_required=map_artifact_gate_required,
                obstacle_thr=obstacle_thr,
                downsample_dist=downsample_dist,
            )
            self._backend = type(
                "NativePCT",
                (),
                {"available": True, "_load_error": ""},
            )()

        def setup(self) -> None:
            if self._planner_name == "octoplanner3d":
                self._inner.setup()
                self._backend = self._inner._backend

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            if self._planner_name == "pct":
                return [[float(start[0]), float(start[1]), 0.0], [float(goal[0]), float(goal[1]), 0.0]], 1.0
            return self._inner.plan(start, goal, safe_goal_tolerance=safe_goal_tolerance)

    monkeypatch.setattr(mod, "GlobalPlanner", MixedService)
    monkeypatch.setattr(
        mod,
        "_pct_planner_runtime_evidence",
        lambda: {"runtime": "rust_process", "ok": True},
    )

    report = mod.run_validation(tmp_path, routes=("terrain_long",), planners=("pct", "octoplanner3d"))

    case = report["cases"][0]
    assert case["ok"] is False
    assert case["planning"][0]["planner"] == "pct"
    assert case["planning"][0]["route_ok"] is False
    assert case["planning"][1]["planner"] == "octoplanner3d"
    assert case["planning"][1]["route_ok"] is True
    assert case["selection"]["primary_planner"] == "pct"
    assert case["selection"]["selected_planner"] == "octoplanner3d"
    assert case["selection"]["fallback_used"] is True
