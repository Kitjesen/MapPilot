from __future__ import annotations

import importlib.util
import json
import pickle
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest


def _load_tool():
    path = Path(__file__).resolve().parents[3] / "scripts" / "planning" / "plan_preview.py"
    spec = importlib.util.spec_from_file_location("lingtu_plan_preview_tool", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_tomogram(root: Path) -> Path:
    active = root / "active"
    active.mkdir(parents=True)
    data = np.full((5, 3, 4, 6), 10.0, dtype=np.float32)
    data[0, 0, 1, 1] = 80.0
    path = active / "tomogram.pickle"
    with path.open("wb") as fh:
        pickle.dump({"data": data, "resolution": 0.5, "center": [10.0, 20.0]}, fh)
    return path


def _write_layered_tomogram(root: Path) -> Path:
    active = root / "active"
    active.mkdir(parents=True)
    data = np.full((5, 3, 4, 6), 50.0, dtype=np.float32)
    data[0, 2, :, :] = 0.0
    path = active / "tomogram.pickle"
    with path.open("wb") as fh:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.5,
                "center": [10.0, 20.0],
                "slice_h0": -0.3,
                "slice_dh": 0.25,
            },
            fh,
        )
    return path


def _write_elevated_layer_tomogram(root: Path) -> Path:
    active = root / "active"
    active.mkdir(parents=True)
    data = np.full((5, 3, 4, 6), 50.0, dtype=np.float32)
    data[0, 2, :, :] = 0.0
    data[3, 2, :, :] = 2.6
    path = active / "tomogram.pickle"
    with path.open("wb") as fh:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.5,
                "center": [10.0, 20.0],
                "slice_h0": -0.3,
                "slice_dh": 0.25,
            },
            fh,
        )
    return path


def _write_multi_elevation_tomogram(root: Path) -> Path:
    active = root / "active"
    active.mkdir(parents=True)
    data = np.full((5, 3, 4, 6), 50.0, dtype=np.float32)
    data[0, 0, :, :] = 0.0
    data[0, 2, :, :] = 0.0
    data[3, 0, :, :] = -0.3
    data[3, 2, :, :] = 2.6
    path = active / "tomogram.pickle"
    with path.open("wb") as fh:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.5,
                "center": [10.0, 20.0],
                "slice_h0": -0.3,
                "slice_dh": 0.25,
            },
            fh,
        )
    return path


def _touch_pct_extensions(lib_dir: Path) -> None:
    abi = f"{sys.version_info.major}{sys.version_info.minor}"
    for name in ("a_star", "ele_planner", "traj_opt"):
        (lib_dir / f"{name}.cpython-{abi}-x86_64-linux-gnu.so").write_bytes(b"")
    for name in (
        "libmetis-gtsam.so",
        "libgtsam.so",
        "libcommon_smoothing.so",
        "liba_star_search.so",
        "libmap_manager.so",
        "libgpmp_optimizer.so",
        "libele_planner_lib.so",
    ):
        (lib_dir / name).write_bytes(b"")


def test_tomogram_info_reports_bounds_and_last_pose_out_of_bounds(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("6.0 18.0 1.57\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)

    assert info.to_dict()["raw_shape"] == [5, 3, 4, 6]
    assert info.to_dict()["ground_shape"] == [4, 6]
    assert info.to_dict()["resolution"] == 0.5
    assert info.to_dict()["origin"] == [8.5, 19.0]
    assert info.to_dict()["max_xy"] == [11.0, 20.5]
    assert info.to_dict()["free_cells"] == 23
    assert last_pose is not None
    assert last_pose.start == [6.0, 18.0, 0.0]
    assert not info.in_bounds(last_pose.start)


def test_default_preview_cases_include_last_pose_and_internal_route(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("6.0 18.0 1.57\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(info, last_pose=last_pose)

    assert [case.name for case in cases] == ["last_pose_to_near_free", "internal_free_route"]
    assert cases[0].start == [6.0, 18.0, 0.0]
    assert info.in_bounds(cases[0].goal)
    assert info.in_bounds(cases[1].start)
    assert info.in_bounds(cases[1].goal)


def test_explicit_goal_uses_last_pose_when_requested(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("9.0 19.5 0.4\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(
        info,
        explicit_goal=[10.0, 20.0, 0.0],
        last_pose=last_pose,
        use_last_pose=True,
    )

    assert len(cases) == 1
    assert cases[0].name == "requested"
    assert cases[0].source == "last_pose_to_explicit_goal"
    assert cases[0].start == [9.0, 19.5, 0.0]
    assert cases[0].goal == [10.0, 20.0, 0.0]


def test_2d_requested_goal_uses_tomogram_traversable_height(tmp_path):
    tool = _load_tool()
    tomo = _write_layered_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("9.0 19.5 0.4\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(
        info,
        explicit_goal=[10.0, 20.0],
        last_pose=last_pose,
        use_last_pose=True,
    )

    assert len(cases) == 1
    assert cases[0].start[2] == 0.2
    assert cases[0].goal[2] == 0.2


def test_2d_requested_goal_uses_tomogram_elevation_height_when_available(tmp_path):
    tool = _load_tool()
    tomo = _write_elevated_layer_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("9.0 19.5 0.4\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(
        info,
        explicit_goal=[10.0, 20.0],
        last_pose=last_pose,
        use_last_pose=True,
    )

    assert len(cases) == 1
    assert cases[0].start[2] == pytest.approx(2.6)
    assert cases[0].goal[2] == pytest.approx(2.6)


def test_2d_last_pose_zero_placeholder_does_not_prefer_upper_layer(tmp_path):
    tool = _load_tool()
    tomo = _write_multi_elevation_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("9.0 19.5 0.4\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(
        info,
        explicit_goal=[10.0, 20.0],
        last_pose=last_pose,
        use_last_pose=True,
    )

    assert len(cases) == 1
    assert cases[0].start[2] == pytest.approx(-0.3)
    assert cases[0].goal[2] == pytest.approx(-0.3)


def test_out_of_bounds_case_is_skipped_before_native_planner(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    info = tool.load_tomogram_info(tomo)

    result = tool.summarize_case(
        info,
        tool.PreviewCase(
            "out_of_bounds",
            start=[1.0, 1.0, 0.0],
            goal=[10.0, 20.0, 0.0],
            source="test",
        ),
        tomogram_path=tomo,
        planner="pct",
        planning_frame="odom",
        obstacle_thr=49.9,
        timeout_s=0.1,
        downsample_dist=2.0,
        allow_out_of_bounds=False,
        pct_runtime_libs={"ok": True},
        max_endpoint_z_error_m=1.0,
    )

    assert result["skipped"] is True
    assert result["reasons"] == ["start_out_of_bounds"]
    assert result["feasible"] is False
    assert "backend_class" not in result


def test_main_outputs_strict_json_for_astar_preview(tmp_path):
    tool = _load_tool()
    _write_tomogram(tmp_path)

    proc = subprocess.run(
        [
            sys.executable,
            str(Path(tool.__file__)),
            "--map-root",
            str(tmp_path),
            "--planner",
            "astar",
            "--internal-only",
            "--strict",
            "--compact",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=10,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    json.dumps(payload, allow_nan=False)
    assert proc.stderr == ""
    assert payload["ok"] is True
    assert payload["cases"][0]["feasible"] is True


def test_extract_first_json_object_tolerates_native_trailing_stdout():
    tool = _load_tool()

    payload, extra = tool.extract_first_json_object('{"ok":true}\nshape: 91, 6\n')

    assert payload == {"ok": True}
    assert "shape: 91" in extra


def test_subprocess_failure_result_keeps_standard_case_shape():
    tool = _load_tool()

    result = tool.subprocess_failure_result(
        planner="pct",
        reason="planning_timeout",
        error="timed out",
        wall_ms=123.4567,
        output="native line\n",
    )

    assert result["planner"] == "pct"
    assert result["backend_available"] is False
    assert result["preview"]["feasible"] is False
    assert result["preview"]["reasons"] == ["planning_timeout"]
    assert result["segments_m"] == []
    assert result["native_output_tail"] == ["native line"]


def test_pct_preview_child_disables_optimizer_by_default(monkeypatch, tmp_path):
    tool = _load_tool()
    captured_env = {}

    def fake_run(cmd, **kwargs):
        captured_env.update(kwargs["env"])
        return subprocess.CompletedProcess(
            cmd,
            returncode=0,
            stdout='{"planner":"pct","preview":{"ok":true,"feasible":true}}\n',
            stderr="",
        )

    monkeypatch.delenv(tool.PCT_OPTIMIZE_TRAJECTORY_ENV, raising=False)
    monkeypatch.setattr(tool.subprocess, "run", fake_run)

    result = tool.run_navigation_preview(
        tomogram_path=tmp_path / "tomogram.pickle",
        case=tool.PreviewCase(
            "requested",
            start=[0.0, 0.0, 0.0],
            goal=[1.0, 1.0, 0.0],
            source="test",
        ),
        planner="pct",
        planning_frame="odom",
        obstacle_thr=49.9,
        timeout_s=1.0,
        downsample_dist=2.0,
    )

    assert captured_env[tool.PCT_OPTIMIZE_TRAJECTORY_ENV] == "0"
    assert result["pct_optimizer_enabled"] is False
    assert result["pct_planner_path_mode"] == "native_astar_raw_path"


def test_pct_preview_child_preserves_explicit_optimizer_env(monkeypatch, tmp_path):
    tool = _load_tool()
    captured_env = {}

    def fake_run(cmd, **kwargs):
        captured_env.update(kwargs["env"])
        return subprocess.CompletedProcess(
            cmd,
            returncode=0,
            stdout='{"planner":"pct","preview":{"ok":true,"feasible":true}}\n',
            stderr="",
        )

    monkeypatch.setenv(tool.PCT_OPTIMIZE_TRAJECTORY_ENV, "1")
    monkeypatch.setattr(tool.subprocess, "run", fake_run)

    result = tool.run_navigation_preview(
        tomogram_path=tmp_path / "tomogram.pickle",
        case=tool.PreviewCase(
            "requested",
            start=[0.0, 0.0, 0.0],
            goal=[1.0, 1.0, 0.0],
            source="test",
        ),
        planner="pct",
        planning_frame="odom",
        obstacle_thr=49.9,
        timeout_s=1.0,
        downsample_dist=2.0,
    )

    assert captured_env[tool.PCT_OPTIMIZE_TRAJECTORY_ENV] == "1"
    assert result["pct_optimizer_enabled"] is True
    assert result["pct_planner_path_mode"] == "optimized_trajectory"


def test_child_preview_exception_keeps_standard_case_shape(capsys):
    tool = _load_tool()

    rc = tool.child_preview("not-base64")
    captured = capsys.readouterr()
    result = json.loads(captured.out)

    assert rc == 0
    assert result["planner"] == "unknown"
    assert result["backend_available"] is False
    assert result["backend_class"] is None
    assert result["has_map"] is False
    assert result["preview"]["feasible"] is False
    assert result["preview"]["reasons"] == ["planning_child_exception"]
    assert result["first_point"] is None
    assert result["last_point"] is None
    assert result["segments_m"] == []
    assert "wall_ms" in result


def test_case_failures_preserves_strict_bounds_and_backend_reasons():
    tool = _load_tool()

    failures = tool.case_failures(
        [
            {"name": "a", "skipped": True, "reasons": ["start_out_of_bounds"]},
            {
                "name": "b",
                "skipped": False,
                "backend_available": False,
                "feasible": False,
                "start_in_bounds": True,
                "goal_in_bounds": False,
            },
        ],
        strict=True,
        allow_out_of_bounds=False,
    )

    assert failures == [
        "a:start_out_of_bounds",
        "b:backend_unavailable",
        "b:infeasible",
        "b:goal_out_of_bounds",
    ]


def test_case_failures_rejects_unsafe_strict_preview():
    tool = _load_tool()

    failures = tool.case_failures(
        [
            {
                "name": "pct_case",
                "skipped": False,
                "backend_available": True,
                "feasible": True,
                "start_in_bounds": True,
                "goal_in_bounds": True,
                "preview": {
                    "selected_planner": "pct",
                    "path_safety": {
                        "ok": False,
                        "blocked_sample_count": 22,
                    }
                },
                "z_consistency": {"ok": True},
            },
        ],
        strict=True,
        allow_out_of_bounds=False,
    )

    assert failures == ["pct_case:path_safety_failed:22"]


def test_path_z_consistency_rejects_pct_endpoint_height_drift():
    tool = _load_tool()

    report = tool.path_z_consistency(
        start=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0, 0.0],
        path=[
            {"x": 0.0, "y": 0.0, "z": -5.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
        ],
        max_endpoint_error_m=1.0,
    )

    assert report["ok"] is False
    assert report["start_error_m"] == 5.0


def test_case_failures_rejects_pct_z_inconsistency_in_strict_mode():
    tool = _load_tool()

    failures = tool.case_failures(
        [
            {
                "name": "pct_case",
                "skipped": False,
                "backend_available": True,
                "feasible": True,
                "start_in_bounds": True,
                "goal_in_bounds": True,
                "preview": {
                    "selected_planner": "pct",
                    "path_safety": {"ok": True},
                },
                "z_consistency": {
                    "ok": False,
                    "start_error_m": 5.0,
                    "goal_error_m": 0.0,
                },
            },
        ],
        strict=True,
        allow_out_of_bounds=False,
    )

    assert failures == ["pct_case:z_endpoint_inconsistent:start=5.0,goal=0.0"]


def test_missing_aarch64_pct_libs_skip_native_backend(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    info = tool.load_tomogram_info(tomo)

    result = tool.summarize_case(
        info,
        tool.PreviewCase(
            "internal",
            start=[9.0, 19.5, 0.0],
            goal=[10.0, 20.0, 0.0],
            source="test",
        ),
        tomogram_path=tomo,
        planner="pct",
        planning_frame="odom",
        obstacle_thr=49.9,
        timeout_s=0.1,
        downsample_dist=2.0,
        allow_out_of_bounds=False,
        pct_runtime_libs={"ok": False, "missing": ["ele_planner.cpython-310-aarch64-linux-gnu.so"]},
        max_endpoint_z_error_m=1.0,
    )

    assert result["skipped"] is True
    assert result["reasons"] == ["pct_runtime_libs_missing"]
    assert result["backend_available"] is False
    assert "ele_planner" in result["error"]


def test_aarch64_pct_runtime_report_lists_missing_root_libs(tmp_path):
    tool = _load_tool()
    report = tool.pct_runtime_lib_report(repo_root=tmp_path, machine="aarch64")
    abi = f"{sys.version_info.major}{sys.version_info.minor}"

    assert report["canonical_arch"] == "aarch64"
    assert report["ok"] is False
    assert f"ele_planner.cpython-{abi}-aarch64-linux-gnu.so" in report["missing"]


def test_pct_runtime_report_honors_env_override(tmp_path, monkeypatch):
    tool = _load_tool()
    env_dir = tmp_path / "runtime_env"
    fallback_dir = tmp_path / "src/global_planning/pct_planner_runnable/native/x86_64"
    env_dir.mkdir(parents=True)
    fallback_dir.mkdir(parents=True)
    _touch_pct_extensions(env_dir)
    _touch_pct_extensions(fallback_dir)
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    report = tool.pct_runtime_lib_report(repo_root=tmp_path, machine="x86_64")

    assert report["ok"] is True
    assert Path(report["lib_dir"]) == env_dir
    assert report["missing"] == []
