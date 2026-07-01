from __future__ import annotations

import hashlib
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from nav.services.plan.global_planner.algorithm.pct.runtime import preview


def test_build_preview_report_includes_samples_runtime_and_diagnostics() -> None:
    planner = SimpleNamespace(
        last_path_mode="optimized_trajectory",
        last_optimizer_enabled=True,
        last_optimizer_attempted=True,
        last_optimizer_accepted=True,
        last_optimizer_reject_reason="",
        last_optimizer_blocked_sample_count=0,
        last_raw_path_blocked_sample_count=0,
        optimize_trajectory=True,
        use_quintic=True,
        max_heading_rate=10,
        obstacle_thr=49.9,
    )
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )
    result = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.1],
            [2.0, 0.0, 0.2],
            [3.0, 0.0, 0.3],
            [4.0, 0.0, 0.4],
        ],
        dtype=np.float64,
    )

    report = preview.build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=result,
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([4.0, 0.0, 0.4], dtype=np.float64),
        sample_count=3,
    )

    assert report["ok"] is True
    assert report["status_code"] == "SUCCESS"
    assert report["status"]["reached_goal"] is True
    assert report["status"]["obstacle_clear"] is True
    assert report["schema"] == "lingtu.pct.preview.actual.v2"
    assert report["runtime"]["backend"] in {"native", "rust_process"}
    assert report["runtime"]["lib_dir"] == str(paths.lib_dir)
    assert report["runtime"]["arch"] == "x86_64"
    assert report["runtime"]["python"] == "py310"
    assert report["runtime"]["native_binary_format"] in {"linux_elf", "rust_cdylib", "rust_process_exe"}
    assert report["input"] == {
        "start": [0.0, 0.0, 0.0],
        "goal": [4.0, 0.0, 0.4],
        "obstacle_thr": 49.9,
    }
    assert report["path"]["shape"] == [5, 3]
    assert report["path"]["finite"] is True
    assert report["path"]["count"] == 5
    assert np.isclose(report["path"]["distance_m"], 4.019950248448356)
    assert report["path"]["start_error_m"] == 0.0
    assert report["path"]["goal_error_m"] == 0.0
    assert report["path"]["goal_tolerance_m"] == 0.3
    assert report["path"]["reached_goal"] is True
    assert report["path"]["samples"]["by_arclength_fraction"] == [
        {"fraction": 0.0, "point": [0.0, 0.0, 0.0]},
        {"fraction": 0.25, "point": [1.0, 0.0, 0.1]},
        {"fraction": 0.5, "point": [2.0, 0.0, 0.2]},
        {"fraction": 0.75, "point": [3.0, 0.0, 0.3]},
        {"fraction": 1.0, "point": [4.0, 0.0, 0.4]},
    ]
    assert report["path_samples"] == [
        {"index": 0, "point": [0.0, 0.0, 0.0]},
        {"index": 2, "point": [2.0, 0.0, 0.2]},
        {"index": 4, "point": [4.0, 0.0, 0.4]},
    ]
    assert report["diagnostics"]["last_path_mode"] == "optimized_trajectory"
    assert report["diagnostics"]["last_optimizer_attempted"] is True
    assert np.isclose(report["path_distance_m"], 4.019950248448356)
    assert report["goal_error_m"] == 0.0


def test_build_preview_report_includes_tomogram_fingerprint(tmp_path: Path) -> None:
    tomogram_path = tmp_path / "tomogram.pickle"
    payload = {
        "schema_version": "test.tomogram.v1",
        "data": np.zeros((2, 3, 4), dtype=np.float32),
        "resolution": 0.2,
    }
    tomogram_path.write_bytes(pickle.dumps(payload))
    digest = hashlib.sha256(tomogram_path.read_bytes()).hexdigest()
    planner = SimpleNamespace(obstacle_thr=12.5)
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )

    report = preview.build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=np.asarray([[0.0, 0.0, 0.0]], dtype=np.float64),
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        tomogram_path=tomogram_path,
    )

    input_spec = report["input"]
    assert input_spec["tomogram"] == str(tomogram_path)
    assert input_spec["tomogram_sha256"] == digest
    assert input_spec["tomogram_size_bytes"] == tomogram_path.stat().st_size
    assert input_spec["tomogram_data_shape"] == [2, 3, 4]
    assert input_spec["tomogram_data_dtype"] == "float32"
    assert input_spec["tomogram_schema_version"] == "test.tomogram.v1"
    assert input_spec["obstacle_thr"] == 12.5
    assert input_spec["tomogram_file"]["keys"] == ["data", "resolution", "schema_version"]


def test_build_preview_report_keeps_diagnostics_on_empty_path() -> None:
    planner = SimpleNamespace(
        last_path_mode="",
        last_optimizer_enabled=True,
        last_optimizer_attempted=False,
    )
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )

    report = preview.build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=[],
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([4.0, 0.0, 0.0], dtype=np.float64),
    )

    assert report["ok"] is False
    assert report["status_code"] == "NO_PATH"
    assert report["status"]["code"] == "NO_PATH"
    assert report["schema"] == "lingtu.pct.preview.actual.v2"
    assert report["error"] == "pct returned no path"
    assert report["runtime"]["backend"] in {"native", "rust_process"}
    assert report["diagnostics"]["last_optimizer_attempted"] is False


def test_build_preview_report_rejects_goal_out_of_map() -> None:
    planner = SimpleNamespace(
        resolution=0.2,
        center=np.asarray([0.0, 0.0], dtype=np.float64),
        map_dim=[10, 10],
        offset=np.asarray([5, 5], dtype=np.int32),
        last_optimizer_accepted=True,
        last_optimizer_reject_reason="",
    )
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )
    result = np.asarray([[0.0, 0.0, 0.0], [0.8, 0.8, 0.0]], dtype=np.float64)

    report = preview.build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=result,
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([5.0, 5.0, 0.0], dtype=np.float64),
    )

    assert report["ok"] is False
    assert report["status_code"] == "GOAL_OUT_OF_MAP"
    assert report["status"]["goal_in_bounds"] is False
    assert report["status"]["goal_bounds"]["clipped_idx"] == [9, 9]
    assert report["status"]["goal_bounds"]["clamp_error_m"] > 0.0


class _BlockedPathPlanner:
    resolution = 0.2
    center = np.asarray([0.0, 0.0], dtype=np.float64)
    map_dim = [100, 100]
    offset = np.asarray([50, 50], dtype=np.int32)
    last_optimizer_accepted = False
    last_optimizer_reject_reason = "optimized_trajectory_hard_obstacle"

    def _hard_obstacle_sample_count(self, _path):
        return 2


def test_build_preview_report_rejects_hard_obstacle_path() -> None:
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )
    result = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)

    report = preview.build_preview_report(
        planner=_BlockedPathPlanner(),
        runtime_paths=paths,
        result=result,
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
    )

    assert report["ok"] is False
    assert report["status_code"] == "HARD_OBSTACLE_COLLISION"
    assert report["status"]["returned_path_blocked_sample_count"] == 2
    assert report["path"]["obstacle_clear"] is False


def test_build_preview_report_rejects_vertical_kinematic_path() -> None:
    planner = SimpleNamespace(
        resolution=0.2,
        center=np.asarray([0.0, 0.0], dtype=np.float64),
        map_dim=[100, 100],
        offset=np.asarray([50, 50], dtype=np.int32),
        last_optimizer_accepted=True,
        last_optimizer_reject_reason="",
    )
    paths = SimpleNamespace(
        lib_dir=Path("/native/pct"),
        canonical_arch="x86_64",
        python_tag="py310",
    )
    result = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 1.0],
            [1.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    report = preview.build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=result,
        start=np.asarray([0.0, 0.0, 0.0], dtype=np.float64),
        goal=np.asarray([1.0, 0.0, 1.0], dtype=np.float64),
    )

    assert report["ok"] is False
    assert report["status_code"] == "KINEMATICALLY_INVALID"
    assert report["status"]["kinematics"]["vertical_like_segment_count"] == 1
    assert report["status"]["kinematics"]["max_slope_dz_per_m"] > 10.0
