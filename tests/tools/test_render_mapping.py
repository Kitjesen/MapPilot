from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "tools/bench/render_mapping.py"
SPEC = importlib.util.spec_from_file_location("render_mapping", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def binary_cloud(path: Path) -> None:
    """Write a tiny synthetic parser fixture, never dataset evidence."""
    values = np.zeros(9, dtype=[("intensity", "<f4"), ("z", "<f8"), ("x", "<f4"), ("y", "<f4")])
    values["x"] = np.arange(9)
    values["y"] = np.arange(9) * 2
    values["z"] = np.arange(9) * 3
    values["z"][1] = 100
    values["x"][3] = np.nan
    header = b"VERSION .7\nFIELDS intensity z x y\nSIZE 4 8 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 9\nHEIGHT 1\nPOINTS 9\nDATA binary\n"
    path.write_bytes(header + values.tobytes())


def test_binary_sampling_preserves_axis_order_and_full_unsampled_bounds(tmp_path):
    cloud = tmp_path / "map.pcd"
    binary_cloud(cloud)
    points, info = MODULE.load_cloud_sample(cloud, max_points=3)
    np.testing.assert_allclose(points, [[0, 0, 0], [4, 8, 12], [8, 16, 24]])
    assert info["source_points"] == 9
    assert info["finite_points"] == 8
    assert info["rendered_points"] == 3
    assert info["bounds_max_m"] == [8, 16, 100]
    np.testing.assert_array_equal(MODULE.load_cloud_sample(cloud, 3)[0], points)


def test_binary_payload_truncation_is_not_rendered_as_a_complete_map(tmp_path):
    cloud = tmp_path / "map.pcd"
    binary_cloud(cloud)
    cloud.write_bytes(cloud.read_bytes()[:-1])
    with pytest.raises(ValueError, match="shorter than POINTS"):
        MODULE.load_cloud_sample(cloud)


def test_ascii_cloud_counts_vector_fields_when_locating_xyz(tmp_path):
    cloud = tmp_path / "map.pcd"
    cloud.write_text("FIELDS normal x y z\nCOUNT 3 1 1 1\nPOINTS 2\nDATA ascii\n0 0 1 2 3 4\n0 0 1 5 6 7\n", encoding="ascii")
    points, info = MODULE.load_cloud_sample(cloud)
    np.testing.assert_array_equal(points, [[2, 3, 4], [5, 6, 7]])
    assert info["bounds_min_m"] == [2, 3, 4]


def test_plan_failure_never_displays_a_stale_route(tmp_path):
    result = tmp_path / "result.json"
    result.write_text(json.dumps({"ok": False, "reached_goal": False, "error": "start_snap_exhausted", "path": [[0, 0, 0]]}), encoding="utf-8")
    points, info = MODULE.load_plan(result)
    assert points.shape == (0, 3)
    assert info["error"] == "start_snap_exhausted"
    assert info["returned_path_points"] == 1
    assert info["displayed_path_points"] == 0
    result.write_text(json.dumps({"ok": True, "reached_goal": True, "path": [[0, 0, 0], [1, 2, 3]]}), encoding="utf-8")
    np.testing.assert_array_equal(MODULE.load_plan(result)[0], [[0, 0, 0], [1, 2, 3]])


def test_render_exports_separate_figures_and_reports_test_input_truthfully(tmp_path):
    map_dir = tmp_path / "synthetic-map"
    map_dir.mkdir()
    binary_cloud(map_dir / "map.pcd")
    trajectory = tmp_path / "estimated.map.tum"
    trajectory.write_text("1 0 0 0 0 0 0 1\n2 1 2 3 0 0 0 1\n", encoding="ascii")
    report = MODULE.render(map_dir, trajectory, tmp_path / "figures", "Synthetic rendering test only", max_points=3)
    assert report["font"] == {"family": "Arial", "size_pt": 10}
    assert report["execution_validated"] is False
    assert report["planning"]["status"] == "not_run"
    assert len(report["files"]) == 6
    assert all(Path(path).stat().st_size > 100 for path in report["files"])
    assert (tmp_path / "figures/figures.json").is_file()


def test_focus_is_a_declared_display_crop_and_does_not_overwrite_full_evidence(tmp_path):
    map_dir = tmp_path / "synthetic-map"
    map_dir.mkdir()
    binary_cloud(map_dir / "map.pcd")
    source = (map_dir / "map.pcd").read_bytes()
    trajectory = tmp_path / "estimated.map.tum"
    trajectory.write_text("1 0 0 0 0 0 0 1\n2 1 2 3 0 0 0 1\n", encoding="ascii")
    output = tmp_path / "figures"
    output.mkdir()
    (output / "map3d.png").write_bytes(b"existing full-view evidence")
    report = MODULE.render(map_dir, trajectory, output, "Synthetic focus test only", max_points=3,
                           focus_trajectory=True)
    assert report["display_view"] == {
        "mode": "trajectory_bounds_plus_margin", "margin_m": 5.0,
        "bounds_min_m": [-5.0, -5.0, -5.0], "bounds_max_m": [6.0, 7.0, 8.0],
        "source_data_changed": False,
    }
    assert report["cloud"]["source_points"] == 9
    assert report["cloud"]["bounds_max_m"] == [8, 16, 100]
    assert report["cloud"]["sampled_points_before_focus"] == 3
    assert report["cloud"]["rendered_points"] == 1
    assert (map_dir / "map.pcd").read_bytes() == source
    assert (output / "map3d.png").read_bytes() == b"existing full-view evidence"
    assert (output / "figures_focus.json").is_file()
    assert not (output / "figures.json").exists()
    assert all("_focus." in Path(path).name for path in report["files"])
