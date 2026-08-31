from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from sim.planning import octoplanner3d_route_viz as viz


def _write_binary_pcd(path: Path) -> None:
    header = "\n".join(
        [
            "# .PCD v0.7",
            "VERSION 0.7",
            "FIELDS x y z _",
            "SIZE 4 4 4 4",
            "TYPE F F F U",
            "COUNT 1 1 1 1",
            "WIDTH 2",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            "POINTS 2",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    payload = b"".join(
        [
            struct.pack("<fffI", 1.0, 2.0, 3.0, 10),
            struct.pack("<fffI", -1.5, 0.25, 4.5, 11),
        ]
    )
    path.write_bytes(header + payload)


def test_default_route_viz_uses_octoplanner3d_pcd():
    default_pcd = viz.DEFAULT_PCD.as_posix()

    assert default_pcd.endswith(
        "sim/fixtures/octoplanner3d/building2_9.pcd"
    )
def test_load_pcd_xyz_reads_binary_xyz_with_extra_fields(tmp_path: Path):
    pcd = tmp_path / "tiny.pcd"
    _write_binary_pcd(pcd)

    pts = viz.load_pcd_xyz(pcd)

    assert pts.shape == (2, 3)
    assert pts[0].tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert pts[1].tolist() == pytest.approx([-1.5, 0.25, 4.5])


def test_load_route_map_reads_octoplanner3d_route_list(tmp_path: Path):
    route_json = tmp_path / "routes.json"
    route_json.write_text(
        json.dumps(
            [
                {
                    "name": "cross_floor_up",
                    "ok": True,
                    "path": [[0, 0, 0.3], [1, 1, 1.2]],
                }
            ]
        ),
        encoding="utf-8",
    )

    routes = viz.load_route_map(route_json)

    assert set(routes) == {"cross_floor_up"}
    assert routes["cross_floor_up"].shape == (2, 3)
    assert routes["cross_floor_up"][-1].tolist() == pytest.approx([1.0, 1.0, 1.2])


def test_route_metrics_reports_slope_and_z_range():
    route = viz.np.asarray(
        [[0.0, 0.0, 0.3], [1.0, 0.0, 0.5], [1.0, 1.0, 1.1]],
        dtype=viz.np.float32,
    )

    metrics = viz.route_metrics(route)

    assert metrics["points"] == pytest.approx(3.0)
    assert metrics["z_min"] == pytest.approx(0.3)
    assert metrics["z_max"] == pytest.approx(1.1)
    assert metrics["z_range"] == pytest.approx(0.8)
    assert metrics["max_segment_dz"] == pytest.approx(0.6)
    assert metrics["max_segment_slope"] == pytest.approx(0.6)
