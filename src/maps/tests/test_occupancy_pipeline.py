"""Focused coverage for native occupancy artifact building."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from maps.modules.service import MapsModule
from maps.services.pipeline import MapPipelineService


def _write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    count = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA ascii\n"
    )
    with path.open("w", encoding="utf-8") as handle:
        handle.write(header)
        for x, y, z in points:
            handle.write(f"{x} {y} {z}\n")


def _room_points() -> np.ndarray:
    points: list[tuple[float, float, float]] = []
    for x in np.arange(-1.5, 1.55, 0.1):
        points.append((float(x), -1.5, 0.5))
        points.append((float(x), 1.5, 0.5))
    for y in np.arange(-1.5, 1.55, 0.1):
        points.append((-1.5, float(y), 0.5))
        points.append((1.5, float(y), 0.5))
    for x in np.arange(-1.5, 1.55, 0.3):
        for y in np.arange(-1.5, 1.55, 0.3):
            points.append((float(x), float(y), -0.05))
    return np.asarray(points, dtype=np.float32)


def _native_maps_available() -> bool:
    lib_path = os.environ.get("LINGTU_MAPS_LIB")
    return bool(lib_path and Path(lib_path).is_file())


@pytest.fixture
def native_mgr(tmp_path: Path) -> MapsModule:
    if not _native_maps_available():
        pytest.skip("LINGTU_MAPS_LIB is required for native occupancy integration")
    return MapsModule(map_dir=str(tmp_path))


def test_occupancy_wrapper_returns_native_response_without_python_metadata() -> None:
    native_response = {
        "action": "build_occupancy_snapshot",
        "success": True,
        "mode": "projection_native",
        "occupancy": "native/occupancy.npz",
        "grid_shape": [4, 5],
    }

    class NativeService:
        def build_occupancy_snapshot(self, name: str) -> dict[str, Any]:
            assert name == "demo"
            return native_response

    class Storage:
        native_service = NativeService()

    pipeline = MapPipelineService(
        storage=Storage(),  # type: ignore[arg-type]
        runtime_bridge=object(),  # type: ignore[arg-type]
        source_profile="test",
        runtime_data_source="test",
    )

    assert pipeline.build_occupancy_snapshot("demo") is native_response


def test_native_projection_writes_grid_pgm_and_yaml(
    native_mgr: MapsModule,
    tmp_path: Path,
) -> None:
    map_name = "box_room"
    map_dir = tmp_path / map_name
    map_dir.mkdir()
    _write_ascii_pcd(map_dir / "map.pcd", _room_points())

    result = native_mgr._build_occupancy_snapshot(map_name)

    assert result["success"] is True, result
    assert result["mode"] == "projection_native"
    assert "metadata" not in result

    occupancy_path = Path(result["occupancy"])
    pgm_path = Path(result["pgm"])
    yaml_path = Path(result["yaml"])
    assert occupancy_path.is_file()
    assert pgm_path.is_file()
    assert yaml_path.is_file()

    data = np.load(occupancy_path)
    grid = data["grid"]
    values = set(int(value) for value in np.unique(grid))
    assert values.issubset({0, 100})
    assert {0, 100}.issubset(values)

    with pgm_path.open("rb") as handle:
        assert handle.readline().strip() == b"P5"
        width, height = [int(value) for value in handle.readline().split()]
        assert int(handle.readline()) == 255
        assert len(handle.read()) == width * height

    yaml_text = yaml_path.read_text(encoding="utf-8")
    assert "image: map.pgm" in yaml_text
    assert "mode: trinary" in yaml_text
