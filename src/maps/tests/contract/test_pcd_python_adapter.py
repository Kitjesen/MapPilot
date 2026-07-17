from __future__ import annotations

from pathlib import Path

import pytest

from runtime.msgs.numpy_compat import np


def test_native_pcd_writer_filters_invalid_points(tmp_path: Path) -> None:
    from maps.adapters.python.pcd import NativePcdWriter

    output = tmp_path / "capture" / "map.pcd"
    points = np.asarray(
        [
            [0.0, 1.0, 2.0, 7.0],
            [float("nan"), 0.0, 0.0, 8.0],
            [600.0, 0.0, 0.0, 9.0],
            [-2.0, -1.0, 0.5, 10.0],
        ],
        dtype=np.float32,
    )

    assert NativePcdWriter().write_xyz(output, points) == 2
    payload = output.read_bytes()
    assert b"WIDTH 2\n" in payload
    assert b"POINTS 2\n" in payload
    assert b"DATA binary\n" in payload


def test_native_pcd_writer_rejects_invalid_shape(tmp_path: Path) -> None:
    from maps.adapters.python.pcd import NativePcdWriter

    with pytest.raises(ValueError, match="Nx3"):
        NativePcdWriter().write_xyz(
            tmp_path / "map.pcd",
            np.zeros((2, 2), dtype=np.float32),
        )
