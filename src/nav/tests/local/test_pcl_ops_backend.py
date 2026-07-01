from __future__ import annotations

import sys
import types

import pytest

from nav.local import pcl_ops
from runtime.msgs.numpy_compat import np


def test_pcl_ops_reports_numpy_fallback_when_native_package_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delitem(sys.modules, "lingtu_pcl_ops", raising=False)
    monkeypatch.setattr(pcl_ops.importlib, "import_module", lambda _name: (_ for _ in ()).throw(ImportError("missing")))

    info = pcl_ops.backend_info()

    assert info.available is False
    assert info.backend == "numpy_fallback"
    assert "not installed" in info.reason


def test_voxel_downsample_xyzi_uses_numpy_fallback_without_pcl() -> None:
    points = np.asarray(
        [
            [0.01, 0.01, 0.0, 1.0],
            [0.02, 0.02, 0.0, 3.0],
            [1.01, 0.0, 0.0, 5.0],
        ],
        dtype=np.float32,
    )

    out = pcl_ops.voxel_downsample_xyzi(points, 0.5, prefer_native=False)

    assert out.shape == (2, 4)
    assert np.allclose(out[0], [0.015, 0.015, 0.0, 2.0])
    assert np.allclose(out[1], [1.01, 0.0, 0.0, 5.0])


def test_voxel_downsample_xyzi_can_delegate_to_optional_native_module(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = types.SimpleNamespace(
        voxel_downsample_xyzi=lambda points, voxel_size: np.asarray([[float(voxel_size), 2.0, 3.0, 4.0]], dtype=np.float32)
    )
    monkeypatch.setattr(pcl_ops, "_load_native", lambda: fake)

    out = pcl_ops.voxel_downsample_xyzi(np.zeros((3, 4), dtype=np.float32), 0.25)

    assert out.shape == (1, 4)
    assert np.allclose(out[0], [0.25, 2.0, 3.0, 4.0])


def test_voxel_downsample_rejects_invalid_shape_and_voxel_size() -> None:
    with pytest.raises(ValueError, match="points must be shaped"):
        pcl_ops.voxel_downsample_xyzi(np.zeros((2, 5), dtype=np.float32), 0.5)
    with pytest.raises(ValueError, match="voxel_size must be positive"):
        pcl_ops.voxel_downsample_xyzi(np.zeros((2, 4), dtype=np.float32), 0.0, prefer_native=False)
