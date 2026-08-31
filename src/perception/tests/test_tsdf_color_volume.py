"""TSDF color-volume interface behavior."""

import sys
import types
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from perception.reconstruction.reconstruction_module import TSDFColorVolume


def _open3d_mock():
    open3d = types.ModuleType("open3d")
    open3d.geometry = types.ModuleType("open3d.geometry")
    open3d.geometry.Image = MagicMock(return_value=MagicMock())
    open3d.geometry.RGBDImage = MagicMock()
    open3d.geometry.RGBDImage.create_from_color_and_depth = MagicMock(return_value=MagicMock())
    open3d.camera = types.ModuleType("open3d.camera")
    open3d.camera.PinholeCameraIntrinsic = MagicMock(return_value=MagicMock())

    integration = types.ModuleType("open3d.pipelines.integration")
    volume = MagicMock()
    point_cloud = MagicMock()
    point_cloud.points = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float64)
    point_cloud.colors = np.array([[0.5, 0.5, 0.5], [1, 0, 0]], dtype=np.float64)
    volume.extract_point_cloud.return_value = point_cloud
    volume.extract_triangle_mesh.return_value = MagicMock()
    integration.ScalableTSDFVolume = MagicMock(return_value=volume)
    integration.TSDFVolumeColorType = types.SimpleNamespace(RGB8="RGB8")
    open3d.pipelines = types.SimpleNamespace(integration=integration)
    return open3d


@pytest.fixture
def open3d_mock(monkeypatch):
    open3d = _open3d_mock()
    monkeypatch.setitem(sys.modules, "open3d", open3d)
    return open3d


def test_initializes_open3d_volume(open3d_mock):
    volume = TSDFColorVolume(voxel_length=0.04, sdf_trunc=0.15)
    assert volume.voxel_length == 0.04
    open3d_mock.pipelines.integration.ScalableTSDFVolume.assert_called_once()


def test_missing_open3d_raises_runtime_error(monkeypatch):
    monkeypatch.delitem(sys.modules, "open3d", raising=False)
    real_import = __import__

    def block_open3d(name, *args, **kwargs):
        if name == "open3d" or name.startswith("open3d."):
            raise ImportError("blocked for test")
        return real_import(name, *args, **kwargs)

    with patch("builtins.__import__", side_effect=block_open3d):
        with pytest.raises(RuntimeError, match="open3d"):
            TSDFColorVolume()


def test_extract_point_cloud_returns_float32_points_and_colors(open3d_mock):
    points, colors = TSDFColorVolume().extract_point_cloud()
    assert points.shape == colors.shape == (2, 3)
    assert points.dtype == colors.dtype == np.float32


def test_extract_mesh_delegates_to_open3d(open3d_mock):
    volume = TSDFColorVolume()
    assert volume.extract_mesh() is not None
    volume._volume.extract_triangle_mesh.assert_called_once()


def test_integrate_delegates_to_open3d(open3d_mock):
    volume = TSDFColorVolume()
    volume.integrate(
        np.full((4, 4), 1000, dtype=np.uint16),
        np.zeros((4, 4, 3), dtype=np.uint8),
        np.array([[200, 0, 2], [0, 200, 2], [0, 0, 1]], dtype=np.float64),
        np.eye(4, dtype=np.float64),
    )
    volume._volume.integrate.assert_called_once()
