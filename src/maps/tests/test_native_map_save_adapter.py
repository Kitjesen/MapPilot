"""Tests for the native SLAM map-save adapter."""

from __future__ import annotations

import subprocess

import pytest

from maps.adapters.native import map_save
from maps.map_save import MapSaveError


def test_native_map_save_rejects_success_without_pcd(monkeypatch, tmp_path):
    class Completed:
        returncode = 0
        stdout = '{"success": true, "message": "accepted"}\n'
        stderr = ""

    monkeypatch.setattr(map_save, "_control_binary", lambda: "slamctl")
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: Completed())

    adapter = map_save.NativeSlamMapSaveAdapter()
    target = tmp_path / "missing" / "map.pcd"

    with pytest.raises(MapSaveError, match="did not write"):
        adapter.save_slam_map(target)


def test_native_map_save_returns_point_count(monkeypatch, tmp_path):
    class Completed:
        returncode = 0
        stdout = '{"success": true, "message": "saved"}\n'
        stderr = ""

    def fake_run(argv, **_kwargs):
        target = tmp_path / "map.pcd"
        target.write_text(
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 2\n"
            "HEIGHT 1\n"
            "POINTS 2\n"
            "DATA ascii\n"
            "0 0 0\n"
            "1 0 0\n",
            encoding="utf-8",
        )
        return Completed()

    monkeypatch.setattr(map_save, "_control_binary", lambda: "slamctl")
    monkeypatch.setattr(subprocess, "run", fake_run)

    adapter = map_save.NativeSlamMapSaveAdapter()
    result = adapter.save_slam_map(tmp_path / "map.pcd")

    assert result["success"] is True
    assert result["source"] == "native_slam_dds_control"
    assert result["point_count"] == 2
