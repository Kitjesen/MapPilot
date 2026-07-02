"""Unit tests for src/nav/services/dynamic_filter.py.

These are pure Python tests — they don't need the actual DUFOMap binary or
Fast-LIO2. Subprocess calls are monkey-patched so the tests run offline.
"""
from __future__ import annotations

import struct
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from runtime import dynamic_filter as df

# ── PCD I/O round-trip ────────────────────────────────────────────────────

def test_read_write_pcd_binary_roundtrip(tmp_path: Path):
    pts = np.array(
        [[1.0, 2.0, 3.0], [-4.5, 5.5, 6.5], [0.1, -0.2, 0.3]],
        dtype=np.float32,
    )
    vp = (0.1, 0.2, 0.3, 0.9, 0.1, 0.2, 0.3)

    p = tmp_path / "frame_0.pcd"
    df._write_pcd_binary(p, pts, vp)

    pts_back = df._read_pcd_binary(p)
    assert pts_back.shape == pts.shape
    np.testing.assert_allclose(pts_back, pts, rtol=1e-6)


def test_read_pcd_with_intensity_column(tmp_path: Path):
    """Simulate a Fast-LIO2 patch with x y z intensity (16 B/point).

    _read_pcd_binary should discard the intensity column and return (N, 3).
    """
    pcd = tmp_path / "with_intensity.pcd"
    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n"
    )
    with open(pcd, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(struct.pack("<ffff", 1.0, 2.0, 3.0, 100.0))
        f.write(struct.pack("<ffff", 4.0, 5.0, 6.0, 200.0))

    pts = df._read_pcd_binary(pcd)
    assert pts.shape == (2, 3)
    np.testing.assert_allclose(pts[0], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(pts[1], [4.0, 5.0, 6.0])


def test_read_pcd_rejects_non_binary(tmp_path: Path):
    bad = tmp_path / "ascii.pcd"
    bad.write_text("# .PCD v0.7\nDATA ascii\n1.0 2.0 3.0\n")
    with pytest.raises(RuntimeError, match="not a binary PCD"):
        df._read_pcd_binary(bad)


# ── poses.txt parser ──────────────────────────────────────────────────────

def test_parse_poses_normal(tmp_path: Path):
    p = tmp_path / "poses.txt"
    p.write_text(
        "0.pcd 1.0 2.0 3.0 0.5 0.5 0.5 0.5\n"
        "1.pcd 1.1 2.1 3.1 1.0 0.0 0.0 0.0\n"
    )
    out = df._parse_poses(p)
    assert set(out.keys()) == {"0.pcd", "1.pcd"}
    assert out["0.pcd"] == (1.0, 2.0, 3.0, 0.5, 0.5, 0.5, 0.5)
    assert out["1.pcd"][3] == 1.0  # qw


def test_parse_poses_skips_malformed(tmp_path: Path):
    p = tmp_path / "poses.txt"
    p.write_text(
        "0.pcd 1.0 2.0 3.0 0.5 0.5 0.5 0.5\n"
        "\n"  # blank
        "1.pcd 1.1 2.1\n"  # too few fields
        "2.pcd 1.2 2.2 3.2 0.1 0.2 0.3 0.4\n"
    )
    out = df._parse_poses(p)
    assert set(out.keys()) == {"0.pcd", "2.pcd"}  # malformed line dropped


# ── count helper ──────────────────────────────────────────────────────────

def test_count_pcd_points(tmp_path: Path):
    pcd = tmp_path / "a.pcd"
    pts = np.zeros((42, 3), dtype=np.float32)
    df._write_pcd_binary(pcd, pts, (0, 0, 0, 1, 0, 0, 0))
    assert df._count_pcd_points(pcd) == 42


# ── refilter_map degraded paths (never raise) ────────────────────────────

def test_refilter_map_missing_binary(tmp_path: Path, monkeypatch):
    """Binary absent → success=False, no exception."""
    monkeypatch.setattr(df, "DUFOMAP_BIN", "/nonexistent/dufomap_run")
    result = df.refilter_map(tmp_path)
    assert result["success"] is False
    assert "dufomap binary missing" in result["error"]
    assert "elapsed_s" in result


def test_refilter_map_incomplete_dir(tmp_path: Path, monkeypatch):
    """Valid binary but no patches/poses/map.pcd → success=False."""
    # Create a fake executable
    fake_bin = tmp_path / "fake_dufomap"
    fake_bin.touch()
    fake_bin.chmod(0o755)
    monkeypatch.setattr(df, "DUFOMAP_BIN", str(fake_bin))
    monkeypatch.setenv("LINGTU_DUFOMAP_MIN_PATCHES", "1")

    # Empty map dir (no patches/, no poses.txt, no map.pcd)
    empty = tmp_path / "empty_map"
    empty.mkdir()

    result = df.refilter_map(empty)
    assert result["success"] is False
    assert "PGO output incomplete" in result["error"]


def test_refilter_map_subprocess_failure(tmp_path: Path, monkeypatch):
    """Subprocess exits nonzero → success=False, original map.pcd intact."""
    # Fake binary
    fake_bin = tmp_path / "fake_dufomap"
    fake_bin.touch()
    fake_bin.chmod(0o755)
    monkeypatch.setattr(df, "DUFOMAP_BIN", str(fake_bin))
    monkeypatch.setenv("LINGTU_DUFOMAP_MIN_PATCHES", "1")

    # Build a minimal valid map dir
    map_dir = tmp_path / "m"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    df._write_pcd_binary(patches / "0.pcd", pts, (0, 0, 0, 1, 0, 0, 0))
    (map_dir / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n")
    df._write_pcd_binary(map_dir / "map.pcd", pts, (0, 0, 0, 1, 0, 0, 0))

    # Mock subprocess.run to fail
    mock_run = MagicMock(return_value=MagicMock(returncode=1, stderr="boom"))
    monkeypatch.setattr(df.subprocess, "run", mock_run)

    result = df.refilter_map(map_dir)
    assert result["success"] is False
    assert "dufomap exit 1" in result["error"]
    # Original untouched — no backup created either on failure
    assert (map_dir / "map.pcd").is_file()


def test_refilter_map_skips_when_patch_count_below_minimum(tmp_path: Path, monkeypatch):
    fake_bin = tmp_path / "fake_dufomap"
    fake_bin.touch()
    fake_bin.chmod(0o755)
    monkeypatch.setattr(df, "DUFOMAP_BIN", str(fake_bin))
    monkeypatch.setenv("LINGTU_DUFOMAP_MIN_PATCHES", "3")

    map_dir = tmp_path / "m"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    df._write_pcd_binary(patches / "0.pcd", pts, (0, 0, 0, 1, 0, 0, 0))
    (map_dir / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n")
    df._write_pcd_binary(map_dir / "map.pcd", pts, (0, 0, 0, 1, 0, 0, 0))

    mock_run = MagicMock()
    monkeypatch.setattr(df.subprocess, "run", mock_run)

    result = df.refilter_map(map_dir)

    assert result["success"] is False
    assert "insufficient patches" in result["error"]
    assert result["patch_count"] == 1
    mock_run.assert_not_called()


def test_refilter_map_success_creates_backup(tmp_path: Path, monkeypatch):
    """Happy path: backs up original, overwrites map.pcd with cleaned output."""
    # Fake binary
    fake_bin = tmp_path / "fake_dufomap"
    fake_bin.touch()
    fake_bin.chmod(0o755)
    monkeypatch.setattr(df, "DUFOMAP_BIN", str(fake_bin))
    monkeypatch.setenv("LINGTU_DUFOMAP_MIN_PATCHES", "1")

    # Minimal map dir
    map_dir = tmp_path / "m"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    orig_pts = np.random.rand(100, 3).astype(np.float32) * 10
    df._write_pcd_binary(
        patches / "scan_000123.pcd",
        orig_pts,
        (0, 0, 0, 1, 0, 0, 0),
    )
    (map_dir / "poses.txt").write_text("scan_000123.pcd 0 0 0 1 0 0 0\n")
    df._write_pcd_binary(map_dir / "map.pcd", orig_pts, (0, 0, 0, 1, 0, 0, 0))
    orig_md5 = (map_dir / "map.pcd").read_bytes()

    def fake_run(cmd, **kwargs):
        # cmd is [bin, tmpdir, config]. Simulate DUFOMap writing output.
        tmpdir = Path(cmd[1])
        assert (tmpdir / "pcd" / "000000.pcd").is_file()
        assert not (tmpdir / "pcd" / "scan_000123.pcd").exists()
        clean_pts = orig_pts[:80]  # drop 20 points
        df._write_pcd_binary(tmpdir / "dufomap_output.pcd", clean_pts,
                             (0, 0, 0, 1, 0, 0, 0))
        return MagicMock(returncode=0, stderr="", stdout="")

    monkeypatch.setattr(df.subprocess, "run", fake_run)

    result = df.refilter_map(map_dir)

    assert result["success"] is True
    assert result["orig_count"] == 100
    assert result["clean_count"] == 80
    assert result["dropped"] == 20
    assert result["elapsed_s"] >= 0

    # Backup created with pre-filter content
    backup = map_dir / "map.pcd.predufo"
    assert backup.is_file()
    assert backup.read_bytes() == orig_md5

    # map.pcd has been replaced with cleaned version
    assert df._count_pcd_points(map_dir / "map.pcd") == 80


def test_refilter_map_subprocess_timeout(tmp_path: Path, monkeypatch):
    """TimeoutExpired → success=False, original intact."""
    fake_bin = tmp_path / "fake_dufomap"
    fake_bin.touch()
    fake_bin.chmod(0o755)
    monkeypatch.setattr(df, "DUFOMAP_BIN", str(fake_bin))
    monkeypatch.setenv("LINGTU_DUFOMAP_MIN_PATCHES", "1")

    map_dir = tmp_path / "m"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    df._write_pcd_binary(patches / "0.pcd", pts, (0, 0, 0, 1, 0, 0, 0))
    (map_dir / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n")
    df._write_pcd_binary(map_dir / "map.pcd", pts, (0, 0, 0, 1, 0, 0, 0))

    import subprocess as sp
    def raise_timeout(*a, **k):
        raise sp.TimeoutExpired(cmd=a[0], timeout=10)
    monkeypatch.setattr(df.subprocess, "run", raise_timeout)

    result = df.refilter_map(map_dir, timeout_s=10)
    assert result["success"] is False
    assert "timeout" in result["error"]
