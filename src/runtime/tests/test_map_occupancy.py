"""End-to-end tests for MapService occupancy grid building.

Verifies:
1. Raycasting path produces three-valued grid (unknown/free/occupied).
2. Projection fallback produces two-valued grid (when no PGO output).
3. PGM + YAML outputs match ROS2 map_server conventions.
4. Log-odds thresholds correctly classify repeated observations as free.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from nav.services.maps import MapService
from nav.services.map.pipeline import MapPipelineService

# 閳光偓閳光偓 PCD helpers 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

def _write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    """Write an ASCII PCD with xyz float32 fields."""
    n = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    with open(path, "w") as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def _box_walls(center: tuple[float, float], half: float,
               z_min: float, z_max: float,
               step: float = 0.05) -> np.ndarray:
    """Generate 4-wall box of (2璺痟alf 鑴?2璺痟alf) centred at `center`."""
    cx, cy = center
    pts: list[tuple[float, float, float]] = []
    # x-axis walls (y = cy 鍗?half)
    for x in np.arange(cx - half, cx + half + step, step):
        for z in np.arange(z_min, z_max + step, 0.10):
            pts.append((x, cy - half, z))
            pts.append((x, cy + half, z))
    # y-axis walls (x = cx 鍗?half)
    for y in np.arange(cy - half, cy + half + step, step):
        for z in np.arange(z_min, z_max + step, 0.10):
            pts.append((cx - half, y, z))
            pts.append((cx + half, y, z))
    # Ground points (so percentile 5 finds ground level)
    for x in np.arange(cx - half, cx + half + step, step * 2):
        for y in np.arange(cy - half, cy + half + step, step * 2):
            pts.append((x, y, z_min - 0.05))
    return np.asarray(pts, dtype=np.float32)


def _transform_to_body(world_pts: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """Simulate body-frame patch: subtract sensor origin (identity rotation)."""
    return (world_pts - origin).astype(np.float32)


# 閳光偓閳光偓 Fixtures 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

@pytest.fixture
def mgr(tmp_path):
    """Fresh MapService rooted at a temp dir."""
    m = MapService(map_dir=str(tmp_path))
    yield m


@pytest.fixture
def box_room_with_pgo(tmp_path):
    """Build a fake PGO output: 3鑴? m box room with one sensor pose at centre.

    Returns the map directory name (under tmp_path).
    """
    map_name = "box_room"
    map_dir = tmp_path / map_name
    map_dir.mkdir()
    patches_dir = map_dir / "patches"
    patches_dir.mkdir()

    center = (0.0, 0.0)
    half = 1.5  # 3 m 鑴?3 m box
    walls = _box_walls(center, half, z_min=0.0, z_max=1.0)

    # Single keyframe at room centre, identity rotation
    origin = np.array([0.0, 0.0, 0.5], dtype=np.float32)
    body_pts = _transform_to_body(walls, origin)
    _write_ascii_pcd(patches_dir / "0.pcd", body_pts)
    _write_ascii_pcd(map_dir / "map.pcd", walls)

    poses_path = map_dir / "poses.txt"
    poses_path.write_text(
        f"0.pcd {origin[0]} {origin[1]} {origin[2]} 1.0 0.0 0.0 0.0\n"
    )
    return map_name


@pytest.fixture
def box_room_no_pgo(tmp_path):
    """Same room but only map.pcd, no poses.txt / patches/ 閳?projection fallback."""
    map_name = "box_room_plain"
    map_dir = tmp_path / map_name
    map_dir.mkdir()
    walls = _box_walls((0.0, 0.0), 1.5, z_min=0.0, z_max=1.0)
    _write_ascii_pcd(map_dir / "map.pcd", walls)
    return map_name


# 閳光偓閳光偓 Raycasting path tests 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

def test_raycasting_produces_three_values(mgr, box_room_with_pgo):
    """log-odds raycasting must produce unknown(-1), free(0), and occupied(100)."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)

    assert result["success"], result
    assert result["mode"] == "raycasting"

    grid_path = Path(result["occupancy"])
    data = np.load(grid_path)
    grid = data["grid"]

    uniq = set(int(v) for v in np.unique(grid))
    assert uniq.issubset({-1, 0, 100}), f"unexpected values: {uniq}"
    assert 100 in uniq, "must have occupied cells (walls)"
    assert 0 in uniq, "must have free cells (raycast from sensor)"
    # unknown may or may not exist depending on map bounds 閳?border produces some


def test_raycasting_room_center_is_free(mgr, box_room_with_pgo):
    """Sensor origin cell must be classified as free after raycasting."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    data = np.load(result["occupancy"])
    grid = data["grid"]
    resolution = float(data["resolution"])
    origin = data["origin"]

    # World (0,0) is sensor origin; cell index:
    col = int(math.floor((0.0 - origin[0]) / resolution))
    row = int(math.floor((0.0 - origin[1]) / resolution))
    assert grid[row, col] == 0, (
        f"sensor origin cell ({row},{col}) should be free, got {grid[row, col]}"
    )


def test_raycasting_walls_are_occupied(mgr, box_room_with_pgo):
    """At least one cell on each wall edge must be occupied."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    data = np.load(result["occupancy"])
    grid = data["grid"]
    resolution = float(data["resolution"])
    origin = data["origin"]

    # Check a point on the +x wall (x=1.5, y=0.0)
    col = int(math.floor((1.5 - origin[0]) / resolution))
    row = int(math.floor((0.0 - origin[1]) / resolution))
    # Wall is 1 cell thick, so check 鍗? neighbourhood
    patch = grid[max(0, row - 1):row + 2, max(0, col - 1):col + 2]
    assert (patch == 100).any(), f"+x wall should have occupied cell; got patch=\n{patch}"


def test_raycasting_outside_room_is_unknown(mgr, box_room_with_pgo):
    """Cells far outside the raycast frustum should stay unknown (-1)."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    data = np.load(result["occupancy"])
    grid = data["grid"]
    # The border (1m) is beyond any ray hit 閳?corners of the grid should be unknown
    corners = [grid[0, 0], grid[0, -1], grid[-1, 0], grid[-1, -1]]
    assert any(c == -1 for c in corners), (
        f"expected unknown in at least one grid corner; got {corners}"
    )


# 閳光偓閳光偓 PGM + YAML output tests 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

def test_pgm_output_ros2_conventions(mgr, box_room_with_pgo):
    """PGM file uses P5 header and 254/0/205 encoding."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    pgm_path = Path(result["pgm"])
    assert pgm_path.exists()

    with open(pgm_path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P5", f"expected P5 header, got {magic!r}"
        dims = f.readline().strip().split()
        assert len(dims) == 2
        w, h = int(dims[0]), int(dims[1])
        maxval = int(f.readline().strip())
        assert maxval == 255
        data = f.read()
        assert len(data) == w * h
        pgm = np.frombuffer(data, dtype=np.uint8).reshape(h, w)

    uniq = set(int(v) for v in np.unique(pgm))
    assert uniq.issubset({0, 205, 254}), (
        f"PGM must use nav2 trinary encoding; got values {uniq}"
    )


def test_yaml_output_ros2_metadata(mgr, box_room_with_pgo):
    """YAML metadata follows ROS2 map_server format."""
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    yaml_path = Path(result["yaml"])
    assert yaml_path.exists()

    text = yaml_path.read_text()
    assert "image: map.pgm" in text
    assert "resolution:" in text
    assert "origin:" in text
    assert "occupied_thresh: 0.65" in text
    assert "free_thresh: 0.196" in text
    assert "mode: trinary" in text
    assert "negate: 0" in text


def test_pgm_row_order_flipped(mgr, box_room_with_pgo):
    """PGM row 0 must be top (high y) 閳?opposite of the numpy grid orientation.

    We verify by comparing PGM[0,:] to np.flipud(npz_grid)[0,:] transformed
    through the value encoding.
    """
    result = mgr._build_occupancy_snapshot(box_room_with_pgo)
    data = np.load(result["occupancy"])
    grid = data["grid"]

    with open(result["pgm"], "rb") as f:
        f.readline()  # P5
        f.readline()  # w h
        f.readline()  # 255
        h, w = grid.shape
        pgm = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

    # grid[-1,:] corresponds to PGM top row after flip
    expected = np.full_like(grid[-1, :], 205, dtype=np.uint8)
    expected[grid[-1, :] == 0] = 254
    expected[grid[-1, :] == 100] = 0
    assert np.array_equal(pgm[0, :], expected)


# 閳光偓閳光偓 Projection fallback tests 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

def test_projection_fallback_when_no_pgo(mgr, box_room_no_pgo):
    """Without poses.txt/patches the algorithm falls back to XY projection."""
    result = mgr._build_occupancy_snapshot(box_room_no_pgo)
    assert result["success"], result
    assert result["mode"] == "projection"

    data = np.load(result["occupancy"])
    grid = data["grid"]
    uniq = set(int(v) for v in np.unique(grid))
    # Projection cannot distinguish unknown 閳?only 0 (not observed) and 100
    assert uniq.issubset({0, 100})
    assert 100 in uniq  # walls detected


# 閳光偓閳光偓 Parser tests 閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓閳光偓

def test_parse_poses_txt_valid(tmp_path):
    p = tmp_path / "poses.txt"
    p.write_text(
        "0.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n"
        "1.pcd 1.5 -2.0 0.3 0.707 0.0 0.0 0.707\n"
    )
    poses = MapPipelineService.parse_poses_txt(p)
    assert len(poses) == 2
    assert poses[0]["patch"] == "0.pcd"
    assert np.allclose(poses[1]["t"], [1.5, -2.0, 0.3])
    assert np.allclose(poses[1]["q"], [0.707, 0.0, 0.0, 0.707])


def test_parse_poses_txt_skips_malformed(tmp_path):
    p = tmp_path / "poses.txt"
    p.write_text(
        "0.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n"
        "bad line with only three tokens\n"
        "\n"
        "1.pcd not_a_float 0 0 1 0 0 0\n"
        "2.pcd 2.0 2.0 0.0 1.0 0.0 0.0 0.0\n"
    )
    poses = MapPipelineService.parse_poses_txt(p)
    assert len(poses) == 2
    assert poses[0]["patch"] == "0.pcd"
    assert poses[1]["patch"] == "2.pcd"


def test_parse_poses_txt_skips_unsafe_patch_paths(tmp_path):
    p = tmp_path / "poses.txt"
    p.write_text(
        "../outside.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n"
        "..\\outside.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n"
        "safe.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n"
    )

    poses = MapPipelineService.parse_poses_txt(p)

    assert [pose["patch"] for pose in poses] == ["safe.pcd"]


def test_raycasting_does_not_load_patch_path_traversal(tmp_path, monkeypatch):
    map_dir = tmp_path / "map"
    patches_dir = map_dir / "patches"
    patches_dir.mkdir(parents=True)
    outside = tmp_path / "outside.pcd"
    _write_ascii_pcd(outside, np.array([[1.0, 0.0, 0.5]], dtype=np.float32))
    poses_path = map_dir / "poses.txt"
    poses_path.write_text(
        "../outside.pcd 0.0 0.0 0.0 1.0 0.0 0.0 0.0\n",
        encoding="utf-8",
    )
    loaded_paths: list[Path] = []

    def fake_load(path: str):
        loaded_paths.append(Path(path))
        return np.array([[1.0, 0.0, 0.5]], dtype=np.float32)

    monkeypatch.setattr(MapPipelineService, "load_pcd_points", staticmethod(fake_load))

    with pytest.raises(RuntimeError, match="no valid poses|no usable patches"):
        MapPipelineService.build_occupancy_raycasting(poses_path, patches_dir)

    assert loaded_paths == []


def test_quat_to_rot_identity():
    R = MapPipelineService.quat_to_rot(np.array([1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(R, np.eye(3), atol=1e-6)


def test_quat_to_rot_90_deg_yaw():
    # 90鎺?yaw rotation: qw=cos(锜?4), qz=sin(锜?4)
    c = math.cos(math.pi / 4)
    s = math.sin(math.pi / 4)
    R = MapPipelineService.quat_to_rot(np.array([c, 0.0, 0.0, s]))
    # +x 閳?+y
    x_rotated = R @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(x_rotated, [0.0, 1.0, 0.0], atol=1e-6)
