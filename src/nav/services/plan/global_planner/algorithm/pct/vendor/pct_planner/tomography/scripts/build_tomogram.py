"""
Standalone PCD -> tomogram builder (no ROS).
Used by planner_wrapper to build .pickle on-the-fly when only .pcd exists.
"""

import os
import pickle

import numpy as np

try:
    import open3d as o3d
except ModuleNotFoundError:
    o3d = None
try:
    import numba as _numba

    _NUMBA_AVAILABLE = True
except ModuleNotFoundError:
    _NUMBA_AVAILABLE = False

# Run from tomography/scripts so tomogram and config can be imported
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)
if os.path.dirname(_script_dir) not in __import__("sys").path:
    __import__("sys").path.insert(0, os.path.dirname(_script_dir))

from tomogram import Tomogram


def _pcd_dtype(type_code, size):
    key = (str(type_code).upper(), int(size))
    mapping = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "<i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "<u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    if key not in mapping:
        raise ValueError(f"unsupported PCD field type/size: {type_code}{size}")
    return mapping[key]


def _read_pcd_xyz(path):
    """Read XYZ from a PCD file without Open3D.

    Supports the map-save outputs used by LingTu: ASCII and binary PCD with
    fields containing x/y/z. Extra fields are ignored.
    """
    with open(path, "rb") as fh:
        header_lines = []
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"PCD header missing DATA line: {path}")
            text = line.decode("utf-8", errors="replace").strip()
            header_lines.append(text)
            if text.upper().startswith("DATA "):
                data_type = text.split(None, 1)[1].strip().lower()
                break
        payload = fh.read()

    header = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0].upper()] = parts[1:]

    fields = header.get("FIELDS", [])
    if not all(axis in fields for axis in ("x", "y", "z")):
        raise ValueError(f"PCD missing x/y/z fields: {fields}")
    sizes = [int(value) for value in header.get("SIZE", ["4"] * len(fields))]
    types = header.get("TYPE", ["F"] * len(fields))
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    point_count = int((header.get("POINTS") or header.get("WIDTH") or ["0"])[0])
    if point_count <= 0:
        return np.empty((0, 3), dtype=np.float32)

    xyz_idx = [fields.index(axis) for axis in ("x", "y", "z")]
    if data_type == "ascii":
        text = payload.decode("utf-8", errors="replace")
        rows = []
        for raw in text.splitlines():
            raw = raw.strip()
            if not raw:
                continue
            values = raw.split()
            if len(values) < len(fields):
                continue
            rows.append([float(values[idx]) for idx in xyz_idx])
        return np.asarray(rows, dtype=np.float32)

    if data_type != "binary":
        raise ValueError(f"unsupported PCD DATA type: {data_type}")

    dtype_fields = []
    for field, size, type_code, count in zip(fields, sizes, types, counts):
        dtype_fields.append((field, _pcd_dtype(type_code, size), (count,)))
    cloud = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=point_count)
    points = np.column_stack([np.asarray(cloud[axis]).reshape(point_count, -1)[:, 0] for axis in ("x", "y", "z")])
    return points.astype(np.float32, copy=False)


def _load_xyz_points(pcd_path):
    if o3d is not None:
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points).astype(np.float32)
    else:
        points = _read_pcd_xyz(pcd_path)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"PCD has invalid XYZ shape: {points.shape}")
    if points.shape[1] > 3:
        points = points[:, :3]
    if points.shape[0] == 0:
        raise ValueError(f"PCD has no points: {pcd_path}")
    return points


def _build_vectorized_tomogram(
    points,
    *,
    resolution,
    slice_dh,
    ground_h,
    cost_barrier,
    interval_min,
):
    points_max = np.max(points, axis=0)
    points_min = np.min(points, axis=0)
    if points_min[-1] > ground_h:
        points_min[-1] = ground_h
    map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice = max(1, int(np.ceil((points_max[2] - points_min[2]) / slice_dh)))
    center = (points_max[:2] + points_min[:2]) / 2
    slice_h0 = points_min[-1] + slice_dh

    xi = np.rint((points[:, 0] - center[0]) / resolution).astype(np.int64) + map_dim_x // 2
    yi = np.rint((points[:, 1] - center[1]) / resolution).astype(np.int64) + map_dim_y // 2
    valid = (xi >= 0) & (xi < map_dim_x) & (yi >= 0) & (yi < map_dim_y)
    xi = xi[valid]
    yi = yi[valid]
    pz = points[valid, 2].astype(np.float32, copy=False)
    flat = xi * map_dim_y + yi
    cell_count = map_dim_x * map_dim_y

    layers_g = np.full((n_slice, cell_count), -1e6, dtype=np.float32)
    layers_c = np.full((n_slice, cell_count), 1e6, dtype=np.float32)
    for s_idx in range(n_slice):
        slice_height = np.float32(slice_h0 + s_idx * slice_dh)
        below = pz <= slice_height
        if np.any(below):
            np.maximum.at(layers_g[s_idx], flat[below], pz[below])
        above = ~below
        if np.any(above):
            np.minimum.at(layers_c[s_idx], flat[above], pz[above])

    layers_g = layers_g.reshape(n_slice, map_dim_x, map_dim_y)
    layers_c = layers_c.reshape(n_slice, map_dim_x, map_dim_y)
    interval = layers_c - layers_g
    has_ground = layers_g > -1e5
    layers_t = np.where(
        has_ground & (interval >= float(interval_min)),
        1.0,
        float(cost_barrier),
    ).astype(np.float32)
    trav_grad_x = np.zeros_like(layers_t, dtype=np.float32)
    trav_grad_y = np.zeros_like(layers_t, dtype=np.float32)
    if map_dim_x > 2:
        trav_grad_x[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
    if map_dim_y > 2:
        trav_grad_y[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]
    layers_g = np.where(has_ground, layers_g, np.nan)
    layers_c = np.where(layers_c < 1e5, layers_c, np.nan)
    tomogram_stack = np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c))
    return {
        "data": tomogram_stack.astype(np.float16),
        "resolution": resolution,
        "center": center,
        "slice_h0": slice_h0,
        "slice_dh": slice_dh,
    }


def _make_scene_cfg(resolution=0.2, slice_dh=0.5, ground_h=0.0, **trav_kw):
    """Minimal scene config for Tomogram (no ROS, no scene_*.py)."""

    class MapCfg:
        pass

    class TravCfg:
        pass

    m = MapCfg()
    m.resolution = resolution
    m.slice_dh = slice_dh
    m.ground_h = ground_h
    t = TravCfg()
    t.kernel_size = trav_kw.get("kernel_size", 7)
    # interval_min: min vertical clearance for robot clearance check (m).
    # Lowered 0.50 → 0.30 so sparse-ground cells in older maps (pre-
    # corrected sweep density) still qualify as traversable.
    t.interval_min = trav_kw.get("interval_min", 0.30)
    t.interval_free = trav_kw.get("interval_free", 0.65)
    # 0.60 rad ≈ 34°: quadrupeds routinely handle ramps and low stairs at
    # this slope; prior 0.40 (~23°) was too timid and flagged legal ramps as
    # impassable. If you need to lock the robot to flat corridors, override
    # via robot_config.yaml tomogram.slope_max.
    t.slope_max = trav_kw.get("slope_max", 0.60)
    t.step_max = trav_kw.get("step_max", 0.25)
    # standable_ratio: fraction of points in a 0.2×0.2 cell that must be
    # locally flat. Lowered 0.40 → 0.20 for older maps with lower scan
    # density — otherwise most cells fail and A* can't find a path at all.
    t.standable_ratio = trav_kw.get("standable_ratio", 0.20)
    t.cost_barrier = trav_kw.get("cost_barrier", 50.0)
    t.safe_margin = trav_kw.get("safe_margin", 1.2)
    t.inflation = trav_kw.get("inflation", 0.2)

    class Cfg:
        map = m
        trav = t

    return Cfg()


def _load_tomogram_config():
    """Load tomogram params from robot_config.yaml if available."""
    defaults = dict(
        resolution=0.2,
        slice_dh=0.5,
        ground_h=0.0,
        kernel_size=7,
        interval_min=0.50,
        interval_free=0.65,
        slope_max=0.60,
        step_max=0.25,
        standable_ratio=0.10,
        cost_barrier=50.0,
        safe_margin=0.4,
        inflation=0.2,
    )
    try:
        import sys

        # Find project root (4 levels up from this script)
        root = os.path.abspath(os.path.join(_script_dir, "..", "..", "..", ".."))
        if root not in sys.path:
            sys.path.insert(0, os.path.join(root, "src"))
        from runtime.config import get_config

        cfg = get_config()
        tomo = cfg.raw.get("tomogram", {})
        for k in defaults:
            if k in tomo:
                defaults[k] = tomo[k]
    except Exception:
        pass
    return defaults


def build_tomogram_from_pcd(
    pcd_path, output_pickle_path=None, resolution=None, slice_dh=None, ground_h=None, **trav_kwargs
):
    """
    Build tomogram from a .pcd file (no ROS).

    Parameters are read from robot_config.yaml (tomogram section).
    Explicit arguments override config values.

    Args:
        pcd_path: Full path to .pcd file.
        output_pickle_path: If set, save data_dict to this path.
        resolution: Grid resolution (m). None = read from config.
        slice_dh: Height step between slices (m). None = read from config.
        ground_h: Ground height for bounds (m). None = read from config.
        **trav_kwargs: Override traversability params (kernel_size, etc.).

    Returns:
        data_dict: (data, resolution, center, slice_h0, slice_dh).
    """
    # Load from config, then override with explicit args
    conf = _load_tomogram_config()
    resolution = resolution if resolution is not None else conf["resolution"]
    slice_dh = slice_dh if slice_dh is not None else conf["slice_dh"]
    ground_h = ground_h if ground_h is not None else conf["ground_h"]
    for k in [
        "kernel_size",
        "interval_min",
        "interval_free",
        "slope_max",
        "step_max",
        "standable_ratio",
        "cost_barrier",
        "safe_margin",
        "inflation",
    ]:
        if k not in trav_kwargs and k in conf:
            trav_kwargs[k] = conf[k]
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError(f"PCD not found: {pcd_path}")

    points = _load_xyz_points(pcd_path)
    if not _NUMBA_AVAILABLE:
        data_dict = _build_vectorized_tomogram(
            points,
            resolution=float(resolution),
            slice_dh=float(slice_dh),
            ground_h=float(ground_h),
            cost_barrier=float(trav_kwargs.get("cost_barrier", conf["cost_barrier"])),
            interval_min=float(trav_kwargs.get("interval_min", conf["interval_min"])),
        )
        if output_pickle_path:
            os.makedirs(os.path.dirname(output_pickle_path) or ".", exist_ok=True)
            with open(output_pickle_path, "wb") as f:
                pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)
        return data_dict

    points_max = np.max(points, axis=0)
    points_min = np.min(points, axis=0)
    # Clamp points_min[z] to ground_h *only if the cloud's min z is above
    # ground_h*. Original code overrode unconditionally which made
    # slice_h0 = ground_h + slice_dh = 0.5m in typical configs — then
    # pos2slice(z≈0) rounded to slice -1 → clamped to 0 → hit the "below
    # ground" barrier layer. Keeping the real min z for maps with
    # negative-z points (e.g. robot body at z=0, floor at z=-0.9).
    if points_min[-1] > ground_h:
        points_min[-1] = ground_h
    map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice_init = int(np.ceil((points_max[2] - points_min[2]) / slice_dh))
    center = (points_max[:2] + points_min[:2]) / 2
    slice_h0 = points_min[-1] + slice_dh

    scene_cfg = _make_scene_cfg(resolution=resolution, slice_dh=slice_dh, ground_h=ground_h, **trav_kwargs)
    tomogram = Tomogram(scene_cfg)
    tomogram.initMappingEnv(center, map_dim_x, map_dim_y, n_slice_init, slice_h0)

    layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, _ = tomogram.point2map(points)
    tomogram_stack = np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c))

    data_dict = {
        "data": tomogram_stack.astype(np.float16),
        "resolution": resolution,
        "center": center,
        "slice_h0": slice_h0,
        "slice_dh": slice_dh,
    }

    if output_pickle_path:
        os.makedirs(os.path.dirname(output_pickle_path) or ".", exist_ok=True)
        with open(output_pickle_path, "wb") as f:
            pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)

    return data_dict
