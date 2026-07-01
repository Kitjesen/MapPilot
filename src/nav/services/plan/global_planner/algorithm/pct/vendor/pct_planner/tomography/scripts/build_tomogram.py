"""
Standalone PCD -> tomogram builder (no ROS).
Used by planner_wrapper to build .pickle on-the-fly when only .pcd exists.
"""
import os
import pickle
import numpy as np
import open3d as o3d

# Run from tomography/scripts so tomogram and config can be imported
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in __import__('sys').path:
    __import__('sys').path.insert(0, _script_dir)
if os.path.dirname(_script_dir) not in __import__('sys').path:
    __import__('sys').path.insert(0, os.path.dirname(_script_dir))

from tomogram import Tomogram


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
    t.kernel_size = trav_kw.get('kernel_size', 7)
    # interval_min: min vertical clearance for robot clearance check (m).
    # Lowered 0.50 → 0.30 so sparse-ground cells in older maps (pre-
    # corrected sweep density) still qualify as traversable.
    t.interval_min = trav_kw.get('interval_min', 0.30)
    t.interval_free = trav_kw.get('interval_free', 0.65)
    # 0.60 rad ≈ 34°: quadrupeds routinely handle ramps and low stairs at
    # this slope; prior 0.40 (~23°) was too timid and flagged legal ramps as
    # impassable. If you need to lock the robot to flat corridors, override
    # via robot_config.yaml tomogram.slope_max.
    t.slope_max = trav_kw.get('slope_max', 0.60)
    t.step_max = trav_kw.get('step_max', 0.25)
    # standable_ratio: fraction of points in a 0.2×0.2 cell that must be
    # locally flat. Lowered 0.40 → 0.20 for older maps with lower scan
    # density — otherwise most cells fail and A* can't find a path at all.
    t.standable_ratio = trav_kw.get('standable_ratio', 0.20)
    t.cost_barrier = trav_kw.get('cost_barrier', 50.0)
    t.safe_margin = trav_kw.get('safe_margin', 1.2)
    t.inflation = trav_kw.get('inflation', 0.2)
    class Cfg:
        map = m
        trav = t
    return Cfg()


def _load_tomogram_config():
    """Load tomogram params from robot_config.yaml if available."""
    defaults = dict(
        resolution=0.2, slice_dh=0.5, ground_h=0.0,
        kernel_size=7, interval_min=0.50, interval_free=0.65,
        slope_max=0.60, step_max=0.25, standable_ratio=0.10,
        cost_barrier=50.0, safe_margin=0.4, inflation=0.2,
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
    pcd_path,
    output_pickle_path=None,
    resolution=None,
    slice_dh=None,
    ground_h=None,
    **trav_kwargs
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
    for k in ["kernel_size", "interval_min", "interval_free", "slope_max",
              "step_max", "standable_ratio", "cost_barrier", "safe_margin", "inflation"]:
        if k not in trav_kwargs and k in conf:
            trav_kwargs[k] = conf[k]
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError(f"PCD not found: {pcd_path}")

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points).astype(np.float32)
    if points.shape[1] > 3:
        points = points[:, :3]

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
        'data': tomogram_stack.astype(np.float16),
        'resolution': resolution,
        'center': center,
        'slice_h0': slice_h0,
        'slice_dh': slice_dh,
    }

    if output_pickle_path:
        os.makedirs(os.path.dirname(output_pickle_path) or '.', exist_ok=True)
        with open(output_pickle_path, 'wb') as f:
            pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)

    return data_dict
