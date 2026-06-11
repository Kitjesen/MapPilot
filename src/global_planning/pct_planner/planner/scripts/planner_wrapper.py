"""Python wrapper for the C++ PCT global planner backend."""

import os
import sys
import pickle
import math
import errno
import numpy as np

# utils not used in this file  transTrajGrid2Map and traj2ros available if needed later

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'
# For on-the-fly PCD -> tomogram (optional)
_tomography_scripts = os.path.join(rsg_root, 'tomography', 'scripts')
if os.path.isdir(_tomography_scripts) and _tomography_scripts not in sys.path:
    sys.path.insert(0, _tomography_scripts)
if os.path.join(rsg_root, 'tomography') not in sys.path:
    sys.path.insert(0, os.path.join(rsg_root, 'tomography'))


_FALSE_ENV_VALUES = {"0", "false", "no", "off"}


def _safe_print(*args, **kwargs):
    """Best-effort diagnostics that must not fail native planning."""
    try:
        print(*args, **kwargs)
    except BrokenPipeError:
        return
    except OSError as exc:
        if getattr(exc, "errno", None) == errno.EPIPE:
            return
        raise


def _env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    text = str(value).strip().lower()
    if text == "":
        return bool(default)
    return text not in _FALSE_ENV_VALUES


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg
        _planner = getattr(cfg, "planner", None)
        _wrapper = getattr(cfg, "wrapper", None)
        self.use_quintic = getattr(_planner, "use_quintic", True)
        self.optimize_trajectory = _env_bool(
            "LINGTU_PCT_OPTIMIZE_TRAJECTORY",
            getattr(_planner, "optimize_trajectory", True),
        )
        self.max_heading_rate = getattr(_planner, "max_heading_rate", 10)
        self.obstacle_thr = getattr(_planner, "obstacle_thr", 50)
        tomo_dir = getattr(_wrapper, "tomo_dir", "/rsc/tomogram/")
        self.tomo_dir = rsg_root + tomo_dir
        self.pcd_dir = getattr(_wrapper, "pcd_dir", None)

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.float32)
        self.end_idx = np.zeros(3, dtype=np.float32)
        self.last_path_mode = ""
        self.last_optimizer_enabled = bool(self.optimize_trajectory)
        self.last_optimizer_attempted = False
        self.last_optimizer_accepted = None
        self.last_optimizer_reject_reason = ""
        self.last_optimizer_blocked_sample_count = 0
        self.last_raw_path_blocked_sample_count = 0

    def _reset_last_plan_metadata(self):
        self.last_path_mode = ""
        self.last_optimizer_enabled = bool(self.optimize_trajectory)
        self.last_optimizer_attempted = False
        self.last_optimizer_accepted = None
        self.last_optimizer_reject_reason = ""
        self.last_optimizer_blocked_sample_count = 0
        self.last_raw_path_blocked_sample_count = 0

    def _initFromDataDict(self, data_dict):
        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        self.resolution = float(data_dict['resolution'])
        self.center = np.asarray(data_dict['center'], dtype=np.double)
        self.n_slice = tomogram.shape[1]
        self.slice_h0 = float(data_dict['slice_h0'])
        self.slice_dh = float(data_dict['slice_dh'])
        # C++ ele_planner expects map_dim as [x_dim=cols=W, y_dim=rows=H].
        # tomogram shape = (channels, n_slice, H=rows, W=cols)
        self.map_dim = [tomogram.shape[3], tomogram.shape[2]]
        self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)
        # Keep raw layers for ROS publishing (tomogram / traversability visualization)
        self.layers_g = tomogram[3].copy()
        self.layers_t = tomogram[0].copy()
        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3].copy()
        # Fill NaN with per-slice nominal height (not -100) to prevent GPMP
        # from producing extreme Z values at smoothed points near NaN cells
        for s in range(self.n_slice):
            mask = np.isnan(elev_g[s])
            elev_g[s][mask] = self.slice_h0 + s * self.slice_dh
        elev_c = tomogram[4].copy()
        for s in range(self.n_slice):
            mask = np.isnan(elev_c[s])
            elev_c[s][mask] = self.slice_h0 + s * self.slice_dh + 3.0
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)

    def loadTomogram(self, tomo_file, resolution=None, slice_dh=None, ground_h=None):
        """Load a tomogram pickle or build one from a matching PCD."""
        if os.path.isabs(tomo_file):
            basename = os.path.basename(tomo_file)
            search_dir = os.path.dirname(tomo_file)
        else:
            basename = tomo_file
            search_dir = self.tomo_dir
        # Strip only known map suffixes; keep names like spiral0.3_2 intact.
        for ext in ('.pickle', '.pcd'):
            if basename.endswith(ext):
                basename = basename[:-len(ext)]
                break
        base = basename
        pcd_dir = self.pcd_dir if self.pcd_dir is not None else self.tomo_dir
        pcd_search_dir = os.path.dirname(tomo_file) if os.path.isabs(tomo_file) else pcd_dir
        pickle_path = os.path.join(search_dir, base + '.pickle')
        pcd_path = os.path.join(pcd_search_dir, base + '.pcd')

        if os.path.isfile(pickle_path):
            with open(pickle_path, 'rb') as handle:
                data_dict = pickle.load(handle)
            _safe_print("tomogram.shape:", np.asarray(data_dict['data']).shape)
            self._initFromDataDict(data_dict)
            _safe_print(f"Tomogram loaded from {pickle_path}. Map: {self.map_dim}, Slices: {self.n_slice}")
        elif os.path.isfile(pcd_path):
            _safe_print(f"[PCT] No .pickle found; building tomogram from PCD: {pcd_path}")
            try:
                from build_tomogram import build_tomogram_from_pcd  # type: ignore[reportMissingImports]
            except ImportError:
                raise ImportError("PCD found but build_tomogram not available. Add pct_planner/tomography/scripts to PYTHONPATH or install tomography deps.")
            _w = getattr(self.cfg, "wrapper", None)
            res = resolution if resolution is not None else getattr(_w, "tomogram_resolution", 0.2)
            dh = slice_dh if slice_dh is not None else getattr(_w, "tomogram_slice_dh", 0.5)
            gh = ground_h if ground_h is not None else getattr(_w, "tomogram_ground_h", 0.0)
            data_dict = build_tomogram_from_pcd(
                pcd_path,
                output_pickle_path=pickle_path,
                resolution=res,
                slice_dh=dh,
                ground_h=gh,
            )
            _safe_print("tomogram.shape:", np.asarray(data_dict['data']).shape)
            self._initFromDataDict(data_dict)
            _safe_print(f"[PCT] Tomogram built from PCD and cached to {pickle_path}. Map: {self.map_dim}, Slices: {self.n_slice}")
        else:
            raise FileNotFoundError(
                f"Neither tomogram nor PCD found for '{tomo_file}'. "
                f"Looked for: {pickle_path} and {pcd_path}"
            )
        import gc
        gc.collect()
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -15.0
        mask_g = (diff_g < 0.5) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)
        _safe_print("np.sum(gateway_up)", np.sum(gateway_up))
        _safe_print("np.sum(mask_t)", np.sum(mask_t))
        _safe_print("np.sum(mask_g)", np.sum(mask_g))

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 15.0
        mask_g = (diff_g < 0.5) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        # ele_planner.so was compiled against numpy 1.x C API.
        # numpy 2.x changed array struct layout (ABI break), so we must
        # force C-contiguous + correct dtype BEFORE passing to C++.
        def _compat(arr, dtype=np.double):
            return np.ascontiguousarray(arr.reshape(-1, arr.shape[-1]), dtype=dtype)

        self.planner.init_map(
            self.obstacle_thr, 30, self.resolution, self.n_slice, 0.5,
            _compat(trav),
            _compat(elev_g),
            _compat(elev_c),
            _compat(gateway, dtype=np.int32),
            _compat(trav_gy),
            _compat(-trav_gx),
        )

    def plan(self, start_pos, end_pos, start_height=0, end_height=0):
        """Plan from start to goal in world XY plus optional start/end heights."""
        self._reset_last_plan_metadata()
        # Reject invalid numeric inputs before calling the native C++ planner.
        if not (np.all(np.isfinite(start_pos)) and np.all(np.isfinite(end_pos))):
            _safe_print(f"[planner_wrapper] rejecting NaN/Inf input: start={start_pos} end={end_pos}",
                        flush=True)
            return None
        if not (np.isfinite(start_height) and np.isfinite(end_height)):
            _safe_print(f"[planner_wrapper] rejecting NaN/Inf height: start_h={start_height} end_h={end_height}",
                        flush=True)
            return None

        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)
        
        # Use explicit heights supplied by the adapter. Raw 2D goals often have
        # z=0, while surface snapping can select a different traversable slice.
        
        self.start_idx[0] = self.pos2slice(start_height)
        self.end_idx[0] = self.pos2slice(end_height)
        
        # Debug information
        _safe_print(f"[planner_wrapper] plan: start_idx={self.start_idx}, end_idx={self.end_idx}, "
                    f"obstacle_thr={self.obstacle_thr}, optimize_trajectory={self.optimize_trajectory}, "
                    f"map_dim={self.map_dim}, center={self.center}",
                    flush=True)

        self.planner.plan(self.start_idx, self.end_idx, bool(self.optimize_trajectory))
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        
        if len(path) == 0:
            return None

        if not self.optimize_trajectory:
            raw_world = self._raw_path_to_world(path)
            if raw_world is None or len(raw_world) == 0:
                return None
            self.last_path_mode = "native_astar_raw_path"
            _safe_print(
                f"[planner_wrapper] optimizer disabled; returning native A* raw path "
                f"with {len(raw_world)} points",
                flush=True,
            )
            return raw_world

        self.last_optimizer_attempted = True
        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = np.asarray(optimizer.get_result_matrix(), dtype=np.float64)
        layers = np.asarray(optimizer.get_layers(), dtype=np.float64).reshape(-1)
        heights = np.asarray(optimizer.get_heights(), dtype=np.float64).reshape(-1)

        if (
            traj_raw.size == 0
            or traj_raw.ndim != 2
            or layers.shape[0] != traj_raw.shape[0]
            or heights.shape[0] != traj_raw.shape[0]
        ):
            raw_world = self._raw_path_to_world(path)
            if raw_world is not None and len(raw_world) > 0:
                self.last_path_mode = "native_astar_raw_path"
                self.last_optimizer_accepted = False
                self.last_optimizer_reject_reason = "optimizer_empty_or_invalid"
                _safe_print(
                    "[planner_wrapper] optimizer returned empty trajectory; "
                    "returning native A* raw path",
                    flush=True,
                )
                return raw_world
            self.last_optimizer_accepted = False
            self.last_optimizer_reject_reason = "optimizer_empty_or_invalid"
            return None

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_grid_xy = np.stack([traj[:, 0], traj[:, y_idx]], axis=1)
        _safe_print(f"[planner_wrapper] traj_grid_xy first={traj_grid_xy[0]}, last={traj_grid_xy[-1]}, "
                    f"y_idx={y_idx}, traj_shape={traj.shape}", flush=True)
        traj_3d = self._optimized_traj_to_world(traj_grid_xy, heights)
        _safe_print(f"[planner_wrapper] traj_world first={traj_3d[0]}, last={traj_3d[-1]}", flush=True)

        blocked = self._hard_obstacle_sample_count(traj_3d)
        if blocked:
            self.last_optimizer_blocked_sample_count = int(blocked)
            raw_world = self._raw_path_to_world(path)
            raw_blocked = self._hard_obstacle_sample_count(raw_world)
            self.last_raw_path_blocked_sample_count = int(raw_blocked)
            if raw_world is not None and raw_blocked == 0:
                self.last_path_mode = "native_astar_raw_path"
                self.last_optimizer_accepted = False
                self.last_optimizer_reject_reason = "optimized_trajectory_hard_obstacle"
                _safe_print(
                    f"[planner_wrapper] optimized trajectory crosses {blocked} hard-obstacle samples; "
                    "returning raw A* path",
                    flush=True,
                )
                return raw_world
            _safe_print(
                f"[planner_wrapper] optimized trajectory crosses {blocked} hard-obstacle samples; "
                f"raw path blocked={raw_blocked}",
                flush=True,
            )
            self.last_path_mode = "optimized_trajectory"
            self.last_optimizer_accepted = False
            self.last_optimizer_reject_reason = "optimized_trajectory_hard_obstacle"
            return traj_3d

        self.last_path_mode = "optimized_trajectory"
        self.last_optimizer_accepted = True
        return traj_3d

    def _optimized_traj_to_world(self, xy_grid, heights):
        """Convert optimizer XY grid coordinates while preserving true Z heights."""
        xy_grid = np.asarray(xy_grid, dtype=np.float64)
        heights = np.asarray(heights, dtype=np.float64).reshape(-1)
        if xy_grid.ndim != 2 or xy_grid.shape[1] < 2 or xy_grid.shape[0] != heights.shape[0]:
            raise ValueError(
                f"invalid optimized trajectory shapes: xy={xy_grid.shape}, heights={heights.shape}"
            )
        world = np.empty((xy_grid.shape[0], 3), dtype=np.float64)
        world[:, 0] = (xy_grid[:, 0] - self.map_dim[0] // 2) * self.resolution + self.center[0]
        world[:, 1] = (xy_grid[:, 1] - self.map_dim[1] // 2) * self.resolution + self.center[1]
        world[:, 2] = heights
        return world

    def _raw_path_to_world(self, path):
        arr = np.asarray(path, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 3 or self.map_dim is None or self.center is None:
            return None

        # Native A* path is [slice, y_idx, x_idx]. Convert directly instead of
        # using transTrajGrid2Map, whose third column expects height/resolution.
        layers = np.clip(np.rint(arr[:, 0]).astype(np.int32), 0, max(self.n_slice - 1, 0))
        rows = np.clip(np.rint(arr[:, 1]).astype(np.int32), 0, self.map_dim[1] - 1)
        cols = np.clip(np.rint(arr[:, 2]).astype(np.int32), 0, self.map_dim[0] - 1)

        xy = np.empty((arr.shape[0], 2), dtype=np.float64)
        xy[:, 0] = (cols - self.map_dim[0] // 2) * self.resolution + self.center[0]
        xy[:, 1] = (rows - self.map_dim[1] // 2) * self.resolution + self.center[1]

        heights = np.empty(arr.shape[0], dtype=np.float64)
        for idx, (layer, row, col) in enumerate(zip(layers, rows, cols)):
            height = float("nan")
            if self.layers_g is not None:
                try:
                    height = float(self.layers_g[layer, row, col])
                except Exception:
                    height = float("nan")
            if not np.isfinite(height):
                height = float(self.slice_h0 + layer * self.slice_dh)
            heights[idx] = height

        return np.column_stack((xy, heights))

    def _hard_obstacle_sample_count(self, path_world):
        if path_world is None or self.layers_t is None:
            return 0
        pts = np.asarray(path_world, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 2:
            return 0

        step = max(float(self.resolution) * 0.5, 0.05)
        blocked = 0
        for start, end in zip(pts[:-1], pts[1:]):
            dist = math.hypot(float(end[0] - start[0]), float(end[1] - start[1]))
            steps = max(1, int(math.ceil(dist / step)))
            for sample_idx in range(1, steps + 1):
                alpha = sample_idx / steps
                sample = start + (end - start) * alpha
                if self._is_hard_obstacle_sample(sample):
                    blocked += 1
        if pts.shape[0] == 1 and self._is_hard_obstacle_sample(pts[0]):
            blocked += 1
        return blocked

    def _is_hard_obstacle_sample(self, sample):
        try:
            idx = self.pos2idx(np.asarray(sample[:2], dtype=np.float64))
            col = int(np.rint(idx[0]))
            row = int(np.rint(idx[1]))
            layer = int(np.rint(self.pos2slice(float(sample[2])))) if len(sample) > 2 else 0
        except Exception:
            return True

        if (
            layer < 0
            or row < 0
            or col < 0
            or layer >= self.layers_t.shape[0]
            or row >= self.layers_t.shape[1]
            or col >= self.layers_t.shape[2]
        ):
            return True
        # The PCT wrapper converts world/grid coordinates around the legacy
        # center/offset convention, while LingTu safety reports use tomogram
        # origin indexing. Check the immediate neighborhood so half-cell
        # origin differences cannot let an optimized curve skim through a
        # hard obstacle undetected.
        for sample_row in range(max(0, row - 1), min(self.layers_t.shape[1] - 1, row + 1) + 1):
            for sample_col in range(max(0, col - 1), min(self.layers_t.shape[2] - 1, col + 1) + 1):
                cost = float(self.layers_t[layer, sample_row, sample_col])
                if (not np.isfinite(cost)) or cost >= float(self.obstacle_thr):
                    return True
        return False
    
    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        
        # Safety clamp for XY
        if self.map_dim:
            idx[0] = np.clip(idx[0], 0, self.map_dim[0] - 1)
            idx[1] = np.clip(idx[1], 0, self.map_dim[1] - 1)

        # C++ ele_planner expects start_idx[1] as x and start_idx[2] as y,
        # while the tomogram array is indexed as trav[slice, y, x].
        return idx.astype(np.float32)
    
    def pos2slice(self, z):
        """Convert a world height to the nearest tomogram slice index."""
        if self.slice_dh is None or self.slice_dh == 0:
            return 0.0
            
        slice_offset = (z - self.slice_h0) / self.slice_dh
        
        slice_idx = np.round(slice_offset)
        
        if self.n_slice is not None:
             slice_idx = max(0.0, min(float(slice_idx), float(self.n_slice - 1)))
        
        return float(slice_idx)

    def get_surface_height(self, pos: np.ndarray) -> float:
        """Return the first valid ground height at a world XY position."""
        if self.layers_g is None or self.resolution is None or self.center is None:
            return float(self.slice_h0) if self.slice_h0 is not None else 0.0
        # pos2idx returns [x_cpp, y_cpp]; layers_g is [slice, row=y, col=x].
        idx = self.pos2idx(pos)
        ai = int(np.clip(idx[1], 0, self.layers_g.shape[1] - 1))  # row = y_cpp = idx[1]
        bi = int(np.clip(idx[0], 0, self.layers_g.shape[2] - 1))  # col = x_cpp = idx[0]
        heights = self.layers_g[:, ai, bi]   # (n_slices,)
        valid_mask = ~np.isnan(heights)
        if not valid_mask.any():
            return float(self.slice_h0) if self.slice_h0 is not None else 0.0
        return float(heights[valid_mask][0])

    def height2idx(self, height):
        return self.pos2slice(height)
