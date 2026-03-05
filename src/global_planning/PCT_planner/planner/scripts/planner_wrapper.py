import os
import sys
import pickle
import numpy as np

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'
# For on-the-fly PCD -> tomogram (optional)
_tomography_scripts = os.path.join(rsg_root, 'tomography', 'scripts')
if os.path.isdir(_tomography_scripts) and _tomography_scripts not in sys.path:
    sys.path.insert(0, _tomography_scripts)
if os.path.join(rsg_root, 'tomography') not in sys.path:
    sys.path.insert(0, os.path.join(rsg_root, 'tomography'))


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg
        _planner = getattr(cfg, "planner", None)
        _wrapper = getattr(cfg, "wrapper", None)
        self.use_quintic = getattr(_planner, "use_quintic", True)
        self.max_heading_rate = getattr(_planner, "max_heading_rate", 10)
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

    def _initFromDataDict(self, data_dict):
        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        self.resolution = float(data_dict['resolution'])
        self.center = np.asarray(data_dict['center'], dtype=np.double)
        self.n_slice = tomogram.shape[1]
        self.slice_h0 = float(data_dict['slice_h0'])
        self.slice_dh = float(data_dict['slice_dh'])
        self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
        self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)
        # Keep raw layers for ROS publishing (tomogram / traversability visualization)
        self.layers_g = tomogram[3].copy()
        self.layers_t = tomogram[0].copy()
        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3].copy()
        elev_g = np.nan_to_num(elev_g, nan=-100, copy=False)
        elev_c = tomogram[4].copy()
        elev_c = np.nan_to_num(elev_c, nan=1e6, copy=False)
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)

    def loadTomogram(self, tomo_file, resolution=None, slice_dh=None, ground_h=None):
        """加载 tomogram。若传 resolution/slice_dh/ground_h 则从 PCD 建图时优先使用（如来自 ROS 参数）。"""
        if os.path.isabs(tomo_file):
            basename = os.path.basename(tomo_file)
            search_dir = os.path.dirname(tomo_file)
        else:
            basename = tomo_file
            search_dir = self.tomo_dir
        # 只剥离已知扩展名 (.pickle / .pcd)，避免 splitext 误把文件名中的
        # 小数点当扩展名 (如 "spiral0.3_2" 会被 splitext 错误分割为 "spiral0")
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
            print("tomogram.shape:", np.asarray(data_dict['data']).shape)
            self._initFromDataDict(data_dict)
            print(f"Tomogram loaded from {pickle_path}. Map: {self.map_dim}, Slices: {self.n_slice}")
        elif os.path.isfile(pcd_path):
            print(f"[PCT] No .pickle found; building tomogram from PCD: {pcd_path}")
            try:
                from build_tomogram import build_tomogram_from_pcd  # type: ignore[reportMissingImports]
            except ImportError:
                raise ImportError("PCD found but build_tomogram not available. Add PCT_planner/tomography/scripts to PYTHONPATH or install tomography deps.")
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
            print("tomogram.shape:", np.asarray(data_dict['data']).shape)
            self._initFromDataDict(data_dict)
            print(f"[PCT] Tomogram built from PCD and cached to {pickle_path}. Map: {self.map_dim}, Slices: {self.n_slice}")
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
        print("np.sum(gateway_up)", np.sum(gateway_up))
        print("np.sum(mask_t)", np.sum(mask_t))
        print("np.sum(mask_g)", np.sum(mask_g))

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
        self.planner.init_map(
            40, 30, self.resolution, self.n_slice, 0.5,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos, start_height=0, end_height=0):
        """
        规划路径的方法，支持设置起始点和终点的高度。
        """
        # 将起始点和终点的二维位置转换为索引
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)
        
        # 使用 pos2slice 替代旧的 height2idx，与参考项目对齐
        # 注意：这里我们优先使用传入的 height 参数，如果它是 0 且 pos Z 信息缺失才用默认逻辑
        # 在 global_planner.py 中，我们传入的是 start_h 和 end_h (来自 get_robot_pose 和 goal_pose.z)
        
        self.start_idx[0] = self.pos2slice(start_height)
        self.end_idx[0] = self.pos2slice(end_height)
        
        # 调试信息
        # print(f"Plan request: StartIdx={self.start_idx}, EndIdx={self.end_idx}")

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d
    
    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        
        # Safety clamp for XY
        if self.map_dim:
            idx[0] = np.clip(idx[0], 0, self.map_dim[0] - 1)
            idx[1] = np.clip(idx[1], 0, self.map_dim[1] - 1)

        # NOTE: 不做交换 — C++ ele_planner 把 start_idx[1] 解释为 x (dim0), start_idx[2] 为 y (dim1),
        #       内部访问 trav[slice, y, x]. 返回 [idx[0], idx[1]] = [x_idx, y_idx] 与 C++ 期望一致.
        return idx.astype(np.float32)
    
    def pos2slice(self, z):
        """将z坐标转换为切片索引 (Copied from reference project)"""
        if self.slice_dh is None or self.slice_dh == 0:
            return 0.0
            
        # 计算相对于起始高度的切片数
        slice_offset = (z - self.slice_h0) / self.slice_dh
        
        # 转换为整数索引并确保在有效范围内
        slice_idx = np.round(slice_offset)
        
        if self.n_slice is not None:
             slice_idx = max(0.0, min(float(slice_idx), float(self.n_slice - 1)))
        
        return float(slice_idx)

    def get_surface_height(self, pos: np.ndarray) -> float:
        """查询 XY 位置的地形表面高度（tomogram 最低有效层）。

        用于将 2D 目标（z=0）自动贴合到 tomogram 实际地形表面，
        解决在有坡度地形上使用 RViz "2D Nav Goal" 时目标高度错误问题。

        Args:
            pos: 世界坐标 [x, y] (np.ndarray, float32)
        Returns:
            地形表面高度 (m)，查询失败时返回 slice_h0 或 0.0
        """
        if self.layers_g is None or self.resolution is None or self.center is None:
            return float(self.slice_h0) if self.slice_h0 is not None else 0.0
        # 使用 pos2idx 得到与规划器一致的格网索引 (已 clamp)
        # pos2idx 返回 [x_cpp, y_cpp]; 数组布局 layers_g[:, dim0=row=y, dim1=col=x]
        idx = self.pos2idx(pos)
        ai = int(np.clip(idx[1], 0, self.layers_g.shape[1] - 1))  # row = y_cpp = idx[1]
        bi = int(np.clip(idx[0], 0, self.layers_g.shape[2] - 1))  # col = x_cpp = idx[0]
        heights = self.layers_g[:, ai, bi]   # (n_slices,)
        valid_mask = ~np.isnan(heights)
        if not valid_mask.any():
            return float(self.slice_h0) if self.slice_h0 is not None else 0.0
        # 最低有效层 = 地形表面
        return float(heights[valid_mask][0])

    # 兼容旧接口，将其指向新函数
    def height2idx(self, height):
        return self.pos2slice(height)
