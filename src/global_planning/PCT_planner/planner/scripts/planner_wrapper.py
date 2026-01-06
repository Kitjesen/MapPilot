import os
import sys
import pickle
import numpy as np

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.float32)
        self.end_idx = np.zeros(3, dtype=np.float32)

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            print("tomogram.shape:", tomogram.shape)
            self.n_slice = tomogram.shape[1]
            print("self.n_slice:", self.n_slice)
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        # Extract slices (use views where possible, copy only when modifying)
        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        # Only copy slices that need modification (nan_to_num)
        elev_g = tomogram[3].copy()
        elev_g = np.nan_to_num(elev_g, nan=-100, copy=False)
        elev_c = tomogram[4].copy()
        elev_c = np.nan_to_num(elev_c, nan=1e6, copy=False)

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
        # Explicitly free memory from large objects
        del tomogram
        del data_dict
        import gc
        gc.collect()
        print(f"Tomogram loaded and memory freed. Map: {self.map_dim}, Slices: {self.n_slice}")
        
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

        :param start_pos: 起始点的二维位置 (x, y)
        :param end_pos: 终点的二维位置 (x, y)
        :param start_height: 起始点的高度（切片索引）
        :param end_height: 终点的高度（切片索引）
        """
        
        # 将起始点和终点的二维位置转换为索引
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)
        
        print("self.slice_h0:", self.slice_h0)
        print("self.slice_dh:", self.slice_dh)
        print("self.resolution:", self.resolution)

        # print("self.center:", self.center)
        # print("self.offset:", self.offset)

        # 设置起始点和终点的高度
        self.start_idx[0] = self.height2idx(start_height)
        self.end_idx[0] = self.height2idx(end_height)
        
        # print("self.start_idx:", self.start_idx)
        # print("self.end_idx:", self.end_idx)
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
        
        # print("heights:", heights[554:554+10])
        # print("heights.shape:", heights.shape)  

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d
    
    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx
    
    def height2idx(self, height):
        height = height + 0.108
        if height > 11.3:
            height = 11.3
        idx = height *6.0/11.9
        # print("height:", height)
        # idx = height * self.slice_dh
        # print("idx:", idx)
        return idx
