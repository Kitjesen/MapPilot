import numpy as np


def transTrajGrid2Map(grid_dim, center, resolution, traj_grid):
    # C++ 输出路径为 [x_cpp, y_cpp, z], 其中:
    #   x_cpp = round((x_world - cx) / res) + grid_dim[0]//2
    #   y_cpp = round((y_world - cy) / res) + grid_dim[1]//2
    # 逆映射: x_world = (x_cpp - dim0//2)*res + cx, y_world = (y_cpp - dim1//2)*res + cy
    offset = np.array([grid_dim[0] // 2, grid_dim[1] // 2, 0])
    center_ = np.array([center[0], center[1], 0.5])

    traj_map = (traj_grid - offset) * resolution + center_

    return traj_map
