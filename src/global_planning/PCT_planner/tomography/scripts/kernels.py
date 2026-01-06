import numpy as np


def get_index_map_1d(px, py, cx, cy, resolution, n_row, n_col):
    """Return 1D index of a point (x, y) in a layer"""
    idx_x = int(round((px - cx) / resolution)) + n_row // 2
    idx_y = int(round((py - cy) / resolution)) + n_col // 2
    
    if idx_x < 0 or idx_x >= n_row or idx_y < 0 or idx_y >= n_col:
        return -1
    return n_col * idx_x + idx_y


def get_index_block_1d(idx, layer_n, layer_size):
    """Return 1D index of a point (x, y) in multi-layer map block"""
    return layer_size * layer_n + idx


def tomographyKernel_cpu(points, center, layers_g, layers_c, resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    """
    CPU version of tomography kernel
    Processes points to build ground and ceiling layers
    """
    cx, cy = center[0], center[1]
    layer_size = n_row * n_col
    
    for i in range(points.shape[0]):
        px = points[i, 0]
        py = points[i, 1]
        pz = points[i, 2]
        
        idx = get_index_map_1d(px, py, cx, cy, resolution, n_row, n_col)
        if idx < 0:
            continue
            
        for s_idx in range(n_slice):
            slice_height = slice_h0 + s_idx * slice_dh
            block_idx = get_index_block_1d(idx, s_idx, layer_size)
            
            if pz <= slice_height:
                # Update ground layer (take maximum)
                layers_g.flat[block_idx] = max(layers_g.flat[block_idx], pz)
            else:
                # Update ceiling layer (take minimum)
                layers_c.flat[block_idx] = min(layers_c.flat[block_idx], pz)


def get_idx_relative(idx, dx, dy, n_row, n_col, layer_size):
    """Return 1D index of the relative point (x+dx, y+dy) in multi-layer map block"""
    idx_2d = idx % layer_size
    idx_x = idx_2d // n_col
    idx_y = idx_2d % n_col
    idx_rx = idx_x + dx
    idx_ry = idx_y + dy
    
    if idx_rx < 0 or idx_rx >= n_row:
        return -1
    if idx_ry < 0 or idx_ry >= n_col:
        return -1
    
    return n_col * dx + dy + idx


def travKernel_cpu(interval, grad_mag_sq, grad_mag_max, trav_cost, 
                   n_row, n_col, half_kernel_size, 
                   interval_min, interval_free, step_cross, step_stand, 
                   standable_th, cost_barrier):
    """
    CPU version of traversability kernel
    Computes travel cost based on interval and terrain gradient
    """
    layer_size = n_row * n_col
    step_cross_sq = step_cross ** 2
    step_stand_sq = step_stand ** 2
    
    total_size = interval.size
    for i in range(total_size):
        if interval.flat[i] < interval_min:
            trav_cost.flat[i] = cost_barrier
            continue
        else:
            trav_cost.flat[i] += max(0.0, 20 * (interval_free - interval.flat[i]))
            
        if grad_mag_sq.flat[i] <= step_stand_sq:
            trav_cost.flat[i] += 15 * grad_mag_sq.flat[i] / step_stand_sq
            continue
        else:
            if grad_mag_max.flat[i] <= step_cross_sq:
                standable_grids = 0
                for dy in range(-half_kernel_size, half_kernel_size + 1):
                    for dx in range(-half_kernel_size, half_kernel_size + 1):
                        idx = get_idx_relative(i, dx, dy, n_row, n_col, layer_size)
                        if idx < 0:
                            continue
                        if grad_mag_sq.flat[idx] < step_stand_sq:
                            standable_grids += 1
                            
                if standable_grids < standable_th:
                    trav_cost.flat[i] = cost_barrier
                    continue
                else:
                    trav_cost.flat[i] += 20 * grad_mag_max.flat[i] / step_cross_sq
            else:
                trav_cost.flat[i] = cost_barrier
                continue


def inflationKernel_cpu(trav_cost, score_table, inflated_cost, 
                        n_row, n_col, half_kernel_size):
    """
    CPU version of inflation kernel
    Applies inflation to travel cost map
    """
    layer_size = n_row * n_col
    total_size = trav_cost.size
    
    for i in range(total_size):
        counter = 0
        max_cost = inflated_cost.flat[i]
        
        for dy in range(-half_kernel_size, half_kernel_size + 1):
            for dx in range(-half_kernel_size, half_kernel_size + 1):
                idx = get_idx_relative(i, dx, dy, n_row, n_col, layer_size)
                if idx >= 0:
                    cost_val = trav_cost.flat[idx] * score_table.flat[counter]
                    max_cost = max(max_cost, cost_val)
                counter += 1
                
        inflated_cost.flat[i] = max_cost


# For backward compatibility, create wrapper functions
def tomographyKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    """Wrapper function for CPU version"""
    def kernel_func(points, center, layers_g, layers_c):
        tomographyKernel_cpu(points, center, layers_g, layers_c, 
                            resolution, n_row, n_col, n_slice, slice_h0, slice_dh)
    return kernel_func


def travKernel(n_row, n_col, half_kernel_size,
               interval_min, interval_free, step_cross, step_stand, 
               standable_th, cost_barrier):
    """Wrapper function for CPU version"""
    def kernel_func(interval, grad_mag_sq, grad_mag_max, trav_cost):
        travKernel_cpu(interval, grad_mag_sq, grad_mag_max, trav_cost,
                      n_row, n_col, half_kernel_size,
                      interval_min, interval_free, step_cross, step_stand,
                      standable_th, cost_barrier)
    return kernel_func


def inflationKernel(n_row, n_col, half_kernel_size):
    """Wrapper function for CPU version"""
    def kernel_func(trav_cost, score_table, inflated_cost):
        inflationKernel_cpu(trav_cost, score_table, inflated_cost,
                           n_row, n_col, half_kernel_size)
    return kernel_func
