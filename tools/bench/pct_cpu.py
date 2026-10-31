"""Offline CPU port of PCT tomography (upstream 35cd73fd).

Uses the equations and slice simplification in tomography/scripts/{kernels,
tomogram}.py. Native PCT search/optimization are separate. No ROS/CUDA imports.
This is a CPU adaptation, not a GPU/CPU numerical-parity certification.
"""

import time
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import convolve


@dataclass(frozen=True)
class Config:
    resolution: float = 0.10
    slice_dh: float = 0.5
    kernel_size: int = 7
    interval_min: float = 0.50
    interval_free: float = 0.65
    slope_max: float = 0.40
    step_max: float = 0.17
    standable_ratio: float = 0.20
    cost_barrier: float = 50.0
    safe_margin: float = 0.4
    inflation: float = 0.2


def select_layer(ground, ceiling, cost, body_z, reference_height=0.35):
    """Select the vertical interval containing the requested body position."""
    layers = np.flatnonzero(np.isfinite(ground) & (ground <= body_z)
                            & (~np.isfinite(ceiling) | (ceiling >= body_z)))
    if not len(layers):
        return None
    error = np.abs(ground[layers] - (body_z - reference_height))
    order = np.lexsort((cost[layers], error))
    return int(layers[order[0]])


def point_indices(points, center, resolution, shape):
    # Upstream getIndexLine receives float16 coordinates and uses C round,
    # whose halfway cases round away from zero (unlike numpy.rint).
    delta = (points[:, :2].astype(np.float16) - np.asarray(center, dtype=np.float16)).astype(np.float64)
    scaled = delta / resolution
    return (np.sign(scaled) * np.floor(np.abs(scaled) + 0.5)).astype(np.int64) + np.array(shape) // 2


def surfaces(points, center, shape, heights, resolution):
    indices = point_indices(points, center, resolution, shape)
    valid = ((indices >= 0) & (indices < shape)).all(axis=1)
    indices = indices[valid]
    z = points[valid, 2]
    flat = indices[:, 0] * shape[1] + indices[:, 1]
    ground = np.full((len(heights), *shape), -1e6, dtype=np.float32)
    ceiling = np.full_like(ground, 1e6)
    for i, height in enumerate(heights):
        below = z <= height
        np.maximum.at(ground[i].reshape(-1), flat[below], z[below])
        np.minimum.at(ceiling[i].reshape(-1), flat[~below], z[~below])
    return ground, ceiling


def traversability(ground, ceiling, cfg):
    sq = np.zeros_like(ground)
    maximum = np.zeros_like(ground)
    dx = np.maximum((ground[:, 1:-1] - ground[:, :-2]) ** 2,
                    (ground[:, 1:-1] - ground[:, 2:]) ** 2)
    dy = np.maximum((ground[:, :, 1:-1] - ground[:, :, :-2]) ** 2,
                    (ground[:, :, 1:-1] - ground[:, :, 2:]) ** 2)
    sq[:, 1:-1, 1:-1] = dx[:, :, 1:-1] + dy[:, 1:-1]
    maximum[:, 1:-1, 1:-1] = np.maximum(dx[:, :, 1:-1], dy[:, 1:-1])
    stand_sq = (1.2 * cfg.resolution * np.tan(cfg.slope_max)) ** 2
    cross_sq = cfg.step_max ** 2
    interval = ceiling - ground
    costs = np.maximum(0, 20 * (cfg.interval_free - interval))
    low = interval < cfg.interval_min
    smooth = sq <= stand_sq
    costs[smooth] += 15 * sq[smooth] / stand_sq
    count = convolve((sq < stand_sq).astype(np.int32),
                     np.ones((1, cfg.kernel_size, cfg.kernel_size), dtype=np.int32),
                     mode="constant", cval=0)
    threshold = int(cfg.standable_ratio * cfg.kernel_size ** 2) - 1
    cross = ~smooth & (maximum <= cross_sq) & (count >= threshold)
    costs[cross] += 20 * maximum[cross] / cross_sq
    costs[low | (~smooth & ~cross)] = cfg.cost_barrier
    return costs.astype(np.float32)


def inflate(costs, cfg):
    result = np.zeros_like(costs)
    radius = int((cfg.safe_margin + cfg.inflation) / cfg.resolution)
    nx, ny = costs.shape[1:]
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            weight = np.clip(1 - (np.hypot(dx, dy) * cfg.resolution - cfg.inflation)
                             / (cfg.safe_margin + cfg.resolution), 0, 1)
            if weight == 0 or abs(dx) >= nx or abs(dy) >= ny:
                continue
            x0, x1 = max(0, -dx), min(nx, nx - dx)
            y0, y1 = max(0, -dy), min(ny, ny - dy)
            dst = result[:, x0:x1, y0:y1]
            np.maximum(dst, costs[:, x0+dx:x1+dx, y0+dy:y1+dy] * np.float32(weight), out=dst)
    return result


def simplify(ground, cost, barrier):
    selected = [0]
    if len(ground) > 1:
        lower, middle = 0, 1
        difference = ground[1:] - ground[:-1]
        while middle < len(ground) - 2:
            unique = ((ground[middle] - ground[lower] > 0) | (cost[lower] > cost[middle]))
            unique &= (difference[middle] > 0) & (cost[middle] < barrier)
            if np.any(unique):
                selected.append(middle)
                lower = middle
            middle += 1
        selected.append(middle)
    return selected


def build(points, cfg=Config(), ground_h=None):
    begin = time.perf_counter()
    points = np.asarray(points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    # Preserve all input elevations unless the caller supplies a scene datum.
    if ground_h is None:
        ground_h = float(minimum[2])
    center = (minimum[:2] + maximum[:2]) / 2
    shape = tuple((np.ceil((maximum[:2] - minimum[:2]) / cfg.resolution).astype(int) + 4).tolist())
    count = max(1, int(np.ceil((maximum[2] - ground_h) / cfg.slice_dh)))
    h0 = ground_h + cfg.slice_dh
    heights = np.asarray(h0 + np.arange(count) * cfg.slice_dh, dtype=np.float32)
    ground, ceiling = surfaces(points, center, shape, heights, cfg.resolution)
    mapped = time.perf_counter()
    raw_cost = traversability(ground, ceiling, cfg)
    costs = inflate(raw_cost, cfg)
    traversed = time.perf_counter()
    selected = simplify(ground, costs, cfg.cost_barrier)
    ground, ceiling, costs = ground[selected], ceiling[selected], costs[selected]
    gx, gy = np.zeros_like(costs), np.zeros_like(costs)
    gx[:, 1:-1] = costs[:, 2:] - costs[:, :-2]
    gy[:, :, 1:-1] = costs[:, :, 2:] - costs[:, :, :-2]
    ground[ground <= -1e6] = np.nan
    ceiling[ceiling >= 1e6] = np.nan
    return dict(data=np.stack((costs, gx, gy, ground, ceiling)).astype(np.float16),
                resolution=cfg.resolution, center=center, slice_h0=h0, slice_dh=cfg.slice_dh,
                selected_slices=selected, raw_cost=raw_cost[selected], input_points=len(points),
                timing_s=dict(mapping=mapped-begin, traversability=traversed-mapped,
                              total=time.perf_counter()-begin))
