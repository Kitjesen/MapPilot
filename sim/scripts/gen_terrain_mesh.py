#!/usr/bin/env python3
"""
spiral0.3_2.pickle → MuJoCo terrain meshes (.stl)

生成内容（写到 sim/assets/meshes/）：
  spiral_floor_0.stl   Floor 1: Z=0~5m
  spiral_floor_1.stl   Floor 2: Z=5~10m
  spiral_floor_2.stl   Floor 3: Z=10~15m
  spiral_floor_3.stl   Floor 4: Z=15~22m
  spiral_ramp_01.stl   Floor1→Floor2 坡道
  spiral_ramp_12.stl   Floor2→Floor3 坡道
  spiral_ramp_23.stl   Floor3→Floor4 坡道

用法：
    python sim/scripts/gen_terrain_mesh.py [--pickle PATH]
"""
import numpy as np
import os
import sys
import argparse
import struct

SIM_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR  = os.path.join(SIM_DIR, 'assets', 'meshes')
os.makedirs(OUT_DIR, exist_ok=True)

# 尝试找 pickle 文件
DEFAULT_PICKLE_PATHS = [
    '/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/rsc/tomogram/spiral0.3_2.pickle',
    r'C:\tmp\spiral0.3_2.pickle',
]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--pickle', default=None,
                   help='path to spiral0.3_2.pickle')
    p.add_argument('--pc-npy', default=None,
                   help='fallback: path to spiral_pc.npy (if no pickle)')
    p.add_argument('--res', type=float, default=0.4,
                   help='floor mesh resolution (m/cell)')
    return p.parse_args()


def write_stl_binary(path: str, vertices: np.ndarray, faces: np.ndarray):
    """
    写二进制 STL 文件。
    vertices: (N, 3) float32
    faces:    (M, 3) int，索引到 vertices
    """
    with open(path, 'wb') as f:
        f.write(b'\x00' * 80)            # header
        f.write(struct.pack('<I', len(faces)))
        for tri in faces:
            v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
            # 法向量
            n = np.cross(v1 - v0, v2 - v0)
            nn = np.linalg.norm(n)
            if nn > 0: n /= nn
            f.write(struct.pack('<fff', *n))
            f.write(struct.pack('<fff', *v0))
            f.write(struct.pack('<fff', *v1))
            f.write(struct.pack('<fff', *v2))
            f.write(struct.pack('<H', 0))  # attribute byte count


def pc_to_floor_mesh(pc: np.ndarray, trav: np.ndarray,
                     z_lo: float, z_hi: float, res: float,
                     label: str):
    """把指定 Z 范围内的点云体素化，生成地板 mesh STL。"""
    from scipy.ndimage import distance_transform_edt

    mask = (pc[:, 2] >= z_lo) & (pc[:, 2] < z_hi) & (trav < 35)
    sub  = pc[mask]
    if len(sub) < 50:
        print(f'  {label}: too few pts ({len(sub)}), skip')
        return None

    xmin, xmax = sub[:, 0].min(), sub[:, 0].max()
    ymin, ymax = sub[:, 1].min(), sub[:, 1].max()
    nx = max(2, int((xmax - xmin) / res) + 1)
    ny = max(2, int((ymax - ymin) / res) + 1)

    # height grid（取最低点作为地板）
    grid = np.full((ny, nx), np.nan)
    ix = np.clip(((sub[:, 0] - xmin) / res).astype(int), 0, nx - 1)
    iy = np.clip(((sub[:, 1] - ymin) / res).astype(int), 0, ny - 1)
    for x, y, z in zip(ix, iy, sub[:, 2]):
        if np.isnan(grid[y, x]) or z < grid[y, x]:
            grid[y, x] = z

    nan_m = np.isnan(grid)
    filled_ratio = 1 - nan_m.sum() / grid.size
    if filled_ratio < 0.1:
        print(f'  {label}: fill ratio too low ({filled_ratio:.1%}), skip')
        return None

    if nan_m.any():
        idx = distance_transform_edt(nan_m, return_distances=False, return_indices=True)
        grid = grid[tuple(idx)]

    # 生成 vertices + faces
    verts = []
    for j in range(ny):
        for i in range(nx):
            wx = xmin + i * res
            wy = ymin + j * res
            wz = float(grid[j, i])
            verts.append([wx, wy, wz])
    verts = np.array(verts, dtype=np.float32)

    def vid(j, i): return j * nx + i

    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            v00, v10 = vid(j, i),   vid(j, i + 1)
            v01, v11 = vid(j+1, i), vid(j+1, i + 1)
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])
    faces = np.array(faces, dtype=np.int32)

    print(f'  {label}: {len(verts):,} verts, {len(faces):,} faces, '
          f'fill={filled_ratio:.0%}, Z=[{sub[:,2].min():.1f},{sub[:,2].max():.1f}]m')
    return verts, faces


def path_to_ramp_mesh(path: np.ndarray, z_lo: float, z_hi: float,
                      half_width: float = 1.0):
    """
    用路径段生成坡道 mesh（路径的扫略体，宽 2*half_width）。
    """
    mask = (path[:, 2] >= z_lo) & (path[:, 2] < z_hi)
    sub  = path[mask]
    if len(sub) < 2:
        return None

    verts, faces = [], []
    for i in range(len(sub) - 1):
        p0, p1 = sub[i], sub[i + 1]
        tang = p1 - p0
        tang_norm = tang / (np.linalg.norm(tang) + 1e-6)
        perp = np.cross(tang_norm, [0, 0, 1])
        perp_n = perp / (np.linalg.norm(perp) + 1e-6)

        # 4 顶点（矩形截面）
        v0 = p0 - perp_n * half_width
        v1 = p0 + perp_n * half_width
        v2 = p1 + perp_n * half_width
        v3 = p1 - perp_n * half_width

        base = len(verts)
        verts.extend([v0, v1, v2, v3])
        faces.append([base, base+1, base+2])
        faces.append([base, base+2, base+3])

    verts = np.array(verts, dtype=np.float32)
    faces = np.array(faces, dtype=np.int32)
    print(f'  Ramp Z=[{z_lo:.0f},{z_hi:.0f}]: {len(verts)} verts, {len(faces)} faces')
    return verts, faces


def main():
    args = parse_args()

    # ── 加载数据 ─────────────────────────────────────────────────────────────
    pc, trav, path = None, None, None

    if args.pc_npy and os.path.exists(args.pc_npy):
        pc   = np.load(args.pc_npy)
        trav = np.load(args.pc_npy.replace('_pc.npy', '_trav.npy'))
        path = np.load(args.pc_npy.replace('_pc.npy', '_path.npy'))
        print(f'[MeshGen] Loaded from npy: {len(pc):,} pts')
    else:
        # 尝试从本地 C:\tmp（Windows dev 环境）
        npy_path = r'C:\tmp\spiral_pc.npy'
        if os.path.exists(npy_path):
            pc   = np.load(npy_path)
            trav = np.load(r'C:\tmp\spiral_trav.npy')
            path = np.load(r'C:\tmp\spiral_path.npy')
            print(f'[MeshGen] Loaded from C:\\tmp: {len(pc):,} pts')
        else:
            print('ERROR: No point cloud data found.')
            print('       Provide --pc-npy or ensure C:\\tmp\\spiral_pc.npy exists')
            sys.exit(1)

    print(f'  PC Z range: [{pc[:,2].min():.1f}, {pc[:,2].max():.1f}] m')

    # ── 生成各楼层 mesh ───────────────────────────────────────────────────────
    floor_ranges = [(0, 5), (5, 10), (10, 15), (15, 23)]

    print('\n[MeshGen] Generating floor meshes...')
    for fi, (z_lo, z_hi) in enumerate(floor_ranges):
        label = f'spiral_floor_{fi}'
        result = pc_to_floor_mesh(pc, trav, z_lo, z_hi, args.res, label)
        if result is not None:
            verts, faces = result
            out = os.path.join(OUT_DIR, f'{label}.stl')
            write_stl_binary(out, verts, faces)
            print(f'    -> {out}')

    # ── 生成坡道 mesh（用路径段扫略）─────────────────────────────────────────
    ramp_ranges = [(4, 6), (9, 11), (14, 16)]  # 楼层衔接区段

    print('\n[MeshGen] Generating ramp meshes...')
    for ri, (z_lo, z_hi) in enumerate(ramp_ranges):
        label = f'spiral_ramp_{ri}{ri+1}'
        result = path_to_ramp_mesh(path, z_lo, z_hi, half_width=1.5)
        if result is not None:
            verts, faces = result
            out = os.path.join(OUT_DIR, f'{label}.stl')
            write_stl_binary(out, verts, faces)
            print(f'    -> {out}')

    print(f'\n[MeshGen] Done. Meshes in: {OUT_DIR}')
    print('Next: python sim/scripts/run_sim.py --world spiral_terrain')


if __name__ == '__main__':
    main()
