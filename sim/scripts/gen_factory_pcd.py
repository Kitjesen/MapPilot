#!/usr/bin/env python3
"""
多层工厂场景 + 旋转楼梯 — MuJoCo 点云地图生成

场景布局 (30m x 20m, 3层):
  1F (z=0):   生产车间 — 设备、货架、传送带
  2F (z=3m):  办公/控制区 — 隔间、走廊
  3F (z=6m):  屋顶/观测层 — 开放区域
  旋转楼梯 (x=25, y=10): 连接 1F→2F→3F, 半径 2.5m

用法:
    python3 gen_factory_pcd.py                           # 默认输出
    python3 gen_factory_pcd.py /tmp/sim_maps/factory.pcd # 自定义路径
"""
import numpy as np
import os
import sys
import mujoco

# ── 旋转楼梯生成 ─────────────────────────────────────────────────
def _spiral_stair_xml(cx, cy, z_start, z_end, radius, n_steps, prefix,
                      step_width=0.8, step_depth=0.7):
    """生成旋转楼梯的 MuJoCo XML geom 列表。

    楼梯绕 (cx, cy) 旋转一整圈(360°)，从 z_start 上升到 z_end。
    每个台阶是一个倾斜放置的 box。
    """
    lines = []
    dh = (z_end - z_start) / n_steps
    dangle = 2 * np.pi / n_steps  # 每步旋转角度

    for i in range(n_steps):
        angle = i * dangle
        z = z_start + (i + 0.5) * dh
        # 台阶中心位置
        x = cx + radius * np.cos(angle)
        y = cy + radius * np.sin(angle)
        # 台阶尺寸: half-sizes
        sx = step_depth / 2
        sy = step_width / 2
        sz = dh / 2
        # euler: 绕 z 轴旋转对齐切线方向
        yaw = angle + np.pi / 2  # 切线方向
        lines.append(
            f'    <geom name="{prefix}_step{i:02d}" type="box" '
            f'size="{sx:.3f} {sy:.3f} {sz:.3f}" '
            f'pos="{x:.3f} {y:.3f} {z:.3f}" '
            f'euler="0 0 {yaw:.4f}" group="1" conaffinity="1" '
            f'rgba=".6 .6 .55 1"/>'
        )

    # 中心柱
    h_total = z_end - z_start
    lines.append(
        f'    <geom name="{prefix}_pole" type="cylinder" '
        f'size="0.15 {h_total/2:.3f}" '
        f'pos="{cx:.3f} {cy:.3f} {z_start + h_total/2:.3f}" '
        f'group="1" conaffinity="1" rgba=".5 .5 .5 1"/>'
    )
    return '\n'.join(lines)


# ── 楼板生成 ─────────────────────────────────────────────────────
def _floor_slab_xml(name, z, x_range, y_range, hole_cx=None, hole_cy=None,
                    hole_r=3.5, thickness=0.15):
    """生成楼板（带楼梯口开孔）。

    用 4 个 box 围绕楼梯口构成 U 形楼板，留出圆形通道区域。
    """
    x0, x1 = x_range
    y0, y1 = y_range
    sz = thickness / 2

    if hole_cx is None:
        # 完整楼板
        cx = (x0 + x1) / 2
        cy = (y0 + y1) / 2
        return (f'    <geom name="{name}" type="box" '
                f'size="{(x1-x0)/2:.2f} {(y1-y0)/2:.2f} {sz:.3f}" '
                f'pos="{cx:.2f} {cy:.2f} {z:.3f}" '
                f'group="1" conaffinity="1" rgba=".75 .75 .72 1"/>')

    # 带开孔的楼板：分成 4 块
    lines = []
    # 左侧板: x0 到 hole_cx - hole_r
    if hole_cx - hole_r > x0 + 0.5:
        lx = (x0 + hole_cx - hole_r) / 2
        lsx = (hole_cx - hole_r - x0) / 2
        lines.append(
            f'    <geom name="{name}_L" type="box" '
            f'size="{lsx:.2f} {(y1-y0)/2:.2f} {sz:.3f}" '
            f'pos="{lx:.2f} {(y0+y1)/2:.2f} {z:.3f}" '
            f'group="1" conaffinity="1" rgba=".75 .75 .72 1"/>')
    # 右侧板: hole_cx + hole_r 到 x1
    if hole_cx + hole_r < x1 - 0.5:
        rx = (hole_cx + hole_r + x1) / 2
        rsx = (x1 - hole_cx - hole_r) / 2
        lines.append(
            f'    <geom name="{name}_R" type="box" '
            f'size="{rsx:.2f} {(y1-y0)/2:.2f} {sz:.3f}" '
            f'pos="{rx:.2f} {(y0+y1)/2:.2f} {z:.3f}" '
            f'group="1" conaffinity="1" rgba=".75 .75 .72 1"/>')
    # 前侧板 (y > hole_cy + hole_r): 中间段
    if hole_cy + hole_r < y1 - 0.5:
        fy = (hole_cy + hole_r + y1) / 2
        fsy = (y1 - hole_cy - hole_r) / 2
        cx_mid = (max(x0, hole_cx - hole_r) + min(x1, hole_cx + hole_r)) / 2
        sx_mid = (min(x1, hole_cx + hole_r) - max(x0, hole_cx - hole_r)) / 2
        if sx_mid > 0.2:
            lines.append(
                f'    <geom name="{name}_F" type="box" '
                f'size="{sx_mid:.2f} {fsy:.2f} {sz:.3f}" '
                f'pos="{cx_mid:.2f} {fy:.2f} {z:.3f}" '
                f'group="1" conaffinity="1" rgba=".75 .75 .72 1"/>')
    # 后侧板 (y < hole_cy - hole_r)
    if hole_cy - hole_r > y0 + 0.5:
        by = (y0 + hole_cy - hole_r) / 2
        bsy = (hole_cy - hole_r - y0) / 2
        cx_mid = (max(x0, hole_cx - hole_r) + min(x1, hole_cx + hole_r)) / 2
        sx_mid = (min(x1, hole_cx + hole_r) - max(x0, hole_cx - hole_r)) / 2
        if sx_mid > 0.2:
            lines.append(
                f'    <geom name="{name}_B" type="box" '
                f'size="{sx_mid:.2f} {bsy:.2f} {sz:.3f}" '
                f'pos="{cx_mid:.2f} {by:.2f} {z:.3f}" '
                f'group="1" conaffinity="1" rgba=".75 .75 .72 1"/>')

    return '\n'.join(lines)


# ── 场景 XML ─────────────────────────────────────────────────────
STAIR_CX, STAIR_CY = 25.0, 10.0
STAIR_R = 2.0
N_STEPS = 24  # 每层楼梯台阶数 (24步 = 360°)


def build_factory_xml():
    stair_1f = _spiral_stair_xml(STAIR_CX, STAIR_CY, 0.0, 3.0, STAIR_R,
                                  N_STEPS, 'stair1f')
    stair_2f = _spiral_stair_xml(STAIR_CX, STAIR_CY, 3.0, 6.0, STAIR_R,
                                  N_STEPS, 'stair2f')

    floor_2f = _floor_slab_xml('floor2f', 3.0, (0, 30), (0, 20),
                                hole_cx=STAIR_CX, hole_cy=STAIR_CY, hole_r=3.5)
    floor_3f = _floor_slab_xml('floor3f', 6.0, (0, 30), (0, 20),
                                hole_cx=STAIR_CX, hole_cy=STAIR_CY, hole_r=3.5)

    xml = f"""<mujoco model="factory_3floor">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <!-- ===== 地面 (1F) ===== -->
    <geom name="floor" type="plane" size="40 40 0.1" group="1" conaffinity="1"
          rgba=".82 .82 .78 1"/>

    <!-- ===== 外墙 (30m x 20m, 高 8m) ===== -->
    <geom name="wall_s" type="box" size="15 0.15 4.0" pos="15 0 4.0" group="1" conaffinity="1" rgba=".7 .7 .68 1"/>
    <geom name="wall_n" type="box" size="15 0.15 4.0" pos="15 20 4.0" group="1" conaffinity="1" rgba=".7 .7 .68 1"/>
    <geom name="wall_w" type="box" size="0.15 10 4.0" pos="0 10 4.0" group="1" conaffinity="1" rgba=".7 .7 .68 1"/>
    <geom name="wall_e" type="box" size="0.15 10 4.0" pos="30 10 4.0" group="1" conaffinity="1" rgba=".7 .7 .68 1"/>

    <!-- ===== 1F 生产车间 (z=0) ===== -->
    <!-- 大型设备 -->
    <geom name="machine1" type="box" size="1.5 1.0 1.5" pos="4 4 1.5" group="1" conaffinity="1" rgba=".4 .5 .6 1"/>
    <geom name="machine2" type="box" size="1.5 1.0 1.5" pos="4 8 1.5" group="1" conaffinity="1" rgba=".4 .5 .6 1"/>
    <geom name="machine3" type="box" size="2.0 1.2 2.0" pos="4 14 2.0" group="1" conaffinity="1" rgba=".45 .5 .55 1"/>

    <!-- 传送带 (长条低矮障碍) -->
    <geom name="conveyor1" type="box" size="5.0 0.4 0.5" pos="12 5 0.5" group="1" conaffinity="1" rgba=".35 .35 .4 1"/>
    <geom name="conveyor2" type="box" size="5.0 0.4 0.5" pos="12 15 0.5" group="1" conaffinity="1" rgba=".35 .35 .4 1"/>

    <!-- 货架 -->
    <geom name="rack1" type="box" size="0.3 2.0 2.0" pos="8 10 2.0" group="1" conaffinity="1" rgba=".55 .45 .3 1"/>
    <geom name="rack2" type="box" size="0.3 2.0 2.0" pos="10 10 2.0" group="1" conaffinity="1" rgba=".55 .45 .3 1"/>

    <!-- 叉车/小型障碍 -->
    <geom name="forklift1" type="box" size="0.6 0.4 0.8" pos="16 3 0.8" group="1" conaffinity="1" rgba=".8 .6 .2 1"/>
    <geom name="pallet1" type="box" size="0.6 0.4 0.15" pos="16 7 0.15" group="1" conaffinity="1"/>
    <geom name="pallet2" type="box" size="0.6 0.4 0.15" pos="18 7 0.15" group="1" conaffinity="1"/>

    <!-- 柱子 (贯穿多层) -->
    <geom name="pillar1" type="cylinder" size="0.3 4.0" pos="7.5 5 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>
    <geom name="pillar2" type="cylinder" size="0.3 4.0" pos="7.5 15 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>
    <geom name="pillar3" type="cylinder" size="0.3 4.0" pos="15 5 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>
    <geom name="pillar4" type="cylinder" size="0.3 4.0" pos="15 15 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>
    <geom name="pillar5" type="cylinder" size="0.3 4.0" pos="22 5 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>
    <geom name="pillar6" type="cylinder" size="0.3 4.0" pos="22 15 4.0" group="1" conaffinity="1" rgba=".6 .6 .6 1"/>

    <!-- 内墙: 1F 隔断 (x=20, 留门洞 y=8~12) -->
    <geom name="iwall1f_a" type="box" size="0.12 4.0 2.0" pos="20 4 2.0" group="1" conaffinity="1"/>
    <geom name="iwall1f_b" type="box" size="0.12 4.0 2.0" pos="20 16 2.0" group="1" conaffinity="1"/>

    <!-- ===== 旋转楼梯 1F→2F ===== -->
{stair_1f}

    <!-- ===== 2F 楼板 (z=3m, 楼梯口开孔) ===== -->
{floor_2f}

    <!-- ===== 2F 办公区 (z=3m) ===== -->
    <!-- 内墙 -->
    <geom name="iwall2f_a" type="box" size="0.12 4.5 1.5" pos="10 4.5 4.5" group="1" conaffinity="1"/>
    <geom name="iwall2f_b" type="box" size="0.12 4.5 1.5" pos="10 15.5 4.5" group="1" conaffinity="1"/>
    <geom name="iwall2f_c" type="box" size="5.0 0.12 1.5" pos="5 10 4.5" group="1" conaffinity="1"/>

    <!-- 办公桌 -->
    <geom name="desk1" type="box" size="0.8 0.4 0.4" pos="3 4 3.4" group="1" conaffinity="1" rgba=".6 .5 .35 1"/>
    <geom name="desk2" type="box" size="0.8 0.4 0.4" pos="3 7 3.4" group="1" conaffinity="1" rgba=".6 .5 .35 1"/>
    <geom name="desk3" type="box" size="0.8 0.4 0.4" pos="3 13 3.4" group="1" conaffinity="1" rgba=".6 .5 .35 1"/>
    <geom name="desk4" type="box" size="0.8 0.4 0.4" pos="3 16 3.4" group="1" conaffinity="1" rgba=".6 .5 .35 1"/>

    <!-- 服务器机柜 -->
    <geom name="server1" type="box" size="0.4 0.5 1.0" pos="14 3 4.0" group="1" conaffinity="1" rgba=".2 .2 .25 1"/>
    <geom name="server2" type="box" size="0.4 0.5 1.0" pos="14 5 4.0" group="1" conaffinity="1" rgba=".2 .2 .25 1"/>

    <!-- 2F 内墙: 隔断 (x=18, 留门洞 y=8~12) -->
    <geom name="iwall2f_d" type="box" size="0.12 4.0 1.5" pos="18 4 4.5" group="1" conaffinity="1"/>
    <geom name="iwall2f_e" type="box" size="0.12 4.0 1.5" pos="18 16 4.5" group="1" conaffinity="1"/>

    <!-- ===== 旋转楼梯 2F→3F ===== -->
{stair_2f}

    <!-- ===== 3F 楼板 (z=6m, 楼梯口开孔) ===== -->
{floor_3f}

    <!-- ===== 3F 观测层 (z=6m) ===== -->
    <!-- 设备平台 -->
    <geom name="platform3f" type="box" size="2.0 1.5 0.3" pos="5 10 6.3" group="1" conaffinity="1" rgba=".6 .6 .55 1"/>

    <!-- 护栏 (楼梯口周围) -->
    <geom name="guard3f_a" type="box" size="3.5 0.05 0.5" pos="25 6.5 6.5" group="1" conaffinity="1"/>
    <geom name="guard3f_b" type="box" size="3.5 0.05 0.5" pos="25 13.5 6.5" group="1" conaffinity="1"/>
    <geom name="guard3f_c" type="box" size="0.05 3.5 0.5" pos="21.5 10 6.5" group="1" conaffinity="1"/>

  </worldbody>
</mujoco>
"""
    return xml


# ── LiDAR 参数 ──────────────────────────────────────────────────
N_RAYS = 6400
GOLDEN_ANG = np.pi * (3 - np.sqrt(5))
RANGE_MAX = 25.0

CLIP_X = (-2, 32)
CLIP_Y = (-2, 22)
CLIP_Z = (-0.5, 9.0)


def build_ray_dirs(n, vfov_min_deg=-30, vfov_max_deg=60):
    i = np.arange(n, dtype=np.float64)
    ha = (i * GOLDEN_ANG) % (2 * np.pi)
    vfov_min = np.deg2rad(vfov_min_deg)
    vfov_max = np.deg2rad(vfov_max_deg)
    va = vfov_min + i / n * (vfov_max - vfov_min)
    cv = np.cos(va)
    return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])


def scan_at(model, data, pos, ray_dirs, geomgroup):
    geomid = np.full(len(ray_dirs), -1, dtype=np.int32)
    dist = np.full(len(ray_dirs), -1.0, dtype=np.float64)
    mujoco.mj_multiRay(
        model, data, np.array(pos, dtype=np.float64),
        ray_dirs.flatten(), geomgroup, 1, -1,
        geomid, dist, None, len(ray_dirs), RANGE_MAX
    )
    mask = dist > 0.05
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float32)
    pts = np.array(pos) + ray_dirs[mask] * dist[mask, None]
    return pts.astype(np.float32)


def main():
    out_path = '/tmp/sim_maps/factory.pcd'
    if len(sys.argv) > 1:
        out_path = sys.argv[1]

    xml = build_factory_xml()

    # 保存 XML 供 sim_viz_full.py 使用
    xml_path = os.path.join(os.path.dirname(out_path), 'factory.xml')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(xml_path, 'w') as f:
        f.write(xml)
    print(f"Saved scene XML: {xml_path}")

    print("Loading MuJoCo scene...")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)

    ray_dirs = build_ray_dirs(N_RAYS)
    geomgroup = np.zeros(6, dtype=np.uint8)
    geomgroup[1] = 1

    # 多层扫描：每层楼都需要扫描点
    scan_positions = []

    # 1F: z=0.7m, 2m 间隔网格
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            scan_positions.append([x, y, 0.7])

    # 2F: z=3.7m
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            scan_positions.append([x, y, 3.7])

    # 3F: z=6.7m
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            scan_positions.append([x, y, 6.7])

    # 楼梯间加密扫描 (每 0.5m 高度一个扫描点)
    for z in np.arange(0.5, 6.5, 0.5):
        for angle in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            x = STAIR_CX + (STAIR_R + 0.5) * np.cos(angle)
            y = STAIR_CY + (STAIR_R + 0.5) * np.sin(angle)
            scan_positions.append([x, y, z + 0.5])

    print(f"Scanning from {len(scan_positions)} positions...")
    all_pts = []
    n_scans = 0
    for pos in scan_positions:
        pts = scan_at(model, data, pos, ray_dirs, geomgroup)
        if len(pts) > 0:
            all_pts.append(pts)
            n_scans += 1

    print(f"Scanned {n_scans} positions")
    cloud = np.concatenate(all_pts)
    print(f"Raw points: {len(cloud)}")

    # Bbox clip
    mask = (
        (cloud[:, 0] >= CLIP_X[0]) & (cloud[:, 0] <= CLIP_X[1]) &
        (cloud[:, 1] >= CLIP_Y[0]) & (cloud[:, 1] <= CLIP_Y[1]) &
        (cloud[:, 2] >= CLIP_Z[0]) & (cloud[:, 2] <= CLIP_Z[1])
    )
    cloud = cloud[mask]
    print(f"After bbox clip: {len(cloud)} pts")

    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.astype(np.float64))
    pcd = pcd.voxel_down_sample(0.1)
    print(f"After voxel downsample: {len(pcd.points)} pts")

    o3d.io.write_point_cloud(out_path, pcd)
    pts = np.asarray(pcd.points)
    print(f"Saved: {out_path}")
    print(f"Bounds: x=[{pts[:,0].min():.1f}, {pts[:,0].max():.1f}], "
          f"y=[{pts[:,1].min():.1f}, {pts[:,1].max():.1f}], "
          f"z=[{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]")


if __name__ == '__main__':
    main()
