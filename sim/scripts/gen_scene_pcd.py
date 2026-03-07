#!/usr/bin/env python3
"""
从 MuJoCo 场景 XML 预生成稠密点云地图 (PCD)。

在场景中均匀采样虚拟扫描位置，用 mj_multiRay 360° 扫描，
合并所有点云 → 体素降采样 → 保存为 Open3D PCD 文件。

用法:
    python3 gen_scene_pcd.py                      # 使用内置场景
    python3 gen_scene_pcd.py scene.xml out.pcd     # 自定义场景
"""
import numpy as np
import os
import sys
import mujoco

# ── 场景 XML (与 sim_viz_full.py 相同) ──────────────────────────
SCENE_XML = """
<mujoco model="building_nav">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <!-- 地面 -->
    <geom name="floor" type="plane" size="30 30 0.1" group="1" conaffinity="1" condim="3"
          rgba=".85 .85 .82 1"/>

    <!-- 外墙 (25m x 18m) -->
    <geom name="wall_s" type="box" size="12.5 0.15 2.0" pos="12.5 0 2.0" group="1" conaffinity="1"/>
    <geom name="wall_n" type="box" size="12.5 0.15 2.0" pos="12.5 18 2.0" group="1" conaffinity="1"/>
    <geom name="wall_w" type="box" size="0.15 9.0 2.0" pos="0 9 2.0" group="1" conaffinity="1"/>
    <geom name="wall_e1" type="box" size="0.15 3.5 2.0" pos="25 3.5 2.0" group="1" conaffinity="1"/>
    <geom name="wall_e2" type="box" size="0.15 3.5 2.0" pos="25 14.5 2.0" group="1" conaffinity="1"/>

    <!-- 内墙 (x=15, 门洞 y=7~10) -->
    <geom name="iwall1" type="box" size="0.12 3.5 2.0" pos="15 3.5 2.0" group="1" conaffinity="1"/>
    <geom name="iwall2" type="box" size="0.12 4.0 2.0" pos="15 14.0 2.0" group="1" conaffinity="1"/>

    <!-- 楼梯 -->
    <geom name="stair01" type="box" size="1.5 1.5 0.125" pos="21.5 2.5 0.125" group="1" conaffinity="1"/>
    <geom name="stair02" type="box" size="1.5 1.5 0.250" pos="21.5 2.5 0.250" group="1" conaffinity="1"/>
    <geom name="stair03" type="box" size="1.5 1.2 0.375" pos="21.5 2.5 0.375" group="1" conaffinity="1"/>
    <geom name="stair04" type="box" size="1.5 0.9 0.500" pos="21.5 2.8 0.500" group="1" conaffinity="1"/>
    <geom name="stair05" type="box" size="1.5 0.6 0.625" pos="21.5 3.1 0.625" group="1" conaffinity="1"/>
    <geom name="stair06" type="box" size="1.5 0.3 0.750" pos="21.5 3.4 0.750" group="1" conaffinity="1"/>
    <geom name="stair_landing" type="box" size="2.0 2.0 0.05" pos="21.5 5.5 1.5" group="1" conaffinity="1"/>

    <!-- 坡道 -->
    <geom name="ramp1" type="box" size="2.5 1.5 0.05" pos="5.5 13.5 0.3"
          euler="0 -0.12 0" group="1" conaffinity="1"/>

    <!-- 1F 障碍物 -->
    <geom name="shelf1" type="box" size="0.8 0.3 1.0" pos="4 4 1.0" group="1" conaffinity="1"/>
    <geom name="shelf2" type="box" size="0.8 0.3 1.0" pos="4 6 1.0" group="1" conaffinity="1"/>
    <geom name="table1" type="box" size="1.0 0.6 0.4" pos="8 9 0.4" group="1" conaffinity="1"/>
    <geom name="pillar1" type="cylinder" size="0.25 2.0" pos="10 5 2.0" group="1" conaffinity="1"/>
    <geom name="pillar2" type="cylinder" size="0.25 2.0" pos="10 13 2.0" group="1" conaffinity="1"/>
    <geom name="crate1" type="box" size="0.4 0.4 0.4" pos="12 3 0.4" group="1" conaffinity="1"/>
    <geom name="crate2" type="box" size="0.35 0.35 0.35" pos="12 3 1.15" group="1" conaffinity="1"/>
    <geom name="barrel1" type="cylinder" size="0.3 0.5" pos="7 15 0.5" group="1" conaffinity="1"/>
    <geom name="barrel2" type="cylinder" size="0.3 0.5" pos="7.8 15.3 0.5" group="1" conaffinity="1"/>

    <!-- 东区障碍物 -->
    <geom name="container1" type="box" size="1.5 0.8 1.5" pos="22 10 1.5" group="1" conaffinity="1"/>
    <geom name="pipe1" type="cylinder" size="0.15 1.5" pos="18 15 1.5" group="1" conaffinity="1"/>
  </worldbody>
</mujoco>
"""

# ── LiDAR 参数 ──────────────────────────────────────────────────
N_RAYS = 6400
GOLDEN_ANG = np.pi * (3 - np.sqrt(5))
RANGE_MAX = 30.0

# 场景边界裁剪 (建筑外轮廓 + 2m 余量)
CLIP_X = (-2, 27)
CLIP_Y = (-2, 20)
CLIP_Z = (-0.5, 5.0)


def build_ray_dirs(n, vfov_min_deg=-30, vfov_max_deg=60):
    """全球面 golden-angle 扫描方向"""
    i = np.arange(n, dtype=np.float64)
    ha = (i * GOLDEN_ANG) % (2 * np.pi)
    vfov_min = np.deg2rad(vfov_min_deg)
    vfov_max = np.deg2rad(vfov_max_deg)
    va = vfov_min + i / n * (vfov_max - vfov_min)
    cv = np.cos(va)
    return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])


def scan_at(model, data, pos, ray_dirs, geomgroup):
    """在给定位置做一次全方位 LiDAR 扫描，返回命中点 (world frame)"""
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
    xml = SCENE_XML
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'maps', 'building_nav.pcd')

    if len(sys.argv) > 1:
        with open(sys.argv[1]) as f:
            xml = f.read()
    if len(sys.argv) > 2:
        out_path = sys.argv[2]

    print("Loading MuJoCo scene...")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)

    ray_dirs = build_ray_dirs(N_RAYS, vfov_min_deg=-30, vfov_max_deg=60)
    geomgroup = np.zeros(6, dtype=np.uint8)
    geomgroup[1] = 1  # only scan environment geoms (group=1)

    # 在场景中均匀采样扫描位置
    # 场景范围: x=[0,25], y=[0,18]
    scan_xs = np.arange(1, 25, 2.0)
    scan_ys = np.arange(1, 18, 2.0)
    scan_z = 0.7  # 扫描高度 (机器人 LiDAR 高度)

    all_pts = []
    n_scans = 0
    for x in scan_xs:
        for y in scan_ys:
            pts = scan_at(model, data, [x, y, scan_z], ray_dirs, geomgroup)
            if len(pts) > 0:
                all_pts.append(pts)
                n_scans += 1

    print(f"Scanned {n_scans} positions")

    if not all_pts:
        print("ERROR: No points captured!")
        sys.exit(1)

    cloud = np.concatenate(all_pts, axis=0)
    print(f"Raw points: {len(cloud)}")

    # 场景边界裁剪 — 去除穿过门洞打到建筑外地面的远处噪声点
    mask = (
        (cloud[:, 0] >= CLIP_X[0]) & (cloud[:, 0] <= CLIP_X[1]) &
        (cloud[:, 1] >= CLIP_Y[0]) & (cloud[:, 1] <= CLIP_Y[1]) &
        (cloud[:, 2] >= CLIP_Z[0]) & (cloud[:, 2] <= CLIP_Z[1])
    )
    cloud = cloud[mask]
    print(f"After bbox clip: {len(cloud)} points")

    # 体素降采样
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.astype(np.float64))
    pcd = pcd.voxel_down_sample(0.1)
    print(f"After voxel downsample (0.1m): {len(pcd.points)} points")

    # 保存
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    o3d.io.write_point_cloud(out_path, pcd)
    print(f"Saved: {out_path}")

    pts = np.asarray(pcd.points)
    print(f"Bounds: x=[{pts[:,0].min():.1f}, {pts[:,0].max():.1f}], "
          f"y=[{pts[:,1].min():.1f}, {pts[:,1].max():.1f}], "
          f"z=[{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]")


if __name__ == '__main__':
    main()
