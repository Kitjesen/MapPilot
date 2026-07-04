#!/usr/bin/env python3
"""
寤虹瓚2_9 鍏ㄦ爤瀵艰埅 鈥?璁烘枃鍙鍖栧浘鐢熸垚鍣?Generates publication-quality figures for the advisor demo:
  1. 3D building cloud + navigation trajectory (perspective)
  2. Top-down floor plan + trajectory heatmap
  3. Velocity time series (vx, wz, |v|) with event markers
  4. 4-panel dashboard for advisor presentation

Usage:
  python3 e2e_generate_viz.py [--result /tmp/sim_result.json] [--pcd /path/to/building2_9.pcd]
"""
import argparse
import json
import os
import sys
import math
from typing import Optional, Dict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable, get_cmap

# mpl_toolkits.mplot3d may conflict if both system and pip matplotlib are installed
try:
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
    _HAS_3D = True
except (ImportError, AttributeError):
    _HAS_3D = False

# 鈹€鈹€ 榛樿璺緞 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
_OCTOPLANNER3D_BUILDING2_9_PCD = os.path.join(
    _REPO_ROOT,
    'src', 'nav', 'services', 'plan', 'global_planner', 'algorithm',
    'OctoPlanner3D', 'octomap', 'pcd_files', 'building2_9.pcd',
)

_PCD_CANDIDATES = [
    os.environ.get('SIM_PCD_PATH', ''),
    _OCTOPLANNER3D_BUILDING2_9_PCD,
    '/home/sunrise/data/SLAM/navigation/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/octomap/pcd_files/building2_9.pcd',
    '/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/rsc/pcd/building2_9.pcd',
    '/home/sunrise/data/SLAM/navigation/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/pcd/building2_9.pcd',
]
_RESULT_DEFAULT = '/tmp/sim_result.json'

# 鈹€鈹€ 鏍峰紡 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
plt.rcParams.update({
    'font.family':      'DejaVu Sans',
    'font.size':        11,
    'axes.labelsize':   12,
    'axes.titlesize':   13,
    'legend.fontsize':  10,
    'figure.dpi':       300,
    'lines.linewidth':  2.0,
})

CMAP_CLOUD = 'rainbow'     # building Z-height coloring
CMAP_TRAJ  = 'plasma'      # trajectory time coloring


# 鈹€鈹€ 鏁版嵁鍔犺浇 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def find_pcd() -> Optional[str]:
    for p in _PCD_CANDIDATES:
        if p and os.path.exists(p):
            return p
    return None


def load_pcd(pcd_path: str, max_pts: int = 120000) -> Optional[np.ndarray]:
    """Return (N,3) float32 [x,y,z]."""
    try:
        with open(pcd_path, 'rb') as f:
            raw = f.read()
        pos, meta = 0, {}
        while pos < len(raw):
            end = raw.find(b'\n', pos)
            if end == -1:
                break
            line = raw[pos:end].decode('ascii', errors='ignore').strip()
            parts = line.split()
            if parts:
                meta[parts[0].upper()] = parts[1:]
            pos = end + 1
            if parts and parts[0].upper() == 'DATA':
                break
        data_offset = pos
        data_type = meta.get('DATA', ['ascii'])[0].lower()
        n_pts  = int(meta.get('POINTS', [0])[0])
        fields = meta.get('FIELDS', [])
        types  = meta.get('TYPE', [])
        if data_type != 'binary' or 'x' not in fields:
            return None
        np_map = {'F': 'f4', 'I': 'i4', 'U': 'u4'}
        dt = np.dtype([(f, np_map.get(t, 'f4')) for f, t in zip(fields, types)])
        arr = np.frombuffer(raw[data_offset: data_offset + n_pts * dt.itemsize], dtype=dt)
        pts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)
        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]
        return pts
    except Exception as e:
        print(f'[warn] PCD load failed: {e}')
        return None


def load_result(path: str) -> Optional[Dict]:
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


# 鈹€鈹€ Figure 1 鈥?3D 閫忚鍥?(寤虹瓚鐐逛簯 + 杞ㄨ抗) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def _fig_3d_fallback(pts: np.ndarray, result: Dict, out_path: str):
    """Fallback when Axes3D unavailable: XZ + XY side-by-side pseudo-3D."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    z_min, z_max = float(pts[:, 2].min()), float(pts[:, 2].max())
    norm_z = Normalize(vmin=z_min, vmax=z_max)
    traj = result['trajectory']
    txs = np.array([p['x'] for p in traj]); tys = np.array([p['y'] for p in traj])

    # Left: X-Y floor plan
    floor = pts[pts[:, 2] < 0.5]; wall = pts[(pts[:, 2] >= 0.5) & (pts[:, 2] < 3.0)]
    ax1.scatter(floor[:, 0], floor[:, 1], s=0.3, c='#cccccc', alpha=0.2, rasterized=True)
    ax1.scatter(wall[:, 0],  wall[:, 1],  s=0.6, c='#888888', alpha=0.5, rasterized=True)
    ax1.plot(txs, tys, 'b-', lw=3, label='Trajectory')
    ax1.plot(*result['start'], 'g^', ms=12, label='Start'); ax1.plot(*result['goal'], 'r*', ms=16, label='Goal')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_title('(a) Top-Down View')
    ax1.legend(fontsize=9); ax1.set_aspect('equal'); ax1.grid(True, alpha=0.3)

    # Right: X-Z elevation (side view)
    cols = get_cmap(CMAP_CLOUD)(norm_z(pts[:, 2]))
    ax2.scatter(pts[:, 0], pts[:, 2], s=0.3, c=cols, alpha=0.25, rasterized=True)
    ax2.axhline(0, color='gray', ls='--', lw=1); ax2.axhline(3.5, color='orange', ls='--', lw=1)
    ax2.text(-7.5, 0.2, 'Ground Floor', color='lime', fontsize=9)
    ax2.text(-7.5, 3.7, '2nd Floor',    color='orange', fontsize=9)
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Z Height (m)')
    ax2.set_title(f'(b) Side Elevation 鈥?{z_min:.1f}m~{z_max:.1f}m')
    sm = ScalarMappable(cmap=CMAP_CLOUD, norm=norm_z); sm.set_array([])
    fig.colorbar(sm, ax=ax2, label='Height Z (m)')

    fig.suptitle('Building2_9 3D Environment + Navigation Trajectory', fontsize=13)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'[fig1-fallback] saved: {out_path}')

def fig_3d_perspective(pts: np.ndarray, result: Dict, out_path: str,
                       elev: float = 32, azim: float = -55):
    """3D scatter: building colored by Z, trajectory colored by time."""
    if not _HAS_3D:
        # Fallback: 2D perspective using scatter plot
        _fig_3d_fallback(pts, result, out_path)
        return
    fig = plt.figure(figsize=(14, 9))
    ax  = fig.add_subplot(111, projection='3d')

    z_min, z_max = float(pts[:, 2].min()), float(pts[:, 2].max())
    norm_bld = Normalize(vmin=z_min, vmax=z_max)

    # 鍒嗗眰閫忔槑搴? 鍦伴潰杞?鈫?澧欏閲?鈫?涓婂眰杞?    floor_m = pts[:, 2] < 0.3
    wall1_m = (pts[:, 2] >= 0.3) & (pts[:, 2] < 2.5)
    wall2_m = (pts[:, 2] >= 2.5) & (pts[:, 2] < 4.5)
    upper_m = pts[:, 2] >= 4.5

    for mask, alpha, sz in [(floor_m, 0.12, 0.3), (wall1_m, 0.45, 0.5),
                             (wall2_m, 0.35, 0.4), (upper_m, 0.25, 0.35)]:
        if mask.sum() == 0:
            continue
        cols = get_cmap(CMAP_CLOUD)(norm_bld(pts[mask, 2]))
        ax.scatter(pts[mask, 0], pts[mask, 1], pts[mask, 2],
                   c=cols, s=sz, alpha=alpha, rasterized=True)

    # 瀵艰埅杞ㄨ抗
    traj = result['trajectory']
    ts   = np.array([p['t'] for p in traj])
    txs  = np.array([p['x'] for p in traj])
    tys  = np.array([p['y'] for p in traj])
    tzs  = np.zeros_like(txs)

    norm_t = Normalize(vmin=ts.min(), vmax=ts.max())
    for i in range(len(txs) - 1):
        c = get_cmap(CMAP_TRAJ)(norm_t(ts[i]))
        ax.plot([txs[i], txs[i+1]], [tys[i], tys[i+1]], [0, 0],
                color=c, linewidth=4, alpha=0.95, solid_capstyle='round')

    sx, sy = result['start']
    gx, gy = result['goal']
    ax.scatter([sx], [sy], [0], c='lime',   s=250, marker='^', zorder=10, depthshade=False)
    ax.scatter([gx], [gy], [0], c='red',    s=300, marker='*', zorder=10, depthshade=False)
    ax.text(sx, sy, 0.3, ' Start', color='lime',  fontsize=9, fontweight='bold')
    ax.text(gx, gy, 0.3, ' Goal',  color='red',   fontsize=9, fontweight='bold')

    # 鑹叉潯 鈥?寤虹瓚楂樺害
    sm_z = ScalarMappable(cmap=CMAP_CLOUD, norm=norm_bld)
    sm_z.set_array([])
    cbar = fig.colorbar(sm_z, ax=ax, pad=0.12, shrink=0.55, aspect=20, location='right')
    cbar.set_label('Building Height Z (m)', fontsize=10)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_xlim(-8, 11); ax.set_ylim(-10, 9); ax.set_zlim(0, 8)
    status = '鉁?Goal Reached' if result['goal_reached'] else '鉁?Timeout'
    ax.set_title(
        f'Building2_9 鈥?3D Environment + Navigation Trajectory  ({status})\n'
        f'Blue鈫扲ed: floor鈫抍eiling (Z: {z_min:.1f}~{z_max:.1f}m) | '
        f'Purple鈫扽ellow: trajectory start鈫抏nd',
        pad=10, fontsize=11,
    )
    ax.view_init(elev=elev, azim=azim)
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'[fig1] 3D perspective saved: {out_path}')


# 鈹€鈹€ Figure 2 鈥?淇鍦板浘 + 杞ㄨ抗鐑浘 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def fig_topdown(pts: np.ndarray, result: Dict, out_path: str):
    """Top-down view: floor-level point cloud + trajectory colored by speed."""
    fig, ax = plt.subplots(figsize=(11, 9), facecolor='#12121f')
    ax.set_facecolor('#12121f')

    floor = pts[pts[:, 2] < 0.5]
    wall  = pts[(pts[:, 2] >= 0.5) & (pts[:, 2] < 3.0)]

    ax.scatter(floor[:, 0], floor[:, 1], s=0.2, c='#2a3a5b', alpha=0.35, rasterized=True)
    ax.scatter(wall[:,  0], wall[:,  1],  s=0.5, c='#5a8aab', alpha=0.65, rasterized=True,
               label='Wall cross-section (0.5鈥?.0m)')

    traj = result['trajectory']
    txs  = np.array([p['x']  for p in traj])
    tys  = np.array([p['y']  for p in traj])
    vs   = np.array([abs(p['vx']) for p in traj])

    norm_v = Normalize(vmin=0, vmax=vs.max() + 0.01)
    for i in range(len(txs) - 1):
        c = get_cmap('hot')(norm_v((vs[i] + vs[i+1]) / 2))
        ax.plot([txs[i], txs[i+1]], [tys[i], tys[i+1]],
                color=c, linewidth=5, alpha=0.92, solid_capstyle='round')

    sx, sy = result['start']
    gx, gy = result['goal']
    ax.plot(sx, sy, 'g^', ms=14, mec='white', mew=1.5, label=f'Start ({sx},{sy})', zorder=10)
    ax.plot(gx, gy, 'r*', ms=18, mec='white', mew=1.5, label=f'Goal ({gx},{gy})',  zorder=10)
    ax.plot(txs[-1], tys[-1], 'cs', ms=10, mec='white', mew=1.0, label='Final pos', zorder=9)

    sm_v = ScalarMappable(cmap='hot', norm=norm_v)
    sm_v.set_array([])
    cbar = fig.colorbar(sm_v, ax=ax, pad=0.02)
    cbar.set_label('Forward Speed |vx| (m/s)', color='white', fontsize=10)
    cbar.ax.yaxis.set_tick_params(color='white')
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color='white')

    ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')
    for sp in ax.spines.values():
        sp.set_edgecolor('#5a8aab')
    status = '鉁?Goal Reached' if result['goal_reached'] else '鉁?Timeout'
    ax.set_title(f'Building2_9 鈥?Floor Plan Navigation  ({status})\n'
                 f'Trajectory colored by speed (blue wall cross-sections visible)',
                 color='white', pad=12)
    ax.legend(loc='lower right', fontsize=9,
              facecolor='#1a1a2e', edgecolor='#5a8aab', labelcolor='white')
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='#12121f')
    plt.close(fig)
    print(f'[fig2] Top-down saved: {out_path}')


# 鈹€鈹€ Figure 3 鈥?閫熷害鏃跺簭鍥?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def fig_velocity(result: Dict, out_path: str):
    """Speed profile: vx, wz, and distance-to-goal."""
    traj = result['trajectory']
    ts   = np.array([p['t']    for p in traj])
    vxs  = np.array([p['vx']   for p in traj])
    wzs  = np.array([p['wz']   for p in traj])
    ds   = np.array([p['dist'] for p in traj])

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(
        'MapPilot 鈥?Navigation Velocity Profile & Convergence\n'
        'Full Stack: PCT A* 鈫?pct_path_adapter (8 waypoints) 鈫?localPlanner 鈫?pathFollower',
        fontsize=12, y=1.01,
    )

    # Panel 1: linear speed
    ax = axes[0]
    ax.plot(ts, vxs, color='#1e90ff', lw=2.5, label='vx 鈥?Forward speed (m/s)')
    ax.fill_between(ts, 0, vxs, where=(vxs > 0), alpha=0.18, color='#1e90ff')
    ax.axhline(0.8, color='#ff6600', ls=':', lw=1.5, alpha=0.7, label='max_speed = 0.8 m/s')
    ax.set_ylabel('Forward Speed vx (m/s)')
    ax.set_ylim(-0.05, 1.05)
    ax.legend(loc='lower right', framealpha=0.85)
    ax.grid(True, alpha=0.3)
    ax.set_title('(a) Forward Velocity', loc='left', fontsize=10)

    # Panel 2: angular rate
    ax2 = axes[1]
    ax2.plot(ts, wzs, color='#ff8c00', lw=2.2, label='蠅z 鈥?Angular rate (rad/s)')
    ax2.fill_between(ts, 0, wzs, where=(wzs > 0), alpha=0.2, color='#ff8c00', label='Left turn')
    ax2.fill_between(ts, wzs, 0, where=(wzs < 0), alpha=0.2, color='#9b59b6', label='Right turn')
    ax2.axhline(0, color='gray', lw=0.8)
    ax2.set_ylabel('Angular Rate 蠅z (rad/s)')
    ax2.legend(loc='upper right', framealpha=0.85)
    ax2.grid(True, alpha=0.3)
    ax2.set_title('(b) Angular Velocity (Turning)', loc='left', fontsize=10)

    # Panel 3: distance to goal
    ax3 = axes[2]
    ax3.plot(ts, ds, color='#e74c3c', lw=2.5, label='Distance to goal (m)')
    ax3.fill_between(ts, 0, ds, alpha=0.14, color='#e74c3c')
    ax3.axhline(0.5, color='#27ae60', ls='--', lw=1.8, label='Goal threshold = 0.5m')
    if result['goal_reached']:
        ax3.axvline(ts[-1], color='#2ecc71', ls='--', lw=1.5, alpha=0.9)
        ax3.annotate(f'鉁?Reached\nt = {ts[-1]:.1f}s',
                     xy=(ts[-1], 0.5), xytext=(ts[-1] - 2.5, ds.max() * 0.7),
                     arrowprops=dict(arrowstyle='->', color='#2ecc71'),
                     color='#2ecc71', fontsize=10, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance to Goal (m)')
    ax3.set_ylim(bottom=0)
    ax3.legend(loc='upper right', framealpha=0.85)
    ax3.grid(True, alpha=0.3)
    ax3.set_title('(c) Navigation Convergence (Distance to Goal)', loc='left', fontsize=10)

    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'[fig3] Velocity profile saved: {out_path}')


# 鈹€鈹€ Figure 4 鈥?缁煎悎 dashboard 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def fig_dashboard(pts: np.ndarray, result: Dict, out_path: str):
    """4-panel publication dashboard."""
    traj = result['trajectory']
    ts   = np.array([p['t']    for p in traj])
    txs  = np.array([p['x']    for p in traj])
    tys  = np.array([p['y']    for p in traj])
    vxs  = np.array([p['vx']   for p in traj])
    wzs  = np.array([p['wz']   for p in traj])
    ds   = np.array([p['dist'] for p in traj])

    fig = plt.figure(figsize=(16, 12))
    gs  = gridspec.GridSpec(2, 2, hspace=0.42, wspace=0.35)

    # 鈹€鈹€ Panel A: 淇杞ㄨ抗 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    ax_a = fig.add_subplot(gs[0, 0])
    floor = pts[pts[:, 2] < 0.5]
    wall  = pts[(pts[:, 2] >= 0.5) & (pts[:, 2] < 2.5)]
    ax_a.scatter(floor[:, 0], floor[:, 1], s=0.3, c='#cccccc', alpha=0.2, rasterized=True)
    ax_a.scatter(wall[:,  0], wall[:,  1], s=0.6, c='#888888', alpha=0.5, rasterized=True)
    norm_t = Normalize(vmin=ts.min(), vmax=ts.max())
    for i in range(len(txs) - 1):
        c = get_cmap(CMAP_TRAJ)(norm_t(ts[i]))
        ax_a.plot([txs[i], txs[i+1]], [tys[i], tys[i+1]],
                  color=c, lw=3.5, alpha=0.95, solid_capstyle='round')
    sx, sy = result['start']
    gx, gy = result['goal']
    ax_a.plot(sx, sy, 'g^', ms=10, mec='k', mew=0.8, label='Start', zorder=10)
    ax_a.plot(gx, gy, 'r*', ms=14, mec='k', mew=0.8, label='Goal',  zorder=10)
    ax_a.set_xlabel('X (m)'); ax_a.set_ylabel('Y (m)')
    ax_a.set_title('(A) Top-Down Trajectory (time-colored)')
    ax_a.legend(fontsize=9); ax_a.set_aspect('equal'); ax_a.grid(True, alpha=0.25)
    sm = ScalarMappable(cmap=CMAP_TRAJ, norm=norm_t); sm.set_array([])
    fig.colorbar(sm, ax=ax_a, label='Time (s)', shrink=0.85)

    # 鈹€鈹€ Panel B: 3D 閫忚 or side elevation 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    z_min, z_max = float(pts[:, 2].min()), float(pts[:, 2].max())
    if _HAS_3D:
        ax_b = fig.add_subplot(gs[0, 1], projection='3d')
        sample_idx = np.random.choice(len(pts), min(55000, len(pts)), replace=False)
        sample = pts[sample_idx]
        norm_z = Normalize(vmin=z_min, vmax=z_max)
        cols_z = get_cmap(CMAP_CLOUD)(norm_z(sample[:, 2]))
        ax_b.scatter(sample[:, 0], sample[:, 1], sample[:, 2],
                     c=cols_z, s=0.4, alpha=0.22, rasterized=True)
        for i in range(len(txs) - 1):
            c = get_cmap(CMAP_TRAJ)(norm_t(ts[i]))
            ax_b.plot([txs[i], txs[i+1]], [tys[i], tys[i+1]], [0, 0], color=c, lw=3, alpha=0.9)
        ax_b.scatter([sx], [sy], [0], c='lime', s=100, marker='^', zorder=5, depthshade=False)
        ax_b.scatter([gx], [gy], [0], c='red',  s=150, marker='*', zorder=5, depthshade=False)
        ax_b.set_xlabel('X'); ax_b.set_ylabel('Y'); ax_b.set_zlabel('Z (m)')
        ax_b.view_init(elev=28, azim=-55)
    else:
        ax_b = fig.add_subplot(gs[0, 1])
        # X-Z side elevation view
        norm_z = Normalize(vmin=z_min, vmax=z_max)
        ax_b.scatter(pts[:, 0], pts[:, 2], s=0.3, c=get_cmap(CMAP_CLOUD)(norm_z(pts[:, 2])),
                     alpha=0.25, rasterized=True)
        ax_b.set_xlabel('X (m)'); ax_b.set_ylabel('Z (m)')
        ax_b.annotate('鈫?2nd floor', xy=(0, 3.5), fontsize=9, color='orange')
        ax_b.annotate('鈫?Ground floor', xy=(0, 0.3), fontsize=9, color='lime')
    ax_b.set_title(f'(B) Building Profile (Z鈭圼{z_min:.1f},{z_max:.1f}]m)')

    # 鈹€鈹€ Panel C: 閫熷害鏃跺簭 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    ax_c = fig.add_subplot(gs[1, 0])
    color_vx = '#1e90ff'; color_wz = '#ff8c00'
    ax_c.plot(ts, vxs, color=color_vx, lw=2.5, label='vx (m/s)')
    ax_c.fill_between(ts, 0, vxs, alpha=0.15, color=color_vx)
    ax_c.axhline(0.8, color='gray', ls=':', lw=1.0, alpha=0.6)
    ax_c2 = ax_c.twinx()
    ax_c2.plot(ts, wzs, color=color_wz, lw=2.0, ls='--', alpha=0.85, label='蠅z (rad/s)')
    ax_c2.set_ylabel('蠅z (rad/s)', color=color_wz)
    ax_c2.tick_params(axis='y', labelcolor=color_wz)
    ax_c.set_xlabel('Time (s)'); ax_c.set_ylabel('vx (m/s)', color=color_vx)
    ax_c.tick_params(axis='y', labelcolor=color_vx)
    ax_c.set_title('(C) Velocity Profile')
    ax_c.grid(True, alpha=0.3)
    l1, lb1 = ax_c.get_legend_handles_labels()
    l2, lb2 = ax_c2.get_legend_handles_labels()
    ax_c.legend(l1 + l2, lb1 + lb2, loc='lower right', fontsize=9)

    # 鈹€鈹€ Panel D: 鏀舵暃鏇茬嚎 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    ax_d = fig.add_subplot(gs[1, 1])
    ax_d.plot(ts, ds, color='#e74c3c', lw=2.5, label='Distance to goal')
    ax_d.fill_between(ts, 0, ds, alpha=0.12, color='#e74c3c')
    ax_d.axhline(0.5, color='#27ae60', ls='--', lw=1.8, label='Threshold 0.5m')
    if result['goal_reached']:
        ax_d.axvline(ts[-1], color='#2ecc71', ls='--', lw=1.5, alpha=0.8)
        ax_d.text(ts[-1] - 0.8, ds.max() * 0.75,
                  f'鉁?{ts[-1]:.1f}s', color='#2ecc71',
                  fontsize=11, ha='right', fontweight='bold')
    ax_d.set_xlabel('Time (s)'); ax_d.set_ylabel('Distance (m)')
    ax_d.set_title('(D) Navigation Convergence')
    ax_d.legend(fontsize=9); ax_d.grid(True, alpha=0.3); ax_d.set_ylim(bottom=0)

    # 鈹€鈹€ 鎬绘爣棰?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    status = '鉁?GOAL REACHED' if result['goal_reached'] else '鉁?TIMEOUT'
    fig.suptitle(
        f'MapPilot (鐏甸€? 鈥?Building2_9 Full-Stack Navigation SITL\n'
        f'PCT A* Global Planner 鈫?pct_path_adapter 鈫?localPlanner 鈫?pathFollower 鈫?sim_robot\n'
        f'{status}  |  Time: {ts[-1]:.1f}s  |  Path: (-5.5,7.3)鈫?5.0,7.3)  |  8 Waypoints  |  10.6m',
        fontsize=11, y=1.008,
    )
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'[fig4] Dashboard saved: {out_path}')


# 鈹€鈹€ main 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--result', default=_RESULT_DEFAULT)
    ap.add_argument('--pcd',    default=None)
    ap.add_argument('--outdir', default='/tmp/nav_figures')
    args = ap.parse_args()

    result = load_result(args.result)
    if result is None:
        print(f'[error] Result not found: {args.result}', file=sys.stderr)
        sys.exit(1)
    print(f'Loaded trajectory: {len(result["trajectory"])} points, '
          f'goal_reached={result["goal_reached"]}')

    pcd_path = args.pcd or find_pcd()
    pts = None
    if pcd_path:
        print(f'Loading PCD: {pcd_path} ...')
        pts = load_pcd(pcd_path, max_pts=120000)
        if pts is not None:
            print(f'PCD loaded: {len(pts)} pts, '
                  f'Z鈭圼{pts[:,2].min():.2f},{pts[:,2].max():.2f}]m')
    if pts is None:
        print('[warn] No PCD 鈥?using empty cloud')
        pts = np.zeros((100, 3), dtype=np.float32)

    out = args.outdir
    os.makedirs(out, exist_ok=True)

    fig_3d_perspective(pts, result, f'{out}/paper_fig1_3d.png')
    fig_topdown(pts, result, f'{out}/paper_fig2_topdown.png')
    fig_velocity(result, f'{out}/paper_fig3_velocity.png')
    fig_dashboard(pts, result, f'{out}/paper_fig4_dashboard.png')

    print(f'\n[done] 4 figures saved to {out}/')


if __name__ == '__main__':
    main()
