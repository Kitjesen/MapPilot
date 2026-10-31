"""Compare robot collision shapes and heading search on explicit offline fixtures.

This pose-geometry experiment does not run OctoPlanner, SCAN, locomotion or
ground-support inference. Product collision settings remain unchanged.
"""
from __future__ import annotations

import argparse
import heapq
import html
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from itertools import pairwise
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.bench.body_model import BodyModel

URDF = ROOT / 'web/public/assets/robots/go2/urdf/go2_mid360_flatpack.urdf'
LABELS = {'circle': 'XYZ: all-heading circle', 'twin': 'XY + yaw: twin cylinders',
          'urdf': 'XY + yaw: URDF collision'}


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def proxy(path, radius, offset, above):
    robot = ET.Element('robot', name=path.stem)
    base = ET.SubElement(robot, 'link', name='base')
    for x in ((0,) if offset == 0 else (-offset, offset)):
        node = ET.SubElement(base, 'collision')
        ET.SubElement(node, 'origin', xyz=f'{x} 0 {(above - .10) / 2}')
        ET.SubElement(ET.SubElement(node, 'geometry'), 'cylinder',
                      radius=str(radius), length=str(above + .10))
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)


def vertices(geom):
    """Sample visual outlines only; collision uses MuJoCo's original primitives."""
    size = np.asarray(geom['size'])
    kind = geom['type']
    theta = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    if kind == 'box':
        points = np.array([[x, y, z] for x in (-size[0], size[0])
                           for y in (-size[1], size[1]) for z in (-size[2], size[2])])
    elif kind == 'cylinder':
        points = np.array([[size[0] * math.cos(a), size[0] * math.sin(a), z]
                           for a in theta for z in (-size[1], size[1])])
    elif kind == 'sphere':
        points = np.array([[size[0]*math.sin(p)*math.cos(a), size[0]*math.sin(p)*math.sin(a),
                            size[0]*math.cos(p)] for p in np.linspace(0, math.pi, 16) for a in theta])
    else:
        raise ValueError(f'No visualization for imported collision type {kind}')
    return points @ np.asarray(geom['rotation']).T + np.asarray(geom['center'])


def bounds(geometry):
    """Exact axis-aligned bounds of the imported sphere/box/cylinder geometry."""
    lows, highs = [], []
    for g in geometry:
        r, s = np.asarray(g['rotation']), np.asarray(g['size'])
        if g['type'] == 'box':
            extent = np.abs(r) @ s[:3]
        elif g['type'] == 'sphere':
            extent = np.repeat(s[0], 3)
        elif g['type'] == 'cylinder':
            extent = s[0] * np.sqrt(np.maximum(0, 1-r[:, 2]**2)) + s[1] * np.abs(r[:, 2])
        else:
            raise ValueError(g['type'])
        lows.append(np.asarray(g['center'])-extent)
        highs.append(np.asarray(g['center'])+extent)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def walls(width, doorway=False):
    if doorway:
        return [dict(name=f'jamb_{sign}', center=[0, sign*(width/2+1), .75],
                     half_size=[.10, 1., .75]) for sign in (-1, 1)]
    return [dict(name=f'wall_{sign}', center=[0, sign*(width/2+.1), .75],
                 half_size=[1.5, .1, .75]) for sign in (-1, 1)]


def box_witness(geom, obstacle):
    """Independently check an oriented box against the retained voxel by SAT."""
    rotation = np.asarray(geom['rotation'])
    axes = [*rotation.T, *np.eye(3)]
    axes += [np.cross(a,b) for a in rotation.T for b in np.eye(3)]
    separation = np.asarray(obstacle['center'])-np.asarray(geom['center'])
    overlaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-10:
            continue
        axis = axis/norm
        support = (np.abs(rotation.T@axis)@np.asarray(geom['size']) +
                   np.abs(axis)@np.asarray(obstacle['half_size']))
        overlaps.append(float(support-abs(separation@axis)))
    return dict(intersects=min(overlaps)>0, minimum_axis_overlap_m=min(overlaps),
                axes_checked=len(overlaps), method='separating axis test, not MuJoCo distance')


def sweep(model, start, end, margin, radius):
    """Bound the displacement between samples, including the rotating body arc."""
    a, b = np.asarray(start), np.asarray(end)
    delta = b-a
    delta[3] = (delta[3]+math.pi) % (2*math.pi)-math.pi
    distance_bound = float(np.linalg.norm(delta[:3]) + radius * abs(delta[3]))
    steps = max(1, math.ceil(distance_bound/.02))
    pad = distance_bound/(2*steps)
    minimum = math.inf
    for i in range(steps+1):
        clearance = model.clearance((a+delta*i/steps).tolist(), cutoff=.5)[0]
        minimum = min(minimum, clearance)
        if clearance <= margin + pad:
            return False, minimum
    return True, minimum


def search(model, start, goal, margin, radius, with_heading):
    """A bounded flat-floor pose search, with full 3D shape queries at each edge."""
    step, angles = .1, 16
    turn = 2*math.pi/angles
    initial = (round(start[0]/step), round(start[1]/step), round(start[3]/turn) % angles)
    target = (round(goal[0]/step), round(goal[1]/step), round(goal[3]/turn) % angles)
    z = start[2]

    def pose(state):
        return [state[0]*step, state[1]*step, z, state[2]*turn]

    def h(state):
        heading = abs((state[2]-target[2]+angles//2) % angles-angles//2)
        return step*math.hypot(state[0]-target[0], state[1]-target[1]) + (.12*heading if with_heading else 0)

    started = time.perf_counter()
    if model.clearance(start, cutoff=.5)[0] <= margin:
        return dict(ok=False, reason='start_collision_or_clearance', expanded=0, poses=[], elapsed_s=time.perf_counter()-started)
    queue, cost, parent, checked = [(h(initial), 0., initial)], {initial: 0.}, {}, {}
    expanded = 0
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost > cost[current] + 1e-10:
            continue
        expanded += 1
        reached = current[:2] == target[:2] and (not with_heading or current[2] == target[2])
        if reached:
            states = [current]
            while states[-1] in parent:
                states.append(parent[states[-1]])
            poses = [pose(s) for s in reversed(states)]
            return dict(ok=True, reason='pose_path_found', expanded=expanded, poses=poses,
                        elapsed_s=time.perf_counter()-started,
                        translation_m=sum(math.dist(a[:3], b[:3]) for a, b in pairwise(poses)))
        neighbors = [((current[0]+dx, current[1]+dy, current[2]), step*math.hypot(dx,dy))
                     for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))]
        if with_heading:
            neighbors += [((current[0],current[1],(current[2]+d) % angles),.12) for d in (-1,1)]
        for candidate, increment in neighbors:
            if abs(candidate[0]) > 14 or abs(candidate[1]) > 10:
                continue
            value = current_cost+increment
            if value >= cost.get(candidate, math.inf)-1e-10:
                continue
            key = (current, candidate)
            if key not in checked:
                checked[key] = sweep(model, pose(current), pose(candidate), margin, radius)[0]
            if not checked[key]:
                continue
            cost[candidate], parent[candidate] = value, current
            heapq.heappush(queue, (value+h(candidate), value, candidate))
    return dict(ok=False, reason='pose_search_exhausted', expanded=expanded, poses=[], elapsed_s=time.perf_counter()-started)


def run(out):
    out.mkdir(parents=True, exist_ok=False)
    proxy(out/'circle.urdf', .44, 0, .25)
    proxy(out/'twin.urdf', .25, .19, .24)
    paths = {'circle': out/'circle.urdf', 'twin': out/'twin.urdf', 'urdf': URDF}
    profiles = {}
    for name,path in paths.items():
        m = BodyModel(path, [])
        geometry = m.geometry([0,0,0,0])
        low, high = bounds(geometry)
        radius = float(math.hypot(max(abs(low[0]),abs(high[0])), max(abs(low[1]),abs(high[1]))))
        profiles[name] = dict(source=str(path), model_report=m.report, geometry=geometry, low=low.tolist(), high=high.tolist(),
                              full_size=(high-low).tolist(), radius_bound=radius,
                              margin_m=.02 if name == 'urdf' else 0.)
    scenes = [dict(name='door_060', label='0.60 m doorway', obstacles=walls(.60,True),
                   start=[-1.2,0,.35,math.pi/2], goal=[1.2,0,.35,0]),
              dict(name='corridor_045', label='0.45 m corridor', obstacles=walls(.45),
                   start=[-.9,0,.35,0], goal=[.9,0,.35,0]),
              dict(name='corridor_028', label='0.28 m corridor', obstacles=walls(.28),
                   start=[-.9,0,.35,0], goal=[.9,0,.35,0])]
    for scene in scenes:
        scene['results'] = {}
        for name,path in paths.items():
            model = BodyModel(path,scene['obstacles'])
            p = profiles[name]
            scene['results'][name] = search(model,scene['start'],scene['goal'],p['margin_m'],p['radius_bound'],name!='circle')
            scene['results'][name]['oracle'] = model.report
            print(scene['name'], name, scene['results'][name]['reason'], flush=True)
        write(out/f"{scene['name']}.json", scene)
    turn_obstacles = walls(.60)
    rotation = {'obstacles': turn_obstacles, 'profiles': {}}
    for name,path in paths.items():
        model, p = BodyModel(path,turn_obstacles), profiles[name]
        a,b = [0,0,.35,0], [0,0,.35,math.pi]
        rotation['profiles'][name] = dict(start_clearance_m=model.clearance(a)[0],
            end_clearance_m=model.clearance(b)[0], sweep_ok=sweep(model,a,b,p['margin_m'],p['radius_bound'])[0],
            samples=[dict(yaw_deg=deg,clearance_m=model.clearance([0,0,.35,math.radians(deg)])[0]) for deg in range(0,181,5)])
    garage_pose = [3.675405740737915,-.20373751223087311,-1.1807905435562134,-.090659887607880671]
    witness = [dict(name='original_occupied_leaf', center=[3.45,-.15,-1.25],half_size=[.05,.05,.05])]
    garage = {'scope':'one original occupied leaf only; no full-map or support qualification',
              'pose':garage_pose, 'obstacles':witness, 'profiles':{}}
    for name,path in paths.items():
        model = BodyModel(path,witness)
        clearance,link,obstacle = model.clearance(garage_pose)
        garage['profiles'][name] = dict(clearance_m=clearance,nearest_link=link,obstacle=obstacle,
                                        accepted_with_profile_margin=clearance>profiles[name]['margin_m'])
        if name == 'urdf':
            base = next(g for g in model.geometry(garage_pose) if g['link']=='base')
            garage['base_box_independent_check'] = box_witness(base,witness[0])
    spread = {f'{leg}_hip_joint': sign*.25 for leg,sign in [('FL',1),('RL',1),('FR',-1),('RR',-1)]}
    # Explicit posture probe, not a recorded or generated gait.
    model = BodyModel(URDF,walls(.45),joint_angles=spread)
    geom = model.geometry([0,0,.35,0])
    low,high = bounds(geom)
    posture = dict(angles=spread,scope='single articulated posture, not a walking policy',
                   size=(high-low).tolist(),corridor_045_clearance_m=model.clearance([0,0,.35,0])[0],geometry=geom)
    report = dict(scope='offline 3D pose geometry and experimental flat-floor XY+yaw search',
        native_global_planner_modified=False,product_started=False,physical_simulation=False,
        joint_state_source='nominal hip=0, thigh=.8, calf=-1.6 rad; not measured',
        missing_geometry=['external D435i and its bracket'],
        clearance_policy='circle/twin retain configured hard envelope; URDF 0.02 m is an experimental margin, not calibrated',
        search=dict(step_m=.1,yaw_bins=16,edge_max_point_displacement_m=.02,
                    sampling_rule='distance > margin + half of bounded inter-sample displacement',
                    fixed_base_z_m=.35, support='flat fixture assumption; no support query'),
        profiles=profiles,scenes=scenes,rotation=rotation,garage=garage,posture=posture)
    write(out/'results.json',report)
    return report


def draw_shapes(ax, geometry, pose=(0,0,0,0), color='#54758b', alpha=.28, projection=(0,1)):
    from matplotlib.patches import Polygon
    c,s = math.cos(pose[3]),math.sin(pose[3])
    rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    for geom in geometry:
        pts = vertices(geom) @ rotation.T + np.asarray(pose[:3])
        points = pts[:,projection]
        hull = ConvexHull(points)
        payload = geom['link'].startswith(('mid360','livox'))
        ax.add_patch(Polygon(points[hull.vertices],facecolor='#bc8345' if payload else color,
                              edgecolor='#92612e' if payload else color,alpha=alpha,lw=.65))


def draw_obstacles(ax, obstacles):
    from matplotlib.patches import Rectangle
    for obs in obstacles:
        p,s = obs['center'],obs['half_size']
        ax.add_patch(Rectangle((p[0]-s[0],p[1]-s[1]),2*s[0],2*s[1],facecolor='#dededa',edgecolor='#8d918c',lw=.7))


def corridor_figures(out, report):
    """Keep corridor opening, body width and posture visibly distinct."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Arial','font.size':10,'axes.titlesize':10,'axes.labelsize':10,
                          'xtick.labelsize':10,'ytick.labelsize':10,'pdf.fonttype':42})

    def annotate(ax, obstacles):
        lower,upper=sorted(obstacles,key=lambda item:item['center'][1])
        low=lower['center'][1]+lower['half_size'][1]
        high=upper['center'][1]-upper['half_size'][1]
        ax.annotate('',xy=(-.46,high),xytext=(-.46,low),
                    arrowprops={'arrowstyle':'<->','color':'#384b55','lw':.9})
        ax.text(-.49,(low+high)/2,f'{high-low:.3f} m opening',rotation=90,va='center',ha='right')
        ax.set(xlim=(-.58,.50),ylim=(-.40,.40),aspect='equal',xlabel='X (m)',ylabel='Y (m)')
        return high-low

    scenes={scene['name']:scene for scene in report['scenes']}
    nominal=report['profiles']['urdf']['geometry']
    body_width=report['profiles']['urdf']['full_size'][1]
    fig,axes=plt.subplots(1,2,figsize=(7.4,3.8),layout='constrained')
    for ax,name in zip(axes,('corridor_045','corridor_028')):
        scene=scenes[name]
        draw_obstacles(ax,scene['obstacles'])
        draw_shapes(ax,nominal)
        gap=annotate(ax,scene['obstacles'])
        per_side=(gap-body_width)/2
        status=(f'Fits: {per_side*1000:.0f} mm gap per side' if per_side>0 else
                f'Overlap: {-per_side*1000:.0f} mm per side')
        ax.set_title(f'Opening {gap:.3f} m / body {body_width:.3f} m\n{status}')
    fig.savefig(out/'corridor_widths.png',dpi=200,bbox_inches='tight')
    fig.savefig(out/'corridor_widths.pdf',bbox_inches='tight')
    plt.close(fig)

    obstacles=scenes['corridor_045']['obstacles']
    fig,axes=plt.subplots(1,2,figsize=(7.4,3.8),layout='constrained')
    for ax,geometry,width,label in [
        (axes[0],nominal,body_width,'Nominal stance - fits'),
        (axes[1],report['posture']['geometry'],report['posture']['size'][1],'Hip spread - overlaps')]:
        draw_obstacles(ax,obstacles)
        draw_shapes(ax,geometry)
        gap=annotate(ax,obstacles)
        ax.set_title(f'Opening {gap:.3f} m / body {width:.3f} m\n{label}')
    fig.savefig(out/'posture_comparison.png',dpi=200,bbox_inches='tight')
    fig.savefig(out/'posture_comparison.pdf',bbox_inches='tight')
    plt.close(fig)


def figures(out, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    plt.rcParams.update({'font.family':'Arial','font.size':10,'axes.titlesize':10,'axes.labelsize':10,
                          'xtick.labelsize':10,'ytick.labelsize':10,'legend.fontsize':10,'pdf.fonttype':42})
    fig, axes = plt.subplots(1,3,figsize=(10.5,3.9),layout='constrained')
    for ax,(name,p) in zip(axes,report['profiles'].items()):
        draw_shapes(ax,p['geometry'])
        if name!='urdf':
            draw_shapes(ax,report['profiles']['urdf']['geometry'],color='#273c4c',alpha=.45)
        ax.set(xlim=(-.55,.55),ylim=(-.55,.55),aspect='equal',xlabel='Body X (m)',ylabel='Body Y (m)',
               title=f"{LABELS[name]}\nWidth {p['full_size'][1]:.3f} m")
        ax.grid(alpha=.15)
    fig.savefig(out/'body_shapes.png',dpi=200,bbox_inches='tight')
    fig.savefig(out/'body_shapes.pdf',bbox_inches='tight')
    plt.close(fig)
    fig,axes = plt.subplots(2,3,figsize=(10.5,5.7),layout='constrained')
    for row,scene in enumerate(report['scenes'][:2]):
        for ax,(name,p) in zip(axes[row],report['profiles'].items()):
            draw_obstacles(ax,scene['obstacles'])
            result = scene['results'][name]
            if result['poses']:
                poses = np.array(result['poses'])
                ax.plot(poses[:,0],poses[:,1],color='#367c63',lw=1.8)
                display = min(result['poses'],key=lambda x:abs(x[0]))
                for pose in result['poses'][::4]:
                    ax.arrow(pose[0],pose[1],.1*math.cos(pose[3]),.1*math.sin(pose[3]),width=.004,color='#367c63')
            else:
                display=[0,0,.35,0]
            draw_shapes(ax,p['geometry'],display,color='#54758b')
            ax.scatter([scene['start'][0],scene['goal'][0]],[0,0],s=22,c=['#4c6378','#ba8847'])
            ax.set(xlim=(-1.4,1.4),ylim=(-.8,.8),aspect='equal',xlabel='X (m)',ylabel='Y (m)',
                   title=f"{scene['label']} / {name}\n{'Pose path found' if result['ok'] else 'Rejected'}")
            ax.grid(alpha=.12)
    fig.savefig(out/'doorway_comparison.png',dpi=200,bbox_inches='tight')
    fig.savefig(out/'doorway_comparison.pdf',bbox_inches='tight')
    plt.close(fig)
    fig,ax = plt.subplots(figsize=(6.8,3.5),layout='constrained')
    for name,p in report['rotation']['profiles'].items():
        ax.plot([s['yaw_deg'] for s in p['samples']],[s['clearance_m'] for s in p['samples']],label=name,lw=1.5)
    ax.axhline(0,color='#a85846',lw=.8)
    ax.axhline(.02,color='#54758b',lw=.6,linestyle=':')
    ax.set(xlabel='Body yaw (degrees)',ylabel='Conservative clearance (m)',title='0.60 m corridor: the full turning arc matters')
    ax.legend(loc='upper center',bbox_to_anchor=(.5,1),ncol=3,frameon=False)
    ax.grid(alpha=.15)
    fig.savefig(out/'turning_clearance.png',dpi=200)
    fig.savefig(out/'turning_clearance.pdf')
    plt.close(fig)
    corridor_figures(out,report)
    fig,axes = plt.subplots(1,3,figsize=(10.5,3.6),layout='constrained')
    from matplotlib.patches import Rectangle
    original=report['garage']['pose']
    leaf=report['garage']['obstacles'][0]
    relative=np.asarray(leaf['center'])-np.asarray(original[:3])
    extent=np.asarray(leaf['half_size'])
    for ax,projection in zip(axes,[(0,1),(0,2),(1,2)]):
        draw_shapes(ax,report['profiles']['urdf']['geometry'],[0,0,0,original[3]],projection=projection)
        i,j=projection
        ax.add_patch(Rectangle((relative[i]-extent[i],relative[j]-extent[j]),2*extent[i],2*extent[j],
                               facecolor='#b44c68',edgecolor='#92334c',alpha=.45,lw=.8))
        ax.set(xlim=(-.45,.45),ylim=(-.4,.4),aspect='equal',xlabel=f'{"XYZ"[i]} relative to start (m)',
               ylabel=f'{"XYZ"[j]} relative to start (m)',title=f'Garage witness: {"XYZ"[i]}{"XYZ"[j]} view')
        ax.grid(alpha=.15)
    fig.savefig(out/'garage_witness.png',dpi=200,bbox_inches='tight')
    fig.savefig(out/'garage_witness.pdf',bbox_inches='tight')
    plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(9.5,3.5),layout='constrained')
    def frame(index):
        yaw=math.pi*index/36
        for ax,(name,p) in zip(axes,report['profiles'].items()):
            ax.clear()
            draw_obstacles(ax,report['rotation']['obstacles'])
            draw_shapes(ax,p['geometry'],[0,0,.35,yaw])
            value=report['rotation']['profiles'][name]['samples'][index]['clearance_m']
            ax.set(xlim=(-.55,.55),ylim=(-.55,.55),aspect='equal',xlabel='X (m)',ylabel='Y (m)',
                   title=f'{name}: yaw {index*5} deg\nclearance bound {value*1000:.0f} mm')
        return []
    animation=FuncAnimation(fig,frame,frames=37,interval=100)
    animation.save(out/'heading_sweep.gif',writer=PillowWriter(fps=10),dpi=110)
    plt.close(fig)


def page(out, report):
    rows=[]
    for scene in report['scenes']:
        cells=''.join(f"<td>{'找到姿态路径' if scene['results'][name]['ok'] else '拒绝'}</td>" for name in LABELS)
        rows.append(f"<tr><td>{html.escape(scene['label'])}</td>{cells}</tr>")
    artifact='''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>LingTu · 机体包络对照</title>
<style>body{margin:0;background:#f5f5f2;color:#232a2c;font:16px/1.7 Arial,"Microsoft YaHei",sans-serif}
main{max-width:1120px;margin:0 auto;padding:48px 30px}h1{font-size:30px;font-weight:550;margin:8px 0}h2{font-size:20px;margin:46px 0 12px}
p{max-width:920px}small,.muted{color:#697171}a{color:#355a6b}figure{background:white;margin:20px 0;padding:18px;border:1px solid #e1e3df}
figure img{width:100%;height:auto;display:block}figcaption{font-size:14px;color:#657171;margin:12px 8px 0}
table{border-collapse:collapse;width:100%;font-size:15px}td,th{padding:12px 10px;border-bottom:1px solid #dcdfdb;text-align:left}
nav{display:flex;gap:20px;flex-wrap:wrap;margin:22px 0}code{font-size:14px}footer{border-top:1px solid #dcdfdb;margin-top:40px;padding-top:20px;color:#657171}
@media(max-width:700px){main{padding:24px 14px}figure{padding:4px}table{font-size:12px}td,th{padding:9px 4px}}</style>
<main><small>LINGTU / GEOMETRY STUDY / 2026.10.22</small><h1>让碰撞形状跟着机器人朝向变化</h1>
<p class="muted">实际 URDF 碰撞模型的离线对照。实验搜索包含 XY 与朝向、固定高度；三维几何检查。正式 OctoPlanner3D / SCAN 尚未切换到此实现。</p>
<nav><a href="#shape">机体形状</a><a href="#route">窄门路径</a><a href="#turn">转身</a><a href="#garage">原起点</a><a href="results.json">原始数据</a><a href="README.md">复现记录</a></nav>
<h2 id="shape">同一尺度看三种包络</h2><figure><img src="body_shapes.png"><figcaption>左：覆盖所有朝向的 0.44 m 半径圆。中：随朝向转动的 0.25 m 双圆柱、前后偏移 0.19 m。右：34 个 URDF 碰撞几何，棕色为雷达与打印支架。均为标称静态姿态。</figcaption></figure>
<h2 id="route">在同一场景重新搜索</h2><table><thead><tr><th>场景</th><th>外接圆</th><th>朝向双圆柱</th><th>URDF + 朝向</th></tr></thead><tbody>ROWS</tbody></table>
<figure><img src="doorway_comparison.png"><figcaption>绿线来自实际实验搜索；箭头是身体朝向，允许四足机器人横移。URDF 额外 20 mm 间隙仅为实验设置，尚未校准成产品安全余量。该实验未验证步态、地面支撑或动态平衡。</figcaption></figure>
<h2 id="turn">首尾能放下，中途仍可能撞墙</h2><figure><img src="heading_sweep.gif"><figcaption>0.60 m 走廊的姿态扫掠动画，不是机器人行走视频。整段转动都需要检查，不能只检查起点与终点。</figcaption></figure>
<figure><img src="turning_clearance.png"><figcaption>正值表示几何间隔的保守下界；负值表示相交。URDF 实验还要求 20 mm 余量。所有角度保留在 results.json。</figcaption></figure>
<h2>先分清通道净宽和机器人宽度</h2><figure><img src="corridor_widths.png"><figcaption>同一标称姿态：左图通道净宽 0.45 m、机器人宽 0.328 m，每侧留 61 mm；右图才是 0.28 m 通道，每侧与模型交叠 24 mm。0.28 m 的拒绝针对这一固定姿态和全高侧墙，不是对所有可变姿态步态的结论。</figcaption></figure>
<h2>同一条 0.45 m 通道，不同姿态结果不同</h2><figure><img src="posture_comparison.png"><figcaption>这两张图的通道净宽均为 0.45 m。左图标称站姿宽 0.328 m，几何通行检查通过；右图髋关节向外各转 0.25 rad，宽约 0.470 m，与侧墙相交。右侧是关节姿态探针，不是实测步态。外装 D435i 与其支架尚未纳入当前 URDF。</figcaption></figure>
<h2 id="garage">Garage 的原占据叶仍需处理</h2><figure><img src="garage_witness.png"><figcaption>保留原始起点、朝向和地图占据叶。紫色为体素，URDF base box 与其仍相交；另用独立分离轴检查复核。这里只检查该原始叶，不能代表整图检查，也不能判断它是真实物体还是建图伪影。</figcaption></figure>
<footer>模型来源：仓库固定版本 Unitree 官方 URDF + 已安装雷达支架名义 CAD。图表 Arial 10 pt，旁附同名 PDF。未启动 Product、未操作机器人、未修改正式机体参数。</footer></main></html>'''
    (out/'index.html').write_text(artifact.replace('ROWS',''.join(rows)),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--figures-only',action='store_true')
    args=parser.parse_args()
    report=json.loads((args.out/'results.json').read_text()) if args.figures_only else run(args.out.resolve())
    figures(args.out,report)
    page(args.out,report)
    print(args.out/'results.json')


if __name__=='__main__':
    main()
