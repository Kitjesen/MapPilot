"""Visualize measured terrain probes; do not infer physical locomotion from route success."""
import csv
import json
import math
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
PAPER=ROOT/'docs/media/terrain-exploration-20261117'
SOURCE=ROOT/'build/startup-global-20261117/viewer'
plt.rcParams.update({'font.family':'Microsoft YaHei','font.size':9,'pdf.fonttype':42,'svg.fonttype':'none'})


def faces(box):
    a,b,c=box['min'];d,e,f=box['max']
    v=np.array([[a,b,c],[d,b,c],[d,e,c],[a,e,c],[a,b,f],[d,b,f],[d,e,f],[a,e,f]])
    return [v[i] for i in [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]


def draw(ax,scene,rows,title):
    x,y=np.asarray(scene['x']),np.asarray(scene['y'])
    h=np.asarray(scene['heights'],dtype=float);xx,yy=np.meshgrid(x,y,indexing='ij')
    ax.plot_surface(xx,yy,h,cmap='cividis',rstride=2,cstride=2,alpha=.67,linewidth=0,antialiased=True,zorder=1)
    for box in scene['extras']:
        ax.add_collection3d(Poly3DCollection(faces(box),facecolor='#a7b5c7',edgecolor='#6d7d8f',linewidth=.3,alpha=.55,zorder=2))
    low=float(np.nanmin(h));high=float(np.nanmax(h))+.55
    for i,row in enumerate(rows):
        p=np.array(row['result'].get('path',[]));request=next(r for r in scene['requests'] if r['id']==row['request'])
        if len(p):
            ax.plot(*p.T,color=['#d55e00','#0072b2','#a23c7a','#57a333'][i%4],linewidth=1.8,linestyle='-' if i%2==0 else '--',zorder=5)
            high=max(high,float(p[:,2].max()))
        if i==0:
            ax.scatter(*request['start'],color='#009e73',s=22,zorder=6,depthshade=False)
            ax.scatter(*request['goal'],color='#b52e35',s=42,marker='*',zorder=6,depthshade=False)
    for box in scene['extras']:high=max(high,box['max'][2])
    ax.set(xlim=(0,scene['length']),ylim=(-scene['width']/2,scene['width']/2),zlim=(low-.05,high+.05),xlabel='x / m',ylabel='y / m',zlabel='z / m',title=title)
    ax.set_box_aspect((scene['length'],scene['width'],max(.5,high-low)))
    ax.set_proj_type('ortho');ax.view_init(30,-63);ax.grid(False)
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_major_locator(MaxNLocator(3));axis.pane.set_alpha(0)


def save(fig,name):
    labels=[axis.label for ax in fig.axes for axis in ([ax.xaxis,ax.yaxis,ax.zaxis] if hasattr(ax,'zaxis') else [ax.xaxis,ax.yaxis])]+fig.legends+fig.texts
    for ext in ('png','pdf','svg'):
        (PAPER/ext).mkdir(parents=True,exist_ok=True)
        fig.savefig(PAPER/ext/f'{name}.{ext}',dpi=600 if ext=='png' else 150,bbox_inches='tight',bbox_extra_artists=labels,pad_inches=.12)
    (PAPER/"thumb").mkdir(exist_ok=True)
    with Image.open(PAPER/"png"/f"{name}.png") as im:
        im.thumbnail((1800,1200)); im.save(PAPER/"thumb"/f"{name}.png")
    plt.close(fig)


def metrics(scene,row):
    route=np.array(row['result'].get('path',[]));h=np.asarray(scene['heights'],dtype=float)
    ideal=np.asarray(scene['ideal_heights']);dx,dy=np.gradient(ideal,scene['resolution'],scene['resolution'])
    samples=[]
    for a,b in zip(route,route[1:]):
        count=max(1,math.ceil(float(np.linalg.norm(b-a))/scene['resolution']))
        samples.extend(a+(b-a)*i/count for i in range(count))
    if len(route):samples.append(route[-1])
    cross=[];span=[];along=[]
    for a,b in zip(samples,samples[1:]):
        heading=math.atan2(b[1]-a[1],b[0]-a[0])
        ix=min(len(scene['x'])-1,max(0,int(a[0]/scene['resolution'])))
        iy=min(len(scene['y'])-1,max(0,int((a[1]+scene['width']/2)/scene['resolution'])))
        cross.append(math.degrees(math.atan(abs(-math.sin(heading)*dx[ix,iy]+math.cos(heading)*dy[ix,iy]))))
        along.append(math.degrees(math.atan(abs(math.cos(heading)*dx[ix,iy]+math.sin(heading)*dy[ix,iy]))))
        config=row['result']['diagnostics']['constraints']
        radius=config['robot_radius'];offset=config['body_offset_m'];r=scene['resolution'];extent=radius+offset
        x0=max(0,int((a[0]-extent)/r));x1=min(h.shape[0],int((a[0]+extent)/r)+1)
        y0=max(0,int((a[1]-extent+scene['width']/2)/r));y1=min(h.shape[1],int((a[1]+extent+scene['width']/2)/r)+1)
        xx,yy=np.meshgrid(np.array(scene['x'][x0:x1]),np.array(scene['y'][y0:y1]),indexing='ij')
        inside=np.zeros_like(xx,dtype=bool)
        for sign in (-1,1):inside|=(xx-a[0]-sign*offset*math.cos(heading))**2+(yy-a[1]-sign*offset*math.sin(heading))**2<=radius**2
        heights=h[x0:x1,y0:y1][inside];heights=heights[np.isfinite(heights)]
        if len(heights):span.append(float(heights.max()-heights.min()))
    return dict(max_sampled_cross_slope_deg=max(cross,default=None),max_sampled_along_slope_deg=max(along,default=None),
                max_observed_height_span_under_body_m=max(span,default=None),diagnostic_sampling_m=scene['resolution'],
                route_z_span_m=float(route[:,2].max()-route[:,2].min()) if len(route) else None)


def main():
    data=json.loads((OUT/'scenes.json').read_text());rows=json.loads((OUT/'results.json').read_text());scenes={s['id']:s for s in data['scenes']}
    PAPER.mkdir(parents=True,exist_ok=True)
    for row in rows:row.update(metrics(scenes[row['scene']],row))
    (OUT/'analysis.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    for name,ids in [('terrain_overview',['wave_25_400','wave_35_120','ripple_2d','crest_30','valley_30','chicane_ramp']),
                     ('terrain_boundaries',['cross_45','cross_55','ridge_20','missing_30','wave_ceiling','wave_upper'])]:
        fig=plt.figure(figsize=(12,8))
        for i,key in enumerate(ids):
            selected=[r for r in rows if r['scene']==key]
            passes=sum(r['status']=='rust_pass' for r in selected)
            draw(fig.add_subplot(2,3,i+1,projection='3d',computed_zorder=False),scenes[key],selected,
                 f"({chr(97+i)}) {scenes[key]['title']}\n{passes}/{len(selected)} 组几何通过")
        fig.subplots_adjust(top=.94,bottom=.11,left=.02,right=.96,hspace=.15,wspace=.10)
        fig.text(.5,.015,'表面按高度采样展示；规划使用 5 cm OctoMap。曲线是实际返回的身体参考路线，尚未验证足端、姿态和物理运动。',ha='center',fontsize=8)
        save(fig,name)
    for scene in data['scenes']:
        selected=[r for r in rows if r['scene']==scene['id']]
        fig=plt.figure(figsize=(8,3.8))
        draw(fig.add_subplot(121,projection='3d',computed_zorder=False),scene,selected,scene['title'])
        ax=fig.add_subplot(122)
        h=np.asarray(scene['heights'],dtype=float);middle=len(scene['y'])//2
        ax.step(scene['x'],h[:,middle],where='mid',color='#8693a3',linewidth=1,label='中心纵断面地形')
        for i,row in enumerate(selected):
            path=np.array(row['result'].get('path',[]))
            if len(path):ax.plot(path[:,0],path[:,2],color=['#d55e00','#0072b2','#a23c7a','#57a333'][i%4],linewidth=1.1,linestyle='-' if i%2==0 else '--',label=row['request'])
        ax.set(xlabel='x / m',ylabel='z / m',title='XZ 投影（纵横轴独立缩放）')
        ax.legend(frameon=False,fontsize=6);ax.grid(alpha=.15);ax.spines[['top','right']].set_visible(False)
        fig.subplots_adjust(left=.02,right=.97,top=.86,bottom=.16,wspace=.23)
        save(fig,scene['id'])
    for source in ('scenes.json','results.json','analysis.json','options.json'):
        shutil.copy2(OUT/source,PAPER/source)
    with (PAPER/'metrics.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=['scene','request','status','elapsed_s','length_m','detour_ratio','max_sampled_cross_slope_deg','max_sampled_along_slope_deg','max_observed_height_span_under_body_m','route_z_span_m']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    html='<!doctype html><meta charset="utf-8"><title>连续地形测试</title><style>body{font:15px system-ui;max-width:1200px;margin:35px auto;color:#253344}img{max-width:100%}section{margin:35px 0}a{margin-right:16px}</style><h1>连续地形与规划边界</h1><p>仅验证全局规划与几何；横坡姿态与足端支撑未验证。</p>'
    headings={'terrain_overview':'波浪与坡面路线','terrain_boundaries':'地形边界与负例','model_boundaries':'当前姿态与足端覆盖不足'}
    for name in ['terrain_overview','terrain_boundaries','model_boundaries',*scenes]:
        html+=f'<section><h2>{scenes[name]["title"] if name in scenes else headings[name]}</h2><a href="pdf/{name}.pdf">PDF</a><a href="svg/{name}.svg">SVG</a><a href="png/{name}.png">PNG</a><p><img loading="lazy" src="thumb/{name}.png"></p></section>'
    (PAPER/'index.html').write_text(html,encoding='utf-8')
    print('Saved',len(scenes),'scenes',len(rows),'requests',PAPER,flush=True)


if __name__=='__main__':main()
