"""Illustrate model coverage using two recorded passing cases, without inventing foot contacts."""
import json
from pathlib import Path
import runpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle

OUT=Path(__file__).resolve().parent
m=runpy.run_path(str(OUT/'export.py'))
data=json.loads((OUT/'scenes.json').read_text());scenes={s['id']:s for s in data['scenes']}
rows=json.loads((OUT/'results.json').read_text());options=data['options']
fig,axes=plt.subplots(1,2,figsize=(9,4))
ax=axes[0];scene=scenes['cross_45'];row=next(r for r in rows if r['scene']=='cross_45' and r['request']=='along_forward')
assert row['status']=='rust_pass'
p=row['result']['path'][0];y=np.array(scene['y']);h=np.array(scene['heights'])[int(p[0]/.05)]
ax.step(y,h,where='mid',color='#58758b',linewidth=1.4,label='输入地图表面')
radius=options['robot_radius'];below=options['body_clearance_below_m'];above=options['body_clearance_above_m']
ax.add_patch(Rectangle((p[1]-radius,p[2]-below),2*radius,below+above,facecolor='#e9eff5',edgecolor='#627f93',linestyle='--'))
ax.scatter(p[1],p[2],s=30,color='#d55e00',label='身体参考点')
ax.text(-.75,.91,'横坡 45°；路线升降角 0°\n几何通过不等于侧倾姿态可执行',fontsize=9)
ax.set(xlim=(-.8,.8),ylim=(-.8,1.12),xlabel='y / m',ylabel='z / m',title='(a) 当前没有验证 roll / pitch 与足端姿态')
ax.set_aspect('equal');ax.grid(alpha=.13);ax.legend(loc='lower right',frameon=False,fontsize=7)
ax=axes[1];row=next(r for r in rows if r['scene']=='ridge_20' and r['request']=='along_forward')
assert row['status']=='rust_pass'
start=np.array(row['result']['path'][0]);goal=np.array(row['result']['path'][-1]);center=(start+goal)/2
ax.add_patch(Rectangle((center[0]-.8,-.65),1.6,1.3,facecolor='#f4e7dc'))
ax.add_patch(Rectangle((center[0]-.8,-.1),1.6,.2,facecolor='#c6d5dd'))
ax.plot([center[0]-.8,center[0]+.8],[center[1],center[1]],color='#d55e00',linewidth=1.7)
for sign in (-1,1):
    ax.add_patch(Circle((center[0]+sign*options['body_offset_m'],center[1]),radius,
                       facecolor='#e0efe9',edgecolor='#168d70',alpha=.55))
ax.scatter(center[0],center[1],color='#d55e00',s=20)
ax.annotate('',(center[0]+.68,-.1),(center[0]+.68,.1),arrowprops=dict(arrowstyle='|-|',color='#58758b'))
ax.text(center[0]+.71,0,'20 cm',rotation=90,ha='center',va='center',fontsize=8)
ax.text(center[0]-.72,.5,'两侧地面低 1 m\n绿色为双圆柱投影，不是脚的位置',fontsize=9)
ax.set(xlim=(center[0]-.8,center[0]+.8),ylim=(-.65,.65),xlabel='x / m',ylabel='y / m',title='(b) 参考点有支撑，未验证各足落点')
ax.set_aspect('equal');ax.grid(alpha=.13)
for ax in axes:ax.spines[['top','right']].set_visible(False)
fig.tight_layout(pad=1)
m['save'](fig,'model_boundaries')
